import gymnasium
from gymnasium.wrappers import RecordVideo
import highway_env
from matplotlib import pyplot as plt
from ctypes import *
import math
import os
import shutil
import argparse
import numpy as np
from typing import List
import distutils.dir_util
import json
from pathlib import Path

# COP: Patch VehicleGraphics.get_color so that crashed always takes priority over custom color.
# Without this, highway-env checks vehicle.color BEFORE vehicle.crashed, so any custom
# color (blue for blind, green for CEX) would suppress the native red crash rendering —
# including during the intermediate simulation sub-steps within env.step().
from highway_env.vehicle.graphics import VehicleGraphics
_original_get_color = VehicleGraphics.get_color
@classmethod
def _patched_get_color(cls, vehicle, transparent=False):
    if vehicle.crashed:
        color = cls.RED
        if transparent:
            color = (color[0], color[1], color[2], 30)
        return color
    return _original_get_color.__func__(cls, vehicle, transparent)
VehicleGraphics.get_color = _patched_get_color

# Prepare connection to morty lib.
import platform
if platform.system() == 'Windows':
    morty_lib = CDLL('./bin/VFM_MAIN_LIB.dll')
else:
    morty_lib = CDLL('./lib/libvfm.so')
morty_lib.expandScript.argtypes = [c_char_p, c_char_p, c_size_t]
morty_lib.expandScript.restype = c_char_p

# Create EnvModels.
dummy_result = create_string_buffer(2000)
script_envmodels = r"""
@{./src/templates/}@.stringToHeap[MY_PATH]
@{../../morty/envmodel_config.tpl.json}@.generateEnvmodels
"""
dummy_result = morty_lib.expandScript(script_envmodels.encode('utf-8'), dummy_result, sizeof(dummy_result)).decode()
# EO Create EnvModels.

# Load parameters from JSON.
with open('morty/envmodel_config.tpl.json') as f:
    d = json.load(f)
    ucd_config_prios_str = [s for s in d["#TEMPLATE"]["UCD_CONFIG_PRIOS"].split(';') if s] # only non-empty strings
    generated_path_prefix = d["#TEMPLATE"]["_GENERATED_PATH"]
    
with open('morty/envmodel_config.json') as f:
    d = json.load(f) # Take only the [0]th config here because on Python side there are no differences for now.
    nonegos = d[ucd_config_prios_str[0]]["NONEGOS"]
    num_actual_lanes = d[ucd_config_prios_str[0]]["NUMLANES"]
    num_technical_lanes = d[ucd_config_prios_str[0]]["LATERAL_LC_GRANULARITY"] + num_actual_lanes
    maxspeed = d[ucd_config_prios_str[0]]["MAXSPEEDNONEGO"]
    backward_driving_car_ids_str = d[ucd_config_prios_str[0]]["BACKWARD_DRIVING_CAR_IDS"]
    min_time_between_lcs = d[ucd_config_prios_str[0]]["MIN_TIME_BETWEEN_LANECHANGES"]
    max_speed = d[ucd_config_prios_str[0]]["MAXSPEEDNONEGO"]
    max_start_speed = min(d[ucd_config_prios_str[0]]["MAXSTARTSPEEDNONEGO_UCD"], max_speed)
    min_start_speed = min(d[ucd_config_prios_str[0]]["MINSTARTSPEEDNONEGO_UCD"], max_start_speed)
    exp_num = d[ucd_config_prios_str[0]]["UCD_EXP_NUM"]

    if backward_driving_car_ids_str.endswith(')@'):
        backward_driving_car_ids_str = backward_driving_car_ids_str[:-2]
    if backward_driving_car_ids_str.startswith('@('):
        backward_driving_car_ids_str = backward_driving_car_ids_str[2:]
    if backward_driving_car_ids_str:
        backward_driving_car_ids = [int(x) for x in backward_driving_car_ids_str.split(")@@(")]
    else:
        backward_driving_car_ids = []
# EO Load parameters from JSON.

# Note: Backward-driving cars are modeled on Highway-env side as forward-driving, backward-facing,
# (because of Highway-env's low-level control model which is not symmetrical when driving backwards),
# only on MC side as backward-driving, forward-facing. This has to be modeled on the interface.



SPECS = []      # Predefined specs as checked in the experiments. (Collision-freedom is implicit.)
SUCC_CONDS = [] # Conditions determining success of the specs.

SPECS.append("INVARSPEC !(@{ env.veh___6[i]9___.v = 0 }@.for[[i], 0, %r, 1, & ]);" % (nonegos - 1)) # 0: All cars stopped.

TARGET_VEL = 50 # Target velocity for the next spec.
SPECS.append("INVARSPEC !(@{ env.veh___6[i]9___.v = %r }@.for[[i], 0, %r, 1, & ]);" % (TARGET_VEL, nonegos - 1)) # 1: All cars at target velocity.

SPECS.append("INVARSPEC !(@{ env.veh___609___.on_lane_max = env.veh___6[i]9___.on_lane_max}@.for[[i], 1, %r, 1, & ]);" % (nonegos - 1)) # 2: All cars at max one lane width apart in lateral direction.

TARGET_DIST = 20 # Target distance for the next spec.
TARGET_VEL_LOW = 10 # Target velocity for the next spec, lower bound.
TARGET_VEL_HIGH = 20 # Target velocity for the next spec, upper bound.
SPECS.append(f"""INVARSPEC !(env.veh___609___.abs_pos >= env.veh___619___.abs_pos - {TARGET_DIST}
 & env.veh___619___.abs_pos >= env.veh___629___.abs_pos - {TARGET_DIST}
 & env.veh___629___.abs_pos >= env.veh___639___.abs_pos - {TARGET_DIST}
 & env.veh___639___.abs_pos >= env.veh___649___.abs_pos - {TARGET_DIST}
 & env.veh___609___.abs_pos <= env.veh___619___.abs_pos
 & env.veh___619___.abs_pos <= env.veh___629___.abs_pos
 & env.veh___629___.abs_pos <= env.veh___639___.abs_pos
 & env.veh___639___.abs_pos <= env.veh___649___.abs_pos
 & env.veh___609___.v <= {TARGET_VEL_LOW}
 & env.veh___619___.v <= {TARGET_VEL_LOW}
 & env.veh___629___.v <= {TARGET_VEL_LOW}
 & env.veh___639___.v <= {TARGET_VEL_LOW}
 & env.veh___649___.v >= {TARGET_VEL_HIGH}
);
""") # 3: All cars longitudinally close to each other, one drives faster.

SPECS.append(("INVARSPEC !(env.veh___609___.abs_pos - env.veh___6%r9___.abs_pos < 50 " % (nonegos - 1)) 
             + ("@{& env.veh___6@{[i] - 1}@.eval[0]9___.abs_pos > env.veh___6[i]9___.abs_pos}@*.for[[i], 1, %r]);" % (nonegos - 1))) 
            # 4: Reversed order of cars, i.e. the first car is the one with the highest abs_pos.

SPECS.append(r"""INVARSPEC TRUE;""") # 5: Benchmark 1.
SPECS.append(r"""INVARSPEC FALSE;""") # 6: Benchmark 2.

TARGET_DIST_NUDGING_FRONT = 300 # Target distance for the next spec.
TARGET_DIST_NUDGING_BACK = 10 # Target distance for the next spec.
TARGET_DIST_NUDGING = 300 # Target distance for the next spec.
#SPECS.append(f"INVARSPEC !(env.veh___609___.abs_pos >= env.veh___619___.abs_pos + {TARGET_DIST_NUDGING} & env.veh___619___.abs_pos >= 0);") # 7
SPECS.append(f"INVARSPEC !(env.veh___609___.abs_pos >= {TARGET_DIST_NUDGING_FRONT} & env.veh___619___.abs_pos <= {TARGET_DIST_NUDGING_BACK});") # 7
SPECS.append(r"""INVARSPEC !(env.veh___609___.lane_b0 & !env.veh___609___.lane_b1);""") # 8: Car 609 reaches leftmost lane (b0)


SUCC_CONDS.append(lambda: all(x < 1 for x in egos_v))
SUCC_CONDS.append(lambda: all(abs(x - TARGET_VEL) < 1 for x in egos_v))
SUCC_CONDS.append(lambda: maxDifferenceArray(egos_y[:-1]) < 4)
SUCC_CONDS.append(lambda: egos_x[0] >= egos_x[1] - TARGET_DIST and egos_x[1] >= egos_x[2] - TARGET_DIST and egos_x[2] >= egos_x[3] - TARGET_DIST and egos_x[3] >= egos_x[4] - TARGET_DIST
                  and egos_v[0] <= TARGET_VEL_LOW and egos_v[1] <= TARGET_VEL_LOW and egos_v[2] <= TARGET_VEL_LOW and egos_v[3] <= TARGET_VEL_LOW and egos_v[4] >= TARGET_VEL_HIGH)
SUCC_CONDS.append(lambda: inverseSortingArray(egos_x))
SUCC_CONDS.append(lambda: True)
SUCC_CONDS.append(lambda: True)
SUCC_CONDS.append(
    lambda: egos_x[0] >= TARGET_DIST_NUDGING_FRONT and egos_x[1] <= TARGET_DIST_NUDGING_BACK
    )
SUCC_CONDS.append(lambda: False) # 8

addons = [''] * len(SPECS) # Add to main.smv depending on the experiment.
ADDONS_CORE_DENOTER = "UCD addons"
ADDONS_BEGIN_DENOTER = "-- " + ADDONS_CORE_DENOTER + " --\n"
ADDONS_END_DENOTER = "-- EO " + ADDONS_CORE_DENOTER + " --\n"

for i in range(0, len(SPECS)):
    addons[i] += ADDONS_BEGIN_DENOTER
    addons[i] += f"-- UCD experiment{i} --\n"

addons[7] += r"""
INVAR env.veh___609___.v >= 5;
INVAR env.veh___619___.v <= -5;
TRANS (((env.veh___619___.abs_pos - env.veh___609___.abs_pos) < 0)
       != ((next(env.veh___619___.abs_pos) - next(env.veh___609___.abs_pos)) < 0))
       -> ((env.veh___609___.on_normalized_lane = next(env.veh___609___.on_normalized_lane)) & 
          (env.veh___619___.on_normalized_lane = next(env.veh___619___.on_normalized_lane)));
"""

for i in range(2, nonegos):
    addons[7] += f"INVAR env.veh___6{i}9___.v = 0;\n"

addons[8] += "INVAR env.veh___609___.v >= 5;\n"
for i in range(1, nonegos):
    addons[8] += f"INVAR env.veh___6{i}9___.v = 0;\n"

for i in range(0, len(SPECS)):
    addons[i] += ADDONS_END_DENOTER

DEFAULT_NUM_RUNS = 100
DEFAULT_NUM_STEPS_PER_RUN = 300
DEFAULT_HEADING_ADAPTATION = -0.5
DEFAULT_ALLOW_BLIND_STEPS = DEFAULT_NUM_STEPS_PER_RUN
DEFAULT_ALLOW_CRASHED_STEPS = 1

parser = argparse.ArgumentParser(
                    prog='morty',
                    description='Model Checking based planning',
                    epilog='Bye!')
parser.add_argument('-n', '--num_runs', default=DEFAULT_NUM_RUNS, type=int,
                    help=f'Number of runs to perform per experiment. Default: {DEFAULT_NUM_RUNS}')
parser.add_argument('-s', '--steps_per_run', default=DEFAULT_NUM_STEPS_PER_RUN, type=int,
                    help=f'Maximum number of steps until the SPEC must be fulfilled. Default: {DEFAULT_NUM_STEPS_PER_RUN}')
parser.add_argument('-a', '--heading_adaptation', default=DEFAULT_HEADING_ADAPTATION, type=float,
                    help=f'How much the heading of the cars is used to adapt their lateral position (see MC code for details). Default: {DEFAULT_HEADING_ADAPTATION}')
parser.add_argument('-b', '--allow_blind_steps', default=DEFAULT_ALLOW_BLIND_STEPS, type=int,
                    help=f'How many times the MC is allowed to be blind (no CEX) before the run is aborted. Default: {DEFAULT_ALLOW_BLIND_STEPS}')
parser.add_argument('-c', '--allow_crashed_steps', default=DEFAULT_ALLOW_CRASHED_STEPS, type=int,
                    help='How many steps with crashes are allowed before the run is aborted. Default: {DEFAULT_ALLOW_CRASHED_STEPS}')
parser.add_argument('-d', '--debug', default=0, type=int,
                    help='Enable writing images in each step to see what the MC thinks (0 or 1). Default: 0')
parser.add_argument('--record_video', action='store_true',
                    help='Record a video of the run. Default: False')
parser.add_argument('--detailed_archive', action='store_true',
                    help='Stores detailed archive of the run in a subfolder. Default: False')
parser.add_argument('--headless', action='store_true',
                    help='Run without opening the simulation UI window. Default: False')
args = parser.parse_args()

if args.headless:
    if not args.record_video:
        os.environ['SDL_VIDEODRIVER'] = 'dummy'  # No rendering at all.
    elif platform.system() == 'Windows':
        os.environ['SDL_VIDEODRIVER'] = 'windows'
        os.environ['SDL_VIDEO_WINDOW_POS'] = '-32000,-32000'
    else:
        os.environ['SDL_VIDEODRIVER'] = 'offscreen'  # Render to memory buffer, no X11.

# Best so far:
# ACCEL_RANGE = 6
ACCEL_RANGE = 6

MAX_EXPs = args.num_runs

good_ones = []


def ensure_empty_file(path: str) -> None:
    p = Path(path)

    if p.exists() and not p.is_file():
        return

    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open('w'):
        pass

def min_max_curr(successful_so_far, done_so_far, max_to_expect):
    percent = 100 * successful_so_far / done_so_far
    min_good = 100 * successful_so_far / max_to_expect
    max_good = 100 * (max_to_expect - done_so_far + successful_so_far) / max_to_expect
        
    return str(round(min_good, 1)) + "% <= " + str(round(percent, 1)) + "% <= " + str(round(max_good, 1)) + "%"

# Pure pursuit.
def dpoint_following_angle(dpoint_y, ego_y, heading, ddist, bwd):
    return heading - math.atan((dpoint_y - ego_y) * bwd / ddist)

def maxDifferenceArray(A):
    maxDiff = -1
    for i in range(len(A)):
        for j in range(len(A)):
            maxDiff = max(maxDiff, abs(A[j] - A[i]))
    return maxDiff

def inverseSortingArray(egos_x: List[float]):
    b = True
    for i in range(len(egos_x) - 1):
        b = b and (egos_x[i + 1] < egos_x[i])
    return b

ensure_empty_file(f'{generated_path_prefix}/results.txt')  # Delete old results from Python side
specification = create_string_buffer(2000)
spec_res = morty_lib.expandScript(SPECS[exp_num].encode('utf-8'), specification, sizeof(specification)).decode()

for ucd_config_str in ucd_config_prios_str:
    ensure_empty_file(f'{generated_path_prefix + ucd_config_str}/morty_mc_results.txt')  # Delete old results from MC side (these are a super set of the above)

    with open(generated_path_prefix + ucd_config_str + "/main.smv", "r+") as f:
        content = f.read()

        begin_idx = content.find(ADDONS_BEGIN_DENOTER) # Cut out addons, if there, and replace with new ones.
        end_idx = content.find(ADDONS_END_DENOTER)
        if begin_idx != -1 and end_idx != -1:
            content = content[:begin_idx] + content[end_idx + len(ADDONS_END_DENOTER):]

        content = content.replace("--EO-ADDONS", addons[exp_num] + "\n--EO-ADDONS")

        invarspec_idx = content.find("INVARSPEC") # Cut out SPEC and replace with new one.
        if invarspec_idx == -1:
            print(f"Error: no INVARSPEC found in {generated_path_prefix + ucd_config_str + '/main.smv'}. Failing fast...")
            exit(1)
        else:
            semicolon_idx = content.find(";", invarspec_idx)
            if semicolon_idx != -1:
                content = content[:invarspec_idx] + spec_res + content[semicolon_idx + 1:]

        f.seek(0)
        f.write(content)
        f.truncate()

import hashlib

def _hash_file(path):
    h = hashlib.md5()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(8192), b''):
            h.update(chunk)
    return h.hexdigest()

def _snapshot_configs(restrict_to=None):
    """Return {config_name: {rel_path: md5_hex}} for all config dirs.
    If restrict_to is given, only include rel_paths present in that dict."""
    snap = {}
    for config_name in ucd_config_prios_str:
        config_dir = generated_path_prefix + config_name
        allowed = restrict_to.get(config_name) if restrict_to else None
        file_hashes = {}
        for dirpath, _, filenames in os.walk(config_dir):
            for fname in filenames:
                full_path = os.path.join(dirpath, fname)
                rel_path = os.path.relpath(full_path, config_dir)
                if allowed is not None and rel_path not in allowed:
                    continue
                file_hashes[rel_path] = _hash_file(full_path)
        snap[config_name] = file_hashes
    return snap

def _save_configs_to_archive(seedo, subfolder, restrict_to=None):
    """Copy config files into the detailed archive under the given subfolder.
    If restrict_to is given ({config_name: set_of_rel_paths}), only copy those files."""
    archive_path = f'{generated_path_prefix}/detailed_archive/run_{seedo}/{subfolder}/'
    for config_name in ucd_config_prios_str:
        config_dir = generated_path_prefix + config_name
        allowed = restrict_to.get(config_name) if restrict_to else None
        for dirpath, _, filenames in os.walk(config_dir):
            for fname in filenames:
                full_path = os.path.join(dirpath, fname)
                rel_path = os.path.relpath(full_path, config_dir)
                if allowed is not None and rel_path not in allowed:
                    continue
                dest = os.path.join(archive_path + config_name, rel_path)
                os.makedirs(os.path.dirname(dest), exist_ok=True)
                shutil.copy2(full_path, dest)

baseline_hashes = {}  # Pre-loop file set (before any MC call).
snapshot_hashes = {}  # Post-first-MC-call hashes (only baseline files).

def archive(seedo, global_counter):
    if args.detailed_archive:
        archive_path = f'{generated_path_prefix}/detailed_archive/run_{seedo}/iteration_{global_counter}/'
        if not os.path.exists(archive_path):
            os.makedirs(archive_path)
        for config_name in ucd_config_prios_str:
            config_dir = generated_path_prefix + config_name
            baseline = snapshot_hashes.get(config_name, {})
            for dirpath, _, filenames in os.walk(config_dir):
                for fname in filenames:
                    full_path = os.path.join(dirpath, fname)
                    rel_path = os.path.relpath(full_path, config_dir)
                    current_hash = _hash_file(full_path)
                    if rel_path not in baseline or baseline[rel_path] != current_hash:
                        dest = os.path.join(archive_path + config_name, rel_path)
                        os.makedirs(os.path.dirname(dest), exist_ok=True)
                        shutil.copy2(full_path, dest)

# Take pre-loop baseline: record which files exist before any MC call.
if args.detailed_archive:
    baseline_hashes = _snapshot_configs()

for seedo in range(0, MAX_EXPs): # TODO: set ==> 0 again.
    env = gymnasium.make('highway-v0', render_mode='rgb_array', config={
        "action": {
            "type": "MultiAgentAction",
            "action_config": {
                "type": "ContinuousAction",
                "acceleration_range": [-ACCEL_RANGE, ACCEL_RANGE],
                "steering_range": [-1, 1],
                "speed_range": [0, maxspeed], # Best so far: [0, 70]. Don't forget to adjust MAXSPEEDNONEGO in the json.
                "longitudinal": True,
                "lateral": True,
            },
        },
        "observation": {
        "type": "MultiAgentObservation",
        "observation_config": {
            "features": ["presence", "x", "y", "vx", "vy", "heading"],
            "type": "Kinematics",
            #        "absolute": True,
            "normalize": False,
        }
        },
        "simulation_frequency": 60,  # [Hz]
        "policy_frequency": 2,  # [Hz]
        "controlled_vehicles": nonegos,
        "lanes_count": num_actual_lanes,
        "vehicles_count": 0,
        "screen_width": 1500,
        "screen_height": 200,
        "scaling": 3,
        "show_trajectories": True,
    })

    action = ([0, 0],) * nonegos
    dpoints_y =     [0] * (nonegos + 1) # The lateral position of the points the cars head towards.
    dpoints_delta = [0] * (nonegos + 1) # The direction we're currently moving towards, laterally.
    lc_time =       [0] * (nonegos + 1) # Time the current lc is taking, abort if too long. Needed to better approximate MC behavior.
    egos_x =        [0] * (nonegos + 1) # Long pos of cars in m.
    egos_y =        [0] * (nonegos + 1) # Lat pos of cars in m.
    egos_v =        [0] * (nonegos + 1) # Absuolute (!) long vel of cars in m/s. Multiply with egos_backward to get the signed velocity as seen by the MC.
    egos_headings = [0] * (nonegos + 1) # Angle rel. to long axis in rad.
    egos_backward = [1] * (nonegos + 1) # If the respective car is a forward (1) or backward (-1) driving car.
    nocex_count = 0
    crashed_count = 0

    # Mark backward driving cars.
    for i in backward_driving_car_ids:
        egos_backward[i] = -1
        
    env.reset(seed=seedo)

    np.random.seed(seedo)
    cnt = 0
    for vehicle in env.unwrapped.controlled_vehicles:
        vehicle.color = (255, 255, 255)
        vehicle.heading = np.pi * (1 - egos_backward[cnt]) / 2 # 0 for forward, pi for backward.
        vehicle.speed = np.random.uniform(min_start_speed, max_start_speed)
        if exp_num == 7:
            if cnt == 0:
                vehicle.position[0] = 0
            if cnt == 1:
                vehicle.position[0] = (nonegos - 1) * 400 / nonegos
            if cnt > 1:
                vehicle.position[0] = (cnt - 1) * 400 / nonegos
                vehicle.speed = 0
        if exp_num == 8:
            if cnt == 0:
                vehicle.position[0] = 0
                vehicle.position[1] = 0 # start at rightmost lane center (y=4m)
        
        cnt = cnt + 1

    if args.record_video:
        env = RecordVideo(env, video_folder="videos", name_prefix=f"vid_{seedo}",
                    episode_trigger=lambda e: True)  # record all episodes
        env.unwrapped.set_record_video_wrapper(env)
        env.start_recording(f"vid_{seedo}")

    first = True
    obs = env.unwrapped.observation_type.observe()
    blind_stats = ""
    crashed = False

    if args.detailed_archive:
        _save_configs_to_archive(seedo, '0_pristine')

    for global_counter in range(args.steps_per_run):
        mcinput = ""
        i = 0
        for el in obs: # Use only el[0] because it contains the abs values from the resp. car's perspective.
            if first:
                dpoints_y[i] = el[0][2] # Set desired lateral position to the actual position in the first step.
                
            egos_x[i] = el[0][1]
            egos_y[i] = el[0][2]
            egos_v[i] = el[0][3] * egos_backward[i]
            egos_headings[i] = el[0][5] - np.pi * (1 - egos_backward[i]) / 2
            for num, val in enumerate(el[0]): # Generate input for model checker.
                if egos_backward[i] == -1 and num == 5:
                    mcinput += str(-val) + ","
                else:
                    mcinput += str(val) + ","
            mcinput += ";"
            i = i + 1

        MC_SCRIPT = f"""@{{
            @{{./src/templates/}}@.stringToHeap[MY_PATH]

            @{{{mcinput}}}@.prepareInputForMortyUCD[{args.heading_adaptation}, {num_actual_lanes}, {num_technical_lanes}]

            @{{nuXmv}}@.killAfter[300].Detach.setScriptVar[scriptID, force]
            @{{../../morty/envmodel_config.tpl.json}}@.runMCJobs[16]
            @{{scriptID}}@.scriptVar.StopScript

            {"@{../../morty/envmodel_config.tpl.json}@.generateTestCases[cex-birdseye/cex-smooth-birdseye]" if args.debug else ""}
            }}@.nil
            @{{}}@.prepareOutputForMortyUCD[{str(seedo)}, {str(global_counter)}, {0}, {str(crashed)}]
        """
        # Test cases´: all or [cex-birdseye/cex-cockpit-only/cex-full/cex-smooth-birdseye/cex-smooth-full/cex-smooth-with-arrows-birdseye/cex-smooth-with-arrows-full/preview/preview2]
        # EO The script to run the MC on C++ side.
        
        first = False
        
        empty_cex = ""
        
        for _i in range(nonegos):
            empty_cex += "|;"
        
        #### MODEL CHECKER CALL ####
        result = create_string_buffer(25000)
        res = morty_lib.expandScript(MC_SCRIPT.encode('utf-8'), result, sizeof(result))
        res_str = res.decode().strip()

        if args.detailed_archive and global_counter == 0:
            # Processed snapshot: only re-hash files that existed in the baseline
            # (excludes iteration-specific artifacts like debug folders).
            snapshot_hashes = _snapshot_configs(restrict_to=baseline_hashes)
            _save_configs_to_archive(seedo, '1_initialized', restrict_to={cn: set(baseline_hashes[cn]) for cn in baseline_hashes})
        
        ### POSTPROCESS RESULT ###
        all_results_dict = {}
        for single_res in res_str.split('\n'):
            if single_res:
                print("\n'" + single_res + "'\n")
                config_name, res_str = single_res.split(':') # initiate res_str with ANY of the results, will get updated later if none-blind exists.
                all_results_dict[config_name] = res_str
        
        blind = "|X"
        for cnt, config_name in enumerate(ucd_config_prios_str):
            if (all_results_dict[config_name] == empty_cex):
                print(f"CEX #{cnt} is EMPTY.")
            else:
                res_str = all_results_dict[config_name]
                print(f"Took CEX #{cnt}[{config_name}] which is the first non-empty one.")
                blind = "|" + str(cnt)
                break;
        
        blind_stats += blind
        
        ### EO POSTPROCESS RESULT ###
        #### EO MODEL CHECKER CALL ####

        successed = SUCC_CONDS[exp_num]()
        
        # We check both (1) the MC result and (2) the success condition to determine if we are done. Reason:
        # (1) alone is not sufficient because the MC cares only about the SPEC being satisfied in the last step.
        # Here we check AFTER the last step, a situation which is not guaranteed to have a solution, as well, so we might
        # get a false negative. Checking only (2) would be possible; we add the additional
        # MC check to make it easier to implement new SPECs. Then, a first impression is possible even
        # if no success condition is given or if it is not implemented in a precise way. 
        if res_str == "FINISHED" or successed:
            print("DONE")
            good_ones.append(seedo)
            archive(seedo, global_counter)
            break
       
        if res_str == empty_cex:
            print("No CEX found")
            for i in range(nonegos): # Set blue when blind.
                if not env.unwrapped.controlled_vehicles[i].crashed:
                    env.unwrapped.controlled_vehicles[i].color = (100, 100, 255)
            if nocex_count > args.allow_blind_steps: # Allow up to this many times being blind per run. (If in doubt: 10)
                archive(seedo, global_counter)
                break
            nocex_count += 1
        else:
            cex_length = len(res_str.split(';')[0].split('|')[0].split(','))
            for i in range(nonegos):
                if not env.unwrapped.controlled_vehicles[i].crashed:
                    env.unwrapped.controlled_vehicles[i].color = (min(cex_length * 15, 255), 200, min(cex_length * 15, 255))

        sum_vel_by_car = []
        sum_lan_by_car = []

        lanes = ""
        accels = ""

        # Best so far:
        # LANE_CHANGE_DURATION = 3
        LANE_CHANGE_DURATION = min_time_between_lcs # Has to match EnvModel.smv ==> min_time_between_lcs

        # Process the MC data.
        for i1, el1 in enumerate(res_str.split(';')):
            sum_lan_by_car.append(0)
            sum_vel_by_car.append(0)
            lanes += "\n"
            accels += "\n"
            for i2, el2 in enumerate(el1.split('|')):
                for i3, el3 in enumerate(el2.split(',')):
                    if el3:
                        if i2 == 1:
                            lanes += "   " + el3
                            if i3 == LANE_CHANGE_DURATION:
                                sum_lan_by_car[i1] += float(el3)
                        else:
                            accels += "   " + el3
                            if i3 == 0:
                                sum_vel_by_car[i1] += float(el3)

        with open(f"{generated_path_prefix}/lanes.txt", "w") as text_file:
            text_file.write(lanes)
        with open(f"{generated_path_prefix}/accels.txt", "w") as text_file:
            text_file.write(accels)

        print(f"summed velocity: {sum_vel_by_car}")
        print(f"summed lane: {sum_lan_by_car}")
        # EO Process the MC data.

        action_list = []
        
        # Best so far:
        # MAXTIME_FOR_LC = 60
        MAXTIME_FOR_LC = 60
        
        eps = 1
        #### COP VARIABLES ### 
        LANE_WIDTH_HE = 4.0  # highway-env lane width in meters
        on_lane_step_y = 2.0 * num_actual_lanes / num_technical_lanes  # y-distance per MC on_lane position
        # Compute valid y range based on technical lane positions.
        y_min_tech = -LANE_WIDTH_HE / 2.0 + LANE_WIDTH_HE * num_actual_lanes / (2.0 * num_technical_lanes)
        y_max_tech = -LANE_WIDTH_HE / 2.0 + (2 * num_technical_lanes - 1) * LANE_WIDTH_HE * num_actual_lanes / (2.0 * num_technical_lanes)
        #### EO COP VARIABLES ### 
        for i, el in enumerate(sum_vel_by_car):
            lc_time[i] += 1
            
            if abs(dpoints_y[i] - egos_y[i]) < eps: # If we're close to the desired lateral position, we can start a new lane change.
                lc_time[i] = 0
                num_tech_lanes = abs(sum_lan_by_car[i])
                
                if sum_lan_by_car[i] < 0:
                    dpoints_delta[i] = num_tech_lanes * on_lane_step_y
                    dpoints_y[i] += dpoints_delta[i]
                elif sum_lan_by_car[i] > 0:
                    dpoints_delta[i] = -num_tech_lanes * on_lane_step_y
                    dpoints_y[i] += dpoints_delta[i]
            
            if lc_time[i] > MAXTIME_FOR_LC and dpoints_delta[i] != 0:
                dpoints_y[i] -= dpoints_delta[i] # Go back to source lane if taking too long.
                dpoints_delta[i] = 0 # Don't care about cases with no ongoing LC since delta is zero, then.
                lc_time[i] = 0
            
            dpoints_y[i] = max(min(dpoints_y[i], y_max_tech), y_min_tech)
            
            # Best so far:
            # accel = sum_vel_by_car[i] * 6/3 / ACCEL_RANGE
            accel = egos_backward[i] * sum_vel_by_car[i] * 6/3 / ACCEL_RANGE

            # Best so far:
            # angle = -dpoint_following_angle(dpoints_y[i], egos_y[i], egos_headings[i], 10 + 2 * egos_v[i]) / 3.1415
            angle = -dpoint_following_angle(dpoints_y[i], egos_y[i], egos_headings[i], 10 + 2 * egos_v[i], egos_backward[i]) / 3.1415 # Magic constants, get over it ;)
            
            if egos_backward[i] == -1: # If the car is backward-driving, we have to adapt the angle to the different perspective.
                print(angle)
                # input("Press Enter to continue...")
            
            action_list.append([accel, angle])
        
        #print(action_list_vel)
        #print(action_list_lane)
        
        action = tuple(action_list)

        # === APPLY MC INSTRUCTIONS ===
        obs, reward, done, truncated, info = env.step(action)

        # === CHECK CRASHES (after applying MC instructions) ===
        crashed = False
        for vehicle in env.unwrapped.road.vehicles:
            crashed = crashed or vehicle.crashed
        if crashed:
            crashed_count += 1

        # === RENDER ===
        if not args.headless:
            env.render()

        archive(seedo, global_counter)

        if crashed_count > args.allow_crashed_steps: # Allow these many crashes per run.
            break
        
    with open(f"{generated_path_prefix}/results.txt", "a") as f:
        outcome = "succeeded" if seedo in good_ones else "failed"
        f.write("{" + min_max_curr(len(good_ones), seedo + 1, MAX_EXPs) + "} " + str(seedo) + " " + outcome + " [" + str(nocex_count) + " blind; " + blind_stats + "|]\n")

    if args.record_video:
        env.stop_recording()
        suffix = ""
        if crashed_count > 0:
            suffix += "_crashed"
        if nocex_count > 0:
            suffix += f"_blind{nocex_count}"
        if suffix:
            import glob
            for video_file in glob.glob(f"videos/vid_{seedo}*"):
                new_name = video_file.replace(f"vid_{seedo}", f"vid_{seedo}{suffix}", 1)
                os.rename(video_file, new_name)
    
print(good_ones)
