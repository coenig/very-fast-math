from ctypes import create_string_buffer, sizeof

import matplotlib
matplotlib.use('Agg')  # Non-interactive backend — no X11/display required

import gymnasium
from gymnasium.wrappers import RecordVideo
from matplotlib import pyplot as plt
import os
import shutil
import argparse
import numpy as np
import json
from pathlib import Path
import glob
from .morty_debug_plots import (
    plot_cex_lengths, plot_cex_lengths_cumulative,
    plot_mc_runtimes, plot_mc_runtimes_cumulative,
)
from .morty_helper import (
    ensure_empty_file, min_max_curr, dpoint_following_angle,
    maxDifferenceArray, inverseSortingArray, _latest_nuxmv_runtime_seconds,
    _hash_file, _snapshot_configs, _save_configs_to_archive, archive, morty_script_context
)
import platform


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
parser.add_argument('--record_video', action='store_true',
                    help='Record a video of the run. Default: False')
parser.add_argument('--hide_trajectories', action='store_true',
                    help='Hide trajectory lines for each vehicle in visualization. Default: False')
parser.add_argument('--show_prio_numbers', action='store_true',
                    help='Show priority numbers along trajectories. Default: False')
parser.add_argument('--hide_pure_pursuit', action='store_true',
                    help='Hide live pure-pursuit target dot and line for each vehicle. Default: False')
parser.add_argument('--hide_planned_positions', action='store_true',
                    help='Hide live planned positions for each vehicle. Default: False')
parser.add_argument('--detailed_archive', action='store_true',
                    help='Stores detailed archive of the run in a subfolder. Default: False')
parser.add_argument('--force', action='store_true',
                    help='Force deleting of existing result files. Default: False')
parser.add_argument('--dryrun', action='store_true',
                    help='Perform a dry run recreating all plots without running the model checker. Default: False')
parser.add_argument('--headless', action='store_true',
                    help='Run without opening the simulation UI window. Default: False')
args = parser.parse_args()

if args.dryrun:
    if args.force or args.detailed_archive:
        print("Error: --dryrun cannot go together with --force or --detailed_archive. Exiting.")
        exit(1)

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

# COP: Global trajectory tracking for visualization.
_vehicle_trajectories = {}  # Maps vehicle id to list of [x, y, priority] tuples
_vehicle_pp_targets = {}    # Maps vehicle id to current pure-pursuit target [x, y]
_current_selected_cnt = None  # Track which CEX priority is currently active

global_pos_to_draw = []

# COP: Patch VehicleGraphics.display to draw trajectories and car IDs.
# Important: Draw trajectories from inside VehicleGraphics.display so lines are rendered
# before the frame blit/flip done by EnvViewer.display.
_original_display = VehicleGraphics.display.__func__
@classmethod
def _patched_display(cls, vehicle, surface, transparent=False, offscreen=False, label=False, draw_roof=False):
    try:
        import pygame

        # Persist trajectory in world coordinates and draw it in the current camera view.
        vehicle_id = id(vehicle)
        if vehicle_id not in _vehicle_trajectories:
            _vehicle_trajectories[vehicle_id] = []

        trajectory = _vehicle_trajectories[vehicle_id]
        current_position = [float(vehicle.position[0]), float(vehicle.position[1])]
        stationary = abs(getattr(vehicle, 'speed', 0.0)) < 0.1

        if not trajectory:
            trajectory.append([*current_position, _current_selected_cnt])
        elif not stationary:
            last_position = trajectory[-1]
            dx = current_position[0] - last_position[0]
            dy = current_position[1] - last_position[1]
            moved = abs(dx) > 0.05 or abs(dy) > 0.05
            # Ignore one-time teleport/snap after initialization to avoid fake long tails.
            if moved:
                jump2 = dx * dx + dy * dy
                if len(trajectory) == 1 and jump2 > 100.0:  # >10 m in one render step is likely a teleport.
                    trajectory[0] = [*current_position, _current_selected_cnt]
                else:
                    trajectory.append([*current_position, _current_selected_cnt])
                    if len(trajectory) > 2000:
                        _vehicle_trajectories[vehicle_id] = trajectory[-2000:]

        if not args.hide_trajectories and len(trajectory) > 1:
            # Draw trajectory segments colored by priority
            priority_color_map = {
                None: (100, 150, 200),
                0: (31, 119, 180),    # tab:blue
                1: (255, 127, 14),    # tab:orange
                2: (44, 160, 44),     # tab:green
                3: (214, 39, 40),     # tab:red
                4: (148, 103, 189),   # tab:purple
                5: (140, 86, 75),     # tab:brown
            }
            font = pygame.font.Font(None, 14)
            for i in range(len(trajectory) - 1):
                p1_pix = surface.pos2pix(trajectory[i][0], trajectory[i][1])
                p2_pix = surface.pos2pix(trajectory[i + 1][0], trajectory[i + 1][1])
                priority = trajectory[i][2] if len(trajectory[i]) > 2 else None
                color = priority_color_map.get(priority, (100, 150, 200))
                pygame.draw.line(surface, color, p1_pix, p2_pix, width=2)

                # Label each priority block once, near its first segment.
                prev_priority = trajectory[i - 1][2] if i > 0 and len(trajectory[i - 1]) > 2 else None
                if i == 0 or priority != prev_priority:
                    label = "X" if priority is None else str(priority)
                    midx = (p1_pix[0] + p2_pix[0]) // 2
                    midy = (p1_pix[1] + p2_pix[1]) // 2
                    text = font.render(label, True, (20, 20, 20), (255, 255, 255))
                    if args.show_prio_numbers:
                        surface.blit(text, (midx + 2, midy - 10))
                        
        if not args.hide_pure_pursuit and vehicle_id in _vehicle_pp_targets:
            target = _vehicle_pp_targets[vehicle_id]
            # Offset line start to front axle instead of vehicle center
            front_offset = vehicle.LENGTH / 2.0
            p_vehicle_x = current_position[0] + front_offset * np.cos(vehicle.heading)
            p_vehicle_y = current_position[1] + front_offset * np.sin(vehicle.heading)
            p_vehicle = surface.pos2pix(p_vehicle_x, p_vehicle_y)
            p_target = surface.pos2pix(target[0], target[1])
            pygame.draw.line(surface, (255, 140, 0), p_vehicle, p_target, width=2)
            pygame.draw.circle(surface, (255, 180, 0), p_target, 5)

    except Exception as e:
        print(f"Warning: Error drawing trajectories: {e}")
        input("Press Enter to continue..." + str(e))

    _original_display(cls, vehicle, surface, transparent=transparent, offscreen=offscreen, label=False, draw_roof=draw_roof)

    try:
        import pygame
        
        for pos in global_pos_to_draw:
            pixx = surface.pos2pix(pos[0][0], pos[0][1])
            pygame.draw.circle(surface, pos[1], pixx, 3)

    except Exception as e:
        raise RuntimeError(f"Error drawing global positions: {e}")

    if not surface.is_visible(vehicle.position):
        return
    try:
        import pygame
        idx = vehicle.road.vehicles.index(vehicle)
        font = pygame.font.Font(None, 18)
        text = font.render(str(idx), True, (0, 0, 0), (255, 255, 255))
        position = [*surface.pos2pix(vehicle.position[0], vehicle.position[1])]
        surface.blit(text, (position[0] - 5, position[1] - 15))
    except (ValueError, AttributeError):
        pass
VehicleGraphics.display = _patched_display

import numpy as np

def get_scene_bounding_box(env, car_ids):
    """
    Finds the tightest axis-aligned (unrotated) bounding box that contains 
    every point of the specified cars in the highway-env scene.
    
    Args:
        env: The highway-env / gymnasium environment instance.
        car_ids (list): List of vehicle IDs to include.
        
    Returns:
        dict: A dictionary containing the bounding box boundaries:
              xmin, ymin, xmax, ymax and the box dimensions.
    """
    all_corners = []
    
    # Access all active vehicles in the current road scene
    vehicles = env.unwrapped.road.vehicles
    
    # Filter for the requested cars based on ID
    # Note: Depending on your wrapper/setup, you can match by ID, or index.
    # Here, we assume vehicles can be filtered or matched. If you have unique custom IDs:
    target_vehicles = [v for v in vehicles if getattr(v, 'id', None) in car_ids]
    
    # Fallback if your vehicles do not have an 'id' attribute (match by list index):
    if not target_vehicles:
        for idx, v in enumerate(vehicles):
            if idx in car_ids:
                target_vehicles.append(v)
                
    if not target_vehicles:
        raise ValueError(
            f"No IDs found in scene. Specified: {car_ids}, Available: {vehicles}")

    for vehicle in target_vehicles:
        # Correctly unpack position coordinates
        x, y = vehicle.position[0], vehicle.position[1]
        heading = vehicle.heading
        length = vehicle.LENGTH
        width = vehicle.WIDTH
        
        # Define corners relative to the vehicle's center (local coordinates)
        local_corners = np.array([
            [length / 2,  width / 2],
            [length / 2, -width / 2],
            [-length / 2, -width / 2],
            [-length / 2,  width / 2]
        ])
        
        # 2. Construct 2D rotation matrix for the heading angle
        cos_h = np.cos(heading)
        sin_h = np.sin(heading)
        rotation_matrix = np.array([
            [cos_h, -sin_h],
            [sin_h,  cos_h]
        ])
        
        # 3. Rotate and translate corners to global coordinates
        global_corners = (local_corners @ rotation_matrix.T) + np.array([x, y])
        all_corners.extend(global_corners)
        
    # 4. Find the global minimum and maximum across all gathered corners
    all_corners = np.array(all_corners)
    x_min, y_min = np.min(all_corners, axis=0)
    x_max, y_max = np.max(all_corners, axis=0)
    
    return {
        "xmin": x_min,
        "ymin": y_min,
        "xmax": x_max,
        "ymax": y_max,
        "width": x_max - x_min,
        "height": y_max - y_min
    }


# Load parameters from JSON.
with open('morty/envmodel_config.tpl.json') as f:
    d = json.load(f)
    ucd_config_prios_str = [s for s in d["#TEMPLATE"]["UCD_CONFIG_PRIOS"].split(';') if s] # only non-empty strings
    generated_path_prefix = d["#TEMPLATE"]["_GENERATED_PATH"]

# Delete old results from Python side and check folder consistence with --dryrun
any = False
for res_path in glob.glob(generated_path_prefix + "*"):
    if Path(res_path).is_dir():
        any = True
        if args.force:
            print(f"Directory {res_path} already exists. Deleting because --force is set.")
            shutil.rmtree(res_path)
            if Path(res_path).is_dir():
                print(f"Error: failed to delete {res_path}. Exiting.")
                exit(1)
            else: 
                print(f"Successfully deleted {res_path}. Continuing.")
        elif args.dryrun:
            pass
        else:
            print(f"Error: Results folder {res_path} already exists.")

if any and not args.force and not args.dryrun:
    print("Use --force to auto-remove existing results folders. Exiting.")
    exit(1)

if args.dryrun and (not Path(generated_path_prefix).is_dir() or not Path(generated_path_prefix + "/" + "detailed_archive").is_dir()):
    print("Error: No existing results folders found, cannot perform dry run. Exiting.")
    exit(1)
# EO Delete old results from Python side

# Create EnvModels. (Even for dry run since we need the jsons.)
dummy_result = create_string_buffer(2000)
script_envmodels = r"""
@{./src/templates/}@.stringToHeap[MY_PATH]
@{../../morty/envmodel_config.tpl.json}@.generateEnvmodels
"""
with morty_script_context() as morty_lib:
    dummy_result = morty_lib.expandScript(script_envmodels.encode('utf-8'), dummy_result, sizeof(dummy_result)).decode()
# EO Create EnvModels.
    
with open('morty/envmodel_config.json') as f:
    d = json.load(f) # Take only the [0]th config here because on Python side there are no differences for now.
    nonegos = d[ucd_config_prios_str[0]]["NONEGOS"]
    num_actual_lanes = d[ucd_config_prios_str[0]]["NUMLANES"]
    num_technical_lanes = d[ucd_config_prios_str[0]]["LATERAL_LC_GRANULARITY"] + num_actual_lanes
    maxspeed = d[ucd_config_prios_str[0]]["MAXSPEEDNONEGO"]
    backward_driving_car_ids_str = d[ucd_config_prios_str[0]]["BACKWARD_DRIVING_CAR_IDS"]
    # min_time_between_lcs = d[ucd_config_prios_str[0]]["MIN_TIME_BETWEEN_LANECHANGES"]       # HERE we DO make a difference, see below!
    max_speed = d[ucd_config_prios_str[0]]["MAXSPEEDNONEGO"]
    max_start_speed = min(d[ucd_config_prios_str[0]]["MAXSTARTSPEEDNONEGO_UCD"], max_speed)
    min_start_speed = min(d[ucd_config_prios_str[0]]["MINSTARTSPEEDNONEGO_UCD"], max_start_speed)
    exp_num = d[ucd_config_prios_str[0]]["UCD_EXP_NUM"]
    dist_scale = d[ucd_config_prios_str[0]]["DISTANCESCALING"] / 1000
    time_scale = d[ucd_config_prios_str[0]]["TIMESCALING"] / 1000
    # distance_formula_pp = d[ucd_config_prios_str[0]]["UCD_FORMULA_PP_DISTANCE"]             # HERE we DO make a difference, see below!
    vel_scale = dist_scale / time_scale  # velocity = distance / time

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

TARGET_VEL = 50 # Target velocity in real-world m/s.
MC_TARGET_VEL = int(TARGET_VEL * vel_scale) # Scaled to MC-space.
SPECS.append("INVARSPEC !(@{ env.veh___6[i]9___.v = %r }@.for[[i], 0, %r, 1, & ]);" % (MC_TARGET_VEL, nonegos - 1)) # 1: All cars at target velocity.

SPECS.append("INVARSPEC !(@{ env.veh___609___.on_lane_max = env.veh___6[i]9___.on_lane_max}@.for[[i], 1, %r, 1, & ]);" % (nonegos - 1)) # 2: All cars at max one lane width apart in lateral direction.

TARGET_DIST = 20 # Target distance in real-world meters.
TARGET_VEL_LOW = 10 # Target velocity lower bound in real-world m/s.
TARGET_VEL_HIGH = 20 # Target velocity upper bound in real-world m/s.
MC_TARGET_DIST = int(TARGET_DIST * dist_scale) # Scaled to MC-space.
MC_TARGET_VEL_LOW = int(TARGET_VEL_LOW * vel_scale) # Scaled to MC-space.
MC_TARGET_VEL_HIGH = int(TARGET_VEL_HIGH * vel_scale) # Scaled to MC-space.
SPECS.append(f"""INVARSPEC !(env.veh___609___.abs_pos >= env.veh___619___.abs_pos - {MC_TARGET_DIST}
 & env.veh___619___.abs_pos >= env.veh___629___.abs_pos - {MC_TARGET_DIST}
 & env.veh___629___.abs_pos >= env.veh___639___.abs_pos - {MC_TARGET_DIST}
 & env.veh___639___.abs_pos >= env.veh___649___.abs_pos - {MC_TARGET_DIST}
 & env.veh___609___.abs_pos <= env.veh___619___.abs_pos
 & env.veh___619___.abs_pos <= env.veh___629___.abs_pos
 & env.veh___629___.abs_pos <= env.veh___639___.abs_pos
 & env.veh___639___.abs_pos <= env.veh___649___.abs_pos
 & env.veh___609___.v <= {MC_TARGET_VEL_LOW}
 & env.veh___619___.v <= {MC_TARGET_VEL_LOW}
 & env.veh___629___.v <= {MC_TARGET_VEL_LOW}
 & env.veh___639___.v <= {MC_TARGET_VEL_LOW}
 & env.veh___649___.v >= {MC_TARGET_VEL_HIGH}
);
""") # 3: All cars longitudinally close to each other, one drives faster.

SPECS.append(("INVARSPEC !(env.veh___609___.abs_pos - env.veh___6%r9___.abs_pos < %r " % (nonegos - 1, int(50 * dist_scale))) 
             + ("@{& env.veh___6@{[i] - 1}@.eval[0]9___.abs_pos > env.veh___6[i]9___.abs_pos}@*.for[[i], 1, %r]);" % (nonegos - 1))) 
            # 4: Reversed order of cars, i.e. the first car is the one with the highest abs_pos.

SPECS.append(r"""INVARSPEC TRUE;""") # 5: Benchmark 1.
SPECS.append(r"""INVARSPEC FALSE;""") # 6: Benchmark 2.

SPECS.append(f"INVARSPEC !(env.veh___609___.abs_pos >= env.veh___649___.abs_pos & env.veh___619___.abs_pos <= env.veh___629___.abs_pos);") # 7

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
    lambda: egos_x[0] >= egos_x[4] and egos_x[1] <= egos_x[2]
    ) #7
SUCC_CONDS.append(lambda: False) # 8

addons = [''] * len(SPECS) # Add to main.smv depending on the experiment.
ADDONS_CORE_DENOTER = "UCD addons"
ADDONS_BEGIN_DENOTER = "-- " + ADDONS_CORE_DENOTER + " --\n"
ADDONS_END_DENOTER = "-- EO " + ADDONS_CORE_DENOTER + " --\n"

for i in range(0, len(SPECS)):
    addons[i] += ADDONS_BEGIN_DENOTER
    addons[i] += f"-- UCD experiment{i} --\n"

MC_MIN_V_FORWARD = int(10 * vel_scale)
MC_MAX_V_BACKWARD = int(-10 * vel_scale)
addons[7] += f"""
-- INVAR env.veh___609___.v >= {MC_MIN_V_FORWARD};
-- INVAR env.veh___619___.v <= {MC_MAX_V_BACKWARD};
TRANS (((env.veh___619___.abs_pos - env.veh___609___.abs_pos) < 0)
       != ((next(env.veh___619___.abs_pos) - next(env.veh___609___.abs_pos)) < 0))
       -> ((env.veh___609___.on_normalized_lane = next(env.veh___609___.on_normalized_lane)) & 
          (env.veh___619___.on_normalized_lane = next(env.veh___619___.on_normalized_lane)));
"""

for i in range(2, nonegos):
    addons[7] += f"INVAR env.veh___6{i}9___.v = 0;\n"

addons[8] += f"INVAR env.veh___609___.v >= {MC_MIN_V_FORWARD};\n"
for i in range(1, nonegos):
    addons[8] += f"INVAR env.veh___6{i}9___.v = 0;\n"

for i in range(0, len(SPECS)):
    addons[i] += ADDONS_END_DENOTER


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
all_cex_length_histories = {}
all_selected_runtime_histories = {}

specification = create_string_buffer(2000)
with morty_script_context() as morty_lib:
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

def clean_and_convert_to_int(data):
    """
    Recursively cleans nested lists and converts deepest string values to integers.
    """
    if not isinstance(data, list): # If the item is not a list, it's at the deepest level.
        try:
            return int(data)
        except (ValueError, TypeError):
            return None # Empty string treatment

    cleaned_list = [clean_and_convert_to_int(item) for item in data]
    return [item for item in cleaned_list if item is not None and item != []]

baseline_hashes = {}  # Pre-loop file set (before any MC call).
snapshot_hashes = {}  # Post-first-MC-call hashes (only baseline files).

# Take pre-loop baseline: record which files exist before any MC call.
if args.detailed_archive:
    baseline_hashes = _snapshot_configs(ucd_config_prios_str, generated_path_prefix)

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
        "show_trajectories": False
    })

    # Monkey-patch WorldSurface to center the display on all vehicles instead of a single ego.
    try:
        from highway_env.road.graphics import WorldSurface
        import highway_env as _he
        import pygame  # Required to draw/scale images

        BACKGROUND_IMAGE_PATH = "./examples/crossing.pngg" # Remove last g for POC

        try:
            # Load image independently from display; convert once a display surface exists.
            bg_image = pygame.image.load(BACKGROUND_IMAGE_PATH)
            bg_image_state = {
                "image": bg_image,
                "converted": False,
            }
        except Exception as e:
            bg_image_state = None
            pass

        _orig_move_display_window_to = WorldSurface.move_display_window_to

        def _move_display_window_to_all(self, position):
            try:
                env_ref = getattr(_he, '_display_env', None)
                if env_ref is None:
                    return _orig_move_display_window_to(self, position)

                vehicles = getattr(env_ref.unwrapped, 'road').vehicles
                if not vehicles:
                    return _orig_move_display_window_to(self, position)

                bbox = get_scene_bounding_box(env_ref, [i for i in range(1000)])

                padding = 10
                target_width_m = (bbox["xmax"] - bbox["xmin"]) + (padding * 2)
                target_height_m = (bbox["ymax"] - bbox["ymin"]) + (padding * 2)
                screen_width, screen_height = self.get_width(), self.get_height()
                scale_x = screen_width / max(1.0, target_width_m)
                scale_y = screen_height / max(1.0, target_height_m)
                self.scaling = min(scale_x, scale_y)
    
                center = np.array([(bbox["xmin"] + bbox["xmax"]) / 2.0 - 50, (bbox["ymin"] + bbox["ymax"]) / 2.0])

                self.origin = center - np.array([
                    self.centering_position[0] * self.get_width() / self.scaling,
                    self.centering_position[1] * self.get_height() / self.scaling,
                ])
                
                if bg_image_state is not None and bg_image_state["image"] is not None:
                    # convert_alpha requires an initialized display/surface.
                    if (not bg_image_state["converted"] and pygame.display.get_init()
                            and pygame.display.get_surface() is not None):
                        bg_image_state["image"] = bg_image_state["image"].convert_alpha()
                        bg_image_state["converted"] = True

                    # Clear the screen first since we bypass self.fill below
                    self.fill(self.GREY) 
                    
                    # Scale image based on camera zoom factor (Assuming 300m x 150m real-world size)
                    scaled_bg = pygame.transform.scale(bg_image_state["image"], (int(300 * self.scaling), int(150 * self.scaling)))
                    
                    # Draw background locked to world coordinates (0,0)
                    self.blit(scaled_bg, self.vec2pix((0, 0)))

                    # Monkey-patch self.fill temporarily to block the default 'fill(self.GREY)' from painting over it
                    orig_fill = self.fill
                    self.fill = lambda color, rect=None, special_flags=0: None

            except Exception as e:
                _orig_move_display_window_to(self, position)
                raise e

        WorldSurface.move_display_window_to = _move_display_window_to_all
        # Provide a hook so the patched function can find the current env.
        _he._display_env = None
    except Exception as e:
        raise e

    # Expose the env reference so the patched WorldSurface can access vehicle positions.
    try:
        import highway_env as _he
        _he._display_env = env
    except Exception as e:
        raise e

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
    cex_length_history = []
    cnt_history = []
    cex_point_colors = []
    mc_runtime_histories = {config_name: [] for config_name in ucd_config_prios_str}
    selected_cnt_history = []
    selected_runtime_history = []

    # Mark backward driving cars.
    for i in backward_driving_car_ids:
        egos_backward[i] = -1
        
    env.reset(seed=seedo)

    # Calculate valid y range for initialization (same as used in pure pursuit later)
    LANE_WIDTH_HE = 4.0  # highway-env lane width in meters
    on_lane_step_y = 2.0 * num_actual_lanes / num_technical_lanes  # y-distance per MC on_lane position
    # Compute valid y range based on technical lane positions.
    y_min_tech = -LANE_WIDTH_HE / 2.0 + LANE_WIDTH_HE * num_actual_lanes / (2.0 * num_technical_lanes)
    y_max_tech = -LANE_WIDTH_HE / 2.0 + (2 * num_technical_lanes - 1) * LANE_WIDTH_HE * num_actual_lanes / (2.0 * num_technical_lanes)

    np.random.seed(seedo)
    rng = np.random.default_rng(seedo)
    cnt = 0
    for vehicle in env.unwrapped.controlled_vehicles:
        vehicle.color = (255, 255, 255)
        vehicle.heading = np.pi * (1 - egos_backward[cnt]) / 2 # 0 for forward, pi for backward.
        vehicle.speed = np.random.uniform(min_start_speed, max_start_speed)
        if exp_num == 7: # nudging
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
        
        # shift cars a little with gauss distribution (scale is deviation in meters for 2/3).
        vehicle.position[0] += rng.normal(loc=0.0, scale=1)
        vehicle.position[1] += rng.normal(loc=0.0, scale=1)
        
        # Clamp lateral position to valid road boundaries
        vehicle.position[1] = max(min(vehicle.position[1], y_max_tech), y_min_tech)
        
        cnt = cnt + 1

    # COP: Reset and seed trajectories after all manual vehicle repositioning.
    _vehicle_trajectories.clear()
    _vehicle_pp_targets.clear()
    for vehicle in env.unwrapped.controlled_vehicles:
        _vehicle_trajectories[id(vehicle)] = [[float(vehicle.position[0]), float(vehicle.position[1]), None]]
        _vehicle_pp_targets[id(vehicle)] = [float(vehicle.position[0]), float(vehicle.position[1])]

    if args.record_video:
        env = RecordVideo(env, video_folder=f"{generated_path_prefix}/videos", name_prefix=f"vid_{seedo}",
                    episode_trigger=lambda e: True)  # record all episodes
        env.unwrapped.set_record_video_wrapper(env)
        env.start_recording(f"vid_{seedo}")
        try:
            import highway_env as _he
            _he._display_env = env
        except Exception:
            pass

    first = True
    obs = env.unwrapped.observation_type.observe()
    blind_stats = ""
    crashed = False

    if args.detailed_archive:
        _save_configs_to_archive(seedo, '0_pristine', generated_path_prefix, ucd_config_prios_str)

    # Prepare dry run
    if args.dryrun:
        target_dir = f"{generated_path_prefix}/detailed_archive/run_{seedo}"
        zip_file = f"{generated_path_prefix}/detailed_archive/run_{seedo}.zip.zip"
        
        if Path(zip_file).is_file():
            print(f"Extracting data from {zip_file} into {target_dir}")
            if Path(target_dir).is_dir():
                print(f"\---> Removing existing directory {target_dir}.")
                shutil.rmtree(target_dir)
            shutil.unpack_archive(zip_file, target_dir)
        else:
            print(f"Warning: Zip file {zip_file} not found for dry run, assuming folder is already extracted.")
            if not Path(target_dir).is_dir():
                print(f"\---> Warning: Target directory {target_dir} not found for dry run. Skipping dry run for seed {seedo}.")
                continue
        
        for config_name in ucd_config_prios_str:
            config_path = f"{target_dir}/0_pristine/{config_name}/"
            scalefile = f"{config_path}/scaling_info.txt"
    # EO Prepare dry run

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

            }}@.nil
            @{{}}@.prepareOutputForMortyUCD[{str(seedo)}, {str(global_counter)}, {0}, {str(crashed)}]
        """
        # EO The script to run the MC on C++ side.

        # Prepare dry run        
        if args.dryrun:
            if not Path(target_dir + f'/iteration_{global_counter}').is_dir():
                print(f"\---> Iteration {target_dir + f'/iteration_{global_counter}'} not found.")
            else:                
                for res_path in glob.glob(generated_path_prefix + "_config*"):
                    if Path(res_path).is_dir():
                        print(f"\---> Removing {res_path} from previous iteration.")
                        shutil.rmtree(res_path)

                if Path(scalefile).is_file():
                    for config_prio in ucd_config_prios_str:
                        scalefile_target = target_dir + f'/iteration_{global_counter}/{config_prio}/scaling_info.txt'
                        print(f"\---> Copying scaling info into {scalefile_target}.")
                        shutil.copyfile(scalefile, scalefile_target)
                else:
                    print(f"\---> Error: Scaling info file {scalefile} not found for dry run. Going on with 1/1.")
            
                for config_prio in ucd_config_prios_str:
                    dest_path = f"{generated_path_prefix}{config_prio}"
                    print(f"\---> Copying archived iteration into {dest_path}.")
                    shutil.copytree(f"{target_dir}/iteration_{global_counter}/{config_prio}", dest_path)

            MC_SCRIPT = f"@{{}}@.prepareOutputForMortyUCD[{str(seedo)}, {str(global_counter)}, {0}, {str(crashed)}]"
        # EO Prepare dry run

        first = False
        
        empty_cex = ""
        
        for _i in range(nonegos):
            empty_cex += "|;"
        
        #### MODEL CHECKER CALL ####
        result = create_string_buffer(100000)
        with morty_script_context() as morty_lib:
            res = morty_lib.expandScript(MC_SCRIPT.encode('utf-8'), result, sizeof(result))
        res_str = res.decode().strip()

        if args.detailed_archive and global_counter == 0:
            # Processed snapshot: only re-hash files that existed in the baseline
            # (excludes iteration-specific artifacts like debug folders).
            snapshot_hashes = _snapshot_configs(ucd_config_prios_str, generated_path_prefix, restrict_to=baseline_hashes)
            _save_configs_to_archive(seedo, '1_initialized', generated_path_prefix, ucd_config_prios_str, restrict_to={cn: set(baseline_hashes[cn]) for cn in baseline_hashes})
        
        ### POSTPROCESS RESULT ###
        all_results_dict = {}
        for single_res in res_str.split('\n'):
            if single_res:
                print("\n'" + single_res + "'\n")
                config_name, res_str = single_res.split(':') # initiate res_str with ANY of the results, will get updated later if none-blind exists.
                all_results_dict[config_name] = res_str
        
        blind = "|X"
        selected_cnt = None
        for cnt, config_name in enumerate(ucd_config_prios_str):
            if (all_results_dict[config_name] == empty_cex):
                print(f"CEX #{cnt} is EMPTY.")
            else:
                res_str = all_results_dict[config_name]
                print(f"Picked CEX #{cnt} [{config_name}] which is the first non-empty one.")
                blind = "|" + str(cnt)
                selected_cnt = cnt
                break;
        
        # Update global priority for trajectory coloring
        _current_selected_cnt = selected_cnt
        
        blind_stats += blind

        # Track latest nuXmv runtime per configured priority and update PDF each iteration.
        selected_cnt_history.append(selected_cnt)
        for config_name in ucd_config_prios_str:
            mc_runtime_histories[config_name].append(_latest_nuxmv_runtime_seconds(config_name, generated_path_prefix))

        if selected_cnt is not None and 0 <= selected_cnt < len(ucd_config_prios_str):
            selected_config = ucd_config_prios_str[selected_cnt]
            selected_runtime = mc_runtime_histories[selected_config][-1]
        else:
            selected_runtime = np.nan
        selected_runtime_history.append(selected_runtime)

        plot_mc_runtimes(
            mc_runtime_histories,
            f"{generated_path_prefix}/mc_runtime_debug_{seedo}.pdf",
            selected_cnt_history=selected_cnt_history,
        )
        all_selected_runtime_histories[seedo] = selected_runtime_history[:]
        plot_mc_runtimes_cumulative(all_selected_runtime_histories, f"{generated_path_prefix}/mc_runtime_debug_all.pdf")
        plot_mc_runtimes_cumulative(all_selected_runtime_histories, f"{generated_path_prefix}/mc_runtime_debug_all_log.pdf", log_scale=True)
        # EO Track latest nuXmv runtime per configured priority and update PDF each iteration.
        
        # Find future positions.
        if not args.hide_planned_positions:
            MC_SCRIPT = f"""@{{
                @{{./src/templates/}}@.stringToHeap[MY_PATH]
                }}@.nil

                @{{../../morty/envmodel_config.tpl.json}}@.extractVehPosFromNusmvFile[{selected_config}]
            """
            result_pos = create_string_buffer(100000)
            with morty_script_context() as morty_lib:
                res_pos = morty_lib.expandScript(MC_SCRIPT.encode('utf-8'), result_pos, sizeof(result_pos))
            
            res_pos_str = res_pos.decode().strip()
            positions = [[els.split(',') for els in line.split(';')] for line in res_pos_str.split('\n')]
            positions = clean_and_convert_to_int(positions)
            
            POS_COLOR = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255), (0, 255, 255)]
            
            global_pos_to_draw.clear()
            for step in positions:
                for i, car_step in enumerate(step):
                    abspos = car_step[0]
                    technical_lane = car_step[1]
                    coord = (abspos / 4, y_max_tech - technical_lane * on_lane_step_y)
                    global_pos_to_draw.append([coord, POS_COLOR[i % len(POS_COLOR)]])                
        # EO Find future positions.

        
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
            archive(seedo, global_counter, args.detailed_archive, generated_path_prefix, ucd_config_prios_str, snapshot_hashes)
            break
       
        if res_str == empty_cex:
            print("No CEX found")
            for i in range(nonegos): # Set blue when blind.
                if not env.unwrapped.controlled_vehicles[i].crashed:
                    env.unwrapped.controlled_vehicles[i].color = (100, 100, 255)
            if nocex_count > args.allow_blind_steps: # Allow up to this many times being blind per run. (If in doubt: 10)
                archive(seedo, global_counter, args.detailed_archive, generated_path_prefix, ucd_config_prios_str, snapshot_hashes)
                break
            nocex_count += 1
            
            cex_length_history.append(np.nan)
            cnt_history.append(-1)
            cex_point_colors.append('tab:red')
        else:
            cex_length = len(res_str.split(';')[0].split('|')[0].split(','))
            cex_length_history.append(cex_length)
            cnt_history.append(cnt)
            cex_point_colors.append('tab:blue')
            
            for i in range(nonegos):
                if not env.unwrapped.controlled_vehicles[i].crashed:
                    env.unwrapped.controlled_vehicles[i].color = (min(cex_length * 15, 255), 200, min(cex_length * 15, 255))

        # Plotting (TODO: refactor to a function to avoid code duplication with the MC runtime plotting above):
        plot_cex_lengths(
            cex_length_history,
            f"{generated_path_prefix}/cex_length_debug_{seedo}.pdf",
            cnt_history=cnt_history,
            point_colors=cex_point_colors,
        )
        all_cex_length_histories[seedo] = cex_length_history[:]
        plot_cex_lengths_cumulative(all_cex_length_histories, f"{generated_path_prefix}/cex_length_debug_all.pdf")
        # EO Plotting

        sum_vel_by_car = []
        sum_lan_by_car = []

        lanes = ""
        accels = ""

        # Best so far:
        # LANE_CHANGE_DURATION = 3
        min_time_between_lcs = d[selected_config]["MIN_TIME_BETWEEN_LANECHANGES"]
        LANE_CHANGE_DURATION = min_time_between_lcs * time_scale # Index in the MC delta trace where the lane change effect appears.

        # Process the MC data.
        mc_car_results = [segment for segment in res_str.split(';') if segment]
        for i1, el1 in enumerate(mc_car_results):
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

            # Formula for speed-dependent distance in pure-pursuit.
            distance_formula_pp = d[selected_config]["UCD_FORMULA_PP_DISTANCE"]
            distance = create_string_buffer(100)
            script_dist_pp = f"""@{{
            @{{@velocity = {egos_v[i]}}}@.eval
            @{{@min_time_between_lcs = {min_time_between_lcs}}}@.eval
            }}@.nil 
            @{{{distance_formula_pp}}}@.eval
            """
            with morty_script_context() as morty_lib:
                distance = morty_lib.expandScript(script_dist_pp.encode('utf-8'), distance, sizeof(distance)).decode().strip()
            # EO Formula for speed-dependent distance in pure-pursuit.

            pp_distance = float(distance)
            if i < len(env.unwrapped.controlled_vehicles):
                vehicle = env.unwrapped.controlled_vehicles[i]
                pp_target_x = float(vehicle.position[0] + egos_backward[i] * pp_distance)
                pp_target_y = float(dpoints_y[i])
                _vehicle_pp_targets[id(vehicle)] = [pp_target_x, pp_target_y]

            # Best so far (for inversion task):
            # angle = -dpoint_following_angle(dpoints_y[i], egos_y[i], egos_headings[i], 10 + 2 * egos_v[i], egos_backward[i]) / 3.1415 # Magic constants, get over it ;)
            angle = -dpoint_following_angle(dpoints_y[i], egos_y[i], egos_headings[i], pp_distance, egos_backward[i]) / 3.1415
            
            #if egos_backward[i] == -1: # If the car is backward-driving, we have to adapt the angle to the different perspective.
             #   print(angle)
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

        if not args.headless:
            env.render()

        # Flush partial video every iteration.
        if args.record_video and not args.dryrun and hasattr(env, 'recorded_frames') and len(env.recorded_frames) > 0:
            from moviepy.video.io.ImageSequenceClip import ImageSequenceClip
            clip = ImageSequenceClip(env.recorded_frames, fps=env.frames_per_sec)
            clip.write_videofile(f"{generated_path_prefix}/videos/vid_{seedo}_partial.mp4", logger=None)
            clip.close()
            print("Video written")

        archive(seedo, global_counter, args.detailed_archive, generated_path_prefix, ucd_config_prios_str, snapshot_hashes)

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
            for video_file in glob.glob(f"{generated_path_prefix}/videos/vid_{seedo}*"):
                new_name = video_file.replace(f"vid_{seedo}", f"vid_{seedo}{suffix}", 1)
                os.rename(video_file, new_name)

    if args.detailed_archive:
        folder_path = f'{generated_path_prefix}/detailed_archive/run_{seedo}'
        print(f"Zipping detailed archive folder {folder_path}.")
        shutil.make_archive(folder_path + ".zip", 'zip', folder_path)
        shutil.rmtree(folder_path)

print(good_ones)
