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

# Load parameters from JSON.
with open('morty/envmodel_config.tpl.json') as f:
    d = json.load(f)
    nonegos = d["#TEMPLATE"]["NONEGOS"]
    num_actual_lanes = d["#TEMPLATE"]["NUMLANES"]
    num_technical_lanes = d["#TEMPLATE"]["LATERAL_LC_GRANULARITY"] + num_actual_lanes
    maxspeed = d["#TEMPLATE"]["MAXSPEEDNONEGO"]
    backward_driving_car_ids_str = d["#TEMPLATE"]["BACKWARD_DRIVING_CAR_IDS"]
    min_time_between_lcs = d["#TEMPLATE"]["MIN_TIME_BETWEEN_LANECHANGES"]

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

MAIN_TEMPLATE = r"""#include "planner.cpp_combined.k2.smv"
#include "EnvModel.smv"

MODULE Globals
VAR
"loc" : boolean;

MODULE main
VAR
  globals : Globals; 
  env : EnvModel;
  planner : "checkLCConditionsFastLane"(globals."loc");

INVAR env.ego.v = env.veh___609___.v;
"""


morty_lib = CDLL('./lib/libvfm.so')
morty_lib.expandScript.argtypes = [c_char_p, c_char_p, c_size_t]
morty_lib.expandScript.restype = c_char_p
morty_lib.morty.argtypes = [c_char_p, c_char_p, c_size_t]
morty_lib.morty.restype = c_char_p


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


# Add to main.smv depending on the experiment.
addons = [''] * len(SPECS)

addons[7] = r"""
INVAR env.veh___609___.v >= 5;
INVAR env.veh___619___.v <= -5;
INVAR env.veh___609___.v >= 0;
INVAR env.veh___619___.v <= 0;
"""

for i in range(2, nonegos):
    addons[7] += f"INVAR env.veh___6{i}9___.v = 0;\n"


DEFAULT_EXP_ID = 7

parser = argparse.ArgumentParser(
                    prog='morty',
                    description='Model Checking based planning',
                    epilog='Bye!')
parser.add_argument('-o', '--output', default="./morty", type=str,
                    help='Output folder for the results.')
parser.add_argument('-n', '--num_runs', default=1000, type=int,
                    help='Number of runs to perform per experiment. Default: 1000')
parser.add_argument('-s', '--steps_per_run', default=300, type=int,
                    help='Maximum number of steps until the SPEC must be fulfilled. Default: 100')
parser.add_argument('-a', '--heading_adaptation', default=-0.5, type=float,
                    help='How much the heading of the cars is used to adapt their lateral position (see MC code for details). Default: -0.5')
parser.add_argument('-b', '--allow_blind_steps', default=300, type=int,
                    help='How many times the MC is allowed to be blind (no CEX) before the run is aborted. Default: 100')
parser.add_argument('-c', '--allow_crashed_steps', default=1, type=int,
                    help='How many steps with crashes are allowed before the run is aborted. Default: 100')
parser.add_argument('-d', '--debug', default=0, type=int,
                    help='Enable writing images in each step to see what the MC thinks (0 or 1). Default: 0')
parser.add_argument('-e', '--exp_num', default=DEFAULT_EXP_ID, type=int, choices=range(len(SPECS)),
                    help='Experiment id to run. Choose from 0 to {}'.format(len(SPECS)-1))
parser.add_argument('--record_video', action='store_true',
                    help='Record a video of the run. Default: False')
parser.add_argument('--detailed_archive', action='store_true',
                    help='Stores detailed archive of the run in a subfolder. Default: False')
parser.add_argument('--headless', action='store_true',
                    help='Run without opening the simulation UI window. Default: False')
args = parser.parse_args()

if args.headless:
    os.environ['SDL_VIDEODRIVER'] = 'offscreen'

output_folder = args.output + "/"


MAIN_TEMPLATE += addons[args.exp_num]

# Best so far:
# ACCEL_RANGE = 6
ACCEL_RANGE = 60

MAX_EXPs = args.num_runs

good_ones = []

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

open(f'{output_folder}results.txt', 'w').close()          # Delete old results from Python side
open(f'{output_folder}morty_mc_results.txt', 'w').close() # Delete old results from MC side (these are a super set of the above)
# if os.path.exists(f"{output_folder}../detailed_results"):
#     distutils.dir_util.remove_tree(f"{output_folder}../detailed_results") # Delete old detailed results

specification = create_string_buffer(2000)
spec_res = morty_lib.expandScript(SPECS[args.exp_num].encode('utf-8'), specification, sizeof(specification)).decode()

with open(f'{output_folder}main.tpl', "w") as text_file:
    text_file.write(MAIN_TEMPLATE + spec_res)

if not os.path.samefile(output_folder, "./morty/"):
    shutil.copyfile("./morty/EnvModel.smv", output_folder + "EnvModel.smv")
    shutil.copyfile("./morty/planner.cpp_combined.k2", output_folder + "planner.cpp_combined.k2")
    shutil.copyfile("./morty/script.cmd", output_folder + "script.cmd")


def archive(seedo, global_counter):
    if args.detailed_archive:
        archive_path = f'{output_folder}../detailed_results/run_{seedo}/iteration_{global_counter}/'
        if not os.path.exists(archive_path):
            os.makedirs(archive_path)
        open(f'{output_folder}mc_runtimes.txt', 'w').close() # Delete "mc_runtimes.txt" which is not used in this context.
        distutils.dir_util.copy_tree(output_folder, archive_path)

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
        vehicle.heading = np.pi * (1 - egos_backward[cnt]) / 2 # 0 for forward, pi for backward.
        vehicle.speed = np.random.uniform(5, 15)
        if args.exp_num == 7:
            if cnt == 0:
                vehicle.position[0] = 0
            if cnt == 1:
                vehicle.position[0] = (nonegos - 1) * 500 / nonegos
            if cnt > 1:
                vehicle.position[0] = (cnt - 1) * 400 / nonegos
                vehicle.speed = 0
        
        cnt = cnt + 1

    if args.record_video:
        env = RecordVideo(env, video_folder="videos", name_prefix=f"vid_{seedo}",
                    episode_trigger=lambda e: True)  # record all episodes
        env.unwrapped.set_record_video_wrapper(env)
        env.start_video_recorder()

    first = True
    for global_counter in range(args.steps_per_run):
        if not args.headless:
            env.render()
        obs, reward, done, truncated, info = env.step(action)

        mcinput = ""
        i = 0
        for el in obs: # Use only el[0] because it contains the abs values from the resp. car's perspective.
            if first:
                dpoints_y[i] = el[0][2] # Set desired lateral position to the actual position in the first step.
                
            egos_x[i] = el[0][1]
            egos_y[i] = el[0][2]
            egos_v[i] = el[0][3] * egos_backward[i]
            egos_headings[i] = el[0][5] - np.pi * (1 - egos_backward[i]) / 2
            i = i + 1
            for num, val in enumerate(el[0]): # Generate input for model checker.
                if egos_backward[i] == -1 and num == 5:
                    mcinput += str(-val) + ","
                else:
                    mcinput += str(val) + ","
                
            mcinput += ";"
        
        crashed = False # Check if any car has crashed.
        for vehicle in env.unwrapped.road.vehicles:
            crashed = crashed or vehicle.crashed
        
        if crashed:
            crashed_count += 1
        
        if crashed_count > args.allow_crashed_steps: # Allow these many crashes per run.
            archive(seedo, global_counter)
            break

        mcinput += "$$$1$$$" + str(args.debug) \
                 + "$$$" + str(args.heading_adaptation) \
                 + "$$$" + str(seedo) \
                 + "$$$" + str(crashed) \
                 + "$$$" + str(global_counter) \
                 + "$$$" + output_folder \
                 + "$$$" + ("/" if output_folder[0] == "/" else ".") \
                 + "$$$" + str(num_actual_lanes) \
                 + "$$$" + str(args.detailed_archive) \
                 + "$$$" + "True"    # Smooth GIF
        
        first = False
        
        #### MODEL CHECKER CALL ####
        result = create_string_buffer(10000)
        res = morty_lib.morty(mcinput.encode('utf-8'), result, sizeof(result))
        res_str = res.decode()
        successed = SUCC_CONDS[args.exp_num]()
        
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
        
        empty_cex = ""
        
        for _i in range(nonegos):
            empty_cex += "|;"
        
        if res_str == empty_cex:
            print("No CEX found")
            if nocex_count > args.allow_blind_steps: # Allow up to this many times being blind per run. (If in doubt: 10)
                archive(seedo, global_counter)
                break
            nocex_count += 1

        #print(f"result: {res_str}")
        #### EO MODEL CHECKER CALL ####

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

        with open(f"{output_folder}lanes.txt", "w") as text_file:
            text_file.write(lanes)
        with open(f"{output_folder}accels.txt", "w") as text_file:
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
                
                if sum_lan_by_car[i] < 0:
                    dpoints_delta[i] = 2
                    dpoints_y[i] += dpoints_delta[i]
                elif sum_lan_by_car[i] > 0:
                    dpoints_delta[i] = -2
                    dpoints_y[i] += dpoints_delta[i]
            
            if lc_time[i] > MAXTIME_FOR_LC and dpoints_delta[i] != 0:
                dpoints_y[i] -= dpoints_delta[i] # Go back to source lane if taking too long.
                dpoints_delta[i] = 0 # Don't care about cases with no ongoing LC since delta is zero, then.
                lc_time[i] = 0
            
            dpoints_y[i] = max(min(dpoints_y[i], num_actual_lanes * 3), 0)
            
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
        archive(seedo, global_counter)
        
    with open(f"{output_folder}results.txt", "a") as f:
        f.write("{" + min_max_curr(len(good_ones), seedo + 1, MAX_EXPs) + "} " + ' '.join(str(x) for x in good_ones) + " [" + str(nocex_count) + " blind]\n")

    if args.record_video:
        env.close_video_recorder()
    
print(good_ones)
