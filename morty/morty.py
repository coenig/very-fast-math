import gymnasium
import highway_env
from matplotlib import pyplot as plt
from ctypes import *
import math
from pathlib import Path
import shutil
import argparse

MAIN_TEMPLATE = r"""#include "planner.cpp_combined.k2.smv"
#include "EnvModel.smv"

MODULE Globals
VAR
"loc" : boolean;

MODULE main
VAR
  globals : Globals; 
  env : EnvModel;
  planner : "checkLCConditionsFastLane"(globals);

"""

SPECS = []      # Predefined specs as checked in the experiments. (Collision freedom is implicit.)
SUCC_CONDS = [] # Conditions determining success of the specs.

SPECS.append(r"""INVARSPEC !(env.veh___609___.v = 0
 & env.veh___619___.v = 0
 & env.veh___629___.v = 0
 & env.veh___639___.v = 0
 & env.veh___649___.v = 0
);
""") # 0: All cars stopped.

TARGET_VEL = 50 # Target velocity for the next spec.
SPECS.append(f"""INVARSPEC !(env.veh___609___.v = {TARGET_VEL}
 & env.veh___619___.v = {TARGET_VEL}
 & env.veh___629___.v = {TARGET_VEL}
 & env.veh___639___.v = {TARGET_VEL}
 & env.veh___649___.v = {TARGET_VEL}
);
""") # 1: All cars at target velocity.

SPECS.append(f"""INVARSPEC !(env.veh___609___.on_lane_max = env.veh___619___.on_lane_max
 & env.veh___619___.on_lane_max = env.veh___629___.on_lane_max
 & env.veh___629___.on_lane_max = env.veh___639___.on_lane_max
 & env.veh___639___.on_lane_max = env.veh___649___.on_lane_max
);
""") # 2: All cars at max one lane width apart in lateral direction.

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

SPECS.append(r"""INVARSPEC !(env.veh___609___.abs_pos - env.veh___649___.abs_pos < 50
 & env.veh___609___.abs_pos > env.veh___619___.abs_pos 
 & env.veh___619___.abs_pos > env.veh___629___.abs_pos 
 & env.veh___629___.abs_pos > env.veh___639___.abs_pos 
 & env.veh___639___.abs_pos > env.veh___649___.abs_pos
);
""") # 4: Reversed order of cars, i.e. the first car is the one with the highest abs_pos.

SUCC_CONDS.append(lambda: egos_v[0] < 1 and egos_v[1] < 1 and egos_v[2] < 1 and egos_v[3] < 1 and egos_v[4] < 1)
SUCC_CONDS.append(lambda: abs(egos_v[0] - TARGET_VEL) < 1 and abs(egos_v[1] - TARGET_VEL) < 1 and abs(egos_v[2] - TARGET_VEL) < 1 and abs(egos_v[3] - TARGET_VEL) < 1 and abs(egos_v[4] - TARGET_VEL) < 1)
SUCC_CONDS.append(lambda: maxDifferenceArray(egos_y[:-1]) < 4)
SUCC_CONDS.append(lambda: egos_x[0] >= egos_x[1] - TARGET_DIST and egos_x[1] >= egos_x[2] - TARGET_DIST and egos_x[2] >= egos_x[3] - TARGET_DIST and egos_x[3] >= egos_x[4] - TARGET_DIST
                  and egos_v[0] <= TARGET_VEL_LOW and egos_v[1] <= TARGET_VEL_LOW and egos_v[2] <= TARGET_VEL_LOW and egos_v[3] <= TARGET_VEL_LOW and egos_v[4] >= TARGET_VEL_HIGH)
SUCC_CONDS.append(lambda: egos_x[4] < egos_x[3] and egos_x[3] < egos_x[2] and egos_x[2] < egos_x[1] and egos_x[1] < egos_x[0])

parser = argparse.ArgumentParser(
                    prog='morty',
                    description='Model Checking based planning',
                    epilog='Bye!')
parser.add_argument('-o', '--output', default="./morty")
parser.add_argument('-n', '--num_runs', default=1000)
parser.add_argument('-s', '--steps_per_run', default=100)
parser.add_argument('-a', '--heading_adaptation', default=-0.5)
parser.add_argument('-b', '--allow_blind_steps', default=100)
parser.add_argument('-c', '--allow_crashed_steps', default=100)
parser.add_argument('-d', '--debug', default=True)
parser.add_argument('-e', '--exp_num', default=0)
args = parser.parse_args()

output_folder = args.output + "/"

# Best so far:
# ACCEL_RANGE = 5
ACCEL_RANGE = 5

MAX_EXPs = args.num_runs

morty_lib = CDLL('./lib/libvfm.so')
morty_lib.morty.argtypes = [c_char_p, c_char_p, c_size_t]
morty_lib.morty.restype = c_char_p
    
good_ones = []

def min_max_curr(successful_so_far, done_so_far, max_to_expect):
    percent = 100 * successful_so_far / done_so_far
    min_good = 100 * successful_so_far / max_to_expect
    max_good = 100 * (max_to_expect - done_so_far + successful_so_far) / max_to_expect
    return str(round(min_good, 1)) + "% <= " + str(round(percent, 1)) + "% <= " + str(round(max_good, 1)) + "%"

def dpoint_following_angle(dpoint_y, ego_y, heading, ddist):
    return heading - math.atan((dpoint_y - ego_y) / ddist)


def maxDifferenceArray(A):
    maxDiff = -1
    for i in range(len(A)):
        for j in range(len(A)):
            maxDiff = max(maxDiff, abs(A[j] - A[i]))
    return maxDiff

open(f'{output_folder}results.txt', 'w').close()          # Delete old results from Python side
open(f'{output_folder}morty_mc_results.txt', 'w').close() # Delete old results from MC side (these are a super set of the above)

with open(f'{output_folder}main.tpl', "w") as text_file:
    text_file.write(MAIN_TEMPLATE + SPECS[args.exp_num])

for seedo in range(0, MAX_EXPs):
    env = gymnasium.make('highway-v0', render_mode='rgb_array', config={
        "action": {
            "type": "MultiAgentAction",
            "action_config": {
                "type": "ContinuousAction",
                "acceleration_range": [-ACCEL_RANGE, ACCEL_RANGE],
                "steering_range": [-1, 1],
                "speed_range": [0, 70], # Best so far: [0, 70]. Don't forget to adjust EnvModel.smv ==> max_vel
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
        "controlled_vehicles": 5,
        "vehicles_count": 0,
        "screen_width": 1500,
        "screen_height": 500,
        "scaling": 5,
        "show_trajectories": True,
    })

    env.reset(seed=seedo) # Use some factor due to (unproven) suspicion that close-by seeds lead to similar starting positions.

    action = ([0, 0], [0, 0], [0, 0], [0, 0], [0, 0])
    dpoints_y = [0, 0, 0, 0, 0, 0]     # The lateral position of the points the cars head towards.
    dpoints_delta = [0, 0, 0, 0, 0, 0] # The direction we're currently moving towards, laterally.
    lc_time = [0, 0, 0, 0, 0, 0]       # Time the current lc is taking, abort if too long. Needed to better approximate MC behavior.
    egos_x = [0, 0, 0, 0, 0, 0]        # Long pos of cars in m.
    egos_y = [0, 0, 0, 0, 0, 0]        # Lat pos of cars in m.
    egos_v = [0, 0, 0, 0, 0, 0]        # Long vel of cars in m/s.
    egos_headings = [0, 0, 0, 0, 0, 0] # Angle rel. to long axis in rad.
    nocex_count = 0
    crashed_count = 0

    first = True
    for global_counter in range(args.steps_per_run):
        env.render()
        obs, reward, done, truncated, info = env.step(action)
        
        input = ""
        i = 0
        for el in obs: # Use only el[0] because it contains the abs values from the resp. car's perspective.
            if first:
                dpoints_y[i] = el[0][2] # Set desired lateral position to the actual position in the first step.
                
            egos_x[i] = el[0][1]
            egos_y[i] = el[0][2]
            egos_v[i] = el[0][3]
            egos_headings[i] = el[0][5]
            i = i + 1
            for val in el[0]: # Generate input for model checker.
                input += str(val) + ","
            input += ";"
        
        crashed = info["crashed"]
        
        if crashed:
            crashed_count += 1
        
        if crashed_count > args.allow_crashed_steps: # Allow these many crashes per run.
            break

        input += "$$$1$$$" + str(args.debug) + "$$$" + str(args.heading_adaptation) + "$$$" + str(seedo) + "$$$" + str(crashed) + "$$$" + str(global_counter) + "$$$" + output_folder
        
        first = False
        
        #### MODEL CHECKER CALL ####
        result = create_string_buffer(10000)
        res = morty_lib.morty(input.encode('utf-8'), result, sizeof(result))
        res_str = res.decode()
        
        # We check both the MC result and the success condition to determine if we are done. Reason:
        # The MC result alone is not sufficient because the MC cares only about the SPEC being satisfied in the last step.
        # Here we check AFTER the last step, a situation which is not guaranteed to have a solution, as well, so we might
        # get a false negative. Checking only the success condition would be possible; we add the additional
        # MC check to make it easier to implement new SPECs. Then, a first impression is possible even
        # if no success condition is given or if it is not implemented in a precise way. 
        if res_str == "FINISHED" or SUCC_CONDS[args.exp_num]():
            print("DONE")
            good_ones.append(seedo)            
            break
        
        if res_str == "|;|;|;|;|;":
            print("No CEX found")
            if nocex_count > args.allow_blind_steps: # Allow up to this many times being blind per run. (If in doubt: 10)
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
        LANE_CHANGE_DURATION = 3 # Has to match EnvModel.smv ==> min_time_between_lcs

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
        
        action_list = []
        
        # Best so far:
        # MAXTIME_FOR_LC = 60
        MAXTIME_FOR_LC = 60
        
        eps = 1
        for i, el in enumerate(sum_vel_by_car):
            lc_time[i] += 1
            
            if abs(dpoints_y[i] - egos_y[i]) < eps:
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
            
            dpoints_y[i] = max(min(dpoints_y[i], 12), 0)
            
            accel = sum_vel_by_car[i] * 5/3 / ACCEL_RANGE # 5/3 is the factor from EnvModel ==> a_min/a_max to ACCEL_RANGE, a discrepancy accounting for highway-env adjusting acceleration smoothly while the MC only does it once/s.

            # Best so far:
            # angle = -dpoint_following_angle(dpoints_y[i], egos_y[i], egos_headings[i], 10 + 2 * egos_v[i]) / 3.1415
            angle = -dpoint_following_angle(dpoints_y[i], egos_y[i], egos_headings[i], 10 + 2 * egos_v[i]) / 3.1415 # Magic constants, get over it ;)
            action_list.append([accel, angle])
        
        #print(action_list_vel)
        #print(action_list_lane)
        
        action = tuple(action_list)

    with open(f"{output_folder}results.txt", "a") as f:
        f.write("{" + min_max_curr(len(good_ones), seedo + 1, MAX_EXPs) + "} " + ' '.join(str(x) for x in good_ones) + " [" + str(nocex_count) + " blind]\n")
    
print(good_ones)