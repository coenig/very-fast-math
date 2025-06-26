import gymnasium
import highway_env
from matplotlib import pyplot as plt
from ctypes import *
import math
from pathlib import Path
import shutil

# Best so far:
# ACCEL_RANGE = 5
ACCEL_RANGE = 5

morty_lib = CDLL('./lib/libvfm.so')
morty_lib.morty.argtypes = [c_char_p, c_char_p, c_size_t]
morty_lib.morty.restype = c_char_p
    
good_ones = []
nocex_count = 0

def dpoint_following_angle(dpoint_y, ego_y, heading, ddist):
    return heading - math.atan((dpoint_y - ego_y) / ddist)

open('./morty/results.txt', 'w').close()
for seedo in range(1, 101):
    env = gymnasium.make('highway-v0', render_mode='rgb_array', config={
        "action": {
            "type": "MultiAgentAction",
            "action_config": {
                "type": "ContinuousAction",
                "acceleration_range": [-ACCEL_RANGE, ACCEL_RANGE],
                "steering_range": [-1, 1],
                "speed_range": [0, 70],
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

    env.reset(seed=seedo * 19) # Use some factor due to (unproven) suspicion that close-by seeds lead to similar starting positions.

    action = ([0, 0], [0, 0], [0, 0], [0, 0], [0, 0])
    dpoints_y = [0, 0, 0, 0, 0, 0]     # The lateral position of the points the cars head towards.
    dpoints_delta = [0, 0, 0, 0, 0, 0] # The direction we're currently moving towards, laterally.
    lc_time = [0, 0, 0, 0, 0, 0]       # Time the current lc is taking, abort if too long. Needed to better approximate MC behavior.
    egos_x = [0, 0, 0, 0, 0, 0]        # Long pos of cars in m.
    egos_y = [0, 0, 0, 0, 0, 0]        # Lat pos of cars in m.
    egos_v = [0, 0, 0, 0, 0, 0]        # Long vel of cars in m/s.
    egos_headings = [0, 0, 0, 0, 0, 0] # Angle rel. to long axis in rad.
    nocex_count = 0

    first = True
    for global_counter in range(100):
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
        
        # Best so far: 
        # input += "$$$1.35$$$false$$$0.5" (64 successful)
        # input += "$$$1.35$$$false$$$0.55" (57 successful)
        # input += "$$$1.3$$$false$$$0.45" (56 successful)
        # input += "$$$1.35$$$false$$$0.525" (58 successful)
        # --- Above this line, we need to possibly add 1 to the successful runs (I assume), depending on the last exp being successful or not. This is due to the premature file-write of the results.
        # input += "$$$1.35$$$false$$$0.5125" (61 successful)
        # input += "$$$1.35$$$false$$$0.4875" (61 successful)
        # input += "$$$1.35$$$false$$$0.50625" (65 successful)
        # --- Above this line we had actually only 99 runs since I got Python's "range" function wrong.
        # input += "$$$1.34$$$false$$$0.50625" (55 successful)
        input += "$$$1.36$$$false$$$0.50625"
        
        if egos_x[4] < egos_x[3] and egos_x[3] < egos_x[2] and egos_x[2] < egos_x[1] and egos_x[1] < egos_x[0]:
            print("DONE") # Completion condition for position reversal SPEC.
            good_ones.append(seedo)
            break
        
        first = False
        
        #### MODEL CHECKER CALL ####
        result = create_string_buffer(10000)
        res = morty_lib.morty(input.encode('utf-8'), result, sizeof(result))
        res_str = res.decode()
        
        if res_str == "|;|;|;|;|;":
            print("No CEX found")
            if nocex_count > 10: # Allow up to this many times being blind per run. (If in doubt: 10)
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

        with open("lanes.txt", "w") as text_file:
            text_file.write(lanes)
        with open("accels.txt", "w") as text_file:
            text_file.write(accels)

        print(f"summed velocity: {sum_vel_by_car}")
        print(f"summed lane: {sum_lan_by_car}")
        
        action_list = []
        
        # Best so far:
        # MAXTIME_FOR_LC = 5
        MAXTIME_FOR_LC = 5
        
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

    with open("./morty/results.txt", "a") as f:
        f.write("(" + str(100 * len(good_ones) / seedo) + "%) " + ' '.join(str(x) for x in good_ones) + " [" + str(nocex_count) + " blind]\n")

print(good_ones)