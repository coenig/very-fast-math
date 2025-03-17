import gymnasium
import highway_env
from matplotlib import pyplot as plt
from ctypes import *

env = gymnasium.make('highway-v0', render_mode='rgb_array', config={
    "action": {
        "type": "MultiAgentAction",
        "action_config": {
            "type": "DiscreteMetaAction",
            "target_speeds": [0, 5, 10, 15, 20, 25, 30],
        },
    "longitudinal": True,
    "lateral": True,
},
    "observation": {
    "type": "Kinematics",
    "absolute": True,
    "normalize": False,
    "clip": False,
    "see_behind": True
    },
    "simulation_frequency": 15,  # [Hz]
    "policy_frequency": 1,  # [Hz]
    "controlled_vehicles": 5,
    "vehicles_count": 0,
    "screen_width": 1500,
    "screen_height": 800,
    "scaling": 4.5,
    "show_trajectories": True,
})

env.reset(seed=6) # 16, 21, 23

morty_lib = CDLL('./lib/libvfm.so')
morty_lib.morty.argtypes = [c_char_p, c_char_p, c_size_t]
morty_lib.morty.restype = c_char_p
action = (2, 2, 2, 2, 2)
for i in range(150):
    obs, reward, done, truncated, info = env.step(action)
    env.render()

    input = ""
    for el in obs:
        for val in el:
            input += str(round(val, 2)) + ","
        input += ";"
    
    result = create_string_buffer(5000)
    print(input)
    res = morty_lib.morty(input.encode('utf-8'), result, sizeof(result))
    
    res_strs = (res.decode()).split(';')
    
    # res_str = res_str.replace("LANE_LEFT", "0").replace("IDLE", "1").replace("LANE_RIGHT", "2").replace("FASTER", "3").replace("SLOWER", "4")
    
    action_vec = []
    
    for res_str in res_strs:
        if res_str.replace('|', "").replace(',', "").replace(';', ""):
            both_str = res_str.split('|')
            delta_velocities_str = both_str[0]
            delta_lanes_str = both_str[1]
            
            delta_vels = delta_velocities_str.split(',')
            delta_lanes = delta_lanes_str.split(',')
            delta_vel = float(delta_vels[0])
            delta_lane = float(delta_lanes[0])
            print(delta_lanes)
            if len(delta_lanes) > 2: # Last index is empty
                delta_lane += float(delta_lanes[1])

            if len(delta_lanes) > 3: # Last index is empty
                delta_lane += float(delta_lanes[2])
        
            if len(delta_lanes) > 4: # Last index is empty
                delta_lane += float(delta_lanes[3])

            # if len(delta_lanes) > 5: # Last index is empty
            #     delta_lane += float(delta_lanes[4])

            # if len(delta_lanes) > 6: # Last index is empty
            #     delta_lane += float(delta_lanes[5])
        
            if len(delta_vels) > 2: # Last index is empty
               delta_vel += float(delta_vels[1])

            if len(delta_vels) > 3: # Last index is empty
               delta_vel += float(delta_vels[2])

            if len(delta_vels) > 4: # Last index is empty
               delta_vel += float(delta_vels[3])

            if len(delta_vels) > 5: # Last index is empty
               delta_vel += float(delta_vels[4])

            if delta_lane < 0:
               action_vec.append(0) # LANE_LEFT
            elif delta_lane > 0:
               action_vec.append(2) # LANE_RIGHT
            elif delta_vel > 0:
               action_vec.append(3) # FASTER
            elif delta_vel < 0:
               action_vec.append(4) # SLOWER
            else:
               action_vec.append(1) # IDLE
    
    if not action_vec:
        print("WARNING: No CEX received.")
        action_vec.append(1)
        action_vec.append(1)
        action_vec.append(1)
        action_vec.append(1)
        action_vec.append(1)
        
    print(action_vec)
    action = tuple(action_vec)
