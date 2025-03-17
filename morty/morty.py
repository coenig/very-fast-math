import gymnasium
import highway_env
from matplotlib import pyplot as plt
from ctypes import *

def line_prepender(filename, line):
    with open(filename, 'r+') as f:
        content = f.read()
        f.seek(0, 0)
        f.write(line.rstrip('\r\n') + '\n' + content)
        
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
    "see_behind": True,
    "order": "shuffled"
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

env.reset(seed=0) # 3 6 7

morty_lib = CDLL('./lib/libvfm.so')
morty_lib.morty.argtypes = [c_char_p, c_char_p, c_size_t]
morty_lib.morty.restype = c_char_p
action = (2, 2, 2, 2, 2)
open('lanes.txt', 'w').close()
open('velocities.txt', 'w').close()

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
    
    lanes_store = ""
    velocities_store = ""
    for res_str in res_strs:
        if res_str.replace('|', "").replace(',', "").replace(';', ""):
            both_str = res_str.split('|')
            delta_velocities_str = both_str[0]
            delta_lanes_str = both_str[1]
            
            delta_vels = delta_velocities_str.split(',')
            delta_lanes = delta_lanes_str.split(',')
            
            delta_vel = float(delta_vels[0])
            delta_lane = float(delta_lanes[0])
            delta_vel_acc = 0
            delta_lane_acc = 0
            
            lanes_store += str(delta_lanes) + "\n"
            velocities_store += str(delta_vels) + "\n"
          
            for i, el in enumerate(delta_lanes):
                if i <= 4:
                    if len(delta_lanes) > i + 1: # Last index is empty
                        delta_lane_acc += float(delta_lanes[i])

            for i, el in enumerate(delta_vels):
                if i <= 2:
                    if len(delta_vels) > i + 1: # Last index is empty
                        delta_vel_acc += float(delta_vels[i])

            if False:
                pass
            elif delta_vel_acc <= -6:
               action_vec.append(4) # SLOWER
            elif delta_vel_acc >= 6:
               action_vec.append(3) # FASTER
            elif delta_lane_acc < 0:
               action_vec.append(2) # LANE_LEFT
            elif delta_lane_acc > 0:
               action_vec.append(0) # LANE_RIGHT
            elif delta_vel > 0:
               action_vec.append(3) # FASTER
            elif delta_vel < 0:
               action_vec.append(4) # SLOWER
            else:
               action_vec.append(1) # IDLE

    line_prepender('lanes.txt', lanes_store)
    line_prepender('lanes.txt', "")
    line_prepender('velocities.txt', velocities_store)
    line_prepender('velocities.txt', "")
    
    if not action_vec:
        print("WARNING: No CEX received.")
        action_vec.append(1)
        action_vec.append(1)
        action_vec.append(1)
        action_vec.append(1)
        action_vec.append(1)
        
    print(action_vec)
    action = tuple(action_vec)
