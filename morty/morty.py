import gymnasium
import highway_env
from matplotlib import pyplot as plt
from ctypes import *


env = gymnasium.make('highway-v0', render_mode='rgb_array', config={
    "action": {
        "type": "MultiAgentAction",
        "action_config": {
            "type": "ContinuousAction",
            "acceleration_range": [-5, 5],
            "steering_range": [-1, 1],
            "speed_range": [0, 30],
            "longitudinal": True,
            "lateral": True,
        },
    },
#    "observation": {
#        "type": "Kinematics",
#        "absolute": True,
#        "normalize": False,
#        "see_behind": True,
#        "clip": False,
#        "order": "sorted"
#    },
    "observation": {
      "type": "MultiAgentObservation",
      "observation_config": {
        "type": "Kinematics",
        #        "absolute": True,
        "normalize": False,

    }
    },
    "simulation_frequency": 30,  # [Hz]
    "policy_frequency": 2,  # [Hz]
    "controlled_vehicles": 5,
    "vehicles_count": 0,
    "screen_width": 1500,
    "screen_height": 500,
    "scaling": 4.5,
    "show_trajectories": True,
})

env.reset(seed=42)

morty_lib = CDLL('./lib/libvfm.so')
morty_lib.morty.argtypes = [c_char_p, c_char_p, c_size_t]
morty_lib.morty.restype = c_char_p
    

action = ([0, 0], [0, 0], [0, 0], [0, 0], [0, 0])

for global_counter in range(1000):
    env.render()
    
    obs, reward, done, truncated, info = env.step(action)
    
    input = ""
    for el in obs:
        for val in el[0]:
            input += str(val) + ","
        input += ";"
    
    result = create_string_buffer(1000)
    #print(f"input: {input}")
    res = morty_lib.morty(input.encode('utf-8'), result, sizeof(result))
    
    res_str = res.decode()    
    #print(f"result: {res_str}")

    sum_vel_by_car = []
    sum_lan_by_car = []

    for i1, el1 in enumerate(res_str.split(';')):
        sum_lan_by_car.append(0)
        sum_vel_by_car.append(0)
        for i2, el2 in enumerate(el1.split('|')):
            for i3, el3 in enumerate(el2.split(',')):
                if el3:
                    if i2 == 1:
                        if i3 == 1:
                            sum_lan_by_car[i1] += float(el3)
                    else:
                        if i3 == 0:
                            sum_vel_by_car[i1] += float(el3)
                        
    print(f"summed velocity: {sum_vel_by_car}")
    print(f"summed lane: {sum_lan_by_car}")
    
    action_list = []
        
    for i, el in enumerate(sum_vel_by_car):
        # if sum_lan_by_car[i] < 0:
        #     action_list_lane.append(0)
        # elif sum_lan_by_car[i] > 0:
        #     action_list_lane.append(0)
        # else:
        #     action_list_lane.append(0)

        action_list.append([sum_vel_by_car[i], 0])            
    
    #print(action_list_vel)
    #print(action_list_lane)
    
    action = tuple(action_list)