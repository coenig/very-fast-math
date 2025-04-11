import gymnasium
import highway_env
from matplotlib import pyplot as plt
from ctypes import *

HACKING_CONSTANT = 4

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
        "see_behind": True,
        "clip": False
    },
    "simulation_frequency": 15,  # [Hz]
    "policy_frequency": HACKING_CONSTANT,  # [Hz]
    "controlled_vehicles": 5,
    "vehicles_count": 0,
    "screen_width": 1500,
    "screen_height": 500,
    "scaling": 4.5,
    "show_trajectories": True,
})

env.reset(seed=0)

morty_lib = CDLL('./lib/libvfm.so')
morty_lib.morty.argtypes = [c_char_p, c_char_p, c_size_t]
morty_lib.morty.restype = c_char_p
    
LANE_LEFT = 0
IDLE = 1
LANE_RIGHT = 2
FASTER = 3
SLOWER = 4

action_idle = (IDLE, IDLE, IDLE, IDLE, IDLE)
action_lane = (IDLE, IDLE, IDLE, IDLE, IDLE)
action_vel = (IDLE, IDLE, IDLE, IDLE, IDLE)

for global_counter in range(1000):
    env.render()
    
    if global_counter % 2 == 0:
        obs, reward, done, truncated, info = env.step(action_lane)
    elif global_counter % 2 == 1:
        obs, reward, done, truncated, info = env.step(action_vel)
    else:
        continue

    input = ""
    for el in obs:
        for val in el:
            input += str(val) + ","
        input += ";"
    
    result = create_string_buffer(1000)
    print(input)
    res = morty_lib.morty(input.encode('utf-8'), result, sizeof(result))
    
    res_str = res.decode()    
    print(res_str)

    LOOKOUT_INTO_FUTURE = 2
    sum_vel_by_car = []
    sum_lan_by_car = []

    for i1, el1 in enumerate(res_str.split(';')):
        sum_lan_by_car.append(0)
        sum_vel_by_car.append(0)
        for i2, el2 in enumerate(el1.split('|')):
            for i3, el3 in enumerate(el2.split(',')):
                if el3 and i3 < LOOKOUT_INTO_FUTURE:
                    if i2 == 1:
                        sum_lan_by_car[i1] += float(el3)
                    else:
                        sum_vel_by_car[i1] += float(el3)
                        
    print(sum_vel_by_car)
    print(sum_lan_by_car)
    
    action_list_vel = []
    action_list_lane = []
        
    for i, el in enumerate(sum_vel_by_car):
        if sum_lan_by_car[i] < 0:
            action_list_lane.append(LANE_RIGHT)
        elif sum_lan_by_car[i] > 0:
            action_list_lane.append(LANE_LEFT)
        else:
            action_list_lane.append(IDLE)
            
        if sum_vel_by_car[i] > 0:
            action_list_vel.append(FASTER)
        elif sum_vel_by_car[i] < 0:
            action_list_vel.append(SLOWER)
        else:
            action_list_vel.append(IDLE)
    
    print(action_list_vel)
    print(action_list_lane)
    
    action_vel = tuple(action_list_vel)
    action_lane = tuple(action_list_lane)

#plt.imshow(env.render())
#plt.show()
env