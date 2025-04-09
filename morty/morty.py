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
        "normalize": False 
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

env.reset(seed=21)

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
            input += str(val) + ","
        input += ";"
    
    result = create_string_buffer(1000)
    print(input)
    res = morty_lib.morty(input.encode('utf-8'), result, sizeof(result))
    
    res_str = res.decode()    
    print(res_str)

    for i1, el1 in enumerate(res_str.split(';')):
        for i2, el2 in enumerate(el1.split('|')):
            vel = []
            lan = []
            for i3, el3 in enumerate(el2.split(',')):
                if el3:
                    if i2 == 1:
                        lan.append(el3)
                    else:
                        vel.append(el3)
        print(vel)
        print(lan)
    
    action = tuple(res_str.split(';'))

#plt.imshow(env.render())
#plt.show()
env