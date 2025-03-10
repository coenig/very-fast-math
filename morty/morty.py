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

env.reset(seed=0)

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
    
    result = create_string_buffer(100)
    print(input)
    res = morty_lib.morty(input.encode('utf-8'), result, sizeof(result))
    
    res_str = res.decode()
    res_str = res_str.replace("LANE_LEFT", "0").replace("IDLE", "1").replace("LANE_RIGHT", "2").replace("FASTER", "3").replace("SLOWER", "4")
    
    print(res_str)
    # output.decode()
    action = tuple(res_str.split(';'))

#plt.imshow(env.render())
#plt.show()
env