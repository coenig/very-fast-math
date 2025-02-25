import gymnasium
import highway_env
from matplotlib import pyplot as plt
from ctypes import *

env = gymnasium.make('highway-v0', render_mode='rgb_array', config={
    "action": {
        "type": "DiscreteMetaAction"
    },
    "observation": {
        "type": "Kinematics",
        "absolute": True,
        "normalize": False 
    },
    "controlled_vehicles": 1,
    "vehicles_count": 4,
})

env.reset()

morty_lib = CDLL('./lib/libvfm.so')
morty_lib.morty.argtypes = [c_char_p]
morty_lib.morty.restype = c_char_p

action = env.unwrapped.action_type.actions_indexes["IDLE"]
for i in range(15):
    obs, reward, done, truncated, info = env.step(action)
    env.render()

    input = ""
    for el in obs:
        for val in el:
            input += str(val) + ","
        input += ";"
    
    res = morty_lib.morty(input.encode('utf-8'))
    
    print(res)
    #action = env.unwrapped.action_type.actions_indexes[output.decode()]

plt.imshow(env.render())
plt.show()
