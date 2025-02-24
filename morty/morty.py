import gymnasium
import highway_env
from matplotlib import pyplot as plt
#%matplotlib inline
from subprocess import Popen, PIPE



env = gymnasium.make('highway-v0', render_mode='rgb_array', config={
    "action": {
        "type": "DiscreteMetaAction"
    },
    "observation": {
        "type": "Kinematics",
        "absolute": True,
        "normalize": False 
    }
})

env.reset()

action = env.unwrapped.action_type.actions_indexes["IDLE"]
for i in range(15):
    obs, reward, done, truncated, info = env.step(action)
    env.render()

    input = ""
    for el in obs:
        for val in el:
            input += str(val) + ","
        input += ";"
    
    process = Popen(["bin/vfm", input], stdout=PIPE)
    (output, err) = process.communicate()
    exit_code = process.wait()
    print(output)
    action = env.unwrapped.action_type.actions_indexes[output.decode()]

plt.imshow(env.render())
plt.show()
