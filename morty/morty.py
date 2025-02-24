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
        "absolute": False,
        "normalize": False 
    }
})

env.reset()

for i in range(15):
    process = Popen(["bin/vfm", str(i)], stdout=PIPE)
    (output, err) = process.communicate()
    exit_code = process.wait()
    print(output)
    action = env.unwrapped.action_type.actions_indexes[output.decode()]
        
    obs, reward, done, truncated, info = env.step(action)
    env.render()

plt.imshow(env.render())
plt.show()
