import os
import time
from assembly_env import Assembly
HERE = os.path.dirname(__file__)
DATA = os.path.join(HERE, "DATA")

a = Assembly(render = True)
pos = [0,0,0.02]
a._action(pos)
print(a._get_env_output(pos))
pos = [0.04,0,0.06]
a._action(pos)
print(a._get_env_output(pos))
time.sleep(60)
