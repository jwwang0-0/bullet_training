import os
import time
from assembly_env import Assembly
HERE = os.path.dirname(__file__)
DATA = os.path.join(HERE, "DATA")

a = Assembly(render = True)
pos = [1,1,1]
a._action(pos)
for i in range (30):
    a._get_env_output(pos)
time.sleep(60)