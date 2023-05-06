import os
import time
from assembly_env import Assembly
HERE = os.path.dirname(__file__)
DATA = os.path.join(HERE, "DATA")

## Test for constraint analysis
## Stability n
a = Assembly(render = True)
shift = 0.04
N=25
for i in range(N):
    pos = [shift,0,i*0.04+0.02]
    a._action(pos)

for i, block in enumerate(a.block_list):
    if i==0:
        pass
    elif i==1:
        pass
    else:
        print(i)
        a._p.createConstraint(a.block_list[1].id,
                              -1,block.id,-1,a._p.JOINT_FIXED,
                              [0,0,0],[0,0,(i-1)*0.04],[0,0,0])
print(a._get_env_output(pos))
time.sleep(10)
a.realtime()
time.sleep(50)
# pos = [0,0,0.02]
# a._action(pos)
# print(a._get_env_output(pos))
# pos = [0.04,0,0.06]
# a._action(pos)
# print(a._get_env_output(pos))
# time.sleep(60)

