import os
import time
from assembly_env import Assembly
HERE = os.path.dirname(__file__)
DATA = os.path.join(HERE, "DATA")


a = Assembly(render = True)
shift = 0.04
N=10
for i in range(N):
    pos = [shift,0,i*0.04+0.02]
    if i != N -1 :
        shift = shift + 1/2 * 1/(N-i-1)/1000*80
    a._action(pos)
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

