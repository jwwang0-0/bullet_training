import os
import time
from assembly_gymenv.envs.assembly_env import Assembly
from assembly_gymenv.envs.compas_bullet import CompasClient
HERE = os.path.dirname(__file__)
DATA = os.path.join(HERE, "..", "DATA")

a = Assembly([[0.498, 0, 0.9]], render = True)
shift = 0.04
N=10

pos = [0.576,0,0.025]
a.interact(pos)

pos = [0.6,0,0.7]
a.interact(pos)

pos = [0.7,0,0.4]
a.interact(pos)
# pos = [0.4,0.0,0.040]
# a._action(pos)
# print(a._get_env_output(pos))

# for i in range(N):
#     pos = [shift,0,i*0.04+0.02]
#     if i != N -1 :
#         shift = shift + 1/2 * 1/(N-i-1)/1000*80
#     a._action(pos)
#     print(a._get_env_output(pos))
time.sleep(5)

# Gofa

# print("Check",os.path.isfile(os.path.join(DATA, "abb_crb15000_dual_support/urdf/crb15000_5_95_macro.xacro")))
# gofaId = a.p.loadURDF(os.path.join(DATA, "abb_crb15000_dual_support/urdf/","abb_crb15000_dual_cell.urdf"), [0.5, 0.6, 0])
# a.p.resetBasePositionAndOrientation(gofaId, [0, 0, 0], [0, 0, 0, 1])


# Compas Client
# compas_a = CompasClient(a._physics_client_id)
# compas_a.load_ur5(load_geometry=True)

a.realtime()
time.sleep(50)

#compas_a.__exit__()
# pos = [0,0,0.02]
# a._action(pos)
# print(a._get_env_output(pos))
# pos = [0.04,0,0.06]
# a._action(pos)
# print(a._get_env_output(pos))
# time.sleep(60)
