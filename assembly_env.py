import pybullet as p2
import pybullet_data
from pybullet_utils import bullet_client as bc


class AssemblySpace():

    def __init__(self, *arg) -> None:
        # prepare the scene or the physical environment

        # if self._physics_client_id < 0:
        #     if self._renders:
        #         # self._p = bc.BulletClient(connection_mode=p2.GUI)
        #     else:
        #         # self._p = bc.BulletClient()
        #     self._physics_client_id = self._p._client
        #     p = self._p
        #     p.resetSimulation()

        pass

    def close(self):
        # close the client
        pass

    def renew(self):
        # make a new client
        pass

    def _action(self):
        # perform actions  in the physical environment
        # possibly the robot actions as well later
        pass

    def _get_env_output(self):
        # calculate the information about the environmnet
        # then output the information/checks
        pass

    def interact(self, *args):
        # perform actions in the physical environment
        # then, output the required information
        pass