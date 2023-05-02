import os
import pybullet as p2
import pybullet_data
from pybullet_utils import bullet_client as bc


class AssemblySpace():

    def __init__(self, *arg) -> None:
        # prepare the scene or the physical environment

        self.block_list = []
        self.sceneobj_list = []
        self._physics_client_id = -1
        self._renders = True

        # create a physical client
        if self._physics_client_id < 0:
            if self._renders:
                self._p = bc.BulletClient(connection_mode=p2.GUI)
            else:
                self._p = bc.BulletClient()
            self._physics_client_id = self._p._client
            p = self._p
            p.resetSimulation()

            # Import Ground URDF
            p.loadURDF(os.path.join(self._urdfRoot, "plane.urdf"), [0, 0, -1])
            
            # Set Gravity Simulation
            p.setGravity(0, 0, -9.8)
            self.timeStep = 0.02
            p.setTimeStep(self.timeStep)
            p.setRealTimeSimulation(0) #问题？？？ 这个应该选什么
        else:
            self.clear()
        # create a scene
        self.boundry_condition()

        ## Set-up Renders
        # self._renders = renders
        # self._render_height = 200
        # self._render_width = 320
        # self._physics_client_id = -1
        # self.seed()
        # #    self.reset()
        # self.viewer = None
        # self._configure()

    def boundry_condition():
        #define the target, obstacle
        pass


    def close(self):
        if self._physics_client_id >= 0:
            self._p.disconnect()
        self._physics_client_id = -1

    def renew(self):
        # make a new client
        pass

    def restore(self):
        pass

    def clear(self):
        #clear all blocks in the scene
        p = self._p
        for id in self.block_list:
            p.removeBody(id)

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
        """
        output: a dictionary of checks
        """
        self._action()
        return self._get_env_output()