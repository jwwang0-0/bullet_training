import os
import numpy as np
import pybullet as p2
import pybullet_data
from pybullet_utils import bullet_client as bc
HERE = os.path.dirname(__file__)
DATA = os.path.join(HERE, "DATA")

#Block Element
class Block():

    def __init__(self, id, pos):
        self.id = id
        self.pos = pos

#Assembly Element
class Assembly():

    def __init__(self, render = False) -> None:
        # prepare the scene or the physical environment

        self.block_list = []
        self.sceneobj_list = []
        self._physics_client_id = -1
        self._renders = render
        self.image = np.zeros((1000, 25))
        #TODO Implement a Graph in the future

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
            p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), [0, 0, 0])
            
            # Set Gravity Simulation
            p.setGravity(0, 0, -9.8)
            self.timeStep = 0.02
            p.setTimeStep(self.timeStep)
            p.setRealTimeSimulation(0) #问题？？？ 这个应该选什么
        else:
            self.clear()
        # TODO create a scene
        #self.boundry_condition()

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
        self.block_list.clear()
        self.sceneobj_list.clear()
        self.image = -1
        if self._physics_client_id >= 0:
            self._p.disconnect()
        self._physics_client_id = -1



    def renew(self):
        # make a new client
        pass

    def restore(self):
        # restore the environment if the block fails
        p = self._p
        self.clear()
        for block in self.block_list:
            assert isinstance(block,Block), f"block is not a Block instance"
            id = p.loadURDF(os.path.join(DATA, "block.urdf"),
                                [block.pos[0], block.pos[1], block.pos[2]])
            block.id = id
            

    def clear(self):
        #clear all blocks in the scene
        #but still remember the history and image 
        p = self._p
        for block in self.block_list:
            assert isinstance(block,Block), f"block is not a Block instance"
            p.removeBody(block.id)

    def _action(self, pos):
        # perform actions in the physical environment
        # possibly the robot actions as well later
        p = self._p
        id = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "block.urdf"),
                            [pos[0], pos[1], pos[2]]) 
        block = Block(id=id,pos=pos)      
        self.block_list.append(block)


    def _get_env_output(self,pos):
        # calculate the information about the environmnet
        # then output the information/checks
        p = self._p
        info = {"collision": None, "robot": None, "stabiltiy":None}

        ## Collision Check
        # Mathmatrical Implementation
        # 2d image implementation

        for x in range(pos[0]-40,pos[0]+40):
            if self.image[x][pos[2]] == 1:
                info["collision"] = True
            self.image[x][pos[2]] = 1
        
        # Pybullet implementation
        # Need to be tested during simulation
        # p.getOverlappingObjects()

        ## TODO Robot Feasibility Check

        ## Stabiltiy Check
        p.stepSimulation()
        id = self.block_list[-1].id
        #position, speed = p.getJointState(id, 0)[0:2]
        speed = p.getBaseVelocity(id)
        # if speed > 0.0001:
        #     info["stabiltiy"] = False
        # else:
        #     info["stabiltiy"] = True
        ## TODO Reach the target or not
        print(speed)

    def interact(self, *args):
        # perform actions in the physical environment
        # then, output the required information
        """
        output: a dictionary of checks
        """
        self._action()
        return self._get_env_output()