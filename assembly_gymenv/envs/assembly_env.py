import os
import math
import numpy as np
import pybullet as p2
import pybullet_data
from pybullet_utils import bullet_client as bc
# HERE = os.path.dirname(__file__)
DATA = "../DATA/"

t_step = 1/240
##################################################
#################Utility Function#################
##################################################

#Speed Magnitude Calculation
def speed_mag(data):
    mod_l = math.pow(data[0][0],2)+math.pow(data[0][1],2)+math.pow(data[0][2],2)
    mod_a = math.pow(data[1][0],2)+math.pow(data[1][1],2)+math.pow(data[1][2],2)
    return math.pow(mod_l,0.5), math.pow(mod_a,0.5)  

#Distance in Space
def distance(pos1, pos2):
    dsquare = 0
    for i,_ in enumerate(pos1):
        dsquare += math.pow(pos1[i]-pos2[i],2)
    return math.pow(dsquare,0.5)

#Block Element
class Block():

    def __init__(self, id, pos):
        self.id = id
        self.pos = pos
        self.quaternion = (0,0,0,1)
##################################################
#################### Assembly ####################
##################################################

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
            self.timeStep = t_step
            p.setTimeStep(self.timeStep)
            p.setRealTimeSimulation(0) #if it is "1" it will be locked
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

    def reset(self):
        #Remove all blocks in the environment

        p = self._p
        for id in self.block_list:
            p.removeBody(id)

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
        # perform add block action in the physical environment
        # not doing simulation 
        p = self._p
        id = p.loadURDF(os.path.join(DATA, "block.urdf"),
                            [pos[0], pos[1], pos[2]]) 
        #print(p.getDynamicsInfo(id,-1))
        block = Block(id=id,pos=pos)      
        self.block_list.append(block)


    def _get_env_output(self, pos):
        # calculate the information about the environmnet
        # then output the information/checks
        p = self._p
        info = {"collision": None, "robot": None, "instability":None}

        #################Step 0: Collision Check#################

        # Mathmatrical Implementation
        # 2d image implementation
        index = [round(pos[0]*1000) , 0 , round((pos[2]*1000-20)/40)]  

        #TODO: below gives error message in check_env
        # for x in range(index[0]-40,index[0]+40):
        #     if self.image[x][index[2]] == 1:
        #         info["collision"] = True
        #     self.image[x][index[2]] = 1
        # if info["collision"] == None:
        #     info["collision"] = False
        
        # Pybullet implementation
        # Need to be tested during simulation
        # p.getOverlappingObjects()

        ############## TODO Step 1: Robot Check####################


        ############## Step 2: Stabiltiy Check#####################
        # Pybullet simulation
        # If it is unstable, restore the environment

        id = self.block_list[-1].id
        pos = self.block_list[-1].pos
        orien = self.block_list[-1].quaternion
        # lspeed_0, aspeed_0 = speed_mag(p.getBaseVelocity(id))
        for i in range(240):
            p.stepSimulation()
        (pos2, orien2) = p.getBasePositionAndOrientation(id)
        d = distance(pos, pos2)
        o = distance(orien,orien2)
        if d > 0.05:
            info["instability"] = True
            self.restore() 
        elif o > 0.1:
            info["instability"] = True 
            self.restore()            
        else:
            info["instability"] = False
        #print(d,o)
        # lspeed, aspeed = speed_mag(p.getBaseVelocity(id))
        # a_l = (lspeed-lspeed_0) / t_step
        # a_a = (aspeed-aspeed_0) / t_step
        # Debug: Check the p,v,a
        # print("Linear Velocity",'{:.5f}'.format(lspeed))
        # print("Position",position)
        # print("Linear Acceleration",'{:.5f}'.format(a_l))
        # print("Angular Velocity",'{:.5f}'.format(aspeed))
        # print("Angular Acceleration",'{:.5f}'.format(a_a))
        # print("Orientation", orientation)


        ############## TODO Step 3: Target Check###################
        return info
        
    def realtime(self):
        p = self._p
        p.setRealTimeSimulation(1)


    def interact(self, ls_pos):
        # perform actions in the physical environment
        # then, output the checks and distance
        """
        output: a dictionary of checks
        """
        self._action(ls_pos)
        return self._get_env_output(ls_pos)
    