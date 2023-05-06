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
        self.image = np.zeros((1000, 25), dtype='int8')
        #TODO Implement a Graph in the future

        # create a physical client
        self.create_bullet_client()

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

    def create_bullet_client(self):
 
        if self._physics_client_id < 0:

            if self._renders:
                self.p = bc.BulletClient(connection_mode=p2.GUI)
            else:
                self.p = bc.BulletClient()

            self._physics_client_id = self.p._client
            self.p.resetSimulation()

            # Import Ground URDF
            self.p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), [0, 0, 0])
            
            # Set Gravity Simulation
            self.p.setGravity(0, 0, -9.8)
            self.timeStep = t_step
            self.p.setTimeStep(self.timeStep)
            self.p.setRealTimeSimulation(0) #if it is "1" it will be locked
        else:
            self.clear()


    def get_image(self):
        return self.image

    def boundry_condition(self):
        #define the target, obstacle
        pass

    def close(self):
        self.block_list.clear()
        self.sceneobj_list.clear()
        self.image = -1
        if self._physics_client_id >= 0:
            self.p.disconnect()
        self._physics_client_id = -1

    def reset(self):
        #Remove all blocks in the environment

        for id in self.block_list:
            self.p.removeBody(id)

    def restore(self):
        # restore the environment if the block fails
        self.clear()
        for block in self.block_list:
            assert isinstance(block,Block), f"block is not a Block instance"
            id = self.p.loadURDF(os.path.join(DATA, "block.urdf"),
                                [block.pos[0], block.pos[1], block.pos[2]])
            block.id = id
            

    def clear(self):
        #clear all blocks in the scene
        #but still remember the history and image 
        for block in self.block_list:
            assert isinstance(block,Block), f"block is not a Block instance"
            self.p.removeBody(block.id)

    def _action(self, pos):
        # perform add block action in the physical environment
        # not doing simulation 
        id = self.p.loadURDF(os.path.join(DATA, "block.urdf"),
                            [pos[0], pos[1], pos[2]]) 
        #print(self.p.getDynamicsInfo(id,-1))
        block = Block(id=id,pos=pos)      
        self.block_list.append(block)

    def _check_collision(self, pos):
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
        # self.p.getOverlappingObjects()
        return None
    
    def _check_robot(self):

        ############## TODO Step 1: Robot Check####################

        return False

    def _check_stability(self, param):
        ############## Step 2: Stabiltiy Check#####################
        # Pybullet simulation
        # If it is unstable, restore the environment

        id = self.block_list[-1].id
        pos = self.block_list[-1].pos
        orien = self.block_list[-1].quaternion
        # lspeed_0, aspeed_0 = speed_mag(self.p.getBaseVelocity(id))
        for i in range(240):
            self.p.stepSimulation()
        (pos2, orien2) = self.p.getBasePositionAndOrientation(id)
        d = distance(pos, pos2)
        o = distance(orien,orien2)
        if d > 0.05:
            param = True
            self.restore() 
        elif o > 0.1:
            param = True 
            self.restore()            
        else:
            param = False
        #print(d,o)
        # lspeed, aspeed = speed_mag(self.p.getBaseVelocity(id))
        # a_l = (lspeed-lspeed_0) / t_step
        # a_a = (aspeed-aspeed_0) / t_step
        # Debug: Check the p,v,a
        # print("Linear Velocity",'{:.5f}'.format(lspeed))
        # print("Position",position)
        # print("Linear Acceleration",'{:.5f}'.format(a_l))
        # print("Angular Velocity",'{:.5f}'.format(aspeed))
        # print("Angular Acceleration",'{:.5f}'.format(a_a))
        # print("Orientation", orientation)

        return param
    
    def _check_target(self):
        ############## TODO Step 3: Target Check###################
        return None

    def _get_env_output(self, ls_pos):
        # calculate and output the information about the environmnet
        info = {"collision": None, 
                "robot": None, 
                "instability":None
                }

        info.update({"collision": self._check_collision(ls_pos)})
        info.update({"robot": self._check_robot(...)})
        info.update({"instability": self._check_stability(info.get("instability"))})

        return info
    
    def _check_feasibility(self):
        # criteria: collision or instable
        # output True or False
        return None
    
    def _update_image(self):
        # if the new block is feasible, then update the image (i.e. state/observation)
        return None
        
    def realtime(self):
        self.p.setRealTimeSimulation(1)


    def interact(self, ls_pos):
        # perform actions in the physical environment
        # then, output the checks and distance
        """
        output: a dictionary of checks
        """
        self._action(ls_pos)
        output = self._get_env_output(ls_pos)
        if self._check_feasibility:
            self._update_image()
        return output
    