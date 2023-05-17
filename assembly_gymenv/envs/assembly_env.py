import os
import math
import numpy as np
import pybullet as p2
import pybullet_data
from pybullet_utils import bullet_client as bc
from assembly_gymenv.envs.compas_bullet import CompasClient
HERE = os.path.dirname(__file__)
DATA = os.path.join(HERE, "..", "DATA")

t_step = 1/240
HALF_WIDTH = 50
HALF_HEIGHT = 25
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

    def __init__(self, target, render = False) -> None:
        # prepare the scene or the physical environment

        self.block_list = []
        self.sceneobj_list = []
        self._physics_client_id = -1
        self._renders = render

        # pixel values for image: 1:target; 0:empty space; 0.5:bricks 
        self.image = np.zeros((1000, 1000), dtype=np.float32)
        self.complete = False
        self.distance_list = []
        # TODO Implement a Graph in the future

        # create a physical client
        self.create_bullet_client()

        # TODO create a scene
        self.set_boundry(target)

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
                #self.p = bc.BulletClient(connection_mode=p2.GUI)
                self.p = CompasClient(connection_type='gui')
            else:
                #self.p = bc.BulletClient()
                self.p = CompasClient(connection_type='direct')

            self._physics_client_id = self.p.client_id
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

    def _check_index(self,index):
        #check if the input is out of image bound
        #TODO define the threashold in the future
        out_bound =  index[0] < 0 \
                or index[0] > 1000 \
                or index[1] != 0 \
                or index[2] < 0 \
                or index[2] > 1000
        out_bound = bool(out_bound) 
        return out_bound

    def set_boundry(self, target):
        """input: list of pos"""
        #define the target
        #Image convention: 0>Nothing -1>Obstacle 0.5>Structures 1>Target 
        self.target_list = []
        for pos in target:
            self.distance_list.append(2)
            target_index = [round(pos[0]*1000) , round(pos[1]*1000) , round(pos[2]*1000)]
            if self._check_index(target_index) == True:
                raise Exception("Boundry is not correctly defined")
            else: 
                self.target_list.append(target_index)
                self.image[target_index[0]][target_index[2]] = 1
        #TODO Update the pybullet environment to show the target

    def get_image(self):
        return self.image

######################################
# Clean Related Functions
######################################   

    def close(self):
        self.block_list.clear()
        self.sceneobj_list.clear()
        self.image = -1
        if self._physics_client_id >= 0:
            self.p.disconnect()
        self._physics_client_id = -1

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

######################################
# Interaction Related Functions
######################################   

    def _action(self, pos):
        # perform add block action in the physical environment
        # not doing simulation 
        id = self.p.loadURDF(os.path.join(DATA, "block.urdf"),
                            [pos[0], pos[1], pos[2]]) 
        #print(self.p.getBasePositionAndOrientation(id))
        #self.p.resetBasePositionAndOrientation(id,[pos[0], pos[1], pos[2]],[0,0,0,1])
        #print(self.p.getDynamicsInfo(id,-1))
        block = Block(id=id,pos=pos)      
        self.block_list.append(block)
        return pos
    
    def _action_Tetris(self, pos):
        # perform add block action in the x|y dimension, assume z dimension is auto-detection
        center_index = [round(pos[0]*1000) , 0 , 0]  
        max_z = -1
        for x in range(center_index[0]-HALF_WIDTH,center_index[0] + HALF_WIDTH):
            for z in range(999,-1,-1):            
                if self.image[x][z] == 0:
                    pass
                elif self.image[x][z] == -1:
                    pass # later for obstacles
                elif self.image[x][z] == 0.5:
                    max_z = max(z,max_z)
                    break
                elif self.image[x][z] == 1:
                    pass
        z_center_index = max_z + HALF_HEIGHT + 1
        pos = [pos[0], 0, z_center_index/1000]  
        id = self.p.loadURDF(os.path.join(DATA, "block.urdf"),
                            [pos[0], pos[1], pos[2]]) 
        block = Block(id=id,pos=pos)      
        self.block_list.append(block)
        return pos

    def _check_collision(self, pos):
        # # Mathmatrical Implementation
        # # 2d image implementation
        # center_index = [round(pos[0]*1000) , 0 , round(pos[2]*1000)]

        # #check if the block is in the image bound
        # if True == self._check_index([center_index[0] - HALF_WIDTH,0,round(pos[2]*1000)]):
        #     raise Exception("Block is out of bound")
        #     # return True
        # if True == self._check_index([center_index[0] + HALF_WIDTH - 1,0,round(pos[2]*1000)]):
        #     raise Exception("Block is out of bound")
        #     # return True

        # # Check based on the image if there is collision
        # for x in [center_index[0]-HALF_WIDTH,center_index[0] + HALF_WIDTH - 1]:
        #     for z in [center_index[2]-HALF_HEIGHT,center_index[2] + HALF_HEIGHT - 1]:
        #         if self.image[x][z] == 0.5:
        #             return True


        return False
        
        # Pybullet implementation
        # Need to be tested during simulation
        # self.p.getOverlappingObjects()

    
    
    def _check_robot(self):

        ############## TODO Step 1: Robot Check####################

        return False

    def _check_stability(self):

        # Pybullet simulation
        # If it is unstable, restore the environment

        id = self.block_list[-1].id
        pos = self.block_list[-1].pos
        orien = self.block_list[-1].quaternion
        # lspeed_0, aspeed_0 = speed_mag(self.p.getBaseVelocity(id))
        for i in range(240):
            self.p.step_simulation()
        (pos2, orien2) = self.p.getBasePositionAndOrientation(id)
        dist = distance(pos, pos2)
        o = distance(orien,orien2)
        if dist > 0.05:
            instable = True
            self.restore() 
        elif o > 0.1:
            instable = True 
            self.restore()            
        else:
            instable = False
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

        return instable

    def _get_env_output(self, pos):
        # calculate and output the information about the environmnet
        info = {"collision": None, 
                "robot": None, 
                "instability":None
                }

        info.update({"collision": self._check_collision(pos)})
        info.update({"robot": self._check_robot()})
        info.update({"instability": self._check_stability()})

        return info
    
    def _update_image_and_score(self,pos):
        # if the new block is feasible, then update the image (i.e. state/observation)
        center_index = [round(pos[0]*1000) , 0 , round(pos[2]*1000)]  
        for x in range(center_index[0]-HALF_WIDTH,center_index[0] + HALF_WIDTH):
            for z in range(center_index[2]-HALF_HEIGHT , center_index[2] + HALF_HEIGHT):            
                if self.image[x][z] == 0:
                    self.image[x][z] = 0.5
                elif self.image[x][z] == -1:
                    raise Exception("obstacle check error")
                elif self.image[x][z] == 0.5:
                    raise Exception("collision check error")
                elif self.image[x][z] == 1:
                    ## TODO In the future if there is multiple target here need to be updated
                    self.complete = True
        
        #update score
        for i, target in enumerate(self.target_list):
            dist = distance(target,center_index)/1000
            if dist < self.distance_list[i]:
                self.distance_list[i] = dist
                
        return None

######################################
# Public Functions for Env
###################################### 

    def get_distance(self):
        # return self.distance
        return self.distance_list[0]

    def check_feasibility(self, info):
        # criteria: collision or instable
        # output True or False
        # True : Feasible
        # False: infeasible
        stop = info.get("collision")\
            or info.get("robot") \
            or info.get("instability")
        stop = bool(stop)
        return not(stop)

    def check_target(self):
        #check if the robot has reach the target
        ## TODO In the future if there is multiple target here need to be updated
        if self.complete == True :
            return True            
        return False    

    def interact(self, pos):
        # perform actions in the physical environment
        # then, output the checks and distance
        """
        output: a dictionary of checks
        """
        pos = self._action_Tetris(pos)
        info = self._get_env_output(pos)
        if self.check_feasibility(info):
            self._update_image_and_score(pos)
            self.check_target()
        return (info, pos)
    
    def realtime(self):
        self.p.setRealTimeSimulation(1)    