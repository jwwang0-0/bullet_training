import gym
import numpy as np
from gym import spaces
import pybullet
import matplotlib.pyplot as plt
import time
import re
import os
import torch


from assembly_gymenv.envs.assembly_env import Assembly

# need unit as meter
HALF_WIDTH = 0.05
HALF_HEIGHT = 0.025
HEIGHT = HALF_HEIGHT*2

BOUND_X_MIN = 0.05
BOUND_X_MAX = 0.95
BOUND_Z_MIN = 0.025
BOUND_Z_MAX = 0.475

RENDER_HEIGHT = 200
RENDER_WIDTH = 320

HERE = os.path.dirname(__file__)
IMG_PATH = os.path.join(HERE, "../", "img/")

global NUM_STEP
NUM_STEP = 0


class AssemblyGymEnv(gym.Env):

    #For rendering
    metadata = {'render.modes': ['human', 'rgb_array'], "render_fps": 4}

    def __init__(self, renders=None, pic_freq=0.005):

        self.render = renders
        self.pic_freq = pic_freq
        
        # Action Space 
        self.action_space = spaces.Box(low=np.array([BOUND_X_MIN]), 
                                       high=np.array([BOUND_X_MAX]))
        
        # Observation Space, need the boundary information
        self.observation_space = spaces.Box(low=np.array([BOUND_X_MIN, BOUND_Z_MIN]), 
                                            high=np.array([BOUND_X_MAX, BOUND_Z_MAX]))

        self.set_env_ceil()

        # Set-up bullet physics server, sample boundry
        self.target = self._sample_target_pos() # a list of pos
        self.assembly_env = Assembly(self.target)

        self.base_distance = (max(self.target[0][0]-0, 1-self.target[0][0]), 
                              self.target[0][-1], 
                              self.assembly_env.distance_list[0])
        self.dist_hist = [self.base_distance] # a list of tuples, previous distance

        self.obs_hist = [[self.target[0][0], self.target[0][-1]]]

    def _sample_target_pos(self):
        return [[np.random.uniform(low=BOUND_X_MIN, high=BOUND_X_MAX), 
                 0, 
                 np.random.uniform(low=BOUND_Z_MIN, high=self.ceil)]]
    
    def _get_observation(self, updated_pose=None):
        # return the rurrent pose, else the target
        if updated_pose==None:
            self.obs_hist = [[self.target[0][0], self.target[0][-1]]]
        else:
            self.obs_hist.append([updated_pose[0], updated_pose[-1]])
        return np.asarray(self.obs_hist, dtype=np.float32)
    
    def _get_info(self, pos):
        # return some auxiliary data
        if pos != None :
            return {'max-height': round((pos[-1]+0.025)/0.05),
                    'pos': pos
                    }
        else:
            return {'max-height': 0,
                    'pos': [0,0,0]
                    }
    
    def _check_termination(self, info_output, updated_pose):

        if updated_pose is None:
            return True
        if not self.assembly_env.check_feasibility(info_output):
            return True
        if self.assembly_env.check_target():
            return True

        return False
        
    def _compute_dist_improve(self, dist_x, dist_z, dist_direct):
        # not used
        res = None
        self.obs_hist.append((dist_x, dist_z, dist_direct))
        return res

    def _sample_to_posxy(self, sample):
        return round(sample, 3)
        
    def _to_pos(self, sample_x):
        # Real valued coordinate, unit meter
        return [self._sample_to_posxy(sample_x), 0, 0]
    
    def set_env_ceil(self, ceil=0.05):
        self.ceil = ceil-0.01

    def step(self, action): 

        #Interact with the PyBullet env       
        action_x = action[0] + 0.5
        pos = self._to_pos(action_x)        
        output, updated_pos = self.assembly_env.interact(pos)

        #Calculate the reward
        term_volumn = np.power(0.5,len(self.obs_hist)) * self.target[0][-1]
        term_x = 1 / (1 + int( abs(pos[0]-self.target[0][0])/HALF_WIDTH ))
        if updated_pos == None:
            reward = - term_volumn * term_x

        else:
            # dist_x, dist_z, dist_direct = self.assembly_env.get_distance(updated_pos)
            term_z = (updated_pos[-1]) / HEIGHT / len(self.obs_hist)
            term_z = - term_z if ( updated_pos[-1]> self.target[0][-1]) else term_z
            reward = term_volumn * term_z * term_x
        # breakpoint()
            
        #Check termination
        termination = self._check_termination(output, updated_pos)
 
        # if termination and (np.random.random() <= self.pic_freq):
        if termination and self.assembly_env.check_target():
            reward += term_volumn

            #take a picture at the termination
            # img_arr = self._take_rgb_arr()
            # filename = time.ctime()
            # filename = filename.replace(':', '-')
            # plt.imshow(img_arr)
            # plt.savefig(IMG_PATH+filename+'.png')
            # plt.close()

        # print('Pos: {0: <15}'.format(str(updated_pos)) + ';\t Reward: ' + str(round(reward*20, 5)))
        # if termination: print("Termination: {0:>60}".format(str(output)))

        return self._get_observation(updated_pos), reward*20, termination, self._get_info(updated_pos)
    
    def reset(self):

        self.assembly_env.close()
        self.target = self._sample_target_pos()
        # print('{0:>10}'.format('Target pos: ') + str(np.round(self.target[0],3)))

        self.assembly_env = Assembly(self.target, render=self.render)
        self.dist_hist = [self.base_distance]
        return self._get_observation()
    
    def _take_rgb_arr(self):
        view_matrix = self.assembly_env.p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=(0,0,0),
            distance=2.3,
            yaw=1.6,
            pitch=-15.4,
            roll=0,
            upAxisIndex=2)
        
        proj_matrix = self.assembly_env.p.computeProjectionMatrixFOV(
            fov=60, aspect=float(RENDER_WIDTH)/RENDER_HEIGHT,
            nearVal=0.1, farVal=100.0)
        
        (_, _, px, _, _) = self.assembly_env.p.getCameraImage(
            width=RENDER_WIDTH, 
            height=RENDER_HEIGHT, 
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix, 
            renderer=pybullet.ER_BULLET_HARDWARE_OPENGL
            )
        rgb_array = np.array(px).reshape((RENDER_HEIGHT, RENDER_WIDTH, -1))
        return rgb_array[:,:,:3]

    def render(self, mode='human', close=False):
        if mode != "rgb_array":
            return np.array([])
        rgb_array = self._take_rgb_arr()
        return rgb_array 
        
    # Close the Simulation 
    def close(self):
        self.assembly_env.close()

if __name__ == "__main__":
    # 如果你安装了pytorch，则使用上面的，如果你安装了tensorflow，则使用from stable_baselines.common.env_checker import check_env
    from stable_baselines3.common.env_checker import check_env
    # from stable_baselines.common.env_checker import check_env 
    env = AssemblyGymEnv(renders=False, pic_freq=0.5)
    check_env(env)
    # _, reward, termination, info = env.step([-1])
    # print(reward,termination,info)
    # _, reward, termination, info = env.step([-1])
    # print(reward,termination,info)
    # _, reward, termination, info = env.step([-1])
    # print(reward,termination,info)
    # _, reward, termination, info = env.step([-1])
    # print(reward,termination,info)
    # _, reward, termination, info = env.step([-1])
    # print(reward,termination,info)
    # _, reward, termination, info = env.step([-1])
    # print(reward,termination,info)
    # _, reward, termination, info = env.step([-1])
    # print(reward,termination,info)
    # _, reward, termination, info = env.step([-1])
    # print(reward,termination,info)
    # _, reward, termination, info = env.step([-1])
    # print(reward,termination,info)
    # _, reward, termination, info = env.step([-1])
    # print(reward,termination,info)
    # _, reward, termination, info = env.step([-1])
    # print(reward,termination,info)