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


HALF_WIDTH = 40
HALF_HEIGHT = 20

RENDER_HEIGHT = 200
RENDER_WIDTH = 320

HERE = os.path.dirname(__file__)
IMG_PATH = os.path.join(HERE, "../", "img/")


class AssemblyGymEnv(gym.Env):

    #For rendering
    metadata = {'render.modes': ['human', 'rgb_array'], "render_fps": 4}

    def __init__(self, renders=None, pic_freq=0.005):

        self.render = renders
        self.pic_freq = pic_freq
        
        # Action Space 
        # x: [0.04, 0.96]
        self.action_space = spaces.Box(low=np.array([0.04, 0.0]), 
                                       high=np.array([0.96, 0.4]))
        # self.action_space = spaces.MultiDiscrete([1000-HALF_WIDTH*2, 25])
        
        # Observation Space, need the boundary information
        self.observation_space = spaces.Box(low=np.zeros((1000, 1000)), 
                                            high=np.add(0.1, np.ones((1000, 1000))))

        # Set-up bullet physics server, sample boundry
        target = [[0.498, 0, 0.38]] # a list of pos
        self.assembly_env = Assembly(target)

        base_distance = np.sqrt(max(target[0][0]-0, 1-target[0][0])**2+target[0][-1]**2)
        self.dist_hist = [base_distance] # a list of floats, previous distance

    def _sample_target_pos(self):
        # implement sampling
        # TODO: if sampling update base_distance
        # return [[np.random.random(), 0, np.random.random()]]
        return [[0.498, 0, 0.38]]
    
    def _get_observation(self):
        # return the occupancy grid as a boolean matrix
        out = self.assembly_env.get_image()
        noise = np.random.uniform(low=0, high=0.1, size=out.shape).astype(np.float32)
        # noise = np.zeros(out.shape)
        return np.add(out, noise)
    
    def _get_info(self):
        # return some auxiliary data
        # TODO: need to define
        return {'History of distance': self.dist_hist}
    
    def _check_termination(self, info_output):
        # return True if a infeasible or reach target
        if not self.assembly_env.check_feasibility(info_output):
            return True
        if self.assembly_env.check_target():
            return True
        return False
    
    def _compute_dist_improve(self, curr_dist, param, info_output, action_z):
        if not self.assembly_env.check_feasibility(info_output):
            return -param*action_z
        # elif len(self.dist_hist) == 0:
        #     return 0.02 * param
        else:
            return (self.dist_hist[-1] - curr_dist) * param
    
    def _sample_to_posxy(self, sample):
        return round(sample, 3)
    
    def _sample_to_posz(self, sample):
        # center of the block at z-axis
        out = (2* round(sample/0.04) + 1) * HALF_HEIGHT
        assert out % 40 == 20, "Pos value at z-axis for PyBullet is incorrect"
        return out/1000
    
    def _to_pos(self, sample_x, sample_z):
        # Real valued coordinate, unit meter
        return [self._sample_to_posxy(sample_x), 0, self._sample_to_posz(sample_z)]

    def step(self, action): 

        #Interact with the PyBullet env
        
        action_x = (action[0]+2)/4
        action_z = (action[1]+2)*0.1
        action_x = np.clip(action_x, 0.04, 0.96)
        action_z = np.clip(action_z, 0, 0.4)

        pos = self._to_pos(action_x, action_z)
        print('Pos: '+str(pos))
        output = self.assembly_env.interact(pos)

        #Calculate the reward
        param_material = -1
        param_distance = 25 # >=25

        dist = self.assembly_env.get_distance()
        reward = param_material + self._compute_dist_improve(
            dist, param_distance, output, action_z)

        #Check termination
        termination = self._check_termination(output)
 
        if termination and (np.random.random() <= self.pic_freq):
        # if termination:
            print("Termination: {}" + str(output))
            #take a picture at the termination
            # img_arr = self._take_rgb_arr()
            # filename = time.ctime()
            # filename = filename.replace(':', '-')
            # plt.imshow(img_arr)
            # plt.savefig(IMG_PATH+filename+'.png')
            # plt.close()

        return self._get_observation(), reward, termination, self._get_info()
    
    def reset(self):
        #print("-----------reset simulation---------------")
        self.assembly_env.close()
        target = self._sample_target_pos()
        self.assembly_env = Assembly(target, render=self.render)
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