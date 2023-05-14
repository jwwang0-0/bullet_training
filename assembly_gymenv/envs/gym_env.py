import gym
import numpy as np
from gym import spaces
import pybullet
import matplotlib.pyplot as plt
import time
import re

from assembly_gymenv.envs.assembly_env import Assembly


HALF_WIDTH = 40
HALF_HEIGHT = 20

RENDER_HEIGHT = 200
RENDER_WIDTH = 320

IMG_PATH = '../img/'


class AssemblyGymEnv(gym.Env):

    #For rendering
    metadata = {'render.modes': ['human', 'rgb_array'], "render_fps": 4}

    def __init__(self, renders=None):

        self.render = renders
        self.dist_hist = [] # a list of floats, previous distance
        
        # Action Space 
        # x: [0.04, 0.96]
        # self.action_space = spaces.Dict({"x_pos":spaces.Discrete(1000-HALF_WIDTH*2),
        #                                  "z_pos":spaces.Discrete(25)})
        self.action_space = spaces.MultiDiscrete([1000-HALF_WIDTH*2, 25])
        
        # Observation Space, need the boundary information
        img_element = np.array([4]*1000*1000).reshape((1000,1000))
        self.observation_space = spaces.MultiDiscrete(img_element)

        # Set-up bullet physics server, sample boundry
        target = [[self._sample_to_posxy(498), 0, self._sample_to_posz(9)]] # a list of pos
        self.assembly_env = Assembly(target)

    def _sample_target_pos(self):
        # implement sampling
        # return [[np.random.random(), 0, np.random.random()]]
        return [[self._sample_to_posxy(498), 0, self._sample_to_posz(9)]]
    
    def _get_observation(self):
        # return the occupancy grid as a boolean matrix
        return self.assembly_env.get_image()
    
    def _get_info(self):
        # return some auxiliary data
        # TODO: need to define
        return {'info': 0}
    
    def _check_termination(self, info_output):
        # return True if a infeasible or reach target
        if not self.assembly_env._check_feasibility(info_output):
            return True
        if self.assembly_env._check_target():
            return True
        return False
    
    def _compute_dist_improve(self, curr_dist):
        return None
    
    def _sample_to_posxy(self, sample):
        return float(sample/1000)
    
    def _sample_to_posz(self, sample):
        out = (2* sample+1) * HALF_HEIGHT
        assert out % 40 == 20, "Pos value at z-axis for PyBullet is incorrect"
        return out/1000
    
    def _to_pos(self, sample_x, sample_z):
        # Real valued coordinate, unit meter
        return [self._sample_to_posxy(sample_x), 0, self._sample_to_posz(sample_z)]

    def step(self, action): 

        #Interact with the PyBullet env
        # pos = self._to_pos(action.get('x_pos')+HALF_WIDTH, action.get('z_pos'))
        pos = self._to_pos(action[0]+HALF_WIDTH, action[1])
        output = self.assembly_env.interact(pos)

        #Calculate the reward
        param_material = -1
        param_distance = 25 # >=25
        # TODO: calulate distance delta

        reward = param_material + param_distance * self._compute_dist_improve(dist)

        #Check termination
        termination = self._check_termination(output)
        if termination:
            #take a picture at the termination
            img_arr = self._take_rgb_arr()
            filename = time.ctime()
            filename = filename.replace(':', '-')
            plt.imshow(img_arr)
            plt.savefig(IMG_PATH+filename+'.png')
            plt.close()

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
    env = AssemblyGymEnv(renders=False)
    check_env(env)