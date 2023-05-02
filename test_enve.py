import os
import gym
import numpy as np
from gym import spaces
import graph
import pybullet as p2
import pybullet_data
from pybullet_utils import bullet_client as bc

class MySim(gym.Env):

    #For rendering
    metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

    def __init__(self, renders=False):
        
        ## Set-up bullet physics server (Need to be rechecked later)
        self._renders = renders
        self._render_height = 200
        self._render_width = 320
        self._physics_client_id = -1
        self.seed()
        #    self.reset()
        self.viewer = None
        self._configure()

        ## Action Space 
        self.action_space = spaces.Dict({"x_pos":spaces.Discrete(1000),
                                         "z_pos":spaces.Discrete(25)})        
        
        ## Observation Space, need the boundary information
        self.observation_space = spaces.MultiBinary([1000,25])

        self.block_list = []


    def step(self, action): 

        p = self._p

        #Sample next action  

        #Interact with the PyBullet env 

        #Calculate the reward 

        info = {}

        #Termination condition
        if done == True:
            pass

        return state, reward, done, info
    
    def reset(self):
        #print("-----------reset simulation---------------")
        
        p.resetSimulation()
        
        p = self._p
        #Remove all blocks in the environment
        for id in self.block_list:
            p.removeBody(id)

        #Return the ground node as the state / observation_space
        self.state = np.zeros((1000, 25))

        return self.state
    
    def render(self, mode='human', close=False):
    # Copied from the example
        if mode == "human":
            self._renders = True
        if mode != "rgb_array":
            return np.array([])
        base_pos=[0,0,0]
        self._cam_dist = 2
        self._cam_pitch = 0.3
        self._cam_yaw = 0
        if (self._physics_client_id>=0):
            view_matrix = self._p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=base_pos,
                distance=self._cam_dist,
                yaw=self._cam_yaw,
                pitch=self._cam_pitch,
                roll=0,
                upAxisIndex=2)
            proj_matrix = self._p.computeProjectionMatrixFOV(fov=60,
                    aspect=float(self._render_width) /
                    self._render_height,
                    nearVal=0.1,
                    farVal=100.0)
            (_, _, px, _, _) = self._p.getCameraImage(
                width=self._render_width,
                height=self._render_height,
                renderer=self._p.ER_BULLET_HARDWARE_OPENGL,
                viewMatrix=view_matrix,
                projectionMatrix=proj_matrix)
        else:
            px = np.array([[[255,255,255,255]]*self._render_width]*self._render_height, dtype=np.uint8)
        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(np.array(px), (self._render_height, self._render_width, -1))
        rgb_array = rgb_array[:, :, :3]
        return rgb_array
    
    def seed(self, seed=None):
        pass

    # Close the Simulation 
    def close(self):
        pass

if __name__ == "__main__":
    from stable_baselines.common.env_checker import check_env 
    # 如果你安装了pytorch，则使用上面的，如果你安装了tensorflow，则使用from stable_baselines.common.env_checker import check_env
    # from stable_baselines3.common.env_checker import check_env
    env = MySim()
    check_env(env)