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


HALF_WIDTH = 50
HALF_HEIGHT = 25
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
        # x: [0.04, 0.96]
        self.action_space = spaces.Box(low=np.array([BOUND_X_MIN]), 
                                       high=np.array([BOUND_X_MAX]))
        # self.action_space = spaces.MultiDiscrete([1000-HALF_WIDTH*2, 25])
        
        # Observation Space, need the boundary information
        self.observation_space = spaces.Box(low=np.zeros((1000, 1000)), 
                                            high=np.add(0.1, np.ones((1000, 1000))))

        # Set-up bullet physics server, sample boundry
        target = self._sample_target_pos() # a list of pos
        self.assembly_env = Assembly(target)

        self.base_distance = (max(target[0][0]-0, 1-target[0][0]), 
                              target[0][-1], 
                              self.assembly_env.distance_list[0])
        self.dist_hist = [self.base_distance] # a list of tuples, previous distance

    def _sample_target_pos(self):
        # implement sampling
        # TODO: if sampling update base_distance
        return [[min(np.random.random(),0.999), 0, np.random.random()/2]]
        # return [[0.498, 0, 0.38]]
    
    def _get_observation(self):
        # return the occupancy grid as a boolean matrix
        out = self.assembly_env.get_image()
        noise = np.random.uniform(low=0, high=0.1, size=out.shape).astype(np.float32)
        # noise = np.zeros(out.shape)
        return np.add(out, noise)
    
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
    
    def _check_termination(self, info_output):
        # return True if a infeasible or reach target
        if not self.assembly_env.check_feasibility(info_output):
            return True, 1
        if self.assembly_env.check_target():
            return True, 0
        return False, 0
        
    def _compute_dist_improve(self, dist_x, dist_z, dist_direct):
 
        if dist_direct >= self.dist_hist[-1][-1]:
            return 0
        else:
            pre = self.dist_hist[-1]
            delta_z = pre[1]-dist_z if pre[1]!=dist_z else HALF_HEIGHT*2
            delta_x = pre[0]-dist_x if pre[0]!=dist_x else HALF_WIDTH*2
            delta_direct = pre[2] - dist_direct 
            # res = 10*delta_x + 0.1/delta_z
            res = 10 / (dist_direct + 0.001)

            self.dist_hist.append((dist_x, dist_z, dist_direct))
            if res < -10:
                breakpoint()
            return res
    
    def _sample_to_posxy(self, sample):
        return round(sample, 3)
    
    # def _sample_to_posz(self, sample):
    #     # center of the block at z-axis
    #     out = (round(sample*1000/HEIGHT)*2+1)*HALF_HEIGHT
    #     assert out % (HALF_HEIGHT*2) == HALF_HEIGHT, "Pos value at z-axis for PyBullet is incorrect"
    #     return out/1000
    
    def _to_pos(self, sample_x):
        # Real valued coordinate, unit meter
        return [self._sample_to_posxy(sample_x), 0, 0]

    def step(self, action): 

        #Interact with the PyBullet env
        
        action_x = (action[0]+2)/4
        #action_x = np.clip(action_x, BOUND_X_MIN, BOUND_X_MAX)

        pos = self._to_pos(action_x)
        
        output, updated_pos = self.assembly_env.interact(pos)
        pos = updated_pos
        global NUM_STEP
        NUM_STEP += 1
        NUM_STEP = NUM_STEP % 128 
        print(str(NUM_STEP) + ' Pos: '+str(updated_pos))

        #Calculate the reward
        param_material = -1
        param_term = -1 # >=25
        if updated_pos != None:

            dist_x, dist_z, dist_direct = self.assembly_env.get_distance(updated_pos)
            reward = param_material + self._compute_dist_improve(
                dist_x, dist_z, dist_direct)

            #Check termination
            termination, reward_term = self._check_termination(output)
        else:
            reward = -1
            termination = True
            reward_term = 1
 
        # if termination and (np.random.random() <= self.pic_freq):
        if termination:
            reward += param_term * reward_term
            if self.assembly_env.check_target():
                reward += 1000
            print("Termination: {}" + str(output))
            #take a picture at the termination
            # img_arr = self._take_rgb_arr()
            # filename = time.ctime()
            # filename = filename.replace(':', '-')
            # plt.imshow(img_arr)
            # plt.savefig(IMG_PATH+filename+'.png')
            # plt.close()

        return self._get_observation(), reward, termination, self._get_info(updated_pos)
    
    def reset(self):
        #print("-----------reset simulation---------------")
        self.assembly_env.close()
        target = self._sample_target_pos()
        self.assembly_env = Assembly(target, render=self.render)
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