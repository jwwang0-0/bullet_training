import gym
import numpy as np
from gym import spaces

from assembly_gymenv.envs.assembly_env import Assembly


HALF_WIDTH = 40
HALF_HEIGHT = 20


class AssemblyGymEnv(gym.Env):

    #For rendering
    metadata = {'render.modes': ['human', 'rgb_array'], "render_fps": 4}

    def __init__(self, renders=None):

        self.render = renders
        
        # Action Space 
        # self.action_space = spaces.Dict({"x_pos":spaces.Discrete(1000-HALF_WIDTH*2),
        #                                  "z_pos":spaces.Discrete(25)})
        self.action_space = spaces.MultiDiscrete([1000-HALF_WIDTH*2, 25])
        
        # Observation Space, need the boundary information
        img_element = np.array([4]*1000*25).reshape((1000,25))
        self.observation_space = spaces.MultiDiscrete(img_element)

        # Set-up bullet physics server, sample boundry
        target = [[self._sample_to_posxy(498), 0, self._sample_to_posz(24)]] # a list of pos
        self.assembly_env = Assembly(target, render=renders)

    def _sample_target_pos(self):
        # TODO: implement sampling
        return [[self._sample_to_posxy(498), 0, self._sample_to_posz(24)]]
    
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
    
    def _sample_to_posxy(self, sample):
        return float(sample/1000)
    
    def _sample_to_posz(self, sample):
        return (2* sample+1) * HALF_HEIGHT /1000
    
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
        reward = param_material + param_distance * output.get('distance', 0)

        #Check termination
        termination = self._check_termination(output)

        return self._get_observation(), reward, termination, self._get_info()
    
    def reset(self):
        #print("-----------reset simulation---------------")
        self.assembly_env.close()
        target = self._sample_target_pos()
        self.assembly_env = Assembly(target, render=self.render)
        return self._get_observation()
    
    def render(self, mode='human', close=False):
        # maybe take a picture at the termination
        pass
        
    # Close the Simulation 
    def close(self):
        self.assembly_env.close()

if __name__ == "__main__":
    # 如果你安装了pytorch，则使用上面的，如果你安装了tensorflow，则使用from stable_baselines.common.env_checker import check_env
    # from stable_baselines3.common.env_checker import check_env
    from stable_baselines.common.env_checker import check_env 
    env = AssemblyGymEnv()
    check_env(env)