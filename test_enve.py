import gym
import numpy as np
from gym import spaces
import graph

from assembly_env import Assembly


class MySim(gym.Env):

    #For rendering
    metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

    def __init__(self, renders=False):
        
        # Set-up bullet physics server (Need to be rechecked later)
        # TODO: any parameters needed by init Assembly?
        self.assembly_env = Assembly(render=renders)

        # Action Space 
        self.action_space = spaces.Dict({"x_pos":spaces.Discrete(1000),
                                         "z_pos":spaces.Discrete(25)})        
        
        # Observation Space, need the boundary information
        self.observation_space = spaces.MultiBinary([1000,25])

        # Keep a history of the assembled blocks, maybe not needed here
        self.block_list = []

    def _get_observation(self):
        # return some observation data
        # TODO: need to define
        return None
    
    def _get_info(self):
        # return some auxiliary data
        # TODO: need to define
        return None
    
    def _check_termination(dict_check):
        # return True if a termination criteria is met
        # criteria: collision or instable
        ls_check = [dict_check.get('collision'), dict_check.get('instability')]
        return (sum(ls_check) > 0)

    def step(self, action): 

        #Interact with the PyBullet env 
        arg_action = [action.get('x_pos', 0), action.get('y_pos', 0), action.get('z_pos', 0)]
        output = self.assembly_env.interact(arg_action)

        #Calculate the reward
        param_material = -1
        param_distance = 1
        reward = param_material + param_distance * output.get('distance')

        #Check termination
        termination = self._check_termination(output)

        return self._get_observation(), reward, termination, self._get_info()
    
    def reset(self):
        self.assembly_env.reset()       
        #print("-----------reset simulation---------------")
        return self._get_observation(), self._get_info()
    
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
    
    # Close the Simulation 
    def close(self):
        pass

if __name__ == "__main__":
    from stable_baselines.common.env_checker import check_env 
    # 如果你安装了pytorch，则使用上面的，如果你安装了tensorflow，则使用from stable_baselines.common.env_checker import check_env
    # from stable_baselines3.common.env_checker import check_env
    env = MySim()
    check_env(env)