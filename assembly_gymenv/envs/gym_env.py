import gym
import numpy as np
from gym import spaces

from assembly_env import Assembly


HALF_WIDTH = 80/2
HALF_HEIGHT = 20


class AssemblyGymEnv(gym.Env):

    #For rendering
    metadata = {'render.modes': ['human', 'rgb_array'], "render_fps": 4}

    def __init__(self, renders=None):
        
        # Action Space 
        self.action_space = spaces.Dict({"x_pos":spaces.Discrete(1000-HALF_WIDTH, start=HALF_WIDTH),
                                         "z_pos":spaces.Discrete(25)})        
        
        # Observation Space, need the boundary information
        self.observation_space = spaces.Dict({'image': spaces.MultiBinary([1000, 25])
                                              ,'boundry': spaces.MultiDiscrete([1000, 25, 1000, 25])
                                              })

        # Set-up bullet physics server (Need to be rechecked later)
        boundry = ...
        self.assembly_env = Assembly(self._boundry_to_pos(boundry), 
                                     render=renders)

    def _get_observation(self):
        # return the occupancy grid as a boolean matrix
        return {"image": self.assembly_env.get_image(),
                "boundry": self.assembly_env.get_boundry()
                }
    
    def _get_info(self):
        # return some auxiliary data
        # TODO: need to define
        return {}
    
    def _check_termination(self):
        # infeasible or reach target
        # return True if a termination criteria is met
        return False
    
    def _sample_to_posxy(self, sample):
        return float(sample/1000)
    
    def _sample_to_posz(self, sample):
        return (2* sample+1) * HALF_HEIGHT
    
    def _to_pos(self, sample_x, sample_z):
        return [self._sample_to_posxy(sample_x), 0, self._sample_to_posz(sample_z)]

    def _boundry_to_pos(self, boundry):       
        return [self._to_pos(boundry[0], boundry[1]), 
                self._to_pos(boundry[2], boundry[3])]
    
    def step(self, action): 

        #Interact with the PyBullet env
        pos = self._to_pos(action.get('x_pos'), action.get('z_pos')))
        output = self.assembly_env.interact(pos)

        #Calculate the reward
        param_material = -1
        param_distance = 1
        reward = param_material + param_distance * output.get('distance', np.inf)

        #Check termination
        termination = self._check_termination(output) is True

        return self._get_observation(), reward, termination, self._get_info()
    
    def reset(self):
        #print("-----------reset simulation---------------")
        self.assembly_env.reset()       
        return self._get_observation()
    
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
        self.assembly_env.close()

if __name__ == "__main__":
    # 如果你安装了pytorch，则使用上面的，如果你安装了tensorflow，则使用from stable_baselines.common.env_checker import check_env
    # from stable_baselines3.common.env_checker import check_env
    from stable_baselines.common.env_checker import check_env 
    env = AssemblyGymEnv()
    check_env(env)