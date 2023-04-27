import os
import gym
import numpy as np
from gym import spaces
import graph
import pybullet as p2
import pybullet_data
from pybullet_utils import bullet_client as bc

class MySim(gym.Env):
    #Not necessary
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
        #Assume all the bricks are sizes 80mm x 40mm 
        #Assume we are working in a 1m x 1m environment (based on two abb gofa robot, adjustable) 
        # 1m / 40mm = 25 As a result, Z is discretized into 25 
        # Future Implementation (Orientation? Y-axis)
        # self.x_min = -0.5
        # self.x_max = 0.5
        # self.z_min = 0
        # self.z_max = 1

        #Option 01 Continuous
        # self.action_space = spaces.Dict({"x_pos":spaces.Box(low=self.x_min,high=self.x_max),
        #                                  "z_pos":spaces.Discrete(25)})

        #Option 02
        self.action_space = spaces.Dict({"x_pos":spaces.Discrete(1000),
                                         "z_pos":spaces.Discrete(25)})        
        
        ## Observation Space
        #Option 01 Use a multi-binary to represent a image
        #Will be hard to connect to the pybullet simulation
        #self.observation_space = spaces.MultiBinary([1000,25])

        #Option 02 Use Graph to Remember the location and Adjacency Matrix
        #Use Adjacency Matrix to reduce the computation time
        #TODO Consider if we should have randomized start and target in the future, but not for now
        self.observation_space = graph.Graph(node_space=spaces.MultiDiscrete([1000,25]),
                                             edge_space =None)


    def step(self, action): 
        p = self._p
        #Get Location of the Block(x,y,z)
        x = action["x_pos"]/1000
        y = 0
        z = action["z_pos"]*25+20  

        ## TODO Check the collision
        #Option 1 Pybullet

        #Option 2 Signed Distance Function
        
        ## TODO Check the Stability
        #Option 1 Pybullet
        
        #TODO Check the robot reachability with collision mesh

        #TODO Calculate the reward 

        state = 1

        if action == 2:
            reward = 1
        else:
            reward = -1
        done = True
        info = {}
        return state, reward, done, info
    
    def reset(self):
    #    print("-----------reset simulation---------------")
        if self._physics_client_id < 0:
            if self._renders:
                self._p = bc.BulletClient(connection_mode=p2.GUI)
            else:
                self._p = bc.BulletClient()
            self._physics_client_id = self._p._client
            p = self._p
            p.resetSimulation()

            # Import Ground URDF
            p.loadURDF(os.path.join(self._urdfRoot, "plane.urdf"), [0, 0, -1])
            
            # Set Gravity Simulation
            p.setGravity(0, 0, -9.8)
            self.timeStep = 0.02
            p.setTimeStep(self.timeStep)
            p.setRealTimeSimulation(0) #问题？？？ 这个应该选什么
        
        p = self._p
        #TODO: Remove all blocks in the environment

        #TODO: Creat the Ground Node

        #TODO: Return the ground node as the state / observation_space
        return np.array(self.state)
    
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

    # Not necessary
    # Close the Simulation 
    def close(self):
        if self._physics_client_id >= 0:
            self._p.disconnect()
        self._physics_client_id = -1

if __name__ == "__main__":
    from stable_baselines.common.env_checker import check_env 
    # 如果你安装了pytorch，则使用上面的，如果你安装了tensorflow，则使用from stable_baselines.common.env_checker import check_env
    # from stable_baselines3.common.env_checker import check_env
    env = MySim()
    check_env(env)