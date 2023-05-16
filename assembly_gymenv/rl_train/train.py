from stable_baselines3 import A2C, PPO
from stable_baselines3.common.env_util import make_vec_env

from assembly_modules import BinaryCNN
import gym
import assembly_gymenv

n_env = 1


def make_env():
    return gym.make('assembly_gymenv/AssemblyGymEnv-v0')

env = make_vec_env(make_env, 
                   n_envs=n_env)

policy_kwargs = dict(
    features_extractor_class=BinaryCNN,
    features_extractor_kwargs=dict(features_dim=32, n_env=n_env),
    normalize_images=False
)

model = PPO("CnnPolicy", 
            env, 
            n_steps=4,
            batch_size=4,
            policy_kwargs=policy_kwargs, 
            verbose=1)


if __name__=="__main__":
    model.learn(1000)
