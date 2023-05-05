from gym.envs.registration import register

register(
    id='assembly_gymenv/AssemblyGymEnv-v0',
    entry_point='assembly_gymenv.envs:AssemblyGymEnv',
    max_episode_steps=2500,
)