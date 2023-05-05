from gym.envs.registration import register

register(
    id='assembly-gymenv/assembly_env',
    entry_point='assembly-gymenv.envs:gym_env',
    max_episode_steps=2500,
)