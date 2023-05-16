import torch as th
import torch.nn as nn

from gym import spaces
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor


class BinaryCNN(BaseFeaturesExtractor):

    def __init__(self, 
                 observation_space: spaces.MultiDiscrete, 
                 n_env: int = 4,
                 features_dim: int = 32):
        super().__init__(observation_space, features_dim)

        self.cnn = nn.Sequential(

            nn.Conv2d(1, 32, kernel_size=7, stride=3),
            nn.ReLU(),
            nn.MaxPool2d(2, stride=1),

            nn.Conv2d(32, 8, kernel_size=3, stride=1),
            nn.ReLU(),

            nn.Flatten(),
        )

        # Compute shape by doing one forward pass
        with th.no_grad():
            n_flatten = self.cnn(
                th.as_tensor(observation_space.sample()[None]).float()
            ).shape[1]

        self.linear = nn.Sequential(nn.Linear(n_flatten, features_dim), nn.Tanh())

    def forward(self, observations: th.Tensor) -> th.Tensor:
        # breakpoint()
        return self.linear(self.cnn(observations))
        # return th.randn(32)
    

