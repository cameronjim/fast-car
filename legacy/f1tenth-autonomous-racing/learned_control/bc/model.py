"""behavioural cloning network: normalized lidar in, normalized steering and speed out."""

import torch
import torch.nn as nn


class BCNet(nn.Module):
    """mlp: num_lidar_rays -> 256 -> 128 -> (steering_angle, speed)."""

    def __init__(self, num_lidar_rays: int = 181) -> None:
        super().__init__()

        self.net = nn.Sequential(
            nn.Linear(num_lidar_rays, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 2),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)
