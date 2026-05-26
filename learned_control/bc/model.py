"""
Behavioural Cloning network.

This module contains the Behavioural Cloning network class.
"""

import torch
import torch.nn as nn


class BCNet(nn.Module):
    """
    This class defines the architecture of the Behavioural Cloning network.

    Architecture:
        181 LiDAR rays -> 256 (ReLU) -> 128 (ReLU) -> 2 (steering_angle, speed)
    """

    def __init__(self, num_lidar_rays: int = 181) -> None:
        """
        Initializes the Behavioural Cloning network.

        Args:
            num_lidar_rays: The number of LiDAR rays.

        Returns:
            None
        """
        super().__init__()

        # Define the network architecture
        self.net = nn.Sequential(
            nn.Linear(num_lidar_rays, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 2),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Forward pass of the Behavioural Cloning network.

        Args:
            x: The input tensor.

        Returns:
            The output tensor.
        """
        return self.net(x)
