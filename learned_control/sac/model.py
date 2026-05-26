"""
SAC actor and critic networks for the F1Tenth car.

Contains SACActorNet (Gaussian policy with tanh squashing) and
SACCriticNet (Q-function). Imported by sac_train_node and sac_demo_node.
"""
from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal

LOG_STD_MIN = -20
LOG_STD_MAX = 2


class SACActorNet(nn.Module):
    """
    SAC Gaussian actor with tanh squashing -> actions in [0, 1].

    Uses tanh squashing (like stable-baselines3) instead of hard clamp.
    This gives proper gradients and correct log_prob for entropy tuning.
    """

    def __init__(self, num_lidar_rays: int = 181, action_dim: int = 2) -> None:
        """
        Initializes the SAC actor network.

        Args:
            num_lidar_rays: The number of LiDAR rays (input features).
            action_dim: The number of action dimensions (steering, speed).

        Returns:
            None
        """
        super().__init__()
        # Initialize the first and second hidden layers
        self.fc1 = nn.Linear(num_lidar_rays, 256)
        self.fc2 = nn.Linear(256, 128)
        # Initialize the mean and log standard deviation heads
        self.mean_head = nn.Linear(128, action_dim)
        self.log_std_head = nn.Linear(128, action_dim)

    def forward(self, state: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """
        Forward pass of the actor network.

        Args:
            state: Normalized LiDAR input, shape (batch, num_lidar_rays).

        Returns:
            A tuple of (mean, log_std) for the action distribution.
        """
        # Forward pass through the first and second hidden layers
        x = F.relu(self.fc1(state))
        x = F.relu(self.fc2(x))
        # Forward pass through the mean and log standard deviation heads
        mean = self.mean_head(x)
        log_std = torch.clamp(self.log_std_head(x), LOG_STD_MIN, LOG_STD_MAX)
        return mean, log_std

    def sample(self, state: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Sample action with tanh squashing to [0, 1].

        Returns:
            action: squashed action in [0, 1]
            log_prob: corrected log-probability (accounts for tanh)
            mean: raw mean (pre-squash)
        """
        # Forward pass through the network
        mean, log_std = self.forward(state)
        # Compute the standard deviation
        std = log_std.exp()
        # Create the normal distribution
        dist = Normal(mean, std)
        # Sample from the distribution
        x_t = dist.rsample()

        # Apply tanh squashing to (-1, 1), then scale to (0, 1)
        y_t = torch.tanh(x_t)
        action = (y_t + 1.0) / 2.0

        # Correct log_prob for the tanh squashing
        log_prob = dist.log_prob(x_t) - torch.log(1.0 - y_t.pow(2) + 1e-6)
        log_prob = log_prob.sum(dim=-1, keepdim=True)

        return action, log_prob, mean

    def get_action(self, state: torch.Tensor, deterministic: bool = False) -> torch.Tensor:
        """
        Get action for driving (no gradient tracking).

        Args:
            state: Normalized LiDAR input, shape (1, num_lidar_rays).
            deterministic: If True, return the mean action without sampling.

        Returns:
            Action tensor in [0, 1], shape (1, action_dim).
        """
        # Forward pass through the network
        with torch.no_grad():
            mean, log_std = self.forward(state)
            # If deterministic, return the mean action
            if deterministic:
                return (torch.tanh(mean) + 1.0) / 2.0
            # Compute the standard deviation
            std = log_std.exp()
            # Sample from the distribution
            x = Normal(mean, std).sample()
            return (torch.tanh(x) + 1.0) / 2.0

    @classmethod
    def from_bc(cls, bc_weights_path: str, num_lidar_rays: int = 181,
                action_dim: int = 2, device: str = "cpu") -> "SACActorNet":
        """Create actor initialized from trained BC model weights.

        BC outputs normalized actions in [0, 1] directly. The SAC actor
        produces a Gaussian mean that is later squashed with tanh and
        rescaled to [0, 1]. Around the nominal operating point y ~= 0.5,
        atanh(2y - 1) is approximately linear, so we map the BC head into
        that pre-squash space with mean ~= 2y - 1.

        Args:
            bc_weights_path: Path to the trained BC model .pth file.
            num_lidar_rays: The number of LiDAR rays (input features).
            action_dim: The number of action dimensions.
            device: The device to load the weights onto.

        Returns:
            A SACActorNet initialized from the BC weights.
        """
        # Initialize the actor network
        actor = cls(num_lidar_rays, action_dim)
        # Load the BC weights
        bc_sd = torch.load(bc_weights_path, map_location=device, weights_only=True)
        # Copy the weights to the actor network
        actor.fc1.weight.data.copy_(bc_sd["net.0.weight"])
        actor.fc1.bias.data.copy_(bc_sd["net.0.bias"])
        actor.fc2.weight.data.copy_(bc_sd["net.2.weight"])
        actor.fc2.bias.data.copy_(bc_sd["net.2.bias"])
        actor.mean_head.weight.data.copy_(2.0 * bc_sd["net.4.weight"])
        actor.mean_head.bias.data.copy_(2.0 * bc_sd["net.4.bias"] - 1.0)
        nn.init.constant_(actor.log_std_head.weight, 0.0)
        nn.init.constant_(actor.log_std_head.bias, -3.0)
        return actor


class SACCriticNet(nn.Module):
    """
    SAC Q-function: (state, action) -> scalar Q-value.
    """

    def __init__(self, num_lidar_rays: int = 181, action_dim: int = 2) -> None:
        """
        Initializes the SAC critic network.

        Args:
            num_lidar_rays: The number of LiDAR rays (state features).
            action_dim: The number of action dimensions.

        Returns:
            None
        """
        super().__init__()
        # Initialize the network
        self.net = nn.Sequential(
            # Initialize the first hidden layer
            nn.Linear(num_lidar_rays + action_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 1),
        )

    def forward(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        """
        Forward pass of the critic network.

        Args:
            state: Normalized LiDAR input, shape (batch, num_lidar_rays).
            action: Action tensor in [0, 1], shape (batch, action_dim).

        Returns:
            Q-value estimate, shape (batch, 1).
        """
        # Forward pass through the network
        return self.net(torch.cat([state, action], dim=-1))
