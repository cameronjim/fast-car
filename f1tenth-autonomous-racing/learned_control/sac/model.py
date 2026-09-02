"""sac networks: a tanh-squashed gaussian actor and the q-function critic."""
from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal

LOG_STD_MIN = -20
LOG_STD_MAX = 2


class SACActorNet(nn.Module):
    """gaussian policy squashed with tanh into actions in [0, 1]."""

    def __init__(self, num_lidar_rays: int = 181, action_dim: int = 2) -> None:
        super().__init__()
        self.fc1 = nn.Linear(num_lidar_rays, 256)
        self.fc2 = nn.Linear(256, 128)
        self.mean_head = nn.Linear(128, action_dim)
        self.log_std_head = nn.Linear(128, action_dim)

    def forward(self, state: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        x = F.relu(self.fc1(state))
        x = F.relu(self.fc2(x))
        mean = self.mean_head(x)
        log_std = torch.clamp(self.log_std_head(x), LOG_STD_MIN, LOG_STD_MAX)
        return mean, log_std

    def sample(self, state: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """reparameterized sample, returning action in [0, 1], its log_prob, and the raw mean."""
        mean, log_std = self.forward(state)
        std = log_std.exp()
        dist = Normal(mean, std)
        x_t = dist.rsample()

        y_t = torch.tanh(x_t)
        action = (y_t + 1.0) / 2.0

        # jacobian correction for the tanh squash, without which entropy tuning is wrong
        log_prob = dist.log_prob(x_t) - torch.log(1.0 - y_t.pow(2) + 1e-6)
        log_prob = log_prob.sum(dim=-1, keepdim=True)

        return action, log_prob, mean

    def get_action(self, state: torch.Tensor, deterministic: bool = False) -> torch.Tensor:
        """action for driving, no gradient tracking."""
        with torch.no_grad():
            mean, log_std = self.forward(state)
            if deterministic:
                return (torch.tanh(mean) + 1.0) / 2.0
            std = log_std.exp()
            x = Normal(mean, std).sample()
            return (torch.tanh(x) + 1.0) / 2.0

    @classmethod
    def from_bc(cls, bc_weights_path: str, num_lidar_rays: int = 181,
                action_dim: int = 2, device: str = "cpu") -> "SACActorNet":
        """actor initialized from trained bc weights."""
        actor = cls(num_lidar_rays, action_dim)
        bc_sd = torch.load(bc_weights_path, map_location=device, weights_only=True)
        actor.fc1.weight.data.copy_(bc_sd["net.0.weight"])
        actor.fc1.bias.data.copy_(bc_sd["net.0.bias"])
        actor.fc2.weight.data.copy_(bc_sd["net.2.weight"])
        actor.fc2.bias.data.copy_(bc_sd["net.2.bias"])
        # bc emits [0, 1] directly; atanh is near-linear around 0.5, so mean ~= 2y - 1
        actor.mean_head.weight.data.copy_(2.0 * bc_sd["net.4.weight"])
        actor.mean_head.bias.data.copy_(2.0 * bc_sd["net.4.bias"] - 1.0)
        nn.init.constant_(actor.log_std_head.weight, 0.0)
        nn.init.constant_(actor.log_std_head.bias, -3.0)
        return actor


class SACCriticNet(nn.Module):
    """q-function: (state, action) -> scalar value."""

    def __init__(self, num_lidar_rays: int = 181, action_dim: int = 2) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(num_lidar_rays + action_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 1),
        )

    def forward(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        return self.net(torch.cat([state, action], dim=-1))
