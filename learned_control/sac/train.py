"""soft actor-critic algorithm: replay buffer, trainer, and checkpoint io, no ros."""

from __future__ import annotations
import argparse
import copy
import os
import numpy as np
import torch
import torch.nn.functional as F

# works both as an installed ros2 package and when run straight out of sac/
try:
    from learned_control.sac.model import SACActorNet, SACCriticNet
except ImportError:
    from model import SACActorNet, SACCriticNet


class ReplayBuffer:
    """fixed-capacity circular buffer backed by pre-allocated numpy arrays."""

    def __init__(self, capacity: int, state_dim: int, action_dim: int) -> None:
        self.capacity = capacity
        self.states = np.zeros((capacity, state_dim), dtype=np.float32)
        self.actions = np.zeros((capacity, action_dim), dtype=np.float32)
        self.rewards = np.zeros(capacity, dtype=np.float32)
        self.next_states = np.zeros((capacity, state_dim), dtype=np.float32)
        self.dones = np.zeros(capacity, dtype=np.float32)
        self.ptr = 0
        self.size = 0

    def push(self, state, action, reward, next_state, done) -> None:
        """store one transition, overwriting the oldest once full."""
        self.states[self.ptr] = state
        self.actions[self.ptr] = action
        self.rewards[self.ptr] = reward
        self.next_states[self.ptr] = next_state
        self.dones[self.ptr] = float(done)
        self.ptr = (self.ptr + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)

    def sample(self, batch_size: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """random batch of (states, actions, rewards, next_states, dones)."""
        idx = np.random.randint(0, self.size, size=batch_size)
        return (
            self.states[idx],
            self.actions[idx],
            self.rewards[idx],
            self.next_states[idx],
            self.dones[idx],
        )

    def __len__(self) -> int:
        return self.size


class SACTrainer:
    """networks, optimisers, and the single sac update step."""

    def __init__(
        self,
        actor: SACActorNet,
        critic1: SACCriticNet,
        critic2: SACCriticNet,
        *,
        state_dim: int = 181,
        action_dim: int = 2,
        lr_actor: float = 3e-4,
        lr_critic: float = 3e-4,
        lr_alpha: float = 3e-4,
        gamma: float = 0.99,
        tau: float = 0.005,
        buffer_size: int = 100_000,
        batch_size: int = 256,
        target_entropy: float | None = None,
        device: str = "cpu",
    ) -> None:
        self.device = torch.device(device)
        self.gamma = gamma
        self.tau = tau
        self.batch_size = batch_size

        self.actor = actor.to(self.device)
        self.critic1 = critic1.to(self.device)
        self.critic2 = critic2.to(self.device)
        self.target_critic1 = copy.deepcopy(self.critic1)
        self.target_critic2 = copy.deepcopy(self.critic2)
        # targets move by polyak averaging only, never by gradient
        for p in self.target_critic1.parameters():
            p.requires_grad = False
        for p in self.target_critic2.parameters():
            p.requires_grad = False

        self.actor_optim = torch.optim.Adam(self.actor.parameters(), lr=lr_actor)
        self.critic1_optim = torch.optim.Adam(self.critic1.parameters(), lr=lr_critic)
        self.critic2_optim = torch.optim.Adam(self.critic2.parameters(), lr=lr_critic)

        self.target_entropy = (
            target_entropy if target_entropy is not None else -float(action_dim)
        )
        self.log_alpha = torch.zeros(1, requires_grad=True, device=self.device)
        self.alpha_optim = torch.optim.Adam([self.log_alpha], lr=lr_alpha)

        self.buffer = ReplayBuffer(buffer_size, state_dim, action_dim)
        self.reference_actor = None

        self.total_updates = 0

    @property
    def alpha(self) -> float:
        """current entropy temperature."""
        return self.log_alpha.exp().item()

    def store(self, state, action, reward, next_state, done) -> None:
        self.buffer.push(state, action, reward, next_state, done)

    def ready(self) -> bool:
        """true once the buffer holds at least one full batch."""
        return len(self.buffer) >= self.batch_size

    def set_reference_actor(self, actor: SACActorNet | None) -> None:
        """freeze a copy of actor as the bc regularization target; None disables it."""
        if actor is None:
            self.reference_actor = None
            return
        self.reference_actor = copy.deepcopy(actor).to(self.device)
        self.reference_actor.eval()
        for p in self.reference_actor.parameters():
            p.requires_grad = False

    def update(
        self,
        *,
        update_actor: bool = True,
        bc_reg_weight: float = 0.0,
    ) -> dict | None:
        """one gradient step on every network, or None while the buffer is short."""
        if not self.ready():
            return None

        states, actions, rewards, next_states, dones = self.buffer.sample(
            self.batch_size
        )
        s = torch.as_tensor(states, device=self.device)
        a = torch.as_tensor(actions, device=self.device)
        r = torch.as_tensor(rewards, device=self.device).unsqueeze(1)
        ns = torch.as_tensor(next_states, device=self.device)
        d = torch.as_tensor(dones, device=self.device).unsqueeze(1)

        alpha = self.log_alpha.exp().detach()

        with torch.no_grad():
            na, nlp, _ = self.actor.sample(ns)
            tq1 = self.target_critic1(ns, na)
            tq2 = self.target_critic2(ns, na)
            target_q = torch.min(tq1, tq2) - alpha * nlp
            target = r + self.gamma * (1.0 - d) * target_q

        q1 = self.critic1(s, a)
        c1_loss = F.mse_loss(q1, target)
        self.critic1_optim.zero_grad()
        c1_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.critic1.parameters(), 1.0)
        self.critic1_optim.step()

        q2 = self.critic2(s, a)
        c2_loss = F.mse_loss(q2, target)
        self.critic2_optim.zero_grad()
        c2_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.critic2.parameters(), 1.0)
        self.critic2_optim.step()

        actor_loss_value = float("nan")
        alpha_loss_value = float("nan")
        bc_loss_value = float("nan")

        if update_actor:
            new_a, log_prob, _ = self.actor.sample(s)
            q1_new = self.critic1(s, new_a)
            q2_new = self.critic2(s, new_a)
            q_new = torch.min(q1_new, q2_new)
            actor_loss = (alpha * log_prob - q_new).mean()

            if self.reference_actor is not None and bc_reg_weight > 0.0:
                with torch.no_grad():
                    ref_action = self.reference_actor.get_action(
                        s, deterministic=True)
                bc_loss = F.mse_loss(new_a, ref_action)
                actor_loss = actor_loss + bc_reg_weight * bc_loss
                bc_loss_value = bc_loss.item()
            else:
                bc_loss_value = 0.0

            self.actor_optim.zero_grad()
            actor_loss.backward()
            torch.nn.utils.clip_grad_norm_(self.actor.parameters(), 1.0)
            self.actor_optim.step()

            alpha_loss = -(
                self.log_alpha * (log_prob.detach() + self.target_entropy)
            ).mean()

            self.alpha_optim.zero_grad()
            alpha_loss.backward()
            self.alpha_optim.step()

            actor_loss_value = actor_loss.item()
            alpha_loss_value = alpha_loss.item()

        with torch.no_grad():
            for tp, p in zip(
                self.target_critic1.parameters(), self.critic1.parameters()
            ):
                tp.data.mul_(1.0 - self.tau).add_(p.data, alpha=self.tau)
            for tp, p in zip(
                self.target_critic2.parameters(), self.critic2.parameters()
            ):
                tp.data.mul_(1.0 - self.tau).add_(p.data, alpha=self.tau)

        self.total_updates += 1

        return {
            "critic1_loss": round(c1_loss.item(), 5),
            "critic2_loss": round(c2_loss.item(), 5),
            "actor_loss": round(actor_loss_value, 5),
            "alpha_loss": round(alpha_loss_value, 5),
            "alpha": round(self.alpha, 5),
            "bc_loss": round(bc_loss_value, 5),
        }

    def save(self, path: str) -> None:
        """write every network and optimizer state to one checkpoint file."""
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        torch.save(
            {
                "actor": self.actor.state_dict(),
                "critic1": self.critic1.state_dict(),
                "critic2": self.critic2.state_dict(),
                "target_critic1": self.target_critic1.state_dict(),
                "target_critic2": self.target_critic2.state_dict(),
                "actor_optim": self.actor_optim.state_dict(),
                "critic1_optim": self.critic1_optim.state_dict(),
                "critic2_optim": self.critic2_optim.state_dict(),
                "log_alpha": self.log_alpha.data,
                "alpha_optim": self.alpha_optim.state_dict(),
                "total_updates": self.total_updates,
            },
            path,
        )

    def load(self, path: str) -> None:
        """restore every network and optimizer state from a checkpoint file."""
        ckpt = torch.load(path, map_location=self.device, weights_only=True)
        self.actor.load_state_dict(ckpt["actor"])
        self.critic1.load_state_dict(ckpt["critic1"])
        self.critic2.load_state_dict(ckpt["critic2"])
        self.target_critic1.load_state_dict(ckpt["target_critic1"])
        self.target_critic2.load_state_dict(ckpt["target_critic2"])
        self.actor_optim.load_state_dict(ckpt["actor_optim"])
        self.critic1_optim.load_state_dict(ckpt["critic1_optim"])
        self.critic2_optim.load_state_dict(ckpt["critic2_optim"])
        self.log_alpha.data.copy_(ckpt["log_alpha"])
        self.alpha_optim.load_state_dict(ckpt["alpha_optim"])
        self.total_updates = ckpt.get("total_updates", 0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Initialise a SAC checkpoint (optionally from BC weights)"
    )
    parser.add_argument("--bc-weights", default=None, help="Path to trained BC model .pth")
    parser.add_argument("--num-lidar", type=int, default=181)
    parser.add_argument("--out", default="sac/sac_checkpoint.pth", help="Output path")
    args = parser.parse_args()

    if args.bc_weights:
        actor = SACActorNet.from_bc(args.bc_weights, num_lidar_rays=args.num_lidar)
        print(f"actor initialised from bc weights: {args.bc_weights}")
    else:
        actor = SACActorNet(num_lidar_rays=args.num_lidar)
        print("actor initialised with random weights")

    critic1 = SACCriticNet(num_lidar_rays=args.num_lidar)
    critic2 = SACCriticNet(num_lidar_rays=args.num_lidar)

    trainer = SACTrainer(
        actor, critic1, critic2,
        state_dim=args.num_lidar,
    )
    trainer.save(args.out)
    print(f"checkpoint saved to {args.out}")
