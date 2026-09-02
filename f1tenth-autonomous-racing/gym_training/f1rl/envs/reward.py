# per-step racing reward built from frenet progress

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ProgressReward:
    """forward arclength minus an action-rate penalty, with collision and lap terms."""

    w_prog: float = 1.0
    w_rate: float = 0.05
    collision_penalty: float = 10.0
    lap_bonus: float = 50.0
    target_lap_time_sec: float = 60.0

    def __post_init__(self) -> None:
        if self.collision_penalty < 0.0:
            raise ValueError(f"collision_penalty must be >= 0, got {self.collision_penalty}")
        if self.target_lap_time_sec <= 0.0:
            raise ValueError(f"target_lap_time_sec must be > 0, got {self.target_lap_time_sec}")

    def step_reward(
        self,
        progress_m: float,
        action,
        prev_action,
        is_collision: bool = False,
        lap_time_sec: float | None = None,
    ) -> float:
        """reward for one control step; lap_time_sec is set only on a lap-completing step."""
        reward = self.w_prog * float(progress_m)
        delta = np.asarray(action, dtype=np.float64) - np.asarray(prev_action, dtype=np.float64)
        reward -= self.w_rate * float(delta @ delta)
        if is_collision:
            reward -= self.collision_penalty
        if lap_time_sec is not None and lap_time_sec > 0.0:
            reward += self.lap_bonus * (self.target_lap_time_sec / float(lap_time_sec))
        return reward

    @classmethod
    def from_config(cls, cfg: dict | None) -> "ProgressReward":
        cfg = cfg or {}
        unknown = set(cfg) - {f for f in cls.__dataclass_fields__}
        if unknown:
            raise ValueError(f"unknown reward keys in config: {sorted(unknown)}")
        return cls(**cfg)
