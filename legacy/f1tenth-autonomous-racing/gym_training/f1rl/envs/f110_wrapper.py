# rl-facing wrapper: normalized obs, [-1, 1] actions, and the training reward

from __future__ import annotations

import gymnasium as gym
import numpy as np

from .obs import OBS_CLIP_ABS, ActionBounds, ObsConfig
from .reward import ProgressReward

STEER_COLUMN = 0
SPEED_COLUMN = 1


class F110RLWrapper(gym.Wrapper):
    """maps a flattened f1tenth env onto a unit action box and a progress reward."""

    def __init__(
        self,
        env: gym.Env,
        obs_cfg: ObsConfig,
        action_bounds: ActionBounds,
        reward: ProgressReward,
        action_repeat: int = 1,
        wrong_way_steps: int = 0,
    ):
        super().__init__(env)
        if action_repeat < 1:
            raise ValueError(f"action_repeat must be >= 1, got {action_repeat}")
        self.obs_cfg = obs_cfg
        self.action_bounds = action_bounds
        self.reward = reward
        self.action_repeat = int(action_repeat)
        self.wrong_way_steps = int(wrong_way_steps)

        raw_low = np.asarray(env.action_space.low, dtype=np.float64)
        raw_high = np.asarray(env.action_space.high, dtype=np.float64)
        if action_bounds.steer_max_rad > raw_high[STEER_COLUMN]:
            raise ValueError(
                f"steer_max_rad {action_bounds.steer_max_rad} exceeds the vehicle limit "
                f"{raw_high[STEER_COLUMN]}"
            )
        if action_bounds.speed_cap_mps > raw_high[SPEED_COLUMN]:
            raise ValueError(
                f"speed_cap_mps {action_bounds.speed_cap_mps} exceeds the vehicle limit "
                f"{raw_high[SPEED_COLUMN]}"
            )
        if action_bounds.speed_min_mps < raw_low[SPEED_COLUMN]:
            raise ValueError(
                f"speed_min_mps {action_bounds.speed_min_mps} is below the vehicle limit "
                f"{raw_low[SPEED_COLUMN]}"
            )

        self.observation_space = gym.spaces.Box(
            low=-OBS_CLIP_ABS,
            high=OBS_CLIP_ABS,
            shape=(obs_cfg.obs_dim,),
            dtype=np.float32,
        )
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        self._prev_action = np.zeros(2, dtype=np.float32)
        self._lap_count = 0
        self._reverse_steps = 0

    def set_speed_cap(self, speed_cap_mps: float) -> None:
        """raise or lower the commanded top speed, for the speed curriculum."""
        self.action_bounds = self.action_bounds.with_updates(speed_cap_mps=float(speed_cap_mps))

    def rescale_action(self, action) -> np.ndarray:
        """unit action to [steering rad, speed m/s], clipped at both boundaries."""
        unit = np.clip(np.asarray(action, dtype=np.float64).reshape(2), -1.0, 1.0)
        bounds = self.action_bounds
        steering = unit[STEER_COLUMN] * bounds.steer_max_rad
        speed_mid = 0.5 * (bounds.speed_cap_mps + bounds.speed_min_mps)
        speed_half = 0.5 * (bounds.speed_cap_mps - bounds.speed_min_mps)
        speed = speed_mid + unit[SPEED_COLUMN] * speed_half
        return np.array(
            [
                np.clip(steering, -bounds.steer_max_rad, bounds.steer_max_rad),
                np.clip(speed, bounds.speed_min_mps, bounds.speed_cap_mps),
            ],
            dtype=np.float32,
        )

    def command_for_substep(self, unit) -> np.ndarray:
        """the [steering, speed] handed to one physics step; held constant across a control step here."""
        return self.rescale_action(unit)

    def context_vector(self):
        """raw reference-line context appended before prev_action; the base wrapper has none."""
        return None

    def reset(self, *, seed=None, options=None):
        raw_obs, info = self.env.reset(seed=seed, options=options)
        self._prev_action = np.zeros(2, dtype=np.float32)
        self._lap_count = 0
        self._reverse_steps = 0
        return self.obs_cfg.normalize(raw_obs, self._prev_action, self.context_vector()), info

    def step(self, action):
        unit = np.clip(np.asarray(action, dtype=np.float32).reshape(2), -1.0, 1.0)

        progress_m = 0.0
        terminated = truncated = False
        lap_time_sec = None
        info: dict = {}
        raw_obs = None
        command = None
        for _ in range(self.action_repeat):
            command = self.command_for_substep(unit)
            raw_obs, _, terminated, truncated, info = self.env.step(command)
            progress_m += float(np.asarray(info["progress"]).reshape(-1)[0])
            lap_count = int(np.asarray(info["lap_counts"]).reshape(-1)[0])
            if lap_count > self._lap_count:
                self._lap_count = lap_count
                lap_time_sec = float(np.asarray(info["lap_times"]).reshape(-1)[0])
            if terminated or truncated:
                break

        is_collision = bool(np.asarray(info["collisions"]).reshape(-1)[0] > 0)
        reward = self.reward.step_reward(
            progress_m, unit, self._prev_action, is_collision, lap_time_sec
        )

        self._reverse_steps = self._reverse_steps + 1 if progress_m < 0.0 else 0
        wrong_way = self.wrong_way_steps > 0 and self._reverse_steps >= self.wrong_way_steps
        if wrong_way:
            terminated = True

        self._prev_action = unit
        obs = self.obs_cfg.normalize(raw_obs, unit, self.context_vector())
        info = dict(info)
        info.update(
            {
                "progress_m": progress_m,
                "is_collision": is_collision,
                "lap_count": self._lap_count,
                "wrong_way": wrong_way,
                # the last physics step's command, which residual mode recomputes every substep
                "command": command,
            }
        )
        if lap_time_sec is not None:
            info["lap_time_sec"] = lap_time_sec
        return obs, reward, terminated, truncated, info
