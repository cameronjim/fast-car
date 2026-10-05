"""per-step reward for sac training, from raw lidar ranges, speed, steering, and crash flag."""

import numpy as np

SURVIVAL_BONUS = 0.1
SPEED_GAIN = 0.1
WALL_MARGIN_M = 0.5
WALL_PENALTY_GAIN = 2.0
JERK_PENALTY_GAIN = 0.8
CRASH_PENALTY = 50.0


def compute_reward(
    lidar_ranges: np.ndarray,
    speed: float,
    steering_angle: float,
    done: bool,
    prev_steering: float = 0.0,
) -> float:
    """reward for one step; lidar_ranges are metres, before normalization."""
    reward = 0.0

    reward += SURVIVAL_BONUS
    reward += speed * SPEED_GAIN

    min_range = float(np.min(lidar_ranges))
    if min_range < WALL_MARGIN_M:
        reward -= (WALL_MARGIN_M - min_range) * WALL_PENALTY_GAIN

    # penalises jerk, not absolute steering, so a sustained turn is not taxed
    reward -= JERK_PENALTY_GAIN * abs(steering_angle - prev_steering)

    if done:
        reward -= CRASH_PENALTY

    return reward
