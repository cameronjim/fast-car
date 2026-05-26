"""
Reward function for SAC training on the F1Tenth car.

Called every step by sac_train_node to compute the reward signal
from raw LiDAR ranges, speed, steering angle, and crash flag.
"""

import numpy as np


def compute_reward(
    lidar_ranges: np.ndarray,
    speed: float,
    steering_angle: float,
    done: bool,
    prev_steering: float = 0.0,
) -> float:
    """Compute single-step reward for SAC training.

    Args:
        lidar_ranges: Downsampled LiDAR distances in **meters** (before
            normalization), shape (num_rays,).
        speed: Physical forward speed in m/s.
        steering_angle: Physical steering angle in radians.
        done: True when the episode ends (emergency stop / crash).
        prev_steering: Previous step's steering angle (for jerk penalty).

    Returns:
        Scalar reward value.
    """
    reward = 0.0

    # 1. Survival bonus: reward for staying alive
    reward += 0.1

    # 2. Forward progress: encourage speed
    reward += speed * 0.1

    # 3. Wall proximity: penalise being close to obstacles
    min_range = float(np.min(lidar_ranges))
    if min_range < 0.5:
        reward -= (0.5 - min_range) * 2.0

    # 4. Steering smoothness: penalise jerk (change), NOT absolute steering
    reward -= 0.8 * abs(steering_angle - prev_steering)

    # 5. Crash penalty: penalise crashing
    if done:
        reward -= 50.0

    return reward
