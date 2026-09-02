"""pure wall-following geometry: ray lookup by bearing and the lookahead distance error."""

import numpy as np

WALL_RAY_A_DEG = -20.0
WALL_RAY_B_DEG = -90.0
DEADBAND_M = 0.02


def range_at_angle(ranges, angle_deg, angle_min, angle_increment):
    """range at a bearing in degrees, measured from straight ahead, index clamped to the scan."""
    index = int(round((np.radians(angle_deg) - angle_min) / angle_increment))
    index = min(max(index, 0), len(ranges) - 1)
    return ranges[index]


def wall_distance_error(ranges, range_min, range_max, angle_min, angle_increment,
                        target_distance_m, speed_mps, dt_sec):
    """signed error to the right wall, projected forward by one timestep of travel."""
    theta = np.radians(WALL_RAY_A_DEG - WALL_RAY_B_DEG)
    dist_b = range_at_angle(ranges, WALL_RAY_B_DEG, angle_min, angle_increment)
    dist_a = range_at_angle(ranges, WALL_RAY_A_DEG, angle_min, angle_increment)

    if dist_a < range_min or dist_a > range_max or dist_b < range_min or dist_b > range_max:
        return 0.0

    alpha = np.arctan((dist_a * np.cos(theta) - dist_b) / (dist_a * np.sin(theta)))
    perpendicular = dist_b * np.cos(alpha)

    if speed_mps is None:
        speed_mps = 0

    lookahead = perpendicular + speed_mps * dt_sec * np.sin(alpha)
    error = target_distance_m - lookahead

    # deadband stops the pid dithering around a wall the car is already tracking
    if np.abs(error) < DEADBAND_M:
        return 0.0

    return error
