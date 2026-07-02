"""pure collision math for the learned safety node: danger zone, ttc, clearance, wall bias."""

import numpy as np

HALF_WIDTH_M = 0.7
STANDSTILL_MPS = 0.01
RECOVERY_SPAN_RAYS = 60


def forward_ray(steering_angle, angle_increment, num_rays):
    """index of the ray the car is currently steering toward."""
    ray = int(steering_angle / angle_increment + num_rays // 2)
    return int(np.clip(ray, 0, num_rays - 1))


def danger_zone_min_range(ranges, target_ray, angle_increment, half_width_m=HALF_WIDTH_M):
    """closest range inside the cone the car's half width subtends at target_ray."""
    num_rays = len(ranges)
    # subtended half-angle in rays, from the scan's own resolution, not a fixed deg/ray
    danger_rays = int(np.arctan2(half_width_m, ranges[target_ray]) / angle_increment)

    lower = target_ray - danger_rays if target_ray - danger_rays > 0 else 0
    upper = (target_ray + danger_rays + 1
             if target_ray + danger_rays + 1 < num_rays - 1 else num_rays - 1)
    # a zero-width zone on the last ray leaves nothing to measure, so report no obstacle
    if lower >= upper:
        return float('inf')
    return float(np.min(ranges[lower:upper]))


def time_to_collision(min_distance_m, speed_mps):
    """seconds to impact, infinite at a standstill where ttc says nothing about safety."""
    if speed_mps < STANDSTILL_MPS:
        return float('inf')
    return min_distance_m / speed_mps


def forward_min_range(ranges, half_span_rays=RECOVERY_SPAN_RAYS):
    """closest range straight ahead, -inf on an empty sector so it never reads as clear."""
    center = len(ranges) // 2
    forward = ranges[max(0, center - half_span_rays):center + half_span_rays]
    if len(forward) == 0:
        return float('-inf')
    return float(np.min(forward))


def wall_steer_bias(ranges, side_margin_m, steer_gain, max_bias):
    """steering nudge away from whichever side wall is inside side_margin_m."""
    num_rays = len(ranges)
    left_sector = ranges[(3 * num_rays) // 4:(7 * num_rays) // 8]
    right_sector = ranges[num_rays // 8:num_rays // 4]

    left_clearance = float(np.min(left_sector)) if len(left_sector) > 0 else float('inf')
    right_clearance = float(np.min(right_sector)) if len(right_sector) > 0 else float('inf')

    left_push = max(0.0, side_margin_m - left_clearance)
    right_push = max(0.0, side_margin_m - right_clearance)

    # positive steering is left, so a close left wall subtracts
    bias = steer_gain * (right_push - left_push)
    return float(np.clip(bias, -max_bias, max_bias))
