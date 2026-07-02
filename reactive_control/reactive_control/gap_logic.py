"""pure gap-following math: disparity extension, widest-gap target, corner dead-ends."""

import numpy as np


def extend_disparities(ranges, disparity_threshold_m, half_width_m, angle_increment):
    """shrink the rays behind each disparity edge so a chosen gap still fits the car."""
    safe_ranges = np.array(ranges, dtype=float)
    raw_ranges = np.asarray(ranges, dtype=float)

    edges = [i for i in range(len(safe_ranges) - 1)
             if np.abs(safe_ranges[i] - safe_ranges[i + 1]) > disparity_threshold_m]

    for i in edges:
        if raw_ranges[i] < raw_ranges[i + 1]:
            near = raw_ranges[i]
            direction = 1
            start = i + 1
        else:
            near = raw_ranges[i + 1]
            direction = -1
            start = i

        # half-width subtended at the near edge, converted to a ray count
        danger_rays = int(np.arctan2(half_width_m, near) / angle_increment)

        for step in range(danger_rays):
            k = start + direction * step
            if 0 <= k < len(safe_ranges) and safe_ranges[k] > near:
                safe_ranges[k] = near

    return safe_ranges


def select_target_ray(ranges, free_space_threshold_m, cone_left_fraction, cone_right_fraction):
    """center ray of the widest free gap in the forward cone, tie-broken toward straight."""
    ranges = np.asarray(ranges)
    num_rays = len(ranges)
    center = num_rays // 2

    # cone bounds are fractions of the scan so any ray count works
    left = int(num_rays * cone_left_fraction)
    right = int(num_rays * cone_right_fraction)
    cone = ranges[left:right]

    free = cone > free_space_threshold_m
    if not np.any(free):
        return center

    best_start = best_end = None
    best_len = -1
    best_center_dist = float("inf")

    i = 0
    while i < len(free):
        if not free[i]:
            i += 1
            continue
        start = i
        while i < len(free) and free[i]:
            i += 1
        end = i - 1

        gap_len = end - start + 1
        gap_center = (start + end) // 2
        dist_to_center = abs((gap_center + left) - center)

        if (gap_len > best_len) or (gap_len == best_len and dist_to_center < best_center_dist):
            best_len = gap_len
            best_start, best_end = start, end
            best_center_dist = dist_to_center

    return int((best_start + best_end) // 2 + left)


def corner_blocked(ranges, steering_angle, min_clearance_m):
    """true when the whole sector the car is turning into is inside min_clearance_m."""
    ranges = np.asarray(ranges)
    num_rays = len(ranges)
    if steering_angle < 0:
        return bool(np.all(ranges[int(num_rays * 5 / 6):] < min_clearance_m))
    return bool(np.all(ranges[:int(num_rays / 6)] < min_clearance_m))
