"""unit tests for the pure reactive safety math, no ros."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reactive_control.safety_logic import (  # noqa: E402
    danger_zone_min_range,
    forward_min_range,
    forward_ray,
    time_to_collision,
)

ANGLE_INCREMENT = np.radians(0.25)


def test_straight_ahead_maps_to_the_middle_ray():
    assert forward_ray(0.0, ANGLE_INCREMENT, 1080) == 540


def test_steering_shifts_the_target_ray_by_the_scan_resolution():
    assert forward_ray(10 * ANGLE_INCREMENT, ANGLE_INCREMENT, 1080) == 550
    assert forward_ray(-10 * ANGLE_INCREMENT, ANGLE_INCREMENT, 1080) == 530


def test_extreme_steering_clamps_inside_the_scan():
    assert forward_ray(10.0, ANGLE_INCREMENT, 1080) == 1079
    assert forward_ray(-10.0, ANGLE_INCREMENT, 1080) == 0


def test_danger_zone_spans_the_cone_the_car_width_subtends():
    ranges = np.full(100, 10.0)
    # arctan2(0.7, 10.0) / 0.01 -> 6 rays either side of ray 50
    ranges[44] = 2.0
    ranges[43] = 0.5

    assert danger_zone_min_range(ranges, 50, 0.01) == 2.0


def test_danger_zone_widens_as_the_obstacle_gets_closer():
    near = np.full(200, 1.0)
    far = np.full(200, 10.0)

    near_rays = int(np.arctan2(0.7, 1.0) / 0.01)
    far_rays = int(np.arctan2(0.7, 10.0) / 0.01)

    assert near_rays > far_rays
    assert danger_zone_min_range(near, 100, 0.01) == 1.0
    assert danger_zone_min_range(far, 100, 0.01) == 10.0


def test_zero_width_zone_on_the_last_ray_reports_no_obstacle():
    # arctan2(0.7, 1000.0) / 0.1 rounds down to 0 rays, so the slice would be empty
    ranges = np.full(10, 1000.0)

    assert danger_zone_min_range(ranges, 9, 0.1) == float('inf')


def test_single_ray_scan_reports_no_obstacle():
    assert danger_zone_min_range(np.array([1000.0]), 0, 0.1) == float('inf')


def test_ttc_is_distance_over_speed():
    assert time_to_collision(10.0, 2.0) == 5.0


def test_ttc_is_infinite_at_a_standstill():
    assert time_to_collision(0.2, 0.0) == float('inf')
    assert time_to_collision(0.2, 0.005) == float('inf')


def test_ttc_is_finite_once_the_car_is_actually_rolling():
    assert np.isfinite(time_to_collision(0.2, 0.5))


def test_forward_clearance_only_looks_at_the_forward_sector():
    ranges = np.full(1080, 3.0)
    ranges[540] = 0.3
    ranges[400] = 0.1

    assert forward_min_range(ranges) == 0.3


def test_empty_scan_never_reads_as_clear():
    assert forward_min_range(np.array([])) == float('-inf')
