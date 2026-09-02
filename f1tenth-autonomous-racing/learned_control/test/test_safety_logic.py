"""unit tests for the pure learned-control safety math, no ros."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "nodes"))

from safety_logic import (  # noqa: E402
    danger_zone_min_range,
    forward_min_range,
    forward_ray,
    time_to_collision,
    wall_steer_bias,
)

ANGLE_INCREMENT = np.radians(0.25)


def test_straight_ahead_maps_to_the_middle_ray():
    assert forward_ray(0.0, ANGLE_INCREMENT, 1080) == 540


def test_extreme_steering_clamps_inside_the_scan():
    assert forward_ray(10.0, ANGLE_INCREMENT, 1080) == 1079
    assert forward_ray(-10.0, ANGLE_INCREMENT, 1080) == 0


def test_danger_zone_spans_the_cone_the_car_width_subtends():
    ranges = np.full(100, 10.0)
    # arctan2(0.7, 10.0) / 0.01 -> 6 rays either side of ray 50
    ranges[44] = 2.0
    ranges[43] = 0.5

    assert danger_zone_min_range(ranges, 50, 0.01) == 2.0


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


def test_forward_clearance_only_looks_at_the_forward_sector():
    ranges = np.full(1080, 3.0)
    ranges[540] = 0.3
    ranges[400] = 0.1

    assert forward_min_range(ranges) == 0.3


def test_empty_scan_never_reads_as_clear():
    assert forward_min_range(np.array([])) == float('-inf')


def test_open_corridor_produces_no_steering_bias():
    ranges = np.full(800, 5.0)

    assert wall_steer_bias(ranges, 0.7, 0.35, 0.18) == 0.0


def test_close_right_wall_pushes_the_steering_left():
    ranges = np.full(800, 5.0)
    ranges[150] = 0.2

    assert wall_steer_bias(ranges, 0.7, 0.35, 0.18) > 0.0


def test_close_left_wall_pushes_the_steering_right():
    ranges = np.full(800, 5.0)
    ranges[650] = 0.2

    assert wall_steer_bias(ranges, 0.7, 0.35, 0.18) < 0.0


def test_bias_is_clamped_to_the_configured_limit():
    ranges = np.full(800, 5.0)
    ranges[150] = 0.0

    assert wall_steer_bias(ranges, 0.7, 0.35, 0.1) == 0.1


def test_walls_equally_close_on_both_sides_cancel_out():
    ranges = np.full(800, 5.0)
    ranges[150] = 0.2
    ranges[650] = 0.2

    assert wall_steer_bias(ranges, 0.7, 0.35, 0.18) == 0.0
