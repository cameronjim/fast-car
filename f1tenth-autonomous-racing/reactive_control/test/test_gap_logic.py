"""unit tests for the pure gap-following math, no ros."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reactive_control.gap_logic import (  # noqa: E402
    corner_blocked,
    extend_disparities,
    select_target_ray,
)

ANGLE_INCREMENT = 0.1
HALF_WIDTH_M = 0.5


def test_disparity_extension_inflates_away_from_the_near_edge():
    ranges = np.array([1.0, 1.0, 1.0, 5.0, 5.0, 5.0, 5.0, 5.0])

    safe = extend_disparities(ranges, 1.0, HALF_WIDTH_M, ANGLE_INCREMENT)

    # arctan2(0.5, 1.0) / 0.1 -> 4 rays, walked forward from index 3
    assert list(safe) == [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 5.0]


def test_disparity_extension_walks_backwards_when_the_near_edge_is_on_the_right():
    ranges = np.array([5.0, 5.0, 5.0, 5.0, 5.0, 1.0, 1.0, 1.0])

    safe = extend_disparities(ranges, 1.0, HALF_WIDTH_M, ANGLE_INCREMENT)

    assert list(safe) == [5.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]


def test_no_disparity_leaves_the_scan_alone():
    ranges = np.array([2.0, 2.1, 2.2, 2.1, 2.0])

    safe = extend_disparities(ranges, 1.0, HALF_WIDTH_M, ANGLE_INCREMENT)

    assert np.array_equal(safe, ranges)


def test_disparity_extension_does_not_mutate_its_input():
    ranges = np.array([1.0, 1.0, 1.0, 5.0, 5.0, 5.0, 5.0, 5.0])
    original = ranges.copy()

    extend_disparities(ranges, 1.0, HALF_WIDTH_M, ANGLE_INCREMENT)

    assert np.array_equal(ranges, original)


def test_target_is_the_center_of_the_widest_gap():
    ranges = np.zeros(12)
    # cone is ranges[3:9]; the only wide free run is cone indices 2..4
    ranges[3] = 2.0
    ranges[5:8] = 2.0

    assert select_target_ray(ranges, 1.0, 0.25, 0.75) == 6


def test_equal_gaps_tie_break_toward_straight_ahead():
    ranges = np.zeros(12)
    # two two-ray gaps in the cone, the second one sits nearer the center ray
    ranges[3:5] = 2.0
    ranges[7:9] = 2.0

    assert select_target_ray(ranges, 1.0, 0.25, 0.75) == 7


def test_fully_blocked_cone_aims_straight_ahead():
    ranges = np.zeros(12)

    assert select_target_ray(ranges, 1.0, 0.25, 0.75) == 6


def test_corner_blocked_checks_the_sector_the_car_turns_into():
    ranges = np.full(12, 5.0)
    ranges[10:] = 0.1

    assert corner_blocked(ranges, -0.3, 0.2) is True
    assert corner_blocked(ranges, 0.3, 0.2) is False


def test_corner_clear_when_any_ray_in_the_sector_is_open():
    ranges = np.full(12, 0.1)
    ranges[11] = 5.0

    assert corner_blocked(ranges, -0.3, 0.2) is False
