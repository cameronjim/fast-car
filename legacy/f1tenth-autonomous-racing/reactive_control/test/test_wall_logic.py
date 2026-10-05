"""unit tests for the pure wall-following geometry, no ros."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reactive_control.wall_logic import range_at_angle, wall_distance_error  # noqa: E402

# the 1080-ray 270 degree scan the sim publishes
NUM_RAYS = 1080
ANGLE_MIN = np.radians(-135.0)
ANGLE_INCREMENT = np.radians(0.25)
RANGE_MIN = 0.05
RANGE_MAX = 30.0


def parallel_wall_scan(distance_m):
    """scan of a car running parallel to a right-hand wall at distance_m."""
    ranges = np.full(NUM_RAYS, RANGE_MAX)
    ranges[180] = distance_m
    ranges[460] = distance_m / np.sin(np.radians(20.0))
    return ranges


def test_known_bearings_map_to_known_ray_indices():
    ranges = np.arange(NUM_RAYS, dtype=float)

    assert range_at_angle(ranges, -135.0, ANGLE_MIN, ANGLE_INCREMENT) == 0
    assert range_at_angle(ranges, -90.0, ANGLE_MIN, ANGLE_INCREMENT) == 180
    assert range_at_angle(ranges, -20.0, ANGLE_MIN, ANGLE_INCREMENT) == 460
    assert range_at_angle(ranges, 0.0, ANGLE_MIN, ANGLE_INCREMENT) == 540
    assert range_at_angle(ranges, 135.0, ANGLE_MIN, ANGLE_INCREMENT) == 1079


def test_bearings_outside_the_scan_clamp_to_the_end_rays():
    ranges = np.arange(NUM_RAYS, dtype=float)

    assert range_at_angle(ranges, -200.0, ANGLE_MIN, ANGLE_INCREMENT) == 0
    assert range_at_angle(ranges, 200.0, ANGLE_MIN, ANGLE_INCREMENT) == 1079


def test_parallel_wall_gives_the_plain_distance_error():
    ranges = parallel_wall_scan(1.0)

    error = wall_distance_error(ranges, RANGE_MIN, RANGE_MAX, ANGLE_MIN,
                                ANGLE_INCREMENT, 1.5, 0.0, 0.05)

    assert np.isclose(error, 0.5)


def test_error_inside_the_deadband_reads_as_zero():
    ranges = parallel_wall_scan(1.0)

    error = wall_distance_error(ranges, RANGE_MIN, RANGE_MAX, ANGLE_MIN,
                                ANGLE_INCREMENT, 1.01, 0.0, 0.05)

    assert error == 0.0


def test_speed_has_no_effect_when_the_car_runs_parallel():
    ranges = parallel_wall_scan(1.0)

    still = wall_distance_error(ranges, RANGE_MIN, RANGE_MAX, ANGLE_MIN,
                                ANGLE_INCREMENT, 1.5, 0.0, 0.05)
    moving = wall_distance_error(ranges, RANGE_MIN, RANGE_MAX, ANGLE_MIN,
                                 ANGLE_INCREMENT, 1.5, 3.0, 0.05)

    assert np.isclose(still, moving)


def test_lookahead_shrinks_the_error_when_the_car_turns_away_from_the_wall():
    ranges = np.full(NUM_RAYS, RANGE_MAX)
    ranges[180] = 1.0
    ranges[460] = 4.0

    still = wall_distance_error(ranges, RANGE_MIN, RANGE_MAX, ANGLE_MIN,
                                ANGLE_INCREMENT, 1.5, 0.0, 0.0)
    moving = wall_distance_error(ranges, RANGE_MIN, RANGE_MAX, ANGLE_MIN,
                                 ANGLE_INCREMENT, 1.5, 2.0, 0.5)

    assert moving < still


def test_out_of_range_rays_report_no_error():
    ranges = np.full(NUM_RAYS, RANGE_MAX)
    ranges[180] = RANGE_MAX + 5.0
    ranges[460] = 2.0

    error = wall_distance_error(ranges, RANGE_MIN, RANGE_MAX, ANGLE_MIN,
                                ANGLE_INCREMENT, 1.0, 1.0, 0.05)

    assert error == 0.0


def test_missing_velocity_is_treated_as_a_standstill():
    ranges = parallel_wall_scan(1.0)

    error = wall_distance_error(ranges, RANGE_MIN, RANGE_MAX, ANGLE_MIN,
                                ANGLE_INCREMENT, 1.5, None, 0.05)

    assert np.isclose(error, 0.5)
