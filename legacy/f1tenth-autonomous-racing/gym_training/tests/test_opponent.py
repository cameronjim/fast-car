# unit tests for the scripted gap-follower opponent, no simulator

import numpy as np
import pytest

from f1rl.envs.opponent import (
    GapFollowerConfig,
    GapFollowerOpponent,
    aim_clearance_m,
    corner_blocked,
    extend_disparities,
    select_target_ray,
    target_bearing_rad,
)

ANGLE_INCREMENT = 0.1
HALF_WIDTH_M = 0.5

# the training sweep: 108 beams over 270 deg, the resolution the opponent actually runs at
NUM_BEAMS = 108
BEAM_ANGLE_MIN = -0.75 * np.pi
BEAM_ANGLE_INCREMENT = 1.5 * np.pi / (NUM_BEAMS - 1)


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
    ranges[3:5] = 2.0
    ranges[7:9] = 2.0

    assert select_target_ray(ranges, 1.0, 0.25, 0.75) == 7


def test_fully_blocked_cone_aims_straight_ahead():
    assert select_target_ray(np.zeros(12), 1.0, 0.25, 0.75) == 6


def test_corner_blocked_checks_the_sector_the_car_turns_into():
    ranges = np.full(12, 5.0)
    ranges[10:] = 0.1

    assert corner_blocked(ranges, -0.3, 0.2) is True
    assert corner_blocked(ranges, 0.3, 0.2) is False


def test_corner_clear_when_any_ray_in_the_sector_is_open():
    ranges = np.full(12, 0.1)
    ranges[11] = 5.0

    assert corner_blocked(ranges, -0.3, 0.2) is False


def test_aim_clearance_reads_the_cone_around_the_aimed_ray():
    ranges = np.full(21, 5.0)
    ranges[12] = 1.5

    # arctan2(0.5, 5.0) / 0.1 -> 0 rays, so a near beam one ray over is outside the cone
    assert aim_clearance_m(ranges, 10, HALF_WIDTH_M, ANGLE_INCREMENT) == pytest.approx(5.0)
    assert aim_clearance_m(ranges, 12, HALF_WIDTH_M, ANGLE_INCREMENT) == pytest.approx(1.5)


def test_bearing_comes_from_the_scan_geometry_not_the_index_center():
    # an even beam count has no ray dead ahead, so index arithmetic would bias the aim
    assert target_bearing_rad(0, BEAM_ANGLE_MIN, BEAM_ANGLE_INCREMENT) == pytest.approx(
        BEAM_ANGLE_MIN
    )
    assert target_bearing_rad(
        NUM_BEAMS - 1, BEAM_ANGLE_MIN, BEAM_ANGLE_INCREMENT
    ) == pytest.approx(-BEAM_ANGLE_MIN)
    assert abs(target_bearing_rad(54, BEAM_ANGLE_MIN, BEAM_ANGLE_INCREMENT)) < 0.5 * (
        BEAM_ANGLE_INCREMENT + 1e-9
    )


def opponent(**changes) -> GapFollowerOpponent:
    return GapFollowerOpponent(
        angle_min_rad=BEAM_ANGLE_MIN,
        angle_increment_rad=BEAM_ANGLE_INCREMENT,
        control_period_sec=0.01,
        config=GapFollowerConfig(**changes),
    )


def open_scan() -> np.ndarray:
    return np.full(NUM_BEAMS, 10.0)


def test_open_track_ahead_holds_the_wheel_near_straight_at_the_cap():
    driver = opponent()
    steering, speed = driver.plan(open_scan())

    # the target is an integer ray, so the aim can sit half a beam off dead ahead
    assert abs(steering) <= driver.config.steer_gain * BEAM_ANGLE_INCREMENT
    assert speed == pytest.approx(driver.config.speed_cap_mps)


def test_a_wall_on_the_right_steers_left():
    ranges = np.full(NUM_BEAMS, 3.5)
    ranges[:40] = 0.8

    steering, _ = opponent().plan(ranges)

    assert steering > 0.0


def test_a_wall_on_the_left_steers_right():
    ranges = np.full(NUM_BEAMS, 3.5)
    ranges[68:] = 0.8

    steering, _ = opponent().plan(ranges)

    assert steering < 0.0


def test_steering_is_clipped_to_the_vehicle_limit():
    ranges = np.full(NUM_BEAMS, 3.5)
    ranges[:40] = 0.8

    steering, _ = opponent(steer_gain=50.0).plan(ranges)

    assert steering == pytest.approx(GapFollowerConfig().steer_max_rad)


def test_a_close_obstacle_ahead_slows_the_car_toward_the_floor():
    driver = opponent()
    open_speed = driver.plan(np.full(NUM_BEAMS, 3.5))[1]
    driver.reset()
    blocked_speed = driver.plan(np.full(NUM_BEAMS, 0.7))[1]

    assert blocked_speed < open_speed
    assert blocked_speed >= driver.config.speed_min_mps


def test_speed_never_leaves_the_configured_band():
    driver = opponent()
    rng = np.random.default_rng(0)
    for _ in range(50):
        _, speed = driver.plan(rng.uniform(0.0, 6.0, size=NUM_BEAMS))
        assert driver.config.speed_min_mps <= speed <= driver.config.speed_cap_mps


def test_non_finite_beams_read_as_max_range():
    ranges = np.full(NUM_BEAMS, np.inf)
    ranges[5] = np.nan

    steering, speed = opponent().plan(ranges)

    assert np.isfinite(steering) and np.isfinite(speed)
    assert speed == pytest.approx(GapFollowerConfig().speed_cap_mps)


def test_the_speed_cap_is_mutable_per_episode():
    driver = opponent()
    driver.set_speed_cap(4.25)

    assert driver.plan(open_scan())[1] == pytest.approx(4.25)


def test_reset_clears_the_derivative_term():
    ranges = np.full(NUM_BEAMS, 3.5)
    ranges[:40] = 0.8
    driver = opponent()

    first = driver.plan(ranges)[0]
    driver.reset()

    assert driver.plan(ranges)[0] == pytest.approx(first)


def test_config_rejects_keys_it_would_otherwise_ignore():
    with pytest.raises(ValueError, match="unknown opponent config keys"):
        GapFollowerConfig.from_dict({"speed_cap_mps": 4.0, "speed_capp_mps": 4.0})


def test_config_rejects_an_inverted_speed_band():
    with pytest.raises(ValueError, match="speed_cap_mps"):
        GapFollowerConfig(speed_cap_mps=0.3, speed_min_mps=0.4)
