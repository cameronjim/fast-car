"""L1 for bags.py: unit conversion exactly once, resampling."""

from __future__ import annotations

import numpy as np
import pytest
from follower_replay.bag import DriveRecord, OdomRecord
from racer_pnp_dynamics import bags
from racer_pnp_dynamics.bags import BagData, calibration


def _bag(speed_recorded: float = 1.401, steer: float = 0.0, odom: bool = True) -> BagData:
    drive = [DriveRecord(10.0 + 0.02 * k, steer, speed_recorded) for k in range(100)]
    odoms = [OdomRecord(10.0 + 0.02 * k, speed_recorded) for k in range(100)] if odom else []
    return BagData(drive=drive, drive_raw=list(drive), odom=odoms)


def test_conversion_is_applied_exactly_once():
    run = bags.run_from_bag_data("b", _bag(1.401))
    assert run.speed_cmd_mps[10] == pytest.approx(1.0)
    assert run.wheel_speed_mps[10] == pytest.approx(1.0)
    assert run.has_odom


def test_a_bag_in_current_units_is_not_converted():
    run = bags.run_from_bag_data("b", _bag(1.0, steer=0.3), recorded=None)
    assert run.speed_cmd_mps[10] == 1.0 and run.wheel_speed_mps[10] == 1.0
    assert run.steering_cmd_rad[5] == 0.3


def test_units_are_decided_per_bag_and_unknown_new_bags_are_refused():
    from racer_pnp_dynamics import recordings

    for name in ("2026-11-12T03-04-56_car_teleop", "2026-11-12T02-57-50_car_teleop"):
        assert recordings.recorded_for(name) is None
    for name in ("2026-10-24T23-13-18_car_teleop", "2026-11-11T04-20-19_car_teleop_v3"):
        assert recordings.recorded_for(name) is calibration.RECORDED_0_13_0
    with pytest.raises(ValueError, match="CURRENT_UNITS_BAGS"):
        recordings.recorded_for("2026-11-13T01-00-00_car_teleop")


def test_input_bag_is_not_mutated():
    bag = _bag(1.401)
    bags.run_from_bag_data("b", bag)
    assert bag.drive[0].speed_mps == 1.401  # a second conversion would not be detectable otherwise


def test_steering_goes_through_the_calibration_table():
    current = calibration.load_current()
    run = bags.run_from_bag_data("b", _bag(1.401, steer=0.3))
    want = calibration.steering_rad(calibration.RECORDED_0_13_0, current, 0.3)
    assert run.steering_cmd_rad[5] == pytest.approx(want)
    assert want > 0.3  # left is about 1.2 times in true units


def test_run_without_odometry_has_nan_wheel_speed():
    run = bags.run_from_bag_data("old", _bag(odom=False))
    assert not run.has_odom and np.isnan(run.wheel_speed_mps).all()
    assert run.command_valid()[:50].all()


def test_empty_drive_is_an_error():
    with pytest.raises(ValueError):
        bags.run_from_bag_data("empty", BagData())


def test_resample_hold_marks_stale_commands():
    t = np.array([0.0, 0.1, 1.0])
    v = np.array([1.0, 2.0, 3.0])
    grid = np.array([-0.1, 0.0, 0.05, 0.12, 0.5, 1.0])
    out = bags.resample_hold(t, v, grid, 0.2)
    assert np.isnan(out[0]) and out[1] == 1.0 and out[2] == 1.0 and out[3] == 2.0
    assert np.isnan(out[4]) and out[5] == 3.0  # 0.4 s after the last message: a gap
    assert np.isnan(bags.resample_hold(np.zeros(0), np.zeros(0), grid, 0.2)).all()


def test_resample_linear_interpolates_and_blanks_gaps():
    t = np.array([0.0, 0.02, 0.04, 1.0])
    v = np.array([0.0, 1.0, 2.0, 3.0])
    out = bags.resample_linear(t, v, np.array([-0.01, 0.01, 0.03, 0.5, 1.01]))
    assert np.isnan(out[0]) and out[1] == pytest.approx(0.5) and out[2] == pytest.approx(1.5)
    assert np.isnan(out[3]) and np.isnan(out[4])
    assert np.isnan(bags.resample_linear(np.array([0.0]), np.array([1.0]), np.array([0.0]))).all()


def test_run_properties():
    run = bags.run_from_bag_data("b", _bag())
    assert run.duration_s == pytest.approx(1.96, abs=0.03)
