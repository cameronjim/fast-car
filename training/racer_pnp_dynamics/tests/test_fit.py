"""L1 for fit.py: recover KNOWN parameters from synthetic data with measurement noise."""

from __future__ import annotations

import numpy as np
import pytest
from racer_pnp_dynamics import fit
from racer_pnp_dynamics.dataset import by_split, make_segments
from racer_pnp_dynamics.model import PLACEHOLDER_NOMINAL, Nominal
from synth import make_run

TRUTH = Nominal(
    speed_gain=0.95,
    speed_deadband_mps=0.0,
    speed_tau_s=0.30,
    accel_max_mps2=1.2,
    decel_max_mps2=0.8,
    steer_gain_left=1.10,
    steer_gain_right=0.90,
    steer_offset_rad=0.02,
    steer_tau_s=0.15,
    command_delay_s=0.10,
    understeer_s2_per_m2=0.5,
)


# With step commands between 0.15 and 0.6 m/s a limit of 1.2 m/s2 almost never binds in a rolling
# window, so the rolling rule (correctly) cannot tell it from the start value. A limit of 0.6
# binds on every large step, which is what "the rolling data show the limit" means.
RATE_LIMITED = TRUTH.replace(accel_max_mps2=0.6)


def test_fit_recovers_known_parameters(toy_vehicle):
    run, motion = make_run(RATE_LIMITED, toy_vehicle, seconds=900.0, seed=1)
    segments = make_segments(run, motion)
    result = fit.fit_nominal(segments, toy_vehicle)
    got, want = result.nominal.to_dict(), RATE_LIMITED.to_dict()
    for name in (*fit.SPEED_PARAMS, *fit.YAW_PARAMS):
        assert got[name] == pytest.approx(want[name], rel=0.10), (name, got[name], want[name])
    assert abs(got["command_delay_s"] - want["command_delay_s"]) <= 0.02 + 1e-9
    assert result.speed_val_rmse_mps < 0.03 and result.yaw_val_rmse_rad < 0.01
    assert set(result.delay_curve) == set(fit.DELAY_GRID_S)
    assert not result.sides_tied and result.counts["yaw_right_train"] >= fit.MIN_SIDE_INTERVALS
    assert "rolling" not in result.flags.get("accel_max_mps2", "")
    assert not result.accel_rule["kept_start"]


def test_accel_limit_is_fitted_when_rolling_data_show_it(toy_vehicle):
    run, motion = make_run(RATE_LIMITED, toy_vehicle, seconds=900.0, seed=7)
    result = fit.fit_nominal(make_segments(run, motion), toy_vehicle, delays=(0.10,))
    assert result.nominal.accel_max_mps2 == pytest.approx(0.6, rel=0.10)
    assert (
        "accel_max_mps2" not in result.flags
        or "kept at its start" not in (result.flags["accel_max_mps2"])
    )
    rule = result.accel_rule
    assert rule["rolling_windows"] >= fit.MIN_VAL_WINDOWS and not rule["kept_start"]
    assert rule["rmse_fitted"] < rule["rmse_start"]


def test_accel_limit_kept_at_start_when_only_launches_support_it(toy_vehicle):
    """Ramped commands (slower than the true limit) and a launch dead zone: the rolling windows
    cannot tell any limit above the ramp rate, so the start value ships and says why."""
    truth = TRUTH.replace(accel_max_mps2=PLACEHOLDER_NOMINAL.accel_max_mps2)
    run, motion = make_run(
        truth, toy_vehicle, seconds=900.0, seed=8, ramp_mps2=0.7, dead_zone_mps=0.25
    )
    result = fit.fit_nominal(make_segments(run, motion), toy_vehicle, delays=(0.10,))
    assert result.nominal.accel_max_mps2 == PLACEHOLDER_NOMINAL.accel_max_mps2
    assert "kept at its start" in result.flags["accel_max_mps2"]
    assert "rolling" in result.flags["accel_max_mps2"]
    assert result.accel_rule["kept_start"]
    assert result.accel_rule["rolling_windows"] >= fit.MIN_VAL_WINDOWS


def test_yaw_gain_is_path_geometry_when_wheels_over_read(toy_vehicle):
    gain = 0.9
    truth = TRUTH.replace(
        steer_gain_left=gain, steer_gain_right=gain, steer_offset_rad=0.0, understeer_s2_per_m2=0.0
    )
    run, motion = make_run(
        truth, toy_vehicle, seconds=900.0, seed=9, wheel_over_read=0.8, name="over-read"
    )
    segments = make_segments(run, motion)
    result = fit.fit_nominal(segments, toy_vehicle, delays=(0.10,))
    assert result.nominal.steer_gain_left == pytest.approx(gain, rel=0.10)
    assert result.nominal.steer_gain_right == pytest.approx(gain, rel=0.10)

    from racer_pnp_dynamics.windows import predict_yaw_change, yaw_data

    train = [s for s in by_split(segments, "train") if s.run.has_odom]
    data = yaw_data(train, result.nominal.command_delay_s)

    def wheel_fn(p: Nominal) -> np.ndarray:
        return predict_yaw_change(p, toy_vehicle, data, speed="wheel") - data.dyaw_rad

    wheel_fit = fit._fit_params(("steer_gain_left", "steer_gain_right"), result.nominal, wheel_fn)
    assert wheel_fit.steer_gain_left < 0.90 * gain
    assert wheel_fit.steer_gain_right < 0.90 * gain


def test_fit_beats_the_placeholder_on_validation(toy_vehicle):
    run, motion = make_run(TRUTH, toy_vehicle, seconds=600.0, seed=2)
    segments = make_segments(run, motion)
    result = fit.fit_nominal(segments, toy_vehicle, delays=(0.08, 0.10, 0.12))
    from racer_pnp_dynamics.windows import speed_windows

    val = by_split(segments, "val")
    base = fit.speed_rmse(speed_windows(val, 0.08), PLACEHOLDER_NOMINAL, toy_vehicle)
    assert result.speed_val_rmse_mps < base


def test_yaw_fit_works_without_odometry(toy_vehicle):
    """A bag with no /odom/wheel gets its wheel speed from the fitted speed model."""
    odom_run, odom_motion = make_run(TRUTH, toy_vehicle, seconds=600.0, seed=3, name="odom")
    bare_run, bare_motion = make_run(
        TRUTH, toy_vehicle, seconds=600.0, seed=4, odom=False, name="bare"
    )
    segments = [*make_segments(odom_run, odom_motion), *make_segments(bare_run, bare_motion)]
    result = fit.fit_nominal(segments, toy_vehicle, delays=(0.10,))
    assert result.nominal.steer_gain_left == pytest.approx(TRUTH.steer_gain_left, rel=0.15)
    assert result.nominal.steer_gain_right == pytest.approx(TRUTH.steer_gain_right, rel=0.15)
    assert result.counts["yaw_intervals_train"] > 3000
    assert np.isfinite(result.yaw_val_rmse_rad)


def test_one_sided_data_ties_the_gains(toy_vehicle):
    """With only left turns the right gain has zero sensitivity; flags say so."""
    run, motion = make_run(TRUTH, toy_vehicle, seconds=600.0, seed=5)
    run.steering_cmd_rad[:] = np.abs(run.steering_cmd_rad)  # left only (inconsistent, fine here)
    result = fit.fit_nominal(make_segments(run, motion), toy_vehicle, delays=(0.10,))
    assert "tied" in result.flags["steer_gain_right"] and result.sides_tied
    assert result.nominal.steer_gain_right == result.nominal.steer_gain_left
    assert result.nominal.steer_offset_rad == 0.0
    assert (
        result.alternative is not None and result.counts["yaw_right_train"] < fit.MIN_SIDE_INTERVALS
    )
