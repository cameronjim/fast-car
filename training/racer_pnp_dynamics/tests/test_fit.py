"""L1 for fit.py: recover KNOWN parameters from synthetic data with measurement noise."""

from __future__ import annotations

import numpy as np
import pytest
from racer_pnp_dynamics import fit
from racer_pnp_dynamics.dataset import make_segments
from racer_pnp_dynamics.model import Nominal
from synth import make_run

TRUTH = Nominal(
    speed_gain=0.95,
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


def test_fit_recovers_known_parameters(toy_vehicle):
    run, motion = make_run(TRUTH, toy_vehicle, seconds=900.0, seed=1)
    segments = make_segments(run, motion)
    result = fit.fit_nominal(segments, toy_vehicle)
    got, want = result.nominal.to_dict(), TRUTH.to_dict()
    for name in (*fit.SPEED_PARAMS, *fit.YAW_PARAMS):
        assert got[name] == pytest.approx(want[name], rel=0.10), (name, got[name], want[name])
    assert abs(got["command_delay_s"] - want["command_delay_s"]) <= 0.02 + 1e-9
    assert result.speed_val_rmse_mps < 0.03 and result.yaw_val_rmse_rad < 0.01
    assert set(result.delay_curve) == set(fit.DELAY_GRID_S)
    assert result.counts["yaw_left_train"] > 100 and result.counts["yaw_right_train"] > 100


def test_fit_beats_the_placeholder_on_validation(toy_vehicle):
    run, motion = make_run(TRUTH, toy_vehicle, seconds=600.0, seed=2)
    segments = make_segments(run, motion)
    result = fit.fit_nominal(segments, toy_vehicle, delays=(0.08, 0.10, 0.12))
    from racer_pnp_dynamics.dataset import by_split
    from racer_pnp_dynamics.model import PLACEHOLDER_NOMINAL
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


def test_unidentified_and_bound_parameters_are_flagged(toy_vehicle):
    """With only left turns the right gain has zero sensitivity; flags say so."""
    run, motion = make_run(TRUTH, toy_vehicle, seconds=600.0, seed=5)
    run.steering_cmd_rad[:] = np.abs(run.steering_cmd_rad)  # left only (inconsistent, fine here)
    result = fit.fit_nominal(make_segments(run, motion), toy_vehicle, delays=(0.10,))
    assert "steer_gain_right" in result.flags
    assert result.nominal.steer_gain_right == pytest.approx(1.0)
