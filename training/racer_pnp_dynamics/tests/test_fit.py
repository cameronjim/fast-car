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
    assert result.speed_heldout_rmse_mps < 0.03 and result.yaw_heldout_rmse_rad < 0.01
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


def test_fit_beats_the_placeholder_on_held_out_data(toy_vehicle):
    run, motion = make_run(TRUTH, toy_vehicle, seconds=600.0, seed=2)
    segments = make_segments(run, motion)
    result = fit.fit_nominal(segments, toy_vehicle, delays=(0.08, 0.10, 0.12))
    from racer_pnp_dynamics.windows import speed_windows

    held = by_split(segments, "heldout")
    base = fit.speed_rmse(speed_windows(held, 0.08), PLACEHOLDER_NOMINAL, toy_vehicle)
    assert result.speed_heldout_rmse_mps < base


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
    assert np.isfinite(result.yaw_heldout_rmse_rad)


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


DEAD_ZONE_TRUTH = TRUTH.replace(speed_deadband_mps=0.20, accel_max_mps2=2.0, decel_max_mps2=1.5)


def test_fit_recovers_a_known_dead_zone_gain_lag_and_delay_from_data_with_launches(toy_vehicle):
    """Commands that ramp through the dead zone (0.5 m/s2, so the launch is pinned to a few
    hundredths) from a standstill, the car answering with the contract model."""
    run, motion = make_run(
        DEAD_ZONE_TRUTH,
        toy_vehicle,
        seconds=900.0,
        seed=21,
        ramp_mps2=0.5,
        speed_noise=0.002,  # the real wheel speed at rest reads exactly 0
        name="launches",
    )
    result = fit.fit_nominal(
        make_segments(run, motion), toy_vehicle, delays=(0.06, 0.10, 0.14), tau_grid=(0.1, 0.3, 0.6)
    )
    got = result.nominal
    assert result.deadband_choice["launches_used"] >= 5
    assert got.speed_deadband_mps == pytest.approx(0.20, abs=0.05)
    assert got.speed_gain == pytest.approx(0.95, rel=0.05)
    assert got.speed_tau_s == pytest.approx(0.30, rel=0.25)
    assert abs(got.command_delay_s - 0.10) <= 0.04 + 1e-9


def test_no_dead_zone_in_the_data_gives_a_small_estimate_from_ramped_launches(toy_vehicle):
    """The truth has no dead zone; commands ramp through 0 to 0.3 at 0.5 m/s2 and the wheels turn
    at once, so each launch command is just what the lag and delay add."""
    run, motion = make_run(
        TRUTH, toy_vehicle, seconds=700.0, seed=22, ramp_mps2=0.5, speed_noise=0.002, name="nodz"
    )
    result = fit.fit_nominal(make_segments(run, motion), toy_vehicle, delays=(0.10,))
    assert result.nominal.speed_deadband_mps < 0.16  # delay 0.1 s and the lag, at 0.5 m/s2
    assert result.deadband_choice["launches_used"] >= 5


def test_a_car_held_still_under_a_high_command_is_not_a_launch_observation():
    from racer_pnp_dynamics.bags import GRID_DT_S, Run

    n = 600
    cmd = np.zeros(n)
    cmd[100:] = 0.4
    wheel = np.zeros(n)
    wheel[250:] = 0.35
    t = 1000.0 + np.arange(n) * GRID_DT_S
    run = Run("held", t, cmd, np.zeros(n), wheel, [])
    out = fit.launch_deadband([run])
    assert out["launches"] == 1 and out["launches_used"] == 0 and out["value_mps"] == 0.0


def test_dead_zone_can_be_searched_on_the_held_out_error_instead(toy_vehicle):
    run, motion = make_run(
        DEAD_ZONE_TRUTH,
        toy_vehicle,
        seconds=900.0,
        seed=23,
        ramp_mps2=0.5,
        speed_noise=0.002,
        name="search",
    )
    grid = tuple(round(0.05 * k, 2) for k in range(9))
    result = fit.fit_nominal(
        make_segments(run, motion),
        toy_vehicle,
        delays=(0.10,),
        deadbands=grid,
        tau_grid=(0.1, 0.3),
        deadband=None,
    )
    assert result.nominal.speed_deadband_mps in (0.15, 0.2, 0.25)
    assert result.deadband_choice["rule"].startswith("argmin")


def test_a_delay_the_data_cannot_see_is_listed_and_ships_the_grid_minimum(toy_vehicle):
    """Commands that barely move: a delay of 0.1 s is below the noise, the delay profile is flat."""
    slow = TRUTH.replace(command_delay_s=0.10, speed_tau_s=0.05)
    run, motion = make_run(slow, toy_vehicle, seconds=900.0, seed=24, ramp_mps2=0.05, name="slow")
    delays = (0.0, 0.04, 0.08, 0.12, 0.16, 0.20)
    result = fit.fit_nominal(
        make_segments(run, motion), toy_vehicle, delays=delays, tau_grid=(0.05, 0.3), deadband=0.0
    )
    assert "command_delay_s" in result.unidentified
    assert result.nominal.command_delay_s == 0.0
    flat = [r["over_min"] for r in result.profiles["command_delay_s"]]
    assert sum(v <= 0.02 for v in flat) > len(flat) // 2


def test_every_profile_has_a_table_and_the_time_constants_are_profiled(toy_vehicle):
    run, motion = make_run(TRUTH, toy_vehicle, seconds=600.0, seed=25)
    result = fit.fit_nominal(
        make_segments(run, motion),
        toy_vehicle,
        delays=(0.10,),
        tau_grid=(0.05, 0.15, 0.3, 0.6),
        deadband=0.0,
    )
    for name in ("command_delay_s", "speed_tau_s", "steer_tau_s", "speed_deadband_mps"):
        rows = result.profiles[name]
        assert rows and all({"value", "rmse", "over_min"} <= set(r) for r in rows)
        assert min(r["over_min"] for r in rows) == 0.0
    taus = [r["value"] for r in result.profiles["speed_tau_s"]]
    assert taus == [0.05, 0.15, 0.3, 0.6]
    # the profile of the speed time constant has its minimum near the truth, 0.30
    best = min(result.profiles["speed_tau_s"], key=lambda r: r["rmse"])["value"]
    assert best in (0.15, 0.3)
