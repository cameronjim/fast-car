"""L1/L2 tests for domain randomisation (racer_train.randomization)."""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st
from racer_gym.dynamics import tire
from racer_gym.dynamics.load_transfer import static_axle_loads
from racer_gym.params import build_dyn_params
from racer_train.randomization import (
    EpisodeDraw,
    RandomizationConfig,
    SensorNoise,
    apply_draw_to_dyn_params,
    sample_draw,
)

DT = 0.02


def _in(value: float, rng: tuple[float, float]) -> bool:
    return rng[0] - 1e-12 <= value <= rng[1] + 1e-12


@settings(max_examples=60, deadline=None)
@given(seed=st.integers(min_value=0, max_value=2**31 - 1))
def test_every_draw_lies_inside_its_configured_range(seed):
    cfg = RandomizationConfig(enabled=True)
    d = sample_draw(np.random.default_rng(seed), cfg, DT)
    assert _in(d.friction_mu, cfg.friction_mu)
    assert _in(d.cornering_stiffness_front_n_per_rad, cfg.cornering_stiffness_front_n_per_rad)
    ratio = d.cornering_stiffness_rear_n_per_rad / d.cornering_stiffness_front_n_per_rad
    assert _in(ratio, cfg.cornering_stiffness_rear_ratio)
    assert _in(d.mass_scale, cfg.mass_scale)
    assert _in(d.steering_limit_left_rad, cfg.steering_limit_left_rad)
    assert _in(d.steering_limit_right_rad, cfg.steering_limit_right_rad)
    assert _in(d.steering_gain, cfg.steering_gain)
    assert _in(d.steering_bias_rad, cfg.steering_bias_rad)
    assert _in(d.speed_scale, cfg.speed_scale)
    assert _in(d.top_speed_mps, cfg.top_speed_mps)
    assert _in(d.max_accel_mps2, cfg.max_accel_mps2)
    assert _in(d.steering_tau_s, cfg.steering_tau_s)
    assert _in(d.throttle_tau_s, cfg.throttle_tau_s)
    assert _in(d.command_delay_s, cfg.command_delay_s)
    assert cfg.lidar_period_steps[0] <= d.lidar_period_steps <= cfg.lidar_period_steps[1]
    assert 0 <= d.obs_latency_steps <= round(cfg.obs_latency_s[1] / DT)


def test_same_seed_same_draw_different_seed_different_draw():
    cfg = RandomizationConfig(enabled=True)
    a = sample_draw(np.random.default_rng(1), cfg, DT)
    b = sample_draw(np.random.default_rng(1), cfg, DT)
    c = sample_draw(np.random.default_rng(2), cfg, DT)
    assert a == b
    assert a != c


def test_stream_position_does_not_depend_on_range_values():
    """Widening one range must not shift later fields' random numbers (fixed rng call count)."""
    base = RandomizationConfig(enabled=True)
    wide = dataclasses.replace(base, friction_mu=(0.1, 2.0))
    a = sample_draw(np.random.default_rng(7), base, DT)
    b = sample_draw(np.random.default_rng(7), wide, DT)
    # A later field uses the same underlying uniform, so its position within its own range is
    # identical.
    ua = (a.command_delay_s - base.command_delay_s[0]) / (
        base.command_delay_s[1] - base.command_delay_s[0]
    )
    ub = (b.command_delay_s - wide.command_delay_s[0]) / (
        wide.command_delay_s[1] - wide.command_delay_s[0]
    )
    assert ua == pytest.approx(ub)


def test_nominal_collapses_ranges_and_removes_noise():
    cfg = RandomizationConfig(enabled=False).nominal()
    assert cfg.enabled
    d1 = sample_draw(np.random.default_rng(1), cfg, DT)
    d2 = sample_draw(np.random.default_rng(99), cfg, DT)
    assert d1 == d2
    assert d1.velocity_noise_std_mps == 0.0 and d1.pose_noise_std_m == 0.0
    assert d1.lidar_dropout_prob == 0.0 and d1.obs_latency_steps == 0
    assert d1.friction_mu == pytest.approx(0.5 * sum(RandomizationConfig().friction_mu))


def test_widened_scales_about_the_centre_and_keeps_nonnegatives_nonnegative():
    base = RandomizationConfig(enabled=True)
    wide = base.widened(2.0)
    lo, hi = base.friction_mu
    wlo, whi = wide.friction_mu
    assert (wlo + whi) / 2 == pytest.approx((lo + hi) / 2)
    assert (whi - wlo) == pytest.approx(2.0 * (hi - lo))
    assert wide.lidar_dropout_prob[0] >= 0.0
    assert wide.mass_scale[0] > 0.0
    assert wide.lidar_period_steps == base.lidar_period_steps


def test_apply_draw_sets_peak_force_and_small_slip_stiffness(real_vehicle_params):
    dyn = build_dyn_params(real_vehicle_params, DT).dyn_params
    d = EpisodeDraw(
        friction_mu=0.8,
        cornering_stiffness_front_n_per_rad=150.0,
        cornering_stiffness_rear_n_per_rad=180.0,
        mass_scale=1.1,
    )
    out = apply_draw_to_dyn_params(dyn, d, DT)
    static = static_axle_loads(out.mass_kg, out.cg_to_front_axle_m, out.cg_to_rear_axle_m)
    assert out.mass_kg == pytest.approx(dyn.mass_kg * 1.1)
    assert out.pacejka_front.d_peak_n == pytest.approx(0.8 * static.front_n)
    assert out.pacejka_rear.d_peak_n == pytest.approx(0.8 * static.rear_n)
    # Small-slip slope of the Magic Formula at the static load equals the requested stiffness.
    alpha = 1e-6
    slope_f = -tire.lateral_force(alpha, static.front_n, static.front_n, out.pacejka_front) / alpha
    slope_r = -tire.lateral_force(alpha, static.rear_n, static.rear_n, out.pacejka_rear) / alpha
    assert slope_f == pytest.approx(150.0, rel=1e-4)
    assert slope_r == pytest.approx(180.0, rel=1e-4)


def test_apply_draw_sets_steering_sides_delay_and_speed_limits(real_vehicle_params):
    dyn = build_dyn_params(real_vehicle_params, DT).dyn_params
    d = EpisodeDraw(
        steering_limit_left_rad=0.51,
        steering_limit_right_rad=0.44,
        command_delay_s=0.1,
        top_speed_mps=2.1,
        max_accel_mps2=2.5,
        steering_tau_s=0.07,
        throttle_tau_s=0.2,
    )
    out = apply_draw_to_dyn_params(dyn, d, DT)
    assert out.s_max == 0.51 and out.s_min == -0.44  # left positive
    assert out.delay_steps == 5
    assert out.v_max == 2.1 and out.a_max == 2.5
    assert out.steer_tau_s == 0.07 and out.throttle_tau_s == 0.2


def test_sensor_noise_is_deterministic_per_seed():
    d = EpisodeDraw(velocity_noise_std_mps=0.05, pose_noise_std_m=0.02, lidar_noise_std_m=0.01)
    n1 = SensorNoise(d, seed=3, dt_s=DT, n_beams=8)
    n2 = SensorNoise(d, seed=3, dt_s=DT, n_beams=8)
    for _ in range(20):
        a, b = n1.step(), n2.step()
        assert a["velocity_noise_mps"] == b["velocity_noise_mps"]
        assert np.array_equal(a["pose_error"], b["pose_error"])
        assert np.array_equal(a["lidar_noise_m"], b["lidar_noise_m"])


def test_sensor_noise_pose_error_is_bounded_in_distribution():
    d = EpisodeDraw(pose_noise_std_m=0.02, pose_yaw_noise_std_rad=0.03)
    noise = SensorNoise(d, seed=11, dt_s=DT, n_beams=4)
    errs = np.array([noise.step()["pose_error"] for _ in range(20000)])
    assert errs[:, 0].std() == pytest.approx(0.02, rel=0.25)
    assert errs[:, 2].std() == pytest.approx(0.03, rel=0.25)
    assert abs(errs[:, 0].mean()) < 0.01


def test_lidar_refresh_period_matches_the_draw():
    d = EpisodeDraw(lidar_period_steps=5)
    noise = SensorNoise(d, seed=1, dt_s=DT, n_beams=4)
    refresh = [noise.step()["lidar_refresh"] for _ in range(20)]
    assert refresh == [(i % 5 == 0) for i in range(20)]


def test_zero_noise_draw_produces_zero_noise():
    noise = SensorNoise(EpisodeDraw(), seed=1, dt_s=DT, n_beams=4)
    out = noise.step()
    assert np.all(out["pose_error"] == 0.0)
    assert out["velocity_noise_mps"] == 0.0
    assert not out["lidar_dropout"].any()
    assert math.isfinite(float(out["lidar_noise_m"].sum()))
