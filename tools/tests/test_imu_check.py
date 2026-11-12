"""L1 tests for tools/imu_check.py: the IMU health verdicts, the yaw-rate fit and the ICP yaw rate.

Synthetic data only; the 2026-11-12 bags carry no IMU topics (the driver was not polling), so
there is no real fixture until a bag recorded with vesc_imu:=true exists.
"""

from __future__ import annotations

import math

import imu_check
import numpy as np
from follower_replay.bag import ScanRecord
from safety_replay.motion import Mount

RNG = np.random.default_rng(7)


def test_no_messages_is_silent():
    out = imu_check.imu_health(np.zeros(0), np.zeros((0, 3)), np.zeros((0, 3)))
    assert out["verdict"] == "silent"


def test_all_zero_stream_is_zeros_not_alive():
    t = np.arange(500) * 0.02
    out = imu_check.imu_health(t, np.zeros((500, 3)), np.zeros((500, 3)))
    assert out["verdict"] == "zeros"
    assert out["rate_hz"] == 50.0 or abs(out["rate_hz"] - 50.0) < 1e-6


def test_alive_stream_reports_rate_bias_and_noise_at_standstill():
    n = 1000
    t = np.arange(n) * 0.02
    gyro = RNG.normal(0.0, 0.01, (n, 3)) + np.array([0.0, 0.0, 0.02])  # rad/s, bias 0.02 on z
    accel = np.tile([0.0, 0.0, 9.81], (n, 1)) + RNG.normal(0, 0.05, (n, 3))
    out = imu_check.imu_health(t, gyro, accel, np.zeros(n))
    assert out["verdict"] == "alive"
    assert abs(out["rate_hz"] - 50.0) < 0.5
    assert abs(out["standstill_gyro_bias"][2] - 0.02) < 0.002
    assert abs(out["standstill_gyro_noise"][2] - 0.01) < 0.002
    assert abs(out["accel_norm_mean"] - 9.81) < 0.05


def test_moving_samples_are_not_counted_as_standstill():
    n = 400
    t = np.arange(n) * 0.02
    gyro = RNG.normal(0.0, 0.01, (n, 3))
    out = imu_check.imu_health(t, gyro, np.ones((n, 3)), np.full(n, 1.0))
    assert "standstill_gyro_bias" not in out


def _series(scale: float, lag_s: float = 0.0):
    """A 50 Hz yaw rate, and a reference at 10 Hz that is scale x it (delayed by lag_s)."""
    t = np.arange(0, 60, 0.02)
    rate = 0.8 * np.sin(2 * math.pi * t / 7.0) + 0.4 * np.sin(2 * math.pi * t / 2.3)
    ref_t = np.arange(1, 59, 0.1)
    ref = scale * np.interp(ref_t - lag_s, t, rate)
    return t, rate, ref_t, ref


def test_rate_fit_recovers_scale_and_sign():
    t, rate, ref_t, ref = _series(0.64)
    out = imu_check.rate_fit(t, rate, ref_t, ref)
    assert abs(out["scale"] - 0.64) < 0.03
    assert out["corr"] > 0.99
    assert out["best_lag"] == 0
    t, rate, ref_t, ref = _series(-1.0)  # board mounted upside down
    assert imu_check.rate_fit(t, rate, ref_t, ref)["scale"] < -0.9


def test_rate_fit_finds_a_delay_of_two_scan_periods():
    t, rate, ref_t, ref = _series(1.0, lag_s=0.2)
    out = imu_check.rate_fit(t, rate, ref_t, ref)
    assert out["best_lag"] == 2


def test_rate_fit_of_a_flat_series_is_nan_not_an_error():
    t, _, ref_t, ref = _series(1.0)
    out = imu_check.rate_fit(t, np.zeros_like(t), ref_t, ref)
    assert math.isnan(out["scale"])


def _room_scan(x: float, y: float, th: float, stamp: float) -> ScanRecord:
    """Ray-cast a 6 x 4 m room with a pillar, from the LiDAR on a car at (x, y, th)."""
    mount = Mount(0.285, 0.0, math.pi)
    lx = x + mount.x_m * math.cos(th)
    ly = y + mount.x_m * math.sin(th)
    angles = -math.pi + np.arange(720) * (2 * math.pi / 720)
    world = th + mount.yaw_rad + angles
    ranges = np.full(720, np.inf)
    for k, a in enumerate(world):
        dx, dy = math.cos(a), math.sin(a)
        best = np.inf
        for lo, hi, d, c in ((-3.0, 3.0, dx, lx), (-2.0, 2.0, dy, ly)):
            for wall in (lo, hi):
                if abs(d) > 1e-9:
                    s = (wall - c) / d
                    if s > 0:
                        best = min(best, s)
        # a 0.5 m square pillar at (1.0, 0.5): breaks the symmetry so the ICP is well posed
        for wall_x in (0.75, 1.25):
            if abs(dx) > 1e-9:
                s = (wall_x - lx) / dx
                if s > 0 and 0.25 <= ly + s * dy <= 0.75:
                    best = min(best, s)
        for wall_y in (0.25, 0.75):
            if abs(dy) > 1e-9:
                s = (wall_y - ly) / dy
                if s > 0 and 0.75 <= lx + s * dx <= 1.25:
                    best = min(best, s)
        ranges[k] = best
    return ScanRecord(
        stamp_s=stamp,
        angle_min=-math.pi,
        angle_increment=2 * math.pi / 720,
        range_min=0.05,
        range_max=12.0,
        ranges=tuple(float(r) for r in ranges),
    )


def test_icp_yaw_rate_matches_a_known_turn():
    mount = Mount(0.285, 0.0, math.pi)
    omega, v = 0.5, 0.4  # rad/s left turn, m/s
    scans = []
    x = y = th = 0.0
    for k in range(8):
        scans.append(_room_scan(x, y, th, 0.1 * k))
        x += v * math.cos(th) * 0.1
        y += v * math.sin(th) * 0.1
        th += omega * 0.1
    t, w, vel = imu_check.icp_yaw_rates(scans, mount)
    assert len(t) == 7
    assert abs(float(np.median(w)) - omega) < 0.05
    assert abs(float(np.median(vel)) - v) < 0.1
