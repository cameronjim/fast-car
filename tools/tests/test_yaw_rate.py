"""Tests for tools/yaw_rate: the pose reference and jump filter, signal integration, closure noise,
delay, a synthetic end-to-end recovery of bias, RMS and delay, and the sign of the scan matcher
through the LiDAR mount (yaw pi on the car). Everything is seeded; nothing reads a bag except the
mcap reader test, which writes its own tiny bag.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest
from follower_replay.bag import ScanRecord
from safety_replay import motion
from yaw_rate import bagio, delay, matching, pipeline, reference, report, run, signals, vehicle

TESTS = Path(__file__).resolve().parent
REPO_ROOT = TESTS.parents[1]


# --- reference ----------------------------------------------------------------------------------


def test_pose_intervals_wrap_the_yaw_change() -> None:
    stamp = np.array([0.0, 0.5, 1.0])
    yaw = np.array([3.0, -3.0, -2.5])  # crosses +-pi between the first two
    iv = reference.pose_intervals(stamp, np.zeros(3), np.zeros(3), yaw)
    assert iv.dyaw[0] == pytest.approx(2 * math.pi - 6.0)
    assert iv.rate[1] == pytest.approx(1.0)


def test_pose_intervals_can_merge_to_a_minimum_length() -> None:
    stamp = np.arange(0.0, 2.01, 0.1)
    iv = reference.pose_intervals(stamp, stamp, 0 * stamp, 0.2 * stamp, min_interval_s=0.5)
    assert np.allclose(iv.dt, 0.5)
    assert np.allclose(iv.rate, 0.2)
    assert np.allclose(iv.chord, 0.5)


def test_jump_filter_drops_a_relocalisation_and_a_long_gap() -> None:
    stamp = np.array([0.0, 0.5, 1.0, 1.5, 3.5, 4.0])
    x = np.array([0.0, 0.5, 1.0, 3.0, 3.5, 4.0])  # a 1.5 m jump between 1.0 and 1.5 s
    iv = reference.pose_intervals(stamp, x, np.zeros(6), np.zeros(6))
    path = iv.dt * 1.0  # the wheels say 1 m/s
    flt = reference.filter_intervals(iv, path)
    assert flt.n_total == 5
    assert flt.n_jumped == 1
    assert flt.n_too_long == 1  # the 2 s gap
    assert flt.keep.tolist() == [True, True, False, False, True]


def test_jump_filter_does_not_mistake_a_tight_corner_for_a_jump() -> None:
    # 1.0 s at 2 m/s round a 0.83 m radius corner: arc 2 m, heading change 2.4 rad, chord 1.5 m.
    radius, arc = 0.83, 2.0
    dyaw = arc / radius
    chord = 2 * radius * math.sin(dyaw / 2)
    iv = reference.Intervals(
        np.array([0.0]), np.array([1.0]), np.array([reference.wrap(dyaw)[()]]), np.array([chord])
    )
    assert abs(chord - arc) > reference.JUMP_TOLERANCE_M  # the plain wheel distance would trip
    assert reference.filter_intervals(iv, np.array([arc])).keep.tolist() == [True]


# --- signals -------------------------------------------------------------------------------------


def test_rate_integral_matches_the_analytic_integral() -> None:
    t = np.arange(0.0, 10.0, 0.02)
    s = signals.RateSeries(t, np.sin(t))
    a, b = np.array([0.3]), np.array([2.1])
    assert s.integral(a, b)[0] == pytest.approx(math.cos(0.3) - math.cos(2.1), abs=1e-4)


def test_rate_integral_is_nan_over_a_hole_and_outside_the_samples() -> None:
    t = np.concatenate([np.arange(0.0, 1.0, 0.02), np.arange(2.0, 3.0, 0.02)])
    s = signals.RateSeries(t, np.ones_like(t))
    out = s.integral(np.array([0.2, 0.5, 2.1, -1.0]), np.array([0.8, 2.5, 2.6, 0.5]))
    assert np.isfinite(out[0]) and np.isfinite(out[2])
    assert np.isnan(out[1]) and np.isnan(out[3])


def test_step_integral_splits_a_step_uniformly_and_rejects_bad_steps() -> None:
    t0 = np.array([0.0, 0.1, 0.2, 0.3])
    s = signals.StepSeries(t0, t0 + 0.1, np.array([0.1, 0.2, np.nan, 0.4]))
    assert s.integral(np.array([0.0]), np.array([0.2]))[0] == pytest.approx(0.3)
    assert s.integral(np.array([0.05]), np.array([0.15]))[0] == pytest.approx(0.05 + 0.1)
    assert np.isnan(s.integral(np.array([0.1]), np.array([0.25]))[0])  # touches the bad step
    assert np.isnan(s.integral(np.array([0.3]), np.array([0.5]))[0])  # runs off the end


@pytest.mark.parametrize("seed", range(5))
def test_integrals_of_adjacent_intervals_add_up_to_the_union(seed: int) -> None:
    """Hypothesis: for any sampling the integral over [a, b] plus [b, c] is the integral over
    [a, c], for both kinds of signal."""
    rng = np.random.default_rng(seed)
    t = np.cumsum(rng.uniform(0.01, 0.1, 400))  # irregular rate samples (holes under 0.2 s)
    rate = signals.RateSeries(t, rng.normal(size=t.size))
    edges = np.sort(rng.uniform(t[0], t[-1], 3))
    t0 = np.arange(0.0, 20.0, 0.1)
    steps = signals.StepSeries(t0, t0 + 0.1, rng.normal(size=t0.size))
    for s, (a, b, c) in (
        (rate, edges),
        (steps, np.sort(rng.uniform(0.0, 19.9, 3))),
    ):
        ab = s.integral(np.array([a]), np.array([b]))[0]
        bc = s.integral(np.array([b]), np.array([c]))[0]
        ac = s.integral(np.array([a]), np.array([c]))[0]
        assert ab + bc == pytest.approx(ac, abs=1e-9)


def test_closure_recovers_a_known_per_step_noise() -> None:
    rng = np.random.default_rng(1)
    sigma = 0.004  # rad per 0.1 s step: 0.04 rad/s
    n = 20000
    truth = np.cumsum(rng.normal(size=n + 2) * 0.01)  # any yaw path
    e = rng.normal(0.0, sigma, size=(3, n))
    step = np.diff(truth)
    s1 = step[:n] + e[0]
    s2 = step[1 : n + 1] + e[1]
    direct = step[:n] + step[1 : n + 1] + e[2]
    resid = signals.closure_residual(s1, s2, direct)
    span = np.full(n, 0.2)
    rms = signals.closure_rate_rms(resid, span)
    assert rms == pytest.approx(math.sqrt(3) * sigma / 0.2, rel=0.03)
    assert signals.per_step_noise_rate(rms) == pytest.approx(sigma / 0.1, rel=0.03)


def test_closure_ignores_invalid_residuals() -> None:
    resid = np.array([0.02, np.nan, -0.02])
    assert signals.closure_rate_rms(resid, np.full(3, 0.2)) == pytest.approx(0.1)
    assert math.isnan(signals.closure_rate_rms(np.array([np.nan]), np.array([0.2])))


# --- delay ---------------------------------------------------------------------------------------


def test_lag_search_finds_a_known_delay_to_better_than_a_sample() -> None:
    rng = np.random.default_rng(3)
    t = np.arange(0.0, 200.0, 0.02)
    # A band-limited yaw rate with content around 0.5 to 2 Hz.
    truth = sum(
        rng.uniform(0.2, 0.6) * np.sin(2 * math.pi * f * t + rng.uniform(0, 6.28))
        for f in rng.uniform(0.3, 2.0, 12)
    )
    d = 0.137
    delayed = signals.RateSeries(t, np.interp(t - d, t, truth))
    a = np.arange(5.0, 190.0, 0.5)
    b = a + 0.5
    ref = signals.RateSeries(t, truth).mean(a, b)
    res = delay.interval_lag(a, b, ref, delayed)
    assert res.lag_s == pytest.approx(d, abs=0.02)
    assert res.peak_corr > 0.98
    assert res.drop_at_0p1 > 0.0


def test_lag_search_without_enough_intervals_is_nan() -> None:
    s = signals.RateSeries(np.arange(0, 10, 0.1), np.ones(100))
    res = delay.interval_lag(np.array([1.0]), np.array([2.0]), np.array([1.0]), s)
    assert math.isnan(res.lag_s)


# --- the pipeline on a synthetic drive -----------------------------------------------------------


POSE_NOISE_RAD = math.radians(0.5)


def synthetic_segment(
    bias: float,
    noise_step_rad: float,
    lag_s: float,
    pose_dt: float,
    seed: int,
    jump: bool = False,
    pose_noise_rad: float = POSE_NOISE_RAD,
) -> pipeline.SegmentInput:
    """A 240 s drive: a known yaw rate profile, the ICP signal sampled at 10 Hz with a bias,
    per-step noise and a delay; the wheel signal is the truth with a different bias; poses at
    `pose_dt` with 0.5 degree noise."""
    rng = np.random.default_rng(seed)
    t = np.arange(0.0, 240.0, 0.01)
    truth = sum(
        rng.uniform(0.1, 0.4) * np.sin(2 * math.pi * f * t + rng.uniform(0, 6.28))
        for f in rng.uniform(0.1, 0.8, 10)
    )
    yaw = np.concatenate([[0.0], np.cumsum(0.5 * (truth[1:] + truth[:-1]) * 0.01)])
    speed = np.full_like(t, 1.2)
    # Poses.
    pt = np.arange(0.0, 240.0, pose_dt)
    pose_yaw = np.interp(pt, t, yaw) + rng.normal(0.0, pose_noise_rad, pt.size)
    px = 1.2 * pt  # a straight track for the position: the chord check sees the wheel speed
    if jump:
        px[len(px) // 2 :] += 2.0  # a relocalisation half way
    # ICP steps: yaw change over [k, k+1] 0.1 s of the delayed truth, plus bias and noise.
    t0 = np.arange(0.0, 239.8, 0.1)
    delayed = np.interp(t - lag_s, t, yaw)
    dyaw = np.interp(t0 + 0.1, t, delayed) - np.interp(t0, t, delayed)
    steps = dyaw + bias * 0.1 + rng.normal(0.0, noise_step_rad, t0.size)
    steps2 = (
        np.interp(t0[:-1] + 0.2, t, delayed)
        - np.interp(t0[:-1], t, delayed)
        + bias * 0.2
        + rng.normal(0.0, noise_step_rad, t0.size - 1)
    )
    icp = pipeline.IcpInput(t0, t0 + 0.1, steps, np.ones(t0.size, bool), steps2)
    return pipeline.SegmentInput(
        name="syn",
        direction="ccw",
        pose_stamp=pt,
        pose_x=px,
        pose_y=np.zeros_like(pt),
        pose_yaw=pose_yaw,
        wheel_t=t,
        wheel_vx=speed,
        wheel_wz=truth - 0.03,
        steer_t=t,
        steer_wz=truth,
        icp={"icp": icp},
    )


def test_pipeline_recovers_bias_and_rms() -> None:
    """No delay: bias within 0.01 rad/s and RMS within 10 percent of what the injected bias,
    step noise (10 steps per 1 s interval) and pose noise (two poses per interval) predict."""
    bias, sigma_step, sigma_pose = 0.03, 0.002, math.radians(0.5)
    seg = synthetic_segment(bias, sigma_step, 0.0, 0.1, seed=7, pose_noise_rad=sigma_pose)
    res = pipeline.evaluate_segment(seg, min_interval_s=1.0)
    summary = pipeline.summarise([res], directions=("all",))
    icp = summary["signals"]["icp"]["all"]["0.85+"]
    ref_noise_sq = 2 * sigma_pose**2  # per second of interval
    assert icp["bias"] == pytest.approx(bias, abs=0.01)
    assert icp["rms"] == pytest.approx(
        math.sqrt(bias**2 + 10 * sigma_step**2 + ref_noise_sq), rel=0.1
    )
    # The wheel signal was built with a -0.03 rad/s bias and no noise of its own.
    wheel = summary["signals"]["wheel"]["all"]["0.85+"]
    assert wheel["bias"] == pytest.approx(-0.03, abs=0.01)
    assert wheel["rms"] == pytest.approx(math.sqrt(0.03**2 + ref_noise_sq), rel=0.1)
    # Closure noise: sqrt(3) sigma over 0.2 s.
    assert icp["closure_rms_rad_per_s"] == pytest.approx(math.sqrt(3) * sigma_step / 0.2, rel=0.1)


def test_pipeline_recovers_the_delay_of_each_signal() -> None:
    lag = 0.08
    seg = synthetic_segment(0.03, 0.002, lag, 0.1, seed=7)
    summary = pipeline.summarise([pipeline.evaluate_segment(seg)], directions=("all",))
    icp = summary["signals"]["icp"]["all"]["0.85+"]["delay_vs_pose"]
    wheel = summary["signals"]["wheel"]["all"]["0.85+"]["delay_vs_pose"]
    assert icp["lag_s"] == pytest.approx(lag, abs=0.02)
    assert wheel["lag_s"] == pytest.approx(0.0, abs=0.02)


def test_delay_against_the_steering_model_when_the_pose_is_too_slow() -> None:
    """A pose stream at 0.5 s does not resolve 10 Hz yaw: the lag is taken against the steering
    model instead (and the pose lag is not measured)."""
    lag = 0.08
    seg = synthetic_segment(0.0, 0.0005, lag, 0.5, seed=9)
    summary = pipeline.summarise([pipeline.evaluate_segment(seg)], directions=("all",))
    row = summary["signals"]["icp"]["all"]["0.85+"]
    assert math.isnan(row["delay_vs_pose"]["lag_s"])
    assert row["delay_vs_steering_model"]["lag_s"] == pytest.approx(lag, abs=0.02)


def test_rms_matches_the_injected_noise_within_ten_percent() -> None:
    """No delay and exact poses: the RMS error is the bias and the window-averaged step noise
    (5 steps per 0.5 s interval), within 10 percent."""
    seg = synthetic_segment(0.02, 0.003, 0.0, 0.5, seed=11)
    seg = pipeline.SegmentInput(**{**seg.__dict__, "pose_yaw": _exact_pose_yaw(seg)})
    res = pipeline.evaluate_segment(seg)
    e = res.err["icp"][np.isfinite(res.err["icp"])]
    rms = float(np.sqrt(np.mean(e**2)))
    assert rms == pytest.approx(math.sqrt(0.02**2 + 0.003**2 * 5 / 0.5**2), rel=0.10)
    assert float(e.mean()) == pytest.approx(0.02, abs=0.01)


def _exact_pose_yaw(seg: pipeline.SegmentInput) -> np.ndarray:
    """Pose yaw without noise: the integral of the steering model (the truth)."""
    t, w = seg.steer_t, seg.steer_wz
    cum = np.concatenate([[0.0], np.cumsum(0.5 * (w[1:] + w[:-1]) * np.diff(t))])
    return np.interp(seg.pose_stamp, t, cum)


def test_pipeline_drops_an_injected_relocalisation() -> None:
    seg = synthetic_segment(0.0, 0.002, 0.0, 0.5, seed=5, jump=True)
    res = pipeline.evaluate_segment(seg)
    assert res.n_jumped == 1
    assert res.n_too_long == 0
    assert res.n_total == len(seg.pose_stamp) - 1
    assert len(res.iv) == res.n_total - 1


def test_a_rejected_step_shows_up_as_a_rejected_interval_and_a_run() -> None:
    seg = synthetic_segment(0.0, 0.002, 0.0, 0.5, seed=5)
    icp = seg.icp["icp"]
    yaw = icp.yaw.copy()
    yaw[1000:1008] = np.nan  # 0.8 s of rejected matches
    seg = pipeline.SegmentInput(
        **{
            **seg.__dict__,
            "icp": {"icp": pipeline.IcpInput(icp.t0, icp.t1, yaw, icp.computed, icp.yaw2)},
        }
    )
    res = pipeline.evaluate_segment(seg)
    row = pipeline.summarise([res], directions=("all",))["signals"]["icp"]["all"]["0.85+"]
    assert row["longest_rejection_run_s"] == pytest.approx(0.8)
    assert 0.0 < row["rejected_share"] < 0.02


def test_verdict_applies_the_three_pre_registered_limits() -> None:
    good = {
        "rms": 0.05,
        "closure_rms_rad_per_s": 0.04,
        "delay_vs_pose": {"lag_s": 0.08},
    }
    assert pipeline.verdict(good, "icp")["label"] == "usable for adaptation"
    for key, bad in (("rms", 0.2), ("closure_rms_rad_per_s", 0.3)):
        row = {**good, key: bad}
        assert pipeline.verdict(row, "icp")["label"] == "not usable for adaptation"
    row = {**good, "delay_vs_pose": {"lag_s": 0.2}}
    assert pipeline.verdict(row, "icp")["label"] == "not usable for adaptation"
    # The wheel model has no closure: judged on RMS and delay.
    assert pipeline.verdict({"rms": 0.05, "delay_vs_pose": {"lag_s": 0.0}}, "wheel")["label"] == (
        "usable for adaptation"
    )
    nodata = {**good, "delay_vs_pose": {"lag_s": float("nan")}}
    assert "not measurable" in pipeline.verdict(nodata, "icp")["label"]


# --- sign and frame of the scan matcher through the LiDAR mount ---------------------------------

MOUNT = motion.Mount(0.2, 0.0, math.pi)  # at the front, facing backwards, as on the car
ANGLE_MIN = -math.pi
ANGLE_INC = 2 * math.pi / 720


def _room() -> list[tuple[np.ndarray, np.ndarray]]:
    """Wall segments (start, end) of a 5 x 4 m room with a 0.8 m box and a pillar in it."""
    rect = lambda x0, y0, x1, y1: [
        (np.array(p), np.array(q))
        for p, q in (
            ((x0, y0), (x1, y0)),
            ((x1, y0), (x1, y1)),
            ((x1, y1), (x0, y1)),
            ((x0, y1), (x0, y0)),
        )
    ]
    return rect(-2.0, -2.0, 3.0, 2.0) + rect(0.9, 0.2, 1.7, 1.0) + rect(-1.2, -1.4, -0.9, -1.1)


def _scan_from(pose: tuple[float, float, float], stamp: float) -> ScanRecord:
    """Ray cast the room from a car pose (x, y, heading of the rear axle in the world)."""
    x, y, psi = pose
    c, s = math.cos(psi), math.sin(psi)
    ox = x + c * MOUNT.x_m - s * MOUNT.y_m
    oy = y + s * MOUNT.x_m + c * MOUNT.y_m
    bearing = ANGLE_MIN + np.arange(720) * ANGLE_INC + MOUNT.yaw_rad + psi
    d = np.stack([np.cos(bearing), np.sin(bearing)], axis=1)
    best = np.full(720, np.inf)
    for p, q in _room():
        e = q - p
        denom = d[:, 0] * e[1] - d[:, 1] * e[0]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((p[0] - ox) * e[1] - (p[1] - oy) * e[0]) / denom
            u = ((p[0] - ox) * d[:, 1] - (p[1] - oy) * d[:, 0]) / denom
        hit = (denom != 0) & (t > 0.0) & (u >= 0.0) & (u <= 1.0)
        best = np.where(hit & (t < best), t, best)
    best[best > 10.0] = np.inf
    return ScanRecord(stamp, ANGLE_MIN, ANGLE_INC, 0.05, 12.0, tuple(float(r) for r in best))


def test_left_turn_is_a_positive_yaw_rate_through_the_mount() -> None:
    start = (0.0, 0.0, 0.0)
    left = (0.04, 0.01, 0.06)  # 0.06 rad left while driving forward
    right = (0.04, -0.01, -0.06)
    a = _scan_from(start, 0.0)
    dx, dy, dyaw, rms, share = matching.scan_motion(a, _scan_from(left, 0.1), MOUNT)
    assert dyaw == pytest.approx(0.06, abs=0.005)
    assert dx == pytest.approx(0.04, abs=0.01)
    assert dy == pytest.approx(0.01, abs=0.01)
    assert rms <= matching.RMS_MAX_M and share >= matching.INLIER_SHARE_MIN
    _, _, dyaw_r, _, _ = matching.scan_motion(a, _scan_from(right, 0.1), MOUNT)
    assert dyaw_r == pytest.approx(-0.06, abs=0.005)


def test_a_wheel_seed_composes_into_the_whole_motion() -> None:
    """Starting the match at the wheel twist's motion gives the same whole motion, and recovers
    a turn that the 40 identity-start iterations of the repo's matcher do not."""
    start = (0.0, 0.0, 0.0)
    moved = (0.12, 0.05, 0.30)  # a large step: 0.3 rad in one scan
    a, b = _scan_from(start, 0.0), _scan_from(moved, 0.1)
    plain = matching.scan_motion(a, b, MOUNT)
    seeded = matching.scan_motion(a, b, MOUNT, init=(0.28, 0.13))
    assert seeded[2] == pytest.approx(0.30, abs=0.005)
    assert seeded[0] == pytest.approx(moved[0] * math.cos(0.0), abs=0.02)
    assert abs(seeded[2] - 0.30) <= abs(plain[2] - 0.30) + 1e-6
    # A seed equal to the identity is the plain match.
    same = matching.scan_motion(a, b, MOUNT, init=(0.0, 0.0))
    assert same[2] == pytest.approx(plain[2], abs=1e-9)


def test_init_motion_is_an_arc() -> None:
    r, t = matching.init_motion(0.0, 0.1)
    assert np.allclose(r, np.eye(2)) and np.allclose(t, [0.1, 0.0])
    r, t = matching.init_motion(math.pi / 2, 1.0)
    assert np.allclose(t, [math.cos(math.pi / 4), math.sin(math.pi / 4)])
    assert r[1, 0] == pytest.approx(1.0)


def test_the_quality_flag_rejects_a_changed_scene() -> None:
    a = _scan_from((0.0, 0.0, 0.0), 0.0)
    other = _scan_from((0.0, 0.0, 0.0), 0.1)
    ranges = np.asarray(other.ranges)
    rng = np.random.default_rng(0)
    ranges[::2] = np.where(
        np.isfinite(ranges[::2]), rng.uniform(0.3, 3.5, ranges[::2].size), np.inf
    )
    noisy = ScanRecord(0.1, ANGLE_MIN, ANGLE_INC, 0.05, 12.0, tuple(float(r) for r in ranges))
    _, _, _, rms, share = matching.scan_motion(a, noisy, MOUNT)
    assert rms > matching.RMS_MAX_M or share < matching.INLIER_SHARE_MIN


# --- bag access and the report -------------------------------------------------------------------


def _fake_recording(n_scans: int = 40) -> bagio.Recording:
    t = np.arange(n_scans) * 0.1
    q = np.stack([bagio.quantise_ranges(np.full(720, 2.0, np.float32)) for _ in range(n_scans)])
    wt = np.arange(0.0, n_scans * 0.1, 0.02)
    return {
        "t0_log_s": np.array(100.0),
        "cache_version": np.array(bagio.CACHE_VERSION),
        "scan_t": t + 0.1,
        "scan_stamp": t,
        "scan_angle_min": np.full(n_scans, ANGLE_MIN),
        "scan_angle_increment": np.full(n_scans, ANGLE_INC),
        "scan_range_min": np.full(n_scans, 0.05),
        "scan_range_max": np.full(n_scans, 12.0),
        "scan_q": q,
        "wt_t": wt,
        "wt_stamp": wt,
        "wt_vx": np.full(wt.size, 1.0),
        "wt_wz": np.full(wt.size, 0.2),
        "odom_t": wt,
        "odom_v": np.full(wt.size, 1.0),
        "drive_t": wt,
        "drive_speed": np.full(wt.size, 1.0),
        "drive_steer": np.full(wt.size, 0.1),
        "pose_t": t + 0.14,
        "pose_stamp": t,
        "pose_x": t * 1.0,
        "pose_y": np.zeros(n_scans),
        "pose_yaw": 0.2 * t,
    }


def test_recording_cache_round_trip_and_version_refusal(tmp_path: Path) -> None:
    rec = _fake_recording()
    path = tmp_path / "rec.npz"
    np.savez(path, **rec)
    again = bagio.read_recording(path)
    assert np.array_equal(again["scan_q"], rec["scan_q"])
    assert bagio.load_recording(path, None)["pose_yaw"][5] == pytest.approx(0.1)
    np.savez(tmp_path / "old.npz", **{**rec, "cache_version": np.array(0)})
    with pytest.raises(ValueError, match="cache version"):
        bagio.read_recording(tmp_path / "old.npz")


def test_quaternion_yaw() -> None:
    half = 0.5 * 1.2
    assert bagio.quaternion_yaw(0.0, 0.0, math.sin(half), math.cos(half)) == pytest.approx(1.2)


def test_segment_selection_and_steering_model_come_from_the_vehicle_binding() -> None:
    veh = vehicle.load_vehicle(REPO_ROOT)
    assert veh.wheelbase_m > 0 and abs(veh.lidar_yaw_rad) == pytest.approx(math.pi, abs=1e-6)
    rec = _fake_recording()
    spec = run.SegmentSpec("bag", "S", 0.5, 3.0, "ccw")
    scans = run.segment_scans(rec, spec)
    assert scans[0] == 4 and scans[-1] == 28
    poses = run.segment_poses(rec, scans)
    assert rec["pose_stamp"][poses[0]] == pytest.approx(rec["scan_stamp"][scans[0]])
    _t, w = run.steering_model_rate(rec, veh)
    assert w[0] == pytest.approx(1.0 * math.tan(0.1) / veh.wheelbase_m)
    inp = run.build_segment_input(rec, spec, veh, variants=())
    res = pipeline.evaluate_segment(inp)
    # The fake car drives 1 m/s and turns 0.2 rad/s: the wheel model is exactly the reference.
    assert np.allclose(res.err["wheel"], 0.0, atol=1e-9)
    assert np.allclose(res.speed, 1.0)


def test_split_segments_reads_the_named_segments_and_refuses_unknown() -> None:
    split = {
        "bags": {
            "b1": {"segments": [{"name": "A", "from": 1, "to": 2, "direction": "cw"}]},
            "b2": {},
        }
    }
    specs = run.split_segments(split, ("A",))
    assert specs[0] == run.SegmentSpec("b1", "A", 1.0, 2.0, "cw")
    with pytest.raises(KeyError):
        run.split_segments(split, ("B",))


def test_json_and_markdown_report_round_trip(tmp_path: Path) -> None:
    res = pipeline.evaluate_segment(synthetic_segment(0.0, 0.002, 0.0, 0.1, seed=2))
    summary = pipeline.summarise([res])
    report.write_json(summary, tmp_path / "s.json")
    loaded = json.loads((tmp_path / "s.json").read_text())
    assert loaded["verdicts"]["icp"]["label"] in (
        "usable for adaptation",
        "not usable for adaptation",
        "not measurable (a criterion has no data)",
    )
    md = report.markdown_tables(summary)
    assert "### Signal: icp" in md and "### Signal: wheel" in md and "syn" in md


def test_plot_writes_a_png_when_matplotlib_is_available(tmp_path: Path) -> None:
    pytest.importorskip("matplotlib")
    res = pipeline.evaluate_segment(synthetic_segment(0.0, 0.002, 0.0, 0.1, seed=2))
    assert report.plot_segment(res, tmp_path / "p.png")
    assert (tmp_path / "p.png").stat().st_size > 1000
