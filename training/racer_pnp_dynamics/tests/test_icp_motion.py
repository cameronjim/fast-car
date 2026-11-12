"""L1 for icp_motion.py: known motion between two synthetic scans of a corridor."""

from __future__ import annotations

import numpy as np
import pytest
from follower_replay.bag import ScanRecord
from racer_pnp_dynamics import icp_motion as im
from racer_pnp_dynamics.bags import Run

N_BEAMS = 720
INC = 2 * np.pi / N_BEAMS


def _room() -> np.ndarray:
    """A 4 m by 1.2 m corridor closed at both ends with a box on one wall, as (N, 4) segments."""
    box = [
        (1.5, 0.6, 1.9, 0.6),
        (1.9, 0.6, 1.9, 0.35),
        (1.9, 0.35, 1.5, 0.35),
        (1.5, 0.35, 1.5, 0.6),
    ]
    walls = [
        (-1.0, -0.6, 3.0, -0.6),
        (-1.0, 0.6, 3.0, 0.6),
        (-1.0, -0.6, -1.0, 0.6),
        (3.0, -0.6, 3.0, 0.6),
    ]
    return np.array(walls + box, dtype=float)


def _scan(pose: tuple[float, float, float], mount, stamp: float) -> ScanRecord:
    """Ray cast from the lidar of a car whose rear axle is at `pose` (x, y, yaw) in the room."""
    x, y, yaw = pose
    ox = x + mount.x_m * np.cos(yaw) - mount.y_m * np.sin(yaw)
    oy = y + mount.x_m * np.sin(yaw) + mount.y_m * np.cos(yaw)
    bearing = -np.pi + np.arange(N_BEAMS) * INC
    world = yaw + bearing + mount.yaw_rad
    d = np.stack([np.cos(world), np.sin(world)], axis=1)
    ranges = np.full(N_BEAMS, np.inf)
    for x1, y1, x2, y2 in _room():
        e = np.array([x2 - x1, y2 - y1])
        denom = d[:, 0] * (-e[1]) + d[:, 1] * e[0]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((x1 - ox) * (-e[1]) + (y1 - oy) * e[0]) / denom
            u = ((x1 - ox) * d[:, 1] - (y1 - oy) * d[:, 0]) / denom
        hit = (t > 0) & (u >= 0) & (u <= 1)
        ranges[hit] = np.minimum(ranges[hit], t[hit])
    return ScanRecord(stamp, -np.pi, INC, 0.05, 12.0, tuple(float(r) for r in ranges))


def test_known_motion_between_two_scans(toy_vehicle):
    mount = im.mount_of(toy_vehicle)
    a = _scan((0.0, 0.0, 0.0), mount, 0.0)
    b = _scan((0.06, 0.012, 0.04), mount, 0.1)
    dx, dy, dyaw, rms, share = im.scan_motion(a, b, mount)
    assert dx == pytest.approx(0.06, abs=0.005)
    assert dy == pytest.approx(0.012, abs=0.005)
    assert dyaw == pytest.approx(0.04, abs=0.01)
    assert rms < im.RMS_MAX_M and share > im.INLIER_SHARE_MIN


def test_compute_motion_flags_quality_and_gaps(toy_vehicle):
    mount = im.mount_of(toy_vehicle)
    poses = [(0.0, 0.0, 0.0), (0.05, 0.0, 0.02), (0.10, 0.01, 0.04)]
    scans = [_scan(p, mount, 0.1 * i) for i, p in enumerate(poses)]
    m = im.compute_motion(scans, toy_vehicle, ignore=frozenset())
    assert m.ok.all() and len(m.ok) == 2
    np.testing.assert_allclose(m.dyaw_rad, 0.02, atol=0.01)
    np.testing.assert_allclose(m.yaw_rate_rad_per_s(), 0.2, atol=0.1)
    assert m.ground_speed_mps()[0] == pytest.approx(0.5, abs=0.06)
    # a time gap is not a 0.1 s interval
    late = [scans[0], scans[1], _scan(poses[2], mount, 0.5)]
    assert list(im.compute_motion(late, toy_vehicle, ignore=frozenset()).ok) == [True, False]
    # not asked for: not ok, not computed
    skipped = im.compute_motion(
        scans, toy_vehicle, want=np.array([True, False]), ignore=frozenset()
    )
    assert list(skipped.ok) == [True, False] and np.isnan(skipped.dyaw_rad[1])


def test_too_few_returns_is_not_ok(toy_vehicle):
    mount = im.mount_of(toy_vehicle)
    good = _scan((0.0, 0.0, 0.0), mount, 0.0)
    empty = ScanRecord(0.1, -np.pi, INC, 0.05, 12.0, tuple([float("inf")] * N_BEAMS))
    m = im.compute_motion([good, empty], toy_vehicle, ignore=frozenset())
    assert not m.ok[0]


def test_compose_chains_motions():
    poses = im.compose(np.array([1.0, 1.0]), np.array([0.0, 0.0]), np.array([np.pi / 2, 0.0]))
    np.testing.assert_allclose(poses[0], [0, 0, 0])
    np.testing.assert_allclose(poses[1], [1, 0, np.pi / 2], atol=1e-12)
    np.testing.assert_allclose(poses[2], [1, 1, np.pi / 2], atol=1e-12)  # second step goes left


def test_driving_intervals_and_cache_round_trip(tmp_path, toy_vehicle):
    t = np.arange(0, 1.0, 0.02)
    speed = np.where(t < 0.5, 0.0, 0.4)
    scans = [ScanRecord(0.1 * i, -np.pi, INC, 0.05, 12.0, (1.0,) * 4) for i in range(11)]
    run = Run("r", t, speed, np.zeros_like(t), np.full_like(t, np.nan), scans)
    want = im.driving_intervals(run)
    assert want.shape == (10,) and not want[:5].any() and want[5:].all()
    assert im.driving_intervals(Run("x", t, speed, speed, speed, [])).shape == (0,)
    m = im.MotionSeries(*(np.arange(3.0),) * 7, ok=np.array([True, False, True]))
    im.save_motion(tmp_path / "c" / "m.npz", m)
    back = im.load_motion(tmp_path / "c" / "m.npz")
    np.testing.assert_array_equal(back.ok, m.ok)
    np.testing.assert_array_equal(back.t_s, m.t_s)
