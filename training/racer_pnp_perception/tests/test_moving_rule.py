"""L1: the moving test of a bag without /odom/wheel (pre-registration v0.3 section 3.2): a scan is
moving when the ICP ground speed is at least 0.2 m/s, implemented once in the bag reader."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pytest
from racer_pnp_perception import selflabel as sl
from racer_track_perception.vehicle import LiveVehicle

N = 12


def scans(n: int = N, dt: float = 0.1) -> sl.ScanSet:
    return sl.ScanSet(
        np.arange(n) * dt,
        np.full((n, 8), 2.0, dtype=np.float32),
        -3.0,
        0.1,
        0.05,
        12.0,
        np.arange(n) * dt,
    )


def series_with_speeds(speeds: list[float], dt: float = 0.1) -> sl.MotionSeries:
    """One interval per entry: straight ahead at the given speed."""
    m = len(speeds)
    dx = np.array(speeds) * dt
    z = np.zeros(m)
    return sl.MotionSeries(
        dx, z.copy(), z.copy(), z.copy() + 0.01, z.copy() + 0.9, np.ones(m, bool)
    )


def test_the_constant_is_the_pre_registered_one() -> None:
    from racer_pnp_perception.prereg import moving_speed_min_mps

    assert moving_speed_min_mps() == sl.ICP_MOVING_MPS == 0.2


def test_icp_ground_speed_is_distance_over_time_of_the_interval_ending_at_the_scan() -> None:
    sc = scans()
    speeds = [0.0, 0.1, 0.19, 0.2, 0.5, 1.0, 0.0, 0.3, 0.3, 0.3, 0.05]
    out = sl.icp_ground_speed(sc, series_with_speeds(speeds))
    assert out[0] == 0.0
    assert np.allclose(out[1:], speeds)


def test_a_scan_without_an_icp_result_reads_zero() -> None:
    sc = scans()
    series = series_with_speeds([0.5] * (N - 1))
    dx = series.dx_m.copy()
    dx[3] = np.nan
    broken = sl.MotionSeries(
        dx, series.dy_m, series.dyaw_rad, series.rms_m, series.inlier_share, series.ok
    )
    out = sl.icp_ground_speed(sc, broken)
    assert out[4] == 0.0 and out[3] == pytest.approx(0.5)


def test_moving_is_at_least_the_threshold() -> None:
    sc = scans()
    speeds = [0.0, 0.1, 0.19, 0.21, 0.5, 1.0, 0.0, 0.3, 0.3, 0.3, 0.05]
    out = sl.icp_ground_speed(sc, series_with_speeds(speeds))
    moving = sl.moving_scans(sc, sc.stamp_s, out, sl.ICP_MOVING_MPS)
    want = [False] + [v >= 0.2 for v in speeds]
    assert list(moving) == want


@dataclass
class FakeScan:
    stamp_s: float
    ranges: np.ndarray
    angle_min: float = -3.0
    angle_increment: float = 0.1
    range_min: float = 0.05
    range_max: float = 12.0
    log_time_s: float = 0.0


@dataclass
class FakeOdom:
    stamp_s: float
    speed_mps: float


@dataclass
class FakeBag:
    scans: list[FakeScan]
    odom: list[FakeOdom] = field(default_factory=list)
    drive: list[FakeOdom] = field(default_factory=list)


def install_fake_reader(monkeypatch: pytest.MonkeyPatch, bag: FakeBag) -> None:
    sl._tools()  # puts tools/ on the path; follower_replay.bag imports mcap only inside readers
    import follower_replay.bag as bag_module

    monkeypatch.setattr(bag_module, "read_bag", lambda path: bag)


def fake_bag(with_odom: bool) -> FakeBag:
    sc = [
        FakeScan(0.1 * i + 5.0, np.full(8, 2.0, dtype=np.float32), log_time_s=0.1 * i)
        for i in range(N)
    ]
    odom = [FakeOdom(0.1 * i + 5.0, 0.77) for i in range(N)] if with_odom else []
    drive = [FakeOdom(0.1 * i + 5.0, 0.0) for i in range(N)]
    return FakeBag(sc, odom, drive)


def test_a_bag_without_odometry_takes_icp_ground_speed(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, live: LiveVehicle
) -> None:
    install_fake_reader(monkeypatch, fake_bag(with_odom=False))
    speeds = [0.0, 0.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0]
    calls = []

    def fake_motion(scans_, live_, want=None, workers=1):
        calls.append(want)
        return series_with_speeds(speeds)

    monkeypatch.setattr(sl, "compute_motion", fake_motion)
    # A bag from 2026-11-13 or later is in current units, so no unit conversion module is needed.
    bag_dir = tmp_path / "2026-11-13T00-00-00_nobody"
    sc, speed_t, speed = sl.load_scanset(bag_dir, tmp_path / "cache", live)
    assert len(calls) == 1 and calls[0] is None  # every interval, once
    assert np.array_equal(speed_t, sc.stamp_s)
    assert np.allclose(speed[1:], speeds)
    moving = sl.moving_scans(sc, speed_t, speed, sl.ICP_MOVING_MPS)
    assert list(np.flatnonzero(moving)) == [3, 4, 5, 6, 7]
    assert sl.speed_source(tmp_path / "cache", bag_dir.name) == "icp"
    # Cached: the second read does not match anything again and gives the same arrays.
    _sc2, speed_t2, speed2 = sl.load_scanset(bag_dir, tmp_path / "cache", live)
    assert len(calls) == 1
    assert np.array_equal(speed2, speed) and np.array_equal(speed_t2, speed_t)
    # The motion was cached for the labelling that follows.
    assert (tmp_path / "cache" / f"{bag_dir.name}.motion.npz").is_file()


def test_a_bag_with_odometry_keeps_wheel_speed(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, live: LiveVehicle
) -> None:
    install_fake_reader(monkeypatch, fake_bag(with_odom=True))

    def must_not_run(*a, **k):
        raise AssertionError("ICP must not run for a bag with /odom/wheel")

    monkeypatch.setattr(sl, "compute_motion", must_not_run)
    bag_dir = tmp_path / "2026-11-13T00-00-00_withodom"
    _sc, speed_t, speed = sl.load_scanset(bag_dir, tmp_path / "cache", live)
    assert np.allclose(speed, 0.77) and len(speed_t) == N
    assert sl.speed_source(tmp_path / "cache", bag_dir.name) == "odom"


def test_a_cache_without_the_field_is_an_odom_cache(tmp_path: Path) -> None:
    sc = scans()
    np.savez(
        tmp_path / "old.scans.npz",
        stamp_s=sc.stamp_s,
        ranges=sc.ranges,
        angle_min=sc.angle_min,
        angle_increment=sc.angle_increment,
        range_min=sc.range_min,
        range_max=sc.range_max,
        speed_t=sc.stamp_s,
        speed_mps=np.zeros(N),
    )
    assert sl.speed_source(tmp_path, "old") == "odom"
