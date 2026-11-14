"""The latency measurement, the scan cache with the split's time axis, and motion extension."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest
from racer_pnp_perception import latency
from racer_pnp_perception import selflabel as sl
from racer_pnp_perception.tracks import IslandParams, island_world
from racer_track_perception import core
from racer_track_perception.vehicle import LiveVehicle
from test_gp1 import random_net
from test_selflabel import SPEED_MPS, drive_poses, scans_from_poses, true_motion


@pytest.fixture(scope="module")
def sim(live: LiveVehicle):
    world = island_world(IslandParams(1.0, 3.0, 2.0, 0.4))
    poses = drive_poses(world, 48)
    return poses, scans_from_poses(world, live, poses)


def test_latency_reports_percentiles_for_forward_and_scan_to_track(sim, live: LiveVehicle) -> None:
    _poses, scans = sim
    perceivers = {
        core.BACKEND_GEOMETRIC: core.TrackPerceiver(core.BACKEND_GEOMETRIC, live),
        core.BACKEND_NET: core.TrackPerceiver(
            core.BACKEND_NET, live, net=random_net(), model_id="random"
        ),
    }
    out = latency.measure(scans, perceivers, 30)
    assert out["scans"] == 30
    for block in (out["net_forward"], *out["scan_to_track"].values()):
        assert 0.0 < block["p50"] <= block["p99"] <= block["max"]
    assert out["p50"] == out["net_forward"]["p50"]
    json.dumps(out)


def write_cache(tmp: Path, scans: sl.ScanSet, log_s: np.ndarray | None) -> Path:
    cache = tmp / "cache"
    cache.mkdir(exist_ok=True)
    extra = {} if log_s is None else {"log_s": log_s}
    np.savez(
        cache / "bag.scans.npz",
        stamp_s=scans.stamp_s,
        ranges=scans.ranges,
        angle_min=scans.angle_min,
        angle_increment=scans.angle_increment,
        range_min=scans.range_min,
        range_max=scans.range_max,
        speed_t=scans.stamp_s,
        speed_mps=np.full(len(scans), SPEED_MPS),
        **extra,
    )
    return tmp / "bag"


def test_scan_cache_keeps_the_log_time_axis_and_old_caches_fall_back(tmp_path: Path, sim) -> None:
    _poses, scans = sim
    log_s = scans.stamp_s + 0.0123
    got, _t, _v = sl.load_scanset(write_cache(tmp_path, scans, log_s), tmp_path / "cache")
    assert got.log_s is not None and np.array_equal(got.bag_time_s(), log_s)
    old_dir = tmp_path / "old"
    old_dir.mkdir()
    old, _t, _v = sl.load_scanset(write_cache(old_dir, scans, None), old_dir / "cache")
    assert old.log_s is None
    assert np.allclose(old.bag_time_s(), scans.stamp_s - scans.stamp_s[0])


def test_extend_motion_fills_only_the_missing_intervals(sim, live: LiveVehicle) -> None:
    poses, scans = sim
    truth = true_motion(poses)
    n = len(truth.ok)
    # A cache that was computed for the first half only (the rest NaN, as compute_motion leaves it).
    half = np.arange(n) < n // 2
    cached = sl.MotionSeries(
        np.where(half, truth.dx_m, np.nan),
        np.where(half, truth.dy_m, np.nan),
        np.where(half, truth.dyaw_rad, np.nan),
        np.where(half, 0.001, np.nan),
        np.where(half, 1.0, np.nan),
        half.copy(),
    )
    # Nothing missing for the wanted intervals: the very same series comes back.
    assert sl.extend_motion(cached, scans, live, half) is cached
    want = np.zeros(n, dtype=bool)
    want[n // 2 + 2 : n // 2 + 6] = True
    out = sl.extend_motion(cached, scans, live, want)
    assert np.array_equal(out.dx_m[half], cached.dx_m[half])  # cached intervals untouched
    filled = want & np.isfinite(out.rms_m)
    assert filled.sum() >= 3
    assert not np.isfinite(out.rms_m[~half & ~want]).any()  # the rest stays uncomputed
