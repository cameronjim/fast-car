"""L1: the fast RMS gate of the self-labeller (owner decision 35): off by default, 0.025 m above
the speed, 0.02 m at or below it, every other item of the quality flag unchanged."""

from __future__ import annotations

import dataclasses
import json
import math

import numpy as np
import pytest
from racer_pnp_perception import selflabel as sl
from racer_pnp_perception.tracks import IslandParams, island_world
from racer_track_perception.vehicle import LiveVehicle
from test_selflabel import SPEED_MPS, drive_poses, scans_from_poses, true_motion
from test_selflabel_cli import write_cache

FAST = sl.RmsGate(0.025, 0.8)


def test_default_gate_is_the_fixed_limit_everywhere() -> None:
    gate = sl.RmsGate()
    assert not gate.on
    np.testing.assert_array_equal(gate.limits(np.array([0.0, 0.8, 1.3, math.nan])), sl.RMS_MAX_M)


def test_fast_gate_applies_only_strictly_above_the_speed() -> None:
    limits = FAST.limits(np.array([0.5, 0.8, 0.8001, 1.3, math.nan]))
    np.testing.assert_array_equal(limits, [0.02, 0.02, 0.025, 0.025, 0.02])


def test_speed_at_holds_the_latest_sample_and_takes_the_magnitude() -> None:
    t = np.array([0.0, 1.0, 2.0])
    v = np.array([0.5, -1.0, 0.9])
    got = sl.speed_at(np.array([-0.1, 0.0, 0.99, 1.5, 2.0, 9.0]), t, v)
    np.testing.assert_array_equal(got[1:], [0.5, 0.5, 1.0, 0.9, 0.9])
    assert math.isnan(got[0])
    assert np.isnan(sl.speed_at(np.array([1.0]), np.zeros(0), np.zeros(0))).all()


def test_interval_speed_reads_the_midpoint() -> None:
    scans = sl.ScanSet(np.array([0.0, 0.1, 0.2]), np.zeros((3, 4), "f4"), 0.0, 0.1, 0.0, 10.0)
    got = sl.interval_speed(scans, np.array([0.0, 0.06, 0.16]), np.array([0.3, 0.9, 1.2]))
    np.testing.assert_array_equal(got, [0.3, 0.9])


def test_speed_bands() -> None:
    assert sl.SPEED_BANDS == ("0.0-0.5", "0.5-0.8", "0.8-1.0", "1.0-1.2", "1.2+")
    assert [sl.speed_band(v) for v in (0.0, 0.49, 0.5, 0.8, 1.19, 1.2, 5.0)] == [
        "0.0-0.5",
        "0.0-0.5",
        "0.5-0.8",
        "0.8-1.0",
        "1.0-1.2",
        "1.2+",
        "1.2+",
    ]


def _series(rms: list[float], share: list[float] | None = None) -> sl.MotionSeries:
    n = len(rms)
    share_a = np.ones(n) if share is None else np.array(share)
    r = np.array(rms)
    ok = (r <= sl.RMS_MAX_M) & (share_a >= sl.INLIER_SHARE_MIN)
    return sl.MotionSeries(np.full(n, 0.1), np.zeros(n), np.zeros(n), r, share_a, ok)


def _scans(dt: list[float]) -> sl.ScanSet:
    stamps = np.concatenate([[0.0], np.cumsum(dt)])
    return sl.ScanSet(stamps, np.zeros((len(stamps), 4), "f4"), 0.0, 0.1, 0.0, 10.0)


def test_regate_off_returns_the_series_unchanged() -> None:
    series = _series([0.022, 0.01])
    assert sl.regate(series, _scans([0.1, 0.1]), np.array([1.0, 1.0]), sl.RmsGate()) is series


def test_regate_changes_only_the_rms_item() -> None:
    series = _series([0.022, 0.022, 0.026, 0.022, math.nan, 0.01], [1, 1, 1, 0.6, 1, 1])
    scans = _scans([0.1, 0.1, 0.1, 0.1, 0.1, 0.2])  # the last interval breaks the dt item
    speeds = np.array([1.0, 0.8, 1.0, 1.0, 1.0, 1.0])
    out = sl.regate(series, scans, speeds, FAST)
    # fast and under 0.025: ok; at 0.8: still 0.02; over 0.025; share; NaN; dt.
    np.testing.assert_array_equal(out.ok, [True, False, False, False, False, False])
    for name in ("dx_m", "dy_m", "dyaw_rad", "rms_m", "inlier_share"):
        np.testing.assert_array_equal(getattr(out, name), getattr(series, name))


def test_regate_with_the_default_limit_reproduces_the_compute_motion_flag() -> None:
    series = _series([0.019, 0.021, 0.02, 0.03])
    same = sl.regate(series, _scans([0.1] * 4), np.full(4, 0.5), sl.RmsGate(0.025, 2.0))
    np.testing.assert_array_equal(same.ok, series.ok)


def _drive(tmp_path, live: LiveVehicle):  # type: ignore[no-untyped-def]
    world = island_world(IslandParams(1.0, 3.0, 2.0, 0.4))
    poses = drive_poses(world, 64)
    scans = scans_from_poses(world, live, poses)
    motion = true_motion(poses)
    rms = motion.rms_m.copy()
    rms[30] = 0.022  # one marginal interval in the middle of every window around it
    motion = dataclasses.replace(motion, rms_m=rms, ok=rms <= sl.RMS_MAX_M)
    return scans, motion


def _run(tmp_path, live, name: str, extra: list[str]):  # type: ignore[no-untyped-def]
    scans, motion = _drive(tmp_path, live)
    out = tmp_path / name
    write_cache(out, "simbag", scans, SPEED_MPS)
    sl.save_motion(out / "cache" / "simbag.motion.npz", motion)
    assert sl.main(["--bags", str(tmp_path / "simbag"), "--out", str(out), *extra]) == 0
    entry = json.loads((out / "manifest.json").read_text())["bags"]["simbag"]
    with np.load(out / "simbag.npz") as z:
        labels = {k: z[k] for k in z.files}
    return entry, labels


def test_cli_flag_keeps_windows_the_default_drops(tmp_path, live: LiveVehicle) -> None:
    default, d_labels = _run(tmp_path, live, "default", [])
    fast, f_labels = _run(
        tmp_path, live, "fast", ["--rms-gate-fast", "0.025", "--rms-fast-speed", "0.8"]
    )
    slow, s_labels = _run(
        tmp_path, live, "slow", ["--rms-gate-fast", "0.025", "--rms-fast-speed", "2.0"]
    )
    assert default["dropped"]["icp_failed"] > 0 and "rms_gate" not in default
    assert fast["dropped"]["icp_failed"] == 0 and fast["kept"] > default["kept"]
    assert fast["rms_gate"] == {"default_m": 0.02, "fast_m": 0.025, "fast_speed_mps": 0.8}
    # Every label the default keeps is kept unchanged; the cached motion is not rewritten.
    common = np.isin(f_labels["index"], d_labels["index"])
    assert common.sum() == len(d_labels["index"])
    np.testing.assert_array_equal(f_labels["label"][common], d_labels["label"])
    with np.load(tmp_path / "fast" / "cache" / "simbag.motion.npz") as z:
        assert not z["ok"][30]
    # Speed at or below the flag's speed: identical to the default.
    assert slow["dropped"] == default["dropped"]
    for k in d_labels:
        np.testing.assert_array_equal(s_labels[k], d_labels[k])
    # The band breakdown adds up (1.5 m/s: one band).
    band = fast["speed_bands"]["1.2+"]
    assert band["moving_scans"] == fast["moving_scans"]
    assert band["kept"] + sum(band["dropped"].values()) == band["moving_scans"]


def test_cli_flags_go_together(tmp_path) -> None:  # type: ignore[no-untyped-def]
    with pytest.raises(SystemExit):
        sl.main(["--bags", str(tmp_path / "x"), "--rms-gate-fast", "0.025"])
