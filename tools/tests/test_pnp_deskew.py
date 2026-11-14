"""L1 of tools/pnp_deskew (the M-DSK measurement): the paired block bootstrap, the KD-tree ICP
against tools/safety_replay/motion.icp, the best-match search of metric C, and the replay of the
relay over a recording."""

from __future__ import annotations

import math

import numpy as np
import pytest
from pnp_deskew import bagio, metric, replay
from pnp_replay.bag import quantise_ranges
from safety_replay import motion

BEAMS = 720
INC = 2.0 * math.pi / 719.0


# -- bootstrap ---------------------------------------------------------------------------------------


def test_bootstrap_of_identical_values_is_one() -> None:
    rng = np.random.default_rng(1)
    x = rng.uniform(0.01, 0.03, 60)
    out = metric.block_bootstrap_median_ratio(x, x, np.repeat(np.arange(12), 5))
    assert out.ratio == 1.0 and out.lo == 1.0 and out.hi == 1.0
    assert out.blocks == 12 and out.scans == 60


def test_bootstrap_of_a_clear_gain_has_its_upper_end_below_one() -> None:
    rng = np.random.default_rng(2)
    raw = rng.uniform(0.02, 0.04, 80)
    dsk = raw * 0.5
    out = metric.block_bootstrap_median_ratio(dsk, raw, np.repeat(np.arange(16), 5))
    assert out.ratio == pytest.approx(0.5) and out.hi < 1.0 and out.lo <= out.ratio <= out.hi


def test_bootstrap_is_deterministic_and_pairs_the_scans() -> None:
    raw = np.array([0.03, 0.04, np.nan, 0.05, 0.02, 0.03])
    dsk = np.array([0.02, np.nan, 0.03, 0.03, 0.01, 0.02])
    blocks = np.array([0, 0, 1, 1, 2, 2])
    a = metric.block_bootstrap_median_ratio(dsk, raw, blocks)
    b = metric.block_bootstrap_median_ratio(dsk, raw, blocks)
    assert a == b
    assert a.scans == 4  # only scans finite in both count
    assert a.ratio == pytest.approx(
        np.median([0.02, 0.03, 0.01, 0.02]) / np.median([0.03, 0.05, 0.02, 0.03])
    )


def test_bootstrap_of_nothing_is_nan() -> None:
    out = metric.block_bootstrap_median_ratio(np.array([np.nan]), np.array([np.nan]), np.array([0]))
    assert math.isnan(out.ratio) and out.scans == 0


def test_median_ignoring_nan() -> None:
    assert metric.median_ignoring_nan(np.array([1.0, np.nan, 3.0])) == 2.0
    assert math.isnan(metric.median_ignoring_nan(np.array([np.nan])))


# -- ICP and the best-match search --------------------------------------------------------------------


def _room(shift: tuple[float, float] = (0.0, 0.0), n: int = 120) -> np.ndarray:
    """Points on an L-shaped set of walls, offset by `shift` (an asymmetric shape: ICP has one
    answer)."""
    t = np.linspace(0.0, 1.0, n)
    wall_a = np.stack([2.0 * t - 1.0, np.full(n, 1.0)], axis=1)
    wall_b = np.stack([np.full(n, 1.0), 1.0 - 1.5 * t], axis=1)
    wall_c = np.stack([np.full(n, -1.0), 1.0 - 0.8 * t], axis=1)
    return np.concatenate([wall_a, wall_b, wall_c]) + np.array(shift)


def test_icp_kdtree_equals_the_brute_force_icp() -> None:
    cKDTree = pytest.importorskip("scipy.spatial").cKDTree
    target = _room()
    source = _room((0.03, -0.02))
    r0, t0, rms0 = motion.icp(source, target, inlier_m=0.05)
    r1, t1, rms1, share = metric.icp_kdtree(source, target, cKDTree(target), inlier_m=0.05)
    assert np.allclose(r0, r1, atol=1e-12) and np.allclose(t0, t1, atol=1e-12)
    assert rms0 == pytest.approx(rms1, abs=1e-12) and share > 0.9


def test_icp_kdtree_refuses_too_few_points() -> None:
    cKDTree = pytest.importorskip("scipy.spatial").cKDTree
    few = np.zeros((5, 2))
    with pytest.raises(ValueError):
        metric.icp_kdtree(few, _room(), cKDTree(_room()))


def test_best_match_picks_the_lowest_rms_among_the_converged_and_nan_without_one() -> None:
    pytest.importorskip("scipy")
    fast = [_room((0.01, 0.0)), np.random.default_rng(3).uniform(-30, 30, (200, 2))]
    slow = [
        _room((0.04, 0.0)),
        _room((0.0, 0.0)),
        np.random.default_rng(4).uniform(-30, 30, (200, 2)),
    ]
    rms, converged = metric.best_match_rms(fast, slow)
    assert np.isfinite(rms[0]) and rms[0] < 1e-6 and converged[0] >= 2  # the exact copy wins
    assert math.isnan(rms[1]) and converged[1] == 0  # noise matches nothing


# -- replay of the relay over a recording ----------------------------------------------------------------


def _recording(n_scans: int = 4, odom: bool = True) -> dict[str, np.ndarray]:
    t0 = 1.79e9
    scan_t = 10.0 + 0.1 * np.arange(n_scans)
    q = np.stack([quantise_ranges(np.full(BEAMS, 2.0 + 0.001 * (np.arange(BEAMS) % 20)))] * n_scans)
    odom_t = np.arange(8.0, 11.0, 0.02) if odom else np.zeros(0)
    drive_t = np.arange(8.0, 11.0, 0.02)
    return {
        "t0_log_s": np.array(t0),
        "scan_t": scan_t,
        "scan_stamp": scan_t - 0.1,  # the stamp is the start of the sweep
        "scan_time": np.full(n_scans, 0.0998),
        "scan_dt": np.full(n_scans, 0.0998 / 719.0),
        "scan_angle_min": np.full(n_scans, -math.pi),
        "scan_angle_increment": np.full(n_scans, INC),
        "scan_range_min": np.full(n_scans, 0.05),
        "scan_range_max": np.full(n_scans, 16.0),
        "scan_q": q,
        "odom_t": odom_t,
        "odom_stamp": odom_t - 0.001,
        "odom_v": np.full(len(odom_t), 1.2),
        "drive_t": drive_t,
        "drive_steer": np.full(len(drive_t), 0.2),
    }


def test_replay_deskews_with_the_node_code_and_reports_the_offset_and_twist() -> None:
    rec = _recording()
    relay = replay.build_relay("sweep_start", "index_descending")
    out = replay.deskew_segment(rec, np.arange(4), relay, bagio.scan_ranges)
    assert out.reason == [""] * 4
    assert np.allclose(out.offset_s, 0.0998, atol=1e-6)
    assert np.allclose(out.v_mps, 1.2) and (out.yaw_rate_radps > 0).all()
    assert not np.array_equal(
        out.ranges, replay.raw_segment(rec, np.arange(4), bagio.scan_ranges).ranges
    )


def test_replay_without_odometry_passes_every_scan_through_unchanged() -> None:
    rec = _recording(odom=False)
    relay = replay.build_relay("sweep_start", "index_ascending")
    out = replay.deskew_segment(rec, np.arange(4), relay, bagio.scan_ranges)
    assert out.reason == ["no_odom"] * 4
    raw = replay.raw_segment(rec, np.arange(4), bagio.scan_ranges)
    assert np.array_equal(out.ranges, raw.ranges) and np.all(out.offset_s == 0.0)


def test_replay_of_no_scans_is_empty() -> None:
    out = replay.deskew_segment(
        _recording(),
        np.zeros(0, dtype=np.int64),
        replay.build_relay("sweep_start", "index_ascending"),
        bagio.scan_ranges,
    )
    assert len(out.index) == 0


def test_wheel_yaw_rate_at_scans_uses_the_newest_samples() -> None:
    rec = _recording()
    v, rate = replay.wheel_yaw_rate_at_scans(rec, np.arange(4))
    assert np.allclose(v, 1.2) and (rate > 0).all()


def test_the_hypotheses_are_the_four_combinations() -> None:
    assert set(replay.HYPOTHESES) == {
        (s, o)
        for s in ("sweep_start", "sweep_end")
        for o in ("index_ascending", "index_descending")
    }


# -- step B, items 1 and 2 with a stub scorer ---------------------------------------------------------


class _StubEv:
    """Stands in for pnp_deskew_eval: recordings are small dicts and pair_scores returns RMS values
    that are 0.6 times raw for the deskewed condition (corner scans) and equal (other scans)."""

    def recording(self, bag):
        return {"bag": bag}

    def pair_scores(self, rec, fast, slow, condition, which, fast_corner, ignore, veh, workers):
        n = 40 if which == "corner" else 10
        rng = np.random.default_rng(7 if which == "corner" else 8)
        raw = rng.uniform(0.012, 0.016, n)
        gain = 0.6 if (condition != "raw" and which == "corner") else 1.0
        return {
            "scan_index": np.arange(n),
            "t_s": fast.t_from + np.arange(n) * 0.1,
            "rms_m": raw * gain,
            "passed_through": 0,
        }


def test_items_1_and_2_pass_a_clear_gain_and_report_the_bootstrap(monkeypatch) -> None:
    import yaml
    from pnp_deskew import stepb

    monkeypatch.setattr(stepb.vehicle, "load_vehicle", lambda: None)
    monkeypatch.setattr(stepb.matching, "self_returns", lambda rec, veh: frozenset())
    split = yaml.safe_load((stepb.REPO_ROOT / "evaluation/prereg/pnp-v0.2-split.yaml").read_text())
    cfg = {
        "result_pairs": [
            {"fast": "R8", "slow": "R3", "direction": "ccw"},
            {"fast": "R7b", "slow": "R4", "direction": "cw"},
        ],
        "fast_corner": {"min_wheel_speed_mps": 0.6, "min_abs_yaw_rate_radps": 0.5},
    }
    out = stepb.items_1_2(_StubEv(), cfg, split, "sweep_end/index_descending", 1)
    assert out["item1"]["ratio"] == pytest.approx(0.6, abs=1e-9)
    assert out["item1"]["ci95_hi"] < 1.0 and out["item1"]["passed"] is True
    assert out["item2"]["ratio"] == pytest.approx(1.0) and out["item2"]["passed"] is True
    assert out["item1"]["paired_scored"] == 80 and out["item1"]["blocks"] == 4  # 4 s per pair, 2 s blocks
    assert set(out["pairs"]) == {"R8/R3", "R7b/R4"}


def test_clean_turns_nan_into_null() -> None:
    from pnp_deskew import stepb

    cleaned = stepb._clean({"a": np.float64("nan"), "b": [np.int64(2)], "c": np.bool_(True)})
    assert cleaned == {"a": None, "b": [2], "c": True}
