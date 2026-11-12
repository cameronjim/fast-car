"""Tests for tools/pnp_replay.py: the metric functions on hand-made cases, the segment finder, the
stream writer, the perception fidelity self-test on the 2026-11-12 fixture, and the planner replay
end to end (skipped, with the reason, unless planner_replay_cli is built; the l3-and-cpp CI job
builds it and sets PNP_REPLAY_REQUIRE_CLI=1 so there a missing binary is a failure).

The fixture, tools/tests/fixtures/pnp_replay_2026-11-12.npz, is four cuts of the real bag:
R0 (the NO_TRACK onset), R1 (the corner approach and the stall), R3 (nose-in) and L (another
driver's laps). tools/pnp_replay/cut_fixture.py regenerates it.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

import numpy as np
import pytest
from pnp_replay import bag, cut_fixture, metrics, perception, planner_stream, report, segments

TESTS = Path(__file__).resolve().parent
REPO_ROOT = TESTS.parents[1]
FIXTURE = TESTS / "fixtures" / "pnp_replay_2026-11-12.npz"
DYNAMICS_BUNDLE = REPO_ROOT / "config" / "pnp" / "fixtures" / "dyn-nominal-v0"


# --- metrics ---------------------------------------------------------------------------------


def wall_points() -> np.ndarray:
    """Dense points on two walls, y = +0.5 and y = -0.5, x from -1 to 4 (base_link)."""
    x = np.arange(-1.0, 4.0, 0.01)
    return np.concatenate(
        [np.stack([x, np.full_like(x, 0.5)], axis=1), np.stack([x, np.full_like(x, -0.5)], axis=1)]
    )


def straight_track(y: float, half: float = 0.5) -> dict[str, np.ndarray]:
    x = 0.2 * np.arange(16)
    return {
        "x": x,
        "y": np.full(16, y),
        "hl": np.full(16, half),
        "hr": np.full(16, half),
    }


def test_clearance_deficit_of_a_centred_track_is_zero() -> None:
    t = straight_track(0.0)
    d = metrics.clearance_deficits(t["x"], t["y"], t["hl"], t["hr"], 16, wall_points())
    assert len(d) == 11  # stations 0 .. 2.0 m: the ones within 2.0 m of the car
    assert np.abs(d).max() < 0.011  # the point spacing


def test_track_025_m_too_close_to_one_wall_has_deficit_025() -> None:
    # the centreline runs 0.25 m off the middle, but the track claims 0.5 m to each side
    t = straight_track(0.25)
    d = metrics.clearance_deficits(t["x"], t["y"], t["hl"], t["hr"], 16, wall_points())
    assert len(d) == 10
    np.testing.assert_allclose(d, 0.25, atol=0.011)


def test_clearance_deficit_ignores_invalid_and_far_stations_and_empty_scans() -> None:
    t = straight_track(0.25)
    d = metrics.clearance_deficits(t["x"], t["y"], t["hl"], t["hr"], 4, wall_points())
    assert len(d) == 4  # only the first 4 stations are valid
    assert len(metrics.clearance_deficits(t["x"], t["y"], t["hl"], t["hr"], 0, wall_points())) == 0
    empty = np.zeros((0, 2))
    assert len(metrics.clearance_deficits(t["x"], t["y"], t["hl"], t["hr"], 16, empty)) == 0
    far = metrics.clearance_deficits(t["x"] + 5.0, t["y"], t["hl"], t["hr"], 16, wall_points())
    assert len(far) == 0


def test_perception_metrics_by_hand() -> None:
    valid = np.array([16, 16, 5, 0, 16])
    intr = np.array([0.0, 0.2, 0.05, math.nan, math.nan])
    res = np.array([0.1, 0.3, 0.2, math.nan, math.nan])
    deficits = np.array([0.0, 0.1, 0.2, 0.3])
    m = metrics.perception_metrics(valid, intr, res, deficits)
    assert m["scans"] == 5
    assert m["scans_with_track"] == 4
    assert m["intrusion_mean"] == pytest.approx((0.0 + 0.2 + 0.05) / 3)  # NaN and no-track skipped
    assert m["intrusion_share_over_0.10"] == pytest.approx(1 / 3)
    assert m["boundary_residual_p50_m"] == pytest.approx(0.2)
    assert m["valid_count_share_under_6"] == pytest.approx(2 / 5)
    assert m["clearance_deficit_share_over_0.15"] == pytest.approx(0.5)
    assert m["clearance_deficit_p90_m"] == pytest.approx(np.percentile(deficits, 90))


def test_perception_metrics_with_nothing_are_nan_not_errors() -> None:
    m = metrics.perception_metrics(np.zeros(0, int), np.zeros(0), np.zeros(0), np.zeros(0))
    assert m["scans"] == 0
    assert math.isnan(m["intrusion_mean"])
    assert math.isnan(m["clearance_deficit_p90_m"])


def test_steering_change_stats_pair_mask_leaves_out_jumps_across_a_gap() -> None:
    steer = np.array([0.0, 0.01, 0.02, 0.5, 0.51, 0.52])
    keep = np.array([True, True, False, True, True])  # the 0.02 -> 0.5 jump crosses a gap
    s = metrics.steering_change_stats(steer, 0.02, keep)
    assert s["std_rad"] == pytest.approx(0.0, abs=1e-12)
    assert s["p99_abs_rad"] == pytest.approx(0.01)
    unmasked = metrics.steering_change_stats(steer, 0.02)
    assert unmasked["p99_abs_rad"] > 0.4


def test_planner_metrics_do_not_count_recovering_ticks_as_zero_or_slow() -> None:
    n = 100
    now = np.arange(n) * 0.02
    state = np.ones(n, dtype=int)
    state[40:60] = 7
    speed = np.full(n, 0.3)
    speed[40:60] = -0.3
    speed[90:] = 0.0
    zeros = np.zeros(n)
    odom_t = np.arange(0.0, 3.0, 0.01)
    m = metrics.planner_metrics(
        now, state, speed, zeros, zeros, zeros, np.full(n, 0.1), odom_t,
        track_confidence=np.where(np.arange(n) < 50, 0.0, 1.0),
        block_reason=np.where(np.arange(n) >= 90, 2, 0),
        freespace_goal_x_m=np.where(np.arange(n) < 80, 2.0, np.nan),
    )  # fmt: skip
    assert m["recovering_ticks"] == 20
    assert m["speed_zero_share"] == pytest.approx(10 / 80)  # only the 80 forward-state ticks
    assert m["speed_below_dead_zone_share"] == 0.0
    assert m["speed_negative_outside_recovering"] == 0
    assert m["block_reason_counts"] == {"BELOW_SPEED_FLOOR": 10}
    assert m["track_confidence_mean"] == pytest.approx(0.5)
    assert m["track_confidence_zero_share"] == pytest.approx(0.5)
    assert m["freespace_goal_share"] == pytest.approx(0.8)
    assert m["state_counts"]["RECOVERING"] == 20


def test_steering_change_stats_count_known_flips() -> None:
    # changes +, -, +, -, +, -: every change after the first flips
    steer = np.concatenate([[0.0], np.cumsum([0.01, -0.01, 0.01, -0.01, 0.01, -0.01])])
    s = metrics.steering_change_stats(steer, 0.02)
    assert s["sign_flips_per_s"] == pytest.approx(5 / (6 * 0.02))
    assert s["std_rad"] == pytest.approx(0.01, rel=0.25)
    assert s["p99_abs_rad"] == pytest.approx(0.01)
    # a monotone ramp, and a held command, are not reversals
    ramp = metrics.steering_change_stats(np.linspace(0, 0.1, 11), 0.02)
    assert ramp["sign_flips_per_s"] == 0.0
    held = metrics.steering_change_stats(np.array([0.0, 0.1, 0.1, 0.1, 0.2, 0.1]), 0.02)
    assert held["sign_flips_per_s"] == pytest.approx(1 / (5 * 0.02))  # +, 0, 0, +, -
    assert math.isnan(metrics.steering_change_stats(np.array([0.0, 1.0]), 0.02)["std_rad"])


def test_eligible_ticks_need_a_fresh_scan_and_fresh_odometry() -> None:
    now = np.arange(0, 1.0, 0.1)
    scan_age = np.array([-1, -1, 0.1, 0.2, 0.34, 0.36, 0.1, 0.1, 0.1, 0.1])
    odom_t = np.array([0.15, 0.25, 0.35, 0.45, 0.55])  # nothing after 0.55
    ok = metrics.eligible_ticks(now, scan_age, odom_t)
    # tick 0: no scan; 1: no scan; 2 (t 0.2): fresh both; 3 (0.3): ok; 4 (0.4): ok;
    # 5 (0.5): scan 0.36 old; 6 (0.6): odom 0.05 old ok; 7 (0.7): odom 0.15 old
    assert ok.tolist() == [False, False, True, True, True, False, True, False, False, False]


def test_planner_metrics_state_counts_shares_and_agreement() -> None:
    n = 100
    now = 0.02 * np.arange(n)
    state = np.where(np.arange(n) < 60, 1, 2)
    speed = np.where(np.arange(n) < 20, 0.1, np.where(np.arange(n) < 60, 0.3, 0.0))
    steer = np.where(np.arange(n) % 2 == 0, 0.05, -0.05).astype(float)
    coll = np.full(n, 0.5)
    solve = np.full(n, 0.002)
    scan_age = np.full(n, 0.05)
    scan_age[:10] = -1.0  # no scan yet: not eligible
    odom_t = np.arange(0.0, 2.0, 0.02)
    rec_state = np.where(np.arange(n) < 50, 1, 2)  # the recorded run stopped 10 ticks earlier
    m = metrics.planner_metrics(
        now, state, speed, steer, coll, solve, scan_age, odom_t,
        dead_zone_mps=0.2, recorded_t=now, recorded_state=rec_state,
    )  # fmt: skip
    assert m["eligible_ticks"] == 90
    assert m["state_counts"] == {"DRIVING": 50, "NO_TRACK": 40}
    assert m["speed_zero_share"] == pytest.approx(40 / 90)
    assert m["speed_below_dead_zone_share"] == pytest.approx(10 / 90)  # ticks 10..19
    assert m["collision_fraction_mean"] == pytest.approx(0.5)
    assert m["solve_time_ms_this_machine"]["p50"] == pytest.approx(2.0)
    # alternating steering: a flip on every change
    assert m["steering_change"]["sign_flips_per_s"] == pytest.approx(48 / (49 * 0.02))
    assert m["agreement_with_recorded"]["share"] == pytest.approx(80 / 90)
    assert m["agreement_with_recorded"]["replayed_to_recorded"] == {
        "DRIVING->DRIVING": 40,
        "DRIVING->NO_TRACK": 10,
        "NO_TRACK->NO_TRACK": 40,
    }


def test_state_agreement_leaves_out_ticks_with_no_recorded_status_nearby() -> None:
    t = np.array([0.0, 0.5, 1.0])
    a = metrics.state_agreement(t, np.array([1, 1, 1]), np.array([0.0, 1.0]), np.array([1, 2]))
    assert a["ticks_compared"] == 2  # the tick at 0.5 has none within 20 ms
    assert a["share"] == pytest.approx(0.5)


# --- segments --------------------------------------------------------------------------------


def synthetic_bag_times() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Status at 50 Hz in three bursts (10-14 s, 20-24 s with a 1 s hole, 40-44 s), and odometry
    at 10 Hz with a fast stretch of 15 s at 1 m/s (60-75 s), one of 5 s (90-95 s), and one of 12 s
    during the second PnP burst."""
    parts = [np.arange(10, 14, 0.02), np.arange(20, 22, 0.02), np.arange(23, 24, 0.02)]
    parts.append(np.arange(40, 44, 0.02))
    status_t = np.concatenate(parts)
    odom_t = np.arange(0, 120, 0.1)
    v = np.zeros_like(odom_t)
    for a, b in ((60, 75), (90, 95), (15, 27)):
        v[(odom_t >= a) & (odom_t < b)] = 1.0
    return status_t, odom_t, v


def test_segment_finder_names_pnp_runs_and_fast_laps() -> None:
    status_t, odom_t, v = synthetic_bag_times()
    found = segments.find_segments(status_t, odom_t, v)
    names = [s.name for s in found]
    # a 1 s hole is inside a run; 2 s is the limit
    assert names == ["R0", "R1", "R2", "L"]
    r1 = found[1]
    assert (r1.t_from, r1.t_to) == pytest.approx((20.0, 23.98))
    lap = found[-1]
    assert lap.kind == "fast"
    assert (lap.t_from, lap.t_to) == pytest.approx((60.0, 74.9))  # 5 s and PnP-time stretches out
    assert not lap.has_planner_status


def test_segment_finder_pnp_gap_is_two_seconds() -> None:
    t = np.concatenate([np.arange(0, 1, 0.02), np.arange(3.5, 4, 0.02)])
    assert [s.name for s in segments.find_segments(t, np.zeros(0), np.zeros(0))] == ["R0", "R1"]
    t = np.concatenate([np.arange(0, 1, 0.02), np.arange(2.5, 3, 0.02)])
    assert [s.name for s in segments.find_segments(t, np.zeros(0), np.zeros(0))] == ["R0"]


def test_longest_fast_stretch_is_L() -> None:
    odom_t = np.arange(0, 200, 0.1)
    v = np.zeros_like(odom_t)
    v[(odom_t >= 10) & (odom_t < 25)] = 1.0
    v[(odom_t >= 100) & (odom_t < 160)] = 1.0
    found = segments.find_segments(np.zeros(0), odom_t, v)
    by = {s.name: s for s in found}
    assert by["L"].t_from == pytest.approx(100.0)
    assert by["L1"].t_from == pytest.approx(10.0)


def test_select_by_name_by_window_and_the_error_lists_what_exists() -> None:
    status_t, odom_t, v = synthetic_bag_times()
    found = segments.find_segments(status_t, odom_t, v)
    assert [s.name for s in segments.select(found, ["R2", "L"])] == ["R2", "L"]
    assert len(segments.select(found, None)) == len(found)
    w = segments.select(found, None, 5.0, 8.0)
    assert (w[0].name, w[0].t_from, w[0].t_to) == ("W", 5.0, 8.0)
    with pytest.raises(ValueError, match=r"no segment R9; the bag has: R0 10-14 s"):
        segments.select(found, ["R9"])
    with pytest.raises(ValueError, match="--from and --to"):
        segments.select(found, None, 5.0, None)
    with pytest.raises(ValueError, match="--from and --to"):
        segments.select(found, None, 8.0, 5.0)


# --- bag format and the stream ---------------------------------------------------------------


def tiny_recording() -> bag.Recording:
    n = 8
    ranges = np.array(
        [[1.0, 2.5, math.inf, 0.75, 3.14175, 0.5, math.inf, 1.25]] * 2, dtype=np.float32
    )
    q = np.stack([bag.quantise_ranges(r) for r in ranges])
    return {
        "t0_log_s": np.array(100.0),
        "cache_version": np.array(bag.CACHE_VERSION),
        "scan_t": np.array([10.10, 10.20]),
        "scan_stamp": np.array([10.00, 10.10]),
        "scan_angle_min": np.full(2, -math.pi),
        "scan_angle_increment": np.full(2, 2 * math.pi / n),
        "scan_range_min": np.full(2, 0.05),
        "scan_range_max": np.full(2, 16.0),
        "scan_q": q,
        "odom_t": np.array([10.00, 10.05, 10.10, 10.15, 10.20]),
        "odom_v": np.array([0.2, 0.4, 1.0, math.nan, 0.3]),
        "trk_t": np.array([10.13]),
        "trk_stamp": np.array([10.00]),
        "trk_x": np.arange(16, dtype=np.float32)[None] * 0.2,
        "trk_y": np.zeros((1, 16), np.float32),
        "trk_hl": np.full((1, 16), 0.5, np.float32),
        "trk_hr": np.full((1, 16), 0.5, np.float32),
        "trk_sigma": np.full((1, 16), 0.02, np.float32),
        "trk_valid": np.array([12]),
        "trk_residual": np.array([math.nan]),
        "trk_intrusion": np.array([0.125]),
    }


def parse_stream(lines: list[str]) -> list[tuple[str, list[float]]]:
    out = []
    for line in lines:
        kind, *rest = line.split()
        out.append((kind, [float(x) for x in rest]))
    return out


def test_ranges_round_trip_through_uint16_quarter_millimetres() -> None:
    r = np.array([0.0, 0.05, 1.0, 3.14175, 16.0, math.inf], dtype=np.float32)
    q = bag.quantise_ranges(r)
    assert q.dtype == np.uint16
    back = bag.dequantise_ranges(q)
    assert back[1:5].tolist() == r[1:5].tolist()  # exact, not close
    assert math.isinf(back[5])
    assert math.isinf(back[0])  # 0 m is not a return


def test_ranges_off_the_grid_are_refused_not_rounded() -> None:
    with pytest.raises(ValueError, match="0.25 mm grid"):
        bag.quantise_ranges(np.array([1.00013], dtype=np.float32))
    with pytest.raises(ValueError, match="does not fit"):
        bag.quantise_ranges(np.array([20.0], dtype=np.float32))


def test_stream_has_every_event_in_arrival_order_and_ticks_from_the_start() -> None:
    rec = tiny_recording()
    lines = planner_stream.stream_lines(
        rec, 10.10, 10.20, tracks_mode="recorded", preroll_s=0.1, tick_s=0.02
    )
    events = parse_stream(lines)
    kinds = [k for k, _ in events]
    assert kinds.count("K") == 6  # 10.10, 10.12, ... 10.20
    assert kinds.count("S") == 2 and kinds.count("T") == 1
    arrivals = [v[0] for _, v in events]
    assert arrivals == sorted(arrivals)
    # at the same instant an event comes before the tick: S (10.10) before K 10.10
    first_k = kinds.index("K")
    assert events[first_k][1][0] == pytest.approx(10.10)
    assert "S" in kinds[:first_k]
    scan = next(v for k, v in events if k == "S")
    assert scan[1] == pytest.approx(10.00)  # the header stamp, not the arrival
    assert int(scan[6]) == 8
    assert np.array(scan[7:])[[2, 6]].tolist() == [math.inf, math.inf]
    track = next(v for k, v in events if k == "T")
    assert track[0] == pytest.approx(10.13) and track[1] == pytest.approx(10.00)
    assert track[2] == 12 and math.isnan(track[3]) and track[4] == pytest.approx(0.125)
    assert len(track) == 5 + 5 * 16
    assert track[5 + 3] == pytest.approx(0.6)  # x of station 3
    odom = [v for k, v in events if k == "O"]
    assert any(math.isnan(v[1]) for v in odom)  # NaN is passed through for the CLI to drop


def test_stream_scan_ranges_survive_the_text_round_trip_exactly() -> None:
    rec = tiny_recording()
    lines = planner_stream.stream_lines(rec, 10.10, 10.20, tracks_mode="none")
    scan = next(v for k, v in parse_stream(lines) if k == "S")
    expected = bag.dequantise_ranges(rec["scan_q"][0])
    assert np.array(scan[7:], dtype=np.float32).tolist() == expected.tolist()


def test_stream_tracks_modes_and_speed_clip() -> None:
    rec = tiny_recording()
    none = planner_stream.stream_lines(rec, 10.10, 10.20, tracks_mode="none")
    assert "T" not in {k for k, _ in parse_stream(none)}
    tracks = perception.Tracks(
        scan_index=np.array([0, 1]),
        x=np.zeros((2, 16), np.float32),
        y=np.zeros((2, 16), np.float32),
        hl=np.full((2, 16), 0.4, np.float32),
        hr=np.full((2, 16), 0.4, np.float32),
        sigma=np.full((2, 16), 0.01, np.float32),
        valid=np.array([16, 3]),
        residual=np.array([0.01, 0.02]),
        intrusion=np.array([0.0, 0.5]),
    )
    replay = planner_stream.stream_lines(
        rec, 10.10, 10.20, tracks_mode="replay", tracks=tracks, track_latency_s=0.03
    )
    ts = [v for k, v in parse_stream(replay) if k == "T"]
    assert ts[0][0] == pytest.approx(10.13)  # scan arrival 10.10 + 0.03
    assert ts[0][1] == pytest.approx(10.00)  # the scan's header stamp
    assert len(ts) == 1  # the second track would arrive at 10.23, after the segment
    with pytest.raises(ValueError, match="needs the replayed tracks"):
        planner_stream.stream_lines(rec, 10.10, 10.20, tracks_mode="replay")
    with pytest.raises(ValueError, match="tracks must be"):
        planner_stream.stream_lines(rec, 10.10, 10.20, tracks_mode="bogus")
    clipped = planner_stream.stream_lines(rec, 10.10, 10.20, tracks_mode="none", clip_speed_mps=0.4)
    speeds = [v[1] for k, v in parse_stream(clipped) if k == "O" and not math.isnan(v[1])]
    assert max(speeds) == pytest.approx(0.4) and min(speeds) == pytest.approx(0.2)


def test_cli_command_native_and_in_docker(tmp_path: Path) -> None:
    binary = tmp_path / "bin" / "planner_replay_cli"
    binary.parent.mkdir()
    binary.write_text("")
    work = tmp_path / "work"
    work.mkdir()
    args = ["--bundle", str(DYNAMICS_BUNDLE), "--stream", str(work / "s.txt"), "--v-max", "0.4"]
    assert planner_stream.Cli(binary).command(args) == [str(binary), *args]
    cmd = planner_stream.Cli(binary, "ros-dev:local").command(args)
    assert cmd[:3] == ["docker", "run", "--rm"]
    assert f"{work}:{work}" in cmd and f"{binary.parent}:{binary.parent}" in cmd
    assert cmd[-len(args) :] == args and "ros-dev:local" in cmd


def test_set_args_are_sorted_and_booleans_spelled_for_the_cli() -> None:
    args = planner_stream.set_args(
        {"samples": 480, "laser_yaw_from_vehicle_params": False, "a": 0.1}
    )
    assert args == [
        "--set", "a=0.1", "--set", "laser_yaw_from_vehicle_params=false", "--set", "samples=480"
    ]  # fmt: skip


# --- the report ------------------------------------------------------------------------------


def test_report_flatten_table_compare_and_json(tmp_path: Path) -> None:
    seg = {
        "kind": "pnp",
        "t_from": 1.0,
        "t_to": 3.0,
        "duration_s": 2.0,
        "perception": metrics.perception_metrics(
            np.array([16, 16]), np.array([0.1, 0.3]), np.array([0.1, 0.2]), np.array([0.0, 0.2])
        ),
        "planner": {
            "eligible_ticks": 10,
            "state_counts": {"DRIVING": 7, "NO_TRACK": 3},
            "steering_change": {"std_rad": 0.02},
            "agreement_with_recorded": {"share": math.nan},
        },
    }
    flat = report.flatten(seg)
    assert flat["perception.intrusion_mean"] == pytest.approx(0.2)
    assert flat["planner.state_counts.DRIVING"] == 7
    assert math.isnan(flat["planner.agreement_with_recorded.share"])
    result = {"backend": "geometric", "tracks": "replay", "v_max_mps": 0.4, "segments": {"R1": seg}}
    table = report.format_table(result)
    assert "intrusion mean" in table and "DRIVING" in table
    base = json.loads(json.dumps({"segments": {"R1": {**seg, "planner": {"eligible_ticks": 4}}}},
                                 default=str))  # fmt: skip
    compared = report.format_table(result, base)
    assert "baseline" in compared and "10" in compared and "4" in compared
    path = tmp_path / "out" / "r.json"
    report.write_json(result, path)
    loaded = json.loads(path.read_text())  # NaN became null: strict JSON
    assert loaded["segments"]["R1"]["planner"]["agreement_with_recorded"]["share"] is None


# --- the fixture -----------------------------------------------------------------------------


def test_fixture_is_small_and_has_the_four_cuts() -> None:
    assert FIXTURE.stat().st_size < 450_000
    for name in ("R0", "R1", "R3", "L"):
        rec, a, b = cut_fixture.load_cut(FIXTURE, name)
        assert b > a
        assert len(rec["scan_t"]) >= (b - a) * 9  # 10 Hz
        assert len(rec["odom_t"]) >= (b - a) * 40
        assert rec["scan_t"].min() >= a - planner_stream.PREROLL_S - 1e-9
    r0, _, _ = cut_fixture.load_cut(FIXTURE, "R0")
    assert len(r0["st_t"]) > 0 and len(r0["trk_t"]) > 0
    lap, _, _ = cut_fixture.load_cut(FIXTURE, "L")
    assert len(lap["st_t"]) == 0 and len(lap["trk_t"]) == 0  # not a PnP run


def test_cut_windows_follow_the_segments() -> None:
    found = [
        segments.Segment("R0", "pnp", 472.6, 491.5),
        segments.Segment("R1", "pnp", 1087.2, 1123.4),
        segments.Segment("R3", "pnp", 1174.1, 1188.9),
        segments.Segment("L", "fast", 1749.0, 1923.0),
    ]
    w = cut_fixture.windows(found)
    assert w["R0"] == pytest.approx((475.6, 480.1))
    assert w["R1"] == pytest.approx((1093.2, 1099.2))
    assert w["R3"] == pytest.approx((1178.1, 1180.1))
    assert w["L"] == (1760.0, 1770.0)
    with pytest.raises(ValueError, match="does not hold"):
        cut_fixture.windows([*found[:3], segments.Segment("L", "fast", 1800.0, 1900.0)])


@pytest.fixture(scope="module")
def perceiver() -> perception.Perceiver:
    return perception.build_perceiver("geometric")


def _v0_tracer_module():
    """The frozen geometric-v0 tracer (tools/tests/fixtures/geometric_v0_tracer.py): the fixture's
    recorded /pnp/local_track was made by it, main's tracer is geometric-v1 since #145."""
    import importlib.util
    import sys

    spec = importlib.util.spec_from_file_location(
        "geometric_v0_tracer", REPO_ROOT / "tools" / "tests" / "fixtures" / "geometric_v0_tracer.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # its dataclasses look their module up by name
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def v0_perceiver(monkeypatch: pytest.MonkeyPatch) -> perception.Perceiver:
    """A perceiver that traces with the frozen v0 tracer (the core module's `geometric` is swapped
    for the length of the test; everything else is the current perception code)."""
    core = perception._import_core()
    monkeypatch.setattr(core, "geometric", _v0_tracer_module())
    p = perception.build_perceiver("geometric")
    p.model_id = "geometric-v0"
    return p


@pytest.mark.parametrize("name", ["R0", "R1", "R3"])
def test_replayed_v0_tracer_equals_the_recorded_track(
    name: str, v0_perceiver: perception.Perceiver
) -> None:
    """The harness self-test: the replayed geometric tracer gives the recorded /pnp/local_track,
    matched by header stamp, to 1e-5 m (on this bag the difference is exactly 0)."""
    rec, a, b = cut_fixture.load_cut(FIXTURE, name)
    scans = planner_stream.scan_indices(rec, a - planner_stream.PREROLL_S, b)
    tracks = perception.run_perceiver(v0_perceiver, rec, scans)
    fid = perception.check_fidelity(rec, tracks, a, b)
    assert fid.matched >= 15, fid
    assert fid.valid_count_mismatches == 0
    assert fid.max_centreline_diff_m <= 1e-5
    assert fid.max_width_diff_m <= 1e-5
    assert fid.ok


def test_harness_runs_the_current_tracer(perceiver: perception.Perceiver) -> None:
    """The harness itself traces with the current geometric tracer (the one on main), not the frozen
    v0 copy the fidelity tests use; the two are different implementations and give different tracks
    on the fixture's scans, so a recorded v0 track is no longer reproduced by the harness."""
    core = perception._import_core()
    from racer_track_perception import geometric

    assert core.geometric is geometric  # the production module, not the test copy
    assert perceiver.model_id == core.GEOMETRIC_MODEL_ID == "geometric-v1"
    rec, a, b = cut_fixture.load_cut(FIXTURE, "R1")
    scans = planner_stream.scan_indices(rec, a - planner_stream.PREROLL_S, b)
    current = perception.run_perceiver(perceiver, rec, scans)
    assert not perception.check_fidelity(rec, current, a, b).ok  # the recording is v0's


def test_fidelity_check_fails_on_a_changed_track(perceiver: perception.Perceiver) -> None:
    rec, a, b = cut_fixture.load_cut(FIXTURE, "R1")
    scans = planner_stream.scan_indices(rec, a - planner_stream.PREROLL_S, b)
    tracks = perception.run_perceiver(perceiver, rec, scans)
    tampered = {**rec, "trk_y": rec["trk_y"] + np.float32(1e-3)}
    fid = perception.check_fidelity(tampered, tracks, a, b)
    assert not fid.ok and fid.max_centreline_diff_m > 1e-4
    # a replay of the wrong scans matches nothing, which is a failure and not a pass
    nothing = perception.run_perceiver(perceiver, rec, scans[:0])
    assert not perception.check_fidelity(rec, nothing, a, b).ok


def test_perception_summary_counts_only_moving_scans(perceiver: perception.Perceiver) -> None:
    rec, a, b = cut_fixture.load_cut(FIXTURE, "L")
    scans = planner_stream.scan_indices(rec, a, b)
    tracks = perception.run_perceiver(perceiver, rec, scans)
    summary, arrays = perception.perception_summary(rec, tracks)
    assert summary["scans"] == len(scans)  # another driver's laps: moving throughout
    assert len(arrays["valid"]) == summary["scans"]
    stride, _ = perception.perception_summary(rec, tracks, stride=5)
    assert stride["scans"] == math.ceil(len(scans) / 5)
    still, _ = perception.perception_summary(rec, tracks, moving_threshold_mps=99.0)
    assert still["scans"] == 0


# --- the planner replay, end to end (needs planner_replay_cli) -------------------------------


def _cli_or_skip() -> planner_stream.Cli:
    try:
        return planner_stream.find_cli()
    except planner_stream.ReplayError as error:
        if os.environ.get("PNP_REPLAY_REQUIRE_CLI"):
            pytest.fail(f"planner_replay_cli is required here but not built: {error}")
        pytest.skip(f"planner_replay_cli not built ({error})")


# Deterministic: the sample budget halves a slow solve, and a loaded CI machine has slow solves.
DETERMINISTIC = {"solve_halve_over_ms": 200.0}


def replay_cut(
    name: str,
    tmp_path: Path,
    tracks_mode: str,
    params: dict[str, float | bool],
    tracks: perception.Tracks | None = None,
) -> tuple[dict, dict]:
    cli = _cli_or_skip()
    rec, a, b = cut_fixture.load_cut(FIXTURE, name)
    lines = planner_stream.stream_lines(rec, a, b, tracks_mode=tracks_mode, tracks=tracks)
    unique = len(list(tmp_path.iterdir()))  # a fresh file name per call
    stream = tmp_path / f"{name}.{unique}.stream"
    planner_stream.write_stream(lines, stream)
    rows = planner_stream.run_cli(
        cli, DYNAMICS_BUNDLE, 0.4, {**DETERMINISTIC, **params}, stream, tmp_path / f"{unique}.csv"
    )
    recorded = (rec["st_t"] >= a) & (rec["st_t"] <= b)
    m = metrics.planner_metrics(
        rows["now_s"], rows["state"], rows["cmd_speed_mps"], rows["cmd_steering_rad"],
        rows["collision_fraction"], rows["solve_time_s"], rows["scan_age_s"], rec["odom_t"],
        recorded_t=rec["st_t"][recorded] if recorded.any() else None,
        recorded_state=rec["st_state"][recorded].astype(int) if recorded.any() else None,
        track_confidence=rows.get("track_confidence"),
        block_reason=rows["block_reason"].astype(int) if "block_reason" in rows else None,
        freespace_goal_x_m=rows.get("freespace_goal_x_m"),
    )  # fmt: skip
    return rows, m


# An open-loop replay: the recorded wheel does not follow the replayed commands, so the wheel stall
# detector (reason 3) would fire whenever the recorded car stood still. It is set out of reach.
OPEN_LOOP = {"stall_hold_s": 600.0}
DEAD_ZONE = 0.20


def assert_only_start_ramp_ticks_are_in_the_dead_zone(rows: dict) -> None:
    """No command strictly between 0 and the dead zone except the one ramp tick of a start from
    rest (0.45 of the vehicle's acceleration limit per tick, so at most 0.09 m/s)."""
    v = rows["cmd_speed_mps"]
    inside = np.nonzero((v > 0.0) & (v < DEAD_ZONE))[0]
    for i in inside:
        before = v[i - 1] if i > 0 else 0.0  # a replay starts from rest
        assert before == 0.0, f"tick {i}: {before} -> {v[i]}"
        assert v[i] <= 0.09, v[i]
        if i + 1 < len(v):
            assert not (0.0 < v[i + 1] < DEAD_ZONE)  # one tick only: out of it (or stopped) next


def test_planner_replay_r0_drives_on_the_scan_where_v0_went_to_no_track(tmp_path: Path) -> None:
    rows, m = replay_cut("R0", tmp_path, "recorded", dict(OPEN_LOOP))
    assert len(rows["now_s"]) == pytest.approx((7.5 - 3.0) / 0.02, abs=2)
    assert m["state_counts"].get("NO_TRACK", 0) == 0
    assert m["state_counts"]["DRIVING"] >= 0.98 * m["eligible_ticks"]
    assert m["speed_zero_share"] < 0.02  # v0: most of the run
    assert_only_start_ramp_ticks_are_in_the_dead_zone(rows)
    assert m["speed_below_dead_zone_share"] < 0.01
    assert m["speed_negative_outside_recovering"] == 0
    assert m["freespace_goal_share"] > 0.95


def test_planner_replay_r1_moves_at_the_floor_or_backs_out_at_the_recorded_corner(
    tmp_path: Path,
) -> None:
    rows, m = replay_cut("R1", tmp_path, "recorded", dict(OPEN_LOOP))
    assert m["state_counts"].get("NO_TRACK", 0) == 0
    assert_only_start_ramp_ticks_are_in_the_dead_zone(rows)  # v0: crawled under the dead zone
    # every tick is a move at or above the floor, a stop (BLOCKED with a reason), or a reverse
    cmd = rows["cmd_speed_mps"]
    state = rows["state"]
    assert set(state.tolist()) <= {1, 5, 7}
    assert (cmd[state == 7] <= 0.0).all()
    assert (cmd[(state != 7) & (state != 1)] == 0.0).all()


def test_planner_replay_r3_nose_in_backs_out(tmp_path: Path) -> None:
    rows, m = replay_cut("R3", tmp_path, "recorded", dict(OPEN_LOOP))
    assert m["state_counts"].get("NO_TRACK", 0) == 0
    assert_only_start_ramp_ticks_are_in_the_dead_zone(rows)
    assert m["state_counts"].get("RECOVERING", 0) > 0  # no collision free forward arc: reverse
    assert m["recovering_ticks"] == m["state_counts"]["RECOVERING"]
    assert (rows["cmd_speed_mps"][rows["state"] == 7] <= 0.0).all()
    assert m["speed_negative_outside_recovering"] == 0


def test_planner_replay_without_tracks_drives_on_the_scan_and_with_replayed_tracks_too(
    tmp_path: Path, perceiver: perception.Perceiver
) -> None:
    rows, m = replay_cut("L", tmp_path, "none", dict(OPEN_LOOP))
    assert 2 not in set(rows["state"])  # NO_TRACK is never produced
    assert set(rows["state"].tolist()) <= {1, 5, 7}  # it drives, holds with a reason, or backs out
    assert m["track_confidence_zero_share"] == 1.0
    assert_only_start_ramp_ticks_are_in_the_dead_zone(rows)
    rec, a, b = cut_fixture.load_cut(FIXTURE, "L")
    scans = planner_stream.scan_indices(rec, a - planner_stream.PREROLL_S, b)
    tracks = perception.run_perceiver(perceiver, rec, scans)
    rows, m = replay_cut("L", tmp_path, "replay", dict(OPEN_LOOP), tracks)
    assert m["state_counts"].get("DRIVING", 0) > 100
    assert max(rows["cmd_speed_mps"]) > 0.0
    assert 6 not in set(rows["state"])  # no FAULT
    assert m["track_confidence_mean"] > 0.0


def test_planner_replay_is_repeatable(tmp_path: Path) -> None:
    params = {**OPEN_LOOP, "conf_intrusion_zero": 0.20}
    a, _ = replay_cut("R1", tmp_path, "recorded", params)
    b, _ = replay_cut("R1", tmp_path, "recorded", params)
    assert a["cmd_steering_rad"].tolist() == b["cmd_steering_rad"].tolist()
    assert a["state"].tolist() == b["state"].tolist()


def test_planner_replay_refuses_the_removed_self_check_parameters(tmp_path: Path) -> None:
    cli = _cli_or_skip()
    stream = tmp_path / "empty.stream"
    stream.write_text("")
    with pytest.raises(planner_stream.ReplayError, match="unknown parameter"):
        planner_stream.run_cli(
            cli, DYNAMICS_BUNDLE, 0.4, {"max_intrusion_fraction": 0.2}, stream, tmp_path / "o.csv"
        )


def test_planner_replay_refuses_a_bundle_that_fails_a_check(tmp_path: Path) -> None:
    cli = _cli_or_skip()
    bad = tmp_path / "bad_bundle"
    bad.mkdir()
    stream = tmp_path / "empty.stream"
    stream.write_text("")
    with pytest.raises(planner_stream.ReplayError, match="ManifestMissing"):
        planner_stream.run_cli(cli, bad, 0.4, {}, stream, tmp_path / "o.csv")


# --- the command line ------------------------------------------------------------------------


def _script():
    """tools/pnp_replay.py as a module (the package of the same name wins `import pnp_replay`)."""
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "pnp_replay_script", REPO_ROOT / "tools" / "pnp_replay.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def r1_recording(tmp_path_factory: pytest.TempPathFactory) -> tuple[Path, float, float]:
    """The fixture's R1 cut saved as an ordinary recording .npz, the way a bag's cache is."""
    rec, a, b = cut_fixture.load_cut(FIXTURE, "R1")
    path = tmp_path_factory.mktemp("rec") / "r1.npz"
    bag.save_recording(rec, path)
    return path, a, b


def test_cli_perception_only_run_writes_json_and_passes_the_fidelity_check(
    r1_recording: tuple[Path, float, float],
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
    v0_perceiver: perception.Perceiver,  # swaps in the frozen v0 tracer the bag was recorded with
) -> None:
    path, a, b = r1_recording
    out = tmp_path / "r1.json"
    code = _script().main(
        [str(path), "--from", str(a), "--to", str(b), "--no-planner", "--check-fidelity",
         "--out", str(out)]
    )  # fmt: skip
    assert code == 0
    result = json.loads(out.read_text())
    seg = result["segments"]["W"]
    assert seg["fidelity"]["ok"] and seg["fidelity"]["max_centreline_diff_m"] <= 1e-5
    assert seg["perception"]["scans"] > 0 and "planner" not in seg
    assert "intrusion mean" in capsys.readouterr().out


def test_cli_usage_errors_exit_2_and_say_why(
    r1_recording: tuple[Path, float, float], tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    path, a, b = r1_recording
    main = _script().main
    out = ["--out", str(tmp_path / "x.json")]
    assert main([str(path), "--segment", "R9", "--no-planner", *out]) == 2
    assert "no segment R9" in capsys.readouterr().err
    assert main([str(path), "--from", str(a), "--to", str(b), *out]) == 2  # no bundle
    assert "--dynamics-bundle is required" in capsys.readouterr().err
    code = main([str(path), "--from", str(a), "--to", str(b), "--no-planner", "--check-fidelity",
                 "--backend", "net", *out])  # fmt: skip
    assert code == 2
    assert "--backend geometric" in capsys.readouterr().err
    assert main([str(tmp_path / "missing.npz"), "--no-planner", *out]) == 2


def test_cli_planner_run_compares_with_a_baseline(
    r1_recording: tuple[Path, float, float], tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    _cli_or_skip()
    path, a, b = r1_recording
    main = _script().main
    common = [str(path), "--from", str(a), "--to", str(b), "--tracks", "recorded",
              "--dynamics-bundle", str(DYNAMICS_BUNDLE), "--v-max", "0.4",
              "--set", "solve_halve_over_ms=200"]  # fmt: skip
    first = tmp_path / "first.json"
    assert main([*common, "--out", str(first)]) == 0
    capsys.readouterr()
    second = tmp_path / "second.json"
    assert main([*common, "--out", str(second), "--compare", str(first)]) == 0
    table = capsys.readouterr().out
    assert "baseline" in table and "DRIVING" in table
    planner = json.loads(second.read_text())["segments"]["W"]["planner"]
    assert planner["state_counts"]["DRIVING"] > 100
    assert planner["parameters"] == {"solve_halve_over_ms": 200.0}
