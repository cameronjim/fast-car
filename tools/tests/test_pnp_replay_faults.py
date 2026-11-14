"""Fault injection on the Plug-and-Play replay (roadmap P3.4, 12-testing.md layer L4).

Three parts:

1. The mutations of tools/pnp_replay/faults.py, unit tested on a small synthetic recording: what
   changed, what did not, determinism under the seed, the input untouched.
2. The rules of tools/pnp_replay/fault_checks.py, tested on hand-made data (a rule that cannot fail
   is not a rule).
3. Every fault against both consumers on the committed fixture
   (tools/tests/fixtures/pnp_fault_2026-11-13.npz, three cuts of bag 2026-11-13T22-20-18): one test
   id per (fault, consumer, variant), `test_perception[...]` for the perception core
   (`TrackPerceiver.process`, what the node calls) and `test_planner[...]` for the real planner
   through planner_replay_cli (skipped, with the reason, unless the binary is built; the CI job
   that builds it sets PNP_REPLAY_REQUIRE_CLI=1 so there a missing binary is a failure).

Open loop: the scans do not react to the commands. The replay shows what the planner would have
commanded at each recorded instant under the fault, not a crash or a lap.

Findings against the planner on main are strict xfails below, each with the bag time and what was
commanded. A frozen scan with fresh stamps, and every wheel odometry fault but silence, is a fault
the planner cannot see (plausible ranges and a fresh header; odometry stamped on arrival): those
tests check the always rules and say so, they do not claim a detection.
"""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from pnp_replay import (
    bag,
    cut_fixture,
    fault_checks,
    fault_run,
    faults,
    perception,
    planner_stream,
    vehicle,
)

TESTS = Path(__file__).resolve().parent
REPO_ROOT = TESTS.parents[1]
FIXTURE = TESTS / "fixtures" / "pnp_fault_2026-11-13.npz"
CAR_BUNDLE = REPO_ROOT / "config" / "pnp" / "fixtures" / "dyn-car-2026-11-12"
SEED = 20261113


# --- a small synthetic recording -------------------------------------------------------------


def make_recording(n_scans: int = 20, n_beams: int = 12) -> bag.Recording:
    """Scans at 10 Hz (header stamp 0.1 s before arrival), wheel odometry at 50 Hz, tracks at
    10 Hz. Every value differs from the next so a swap or a repeat shows."""
    scan_t = 10.0 + 0.1 * np.arange(n_scans)
    rng = np.random.default_rng(1)
    q = (rng.integers(400, 8000, size=(n_scans, n_beams)) * 4).astype(np.uint16)  # 0.1 to 2 m
    odom_t = 10.0 + 0.02 * np.arange(5 * n_scans)
    trk_t = scan_t + 0.03
    return {
        "t0_log_s": np.array(100.0),
        "cache_version": np.array(bag.CACHE_VERSION),
        "scan_t": scan_t,
        "scan_stamp": scan_t - 0.1,
        "scan_angle_min": np.full(n_scans, -math.pi),
        "scan_angle_increment": np.full(n_scans, 2 * math.pi / n_beams),
        "scan_range_min": np.full(n_scans, 0.05),
        "scan_range_max": np.full(n_scans, 12.0),
        "scan_q": q,
        "odom_t": odom_t,
        "odom_v": 0.5 + 0.001 * np.arange(len(odom_t)),
        "trk_t": trk_t,
        "trk_stamp": scan_t - 0.1,
        "trk_x": np.tile(np.arange(16, dtype=np.float32) * 0.2, (n_scans, 1)),
        "trk_y": np.zeros((n_scans, 16), np.float32),
        "trk_hl": np.full((n_scans, 16), 0.5, np.float32),
        "trk_hr": np.full((n_scans, 16), 0.5, np.float32),
        "trk_sigma": np.full((n_scans, 16), 0.02, np.float32),
        "trk_valid": np.full(n_scans, 16),
        "trk_residual": np.full(n_scans, 0.05),
        "trk_intrusion": np.full(n_scans, 0.01),
    }


def assert_same(a: bag.Recording, b: bag.Recording, keys: list[str] | None = None) -> None:
    for k in keys if keys is not None else a:
        assert np.array_equal(a[k], b[k], equal_nan=True), k


def snapshot(rec: bag.Recording) -> bag.Recording:
    return {k: np.array(v, copy=True) for k, v in rec.items()}


# --- part 1: the mutations -------------------------------------------------------------------


def test_every_mutation_leaves_its_input_untouched_and_is_deterministic() -> None:
    rec = make_recording()
    before = snapshot(rec)
    for fault in faults.SCAN_FAULTS:
        params = {"fraction": 0.5, "fill": "nan"} if fault == "nan_ranges" else {}
        a = faults.apply(rec, fault, 10.5, 11.2, SEED, **params)
        b = faults.apply(rec, fault, 10.5, 11.2, SEED, **params)
        assert_same(rec, before)  # not mutated in place
        assert set(a) >= set(rec)
        assert_same(a, b, [k for k in a if k in b])  # same seed, same result
    kinds = ("nan", "valid_out_of_range")
    for kind in kinds:
        faults.track_garbage(rec, 10.5, 11.2, SEED, kind=kind)
    assert_same(rec, before)


def test_nan_ranges_scan_hits_the_share_of_beams_in_the_window_only() -> None:
    rec = make_recording(n_beams=40)
    t0, t1 = 10.5, 11.2
    for fraction in faults.NAN_FRACTIONS:
        for fill, check in (("nan", np.isnan), ("inf", np.isinf), ("zero", lambda x: x == 0.0)):
            out = faults.nan_ranges(rec, t0, t1, SEED, fraction, fill, targets=("scan",))
            idx = faults.rows_in(rec, "scan", t0, t1)
            assert len(idx) == 8
            for i in range(len(rec["scan_t"])):
                r = bag.scan_ranges(out, i)
                bad = check(r)
                if i in idx:
                    assert bad.sum() == round(fraction * 40), (fraction, fill, i)
                else:
                    assert not bad.any() or fill == "inf"  # a recording's 0 m is already inf
            outside = np.setdiff1d(np.arange(len(rec["scan_t"])), idx)
            for i in outside:  # untouched rows are bit for bit the original float ranges
                assert np.array_equal(bag.scan_ranges(out, int(i)), bag.scan_ranges(rec, int(i)))
            assert_same(out, rec, ["scan_t", "scan_stamp", "odom_v", "odom_t", "trk_x"])
    # a fresh random choice per scan: two scans do not lose the same beams
    out = faults.nan_ranges(rec, t0, t1, SEED, 0.5, "nan", targets=("scan",))
    masks = [np.isnan(bag.scan_ranges(out, int(i))) for i in faults.rows_in(rec, "scan", t0, t1)]
    assert not all(np.array_equal(masks[0], m) for m in masks[1:])
    other = faults.nan_ranges(rec, t0, t1, SEED + 1, 0.5, "nan", targets=("scan",))
    assert not np.array_equal(np.isnan(other["scan_f"]), np.isnan(out["scan_f"]))


def test_nan_ranges_odometry_sets_the_speed_nan_in_the_window_only() -> None:
    rec = make_recording()
    out = faults.nan_ranges(rec, 10.5, 11.2, SEED, targets=("odom",))
    inside = (rec["odom_t"] >= 10.5) & (rec["odom_t"] <= 11.2)
    assert np.isnan(out["odom_v"][inside]).all() and not np.isnan(out["odom_v"][~inside]).any()
    assert np.array_equal(out["odom_v"][~inside], rec["odom_v"][~inside])
    assert "scan_f" not in out  # the scan stream was not asked for
    assert_same(out, rec, ["scan_t", "scan_q", "odom_t"])


def test_drop_removes_every_row_of_the_window_from_the_targeted_streams() -> None:
    rec = make_recording()
    out = faults.drop(rec, 10.5, 11.2, SEED, targets=("scan",))
    inside = (rec["scan_t"] >= 10.5) & (rec["scan_t"] <= 11.2)
    assert len(out["scan_t"]) == len(rec["scan_t"]) - inside.sum() == 12
    for key in ("scan_t", "scan_stamp", "scan_q", "scan_angle_min"):
        assert np.array_equal(out[key], rec[key][~inside]), key
    assert_same(out, rec, ["odom_t", "odom_v", "trk_t", "trk_x"])  # other streams intact
    both = faults.drop(rec, 10.5, 11.2, SEED)
    odd = (rec["odom_t"] >= 10.5) & (rec["odom_t"] <= 11.2)
    assert len(both["odom_t"]) == len(rec["odom_t"]) - odd.sum()
    assert np.array_equal(both["odom_v"], rec["odom_v"][~odd])


def test_window_edges_are_inclusive_and_an_empty_window_changes_nothing() -> None:
    rec = make_recording()
    out = faults.drop(rec, rec["scan_t"][3], rec["scan_t"][5], SEED, targets=("scan",))
    assert len(out["scan_t"]) == len(rec["scan_t"]) - 3  # both end rows go
    none = faults.drop(rec, 50.0, 60.0, SEED)
    assert_same(none, rec)
    with pytest.raises(ValueError, match="window"):
        faults.drop(rec, 11.0, 10.0, SEED)
    with pytest.raises(ValueError, match="window"):
        faults.drop(rec, math.nan, 10.0, SEED)
    with pytest.raises(ValueError, match="targets"):
        faults.drop(rec, 10.0, 11.0, SEED, targets=("lidar",))
    with pytest.raises(ValueError, match="fraction"):
        faults.nan_ranges(rec, 10.0, 11.0, SEED, fraction=0.0)
    with pytest.raises(ValueError, match="fill"):
        faults.nan_ranges(rec, 10.0, 11.0, SEED, fill="huge")
    with pytest.raises(ValueError, match="fault must be"):
        faults.apply(rec, "melt", 10.0, 11.0, SEED)


@pytest.mark.parametrize(("fault", "sign"), [("stamp_jump_forward", +1),
                                              ("stamp_jump_backward", -1)])  # fmt: skip
def test_stamp_jumps_move_the_header_stamps_by_5_s_and_not_the_arrival(
    fault: str, sign: int
) -> None:
    rec = make_recording()
    out = faults.apply(rec, fault, 10.5, 11.2, SEED)
    s_in = (rec["scan_t"] >= 10.5) & (rec["scan_t"] <= 11.2)
    assert np.allclose(out["scan_stamp"][s_in], rec["scan_stamp"][s_in] + sign * 5.0)
    assert np.array_equal(out["scan_stamp"][~s_in], rec["scan_stamp"][~s_in])
    assert_same(out, rec, ["scan_t", "scan_q", "odom_t", "odom_v", "trk_t"])
    o_in = (rec["odom_t"] >= 10.5) & (rec["odom_t"] <= 11.2)
    assert np.allclose(out["odom_stamp"][o_in], rec["odom_t"][o_in] + sign * 5.0)
    assert np.array_equal(out["odom_stamp"][~o_in], rec["odom_t"][~o_in])
    scan_only = faults.apply(rec, fault, 10.5, 11.2, SEED, targets=("scan",))
    assert "odom_stamp" not in scan_only
    assert faults.STAMP_JUMP_S == 5.0


def test_out_of_order_swaps_adjacent_pairs_of_the_window_and_keeps_the_arrival_axis() -> None:
    rec = make_recording()
    out = faults.out_of_order(rec, 10.5, 11.2, SEED, targets=("scan",))
    idx = faults.rows_in(rec, "scan", 10.5, 11.2)  # 8 rows: 4 pairs
    assert len(idx) == 8
    assert np.array_equal(out["scan_t"], rec["scan_t"])  # arrival slots are the same
    for a, b in zip(idx[0::2], idx[1::2], strict=True):
        for key in ("scan_stamp", "scan_q"):
            assert np.array_equal(out[key][a], rec[key][b]), key
            assert np.array_equal(out[key][b], rec[key][a]), key
    rest = np.setdiff1d(np.arange(len(rec["scan_t"])), idx)
    assert np.array_equal(out["scan_q"][rest], rec["scan_q"][rest])
    # the stamps now arrive out of order: some message is older than the one before it
    assert (np.diff(out["scan_stamp"][idx]) < 0).any()
    # an odd window leaves its last row alone
    odd = faults.out_of_order(rec, 10.5, 11.1, SEED, targets=("scan",))
    last = faults.rows_in(rec, "scan", 10.5, 11.1)[-1]
    assert np.array_equal(odd["scan_q"][last], rec["scan_q"][last])
    od = faults.out_of_order(rec, 10.5, 11.2, SEED, targets=("odom",))
    oi = faults.rows_in(rec, "odom", 10.5, 11.2)
    assert (
        od["odom_v"][oi[0]] == rec["odom_v"][oi[1]] and od["odom_v"][oi[1]] == rec["odom_v"][oi[0]]
    )
    assert_same(od, rec, ["odom_t", "scan_q"])


def test_frozen_repeats_the_last_row_before_the_window_with_its_stamp() -> None:
    rec = make_recording()
    out = faults.frozen(rec, 10.5, 11.2, SEED, targets=("scan",))
    idx = faults.rows_in(rec, "scan", 10.5, 11.2)
    src = idx[0] - 1
    for i in idx:
        assert np.array_equal(out["scan_q"][i], rec["scan_q"][src])
        assert out["scan_stamp"][i] == rec["scan_stamp"][src]  # the stamp is frozen too
    assert np.array_equal(out["scan_t"], rec["scan_t"])  # arrival at the scan rate
    rest = np.setdiff1d(np.arange(len(rec["scan_t"])), idx)
    assert np.array_equal(out["scan_q"][rest], rec["scan_q"][rest])
    od = faults.frozen(rec, 10.5, 11.2, SEED, targets=("odom",))
    oi = faults.rows_in(rec, "odom", 10.5, 11.2)
    assert (od["odom_v"][oi] == rec["odom_v"][oi[0] - 1]).all()
    assert np.array_equal(od["odom_t"], rec["odom_t"])


def test_frozen_fresh_stamp_repeats_the_ranges_and_keeps_the_stamps_advancing() -> None:
    rec = make_recording()
    out = faults.frozen_fresh_stamp(rec, 10.5, 11.2, SEED, targets=("scan",))
    idx = faults.rows_in(rec, "scan", 10.5, 11.2)
    src = idx[0] - 1
    for i in idx:
        assert np.array_equal(out["scan_q"][i], rec["scan_q"][src])
    assert np.array_equal(out["scan_stamp"], rec["scan_stamp"])  # stamps advance as recorded
    assert (np.diff(out["scan_stamp"]) > 0).all()
    od = faults.frozen_fresh_stamp(rec, 10.5, 11.2, SEED)
    oi = faults.rows_in(rec, "odom", 10.5, 11.2)
    assert (od["odom_v"][oi] == rec["odom_v"][oi[0] - 1]).all()
    assert np.array_equal(od["odom_stamp"], rec["odom_t"])  # the stamp keeps advancing


def test_a_window_at_the_start_freezes_on_its_own_first_row() -> None:
    rec = make_recording()
    out = faults.frozen(rec, 0.0, 10.25, SEED, targets=("scan",))  # rows 0..2, no row before
    assert (out["scan_q"][:3] == rec["scan_q"][0]).all()


def test_track_garbage_replaces_the_tracks_that_arrive_in_the_window() -> None:
    rec = make_recording()
    idx = faults.rows_in(rec, "trk", 10.5, 11.2)
    nan = faults.track_garbage(rec, 10.5, 11.2, SEED, kind="nan")
    for key in ("trk_x", "trk_y", "trk_hl", "trk_hr", "trk_sigma"):
        assert np.isnan(nan[key][idx]).all(), key
        rest = np.setdiff1d(np.arange(len(rec["trk_t"])), idx)
        assert np.array_equal(nan[key][rest], rec[key][rest])
    assert np.isnan(nan["trk_residual"][idx]).all() and (nan["trk_valid"] == 16).all()
    bad = faults.track_garbage(rec, 10.5, 11.2, SEED, kind="valid_out_of_range")
    assert (bad["trk_valid"][idx] == 17).all() and (bad["trk_valid"][:2] == 16).all()
    assert_same(bad, rec, ["trk_x", "trk_t", "scan_q", "odom_v"])


@pytest.mark.parametrize("kind", faults.TRACK_GARBAGE_NOT_INJECTABLE)
def test_track_garbage_of_a_length_frame_or_spacing_the_stream_cannot_carry_is_refused(
    kind: str,
) -> None:
    """planner_replay_cli's T line is 16 stations, frame base_link, spacing 0.20 m. These three
    faults are covered by the node's own validity check (local_track.cpp: WrongSize, WrongFrame,
    WrongSpacing) and its gtest, not by a replay; asking for them says so instead of sending
    something else."""
    with pytest.raises(faults.NotInjectable, match="stream"):
        faults.track_garbage(make_recording(), 10.0, 11.0, SEED, kind=kind)
    with pytest.raises(ValueError, match="kind must be"):
        faults.track_garbage(make_recording(), 10.0, 11.0, SEED, kind="melted")


def test_window_helpers_hold_exactly_the_scans_or_samples_asked_for() -> None:
    rec = make_recording()
    for n in (1, 3):
        t0, t1 = faults.window_of_scans(rec, 10.45, n)
        assert len(faults.rows_in(rec, "scan", t0, t1)) == n
        assert t0 == pytest.approx(10.5)
    with pytest.raises(ValueError, match="fewer than"):
        faults.window_of_scans(rec, 11.95, 3)
    a, b = faults.window_of_odom(rec, 10.5, 0.06)
    assert len(faults.rows_in(rec, "odom", a, b)) == 3  # 60 ms at 50 Hz
    a, b = faults.window_of_odom(rec, 10.5, 1.0)
    assert len(faults.rows_in(rec, "odom", a, b)) == 50


def test_a_mutated_recording_feeds_the_stream_with_nan_and_the_frozen_stamp() -> None:
    rec = make_recording()
    out = faults.nan_ranges(rec, 10.5, 10.5, SEED, 1.0, "nan", targets=("scan",))
    lines = planner_stream.stream_lines(out, 10.0, 12.0, tracks_mode="none", preroll_s=0.0)
    scans = [ln.split() for ln in lines if ln.startswith("S ")]
    nan_scan = next(s for s in scans if abs(float(s[1]) - 10.5) < 1e-6)
    assert set(nan_scan[8:]) == {"nan"} and int(nan_scan[7]) == 12
    ok_scan = next(s for s in scans if abs(float(s[1]) - 10.6) < 1e-6)
    assert "nan" not in ok_scan[8:]
    fr = faults.frozen(rec, 10.5, 10.8, SEED, targets=("scan",))
    stamps = [float(ln.split()[2]) for ln in planner_stream.stream_lines(
        fr, 10.0, 12.0, tracks_mode="none", preroll_s=0.0) if ln.startswith("S ")]  # fmt: skip
    assert stamps[5:9] == [pytest.approx(10.3)] * 4  # four scans with the frozen stamp


# --- part 2: the rules, on hand-made data ----------------------------------------------------


LIMITS = fault_checks.PlannerLimits(
    v_max_mps=1.0,
    reverse_limit_mps=0.3,
    floor_mps=0.25,
    steering_min_rad=-0.449,
    steering_max_rad=0.449,
    max_speed_up_step_mps=0.0857,
    tick_s=0.02,
)


def make_rows(n: int = 300, speed: float = 0.9) -> dict[str, np.ndarray]:
    now = 100.0 + 0.02 * np.arange(n)
    return {
        "now_s": now,
        "state": np.full(n, 1),
        "state_name": np.full(n, "DRIVING"),
        "cmd_speed_mps": np.full(n, speed),
        "cmd_steering_rad": np.zeros(n),
        "v_cap_mps": np.full(n, 1.0),
        "scan_age_s": np.full(n, 0.15),
        "track_confidence": np.ones(n),
    }


def set_state(rows: dict[str, np.ndarray], sel: np.ndarray, state: int, speed: float) -> None:
    names = {1: "DRIVING", 3: "NO_SCAN", 4: "NO_ODOM", 5: "BLOCKED", 2: "NO_TRACK"}
    rows["state"][sel] = state
    rows["state_name"][sel] = names[state]
    rows["cmd_speed_mps"][sel] = speed


def test_always_rules_pass_a_clean_run_and_name_each_violation() -> None:
    assert fault_checks.always_violations(make_rows(), LIMITS) == []
    cases = {
        "non finite": lambda r: r["cmd_speed_mps"].__setitem__(10, math.nan),
        "below": lambda r: r["cmd_speed_mps"].__setitem__(10, -0.31),
        "above v_max": lambda r: r["cmd_speed_mps"].__setitem__(10, 1.01),
        "above the cap": lambda r: r["v_cap_mps"].__setitem__(slice(0, 20), 0.5),
        "steering": lambda r: r["cmd_steering_rad"].__setitem__(10, 0.5),
        "dead zone": lambda r: r["cmd_speed_mps"].__setitem__(slice(5, 9), 0.1),
    }
    for needle, mutate in cases.items():
        rows = make_rows()
        mutate(rows)
        found = fault_checks.always_violations(rows, LIMITS)
        assert found and needle.split()[0] in found[0], (needle, found)


def test_a_start_ramp_through_the_dead_zone_is_allowed_and_a_crawl_is_not() -> None:
    rows = make_rows(20, speed=0.0)
    rows["cmd_speed_mps"][2:] = 0.9
    rows["cmd_speed_mps"][2], rows["cmd_speed_mps"][3] = 0.077, 0.154  # two ticks, then the floor
    rows["cmd_speed_mps"][4] = 0.25
    assert fault_checks.always_violations(rows, LIMITS) == []
    rows["cmd_speed_mps"][4:12] = 0.15  # crawling under the floor
    assert fault_checks.always_violations(rows, LIMITS)
    cold = make_rows(20)
    cold["cmd_speed_mps"][5] = 0.1  # into the dead zone from 0.9, not from rest
    assert "not from rest" in fault_checks.always_violations(cold, LIMITS)[0]


def fault_of(name: str, target: str, t0: float, t1: float, **kw: Any) -> fault_checks.Fault:
    # a good message every 0.1 s (scan) or 0.02 s (odometry); the one before t0 and the one after t1
    period = 0.02 if target == "odom" else 0.1
    return fault_checks.Fault(name, target, t0, t1, t0 - period, t1 + period, t0, **kw)


def test_a_long_scan_silence_must_stop_the_car_with_no_scan_by_0_6_s_plus_two_ticks() -> None:
    rows = make_rows()
    f = fault_of("drop", "scan", 101.0, 102.0)  # last good 100.9, back at 102.1
    assert f.gap_s > 0.6
    hold = (rows["now_s"] >= 101.25) & (rows["now_s"] < 102.1)
    set_state(rows, hold, 3, 0.0)
    assert fault_checks.planner_violations(rows, f, LIMITS) == []  # stops at once: fine
    # still driving at 0.9 until 0.2 s before the bound is also fine: the bound is 0.6 s
    late = make_rows()
    set_state(late, (late["now_s"] >= 101.5 + 0.02 * 2) & (late["now_s"] < 102.1), 3, 0.0)
    assert fault_checks.planner_violations(late, f, LIMITS) == []
    never = make_rows()  # drives through the whole silence
    found = fault_checks.planner_violations(never, f, LIMITS)
    assert any("hard bound 0.6 s" in m for m in found)
    wrong = make_rows()
    set_state(wrong, hold, 5, 0.0)  # speed 0 but BLOCKED, not NO_SCAN
    assert any("expected NO_SCAN" in m for m in fault_checks.planner_violations(wrong, f, LIMITS))


def test_the_state_must_come_back_to_driving_within_a_second_when_the_unfaulted_run_drives() -> (
    None
):
    f = fault_of("frozen", "scan", 101.0, 102.0)
    rows = make_rows()
    stuck = (rows["now_s"] >= 101.3) & (rows["now_s"] < 104.0)
    set_state(rows, stuck, 3, 0.0)
    found = fault_checks.planner_violations(rows, f, LIMITS, baseline=make_rows())
    assert any("not DRIVING within 1.0 s" in m for m in found)
    # the rule is the planner's, not the fault's: where the unfaulted run was not driving, silent
    idle = make_rows()
    set_state(idle, idle["now_s"] >= 100.5, 5, 0.0)
    assert not any(
        "not DRIVING" in m for m in fault_checks.planner_violations(rows, f, LIMITS, baseline=idle)
    )


def test_odometry_silence_has_the_tighter_bound_and_no_odom() -> None:
    f = fault_of("drop", "odom", 101.0, 102.0)
    rows = make_rows()
    set_state(rows, (rows["now_s"] >= 101.24) & (rows["now_s"] < 102.02), 4, 0.0)
    assert fault_checks.planner_violations(rows, f, LIMITS) == []
    slow = make_rows()
    found = fault_checks.planner_violations(slow, f, LIMITS)
    assert any("hard bound 0.2 s" in m for m in found)
    nan_odom = fault_of("nan_ranges", "odom", 101.0, 102.0)  # NaN never refreshes the clock
    assert fault_checks.planner_violations(slow, nan_odom, LIMITS)


def test_what_the_planner_cannot_see_gets_the_always_rules_and_nothing_more() -> None:
    rows = make_rows()  # drives straight through, which is all it can do on these faults
    for name, target in (
        ("frozen_fresh_stamp", "scan"),
        ("frozen", "odom"),
        ("frozen_fresh_stamp", "odom"),
        ("stamp_jump_forward", "odom"),
        ("stamp_jump_backward", "odom"),
        ("out_of_order", "odom"),
    ):
        f = fault_of(name, target, 101.0, 102.0)
        assert not f.detectable_silence, (name, target)
        assert fault_checks.planner_violations(rows, f, LIMITS) == []
    for name in ("drop", "frozen", "stamp_jump_forward", "stamp_jump_backward"):
        assert fault_of(name, "scan", 101.0, 102.0).detectable_silence
    assert fault_of("drop", "odom", 101.0, 102.0).detectable_silence
    assert not fault_of("out_of_order", "scan", 101.0, 102.0).detectable_silence


def test_a_short_gap_must_not_raise_the_speed_nor_use_other_states() -> None:
    f = fault_of("drop", "scan", 101.0, 101.05)  # one scan
    assert f.gap_s < 0.6
    rows = make_rows(speed=0.8)
    assert fault_checks.planner_violations(rows, f, LIMITS) == []
    surge = make_rows(speed=0.8)
    surge["cmd_speed_mps"][(surge["now_s"] > 101.1) & (surge["now_s"] < 101.2)] = 0.95
    assert any("exceeds" in m for m in fault_checks.planner_violations(surge, f, LIMITS))
    blocked = make_rows(speed=0.8)
    set_state(blocked, (blocked["now_s"] > 101.1) & (blocked["now_s"] < 101.14), 5, 0.0)
    assert any("short fault" in m for m in fault_checks.planner_violations(blocked, f, LIMITS))
    # with the baseline slack an acceleration the unfaulted run also made is not the fault's
    base = make_rows(speed=0.8)
    base["cmd_speed_mps"][(base["now_s"] > 101.0)] = 0.95
    ok = fault_checks.planner_violations(surge, f, LIMITS, baseline=base, baseline_speed_slack=True)
    assert ok == []


def test_a_fully_unusable_scan_must_read_as_no_scan_or_blocked_with_speed_0() -> None:
    f = fault_of("nan_ranges", "scan", 101.0, 102.0, fraction=1.0)
    driving = make_rows()
    found = fault_checks.planner_violations(driving, f, LIMITS)
    assert any("no usable beam but DRIVING" in m for m in found)
    stopped = make_rows()
    set_state(stopped, (stopped["now_s"] >= 101.04) & (stopped["now_s"] < 102.1), 5, 0.0)
    assert fault_checks.planner_violations(stopped, f, LIMITS) == []
    half = fault_of("nan_ranges", "scan", 101.0, 102.0, fraction=0.5)
    assert fault_checks.planner_violations(driving, half, LIMITS) == []  # always rules only


def test_garbage_track_rules() -> None:
    f = fault_of("track_garbage", "track", 101.0, 102.0)
    rows = make_rows()
    rows["track_confidence"][(rows["now_s"] >= 101.02) & (rows["now_s"] < 102.2)] = 0.0
    assert fault_checks.planner_violations(rows, f, LIMITS, baseline=make_rows()) == []
    high = make_rows()
    assert any("expected 0" in m for m in fault_checks.planner_violations(high, f, LIMITS))
    no_track = make_rows()
    no_track["state"][50] = 2
    assert any("NO_TRACK" in m for m in fault_checks.planner_violations(no_track, f, LIMITS))
    lost = make_rows()
    lost["track_confidence"][:] = 0.0
    set_state(lost, (lost["now_s"] >= 101.2) & (lost["now_s"] < 101.4), 5, 0.0)
    found = fault_checks.planner_violations(lost, f, LIMITS, baseline=make_rows())
    assert any("left DRIVING" in m for m in found)


def test_out_of_order_scan_ages_must_not_be_negative() -> None:
    f = fault_of("out_of_order", "scan", 101.0, 102.0)
    rows = make_rows()
    assert fault_checks.planner_violations(rows, f, LIMITS) == []
    rows["scan_age_s"][100] = -0.04
    assert any("negative scan age" in m for m in fault_checks.planner_violations(rows, f, LIMITS))
    rows["scan_age_s"][100] = -1.0  # the CSV's "no scan yet"
    assert fault_checks.planner_violations(rows, f, LIMITS) == []


def test_fault_for_finds_the_good_messages_around_the_window() -> None:
    rec = make_recording()
    f = fault_checks.fault_for(
        rec["scan_t"], rec["odom_t"], rec["trk_t"], "drop", "scan", 10.5, 11.2
    )
    assert f.last_good_t == pytest.approx(10.4) and f.resume_t == pytest.approx(11.3)
    assert f.first_t == pytest.approx(10.5) and f.gap_s == pytest.approx(0.9)
    o = fault_checks.fault_for(
        rec["scan_t"], rec["odom_t"], rec["trk_t"], "drop", "odom", 10.5, 10.55
    )
    assert o.gap_s == pytest.approx(0.08)


def make_track(n: int = 16, **over: Any) -> Any:
    from types import SimpleNamespace

    base = {
        "x_m": np.zeros(n), "y_m": np.zeros(n), "half_width_left_m": np.full(n, 0.5),
        "half_width_right_m": np.full(n, 0.5), "sigma_m": np.full(n, 0.02), "valid_count": n,
        "boundary_residual_m": 0.05, "intrusion_fraction": 0.0,
    }  # fmt: skip
    return SimpleNamespace(**{**base, **over})


def test_perception_rules_pass_a_good_track_and_name_each_violation() -> None:
    assert fault_checks.perception_violations([make_track(), make_track(valid_count=0)]) == []
    bad_cases: dict[str, Any] = {
        "not 16": make_track(n=15, valid_count=0),
        "valid_count 17": make_track(valid_count=17),
        "not finite": make_track(x_m=np.r_[np.full(15, 0.0), np.nan], valid_count=16),
        "infinite": make_track(boundary_residual_m=math.inf),
        "NaN on a track": make_track(intrusion_fraction=math.nan),
    }
    for needle, track in bad_cases.items():
        found = fault_checks.perception_violations([track])
        assert found and needle in " ".join(found), (needle, found)
    # NaN beyond valid_count is allowed, a track of fewer than two stations may carry NaN checks
    assert (
        fault_checks.perception_violations(
            [make_track(x_m=np.r_[np.zeros(8), np.full(8, np.nan)], valid_count=8)]
        )
        == []
    )
    assert (
        fault_checks.perception_violations(
            [make_track(valid_count=1, boundary_residual_m=math.nan, intrusion_fraction=math.nan)]
        )
        == []
    )
    assert fault_checks.perception_violations([make_track()], [True])  # valid_count must be 0
    assert fault_checks.perception_violations([make_track(valid_count=0)], [True]) == []


# --- part 3: every fault against both consumers, on the fixture ------------------------------


def test_fixture_is_small_and_has_the_three_cuts() -> None:
    assert FIXTURE.stat().st_size < 500_000
    for name, (a, b) in cut_fixture.FAULT_CUTS.items():
        rec, wa, wb = cut_fixture.load_cut(FIXTURE, name)
        assert (wa, wb) == (a, b)
        assert len(rec["scan_t"]) >= (b - a) * 9 and len(rec["odom_t"]) >= (b - a) * 40
        assert len(rec["trk_t"]) >= (b - a) * 8 and len(rec["st_t"]) > 0  # R2's first track: +1.3 s
        assert rec["scan_t"].min() >= a - planner_stream.PREROLL_S - 1e-9
    r2, a, _ = cut_fixture.load_cut(FIXTURE, "R2")
    assert cut_fixture.FAULT_CUTS["R2"] == (625.5, 633.5)  # the first 8 s of run R2
    v = r2["odom_v"][r2["odom_t"] >= a]
    assert v[:10].max() < 0.2 and v[-10:].min() > 0.7  # a standing start to about 1.0 m/s
    r7, a7, b7 = cut_fixture.load_cut(FIXTURE, "R7")
    assert cut_fixture.FAULT_CUTS["R7"] == (2975.0, 2981.0)
    assert r7["odom_v"][(r7["odom_t"] >= a7 + 2) & (r7["odom_t"] <= b7)].min() > 0.7


def test_the_case_table_covers_every_fault_for_the_scan_and_the_planner() -> None:
    by_fault: dict[str, set[str]] = {}
    for c in fault_run.CASES:
        by_fault.setdefault(c.fault, set()).add(c.target)
    for fault in faults.SCAN_FAULTS:
        assert by_fault[fault] >= {"scan", "odom"}, fault
    assert by_fault["track_garbage"] == {"track"}
    assert len({c.id for c in fault_run.CASES}) == len(fault_run.CASES)
    nan = {c.params["fraction"] for c in fault_run.CASES if c.fault == "nan_ranges"
           and c.target == "scan"}  # fmt: skip
    assert nan == set(faults.NAN_FRACTIONS)
    assert {c.params["fill"] for c in fault_run.CASES if c.fault == "nan_ranges"
            and c.target == "scan"} == set(faults.NAN_FILLS)  # fmt: skip


@pytest.fixture(scope="module")
def cuts() -> dict[str, tuple[bag.Recording, float, float]]:
    return {name: cut_fixture.load_cut(FIXTURE, name) for name in cut_fixture.FAULT_CUTS}


@pytest.fixture(scope="module")
def perceiver() -> perception.Perceiver:
    return perception.build_perceiver("geometric")


def case_window(
    cuts: dict[str, tuple[bag.Recording, float, float]], case: fault_run.Case
) -> tuple[bag.Recording, float, float, float, float]:
    rec, a, b = cuts[case.cut]
    t0, t1 = case.window(rec, a + case.start_s)
    return rec, a, b, t0, t1


@pytest.mark.parametrize("case", fault_run.SCAN_CASES, ids=lambda c: c.id)
def test_perception(
    case: fault_run.Case,
    cuts: dict[str, tuple[bag.Recording, float, float]],
    perceiver: perception.Perceiver,
) -> None:
    """TrackPerceiver.process never raises on the mutated scans, returns arrays of 16 with a
    valid_count whose leading entries are finite and self-check numbers that are never infinite
    (finite on a track of two stations or more), and a scan without a usable beam gives
    valid_count 0."""
    rec, a, b, t0, t1 = case_window(cuts, case)
    mutated = fault_run.inject(rec, case, t0, t1, SEED)
    tracks, empty, _swallowed = fault_run.perceive(perceiver, mutated, a - 0.5, b)
    assert len(tracks) == len(planner_stream.scan_indices(mutated, a - 0.5, b))
    assert fault_checks.perception_violations(tracks, empty) == []
    if case.fault == "nan_ranges" and case.params["fraction"] == 1.0:
        assert sum(empty) >= 9  # a second of scans, every beam unusable
        assert all(t.valid_count == 0 for t, e in zip(tracks, empty, strict=True) if e)
    # the fault does not leak: a scan well before the window gives the unmutated track
    clean = fault_run.perceive(perceiver, rec, a - 0.5, b)[0]
    early = [i for i, k in enumerate(planner_stream.scan_indices(mutated, a - 0.5, b))
             if mutated["scan_t"][k] < t0 - 0.2]  # fmt: skip
    for i in early[:5]:
        assert tracks[i].valid_count == clean[i].valid_count
        assert np.array_equal(tracks[i].x_m, clean[i].x_m)


def _cli_or_skip() -> planner_stream.Cli:
    try:
        return planner_stream.find_cli()
    except planner_stream.ReplayError as error:
        if os.environ.get("PNP_REPLAY_REQUIRE_CLI"):
            pytest.fail(f"planner_replay_cli is required here but not built: {error}")
        pytest.skip(f"planner_replay_cli not built ({error})")


@pytest.fixture(scope="module")
def setup() -> fault_run.PlannerSetup:
    return fault_run.planner_setup(_cli_or_skip(), CAR_BUNDLE, 1.0)


@pytest.fixture(scope="module")
def baselines(
    setup: fault_run.PlannerSetup,
    cuts: dict[str, tuple[bag.Recording, float, float]],
    tmp_path_factory: pytest.TempPathFactory,
) -> dict[str, dict[str, Any]]:
    """The replay of each cut without a fault. The always rules hold on it, or no fault test
    means anything."""
    work = tmp_path_factory.mktemp("faults_base")
    out = {}
    for name, (rec, a, b) in cuts.items():
        out[name] = fault_run.run_planner(setup, rec, a, b, work, f"base_{name}")
        assert fault_checks.always_violations(out[name], setup.limits) == [], name
        states = out[name]["state"].tolist()
        lead = next((i for i, x in enumerate(states) if x != 0), len(states))
        # the planner v0.2 warm start (STARTING) leads the cut; after it the cut is all DRIVING
        assert set(states[lead:]) == {1}, f"{name}: the cut is all DRIVING after the warm start"
    return out


# F1 (a scan with no usable beam) and F2 (a negative scan age from swapped scans) were findings
# against the planner of main; planner v0.2 fixes both (min_scan_points, age clamped to 0).
PLANNER_XFAIL: dict[str, str] = {}
# What the planner cannot see (module docstring); the test still runs the always rules.
BLIND = (
    "scan-frozen_fresh_stamp-1.0s",
    "odom-stamp_jump_forward-1.0s",
    "odom-stamp_jump_backward-1.0s",
    "odom-out_of_order-1.0s",
    "odom-frozen-1.0s",
    "odom-frozen_fresh_stamp-1.0s",
)


def _planner_param(case: fault_run.Case) -> Any:
    marks = []
    if case.id in PLANNER_XFAIL:
        marks.append(pytest.mark.xfail(strict=True, reason=PLANNER_XFAIL[case.id]))
    return pytest.param(case, id=case.id, marks=marks)


@pytest.mark.parametrize("case", [_planner_param(c) for c in fault_run.CASES])
def test_planner(
    case: fault_run.Case,
    cuts: dict[str, tuple[bag.Recording, float, float]],
    setup: fault_run.PlannerSetup,
    baselines: dict[str, dict[str, Any]],
    tmp_path: Path,
) -> None:
    """The real planner on the mutated stream (see fault_checks for the rules). The CLI exits 0
    (run_cli raises otherwise), every always rule holds, and the rules of this fault hold.
    The BLIND cases are faults the planner cannot detect: only the always rules apply and the
    outcome is recorded in the test's output, not asserted as a detection."""
    rec, a, b, t0, t1 = case_window(cuts, case)
    mutated = fault_run.inject(rec, case, t0, t1, SEED)
    record = fault_run.fault_record(rec, case, t0, t1)
    rows = fault_run.run_planner(setup, mutated, a, b, tmp_path, case.id)
    assert len(rows["now_s"]) == pytest.approx((b - a) / 0.02, abs=2)  # a row per tick, none lost
    violations = fault_checks.planner_violations(rows, record, setup.limits, baselines[case.cut])
    assert violations == [], violations
    states = {str(s): int(n) for s, n in zip(*np.unique(rows["state_name"], return_counts=True),
                                              strict=True)}  # fmt: skip
    base = baselines[case.cut]
    inside = (rows["now_s"] >= t0) & (rows["now_s"] <= t1 + 1.0)
    print(
        f"{case.id}: gap {record.gap_s:.2f} s, states {states}, speed in the fault "
        f"{rows['cmd_speed_mps'][inside].min():.2f} to {rows['cmd_speed_mps'][inside].max():.2f} "
        f"m/s (unfaulted {base['cmd_speed_mps'][inside].min():.2f} to "
        f"{base['cmd_speed_mps'][inside].max():.2f})"
        + ("; BLIND: the planner cannot detect this fault" if case.id in BLIND else "")
    )
    if case.id in BLIND:
        assert not record.detectable_silence


def test_the_planner_cannot_see_a_frozen_scan_with_fresh_stamps(
    cuts: dict[str, tuple[bag.Recording, float, float]],
    setup: fault_run.PlannerSetup,
    baselines: dict[str, dict[str, Any]],
    tmp_path: Path,
) -> None:
    """Stated plainly: a stuck LiDAR that still talks (the same ranges, a fresh stamp each time)
    is NOT detected by the planner. It keeps driving on the frozen scan and the state stays
    DRIVING for the whole second. It is caught, if at all, by layer 3 and layer 1 (safety node,
    hardware mux). This test records what the planner does so a change to that is noticed; it
    does not assert a detection, because there is none."""
    case = fault_run.CASES_BY_ID["scan-frozen_fresh_stamp-1.0s"]
    rec, a, b, t0, t1 = case_window(cuts, case)
    rows = fault_run.run_planner(setup, fault_run.inject(rec, case, t0, t1, SEED), a, b,
                                 tmp_path, "ffs")  # fmt: skip
    inside = (rows["now_s"] >= t0 + 0.2) & (rows["now_s"] <= t1)
    assert (rows["state"][inside] == fault_checks.STATE_DRIVING).all()
    assert (rows["cmd_speed_mps"][inside] > 0.5).all()


def test_wheel_odometry_stamp_faults_change_nothing_the_planner_can_see(
    cuts: dict[str, tuple[bag.Recording, float, float]],
    setup: fault_run.PlannerSetup,
    baselines: dict[str, dict[str, Any]],
    tmp_path: Path,
) -> None:
    """The node stamps odometry with its arrival time, so a stamp jump of the wheel odometry
    stream writes the same planner stream as no fault: the CSV is identical to the unfaulted one."""
    base = baselines["R7"]
    for name in ("stamp_jump_forward", "stamp_jump_backward"):
        case = fault_run.CASES_BY_ID[f"odom-{name}-1.0s"]
        rec, a, b, t0, t1 = case_window(cuts, case)
        rows = fault_run.run_planner(setup, fault_run.inject(rec, case, t0, t1, SEED), a, b,
                                     tmp_path, name)  # fmt: skip
        for key in ("state", "cmd_speed_mps", "cmd_steering_rad"):
            assert np.array_equal(rows[key], base[key]), (name, key)


def test_the_planner_run_is_repeatable_under_a_fault(
    cuts: dict[str, tuple[bag.Recording, float, float]],
    setup: fault_run.PlannerSetup,
    tmp_path: Path,
) -> None:
    case = fault_run.CASES_BY_ID["scan-nan_ranges-50-nan"]
    rec, a, b, t0, t1 = case_window(cuts, case)
    x = fault_run.run_planner(setup, fault_run.inject(rec, case, t0, t1, SEED), a, b, tmp_path, "x")
    y = fault_run.run_planner(setup, fault_run.inject(rec, case, t0, t1, SEED), a, b, tmp_path, "y")
    assert x["cmd_speed_mps"].tolist() == y["cmd_speed_mps"].tolist()
    assert x["cmd_steering_rad"].tolist() == y["cmd_steering_rad"].tolist()


def test_the_setup_reads_its_limits_from_the_bindings_the_cli_and_the_bundle(
    setup: fault_run.PlannerSetup,
) -> None:
    v = vehicle.load_vehicle_limits()
    assert v.schema_version == "0.17.0"
    lim = setup.limits
    assert lim.steering_max_rad == v.steering_max_rad and lim.steering_min_rad == v.steering_min_rad
    assert lim.max_speed_up_step_mps == pytest.approx(0.45 * v.max_acceleration_mps2 * 0.02)
    assert lim.floor_mps == pytest.approx(0.22 + 0.03)  # the car bundle's dead band plus the margin
    assert lim.reverse_limit_mps >= 0.3 and lim.v_max_mps == 1.0


def test_random_starts_leave_room_before_and_after_the_window() -> None:
    rng = np.random.default_rng(3)
    case = fault_run.CASES_BY_ID["scan-drop-1.0s"]
    starts = fault_run.random_starts(rng, 100.0, 120.0, case, 5)
    assert len(starts) == 5 and starts == sorted(starts)
    assert min(starts) >= 101.0 and max(starts) + 1.0 <= 120.0 - 1.5
    assert fault_run.random_starts(rng, 100.0, 103.0, case, 5) == []  # a run too short
    again = fault_run.random_starts(np.random.default_rng(3), 100.0, 120.0, case, 5)
    assert again == starts
