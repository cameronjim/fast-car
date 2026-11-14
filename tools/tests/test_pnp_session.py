"""L1, L2 and L4 tests of tools/pnp_session (brief 34): gap statistics, detail parsing, the counting
window, D3-C0, host facts, the CLI, the limiter plumbing, and the committed R2 fixture with its
golden report. Needs the `session` extra (racer_eval)."""

from __future__ import annotations

import json
import os
import stat
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("racer_eval")
pytest.importorskip("mcap")

from hypothesis import given
from hypothesis import strategies as st
from pnp_session import (
    cli,
    cut_fixture,
    detail,
    gaps,
    hostfacts,
    laps,
    report,
    timing,
    whatif,
)
from pnp_session.bagread import SessionBag, read_session_bag

FIX = Path(__file__).resolve().parent / "fixtures"
CUT = FIX / "pnp_session_r2_cut.mcap"
GOLDEN = FIX / "pnp_session_r2_golden.json"


# ------------------------------------------------------------------------------------ gaps


def test_gap_counts_exactly_at_a_limit_and_epsilon_over():
    t = np.array([0.0, 0.060, 0.120 + 1e-6, 0.120 + 1e-6 + 0.080])
    s = gaps.timeline_stats(t)
    assert s.n_messages == 4 and s.n_gaps == 3
    assert s.above["60"] == 2  # 0.060 is not above 60 ms; 0.060001 and 0.080 are
    assert s.above["80"] == 0
    assert s.above["40"] == 3


def test_gap_stats_empty_and_single_message():
    for t in (np.zeros(0), np.array([1.0])):
        s = gaps.timeline_stats(t)
        assert s.n_gaps == 0 and s.p50_s is None and s.max_s is None
        assert set(s.above.values()) == {0}
    assert gaps.timeline_stats(np.array([1.0])).n_messages == 1


def test_percentiles_and_pooling_do_not_span_runs():
    a = gaps.gaps_of([0.0, 0.02, 0.04])
    b = gaps.gaps_of([100.0, 100.2])
    pooled = gaps.pooled_stats([a, b], 5)
    assert pooled.n_gaps == 3 and pooled.max_s == pytest.approx(0.2)
    assert pooled.p50_s == pytest.approx(0.02)
    assert pooled.above["150"] == 1
    assert gaps.pooled_stats([], 0).n_gaps == 0
    assert gaps.count_below([0.005, 0.01, 0.02], 0.01) == 1
    assert gaps.limit_key(0.0625) == "62.5"


@given(
    st.lists(st.floats(0.0, 1000.0), min_size=0, max_size=40),
    st.floats(-500.0, 500.0),
)
def test_shifting_a_timeline_changes_no_statistic(times, shift):
    t = np.sort(np.array(times))
    a = gaps.timeline_stats(t)
    b = gaps.timeline_stats(t + shift)
    assert a.n_gaps == b.n_gaps
    assert a.above.keys() == b.above.keys()
    for k in a.above:
        # a shift can move a gap across a limit only by float rounding of the stamps themselves
        assert abs(a.above[k] - b.above[k]) <= max(1, a.n_gaps // 4)


@given(st.lists(st.floats(0.0, 2.0), min_size=0, max_size=60))
def test_threshold_counts_are_non_increasing_in_the_limit(g):
    counts = [gaps.count_above(g, lim) for lim in (0.04, 0.06, 0.08, 0.10, 0.15)]
    assert counts == sorted(counts, reverse=True)


# ---------------------------------------------------------------------------------- detail


def test_detail_parsing_both_sources():
    assert detail.parse_rate_limit_dt(
        "command rate-limited relative to previous output (dt=0.006800s)"
    ) == pytest.approx(0.0068)
    wd = detail.parse_watchdog(
        "/drive_raw stale or age invalid (timeout=0.060000s, age=0.349454s); braking"
    )
    assert wd is not None and wd.timeout_s == 0.06 and wd.age_s == pytest.approx(0.349454)


@pytest.mark.parametrize(
    "text",
    [
        "gate released after 19.378680s engaged",
        "command rate-limited relative to previous output (dt=inf s)",
        "command rate-limited relative to previous output (dt=nans)",
        "/drive_raw stale or age invalid (timeout=0.060000s, age=infs); braking",
        "",
    ],
)
def test_detail_that_does_not_parse_is_none(text):
    assert detail.parse_rate_limit_dt(text) is None
    assert detail.parse_watchdog(text) is None


# ------------------------------------------------------------------------------- timelines


def make_bag(
    *,
    status_end: float = 10.0,
    first_cmd: float = 1.0,
    events: list[tuple[float, str, str]] | None = None,
    draw_t: np.ndarray | None = None,
) -> SessionBag:
    """A PnP run: status at 50 Hz from 0 to status_end, /drive_raw non-zero from first_cmd."""
    b = SessionBag(name="synthetic", t0_log_s=1000.0)
    b.st_t = np.arange(0.0, status_end + 1e-9, 0.02)
    b.st_stamp = b.st_t.copy()
    b.st_state = np.where(b.st_t < first_cmd, 0, 1).astype(np.int64)
    b.st_cycle = np.arange(len(b.st_t), dtype=np.int64)
    b.st_solve_s = np.full(len(b.st_t), 0.003)
    b.draw_t = np.arange(0.0, status_end + 1e-9, 0.02) if draw_t is None else draw_t
    b.draw_stamp = b.draw_t.copy()
    b.draw_speed = np.where(b.draw_t >= first_cmd, 0.5, 0.0)
    b.draw_steer = np.zeros(len(b.draw_t))
    ev = events or []
    b.ev_t = np.array([e[0] for e in ev], dtype=np.float64)
    b.ev_stamp = b.ev_t.copy()
    b.ev_phase = np.zeros(len(ev), dtype=np.int64)
    b.ev_severity = np.ones(len(ev), dtype=np.int64)
    b.ev_source = np.array([e[1] for e in ev], dtype=str)
    b.ev_detail = np.array([e[2] for e in ev], dtype=str)
    # one /drive cycle per 20 ms
    b.drv_t = b.draw_t.copy()
    b.drv_stamp = b.draw_t.copy()
    b.drv_speed = b.draw_speed.copy()
    b.drv_steer = np.zeros(len(b.draw_t))
    return b


RL = "command rate-limited relative to previous output (dt=0.006800s)"
WD = "/drive_raw stale or age invalid (timeout=0.060000s, age=0.349454s); braking"


def test_counting_window_edges_and_per_start_window():
    # window is [1.0, 10.0]; 10.01 is after the last status (the stop mechanism), 0.5 is before
    # the first command; 2.99 and 3.01 are either side of the first 2.0 s of the start.
    bag = make_bag(
        events=[
            (0.5, "watchdog", WD),
            (1.5, "rate_limit", RL),
            (2.99, "rate_limit", RL),
            (3.01, "rate_limit", RL),
            (10.0, "watchdog", WD),
            (10.01, "watchdog", WD),
        ]
    )
    a = timing.analyse_timing(bag)
    run = a.report["runs"][0]
    counts = run["safety"]["engage_counts"]
    assert counts["rate_limit"] == 3 and counts["watchdog"] == 1
    assert run["safety"]["first_2s_counts"]["rate_limit"] == 2  # 1.5 and 2.99, not 3.01
    assert run["safety"]["start_event"] is True
    d = run["safety"]["details"]
    assert d["rate_limit"]["dt_s"]["n"] == 3
    assert d["watchdog"]["age_s"]["max"] == pytest.approx(0.349454)
    assert d["watchdog"]["timeouts_s"] == [0.06]


def test_unparsed_details_are_counted_and_shown():
    bag = make_bag(events=[(2.0, "rate_limit", "something else"), (3.0, "watchdog", "odd")])
    d = timing.analyse_timing(bag).report["runs"][0]["safety"]["details"]
    assert d["rate_limit"]["n_unparsed"] == 1
    assert d["rate_limit"]["unparsed_shown"] == ["something else"]
    assert d["watchdog"]["unparsed_shown"] == ["odd"]


def test_gaps_inside_the_window_only_and_starting_block():
    t = np.concatenate([np.arange(0.0, 5.0, 0.02), np.arange(5.5, 10.0 + 1e-9, 0.02)])
    bag = make_bag(draw_t=t)
    a = timing.analyse_timing(bag)
    g = a.report["runs"][0]["drive_raw"]["recv"]
    assert g["above"]["150"] == 1 and g["max_s"] == pytest.approx(0.52)
    s = a.report["runs"][0]["starting"]
    assert s["ticks"] == 50 and s["duration_s"] == pytest.approx(1.0)
    pooled = a.report["pooled"]
    assert pooled["planner"]["fault_ticks"] == 0
    assert a.report["watchdog_whatif"]["gaps_above_limit"]["60"] == 1
    assert a.report["watchdog_whatif"]["recorded_without_gap"] == 0


def test_run_without_a_command_is_skipped_and_no_status_means_no_runs():
    bag = make_bag()
    bag.draw_speed[:] = 0.0
    a = timing.analyse_timing(bag)
    assert a.skipped_runs == ["R0"] and not a.pieces
    assert a.report["runs"] == [{"run_id": "R0", "commanded": False}]
    empty = SessionBag(name="e")
    assert timing.find_pnp_runs(empty) == []


def test_watchdog_event_matched_to_a_gap():
    t = np.concatenate([np.arange(0.0, 5.0, 0.02), np.arange(5.2, 10.0 + 1e-9, 0.02)])
    bag = make_bag(draw_t=t, events=[(5.1, "watchdog", WD)])
    w = timing.analyse_timing(bag).report["watchdog_whatif"]
    assert w["gaps_above_current_limit"] == 1 and w["matched"] == 1


def test_acceptance_computed_or_not():
    bag = make_bag()
    assert timing.analyse_timing(bag).report["acceptance_4_2"]["status"] == "not computed"
    bad = timing.analyse_timing(bag, [0.5, 1.0]).report["acceptance_4_2"]
    assert bad["status"] == "not computed" and "caps given" in bad["reason"]
    ok = timing.analyse_timing(bag, [0.5], wall_contacts=0).report["acceptance_4_2"]
    assert ok["status"] == "computed" and ok["items"][0]["item_id"] == "4.2-a"


def test_markdown_and_baseline_check_on_a_synthetic_bag():
    bag = make_bag(events=[(2.0, "rate_limit", RL)])
    a = timing.analyse_timing(bag, [0.5])
    doc = report.assemble(bag.name, a.report, whatif.not_run("no tool"), True)
    assert doc["baseline_check"]["matches"] is False
    md = report.markdown(doc)
    assert "NOT RUN: no tool" in md and "DIFFERS" in md and "4.2-a" in md
    assert doc["schema"] == "pnp-session/1"
    json.dumps(doc)


# -------------------------------------------------------------------------------------- laps


def runs_of(n_runs: int, n: int, shift=None, ref_n=None):
    ref_n = n if ref_n is None else ref_n
    out = []
    for r in range(n_runs):
        pnp = tuple(10.0 + 0.1 * i for i in range(n))
        ref = tuple(10.0 + 0.1 * i for i in range(ref_n))
        if shift and r == 0:
            ref = tuple(x + (shift if i == 3 else 0.0) for i, x in enumerate(ref))
        out.append(laps.RunLapInput(f"R{r}", pnp, ref))
    return out


def test_d3c0_pass_when_counts_equal_and_all_within():
    assert laps.d3c0(runs_of(2, 10))["verdict"] == "PASS"


def test_d3c0_one_lap_031_off_is_listed_but_passes_at_19_of_20():
    r = laps.d3c0(runs_of(2, 10, shift=0.31))
    assert r["verdict"] == "PASS" and r["laps_within"] == 19
    assert r["laps_off_by_more_than_tolerance"][0]["lap"] == 4


def test_d3c0_exactly_03_is_within():
    assert laps.d3c0(runs_of(2, 10, shift=0.3))["laps_within"] == 20


def test_d3c0_fails_at_17_of_20():
    runs = runs_of(2, 10)
    bad = []
    for r in runs:
        ref = tuple(x + (0.5 if i < 2 else 0.0) for i, x in enumerate(r.reference_times))
        bad.append(laps.RunLapInput(r.run_id, r.pnp_times, ref))
    bad[1] = laps.RunLapInput(
        "R1",
        runs[1].pnp_times,
        tuple(x + (0.5 if i == 0 else 0.0) for i, x in enumerate(runs[1].reference_times)),
    )
    r = laps.d3c0(bad)
    assert r["laps_within"] == 17 and r["verdict"] == "FAIL"


def test_d3c0_count_mismatch_fails():
    assert laps.d3c0(runs_of(2, 10, ref_n=9))["verdict"] == "FAIL"


def test_d3c0_no_camera_events_is_no_reference_not_pass():
    r = laps.d3c0(runs_of(2, 10, ref_n=0))
    assert r["verdict"] == "NO REFERENCE" and r["laps_within_needed"] is None


def test_d3c0_insufficient_runs_or_laps():
    assert laps.d3c0(runs_of(1, 10))["verdict"] == "INSUFFICIENT"
    assert laps.d3c0(runs_of(2, 5))["verdict"] == "INSUFFICIENT"
    assert laps.d3c0([])["verdict"] == "NO REFERENCE"


def test_video_times_csv_and_inputs_from_bag(tmp_path):
    csv = tmp_path / "t.csv"
    csv.write_text("run,lap,lap_time_s\nR0,2,10.1\nR0,1,10.0\n")
    assert laps.read_video_times(csv) == {"R0": (10.0, 10.1)}
    bad = tmp_path / "b.csv"
    bad.write_text("a,b\n1,2\n")
    with pytest.raises(ValueError):
        laps.read_video_times(bad)
    bag = make_bag()
    bag.cam_t = np.array([3.0, 4.0])
    bag.cam_lap = np.array([1, 2])
    bag.cam_lap_time_s = np.array([10.0, 10.1])
    bag.cam_distance_m = np.array([7.4, 7.4])
    bag.pnp_t = np.array([3.1, 4.1])
    bag.pnp_lap_index = np.array([1, 2])
    bag.pnp_lap_time_s = np.array([10.05, 10.1])
    cam = laps.inputs_from_bag(bag)[0]
    assert cam.reference_times == (10.0, 10.1) and cam.pnp_times == (10.05, 10.1)
    vid = laps.inputs_from_bag(bag, {"R0": (9.0,)})[0]
    assert vid.reference_times == (9.0,)


# ------------------------------------------------------------------------------ host facts


class Done:
    def __init__(self, out="", code=0):
        self.stdout, self.stderr, self.returncode = out, "", code


def test_host_facts_run_only_read_only_commands():
    seen: list[list[str]] = []

    def runner(cmd):
        seen.append(list(cmd))
        return Done("1234\n" if cmd[0] == "pgrep" else "ok\n")

    facts = hostfacts.collect(runner, governor_files=["/sys/x/scaling_governor"])
    assert facts["planner_pids"] == [1234]
    heads = [c[0] for c in seen]
    assert heads == ["pgrep", "pgrep", "chrt", "cat", "uptime"]  # lookup, then the logged commands
    chrt = next(c for c in seen if c[0] == "chrt")
    assert chrt == ["chrt", "-p", "1234"]  # -p only reads; any other option would set a policy
    for c in seen:
        assert c[0] in {"pgrep", "chrt", "cat", "uptime"}
        assert not any(
            x.startswith(">") or x in {"sudo", "-f", "-r", "-o"} for x in c[1:] if c[0] != "pgrep"
        )
    assert "NOTE" not in hostfacts.format_text(facts)


def test_host_facts_without_a_planner_process_and_a_failing_command():
    def runner(cmd):
        if cmd[0] == "uptime":
            raise OSError("no uptime")
        return Done("")

    facts = hostfacts.collect(runner, governor_files=[])
    assert facts["planner_pids"] == [] and "NOTE" in hostfacts.format_text(facts)
    assert facts["entries"][-1]["returncode"] is None


# ------------------------------------------------------------------------------ the fixture


def test_fixture_is_small_and_reads():
    assert CUT.stat().st_size < 300_000
    bag = read_session_bag(CUT)
    assert len(bag.st_t) > 900 and len(bag.drv_t) > 900 and bag.t0_log_s > 1e9
    assert set(bag.ev_source) <= {"rate_limit", "watchdog"}


def _close(a, b, path=""):
    if isinstance(a, dict):
        assert a.keys() == b.keys(), path
        for k in a:
            _close(a[k], b[k], f"{path}/{k}")
    elif isinstance(a, list):
        assert len(a) == len(b), path
        for i, (x, y) in enumerate(zip(a, b, strict=True)):
            _close(x, y, f"{path}/{i}")
    elif isinstance(a, float) or isinstance(b, float):
        assert a == pytest.approx(b, rel=1e-9, abs=1e-12), path  # tolerance: 1e-9 relative
    else:
        assert a == b, path


def test_golden_report_of_the_r2_cut():
    bag = read_session_bag(CUT)
    a = timing.analyse_timing(bag, [1.0], 0)
    doc = report.assemble(bag.name, a.report, whatif.not_run("--no-limiter given"), False)
    _close(json.loads(json.dumps(doc, sort_keys=True)), json.loads(GOLDEN.read_text()))


def test_cut_fixture_roundtrip(tmp_path):
    out = tmp_path / "again.mcap"
    n, what = cut_fixture.cut(CUT, "R0", out)
    assert n > 3000 and "R0" in what
    again = read_session_bag(out)
    orig = read_session_bag(CUT)
    assert len(again.st_t) == len(orig.st_t)


# ------------------------------------------------------------------------------------ whatif


def test_cycle_table_command_in_force_and_age():
    bag = make_bag()
    bag.drv_stamp = np.array([0.5, 1.0, 1.01])
    bag.draw_t = bag.draw_stamp = np.array([0.7, 0.995])
    bag.draw_speed = np.array([0.1, 0.2])
    bag.draw_steer = np.array([0.0, 0.3])
    _stamp, dt, steer, speed, age = whatif.cycle_table(bag)
    assert np.isinf(age[0]) and speed[0] == 0.0  # nothing had arrived yet
    assert speed[1] == 0.2 and steer[1] == 0.3 and age[1] == pytest.approx(0.005)
    assert dt[1] == pytest.approx(0.5)
    r = whatif.cycle_table(bag, "recv")
    assert r[4][1] == pytest.approx(0.005)
    with pytest.raises(ValueError):
        whatif.cycle_table(bag, "nope")


def test_write_stream_segments_and_teacher(tmp_path):
    bag = make_bag()
    out = tmp_path / "s.txt"
    n = whatif.write_stream(bag, out, [(1.0, 1.1)], teacher=True)
    lines = [ln for ln in out.read_text().splitlines() if not ln.startswith("#")]
    assert lines[0].startswith("R ") and n == len(lines) - 1
    assert all(len(ln.split()) == 8 for ln in lines[1:])
    whole = tmp_path / "w.txt"
    assert whatif.write_stream(bag, whole) == len(bag.drv_stamp)
    assert whatif.write_stream(bag, tmp_path / "e.txt", [(50.0, 60.0)]) == 0


def test_compare_events_matched_missed_extra():
    rec = [(1.0, RL), (2.0, RL), (3.0, RL)]
    rep = [{"t": 1.0, "detail": RL}, {"t": 2.5, "detail": RL}]
    r = whatif.compare_events(rec, rep)
    assert (r["matched"], r["missed"], r["extra"]) == (1, 2, 1)
    assert r["dt_detail_max_abs_diff_s"] == 0.0 and r["dt_detail_compared"] == 1
    none = whatif.compare_events([], [])
    assert none["dt_detail_max_abs_diff_s"] is None


FAKE = """#!/usr/bin/env python3
import json, sys
mode = sys.argv[sys.argv.index("--dt-mode") + 1]
teacher = "--teacher" in sys.argv
print(json.dumps({"type": "clip", "t": 1.5, "dt_s": 0.01}))
print(json.dumps({"type": "event", "t": 1.5, "phase": "engage", "source": "rate_limit",
                  "severity": "warning", "duration_s": 0.0,
                  "detail": "command rate-limited relative to previous output (dt=0.010000s)"}))
print(json.dumps({"type": "summary", "dt_mode": mode, "teacher": teacher}))
"""


def fake_cli(tmp_path: Path, body: str = FAKE) -> Path:
    p = tmp_path / "limiter_whatif_cli"
    p.write_text(body)
    p.chmod(p.stat().st_mode | stat.S_IEXEC)
    return p


def test_whatif_report_with_a_fake_tool(tmp_path):
    bag = make_bag(events=[(1.5, "rate_limit", RL)])
    a = timing.analyse_timing(bag)
    w = whatif.whatif_report(bag, a, fake_cli(tmp_path))
    assert w["status"] == "run" and w["modes"]["recorded"]["pooled"]["clip_cycles"] == 1
    assert w["reproduction_recorded_dt"]["rate_limit"]["matched"] == 1
    doc = report.assemble(bag.name, a.report, w, False)
    md = report.markdown(doc)
    assert "Reproduction with the recorded dt" in md


def test_find_cli_and_failures(tmp_path, monkeypatch):
    cli_path = fake_cli(tmp_path)
    assert whatif.find_cli(str(cli_path)) == cli_path
    with pytest.raises(whatif.LimiterUnavailable):
        whatif.find_cli(str(tmp_path / "missing"))
    monkeypatch.setenv(whatif.CLI_ENV, str(cli_path))
    assert whatif.find_cli() == cli_path
    monkeypatch.delenv(whatif.CLI_ENV)
    monkeypatch.setenv("PATH", str(tmp_path))
    assert whatif.find_cli() == tmp_path / "limiter_whatif_cli"
    monkeypatch.setenv("PATH", "")
    monkeypatch.setattr(whatif, "REPO_ROOT", tmp_path / "none")
    with pytest.raises(whatif.LimiterUnavailable, match="not found"):
        whatif.find_cli()
    bad = fake_cli(tmp_path, "#!/bin/sh\necho boom >&2\nexit 4\n")
    s = tmp_path / "s.txt"
    s.write_text("")
    with pytest.raises(whatif.LimiterUnavailable, match="exited 4"):
        whatif.run_cli(bad, s, "recorded")
    with pytest.raises(ValueError):
        whatif.run_cli(bad, s, "weird")


def _real_cli() -> Path | None:
    try:
        return whatif.find_cli()
    except whatif.LimiterUnavailable:
        return None


@pytest.mark.skipif(_real_cli() is None, reason="limiter_whatif_cli not built (ROS job covers it)")
def test_real_tool_reproduces_the_recorded_event_of_the_r2_cut():
    bag = read_session_bag(CUT)
    a = timing.analyse_timing(bag)
    w = whatif.whatif_report(bag, a, _real_cli())  # type: ignore[arg-type]
    r = w["reproduction_recorded_dt"]["rate_limit"]
    assert (r["recorded"], r["matched"], r["missed"], r["extra"]) == (1, 1, 0, 0)


# ---------------------------------------------------------------------------------------- cli


def test_cli_timing_laps_hostfacts_and_stream(tmp_path, monkeypatch, capsys):
    bag = make_bag(events=[(2.0, "rate_limit", RL)])
    bag.cam_t = np.array([3.0])
    bag.cam_lap = np.array([1])
    bag.cam_lap_time_s = np.array([10.0])
    bag.cam_distance_m = np.array([7.4])
    bag.pnp_t = np.array([3.0])
    bag.pnp_lap_index = np.array([1])
    bag.pnp_lap_time_s = np.array([10.0])
    monkeypatch.setattr(cli, "_read", lambda p: bag)
    out = tmp_path / "r.json"
    assert (
        cli.main(["timing", "--bag", "x", "--caps", "0.5", "--no-limiter", "--out", str(out)]) == 0
    )
    assert json.loads(out.read_text())["schema"] == "pnp-session/1"
    assert (
        cli.main(["timing", "--bag", "x", "--no-limiter", "--check-baseline"])
        == cli.EXIT_BASELINE_DIFFERS
    )
    assert cli.main(["timing", "--bag", "x", "--limiter-cli", str(tmp_path / "nope")]) == 0
    assert "NOT RUN" in capsys.readouterr().out
    lj = tmp_path / "l.json"
    assert cli.main(["laps", "--bag", "x", "--out", str(lj)]) == 2  # one run: insufficient
    assert json.loads(lj.read_text())["verdict"] == "INSUFFICIENT"
    assert cli.main(["laps", "--bag", "x", "--reference", "video"]) == 2
    assert cli.main(["laps", "--bag", "x", "--times", "t.csv"]) == 2
    csv = tmp_path / "t.csv"
    csv.write_text("run,lap,lap_time_s\nR0,1,10.0\n")
    assert cli.main(["laps", "--bag", "x", "--reference", "video", "--times", str(csv)]) == 2
    monkeypatch.setattr(hostfacts, "default_runner", lambda c: Done(""))
    assert cli.main(["host-facts", "--out-dir", str(tmp_path / "h")]) == 0
    assert (tmp_path / "h" / "host_facts.json").exists()
    assert (
        cli.main(
            [
                "limiter-stream",
                "--bag",
                "x",
                "--out",
                str(tmp_path / "st.txt"),
                "--events-out",
                str(tmp_path / "ev.json"),
            ]
        )
        == 0
    )
    assert len(json.loads((tmp_path / "ev.json").read_text())) == 1
    empty = SessionBag(name="e")
    monkeypatch.setattr(cli, "_read", lambda p: empty)
    assert cli.main(["timing", "--bag", "x", "--no-limiter"]) == 2


def test_caps_from_split(tmp_path):
    bag = make_bag()
    a = timing.analyse_timing(bag)
    y = tmp_path / "split.yaml"
    y.write_text(
        "bags:\n  synthetic:\n    segments:\n"
        "      - {name: R0, from: 0.5, to: 10.5, driver: pnp, cap_mps: 0.7}\n"
        "      - {name: G, from: 0.0, to: 20.0, driver: gap_follow, cap_mps: 0.9}\n"
    )
    assert cli.caps_from_split(y, "synthetic", a) == [0.7]
    assert cli.caps_from_split(y, "other", a) is None
    y.write_text(
        "bags:\n  synthetic:\n    segments:\n      - {name: R0, from: 50, to: 60, driver: pnp, cap_mps: 0.7}\n"
    )
    assert cli.caps_from_split(y, "synthetic", a) is None


def test_entry_script_runs_help():
    done = subprocess.run(
        [
            sys.executable,
            str(Path(__file__).resolve().parents[1] / "pnp_session_report.py"),
            "--help",
        ],
        capture_output=True,
        text=True,
        check=False,
        env={**os.environ},
    )
    assert done.returncode == 0 and "timing" in done.stdout


def test_npz_roundtrip(tmp_path):
    bag = make_bag()
    p = tmp_path / "b.npz"
    bag.save(p)
    again = SessionBag.load(p)
    assert again.name == "synthetic" and len(again.st_t) == len(bag.st_t)
