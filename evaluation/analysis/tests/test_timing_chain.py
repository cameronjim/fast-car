"""The planner to safety timing chain (brief 42): echo identification of synthetic /drive_raw and
/drive streams with a known latch, the ambiguity cases, the /safety/cycle path, the latched table of
M-T2, the report on a bag with and without the timing topic, M-C3 and the pre-registration check."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
from racer_eval import timing_chain as tc

PERIOD = 0.02
LATENCY = 0.004  # the command reaches safety_node this long after its header stamp


def f32(x: float) -> float:
    return float(np.float32(x))


def stream(
    speeds: list[float],
    steers: list[float],
    gate: dict | None = None,
    n_cycles: int | None = None,
    phase: float = 0.01,
):
    """Commands every 20 ms at stamp j*PERIOD + 0.001; a gate cycle every 20 ms (offset `phase`)
    that latches the latest command that has arrived and passes it through, `gate` mapping a
    command index to the (speed, steer) the gate outputs for it instead."""
    gate = gate or {}
    n = len(speeds)
    cmd_stamp = np.arange(n) * PERIOD + 0.001
    nc = n_cycles if n_cycles is not None else n
    cyc_stamp = np.arange(nc) * PERIOD + phase
    arrive = cmd_stamp + LATENCY
    latched = np.full(nc, -1)
    out_s, out_d = np.zeros(nc), np.zeros(nc)
    for i in range(nc):
        j = int(np.searchsorted(arrive, cyc_stamp[i], side="right")) - 1
        latched[i] = j
        if j >= 0:
            s, d = gate.get(j, (speeds[j], steers[j]))
            out_s[i], out_d[i] = f32(s), f32(d)
    inp = tc.ChainInputs(
        cmd_stamp=cmd_stamp,
        cmd_speed=np.array([f32(v) for v in speeds]),
        cmd_steer=np.array([f32(v) for v in steers]),
        cyc_stamp=cyc_stamp,
        cyc_speed=out_s,
        cyc_steer=out_d,
    )
    return inp, latched, arrive


def test_a_known_latch_gives_the_right_cycle_and_latency():
    speeds = [0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
    steers = [0.0, 0.05, 0.10, 0.15, 0.20, 0.25]
    inp, latched, _arrive = stream(speeds, steers, n_cycles=8)
    e = tc.identify_echoes(inp)
    assert e.source == "echo"
    assert e.status == [tc.STATUS_IDENTIFIED] * 6
    for j in range(6):
        first = int(np.nonzero(latched == j)[0][0])  # the first cycle that latched command j
        assert e.cycle[j] == first
        assert e.latency_s[j] == pytest.approx(inp.cyc_stamp[first] - inp.cmd_stamp[j])
    assert e.share_identified() == 1.0


def test_equal_consecutive_commands_are_ambiguous():
    inp, _l, _a = stream([0.5, 0.5, 0.5, 0.7], [0.1, 0.1, 0.1, 0.1], n_cycles=6)
    e = tc.identify_echoes(inp)
    # command 0 is new (first), 1 and 2 repeat it, 3 changes the speed only
    assert e.status == [
        tc.STATUS_IDENTIFIED,
        tc.STATUS_EQUAL,
        tc.STATUS_EQUAL,
        tc.STATUS_IDENTIFIED,
    ]
    assert e.share_identified() == 0.5


def test_a_gate_that_changes_one_field_still_identifies_by_the_other():
    speeds = [0.5, 0.6, 0.7]
    steers = [0.0, 0.1, 0.2]
    inp, _l, _a = stream(speeds, steers, gate={1: (0.0, 0.1)}, n_cycles=5)  # speed braked to 0
    e = tc.identify_echoes(inp)
    assert e.status == [tc.STATUS_IDENTIFIED] * 3
    # command 1 is found through its steering, which passed through
    assert inp.cyc_steer[e.cycle[1]] == f32(0.1)


def test_a_gate_that_changes_both_fields_is_unmatched():
    speeds = [0.5, 0.6, 0.7]
    steers = [0.0, 0.1, 0.2]
    inp, _l, _a = stream(speeds, steers, gate={1: (0.0, -0.3)}, n_cycles=5)
    e = tc.identify_echoes(inp)
    assert e.status[1] == tc.STATUS_UNMATCHED
    assert e.cycle[1] == -1 and np.isnan(e.latency_s[1])
    assert e.status[0] == e.status[2] == tc.STATUS_IDENTIFIED


def test_a_command_never_latched_is_unmatched():
    # Two commands inside one cycle: the first is overwritten before any cycle sees it.
    inp = tc.ChainInputs(
        cmd_stamp=np.array([0.001, 0.003, 0.05]),
        cmd_speed=np.array([0.5, 0.6, 0.7]),
        cmd_steer=np.array([0.0, 0.1, 0.2]),
        cyc_stamp=np.array([0.01, 0.03, 0.05, 0.07]),
        cyc_speed=np.array([0.6, 0.6, 0.6, 0.7], dtype=np.float32).astype(np.float64),
        cyc_steer=np.array([0.1, 0.1, 0.1, 0.2], dtype=np.float32).astype(np.float64),
    )
    e = tc.identify_echoes(inp)
    assert e.status[0] == tc.STATUS_UNMATCHED


def test_report_without_the_timing_topic_says_so_and_still_does_the_echo():
    inp, _l, _a = stream([0.5, 0.6, 0.7, 0.7], [0.0, 0.1, 0.2, 0.2], n_cycles=7)
    r = tc.chain_report(inp)
    assert r["timing_topic"] is False and "no /pnp/planner/timing" in r["timing_note"]
    assert r["latch_source"] == "echo" and "no /safety/cycle" in r["latch_note"]
    st = r["legs_s"]["publish_to_echo"]
    assert st["n"] == 3 and st["p50"] <= st["p99"] <= st["max"]
    assert r["status_counts"][tc.STATUS_EQUAL] == 1
    assert [a["command"] for a in r["ambiguous"]] == [3]
    json.dumps(r)  # serialisable


def test_report_with_the_timing_topic_has_every_leg():
    inp, _l, _a = stream([0.5, 0.6, 0.7], [0.0, 0.1, 0.2], n_cycles=5)
    n = 3
    base = inp.cmd_stamp + 10.0  # node clock stamps are positive
    inp.tim_stamp = base.copy()
    inp.tim_scan_stamp = base - 0.1
    inp.tim_scan_receipt = base - 0.08  # 20 ms transport
    inp.tim_solve_end = base - 0.002
    inp.tim_final = np.array([0, 1, 2])
    inp.tim_limiter = np.array([1, 2, 7])
    r = tc.chain_report(inp)
    assert r["timing_topic"] is True and r["timing_rows"] == n
    assert r["legs_s"]["scan_stamp_to_receipt"]["p50"] == pytest.approx(0.02)
    assert r["legs_s"]["scan_receipt_to_solve_end"]["p50"] == pytest.approx(0.078)
    assert r["legs_s"]["solve_end_to_publish"]["p50"] == pytest.approx(0.002)
    assert r["final_sequence"] == {"mean": 1, "fallback": 1, "all_collide": 1}
    assert r["speed_limiter"]["cap"] == 1 and r["speed_limiter"]["sample_choice"] == 1


def test_timing_rows_without_a_scan_leave_the_scan_legs_empty():
    inp, _l, _a = stream([0.5, 0.6], [0.0, 0.1], n_cycles=4)
    inp.tim_stamp = inp.cmd_stamp.copy()
    inp.tim_scan_stamp = np.zeros(2)
    inp.tim_scan_receipt = np.zeros(2)
    inp.tim_solve_end = np.zeros(2)
    inp.tim_final = np.zeros(2, dtype=np.int64)
    inp.tim_limiter = np.zeros(2, dtype=np.int64)
    r = tc.chain_report(inp)
    assert r["legs_s"]["scan_stamp_to_receipt"]["n"] == 0
    assert r["legs_s"]["solve_end_to_publish"]["n"] == 0


def test_empty_bag_is_not_a_crash():
    r = tc.chain_report(tc.ChainInputs())
    assert r["commands"] == 0 and r["share_identified"] is None
    assert tc.identify_echoes(tc.ChainInputs()).share_identified() != 0.0  # nan


def test_latched_table_from_the_echo_matches_the_truth_when_nothing_is_ambiguous():
    speeds = [0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2]
    steers = [0.0, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.14]
    inp, latched, _a = stream(speeds, steers, n_cycles=10)
    lat = tc.latched_commands(inp)
    assert lat.source == "echo"
    for i in range(len(latched)):
        if i in lat.ambiguous:
            continue
        assert lat.command[i] == latched[i]
        assert lat.speed[i] == pytest.approx(f32(speeds[latched[i]]))
        assert lat.age[i] == pytest.approx(inp.cyc_stamp[i] - inp.cmd_stamp[latched[i]])


def test_cycles_after_an_unidentified_command_are_listed_ambiguous():
    # Command 2 equals command 1: its latch is unknown, so the cycles from its publish on are
    # ambiguous until the next identified command is echoed.
    speeds = [0.5, 0.6, 0.6, 0.8, 0.9]
    steers = [0.0, 0.1, 0.1, 0.3, 0.4]
    inp, _l, _a = stream(speeds, steers, n_cycles=7)
    lat = tc.latched_commands(inp)
    assert len(lat.ambiguous) >= 1
    assert all(inp.cyc_stamp[i] > inp.cmd_stamp[2] for i in lat.ambiguous if i > 1)
    # the replay still gets a command for them (the latest published at or before the cycle)
    assert all(lat.command[i] >= 0 for i in lat.ambiguous)


def test_before_any_command_the_age_is_infinite():
    inp, _l, _a = stream([0.5, 0.6], [0.0, 0.1], n_cycles=4, phase=0.0005)
    lat = tc.latched_commands(inp)
    assert lat.command[0] == -1 and np.isinf(lat.age[0]) and 0 in lat.ambiguous


def safety_cycle_inputs():
    speeds = [0.5, 0.6, 0.6, 0.8]
    steers = [0.0, 0.1, 0.1, 0.3]
    inp, latched, arrive = stream(speeds, steers, n_cycles=6)
    # /safety/cycle: per cycle the header stamp of the latched command (nan before the first), age
    inp.sc_stamp = inp.cyc_stamp.copy()
    inp.sc_raw_stamp = np.array(
        [inp.cmd_stamp[j] if j >= 0 else np.nan for j in latched], dtype=np.float64
    )
    inp.sc_age = np.array(
        [inp.cyc_stamp[i] - arrive[j] if j >= 0 else -1.0 for i, j in enumerate(latched)]
    )
    return inp, latched


def test_with_a_safety_cycle_record_the_latch_is_exact_and_says_so():
    inp, latched = safety_cycle_inputs()
    e = tc.identify_echoes(inp)
    assert e.source == "safety_cycle"
    # Equal consecutive commands are identified exactly now (the record names the command).
    assert e.status == [tc.STATUS_IDENTIFIED] * 4
    r = tc.chain_report(inp)
    assert r["latch_source"] == "safety_cycle" and "from /safety/cycle" in r["latch_note"]
    lat = tc.latched_commands(inp)
    assert lat.source == "safety_cycle"
    for i, j in enumerate(latched):
        if j >= 0:
            assert lat.command[i] == j
            assert lat.age[i] == pytest.approx(inp.sc_age[i])
        else:
            assert lat.command[i] == -1


def test_a_safety_record_for_a_command_not_in_the_bag_is_listed():
    inp, _latched = safety_cycle_inputs()
    inp.sc_raw_stamp = inp.sc_raw_stamp + 5.0  # no such command
    lat = tc.latched_commands(inp)
    finite = np.isfinite(inp.sc_raw_stamp)
    assert set(lat.ambiguous) >= set(np.nonzero(finite)[0].tolist())


def test_a_cycle_without_a_record_is_listed():
    inp, _latched = safety_cycle_inputs()
    inp.sc_stamp = inp.sc_stamp[1:]
    inp.sc_raw_stamp = inp.sc_raw_stamp[1:]
    inp.sc_age = inp.sc_age[1:]
    assert 0 in tc.latched_commands(inp).ambiguous


def test_command_quantisation_distinct_speeds_and_steady_wheel_speed():
    # 0.57 held 2 s then 0.75 held 2 s, the wheel speed settling to 0.55 and 0.74
    t = np.arange(0.0, 4.0, 0.02)
    cmd = np.where(t < 2.0, 0.57, 0.75)
    odom_t = t + 0.003
    odom_v = np.where(t < 2.0, 0.55, 0.74)
    r = tc.command_quantisation(t, cmd, odom_t, odom_v)
    assert r["distinct_commanded_speeds"] == 2
    rows = {round(x["commanded_mps"], 2): x for x in r["rows"]}
    assert rows[0.57]["steady_wheel_mps_median"] == pytest.approx(0.55)
    assert rows[0.75]["steady_wheel_mps_median"] == pytest.approx(0.74)
    assert rows[0.57]["holds"] == 1 and rows[0.57]["commands"] == 100


def test_a_speed_held_too_briefly_has_no_steady_value():
    t = np.arange(0.0, 1.0, 0.02)
    cmd = np.where(t < 0.5, 0.57, 0.75)
    r = tc.command_quantisation(t, cmd, t, np.full_like(t, 0.5))
    assert all(row["holds"] == 0 and row["steady_wheel_mps_median"] is None for row in r["rows"])


def test_inputs_from_rows_and_the_missing_topics():
    rows = {
        tc.DRIVE_RAW: [(1.0, 0.5, 0.1), (1.02, 0.6, 0.1)],
        tc.DRIVE: [(1.01, 0.5, 0.1)],
        tc.TIMING: [(1.0, 0.9, 0.92, 0.99, 1, 2)],
        tc.SAFETY_CYCLE: [(1.01, 1.0, 0.004)],
        tc.ODOM: [(1.0, 0.4)],
    }
    inp, odom_t, odom_v = tc.inputs_from_rows(rows)
    assert len(inp.cmd_stamp) == 2 and inp.tim_final[0] == 1 and inp.tim_limiter[0] == 2
    assert inp.sc_raw_stamp[0] == 1.0 and odom_v[0] == 0.4 and odom_t[0] == 1.0
    empty, _t, _v = tc.inputs_from_rows({})
    assert len(empty.cmd_stamp) == 0 and len(empty.sc_stamp) == 0


def fake_repo(root: Path, v04_text: str, add_text: str) -> None:
    (root / "evaluation" / "prereg").mkdir(parents=True)
    (root / tc.V04_YAML).write_text(v04_text)
    (root / tc.ADDENDUM_YAML).write_text(add_text)


def test_prereg_check_refuses_without_the_files_or_with_a_different_sha(tmp_path):
    with pytest.raises(SystemExit, match="not on this checkout"):
        tc.require_prereg(tmp_path)
    fake_repo(tmp_path / "r", "x", "y")
    with pytest.raises(SystemExit, match="not the registered"):
        tc.require_prereg(tmp_path / "r")


def test_prereg_check_passes_on_the_registered_bytes(tmp_path, monkeypatch):
    fake_repo(tmp_path, "a", "b")
    monkeypatch.setattr(tc, "V04_SHA256", hashlib.sha256(b"a").hexdigest())
    monkeypatch.setattr(tc, "ADDENDUM_SHA256", hashlib.sha256(b"b").hexdigest())
    tc.require_prereg(tmp_path)


def test_main_writes_the_report(tmp_path, monkeypatch, capsys):
    fake_repo(tmp_path, "a", "b")
    monkeypatch.setattr(tc, "V04_SHA256", hashlib.sha256(b"a").hexdigest())
    monkeypatch.setattr(tc, "ADDENDUM_SHA256", hashlib.sha256(b"b").hexdigest())
    inp, _l, _a = stream([0.5, 0.6, 0.7], [0.0, 0.1, 0.2], n_cycles=5)
    odom = np.arange(0.0, 0.2, 0.01)
    monkeypatch.setattr(tc, "read_chain_bag", lambda p: (inp, odom, np.full_like(odom, 0.4)))
    out = tmp_path / "r.json"
    rc = tc.main([str(tmp_path / "bag"), "--json", str(out), "--repo-root", str(tmp_path)])
    assert rc == 0
    doc = json.loads(out.read_text())
    assert doc["m_c2"]["commands"] == 3 and "m_c3" in doc
    assert tc.ADDENDUM_YAML in doc["prereg"]
    assert "m_c2" in capsys.readouterr().out
