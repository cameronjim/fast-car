"""tools/pnp_session/session_checks.py (checklist P2 of claude-docs/fix-checklist-2026-11-22.md): the
checks done by hand on car session 1, on small written message lists (L1) and on a cut of the
real 2026-11-22 bag (L4, tools/tests/fixtures/pnp_session_2026_11_22_cut.mcap, made by
pnp_session/cut_session_fixture.py). The synthetic runs are a planner at 50 Hz with the gate
cycle at 50 Hz, wheel odometry, and the lap node and the camera at fixed laps."""

from __future__ import annotations

import io
import json
import math
from contextlib import redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

pytest.importorskip("mcap")
pytest.importorskip("mcap_ros2")

from pnp_session import cli, cut_session_fixture
from pnp_session import session_checks as sc

FIX = Path(__file__).parent / "fixtures" / "pnp_session_2026_11_22_cut.mcap"
T0_NS = 1_700_000_000 * 1_000_000_000
DT = 0.02


def _stamp(t: float) -> SimpleNamespace:
    # header stamps are absolute (epoch) seconds; the bag axis starts at T0_NS
    a = T0_NS / 1e9 + t
    return SimpleNamespace(sec=int(a), nanosec=round((a - int(a)) * 1e9))


def ns(t: float) -> int:
    return T0_NS + round(t * 1e9)


class Bag:
    """Messages of one synthetic bag, on the bag axis."""

    def __init__(self) -> None:
        self.msgs: list[tuple[int, str, Any]] = []

    def add(self, t: float, topic: str, msg: Any) -> None:
        self.msgs.append((ns(t), topic, msg))

    def planner(
        self, t: float, state: int = 1, cmd: float = 1.0, est: float = 1.0, reason: int = 0,
        samples: int = 256, solve: float = 0.002, clear: float = 0.3,
    ) -> None:  # fmt: skip
        self.add(
            t,
            sc.TOPIC_PLANNER,
            SimpleNamespace(
                header=SimpleNamespace(stamp=_stamp(t)), state=state, cmd_speed_mps=cmd,
                est_speed_mps=est, v_cap_mps=1.0, plan_min_clearance_m=clear,
                solve_time_s=solve, samples=samples, block_reason=reason,
            ),
        )  # fmt: skip

    def odom(self, t: float, v: float) -> None:
        self.add(
            t,
            sc.TOPIC_ODOM,
            SimpleNamespace(
                header=SimpleNamespace(stamp=_stamp(t)),
                twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=v))),
            ),
        )

    def raw(self, t: float) -> None:
        self.add(t, sc.TOPIC_DRIVE_RAW, SimpleNamespace(header=SimpleNamespace(stamp=_stamp(t))))

    def cycle(self, t: float, count: int, dt: float = DT, age: float = 0.01) -> None:
        self.add(
            t,
            sc.TOPIC_SAFETY_CYCLE,
            SimpleNamespace(has_command=True, drive_raw_age_s=age, drive_raw_count=count, cycle_dt_s=dt),
        )  # fmt: skip

    def safety(self, t: float, source: str, phase: int = 0, severity: int = 0) -> None:
        self.add(
            t,
            sc.TOPIC_SAFETY_EVENTS,
            SimpleNamespace(severity=severity, phase=phase, source=source, detail="x"),
        )

    def camera(self, t: float, lap: int, peak: float, dist: float) -> None:
        d = {
            "lap": lap, "lap_time_s": dist, "peak_similarity": peak,
            "crossing_time_s": T0_NS / 1e9 + t - 0.2, "distance_m": dist,
        }  # fmt: skip
        self.add(t, sc.TOPIC_CAM_EVENT, SimpleNamespace(data=json.dumps(d)))

    def lap(
        self, t: float, lap: int, length: float, clean: bool = True, brakes: int = 0,
        clear: float = 0.3, cap: float = 1.0,
    ) -> None:  # fmt: skip
        self.add(
            t,
            sc.TOPIC_PNP_EVENT,
            SimpleNamespace(
                lap_index=lap, lap_time_s=length, lap_length_m=length, v_cap_mps=cap,
                safety_brakes=brakes, min_clearance_m=clear, clean=clean,
            ),
        )  # fmt: skip

    def status(self, t: float, **kw: Any) -> None:
        d = {"lap_mode": "observe", "lap_source": "camera", "l_ref_m": 5.5, **kw}
        self.add(t, sc.TOPIC_PNP_STATUS, SimpleNamespace(data=json.dumps(d)))

    def drive(self, t0: float, t1: float, v: float = 1.0, **planner: Any) -> None:
        """Planner, odometry, /drive_raw and gate cycle at 50 Hz over [t0, t1]."""
        n = round((t1 - t0) / DT)
        for k in range(n + 1):
            t = t0 + k * DT
            self.planner(t, est=v, **planner)
            self.odom(t, v)
            self.raw(t)
            self.cycle(t + 0.001, k + 1)

    def data(self) -> sc.SessionData:
        self.msgs.sort(key=lambda m: m[0])
        return sc.data_from_messages("synthetic", T0_NS, self.msgs)


def blocked_run(t_block: float, ticks: int, v: float = 1.0) -> Bag:
    """A run of 20 s at v with BLOCKED (below the speed floor) for `ticks` ticks from t_block."""
    b = Bag()
    b.drive(0.0, 20.0, v)
    # overwrite the planner ticks of the block
    for k, m in enumerate(b.msgs):
        if m[1] == sc.TOPIC_PLANNER:
            t = (m[0] - T0_NS) / 1e9
            if t_block - 1e-6 <= t < t_block + ticks * DT - 1e-6:
                m[2].state = sc.STATE_BLOCKED
                m[2].block_reason = 2
                m[2].cmd_speed_mps = 0.0
                m[2].est_speed_mps = v * 0.5
    return b


# ---- runs -------------------------------------------------------------------------------------------


def test_runs_split_on_a_gap_over_five_seconds() -> None:
    b = Bag()
    b.drive(0.0, 3.0)
    b.drive(8.02, 10.0)  # gap of 5.04 s
    b.drive(14.0, 15.0)  # gap of 4.0 s: the same run
    runs = sc.find_runs(b.data())
    assert len(runs) == 2
    assert runs[0] == pytest.approx((0.0, 3.0))
    assert runs[1] == pytest.approx((8.02, 15.0))


def test_a_bag_with_no_planner_status_has_no_run_and_says_so() -> None:
    d = Bag().data()
    assert sc.find_runs(d) == []
    assert "no run found" in sc.markdown(sc.analyse(d))


# ---- blips ------------------------------------------------------------------------------------------


def test_a_short_blocked_stretch_while_moving_is_a_blip_with_its_speed_lost() -> None:
    d = blocked_run(10.0, ticks=3).data()
    (x,) = sc.blips(d, 0.0, 20.0)
    assert x["kind"] == "blip"
    assert x["reason"] == "below_speed_floor"
    assert x["ticks"] == 3
    assert x["duration_s"] == pytest.approx(0.06, abs=1e-6)
    assert x["cmd_min_mps"] == 0.0
    assert x["speed_before_mps"] == pytest.approx(1.0)
    assert x["speed_lost_mps"] == pytest.approx(0.5)


def test_a_long_blocked_stretch_is_a_block_not_a_blip() -> None:
    d = blocked_run(10.0, ticks=40).data()  # 0.8 s
    (x,) = sc.blips(d, 0.0, 20.0)
    assert x["kind"] == "block"


def test_a_blocked_stretch_with_the_car_standing_is_a_block() -> None:
    d = blocked_run(10.0, ticks=3, v=0.1).data()
    (x,) = sc.blips(d, 0.0, 20.0)
    assert x["kind"] == "block"


def test_a_blocked_stretch_at_the_end_of_the_run_is_a_block() -> None:
    d = blocked_run(19.9, ticks=6).data()
    (x,) = sc.blips(d, 0.0, 20.0)
    assert x["kind"] == "block"


# ---- samples ----------------------------------------------------------------------------------------


def test_samples_histogram_share_at_max_and_late_solves() -> None:
    b = Bag()
    for k in range(100):
        b.planner(k * DT, samples=512 if k < 10 else 256, solve=0.020 if k == 50 else 0.002)
    b.planner(2.1, state=0, samples=999)  # not DRIVING: not in the histogram
    s = sc.samples_block(b.data(), 0.0, 3.0)
    assert s["histogram"] == {256: 90, 512: 10}
    assert s["driving_ticks"] == 100
    assert s["share_at_max"] == pytest.approx(0.10)
    assert s["share_at_full"] == pytest.approx(0.10) and s["median"] == 256.0
    assert s["solve_ms"]["late_over_15ms"] == 1
    assert s["solve_ms"]["max"] == pytest.approx(20.0)
    assert s["solve_ms"]["p50"] == pytest.approx(2.0)


# ---- start ------------------------------------------------------------------------------------------


def test_start_reports_the_wheel_response_and_flags_an_unanswered_car() -> None:
    ok = Bag()
    for k in range(100):
        t = k * DT
        ok.planner(t, cmd=1.0)
        ok.odom(t, 0.0 if t < 0.5 else 0.8)
    s = sc.start_block(ok.data(), 0.0, 2.0)
    assert s["wheel_response_s"] == pytest.approx(0.5, abs=DT)
    assert not s["unanswered"]
    stuck = Bag()
    for k in range(400):
        t = k * DT
        stuck.planner(t, cmd=1.0)
        stuck.odom(t, 0.0 if t < 5.0 else 0.8)
    s = sc.start_block(stuck.data(), 0.0, 8.0)
    assert s["wheel_response_s"] == pytest.approx(5.0, abs=DT)
    assert s["unanswered"]
    never = Bag()
    for k in range(100):
        never.planner(k * DT, cmd=1.0)
        never.odom(k * DT, 0.0)
    assert sc.start_block(never.data(), 0.0, 2.0)["unanswered"]
    idle = Bag()
    idle.planner(0.0, cmd=0.0)
    assert sc.start_block(idle.data(), 0.0, 1.0)["first_command_s"] is None


# ---- engagements and latch --------------------------------------------------------------------------


def _cycles_with(b: Bag, late_at: float | None = None, gap_at: float | None = None) -> None:
    """Planner, odometry and a gate cycle, with one late gate cycle or one missing /drive_raw."""
    count = 0
    for k in range(500):
        t = k * DT
        b.planner(t)
        b.odom(t, 1.0)
        if gap_at is None or not (gap_at <= t < gap_at + 0.06):
            b.raw(t)
            count += 1
        late = late_at is not None and abs(t - late_at) < 1e-9
        b.cycle(t + 0.001, count, dt=0.066 if late else DT)


def test_a_late_gate_cycle_is_the_leg_at_fault() -> None:
    b = Bag()
    _cycles_with(b, late_at=5.0)
    b.safety(5.01, "watchdog")
    (x,) = sc.engagements(b.data(), 0.0, 10.0)
    assert x["leg"] == "safety_cycle_late"
    assert x["cycle_dt_max_s"] == pytest.approx(0.066)


def test_a_drive_raw_publish_gap_is_the_leg_at_fault() -> None:
    b = Bag()
    _cycles_with(b, gap_at=5.0)  # 3 messages missing: a receive gap of 80 ms
    b.safety(5.07, "watchdog")
    (x,) = sc.engagements(b.data(), 0.0, 10.0)
    assert x["leg"] == "command_gap"
    assert x["drive_raw_gap_max_s"] == pytest.approx(0.08, abs=1e-6)


def test_late_cycle_and_gap_together_and_neither() -> None:
    b = Bag()
    _cycles_with(b, late_at=5.0, gap_at=4.96)
    b.safety(5.01, "watchdog")
    b.safety(8.0, "rate_limit")
    first, second = sc.engagements(b.data(), 0.0, 10.0)
    assert first["leg"] == "both"
    assert second["leg"] == "undetermined"


def test_only_engages_of_watchdog_and_rate_limit_inside_the_run_count() -> None:
    b = Bag()
    _cycles_with(b)
    b.safety(5.0, "watchdog", phase=1)  # a release
    b.safety(5.0, "ttc", severity=2)  # another gate
    b.safety(5.0, "rate_limit")
    b.safety(60.0, "watchdog")  # outside
    out = sc.engagements(b.data(), 0.0, 10.0)
    assert [x["source"] for x in out] == ["rate_limit"]


def test_an_engagement_at_the_first_drive_raw_is_marked() -> None:
    b = Bag()
    _cycles_with(b)
    b.safety(0.1, "rate_limit")
    b.safety(5.0, "rate_limit")
    first, later = sc.engagements(b.data(), 0.0, 10.0)
    assert first["at_first_command"] and not later["at_first_command"]


def test_latch_repeats_and_skips() -> None:
    b = Bag()
    counts = [1, 2, 2, 2, 4, 5, 7, 7]
    for k, c in enumerate(counts):
        b.cycle(k * DT, c, dt=DT, age=0.02)
    lt = sc.latch_block(b.data(), 0.0, 1.0)
    assert lt["cycles"] == 8
    assert lt["repeated"] == 3  # 2->2, 2->2, 7->7
    assert lt["skipped"] == 2  # 2->4 and 5->7; 4->5 is a step of one
    assert lt["command_age_ms_p50"] == pytest.approx(20.0)
    assert lt["dt_ms"]["over_45ms"] == 0


# ---- lap table --------------------------------------------------------------------------------------


def _laps_bag() -> Bag:
    b = Bag()
    b.drive(0.0, 60.0)
    # lap 1 starts without a reference, laps 2 to 4 are 5.5, 5.6 and 11.0 (a missed crossing)
    for t, lap, peak, dist in ((6.0, 1, 0.91, 5.5), (12.0, 2, 0.86, 5.6), (24.0, 3, 0.83, 11.0)):
        b.camera(t - 0.05, lap, peak, dist)
    b.status(5.0, l_ref_m=0.0)
    b.status(7.0, l_ref_m=5.5)
    b.status(
        30.0,
        l_ref_m=5.5,
        doubtful_laps=[{"lap": 3, "event_lap": 3, "reasons": ["length", "heading"]}],
        missed_gate_laps=[3],
    )
    b.lap(6.0, 1, 5.5)
    b.lap(12.0, 2, 5.6, clean=False, clear=0.05)
    b.lap(24.0, 3, 11.0, clean=False, brakes=1)
    return b


def test_lap_table_rows_ratio_and_why_not_clean() -> None:
    d = _laps_bag().data()
    t = sc.lap_table(d, 0.0, 60.0, [])
    r1, r2, r3 = t["rows"]
    assert r1["peak"] == pytest.approx(0.91) and r1["ref_m"] is None and r1["ratio"] is None
    assert r1["clean"] and r1["why_not_clean"] == []
    assert r2["ratio"] == pytest.approx(5.6 / 5.5)
    assert r2["why_not_clean"] == ["clearance 0.05"]
    assert r3["why_not_clean"] == ["brakes 1", "doubtful length+heading", "gate missed"]
    assert t["camera_events"] == 3 and t["lap_node_events"] == 3
    assert t["mode"] == "observe" and t["source"] == "camera"


def test_peak_table_and_missed_crossings() -> None:
    t = sc.lap_table(_laps_bag().data(), 0.0, 60.0, [])
    p = t["peaks"]
    assert (p["n"], p["min"], p["max"]) == (3, pytest.approx(0.83), pytest.approx(0.91))
    assert p["median"] == pytest.approx(0.86)
    assert p["under_floor"] == [0.83, 0.86]
    (m,) = t["missed_crossings"]
    assert (m["camera_lap"], m["laps_in_it"], m["missed"]) == (3, 2, 1)
    assert t["reference_m"] == pytest.approx(5.5)
    assert t["wheel_distance_m"] == pytest.approx(60.0, rel=1e-3)
    assert t["laps_driven"] == pytest.approx(60.0 / 5.5, rel=1e-3)


def test_a_blip_and_a_non_driving_tick_inside_a_lap_are_named() -> None:
    b = _laps_bag()
    for m in b.msgs:
        if m[1] == sc.TOPIC_PLANNER and abs((m[0] - T0_NS) / 1e9 - 8.0) < 1e-6:
            m[2].state = sc.STATE_BLOCKED
    d = b.data()
    t = sc.lap_table(d, 0.0, 60.0, sc.blips(d, 0.0, 60.0))
    assert t["rows"][1]["blips"] == 1
    assert "blip x1" in t["rows"][1]["why_not_clean"]
    assert "not DRIVING x1" in t["rows"][1]["why_not_clean"]


def test_a_clean_lap_with_a_brake_keeps_it_as_a_flag_not_a_reason() -> None:
    b = Bag()
    b.drive(0.0, 10.0)
    b.status(1.0)
    b.lap(5.0, 1, 5.5, clean=True, brakes=1)
    (row,) = sc.lap_table(b.data(), 0.0, 10.0, [])["rows"]
    assert row["clean"] and row["why_not_clean"] == [] and row["other_flags"] == ["brakes 1"]


def test_the_report_text_and_json_of_a_synthetic_bag() -> None:
    b = _laps_bag()
    b.safety(5.0, "watchdog")
    doc = sc.analyse(b.data())
    text = sc.markdown(doc)
    assert "## Run 1" in text and "Missed crossings: camera lap 3" in text
    assert f"log time {T0_NS / 1e9:.3f} to {T0_NS / 1e9 + 60.0:.3f}" in text
    assert "Watchdog and rate_limit engagements:" in text
    assert "Blips: none." in text
    json.dumps(doc)  # serialisable as it is


# ---- L4: a cut of the real bag ----------------------------------------------------------------------


@pytest.fixture(scope="module")
def real() -> dict[str, Any]:
    d = sc.read_bag(FIX)
    assert all(rep.complete for rep in d.reports)
    return sc.analyse(d)


def test_real_cut_has_the_four_windows_as_runs(real: dict[str, Any]) -> None:
    starts = [r["start_s"] for r in real["runs"]]
    assert starts == pytest.approx([428.5, 1204.5, 1256.0, 1265.5], abs=0.05)


def test_real_cut_the_double_lap_and_the_blip_of_run_one_of_the_session(
    real: dict[str, Any],
) -> None:
    run = real["runs"][0]
    (row,) = run["laps"]["rows"]
    assert row["peak"] == pytest.approx(0.849, abs=0.001)
    assert row["length_m"] == pytest.approx(11.16, abs=0.01)
    assert row["ratio"] == pytest.approx(2.03, abs=0.01)
    assert not row["clean"]
    assert {"doubtful length+heading", "gate missed", "blip x1"} <= set(row["why_not_clean"])
    (m,) = run["laps"]["missed_crossings"]
    assert m["missed"] == 1
    (blip,) = run["blips"]
    assert blip["t_s"] == pytest.approx(430.35, abs=0.02)
    assert blip["reason"] == "below_speed_floor"
    assert blip["duration_s"] == pytest.approx(0.037, abs=0.01)
    assert blip["speed_lost_mps"] == pytest.approx(0.45, abs=0.05)


def test_real_cut_the_three_watchdog_engagements_with_their_legs(real: dict[str, Any]) -> None:
    late = real["runs"][1]["engagements"]
    assert [x["source"] for x in late] == ["watchdog", "rate_limit"]
    assert {x["leg"] for x in late} == {"safety_cycle_late"}
    assert late[0]["cycle_dt_max_s"] == pytest.approx(0.066, abs=0.002)
    (gap1,) = real["runs"][2]["engagements"]
    (gap2,) = real["runs"][3]["engagements"]
    assert (gap1["leg"], gap2["leg"]) == ("command_gap", "command_gap")
    assert gap1["drive_raw_gap_max_s"] == pytest.approx(0.074, abs=0.002)
    assert gap2["drive_raw_gap_max_s"] == pytest.approx(0.075, abs=0.002)


def test_real_cut_lap_five_and_six_of_the_d_run(real: dict[str, Any]) -> None:
    (five,) = real["runs"][2]["laps"]["rows"]
    (six,) = real["runs"][3]["laps"]["rows"]
    assert five["peak"] == pytest.approx(0.901, abs=0.001) and five["clean"]
    assert five["ratio"] == pytest.approx(0.98, abs=0.01)
    assert six["peak"] == pytest.approx(0.873, abs=0.001) and not six["clean"]
    assert six["ratio"] == pytest.approx(0.87, abs=0.01)
    assert "doubtful length" in six["why_not_clean"]
    assert real["runs"][3]["laps"]["peaks"]["under_floor"] == [0.873]


def test_real_cut_samples_are_far_below_the_most_seen(real: dict[str, Any]) -> None:
    pl = real["runs"][2]["planner"]
    assert pl["max_samples"] == 160 and pl["median"] == 128.0
    assert pl["share_at_full"] == 0.0
    assert pl["solve_ms"]["p50"] == pytest.approx(1.5, abs=0.3)


# ---- the command line and the cutting tool ----------------------------------------------------------


def test_cli_session_prints_and_writes_json(tmp_path: Path) -> None:
    out = tmp_path / "s.json"
    buf = io.StringIO()
    with redirect_stdout(buf):
        code = cli.main(["session", "--bag", str(FIX), "--out", str(out)])
    assert code == 0
    assert "# Session report: pnp_session_2026_11_22_cut.mcap" in buf.getvalue()
    doc = json.loads(out.read_text())
    assert doc["schema"] == sc.SCHEMA and len(doc["bags"][0]["runs"]) == 4


def test_cut_session_fixture_keeps_the_axis_and_the_window(tmp_path: Path) -> None:
    out = tmp_path / "cut.mcap"
    n = cut_session_fixture.cut(FIX, [(1256.0, 1257.5)], out)
    assert 0 < n < 2315
    d = sc.read_bag(out)
    (run,) = sc.find_runs(d)
    assert run[0] == pytest.approx(1256.0, abs=0.05) and run[1] <= 1257.5
    assert d.t0_log_s == pytest.approx(sc.read_bag(FIX).t0_log_s)
    assert math.isfinite(run[1])
