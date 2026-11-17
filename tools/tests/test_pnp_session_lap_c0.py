"""tools/pnp_session/lap_c0.py (brief 50): D3-C0, the D7-C0 rule and D8-C0 for the same runs.

Each item (i) to (v) at pass, at the threshold and epsilon over, a reference that fails its validity
rule (REPEAT, never PASS or FAIL), lap 1 judged by its crossing instant, one run or under 10 laps
(INSUFFICIENT), the video CSV as reference, D3-C0's numbers through the new command equal
`laps.d3c0`, the thresholds read from the two registered yaml files, a bag written with
mcap-ros2-support and the command line."""

from __future__ import annotations

import io
import json
import math
from contextlib import redirect_stdout
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("mcap")
pytest.importorskip("racer_eval")

from pnp_session import cli, lap_c0
from pnp_session import laps as d3
from pnp_session.lap_c0 import FAIL, INSUFFICIENT, NO_REFERENCE, PASS, REPEAT, LapRun

TH = lap_c0.load_thresholds()
LAP = 5.2


def make_run(
    run_id: str = "run_ccw",
    n: int = 12,
    offset: float = 0.5,
    missed: tuple[int, ...] | None = (),
    p99: float = 12.0,
    sources: str = "start_scan",
    wheel_extra: float = 0.4,
    ref_laps: int | None = None,
) -> LapRun:
    """A clean run: system and reference laps of 5.2 s, the camera crossing `offset` s after the
    system's, lap distance 5.2 m, wheel distance a little over the n laps."""
    n_ref = n if ref_laps is None else ref_laps
    sys_cross = tuple(100.0 + LAP * (k + 1) for k in range(n))
    ref_cross = tuple(100.0 + LAP * (k + 1) + offset for k in range(n_ref))
    return LapRun(
        run_id=run_id,
        sys_times=(LAP,) * n,
        sys_cross=sys_cross,
        ref_times=(LAP,) * n_ref,
        ref_cross=ref_cross,
        ref_distance_m=(LAP,) * n_ref,
        wheel_distance_m=n * LAP + wheel_extra,
        missed_laps=missed,
        line_sources=(sources,) * n,
        on_scan_p99_ms=p99,
        lap_mode="observe",
    )


def two(**kw: object) -> list[LapRun]:
    return [make_run("run_ccw", **kw), make_run("run_cw", **kw)]  # type: ignore[arg-type]


def items(block: dict) -> dict[str, str]:
    return {it["item"].split(")")[0] + ")": it["verdict"] for it in block["items"]}


def verdict_of(runs: list[LapRun], rule: str = "d8") -> dict:
    return lap_c0.evaluate(runs, rule, TH)


# -- thresholds ---------------------------------------------------------------------------------------


def test_the_thresholds_are_read_from_the_two_registered_files() -> None:
    assert TH.d7["laps_2_to_10_min_within"] == 17 and TH.d7["missed_gate_flags_allowed"] == 0
    assert TH.d8["on_scan_p99_max_ms"] == 80 and TH.d8["runs"] == 2 and TH.d8["laps_judged"] == 10
    assert TH.d7["tolerance_s"] == TH.d8["tolerance_s"] == 0.3
    assert TH.d7_sha256 == lap_c0._sha256(lap_c0.REPO_ROOT / "evaluation/prereg/pnp-v0.5-laps.yaml")
    assert TH.d8_sha256 == lap_c0._sha256(
        lap_c0.REPO_ROOT / "evaluation/prereg/pnp-v0.5-laps-ring.yaml"
    )


def test_a_changed_constants_file_is_refused(tmp_path: Path) -> None:
    ring = lap_c0.REPO_ROOT / "evaluation/prereg/pnp-v0.5-laps-ring.yaml"
    changed = tmp_path / "ring.yaml"
    changed.write_text(ring.read_text().replace("on_scan_p99_max_ms: 80", "on_scan_p99_max_ms: 90"))
    with pytest.raises(lap_c0.PreregError, match="SHA-256"):
        lap_c0.load_thresholds(d8_path=changed)
    v05 = lap_c0.REPO_ROOT / "evaluation/prereg/pnp-v0.5-laps.yaml"
    other = tmp_path / "v05.yaml"
    other.write_text(v05.read_text().replace("tolerance_s: 0.3", "tolerance_s: 0.4"))
    with pytest.raises(lap_c0.PreregError, match="SHA-256"):
        lap_c0.load_thresholds(d7_path=other)


# -- the clean run passes ----------------------------------------------------------------------------------


@pytest.mark.parametrize("rule", ["d7", "d8"])
def test_a_clean_pair_of_runs_passes_every_item(rule: str) -> None:
    out = verdict_of(two(), rule)
    assert out["verdict"] == PASS
    assert set(items(out).values()) == {PASS}
    assert len(out["items"]) == (5 if rule == "d8" else 4)


def test_the_d7_rule_has_no_on_scan_item_and_d8_has_it() -> None:
    labels = [it["item"] for it in verdict_of(two(), "d8")["items"]]
    assert labels[-1].startswith("(v)") and "80" in labels[-1]
    assert not any(it["item"].startswith("(v)") for it in verdict_of(two(), "d7")["items"])


# -- (i) the count ----------------------------------------------------------------------------------------


def test_item_i_the_lap_count_equals_the_references_in_every_run() -> None:
    runs = [make_run("a", n=12), replace(make_run("b", n=12), sys_times=(LAP,) * 11)]
    out = verdict_of(runs)
    assert items(out)["(i)"] == FAIL and out["verdict"] == FAIL
    # the same in the other direction: a lap the system has and the reference lacks
    runs = [make_run("a", n=12), make_run("b", n=12, wheel_extra=0.4)]
    runs[1] = replace(runs[1], sys_times=(LAP,) * 13, sys_cross=runs[1].sys_cross + (200.0,))
    assert items(verdict_of(runs))["(i)"] == FAIL


# -- (ii) 17 of the 18 lap times of laps 2 to 10 ---------------------------------------------------------------


def with_lap_diffs(diffs: list[float]) -> list[LapRun]:
    """The system's lap k+2 of run a is `diffs[k]` longer than the reference's."""
    a = make_run("a")
    times = list(a.sys_times)
    for k, d in enumerate(diffs):
        times[k + 1] = LAP + d
    return [replace(a, sys_times=tuple(times)), make_run("b")]


@pytest.mark.parametrize(
    ("diffs", "verdict"),
    [
        ([], PASS),
        ([0.3], PASS),  # exactly at the tolerance
        ([-0.3], PASS),
        ([0.3001], PASS),  # one lap outside: 17 of 18 within
        ([0.31, 0.31], FAIL),  # two outside: 16 of 18
        ([0.3, 0.3, 0.3, 0.3], PASS),  # four at the threshold, all within
        ([0.3001, 0.3], PASS),
        ([0.3001, -0.3001], FAIL),
    ],
)
def test_item_ii_at_least_17_of_18_within_0_3_s(diffs: list[float], verdict: str) -> None:
    out = verdict_of(with_lap_diffs(diffs))
    assert items(out)["(ii)"] == verdict
    assert out["items"][1]["numbers"]["of"] == 18 and out["items"][1]["numbers"]["needed"] == 17


def test_item_ii_lap_1_is_not_one_of_the_18() -> None:
    a = make_run("a")
    times = list(a.sys_times)
    times[0] = LAP + 2.0  # lap 1 two seconds long: judged by crossing, not duration
    out = verdict_of([replace(a, sys_times=tuple(times)), make_run("b")])
    assert items(out)["(ii)"] == PASS


# -- (iii) lap 1 by its crossing instant -------------------------------------------------------------------------


def with_lap1_cross(shift: float, run_b: bool = False) -> list[LapRun]:
    a, b = make_run("a"), make_run("b")
    target = b if run_b else a
    cross = list(target.sys_cross)
    cross[0] += shift
    moved = replace(target, sys_cross=tuple(cross))
    return [a, moved] if run_b else [moved, b]


@pytest.mark.parametrize(
    ("shift", "verdict"),
    [(0.0, PASS), (0.3, PASS), (-0.3, PASS), (0.3001, FAIL), (-0.3001, FAIL), (2.0, FAIL)],
)
def test_item_iii_lap_1_crossing_within_0_3_s_of_the_median_offset(
    shift: float, verdict: str
) -> None:
    for run_b in (False, True):  # in both runs
        out = verdict_of(with_lap1_cross(shift, run_b))
        assert items(out)["(iii)"] == verdict


def test_item_iii_a_constant_clock_offset_between_the_two_sources_cancels() -> None:
    out = verdict_of(two(offset=7.3))
    assert items(out)["(iii)"] == PASS
    numbers = out["items"][2]["numbers"]["run_ccw"]
    assert numbers["median_2_to_n"] == pytest.approx(-7.3) and numbers[
        "difference"
    ] == pytest.approx(0)


def test_item_iii_a_lap_1_duration_difference_is_not_judged_by_item_ii_or_iii() -> None:
    """The camera starts lap 1 at its reference frame, the system at its start scan: durations
    differ by seconds, crossings agree."""
    a = make_run("a")
    shifted = replace(a, ref_times=(LAP + 2.6,) + a.ref_times[1:])
    out = verdict_of([shifted, make_run("b")])
    assert out["verdict"] == PASS


def test_item_iii_without_crossing_instants_is_insufficient() -> None:
    runs = [replace(make_run("a"), sys_cross=()), make_run("b")]
    out = verdict_of(runs)
    assert items(out)["(iii)"] == INSUFFICIENT and out["verdict"] == INSUFFICIENT


# -- (iv) missed gate --------------------------------------------------------------------------------------------


def test_item_iv_d7_counts_every_lap_of_the_run_d8_the_judged_laps() -> None:
    runs = two(missed=(11,))  # lap 11 flagged: beyond the 10 judged laps
    assert items(verdict_of(runs, "d7"))["(iv)"] == FAIL
    assert items(verdict_of(runs, "d8"))["(iv)"] == PASS
    runs = two(missed=(10,))
    assert items(verdict_of(runs, "d8"))["(iv)"] == FAIL
    assert items(verdict_of(runs, "d7"))["(iv)"] == FAIL
    assert items(verdict_of(two(missed=()), "d8"))["(iv)"] == PASS


def test_item_iv_a_status_without_the_key_is_insufficient_not_a_pass() -> None:
    out = verdict_of(two(missed=None))
    assert items(out)["(iv)"] == INSUFFICIENT and out["verdict"] == INSUFFICIENT


# -- (v) the cost -------------------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("p99", "verdict"), [(12.0, PASS), (80.0, PASS), (80.001, FAIL), (200.0, FAIL)]
)
def test_item_v_on_scan_p99_at_most_80_ms(p99: float, verdict: str) -> None:
    runs = [make_run("a", p99=p99), make_run("b")]
    assert items(verdict_of(runs))["(v)"] == verdict
    runs = [make_run("a"), make_run("b", p99=p99)]  # in both runs
    assert items(verdict_of(runs))["(v)"] == verdict


def test_item_v_without_the_key_is_insufficient() -> None:
    out = verdict_of([make_run("a", p99=math.nan), make_run("b")])
    assert items(out)["(v)"] == INSUFFICIENT and out["verdict"] == INSUFFICIENT


# -- the reference's validity --------------------------------------------------------------------------------------


def test_a_reference_that_fails_its_validity_rule_gives_repeat_never_pass_or_fail() -> None:
    for runs in (
        # the camera counted one lap too few over the run
        [make_run("a", ref_laps=11), make_run("b")],
        # the wheel distance says 13 laps
        [make_run("a", wheel_extra=1.1 * LAP), make_run("b")],
    ):
        for rule in ("d7", "d8"):
            out = verdict_of(runs, rule)
            assert out["verdict"] == REPEAT
            assert {it["verdict"] for it in out["items"]} == {REPEAT}
            assert out["validity"][0]["valid"] is False


def test_the_validity_rule_rounds_the_wheel_count_down() -> None:
    ok, _ = lap_c0.reference_valid(make_run(n=12, wheel_extra=0.99 * LAP))
    assert ok is True  # 12.99 laps of wheel: 12
    ok, _ = lap_c0.reference_valid(make_run(n=12, wheel_extra=-0.01))
    assert ok is False  # 11.998 laps of wheel: 11


def test_a_reference_without_lap_distances_cannot_be_validated() -> None:
    run = replace(make_run(), ref_distance_m=())
    ok, why = lap_c0.reference_valid(run)
    assert ok is False and "no lap distances" in why
    assert verdict_of([run, make_run("b")])["verdict"] == REPEAT
    no_odom = replace(make_run(), wheel_distance_m=math.nan)
    assert lap_c0.reference_valid(no_odom)[0] is False


def test_no_reference_at_all_is_no_reference() -> None:
    runs = [replace(make_run(r), ref_times=(), ref_cross=(), ref_distance_m=()) for r in "ab"]
    out = verdict_of(runs)
    assert out["verdict"] == NO_REFERENCE


@pytest.mark.parametrize("n", [1, 9])
def test_under_10_laps_is_insufficient(n: int) -> None:
    out = verdict_of([make_run("a", n=n), make_run("b", n=n)])
    assert out["verdict"] == INSUFFICIENT
    assert {it["verdict"] for it in out["items"]} == {INSUFFICIENT}


def test_exactly_10_laps_is_enough_and_one_run_is_insufficient() -> None:
    assert verdict_of([make_run("a", n=10), make_run("b", n=10)])["verdict"] == PASS
    assert verdict_of([make_run("a")])["verdict"] == INSUFFICIENT


def test_the_rule_must_be_d7_or_d8() -> None:
    with pytest.raises(ValueError):
        lap_c0.evaluate(two(), "d9", TH)


# -- the lap table and the sources ------------------------------------------------------------------------------------


def test_the_lap_rows_carry_time_reference_difference_offset_source_and_missed() -> None:
    run = make_run(missed=(2,), sources="anchor")
    rows = lap_c0.lap_rows(run, 3)
    assert [r["lap"] for r in rows] == [1, 2, 3]
    assert rows[1]["missed"] is True and rows[0]["missed"] is False
    assert rows[0]["system_s"] == LAP and rows[0]["diff_s"] == pytest.approx(0.0)
    assert rows[0]["crossing_offset_s"] == pytest.approx(-0.5)
    assert rows[2]["line_source"] == "anchor"
    unknown = lap_c0.lap_rows(replace(run, missed_laps=None), 1)
    assert unknown[0]["missed"] is None
    assert lap_c0.sources_summary([run]) == {"run_ccw": {"anchor": 12}}


# -- the video as reference ---------------------------------------------------------------------------------------------


def test_the_video_csv_is_a_reference_with_crossings_and_distances(tmp_path: Path) -> None:
    csv_path = tmp_path / "laps.csv"
    lines = ["run,lap,lap_time_s,t_cross_s,distance_m"]
    for k in range(12):
        lines.append(f"run_ccw,{k + 1},{LAP},{100.0 + LAP * (k + 1) + 1.0},{LAP}")
    csv_path.write_text("\n".join(lines) + "\n")
    video = lap_c0.read_video_laps(csv_path)
    bag = lap_c0.BagLaps("run_ccw")
    bag.sys_times = [LAP] * 12
    bag.sys_cross = [100.0 + LAP * (k + 1) for k in range(12)]
    bag.wheel_distance_m = 12 * LAP + 0.4
    bag.statuses = [
        {
            "missed_gate_laps": [],
            "lap_line_sources": ["start_scan"] * 12,
            "lap_mode": "observe",
            "on_scan_ms_p99": 9.0,
        }
    ]
    run = lap_c0.run_from_bag(bag, video)
    assert run.ref_times == (LAP,) * 12 and run.ref_distance_m == (LAP,) * 12
    assert lap_c0.reference_valid(run)[0] is True
    assert run.on_scan_p99_ms == 9.0 and run.missed_laps == ()


def test_the_video_csv_without_a_distance_column_cannot_validate(tmp_path: Path) -> None:
    csv_path = tmp_path / "laps.csv"
    csv_path.write_text(
        "run,lap,lap_time_s,t_cross_s\n"
        + "\n".join(f"r,{k + 1},5.2,{5.2 * k}" for k in range(11))
        + "\n"
    )
    video = lap_c0.read_video_laps(csv_path)
    assert video["r"]["distance"] == () and len(video["r"]["times"]) == 11
    bad = tmp_path / "bad.csv"
    bad.write_text("run,lap,lap_time_s\nr,1,5.2\n")
    with pytest.raises(ValueError, match="t_cross_s"):
        lap_c0.read_video_laps(bad)


# -- D3-C0 through the new command -------------------------------------------------------------------------------------


def test_d3_c0_through_the_new_command_equals_laps_d3c0() -> None:
    runs = two()
    report = lap_c0.build_report(runs, "camera", TH, replay=False, rule="d8")
    direct = d3.d3c0([d3.RunLapInput(r.run_id, r.sys_times, r.ref_times) for r in runs], "camera")
    assert report["d3_c0"] == direct
    assert report["d3_c0"]["verdict"] == PASS
    assert report["label"] == "recorded events: the gate"
    off = [replace(runs[0], sys_times=(LAP + 0.4,) + (LAP,) * 11), runs[1]]
    report = lap_c0.build_report(off, "camera", TH, replay=True, rule=None)
    direct = d3.d3c0([d3.RunLapInput(r.run_id, r.sys_times, r.ref_times) for r in off], "camera")
    assert report["d3_c0"] == direct and report["label"] == "REGRESSION, NOT A GATE"


def test_the_markdown_has_the_three_blocks_the_tables_and_the_regression_label() -> None:
    report = lap_c0.build_report(two(), "camera", TH, replay=True, rule="d8")
    text = lap_c0.markdown(report)
    assert text.startswith("REGRESSION, NOT A GATE")
    for needle in (
        "# D3-C0",
        "# D7-C0 rule",
        "# D8-C0",
        "| lap | system s |",
        "line sources per run",
    ):
        assert needle in text
    gate = lap_c0.markdown(lap_c0.build_report(two(), "camera", TH, replay=False, rule="d8"))
    assert "REGRESSION" not in gate


# -- a bag, written with mcap-ros2-support, and the command line ---------------------------------------------------------

STRING_DEF = "string data\n"
LAP_DEF = """std_msgs/Header header
uint32 lap_index
float32 lap_time_s
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""
ODOM_DEF = """std_msgs/Header header
geometry_msgs/TwistWithCovariance twist
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/TwistWithCovariance
geometry_msgs/Twist twist
================================================================================
MSG: geometry_msgs/Twist
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
"""


def _time(t: float) -> dict[str, int]:
    return {"sec": int(t), "nanosec": round((t - int(t)) * 1e9)}


def write_bag(
    path: Path,
    n: int = 12,
    camera_laps: int | None = None,
    p99: float | None = 9.0,
    missed: list[int] | None = None,
    speed: float = 1.0,
) -> None:
    from mcap_ros2.writer import Writer

    n_cam = n if camera_laps is None else camera_laps
    with path.open("wb") as stream:
        w = Writer(stream, chunk_size=64 * 1024)
        s_string = w.register_msgdef("std_msgs/msg/String", STRING_DEF)
        s_lap = w.register_msgdef("racer_pnp_msgs/msg/LapEvent", LAP_DEF)
        s_odom = w.register_msgdef("nav_msgs/msg/Odometry", ODOM_DEF)
        t0 = 1000.0
        duration = n * LAP + 0.4
        k = 0
        while k * 0.02 <= duration:  # wheel odometry at 50 Hz, LAP s per LAP m
            t = t0 + k * 0.02
            w.write_message(
                "/odom/wheel",
                s_odom,
                {
                    "header": {"stamp": _time(t), "frame_id": "odom"},
                    "twist": {
                        "twist": {
                            "linear": {"x": speed, "y": 0.0, "z": 0.0},
                            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                        }
                    },
                },
                log_time=round(t * 1e9),
            )
            k += 1
        for i in range(n):
            t = t0 + LAP * (i + 1)
            w.write_message(
                "/pnp/lap/event",
                s_lap,
                {
                    "header": {"stamp": _time(t), "frame_id": "start_scan"},
                    "lap_index": i + 1,
                    "lap_time_s": LAP,
                },
                log_time=round((t + 0.3) * 1e9),
            )
        for i in range(n_cam):
            t = t0 + LAP * (i + 1) + 0.5
            payload = {
                "lap": i + 1,
                "lap_time_s": LAP,
                "crossing_time_s": t,
                "distance_m": LAP * speed,
            }
            w.write_message(
                "/lap/event", s_string, {"data": json.dumps(payload)}, log_time=round(t * 1e9)
            )
        if p99 is not None:
            status = {
                "lap_mode": "observe",
                "missed_gate_laps": missed or [],
                "lap_line_sources": ["start_scan"] * n,
                "on_scan_ms_p99": p99,
            }
            for t in (t0 + 5.0, t0 + 30.0):
                w.write_message(
                    "/pnp/lap/status",
                    s_string,
                    {"data": json.dumps(status)},
                    log_time=round(t * 1e9),
                )
            w.write_message(
                "/pnp/lap/status",
                s_string,
                {"data": json.dumps({"lap_mode": "observe", "on_scan_ms_p99": p99 / 2})},
                log_time=round((t0 + 1.0) * 1e9),
            )
        w.finish()


def test_read_bag_collects_the_laps_the_status_and_the_wheel_distance(tmp_path: Path) -> None:
    bag = tmp_path / "run_ccw.mcap"
    write_bag(bag, missed=[3])
    got = lap_c0.read_bag(bag)
    assert got.name == "run_ccw.mcap" and len(got.sys_times) == 12 and len(got.ref_times) == 12
    assert got.wheel_distance_m == pytest.approx(12 * LAP + 0.4, abs=0.05)
    run = lap_c0.run_from_bag(got)
    assert run.missed_laps == (3,) and run.on_scan_p99_ms == 9.0 and run.lap_mode == "observe"
    assert run.sys_cross[0] == pytest.approx(1000.0 + LAP) and run.ref_cross[0] == pytest.approx(
        1000.0 + LAP + 0.5
    )
    assert run.line_sources == ("start_scan",) * 12
    assert lap_c0.reference_valid(run)[0] is True


def test_a_bag_without_a_status_has_no_missed_flags_and_no_cost(tmp_path: Path) -> None:
    bag = tmp_path / "old.mcap"
    write_bag(bag, p99=None)
    run = lap_c0.run_from_bag(lap_c0.read_bag(bag))
    assert run.missed_laps is None and math.isnan(run.on_scan_p99_ms)


def run_cli(argv: list[str]) -> tuple[int, str]:
    buf = io.StringIO()
    with redirect_stdout(buf):
        code = cli.main(argv)
    return code, buf.getvalue()


def test_the_command_prints_three_blocks_writes_the_json_and_passes_a_clean_session(
    tmp_path: Path,
) -> None:
    a, b = tmp_path / "ccw.mcap", tmp_path / "cw.mcap"
    write_bag(a)
    write_bag(b)
    out = tmp_path / "report.json"
    code, text = run_cli(["lap-c0", "--bag", str(a), "--bag", str(b), "--out", str(out)])
    assert code == 0
    assert "# D3-C0" in text and "# D7-C0 rule" in text and "# D8-C0" in text
    doc = json.loads(out.read_text())
    assert doc["d8_c0"]["verdict"] == PASS and doc["d7_c0"]["verdict"] == PASS
    assert doc["d3_c0"]["verdict"] == PASS and doc["replay"] is False
    assert doc["d7_yaml_sha256"] == TH.d7_sha256 and doc["d8_yaml_sha256"] == TH.d8_sha256


def test_the_command_exits_nonzero_on_a_fail_and_on_a_repeat(tmp_path: Path) -> None:
    a, b = tmp_path / "ccw.mcap", tmp_path / "cw.mcap"
    write_bag(a)
    write_bag(b, p99=95.0)
    code, text = run_cli(["lap-c0", "--bag", str(a), "--bag", str(b)])
    assert code == 1 and "(v0.5 laps ring section 5): FAIL" in text
    code7, _ = run_cli(["lap-c0", "--bag", str(a), "--bag", str(b), "--rule", "d7"])
    assert code7 == 0  # the D7 rule has no cost item
    c = tmp_path / "short_camera.mcap"
    write_bag(c, camera_laps=11)
    code, text = run_cli(["lap-c0", "--bag", str(a), "--bag", str(c)])
    assert code == 1 and "REPEAT" in text


def test_the_replay_flag_recomputes_the_events_and_labels_the_report(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    a, b = tmp_path / "ccw.mcap", tmp_path / "cw.mcap"
    write_bag(a)
    write_bag(b)

    def fake(_path: Path) -> SimpleNamespace:
        laps = [
            SimpleNamespace(
                lap_time_s=LAP,
                t_cross_s=1000.0 + LAP * (k + 1),
                missed_gate=False,
                lap_index=k + 1,
                line_source="winding",
            )
            for k in range(12)
        ]
        return SimpleNamespace(laps=laps, scan_times_s=[0.004] * 200)

    monkeypatch.setattr(lap_c0, "replay_bag", fake)
    out = tmp_path / "r.json"
    code, text = run_cli(
        ["lap-c0", "--bag", str(a), "--bag", str(b), "--replay", "--out", str(out)]
    )
    assert code == 0 and text.startswith("REGRESSION, NOT A GATE")
    doc = json.loads(out.read_text())
    assert doc["replay"] is True and doc["label"] == "REGRESSION, NOT A GATE"
    assert doc["line_sources"]["ccw.mcap"] == {"winding": 12}
    assert doc["d8_c0"]["items"][-1]["numbers"]["ccw.mcap"] == pytest.approx(4.0)


def test_the_command_takes_the_video_csv(tmp_path: Path) -> None:
    a, b = tmp_path / "ccw.mcap", tmp_path / "cw.mcap"
    write_bag(a, camera_laps=0)
    write_bag(b, camera_laps=0)
    lines = ["run,lap,lap_time_s,t_cross_s,distance_m"]
    for name in (a.name, b.name):
        for k in range(12):
            lines.append(f"{name},{k + 1},{LAP},{1000.0 + LAP * (k + 1) + 2.0},{LAP}")
    csv_path = tmp_path / "laps.csv"
    csv_path.write_text("\n".join(lines) + "\n")
    out = tmp_path / "r.json"
    code, _ = run_cli(
        ["lap-c0", "--bag", str(a), "--bag", str(b), "--video", str(csv_path), "--out", str(out)]
    )
    doc = json.loads(out.read_text())
    assert code == 0 and doc["reference"] == "video" and doc["d8_c0"]["verdict"] == PASS
    # no reference lap in any run: NO REFERENCE
    code, text = run_cli(["lap-c0", "--bag", str(a), "--bag", str(b)])
    assert code == 1 and "NO REFERENCE" in text


def test_python_dash_m_pnp_session_is_the_same_command() -> None:
    import runpy

    with pytest.raises(SystemExit) as exc:
        runpy.run_module("pnp_session", run_name="__main__", alter_sys=True)
    assert exc.value.code == 2  # no subcommand: argparse's usage error
