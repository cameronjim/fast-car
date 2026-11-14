"""The session pipeline and the command line, on a synthetic bag and on the R3 fixture."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import pytest
import yaml
from racer_eval import cli
from racer_eval.recording import BagData
from racer_eval.session import SessionError, analyse_session, load_session, locate_run

FIXTURE = Path(__file__).parent / "fixtures" / "r3_22-20-18.npz"


def make_bag(
    name: str = "bagX", n_laps: int = 10, first_lap_s: float = 10.0, camera: bool = True
) -> BagData:
    """One PnP run from 0 to 130 s at 50 Hz, a camera lap every 10 s getting 0.2 s faster."""
    t = np.arange(0.0, 130.0, 0.02)
    bag = BagData(name=name)
    bag.st_t, bag.st_state = t, np.full(len(t), 1)
    bag.draw_t, bag.draw_speed = t, np.where(t < 1.0, 0.0, 0.5)
    bag.odom_t, bag.odom_v = t, np.full(len(t), 0.5)
    bag.ev_t = np.array([0.5, 1.5, 60.0, 60.1, 140.0])
    bag.ev_severity = np.array([0, 0, 0, 1, 0])
    bag.ev_phase = np.array([0, 0, 0, 1, 0])
    bag.ev_source = np.array(["watchdog", "rate_limit", "ttc", "ttc", "watchdog"])
    times = np.array([first_lap_s - 0.2 * k for k in range(n_laps)])
    ends = 1.0 + np.cumsum(times)
    if camera:
        bag.cam_t, bag.cam_lap_time_s = ends, times
        bag.cam_lap = np.arange(1, n_laps + 1)
        bag.cam_distance_m = times * 0.5
    bag.pnp_t, bag.pnp_lap_time_s = ends + 0.05, times + 0.1
    bag.pnp_lap_index = np.arange(1, n_laps + 1)
    return bag


def write_session(tmp_path: Path, extra: dict[str, object] | None = None) -> Path:
    doc: dict[str, object] = {
        "n_laps": 10,
        "gp4": {"on": "d_on", "off": "d_off"},
        "acceptance": {"conditions": ["d_on"], "wall_contacts": 0},
        "runs": [
            {"id": "on1", "bag": "bagA", "at_s": 50.0, "condition": "d_on", "cap_mps": 0.5},
            {"id": "on2", "bag": "bagB", "at_s": 50.0, "condition": "d_on", "cap_mps": 1.0},
            {"id": "off1", "bag": "bagC", "at_s": 50.0, "condition": "d_off", "cap_mps": 0.5,
             "contacts": 1},
            {"id": "off2", "bag": "bagD", "at_s": 50.0, "condition": "d_off", "cap_mps": 0.5,
             "dnf_cause": "stall"},
        ],
    }  # fmt: skip
    doc.update(extra or {})
    path = tmp_path / "session.yaml"
    path.write_text(yaml.safe_dump(doc))
    return path


def bags() -> dict[str, BagData]:
    return {k: make_bag(k, camera=k != "bagD") for k in ("bagA", "bagB", "bagC", "bagD")}


def test_locate_run_prefers_status_runs_and_falls_back_to_drive_raw() -> None:
    bag = make_bag()
    assert locate_run(bag, 50.0).kind == "pnp"
    bag.st_t = np.zeros(0)  # a bag without status: the non-zero /drive_raw stretch
    assert locate_run(bag, 50.0).kind == "drive_raw"
    with pytest.raises(SessionError, match="no run contains"):
        locate_run(bag, 500.0)


def test_analyse_session_end_to_end(tmp_path: Path) -> None:
    result = analyse_session(load_session(write_session(tmp_path)), bags())
    on = result.curves["d_on"]
    assert on.n_runs == 2 and on.n_completed == 2
    assert on.points[0].mean == pytest.approx(10.0)
    assert on.points[9].mean == pytest.approx(10.0 - 1.8)
    # bagD has no camera events: its laps come from D's own times and the run is labelled
    assert result.lap_sources == {"on1": "camera", "on2": "camera", "off1": "camera",
                                  "off2": "pnp"}  # fmt: skip
    d_off = next(r for r in result.run_laps if r.run_id == "off2")
    assert d_off.dnf_after == 10 and d_off.dnf_cause == "stall"
    rows = [row for sr, row in result.laps if sr.run_id == "on1"]
    assert len(rows) == 10 and all(r.flag_differs is False for r in rows)  # 0.1 s apart
    counts = {sr.run_id: iv.counts for sr, iv in result.interventions}
    # window is [1.0, 129.98]: the events at 0.5 and 140.0 are outside, the RELEASE is not counted
    assert counts["on1"]["rate_limit"] == 1 and counts["on1"]["watchdog"] == 0
    assert counts["on1"]["ttc"] == 1
    assert result.decisions["gp4"].preregistered
    assert "planner_car_acceptance" in result.decisions


def test_session_cli_writes_all_outputs(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    data = bags()
    out = tmp_path / "out"
    args = argparse.Namespace(
        bags=[str(tmp_path / k) for k in data],
        conditions=str(write_session(tmp_path)),
        out=str(out),
    )
    assert cli.cmd_session(args, read=lambda p: data[p.name]) == 0
    assert "Session summary" in capsys.readouterr().out
    for name in ("laps.csv", "interventions.csv", "curves.json", "decisions.json", "summary.md"):
        assert (out / name).stat().st_size > 0
    with (out / "laps.csv").open() as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 40 and {r["condition"] for r in rows} == {"d_on", "d_off"}
    with (out / "interventions.csv").open() as f:
        iv = {r["run_id"]: r for r in csv.DictReader(f)}
    assert iv["on1"]["rate_limit"] == "1" and iv["on1"]["ttc"] == "1" and iv["on1"]["other"] == "0"
    curves = json.loads((out / "curves.json").read_text())
    assert curves["d_on"]["n_completed"] == 2
    decisions = json.loads((out / "decisions.json").read_text())
    assert decisions["gp4"]["name"] == "gp4" and len(decisions["gp4"]["items"]) == 7
    assert "FAIL" in (out / "summary.md").read_text()  # two D-on runs cannot pass GP4


def test_summary_lists_flagged_laps() -> None:
    bag = make_bag()
    bag.pnp_lap_time_s = bag.pnp_lap_time_s + 0.5  # every lap differs by 0.6 s
    doc = {"runs": [{"id": "a", "bag": "bagA", "at_s": 50.0, "condition": "c", "cap_mps": 0.5}]}
    result = analyse_session(doc, {"bagA": bag})
    assert "('a', 1)" in cli.summary(result)


def test_session_errors(tmp_path: Path) -> None:
    good = {"id": "a", "bag": "bagA", "at_s": 50.0, "condition": "d_on", "cap_mps": 0.5}
    p = tmp_path / "s.yaml"
    for body in ("[]", "runs: []", "runs: 3"):
        p.write_text(body)
        with pytest.raises(SessionError):
            load_session(p)
    with pytest.raises(SessionError, match="missing"):
        analyse_session({"runs": [{"id": "a"}]}, {})
    with pytest.raises(SessionError, match="used twice"):
        analyse_session({"runs": [good, good]}, {"bagA": make_bag()})
    with pytest.raises(SessionError, match="was not given"):
        analyse_session({"runs": [good]}, {})
    doc = {"runs": [good], "gp4": {"on": "d_on", "off": "nope"}}
    with pytest.raises(SessionError, match=r"gp4\.off"):
        analyse_session(doc, {"bagA": make_bag()})
    doc = {"runs": [good], "acceptance": {"conditions": ["nope"]}}
    with pytest.raises(SessionError, match="acceptance"):
        analyse_session(doc, {"bagA": make_bag()})


def test_baseline_on_the_r3_fixture(capsys: pytest.CaptureFixture[str]) -> None:
    data = BagData.load(FIXTURE)
    rows, tot = cli.baseline_table(data)
    assert len(rows) == 1 and tot.count("watchdog") == 3 and tot.count("rate_limit") == 2
    assert cli.cmd_baseline(argparse.Namespace(bag="x"), read=lambda _p: data) == 0
    out = capsys.readouterr().out
    assert "R0" in out and "total" in out and "pre-registration hand count" in out
    assert "per minute" in out


def test_baseline_format_without_any_driving() -> None:
    empty = BagData()
    rows, tot = cli.baseline_table(empty)
    text = cli.format_baseline(rows, tot)
    assert "total" in text and "per minute" not in text


def test_parser_has_both_commands_and_no_threshold_options() -> None:
    parser = cli.build_parser()
    ns = parser.parse_args(["baseline", "--bag", "x"])
    assert ns.command == "baseline"
    ns = parser.parse_args(["session", "--bags", "a", "b", "--conditions", "c", "--out", "o"])
    assert ns.bags == ["a", "b"]
    # no threshold can be passed on the command line
    with pytest.raises(SystemExit):
        parser.parse_args(["session", "--bags", "a", "--conditions", "c", "--out", "o",
                           "--max-median-r", "0.9"])  # fmt: skip
    with pytest.raises(SystemExit):
        parser.parse_args(["nonsense"])


def test_main_runs_baseline(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(cli, "_read", lambda _p: BagData.load(FIXTURE))
    assert cli.main(["baseline", "--bag", "ignored"]) == 0
    assert "total" in capsys.readouterr().out


def test_bag_data_round_trip(tmp_path: Path) -> None:
    bag = make_bag()
    bag.t0_log_s = 12.5
    path = tmp_path / "b.npz"
    bag.save(path)
    back = BagData.load(path)
    assert back.name == bag.name and back.t0_log_s == 12.5
    assert np.array_equal(back.ev_source, bag.ev_source)
    assert np.array_equal(back.cam_lap, bag.cam_lap)
