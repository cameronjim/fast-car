"""tools/pnp_session/lap_src.py, D10-C0 (brief 51c): item (vi), the start order read from the recorded
topics, the lapref period and autocorrelation peak reported with no threshold, the thresholds read
through the new yaml's hash, and the command line flag. D9-C0's own items are tested in
test_pnp_session_lap_src.py and are untouched."""

from __future__ import annotations

import dataclasses
import hashlib
import io
import json
import math
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from typing import Any

import numpy as np
import pytest

pytest.importorskip("mcap")
pytest.importorskip("racer_eval")
pytest.importorskip("mcap_ros2")

from pnp_session import cli, lap_src
from pnp_session.lap_src import FAIL, INSUFFICIENT, PASS
from test_pnp_session_lap_src import Scenario, write_bag

REPO_ROOT = Path(__file__).resolve().parents[2]
PREFIX = "/pnp/lap_shadow"
EPS = 1e-6


def ready_body(**changes: Any) -> dict[str, Any]:
    body: dict[str, Any] = {
        "lap_source": "camera",
        "phase": "waiting_for_driving",
        "source_ready_now": True,
        "source_alive": True,
        "source_not_ready_reason": "",
    }
    body.update(changes)
    return body


def bag_with(statuses: list[tuple[float, dict[str, Any]]], planner_first_s: float | None):
    """A clean synthetic bag whose camera-source lap node says `statuses` and whose first planner
    status arrived at `planner_first_s` (None: no planner status at all)."""
    bag = Scenario().bag()
    bag.nodes[PREFIX].statuses = list(statuses)
    s = bag.streams
    if planner_first_s is None:
        planner = {"planner_arrival": np.array([], dtype=float)}
        planner |= {"planner_state": np.array([], dtype=np.int64)}
        planner |= {"planner_clearance": np.array([], dtype=float)}
    else:
        planner = {
            "planner_arrival": np.array([planner_first_s, planner_first_s + 0.1]),
            "planner_state": np.array([0, 1], dtype=np.int64),
            "planner_clearance": np.array([0.3, 0.3]),
        }
    bag.streams = dataclasses.replace(s, **planner)
    return bag


# -- the constants --------------------------------------------------------------------------------


def test_the_d10_yaml_hash_constant_is_the_files() -> None:
    from pnp_session.constants import D10_YAML, D10_YAML_SHA256

    path = REPO_ROOT / D10_YAML
    assert hashlib.sha256(path.read_bytes()).hexdigest() == D10_YAML_SHA256


def test_the_thresholds_come_through_the_d10_yaml_and_the_d9_yaml_they_name(tmp_path: Path) -> None:
    th = lap_src.load_d10()
    assert th["d9_c0"]["peak_similarity_min"] == 0.88  # the D9-C0 thresholds, unchanged
    bad = tmp_path / "x.yaml"
    bad.write_text("schema: nope\n")
    with pytest.raises(lap_src.PreregError):
        lap_src.load_d10(bad)
    # the D10 yaml names the D9 yaml's hash: a D9 yaml that is not that one is refused
    other_d9 = tmp_path / "d9.yaml"
    other_d9.write_text("schema: nope\n")
    with pytest.raises(lap_src.PreregError):
        lap_src.load_d10(None, other_d9)


# -- the second of two consecutive ready statuses ---------------------------------------------------


@pytest.mark.parametrize(
    ("name", "times", "bodies", "expected"),
    [
        ("two ready", [1.0, 2.0], [{}, {}], 2.0),
        ("one ready", [1.0], [{}], None),
        ("exactly at the gap limit", [1.0, 3.5], [{}, {}], 3.5),
        ("epsilon over the gap limit", [1.0, 3.5 + EPS], [{}, {}], None),
        ("not ready between", [1.0, 2.0, 3.0], [{}, {"source_ready_now": False}, {}], None),
        ("then ready twice", [1.0, 2.0, 3.0, 4.0], [{}, {"source_alive": False}, {}, {}], 4.0),
        ("running does not say ready", [1.0, 2.0], [{}, {"phase": "running"}], None),
        ("start_scan source", [1.0, 2.0], [{"lap_source": "start_scan"}, {}], None),
        ("a missing key", [1.0, 2.0], [{}, {"source_ready_now": None}], None),
    ],
)
def test_the_second_of_two_consecutive_ready_statuses(
    name: str, times: list[float], bodies: list[dict[str, Any]], expected: float | None
) -> None:
    statuses = [(t, ready_body(**b)) for t, b in zip(times, bodies, strict=True)]
    assert lap_src.second_ready_status_s(statuses) == expected, name


def test_statuses_out_of_order_are_read_in_time_order() -> None:
    statuses = [(2.0, ready_body()), (1.0, ready_body())]
    assert lap_src.second_ready_status_s(statuses) == 2.0


# -- item (vi) -------------------------------------------------------------------------------------------


def vi(bag: lap_src.BagData) -> dict[str, Any]:
    return lap_src.evaluate_start_order(bag, bag.nodes[PREFIX])


def test_vi_passes_when_the_planner_is_later_than_the_second_ready_status() -> None:
    item = vi(bag_with([(1.0, ready_body()), (2.0, ready_body())], 2.5))
    assert item["verdict"] == PASS, item
    assert item["numbers"]["second_ready_status_s"] == 2.0
    assert item["numbers"]["planner_first_status_s"] == 2.5


def test_vi_fails_when_the_planner_came_first() -> None:
    item = vi(bag_with([(1.0, ready_body()), (4.0, ready_body())], 2.5))
    assert item["verdict"] == FAIL


def test_vi_wants_the_planner_strictly_later() -> None:
    item = vi(bag_with([(1.0, ready_body()), (2.0, ready_body())], 2.0))
    assert item["verdict"] == FAIL
    item = vi(bag_with([(1.0, ready_body()), (2.0, ready_body())], 2.0 + EPS))
    assert item["verdict"] == PASS


def test_vi_fails_when_the_planner_ran_and_the_lap_node_never_said_ready_twice() -> None:
    item = vi(bag_with([(1.0, ready_body())], 2.5))
    assert item["verdict"] == FAIL
    assert item["numbers"]["second_ready_status_s"] is None


def test_vi_is_insufficient_without_a_planner_status_in_the_bag() -> None:
    item = vi(bag_with([(1.0, ready_body()), (2.0, ready_body())], None))
    assert item["verdict"] == INSUFFICIENT


def test_vi_is_insufficient_when_nothing_was_ready_and_no_planner_ran() -> None:
    # the launch refused: the planner never started, which is what the gate is for, and there is
    # no run in the bag to put in order
    item = vi(bag_with([(1.0, ready_body(source_ready_now=False))], None))
    assert item["verdict"] == INSUFFICIENT


# -- the lapref line ----------------------------------------------------------------------------------------


def test_the_lapref_line_reports_the_period_and_the_peak_and_no_verdict() -> None:
    bag = Scenario(n=12).bag()
    line = lap_src.lapref_line(bag)
    assert line["runs"], line
    run = line["runs"][0]
    assert run["laps"] > 0 and run["period_m"] == pytest.approx(5.2, abs=0.3)
    assert 0.0 < run["autocorr_peak"] <= 1.0
    assert "verdict" not in line  # reported, no threshold


def test_the_lapref_line_for_a_run_with_no_period_says_why() -> None:
    bag = Scenario(n=12, steering_steps=False).bag()
    line = lap_src.lapref_line(bag)
    run = line["runs"][0]
    assert run["laps"] == 0 and run["period_m"] is None and run["reason"]


# -- evaluate_c0_d10 and run -----------------------------------------------------------------------------


def d10_bag(planner_first_s: float) -> lap_src.BagData:
    """A clean 12 lap bag: the lap node said ready at 0.5 s and 1.5 s, then its own two final
    statuses; the first planner status arrived at `planner_first_s`."""
    bag = Scenario().bag()
    final = bag.nodes[PREFIX].statuses
    return bag_with([(0.5, ready_body()), (1.5, ready_body()), *final], planner_first_s)


def test_d10_c0_is_d9_c0_plus_item_vi_and_the_lapref_line() -> None:
    th = lap_src.load_d10()
    block = lap_src.evaluate_c0_d10(d10_bag(2.0), 12, th)
    names = [i["item"][:4] for i in block["items"]]
    assert names == ["(i) ", "(ii)", "(iii", "(iv)", "(v) ", "(vi)"]
    d9 = lap_src.evaluate_c0(Scenario().bag(), 12, th)
    assert [i["verdict"] for i in block["items"][:5]] == [i["verdict"] for i in d9["items"]]
    assert block["items"][5]["verdict"] == PASS
    assert block["verdict"] == PASS, block["items"]
    assert "lapref" in block and block["lapref"]["runs"]


def test_a_failed_item_vi_fails_the_bag() -> None:
    th = lap_src.load_d10()
    block = lap_src.evaluate_c0_d10(d10_bag(1.0), 12, th)  # the planner before the second status
    assert block["items"][5]["verdict"] == FAIL and block["verdict"] == FAIL


def test_under_ten_laps_is_still_insufficient_and_still_judges_item_vi() -> None:
    th = lap_src.load_d10()
    bag = Scenario(n=4).bag()
    bag.streams = dataclasses.replace(
        bag.streams,
        planner_arrival=np.array([2.0, 2.1]),
        planner_state=np.array([0, 1], dtype=np.int64),
        planner_clearance=np.array([0.3, 0.3]),
    )
    bag.nodes[PREFIX].statuses = [(0.5, ready_body()), (1.5, ready_body())]
    block = lap_src.evaluate_c0_d10(bag, 4, th)
    assert [i["verdict"] for i in block["items"][:5]] == [INSUFFICIENT] * 5
    assert block["items"][5]["item"].startswith("(vi)") and block["items"][5]["verdict"] == PASS
    assert block["verdict"] == INSUFFICIENT


def test_the_run_function_reads_d10_through_its_own_hash_and_has_no_offline_gate(
    tmp_path: Path,
) -> None:
    bag = tmp_path / "bag.mcap"
    write_bag(bag)
    report, text, code = lap_src.run([bag], laps_counted=[12], d10=True)
    assert report["prereg_yaml_sha256"] == lap_src.D10_YAML_SHA256
    assert "d10_c0" in report and "D10-C0 bag.mcap" in text
    assert "(vi)" in text and "lapref" in text
    assert code in (0, 1)
    with pytest.raises(lap_src.PreregError, match="no D10-B0"):
        lap_src.run([bag], d10=True, offline=True)


def run_cli(*argv: str) -> tuple[int, str, str]:
    out, err = io.StringIO(), io.StringIO()
    with redirect_stdout(out), redirect_stderr(err):
        code = cli.main(["lap-src", *argv])
    return code, out.getvalue(), err.getvalue()


def test_the_command_line_flag_d10(tmp_path: Path) -> None:
    bag = tmp_path / "bag.mcap"
    write_bag(bag)
    out = tmp_path / "report.json"
    _code, text, _err = run_cli(
        "--bag", str(bag), "--laps-counted", "12", "--d10", "--out", str(out)
    )
    assert "D10-C0 bag.mcap" in text
    report = json.loads(out.read_text())
    assert report["d10_c0"][0]["items"][5]["item"].startswith("(vi)")
    assert math.isfinite(report["d10_c0"][0]["camera_laps"])
    # without the flag the report is D9-C0 exactly as before
    _code, text, _err = run_cli("--bag", str(bag), "--laps-counted", "12")
    assert "D9-C0 bag.mcap" in text and "(vi)" not in text
