"""L1 tests of tools/track_bench.py: the run plan, the runner command line and the report table,
without a simulator."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest

_PATH = Path(__file__).resolve().parents[1] / "track_bench.py"
_spec = importlib.util.spec_from_file_location("track_bench", _PATH)
bench = importlib.util.module_from_spec(_spec)
sys.modules["track_bench"] = bench
_spec.loader.exec_module(bench)


def test_plan_is_every_track_in_both_directions_for_every_variant():
    runs = bench.plan("family", bench.DEFAULT_VARIANTS, ["ccw", "cw"])
    assert len(runs) == len(bench.TRACK_SETS["family"]) * 2 * len(bench.DEFAULT_VARIANTS)
    assert len({r["id"] for r in runs}) == len(runs)
    assert {r["direction"] for r in runs} == {"ccw", "cw"}


def test_family_covers_the_requested_ranges():
    params = [p for _, p in bench.TRACK_SETS["family"].values()]
    radii = {p.get("island_radius_m") for p in params}
    assert 0.0 in radii and 0.8 in radii
    assert {p["lane_width_m"] for p in params} >= {1.0, 1.2}
    straights = {p["straight_length_m"] for _, p in bench.TRACK_SETS["family"].values()}
    assert min(straights) <= 1.0 and max(straights) >= 4.0
    kinds = {t for t, _ in bench.TRACK_SETS["family"].values()}
    assert kinds == {"stadium", "rounded_rect"}


def test_command_carries_the_variant_and_the_direction(tmp_path):
    run = bench.plan(
        "family",
        {"x": {"driver": "tracker", "raceline": {"v_max_mps": 0.5}, "car": "params", "laps": 2}},
        ["cw"],
    )[0]
    command = bench.command(run, 107, tmp_path, ["--noise"])
    assert "--reverse" in command
    assert command[command.index("--driver") + 1] == "tracker"
    assert json.loads(command[command.index("--raceline-json") + 1]) == {"v_max_mps": 0.5}
    assert command[command.index("--car") + 1] == "params"
    assert command[command.index("--stop-after-laps") + 1] == "2"
    assert command[command.index("--domain-id") + 1] == "107"
    assert command[-1] == "--noise"
    ccw = bench.command(
        bench.plan("family", {"x": {"driver": "gap_follow"}}, ["ccw"])[0], 1, tmp_path, []
    )
    assert "--reverse" not in ccw


def test_an_unknown_variant_key_is_refused(tmp_path):
    run = bench.plan("family", {"x": {"driver": "tracker", "nonsense": 1}}, ["ccw"])[0]
    with pytest.raises(KeyError):
        bench.command(run, 1, tmp_path, [])


def test_cell_and_report(tmp_path):
    good = {
        "laps": 3,
        "lap_time_s": 15.2,
        "obstacle_brakes": 0,
        "escapes": 0,
        "wall_contacts": 0,
        "min_body_clearance_m": 0.115,
    }
    assert bench.cell(good) == "3 laps 15.2s, 0b 0e 0c, clr 0.115"
    assert bench.cell({"error": "x"}) == "ERR"
    (tmp_path / "st_a__ccw__tracker.json").write_text(json.dumps(good))
    (tmp_path / "st_a__ccw__gap.json").write_text(
        json.dumps({**good, "laps": 0, "lap_time_s": None})
    )
    table = bench.report(tmp_path)
    lines = table.splitlines()
    assert lines[0].startswith("| Track | Dir |")
    assert "tracker" in lines[0] and "gap" in lines[0]
    assert any(line.startswith("| st_a | ccw |") for line in lines)
    assert "0 laps," in table
