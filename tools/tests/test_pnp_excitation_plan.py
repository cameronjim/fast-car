"""L1 tests of tools/pnp_session/excitation_plan.py (brief 41b, M-SID2)."""

from __future__ import annotations

import hashlib
from pathlib import Path

import pytest
import yaml
from pnp_session import excitation_plan as ep

REPO = Path(__file__).resolve().parents[2]
YAML = REPO / ep.PLAN_YAML
SHA256 = "2767f297d951c11683b40834b864b832e8cbd0b01343e2765681e11b22e86a82"
GAP = "ros2 launch racer_control gap_follow.launch.py "


def plan():
    return ep.build_plan(ep.load_block())


def test_the_plan_equals_the_yaml_and_is_552_seconds():
    assert hashlib.sha256(YAML.read_bytes()).hexdigest() == SHA256
    block = ep.load_block()
    runs = plan()
    assert ep.total_s(runs) == 552.0
    assert block["planned_total_driving_s"] == 552 <= block["max_total_driving_s"] == 600
    got = [(r.block, r.direction, r.seconds) for r in runs]
    assert got == [
        ("P", "cw", 96),
        ("G", "cw", 30),
        ("G", "cw", 45),
        ("L", "cw", 40),
        ("F", "cw", 20),
        ("F", "cw", 60),
        ("P", "ccw", 96),
        ("G", "ccw", 45),
        ("L", "ccw", 40),
        ("F", "ccw", 20),
        ("F", "ccw", 60),
    ]
    assert [r.order for r in runs] == list(range(1, 12))


def test_the_launch_lines_are_the_ones_of_the_brief():
    lines = {r.order: r.launch_line()[-1] for r in plan() if r.block != "P"}
    race = GAP + "profile:=race-2026-11-12 "
    assert lines[2] == race + "max_speed_mps:=1.0 a_lat_max_mps2:=2.0"
    assert lines[3] == race + "max_speed_mps:=1.3 a_lat_max_mps2:=2.0"
    assert lines[4] == (
        GAP
        + "profile:=lane-2026-10-25 target_mode:=gap_deepest max_speed_mps:=0.6 "
        + "reverse_escape:=false"
    )
    assert lines[5] == race + "max_speed_mps:=1.4 a_lat_max_mps2:=2.4"
    assert lines[6] == race + "max_speed_mps:=1.5 a_lat_max_mps2:=2.8"
    assert lines[8] == lines[3] and lines[9] == lines[4]
    assert lines[10] == lines[5] and lines[11] == lines[6]


def test_block_p_uses_the_unchanged_cap_schedule_tool():
    p = plan()[0].launch_line("dyn-x", "/o")
    assert "--write-params /o/excitation_params.yaml" in p[0]
    assert "v_max_mps:=1.3" in p[1] and "params_file:=/o/excitation_params.yaml" in p[1]
    assert p[2].endswith("--v-max-mps 1.3") and "lap_memory" not in " ".join(p)


def test_every_gap_follow_argument_is_declared_by_the_launch_file():
    declared = ep.declared_gap_follow_arguments()
    wanted = {"profile", "max_speed_mps", "a_lat_max_mps2", "target_mode", "reverse_escape"}
    assert wanted <= declared
    for r in plan():
        for a in r.args:
            assert a.split(":=")[0] in declared, a


def test_a_plan_above_the_budget_is_refused(tmp_path, capsys):
    doc = yaml.safe_load(YAML.read_text())
    doc["excitation"]["max_total_driving_s"] = 551
    bad = tmp_path / "bad.yaml"
    bad.write_text(yaml.safe_dump(doc))
    block = ep.load_block(bad)
    with pytest.raises(ep.PlanRefused, match="552 s"):
        ep.check_budget(ep.build_plan(block), block)
    assert ep.main(["--yaml", str(bad)]) == ep.EXIT_REFUSED
    assert "REFUSED" in capsys.readouterr().err


def test_main_prints_the_table_and_the_rules(capsys):
    assert ep.main(["--dyn", "d", "--out", "/o"]) == 0
    out = capsys.readouterr().out
    assert "planned 552 s, budget 600 s" in out
    assert "A safety brake is not a contact." in out
