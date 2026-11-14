"""The car session procedure of brief 34 (tests/bench/procedures/pnp_session_v03.yaml): it loads in
the engine, every session step has a pass rule and a rollback, the fixed items are in the file,
and every launch command parses against the arguments pnp.launch.py declares on main. Arguments
that another brief adds are listed below with the brief number; the test fails once such an
argument exists on main and is still on the list (take it off, it is then checked like any other),
and fails if the brief's marker exists while the argument is still missing."""

from __future__ import annotations

import ast
import subprocess
from pathlib import Path

import pytest
import yaml
from racer_bench.engine import HumanConfirmStep, ScriptedStep, load_procedure, run_procedure

REPO = Path(__file__).resolve().parents[3]
PROCEDURE = REPO / "tests/bench/procedures/pnp_session_v03.yaml"
LAUNCH = REPO / "ros_ws/src/racer_bringup/launch/pnp.launch.py"

# (argument, value or None) -> (brief, marker file, text the marker file must contain or None)
ALLOW = {
    ("deskew", None): (
        33,
        "ros_ws/src/racer_track_perception/src/racer_track_perception/deskew.py",
        None,
    ),
    ("lap_mode", "observe"): (
        29,
        "ros_ws/src/racer_laps/src/racer_laps/lap_memory_node.py",
        '"observe"',
    ),
}
SESSION_STEPS = set(range(9))
KNOWN_REPORT_COMMANDS = {"timing", "laps", "host-facts"}


def declared_arguments() -> tuple[set[str], set[str]]:
    """(names of the DeclareLaunchArgument calls, the LAP_MODES tuple) of pnp.launch.py."""
    tree = ast.parse(LAUNCH.read_text())
    names: set[str] = set()
    lap_modes: set[str] = set()
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Call)
            and getattr(node.func, "id", "") == "DeclareLaunchArgument"
            and node.args
            and isinstance(node.args[0], ast.Constant)
        ):
            names.add(str(node.args[0].value))
        if isinstance(node, ast.Assign) and any(
            getattr(t, "id", "") == "LAP_MODES" for t in node.targets
        ):
            lap_modes = {e.value for e in node.value.elts}  # type: ignore[attr-defined]
    return names, lap_modes


def raw() -> dict:
    return yaml.safe_load(PROCEDURE.read_text())


def session_steps() -> list[dict]:
    return [s for s in raw()["steps"] if "session_step" in s]


def marker_exists(path: str, needle: str | None) -> bool:
    p = REPO / path
    if not p.is_file():
        return False
    return needle is None or needle in p.read_text()


def test_the_procedure_loads_in_the_engine():
    proc = load_procedure(PROCEDURE)
    assert proc.name == "pnp_session_v03"
    assert any(isinstance(s, HumanConfirmStep) for s in proc.steps)
    assert any(isinstance(s, ScriptedStep) for s in proc.steps)


def test_steps_zero_to_eight_are_all_there_in_order():
    seen = [s["session_step"] for s in session_steps()]
    assert set(seen) == SESSION_STEPS
    assert seen == sorted(seen)


def test_every_session_step_has_a_pass_rule_with_a_section_and_a_rollback():
    for step in session_steps():
        rule = step.get("pass_rule")
        assert rule and rule["section"] and len(rule["text"]) > 10, step["id"]
        assert str(step.get("rollback", "")).strip(), step["id"]


def test_fixed_items_are_in_the_file_not_left_to_the_day():
    fixed = raw()["fixed"]
    assert fixed["caps_mps_step1"] == [0.5, 1.0]
    assert fixed["direction_order"] == ["ccw", "cw"]
    assert fixed["runs_per_direction_per_step"] == 2
    assert fixed["people"] == 2 and fixed["kill_knob_in_hand"] is True
    assert fixed["wheels_off_ground_check_before_the_floor"] is True
    assert fixed["battery_window_v"] == pytest.approx(0.8)
    assert "before the first run" in fixed["battery_window_rule"].lower()
    assert "growing" in fixed["recorder_rule"] and "refuses" in fixed["recorder_rule"]
    assert "scheduling policy" in fixed["no_settings_changed"]


def test_floor_steps_run_ccw_first_and_two_per_direction():
    for step in session_steps():
        runs = step.get("runs")
        if not runs:
            continue
        dirs = runs["directions"]
        assert dirs[0] == "ccw", step["id"]
        if len(dirs) > 1:
            assert dirs.count("ccw") == 2 and dirs.count("cw") == 2, step["id"]


def test_step_zero_checks_the_wheels_and_the_recorder_before_the_floor():
    step0 = next(s for s in session_steps() if s["session_step"] == 0)
    text = step0["prompt"].lower()
    assert "wheels off the ground" in text and "kill knob" in text and "growing" in text
    assert any("wheels off" in c for c in step0["human_confirms"])


def test_step_one_carries_the_read_only_host_commands_and_sets_nothing():
    step = next(s for s in raw()["steps"] if s["id"] == "s1_host_facts")
    cmds = [c[0] for c in step["read_only_commands"]]
    assert cmds == ["chrt", "cat", "uptime"]
    chrt = step["read_only_commands"][0]
    assert chrt[:2] == ["chrt", "-p"] and len(chrt) == 3  # -p alone only reads
    text = PROCEDURE.read_text()
    for forbidden in ("SCHED_FIFO", "nice -n", "scaling_governor performance", "sudo"):
        assert forbidden not in text.replace("no_settings_changed", "")
    assert any("owner decision, not taken" in s.get("prompt", "").lower() for s in session_steps())


def _launch_commands() -> list[tuple[str, list[str]]]:
    out = []
    for step in raw()["steps"]:
        for cmd in step.get("launch", []):
            out.append((step["id"], cmd))
    return out


def test_every_launch_command_parses_against_pnp_launch_py():
    declared, lap_modes = declared_arguments()
    assert {"dynamics_bundle", "v_max_mps", "lap_memory", "lap_counter"} <= declared
    commands = _launch_commands()
    assert commands
    for step_id, cmd in commands:
        assert cmd[:4] == ["ros2", "launch", "racer_bringup", "pnp.launch.py"], step_id
        seen = set()
        for arg in cmd[4:]:
            assert ":=" in arg, f"{step_id}: {arg}"
            name, value = arg.split(":=", 1)
            assert name not in seen, f"{step_id}: {name} given twice"
            seen.add(name)
            if name in declared and not (name == "lap_mode" and value not in lap_modes):
                continue
            key = (name, value) if (name, value) in ALLOW else (name, None)
            assert key in ALLOW, f"{step_id}: {arg} is not declared in pnp.launch.py"
        assert "require_recorder:=false" not in cmd, step_id


def test_the_allow_list_is_current():
    declared, lap_modes = declared_arguments()
    for (name, value), (brief, marker, needle) in ALLOW.items():
        present = (value in lap_modes) if name == "lap_mode" else (name in declared)
        label = f"{name}:={value}" if value else name
        assert not present, (
            f"{label} (brief {brief}) is on main: take it off ALLOW, it is checked like any other"
        )
        assert not marker_exists(marker, needle), (
            f"brief {brief}'s marker {marker} exists but {label} is still not declared in "
            "pnp.launch.py: the brief changed its interface, fix the procedure"
        )


def test_after_run_commands_point_at_things_that_exist():
    for step in raw()["steps"]:
        for cmd in step.get("after", []):
            assert cmd[0] == "uv" and cmd[1] == "run", step["id"]
            project = cmd[cmd.index("--project") + 1]
            assert (REPO / project / "pyproject.toml").is_file(), (step["id"], project)
            if "tools/pnp_session_report.py" in cmd:
                assert (REPO / "tools/pnp_session_report.py").is_file()
                sub = cmd[cmd.index("tools/pnp_session_report.py") + 1]
                assert sub in KNOWN_REPORT_COMMANDS, (step["id"], sub)
            if "racer_eval" in cmd:
                assert cmd[cmd.index("racer_eval") + 1] == "session"
            if "racer_pnp_dynamics.cli" in cmd:
                # brief 31's confirm command: skip if not on main (the step says so)
                assert any("brief 31" in r for r in step["requires"]), step["id"]


def test_steps_that_depend_on_other_briefs_say_skip_if_not_on_main():
    for step in raw()["steps"]:
        text = " ".join(step.get("requires", []))
        for arg, brief in (("deskew:=true", 33), ("lap_mode:=observe", 29)):
            uses = any(arg in " ".join(c) for c in step.get("launch", []))
            if uses:
                assert f"brief {brief}" in text and "skip" in text, (step["id"], arg)


def test_the_scripted_step_runs_in_the_engine_with_a_fake_runner():
    proc = load_procedure(PROCEDURE)
    seen = []

    def fake_run(command):
        seen.append(list(command))
        return subprocess.CompletedProcess(command, 0, stdout="", stderr="")

    record = run_procedure(proc, confirm=lambda _prompt: True, run=fake_run)
    assert record.passed
    assert any("host-facts" in c for c in seen)
    assert len(record.results) == len(proc.steps)
