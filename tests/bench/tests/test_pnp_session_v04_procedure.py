"""The v0.4 car session procedure of brief 41 (tests/bench/procedures/pnp_session_v04.yaml).

It loads in the engine; every session step has a pass rule whose sources are files and sections
that exist, a rollback and a `requires` list; a step with a result that is not `met` cannot be
marked runnable; every launch command parses against the arguments the launch files declare on
main. Tools and launch arguments of other briefs that may not be on main yet are listed in
TOOLS_NOT_ON_MAIN and PENDING below; the test fails once one is on main and still on the list
(take it off, it is then checked like any other).
"""

from __future__ import annotations

import ast
import hashlib
import re
import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest
import yaml
from racer_bench.engine import HumanConfirmStep, ScriptedStep, load_procedure, run_procedure

REPO = Path(__file__).resolve().parents[3]
PROCEDURE = REPO / "tests/bench/procedures/pnp_session_v04.yaml"
RUNBOOK = REPO / "docs/notes/pnp-session-runbook-v0.4.md"
LAUNCH = REPO / "ros_ws/src/racer_bringup/launch/pnp.launch.py"
GAP_FOLLOW = REPO / "ros_ws/src/racer_control/launch/gap_follow.launch.py"
PREREG_YAML = REPO / "evaluation/prereg/pnp-v0.4.yaml"
PREREG_SHA256 = "78510c1423278f80c42511e6caab095818598af6db6a284dd5479f74a03a1d63"

# Tools of other briefs that are not on main when this file is written: script -> brief. The test
# fails when the script exists and is still listed.
TOOLS_NOT_ON_MAIN: dict[str, int] = {
    "tools/pnp_data.py": 36,
    "tools/lapref.py": 36,
}
# racer_eval modules of other briefs, and topics, that are not on main when this file is written:
# name -> (brief, marker file). The test fails when the marker exists and the name is still listed.
RACER_EVAL_NOT_ON_MAIN = {
    "timing_chain": (42, "evaluation/analysis/src/racer_eval/timing_chain.py"),
}
TOPICS_NOT_ON_MAIN = {
    "/pnp/planner/timing": (42, "ros_ws/src/racer_pnp_msgs/msg/PlannerTiming.msg"),
}
SESSION_STEPS = list(range(10))
MINIMUM_SESSION = {1, 2, 9}
KNOWN_REPORT_COMMANDS = {"timing", "laps", "host-facts", "v04", "variant-form", "check-prereg"}
RESULT_STATUSES = {"met", "unmet", "unknown"}


def raw() -> dict[str, Any]:
    return yaml.safe_load(PROCEDURE.read_text())


def session_steps() -> list[dict[str, Any]]:
    return [s for s in raw()["steps"] if "session_step" in s]


def step(step_id: str) -> dict[str, Any]:
    return next(s for s in raw()["steps"] if s["id"] == step_id)


def declared_arguments(path: Path) -> set[str]:
    tree = ast.parse(path.read_text())
    names = {
        str(node.args[0].value)
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and getattr(node.func, "id", "") == "DeclareLaunchArgument"
        and node.args
        and isinstance(node.args[0], ast.Constant)
    }
    # gap_follow.launch.py declares its tuning arguments from module level dicts named
    # _OPTIONAL_*_ARGS (one DeclareLaunchArgument per key, in a comprehension)
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Assign)
            and isinstance(node.value, ast.Dict)
            and any(
                getattr(t, "id", "").startswith("_OPTIONAL_")
                and getattr(t, "id", "").endswith("_ARGS")
                for t in node.targets
            )
        ):
            names |= {str(k.value) for k in node.value.keys if isinstance(k, ast.Constant)}
    return names


def lap_modes() -> set[str]:
    tree = ast.parse(LAUNCH.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(
            getattr(t, "id", "") == "LAP_MODES" for t in node.targets
        ):
            return {e.value for e in node.value.elts}  # type: ignore[attr-defined]
    return set()


def all_declared_arguments() -> set[str]:
    out: set[str] = set()
    for p in (REPO / "ros_ws/src").glob("*/launch/*.py"):
        out |= declared_arguments(p)
    return out


def heading_exists(path: Path, section: str) -> bool:
    text = path.read_text()
    return re.search(rf"^#+ {re.escape(section)}(?:[ .]|$)", text, re.MULTILINE) is not None


def unmet_of(req: dict[str, Any], status: dict[str, str]) -> list[str]:
    """Results a `requires` entry still waits for: {result: id} needs id met; {any_of: [[ids],
    [ids]]} is met when every id of at least one branch is (empty list: met)."""
    if "result" in req:
        return [] if status[req["result"]] == "met" else [req["result"]]
    best: list[str] | None = None
    for branch in req["any_of"]:
        missing = [r for r in branch if status[r] != "met"]
        if not missing:
            return []
        best = missing if best is None or len(missing) < len(best) else best
    return best or []


def requirement_ids(req: dict[str, Any]) -> set[str]:
    return {req["result"]} if "result" in req else {r for b in req["any_of"] for r in b}


def runnable_problems(doc: dict[str, Any]) -> list[str]:
    """A step marked runnable with a result that is not met, or one whose mark is not the rule."""
    status = {k: v["status"] for k, v in doc["results"].items()}
    problems = []
    for s in doc["steps"]:
        if "session_step" not in s:
            continue
        unmet = [m for r in s["requires"] for m in unmet_of(r, status)]
        if s["runnable"] and unmet:
            problems.append(f"{s['id']} is marked runnable but {unmet} is not met")
        if not s["runnable"] and not unmet:
            problems.append(f"{s['id']} is marked not runnable with every requirement met")
    return problems


# ----------------------------------------------------------------------------- the file itself


def test_the_procedure_loads_in_the_engine():
    proc = load_procedure(PROCEDURE)
    assert proc.name == "pnp_session_v04"
    assert any(isinstance(s, HumanConfirmStep) for s in proc.steps)
    assert any(isinstance(s, ScriptedStep) for s in proc.steps)


def test_steps_zero_to_nine_are_all_there_in_order():
    seen = [s["session_step"] for s in session_steps()]
    assert sorted(set(seen)) == SESSION_STEPS
    assert seen == sorted(seen)


def test_every_session_step_has_sources_a_rollback_and_a_requires_list():
    for s in session_steps():
        rule = s.get("pass_rule")
        assert rule and len(rule["text"]) > 10 and rule["sources"], s["id"]
        assert str(s.get("rollback", "")).strip(), s["id"]
        assert isinstance(s.get("requires"), list), s["id"]
        assert isinstance(s.get("runnable"), bool), s["id"]
        assert str(s.get("bagged", "")).strip(), s["id"]
        assert "id" in s and s["type"] in ("human_confirm", "scripted")


def test_every_pass_rule_and_result_cites_a_file_and_section_that_exist():
    cited = [(s["id"], src) for s in session_steps() for src in s["pass_rule"]["sources"]]
    cited += [(k, v["source"]) for k, v in raw()["results"].items()]
    assert cited
    for owner, src in cited:
        path = REPO / src["file"]
        assert path.is_file(), (owner, src)
        assert heading_exists(path, str(src["section"])), (owner, src)


def test_the_cited_files_are_the_registered_ones():
    cited = {src["file"] for s in session_steps() for src in s["pass_rule"]["sources"]}
    assert cited <= {
        "docs/notes/preregistration-pnp-v0.2.md",
        "docs/notes/preregistration-pnp-v0.3.md",
        "docs/notes/preregistration-pnp-v0.4.md",
        "docs/notes/preregistration-pnp-v0.4-addendum-1.md",
        "docs/notes/pnp-session-runbook-v0.4.md",
    }
    assert "preregistration-pnp-v0.4.md" in " ".join(cited)


def test_the_frozen_constants_file_hashes_to_the_registered_value():
    assert PREREG_YAML.is_file(), "pull main: PR 186 (the v0.4 pre-registration) is not here"
    assert hashlib.sha256(PREREG_YAML.read_bytes()).hexdigest() == PREREG_SHA256
    assert PREREG_SHA256 in step("s0_prereg_sha")["pass_rule"]["text"]


def test_fixed_items_are_in_the_file_not_left_to_the_day():
    fixed = raw()["fixed"]
    assert fixed["caps_mps_step1"] == [0.5, 1.0]
    assert fixed["direction_order"] == ["ccw", "cw"]
    assert fixed["runs_per_direction_per_step"] == 2 and fixed["laps_per_run_step1"] == 10
    assert fixed["people"] == 2 and fixed["kill_knob_in_hand"] is True
    assert fixed["wheels_off_ground_check_before_the_floor"] is True
    assert fixed["phone_films_every_step1_run"] is True
    assert fixed["camera_counter_record_hz"] == 10
    assert fixed["battery_window_v"] == pytest.approx(0.8)
    assert "before the first run" in fixed["battery_window_rule"].lower()
    assert "growing" in fixed["recorder_rule"] and "refuses" in fixed["recorder_rule"]
    assert "scheduling policy" in fixed["no_settings_changed"]
    assert "watchdog limit" in fixed["no_settings_changed"]
    assert "refuses a cap above" in fixed["app_rule"]
    assert "toggle persists" in fixed["app_rule"]
    assert "Pull main" in fixed["pull_main_rule"]


def test_minimum_useful_session_is_steps_one_two_and_nine():
    marked = {s["session_step"] for s in session_steps() if s.get("minimum_session")}
    assert marked == MINIMUM_SESSION


def test_floor_steps_run_ccw_first_and_two_per_direction():
    for s in session_steps():
        runs = s.get("runs")
        if not runs:
            continue
        dirs = runs["directions"]
        assert dirs[0] == "ccw", s["id"]
        if len(dirs) > 2:
            assert dirs.count("ccw") == 2 and dirs.count("cw") == 2, s["id"]


def test_step_zero_checks_the_wheels_the_counter_and_the_recorder_before_the_floor():
    s0 = step("s0_build_and_bench")
    text = s0["prompt"].lower()
    assert "wheels off the ground" in text and "kill knob" in text and "growing" in text
    assert "camera counter" in text and "10 hz" in text
    assert any("wheels off" in c for c in s0["human_confirms"])
    assert step("s0_prereg_sha")["command"][-1] == "check-prereg"


def test_nothing_sets_a_scheduling_policy_a_limit_or_a_safety_parameter():
    text = PROCEDURE.read_text()
    for forbidden in (
        "SCHED_FIFO",
        "nice -n",
        "chrt -f",
        "scaling_governor performance",
        "sudo",
        "require_recorder:=false",
        "watchdog_limit",
        "limiter_dt",
    ):
        assert forbidden not in text, forbidden
    assert any("owner decision, not taken" in s.get("prompt", "").lower() for s in session_steps())
    assert "—" not in text and "–" not in text  # no em or en dashes


def test_step_one_judges_4_2_and_d3c0_with_the_registered_reference_rule():
    s1 = step("s1_c_timing_and_d3c0")
    text = s1["pass_rule"]["text"]
    assert "v0.2 4.2" in text and "D3-C0" in text and "phone video" in text
    assert "lapref" in text and "decides nothing" in text
    assert s1["runs"]["caps_mps"] == [0.5, 1.0] and s1["runs"]["laps_per_run"] == 10
    assert all("lap_mode:=observe" in " ".join(c) for c in s1["launch"])
    assert any("v04" in c for c in s1["after"])
    assert any(c[0] == "uv" and "tools/lapref.py" in c for c in s1["after"])


def test_variant_protocol_is_written_down_and_steps_six_and_seven_use_different_variants():
    proto = raw()["variant_protocol"]
    assert proto["min_objects"] == 2 and proto["min_lane_change_m"] == 0.15
    assert proto["min_remaining_lane_m"] == 0.80 and proto["free_standing_objects"] is False
    assert proto["step6_and_step7_use_different_variants"] is True
    assert "GP4-V" in proto["gp4v_label"] and "known track" in proto["gp4v_label"]
    s6, s7 = step("s6_variant_1_d_off"), step("s7_gp4v_variant_2")
    assert s6["variant_form"]["file"] != s7["variant_form"]["file"]
    assert s6["variant_form"]["file"] in s7["variant_form"]["earlier_forms"]
    assert "NEW variant" in s7["prompt"] and "not variant 1" in " ".join(s7["human_confirms"])
    for s in (s6, s7):
        assert "variant-form" in s["variant_form"]["check"]
        assert "variant not documented" in s["prompt"] or "seed" in s["prompt"]
    assert "seed" in s7["prompt"] and "BEFORE the first run" in s7["prompt"]
    # the variant is never called an unseen layout or GP4 in a pass rule
    assert "GP4-V" in s7["pass_rule"]["text"] and "never as GP4" in s7["prompt"]
    assert "unseen layout" not in s6["pass_rule"]["text"].replace("not as an unseen layout", "")


# ------------------------------------------------------------------------------ requires logic


def test_requires_ids_are_in_the_results_registry_with_known_statuses():
    doc = raw()
    for s in session_steps():
        for r in s["requires"]:
            assert requirement_ids(r) <= set(doc["results"]), (s["id"], r)
    for k, v in doc["results"].items():
        assert v["status"] in RESULT_STATUSES, k
        assert v["what"].strip(), k


def test_a_step_with_an_unmet_requirement_cannot_be_marked_runnable():
    assert runnable_problems(raw()) == []
    wrongly_open = raw()
    for s in wrongly_open["steps"]:
        if s["id"] == "s5_d_v0_plain_donut":
            s["runnable"] = True
    assert "s5_d_v0_plain_donut is marked runnable" in " ".join(runnable_problems(wrongly_open))
    wrongly_shut = raw()
    for k in ("d3_s0", "d3_s1", "d3_c0", "code_equals_b4"):
        wrongly_shut["results"][k]["status"] = "met"
    assert "marked not runnable" in " ".join(runnable_problems(wrongly_shut))
    wrongly_shut["steps"][[s["id"] for s in wrongly_shut["steps"]].index("s5_d_v0_plain_donut")][
        "runnable"
    ] = True
    assert runnable_problems(wrongly_shut) == []
    unmet = raw()
    unmet["results"]["d3_c0"]["status"] = "unmet"  # a FAIL is not met either
    assert runnable_problems(unmet) == []  # step 5 stays closed


def test_the_registered_preconditions_are_the_requirements():
    needs = {
        s["id"]: {i for r in s["requires"] for i in requirement_ids(r)} for s in session_steps()
    }
    code_or_b6 = {"code_equals_b4", "d5_s0", "d5_s1"}
    assert needs["s5_d_v0_plain_donut"] == {"d3_s0", "d3_s1", "d3_c0"} | code_or_b6  # 6.1, add. 5.2
    assert needs["s7_gp4v_variant_2"] == (
        {"d4_s0", "d4_s1", "step5_passed", "d6_s0", "d6_s1"} | code_or_b6
    )  # v0.4 6.2, add. 5.2
    assert needs["s3_cap_above_1_3"] == {"bundle_validity_above_1_3"}  # v0.4 6.4
    assert needs["s4_fused_shadow"] == {"fused_bundle_frozen"}
    assert needs["s1b_gp3_r1"] == {"m_cam"}  # brief 41a: GP3-R1 needs M-CAM held
    for sid in (
        "s0_prereg_sha",
        "s0_build_and_bench",
        "s1_c_timing_and_d3c0",
        "s2_dynamics_confirmation",
        "s6_variant_1_d_off",
        "s8_tracker_against_pnp",
        "s9a_excitation",
        "s9_data_laps",
    ):
        assert needs[sid] == set(), sid


def _with(doc: dict[str, Any], **statuses: str) -> dict[str, Any]:
    for k, v in statuses.items():
        doc["results"][k]["status"] = v
    return doc


def _runnable(doc: dict[str, Any], step_id: str) -> bool:
    status = {k: v["status"] for k, v in doc["results"].items()}
    s = next(x for x in doc["steps"] if x["id"] == step_id)
    return not [m for r in s["requires"] for m in unmet_of(r, status)]


def test_steps_five_and_seven_open_on_the_b4_code_or_on_b6_and_on_nothing_else():
    d3 = {"d3_s0": "met", "d3_s1": "met", "d3_c0": "met"}
    d4 = {"d4_s0": "met", "d4_s1": "met", "step5_passed": "met"}
    five, seven = "s5_d_v0_plain_donut", "s7_gp4v_variant_2"
    assert not _runnable(_with(raw(), **d3), five)  # neither branch
    # branch 1: the deployed code is the code B4 ran on
    assert _runnable(_with(raw(), **d3, code_equals_b4="met"), five)
    assert not _runnable(_with(raw(), **d3, code_equals_b4="unmet"), five)
    # branch 2: B6 passed on the changed code, D5 for the plain donut
    assert _runnable(_with(raw(), **d3, d5_s0="met", d5_s1="met"), five)
    assert not _runnable(_with(raw(), **d3, d5_s0="met"), five)
    assert not _runnable(_with(raw(), **d3, d5_s0="met", d5_s1="unmet"), five)
    # the B6 branch alone does not skip the B4 gates it requires them to have passed
    assert not _runnable(
        _with(raw(), d3_s0="unmet", d3_s1="met", d3_c0="met", code_equals_b4="met"), five
    )
    # step 7: D6 in addition on the B6 branch
    assert not _runnable(_with(raw(), **d4), seven)
    assert _runnable(_with(raw(), **d4, code_equals_b4="met"), seven)
    assert not _runnable(_with(raw(), **d4, d5_s0="met", d5_s1="met"), seven)
    assert _runnable(_with(raw(), **d4, d5_s0="met", d5_s1="met", d6_s0="met", d6_s1="met"), seven)
    assert not _runnable(_with(raw(), **d4, d5_s0="met", d5_s1="met", d6_s0="met"), seven)
    # runnable_problems sees an any_of step marked open without either branch
    bad = _with(raw(), **d3)
    next(x for x in bad["steps"] if x["id"] == five)["runnable"] = True
    assert "s5_d_v0_plain_donut is marked runnable" in " ".join(runnable_problems(bad))


def test_gp3_is_worded_as_not_met_as_registered_and_later_runs_are_gp3_r1():
    fixed = raw()["fixed"]
    assert fixed["gp3_record"] == (
        "GP3 not met as registered; 20 clean laps observed, camera count 10 of 20"
    )
    assert (
        "GP3-R1" in fixed["gp3_repeat_rule"] and "never reported as GP3" in fixed["gp3_repeat_rule"]
    )
    text = PROCEDURE.read_text() + RUNBOOK.read_text()
    assert not re.search(r"GP3 (PASSED|passed)", text)
    s1b = step("s1b_gp3_r1")
    assert s1b["optional"] is True and s1b["session_step"] == 1
    assert s1b["runs"]["laps_per_run"] == 20 and s1b["runs"]["caps_mps"] == [1.0]
    assert "GP3-R1" in s1b["pass_rule"]["text"] and "never as GP3" in s1b["pass_rule"]["text"]
    order = [s["id"] for s in raw()["steps"]]
    assert (
        order.index("s1b_gp3_r1") == order.index("s1_c_timing_and_d3c0") + 1
    )  # right after step 1


def test_the_timing_chain_is_in_step_one_and_listed_until_brief_42_lands():
    for name, (brief, marker) in RACER_EVAL_NOT_ON_MAIN.items():
        assert not (REPO / marker).is_file(), (
            f"racer_eval {name} (brief {brief}) is on main: take it off the list"
        )
    for topic, (brief, marker) in TOPICS_NOT_ON_MAIN.items():
        assert not (REPO / marker).is_file(), (
            f"{topic} (brief {brief}) is on main: take it off the list"
        )
    s1 = step("s1_c_timing_and_d3c0")
    assert any("timing_chain" in c for c in s1["after"])
    assert "brief 42" in " ".join(s1["needs_on_main"]) and "skip" in " ".join(s1["needs_on_main"])
    assert "/pnp/planner/timing" in raw()["fixed"]["timing_chain_rule"]
    assert "judged as before" in raw()["fixed"]["timing_chain_rule"]
    for s in session_steps():
        if s["session_step"] in (1, 9) or "pnp.launch.py" in str(s.get("launch", "")):
            assert "/drive_raw" in s["bagged"] or s["session_step"] == 0, s["id"]


def test_the_excitation_step_follows_the_v05_yaml_and_runs_clockwise_first():
    s = step("s9a_excitation")
    ex = s["excitation"]
    yaml_block = yaml.safe_load((REPO / ex["source"]["file"]).read_text())["excitation"]
    assert ex["max_total_driving_s"] == yaml_block["max_total_driving_s"] == 600
    assert ex["planned_total_driving_s"] == yaml_block["planned_total_driving_s"] == 552
    assert sum(b["s"] for b in ex["blocks"]) == 552
    dirs = [b["direction"] for b in ex["blocks"]]
    assert dirs == ["cw"] * 6 + ["ccw"] * 5  # clockwise first, no G at 1.0 counter-clockwise
    assert s["minimum_session"] is True and step("s9_data_laps")["minimum_session"] is True
    assert "lap_memory" not in " ".join(" ".join(c) for c in s["launch"])  # off by default
    text = s["prompt"]
    for needle in (
        "a wall contact ends the blocks of that direction",
        "rung F 1.5 only if rung F 1.4 had no contact",
        "owner's yes",
        "stalls twice in its first 20 s",
        "a safety brake is not a contact",
        "2.4 and 2.8",
        "have not been driven on the car",
        "STOP and a standing start every 30 s",
    ):
        assert needle in text, needle
    assert (REPO / "tools/pnp_session/cap_schedule.py").is_file()
    assert (REPO / "tools/pnp_session/excitation_plan.py").is_file()
    for cmd in s["schedule_commands"]:
        assert cmd[:2] == ["python3", "tools/pnp_session/cap_schedule.py"]
    assert not any("lap_mode" in " ".join(c) for c in s["launch"])


def test_the_gap_follow_lines_of_the_step_are_the_plan_tools_lines():
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "excitation_plan", REPO / "tools/pnp_session/excitation_plan.py"
    )
    ep = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = ep  # its dataclasses look their module up here
    spec.loader.exec_module(ep)
    plan = [r for r in ep.build_plan(ep.load_block()) if r.block != "P"]
    wanted = {r.launch_line()[0].split(" ", 2)[2] for r in plan}
    got = {" ".join(c[2:]) for c in step("s9a_excitation")["launch_other"]}
    assert got == wanted


def test_nothing_is_runnable_that_waits_on_a_result_nobody_has():
    status = {k: v["status"] for k, v in raw()["results"].items()}
    assert set(status.values()) == {"unknown"}  # nothing in this file was measured yet
    assert {s["session_step"] for s in session_steps() if s["runnable"]} == {0, 1, 2, 6, 8, 9}
    assert not step("s1b_gp3_r1")["runnable"]


# ------------------------------------------------------------------------ commands that exist


def launch_commands(key: str = "launch") -> list[tuple[str, list[str]]]:
    return [(s["id"], c) for s in raw()["steps"] for c in s.get(key, [])]


def test_every_launch_command_parses_against_pnp_launch_py():
    declared = declared_arguments(LAUNCH)
    assert {
        "dynamics_bundle",
        "v_max_mps",
        "lap_memory",
        "lap_counter",
        "shadow_perception_bundle",
    } <= declared
    modes = lap_modes()
    commands = launch_commands()
    assert commands
    for step_id, cmd in commands:
        assert cmd[:4] == ["ros2", "launch", "racer_bringup", "pnp.launch.py"], step_id
        seen: set[str] = set()
        for arg in cmd[4:]:
            assert ":=" in arg, f"{step_id}: {arg}"
            name, value = arg.split(":=", 1)
            assert name not in seen, f"{step_id}: {name} given twice"
            seen.add(name)
            assert name in declared, f"{step_id}: {arg} is not declared in pnp.launch.py"
            if name == "lap_mode":
                assert value in modes, f"{step_id}: lap_mode {value} is not in LAP_MODES"
        assert "require_recorder:=false" not in cmd, step_id


def test_other_launch_commands_parse_against_their_launch_file():
    declared = declared_arguments(GAP_FOLLOW)
    commands = launch_commands("launch_other")
    assert commands
    for step_id, cmd in commands:
        assert cmd[:4] == ["ros2", "launch", "racer_control", "gap_follow.launch.py"], step_id
        for arg in cmd[4:]:
            assert arg.split(":=", 1)[0] in declared, (step_id, arg)


def test_pending_launch_arguments_are_current():
    declared = all_declared_arguments()
    pending = [(s["id"], p) for s in raw()["steps"] for p in s.get("pending_launch_args", [])]
    assert pending
    for step_id, p in pending:
        matches = sorted(a for a in declared if all(m in a.lower() for m in p["match"]))
        assert not matches, (
            f"{step_id}: brief {p['brief']}'s argument is on main ({matches}): put it in the "
            "launch commands of the step and remove the pending entry"
        )
        assert f"brief {p['brief']}" in " ".join(step(step_id)["needs_on_main"])


def test_tools_of_other_briefs_are_listed_until_they_land():
    for script, brief in TOOLS_NOT_ON_MAIN.items():
        assert not (REPO / script).is_file(), (
            f"{script} (brief {brief}) is on main: take it off TOOLS_NOT_ON_MAIN, it is then "
            "checked like any other"
        )
    for s in raw()["steps"]:
        for cmd in s.get("after", []):
            assert cmd[0] == "uv" and cmd[1] == "run", s["id"]
            project = cmd[cmd.index("--project") + 1]
            assert (REPO / project / "pyproject.toml").is_file(), (s["id"], project)
            scripts = [c for c in cmd if re.fullmatch(r"tools/[a-z_0-9]+\.py", c)]
            for script in scripts:
                if script in TOOLS_NOT_ON_MAIN:
                    brief = TOOLS_NOT_ON_MAIN[script]
                    needs = " ".join(s["needs_on_main"])
                    assert f"brief {brief}" in needs and "skip" in needs, (s["id"], script)
                else:
                    assert (REPO / script).is_file(), (s["id"], script)
            if "tools/pnp_session_report.py" in cmd:
                sub = cmd[cmd.index("tools/pnp_session_report.py") + 1]
                assert sub in KNOWN_REPORT_COMMANDS, (s["id"], sub)
            if "racer_eval" in cmd:
                sub = cmd[cmd.index("racer_eval") + 1]
                if sub in RACER_EVAL_NOT_ON_MAIN:
                    brief = RACER_EVAL_NOT_ON_MAIN[sub][0]
                    needs = " ".join(s["needs_on_main"])
                    assert f"brief {brief}" in needs and "skip" in needs, (s["id"], sub)
                else:
                    assert sub == "session", (s["id"], sub)


def test_steps_that_use_another_briefs_work_say_skip_if_not_on_main():
    for s in raw()["steps"]:
        text = " ".join(s.get("needs_on_main", []))
        if any("pnp_data" in " ".join(c) for c in s.get("after", [])):
            assert "brief 36" in text and "skip" in text, s["id"]
        if any("<fused>" in " ".join(c) for c in s.get("launch", [])):
            assert "brief 38" in text, s["id"]
        if s.get("pending_launch_args"):
            assert "brief 39" in text and "skip" in text, s["id"]


def test_the_scripted_step_runs_in_the_engine_with_a_fake_runner():
    proc = load_procedure(PROCEDURE)
    seen = []

    def fake_run(command):
        seen.append(list(command))
        return subprocess.CompletedProcess(command, 0, stdout="", stderr="")

    record = run_procedure(proc, confirm=lambda _prompt: True, run=fake_run)
    assert record.passed
    assert any("check-prereg" in c for c in seen)
    assert len(record.results) == len(proc.steps)


# ---------------------------------------------------------------------------------- the runbook


def test_the_runbook_has_a_page_per_step_and_the_gotchas():
    text = RUNBOOK.read_text()
    for n in SESSION_STEPS:
        assert re.search(rf"^## Step {n}\b", text, re.MULTILINE), n
    for gotcha in (
        "tools/.venv",
        "toggle",
        "pull main",
        "refuses a cap above",
        "variant not documented",
    ):
        assert gotcha.lower() in text.lower(), gotcha
    assert "—" not in text and "–" not in text
    assert "GP4-V" in text
    for s in session_steps():
        assert s["id"].split("_", 1)[0] and f"Step {s['session_step']}" in text
