"""L1 tests of tools/pnp_session/cap_schedule.py (brief 41a, M-SID): the schedule, the refusals,
the guidance fields, the planner params, and that the module can only publish the guidance topic.

The L3 check (the planner follows the cap, a dead tool falls back to v_no_guidance_mps) needs a
sourced ROS workspace and a runner mode in tests/l5_reactive_bench/track_family_runner.py, which
is frozen until the B4 report: it is not in this file and the PR says so."""

from __future__ import annotations

import ast
from pathlib import Path

import pytest
import yaml
from pnp_session import cap_schedule as cs

MODULE = Path(cs.__file__)
REPO = Path(__file__).resolve().parents[2]
ADDENDUM = REPO / cs.EXCITATION_YAML


def manifest(tmp_path: Path, v_max: float = 1.3) -> Path:
    p = tmp_path / "manifest.yaml"
    p.write_text(yaml.safe_dump({"io": {"validity": {"v_min_mps": 0.0, "v_max_mps": v_max}}}))
    return p


def test_the_schedule_is_read_from_the_addendum_yaml():
    exc = cs.load_excitation()
    assert exc.caps_mps == (0.5, 1.0, 0.7, 1.3)
    assert exc.hold_s == 8.0 and exc.cycles == 3
    assert yaml.safe_load(ADDENDUM.read_text())["excitation"]["caps_mps"] == list(exc.caps_mps)


def test_the_cap_at_each_second_and_the_end_of_the_schedule():
    sched = cs.CapSchedule(cs.Excitation((0.5, 1.0, 0.7, 1.3), 8.0, 3))
    assert sched.duration_s == 96.0
    assert sched.cap_at(-0.001) is None
    assert sched.cap_at(0.0) == 0.5 and sched.cap_at(7.999) == 0.5
    assert sched.cap_at(8.0) == 1.0 and sched.cap_at(16.0) == 0.7 and sched.cap_at(24.0) == 1.3
    assert sched.cap_at(32.0) == 0.5  # the second cycle starts at 32 s
    assert sched.cap_at(95.999) == 1.3
    assert sched.cap_at(96.0) is None and sched.cap_at(1e9) is None
    steps = sched.steps()
    assert len(steps) == 12 and steps[0] == (0.0, 8.0, 0.5) and steps[-1] == (88.0, 96.0, 1.3)
    assert [c for _, _, c in steps[:4]] == [0.5, 1.0, 0.7, 1.3]


@pytest.mark.parametrize(
    ("caps", "validity", "v_max", "text"),
    [
        ((0.5, 1.0, 0.7, 1.3), 1.3, 1.3, None),  # at the validity is allowed
        ((0.5, 1.3000001), 1.3, 1.5, "above the bundle's validity"),
        ((0.5, 1.4), 1.5, 1.3, "above the planner's v_max_mps"),
        ((0.5, 0.0), 1.3, 1.3, "positive finite"),
        ((0.5, float("nan")), 1.3, 1.3, "positive finite"),
        ((0.5, float("inf")), 1.3, 1.3, "positive finite"),
        ((), 1.3, 1.3, "no caps"),
    ],
)
def test_caps_above_validity_or_v_max_are_refused_before_start(caps, validity, v_max, text):
    if text is None:
        cs.check_caps(caps, validity, v_max)
        return
    with pytest.raises(cs.CapRefused, match=text):
        cs.check_caps(caps, validity, v_max)


def test_another_guidance_publisher_refuses():
    cs.refuse_if_other_publisher(0)
    with pytest.raises(cs.CapRefused, match="never runs together with the lap node"):
        cs.refuse_if_other_publisher(1)


def test_guidance_fields_set_only_the_cap():
    f = cs.guidance_fields(0.7)
    assert f == {"lap_index": 1, "v_cap_mps": 0.7, "s_m": [], "v_ref_mps": [], "cost_to_go_s": []}


def test_planner_params_turn_guidance_on_with_the_lowest_cap_as_the_fallback():
    doc = yaml.safe_load(cs.planner_params_yaml(0.5))
    params = doc["mppi_planner_node"]["ros__parameters"]
    assert params == {"use_lap_guidance": True, "v_no_guidance_mps": 0.5}


def test_bundle_validity_from_the_manifest(tmp_path):
    assert cs.bundle_validity_mps(manifest(tmp_path, 1.3)) == 1.3
    bad = tmp_path / "bad.yaml"
    bad.write_text("io: {}\n")
    with pytest.raises(cs.CapRefused, match="validity"):
        cs.bundle_validity_mps(bad)


def test_main_writes_params_and_refuses(tmp_path, capsys):
    out = tmp_path / "p.yaml"
    argv = ["--manifest", str(manifest(tmp_path)), "--v-max-mps", "1.3", "--write-params", str(out)]
    assert cs.main(argv) == 0
    params = yaml.safe_load(out.read_text())["mppi_planner_node"]["ros__parameters"]
    assert params["v_no_guidance_mps"] == min(cs.load_excitation().caps_mps)
    # a bundle whose validity is below 1.3: refused before anything is written or published
    out.unlink()
    low = [
        "--manifest",
        str(manifest(tmp_path, 1.0)),
        "--v-max-mps",
        "1.3",
        "--write-params",
        str(out),
    ]
    assert cs.main(low) == cs.EXIT_REFUSED and not out.exists()
    assert "REFUSED" in capsys.readouterr().err
    # a planner v_max below the top cap is refused too
    assert cs.main(["--manifest", str(manifest(tmp_path)), "--v-max-mps", "1.0"]) == cs.EXIT_REFUSED


def test_main_starts_the_schedule_when_nothing_refuses(tmp_path, monkeypatch, capsys):
    started = []
    monkeypatch.setattr(cs, "run_schedule", lambda sched: started.append(sched.duration_s) or 0)
    assert cs.main(["--manifest", str(manifest(tmp_path)), "--v-max-mps", "1.3"]) == 0
    assert started == [96.0]
    assert "caps [0.5, 1.0, 0.7, 1.3]" in capsys.readouterr().out
    custom = tmp_path / "ex.yaml"
    custom.write_text(
        yaml.safe_dump(
            {"excitation": {"caps_mps": [0.5], "hold_s": 1.0, "cycles_per_direction": 2}}
        )
    )
    started.clear()
    assert (
        cs.main(
            [
                "--manifest",
                str(manifest(tmp_path)),
                "--v-max-mps",
                "1.3",
                "--excitation-yaml",
                str(custom),
            ]
        )
        == 0
    )
    assert started == [2.0]


def test_the_module_can_only_publish_the_guidance_topic():
    tree = ast.parse(MODULE.read_text())
    publishers = [
        n
        for n in ast.walk(tree)
        if isinstance(n, ast.Call) and getattr(n.func, "attr", "") == "create_publisher"
    ]
    assert len(publishers) == 1
    topic = publishers[0].args[1]
    assert isinstance(topic, ast.Name) and topic.id == "GUIDANCE_TOPIC"
    assert cs.GUIDANCE_TOPIC == "/pnp/lap/guidance"
    strings = {
        n.value for n in ast.walk(tree) if isinstance(n, ast.Constant) and isinstance(n.value, str)
    }
    assert not {s for s in strings if s.strip().startswith(("/drive", "/cmd"))}
    # rclpy is imported only inside the shell, so the module imports without ROS
    top_level = [n for n in tree.body if isinstance(n, ast.Import | ast.ImportFrom)]
    assert not any("rclpy" in ast.dump(n) for n in top_level)


def test_the_parser_and_the_procedure_commands_agree(tmp_path):
    doc = yaml.safe_load((REPO / "tests/bench/procedures/pnp_session_v04.yaml").read_text())
    step = next(x for x in doc["steps"] if x["id"] == "s9a_excitation")
    for cmd in step["schedule_commands"]:
        cs.build_parser().parse_args(cmd[2:])
    ex = step["excitation"]
    exc = cs.load_excitation()
    assert (list(exc.caps_mps), exc.hold_s, exc.cycles) == (
        ex["caps_mps"],
        ex["hold_s"],
        ex["cycles_per_direction"],
    )
    assert cs.build_parser().parse_args(["--manifest", "m", "--v-max-mps", "1.3"]).v_max_mps == 1.3
    assert cs.build_parser().parse_args(["--manifest", "m", "--v-max-mps", "1.3"]).v_max_mps == 1.3
