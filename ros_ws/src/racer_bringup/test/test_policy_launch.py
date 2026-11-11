"""L1 check of policy.launch.py (launch-description inspection, no processes started): the tracker
is launched in handoff mode with the lookahead the policy trained on, policy_node gets the same
raceline and pose source, and the launch refuses to build without a policy directory, a raceline
or a sidecar. Nothing here publishes /drive; that is racer_policy's L3 test."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
import yaml

pytest.importorskip("launch_ros")

_LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "policy.launch.py"


def _module():
    spec = importlib.util.spec_from_file_location("policy_launch", _LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _policy_dir(tmp_path: Path, base: dict | None = None, name: str = "p") -> Path:
    d = tmp_path / name
    d.mkdir()
    meta = {
        "must_match_at_deploy": {
            "base_controller": base
            or {
                "lookahead_min_m": 1.0,
                "lookahead_max_m": 1.2,
                "lookahead_curvature_ref_1pm": 1.0,
            }
        }
    }
    (d / "policy_meta.yaml").write_text(yaml.safe_dump(meta), encoding="utf-8")
    return d


def test_tracker_is_in_handoff_mode_with_the_policys_lookahead(tmp_path: Path) -> None:
    m = _module()
    d = str(_policy_dir(tmp_path))
    p, _ = m.parameters_for(d, "/r/line.csv", "map", "/odom")
    tracker, policy = m.nodes_for(d, "/r/line.csv", "map", "/odom")
    assert p["drive_topic"] == "/tracker/command"
    assert (p["lookahead_min_m"], p["lookahead_max_m"], p["lookahead_curvature_ref_1pm"]) == (
        1.0,
        1.2,
        1.0,
    )
    assert p["raceline_path"] == "/r/line.csv" and p["pose_frame"] == "map"
    assert (tracker.node_package, tracker.node_executable) == ("racer_control", "tracker_node")
    assert (policy.node_package, policy.node_executable) == ("racer_policy", "policy_node")


def test_policy_node_gets_the_same_raceline_pose_source_and_handoff_topic(tmp_path: Path) -> None:
    m = _module()
    d = _policy_dir(tmp_path)
    t, p = m.parameters_for(str(d), "/r/line.csv", "map", "/odom")
    assert p["policy_dir"] == str(d)
    for key in ("raceline_path", "pose_frame", "odom_topic"):
        assert p[key] == t[key]
    assert p["tracker_command_topic"] == t["drive_topic"] == m.TRACKER_HANDOFF_TOPIC


def test_neither_node_is_pointed_at_drive(tmp_path: Path) -> None:
    """Invariant 1: nothing in this launch names /drive, and the tracker's output is never
    /drive_raw (policy_node owns that topic in this deployment)."""
    m = _module()
    tracker, policy = m.parameters_for(str(_policy_dir(tmp_path)), "/r/l.csv", "", "/odom")
    for params in (tracker, policy):
        assert "/drive" not in [str(v) for v in params.values()]
    assert tracker["drive_topic"] != "/drive_raw"


def test_the_launch_shuts_down_with_policy_node(tmp_path: Path) -> None:
    from launch.actions import RegisterEventHandler
    from launch.launch_context import LaunchContext

    m = _module()
    d = _policy_dir(tmp_path)
    ld = m.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(
        {"policy_dir": str(d), "raceline": "/r/l.csv", "pose_frame": "map", "odom_topic": "/odom"}
    )
    actions = m._make_nodes(context)
    assert len(actions) == 3 and isinstance(actions[2], RegisterEventHandler)
    assert ld is not None


@pytest.mark.parametrize(
    ("policy_dir", "raceline", "match"),
    [("", "/r/l.csv", "policy_dir"), ("/x", "", "raceline")],
)
def test_refuses_to_build_without_a_policy_dir_or_raceline(policy_dir, raceline, match) -> None:
    with pytest.raises(RuntimeError, match=match):
        _module().nodes_for(policy_dir, raceline, "map", "/odom")


def test_refuses_a_directory_without_a_usable_sidecar(tmp_path: Path) -> None:
    m = _module()
    empty = tmp_path / "e"
    empty.mkdir()
    with pytest.raises(RuntimeError, match="policy_meta.yaml"):
        m.nodes_for(str(empty), "/r/l.csv", "map", "/odom")
    broken = _policy_dir(tmp_path, {"a": 1}, name="broken")
    with pytest.raises(RuntimeError, match="base_controller"):
        m.nodes_for(str(broken), "/r/l.csv", "map", "/odom")
