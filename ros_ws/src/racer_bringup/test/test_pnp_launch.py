"""L1 check of pnp.launch.py (launch-description inspection, no processes started): argument
defaults, exactly the two driver nodes, parameters forwarded, the recorder guard on by default
and off with require_recorder:=false, a bundle refusal before anything starts, shutdown with
either node, and no /drive remap or mention. Also the recorder regex's /pnp/ alternative."""

from __future__ import annotations

import importlib.util
import re
import sys
import types
from pathlib import Path

import pytest

pytest.importorskip("launch_ros")

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import Shutdown
from launch.launch_context import LaunchContext
from launch.utilities import perform_substitutions

_LAUNCH_DIR = Path(__file__).resolve().parents[1] / "launch"


def _load(name: str):
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), _LAUNCH_DIR / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _module():
    return _load("pnp.launch.py")


def _context(**overrides: str) -> LaunchContext:
    configs = {
        "perception_bundle": "/b/trackperc",
        "dynamics_bundle": "/b/dyn",
        "backend": "net",
        "v_max_mps": "0.5",
        "require_recorder": "true",
        "laser_yaw_from_vehicle_params": "true",
        "lap_memory": "false",
        "params_file": "",
    }
    configs.update(overrides)
    context = LaunchContext()
    context.launch_configurations.update(configs)
    return context


def _node_params(node) -> list:
    return list(node._Node__parameters)


def _perform(value):
    if isinstance(value, (tuple, list)):
        # launch_ros turns a string parameter into a YAML scalar document ("net\n...\n").
        text = perform_substitutions(LaunchContext(), list(value))
        return text.split("\n", 1)[0]
    return value


def _flat_params(node) -> dict:
    merged: dict = {}
    for entry in _node_params(node):
        if isinstance(entry, dict):
            merged.update({_perform(k): _perform(v) for k, v in entry.items()})
    return merged


def _params_file(node) -> str:
    first = _node_params(node)[0]
    return str(first.param_file[0].text)


def test_argument_defaults() -> None:
    declared = {
        e.name: e
        for e in _module().generate_launch_description().entities
        if isinstance(e, DeclareLaunchArgument)
    }
    defaults = {name: "".join(t.text for t in arg.default_value) for name, arg in declared.items()}
    assert defaults == {
        "perception_bundle": "",
        "dynamics_bundle": "",
        "backend": "geometric",
        "v_max_mps": "0.5",
        "require_recorder": "true",
        "laser_yaw_from_vehicle_params": "true",
        "lap_memory": "false",
        "params_file": "",
    }
    assert declared["backend"].choices == ["net", "geometric"]


def test_launch_description_builds() -> None:
    assert isinstance(_module().generate_launch_description(), LaunchDescription)


def test_starts_exactly_the_two_driver_nodes(monkeypatch) -> None:
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    from launch_ros.actions import Node

    nodes = [a for a in m._make_actions(_context()) if isinstance(a, Node)]
    assert [(n.node_package, n.node_executable) for n in nodes] == [
        ("racer_track_perception", "track_perception_node"),
        ("racer_planner", "mppi_planner_node"),
    ]


def test_parameters_are_forwarded(monkeypatch) -> None:
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    ctx = _context(v_max_mps="0.4", laser_yaw_from_vehicle_params="false", params_file="/p.yaml")
    perception, planner = (a for a in m._make_actions(ctx) if hasattr(a, "node_package"))
    p, q = _flat_params(perception), _flat_params(planner)
    assert p["backend"] == "net" and p[m.PERCEPTION_BUNDLE_PARAM] == "/b/trackperc"
    assert q[m.DYNAMICS_BUNDLE_PARAM] == "/b/dyn" and float(q["v_max_mps"]) == 0.4
    assert p["laser_yaw_from_vehicle_params"] is False
    assert q["laser_yaw_from_vehicle_params"] is False
    for node in (perception, planner):
        assert _params_file(node) == "/p.yaml"


def test_geometric_backend_needs_no_perception_bundle() -> None:
    m = _module()
    p, q = m.parameters_for("", "/b/dyn", "geometric", 0.5, True)
    assert p["backend"] == "geometric" and m.PERCEPTION_BUNDLE_PARAM not in p
    assert q["v_max_mps"] == 0.5


@pytest.mark.parametrize(
    ("perception", "dynamics", "backend", "match"),
    [
        ("", "/b/dyn", "net", "perception_bundle"),
        ("/b/p", "", "net", "dynamics_bundle"),
        ("", "", "geometric", "dynamics_bundle"),
        ("/b/p", "/b/dyn", "teleport", "backend"),
    ],
)
def test_refuses_to_build_on_a_bad_bundle_or_backend(perception, dynamics, backend, match) -> None:
    with pytest.raises(RuntimeError, match=match):
        _module().parameters_for(perception, dynamics, backend, 0.5, True)


def test_recorder_guard_runs_by_default_and_refuses(monkeypatch) -> None:
    m = _module()
    calls = []
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: calls.append(1) or False)
    with pytest.raises(RuntimeError, match="no rosbag recorder in the graph"):
        m._make_actions(_context())
    assert calls == [1]


def test_recorder_guard_passes_when_a_recorder_is_there(monkeypatch) -> None:
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    assert len(m._make_actions(_context())) == 4


def test_recorder_guard_is_absent_with_require_recorder_false(monkeypatch) -> None:
    m = _module()

    def boom(*a, **k):
        raise AssertionError("recorder checked although require_recorder is false")

    monkeypatch.setattr(m, "recorder_in_graph", boom)
    assert len(m._make_actions(_context(require_recorder="false"))) == 4


def test_bad_boolean_argument_is_refused() -> None:
    with pytest.raises(RuntimeError, match="require_recorder"):
        _module()._make_actions(_context(require_recorder="maybe"))


def test_either_node_exiting_shuts_the_launch_down(monkeypatch) -> None:
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    actions = m._make_actions(_context())
    handlers = [a for a in actions if isinstance(a, RegisterEventHandler)]
    nodes = [a for a in actions if hasattr(a, "node_package")]
    assert len(handlers) == 2
    watched = [h.event_handler._OnActionEventBase__action_matcher for h in handlers]
    assert watched == nodes
    for handler in handlers:
        (action,) = handler.event_handler._OnActionEventBase__actions_on_event
        assert isinstance(action, EmitEvent) and isinstance(action.event, Shutdown)


def test_nothing_names_or_remaps_drive(monkeypatch) -> None:
    """Invariant 1: the only command topic is /drive_raw, owned by the planner node."""
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    for node in (a for a in m._make_actions(_context()) if hasattr(a, "node_package")):
        assert node._Node__remappings is None or list(node._Node__remappings) == []
        assert "/drive" not in [str(v) for v in _flat_params(node).values()]
    source = (_LAUNCH_DIR / "pnp.launch.py").read_text()
    assert not re.search(r"""["']/drive["']""", source)


class _FakeNode:
    def __init__(self, names: list[str]) -> None:
        self._names = names

    def get_node_names(self) -> list[str]:
        return self._names

    def destroy_node(self) -> None:
        pass


def _fake_rclpy(names: list[str]) -> types.ModuleType:
    fake = types.ModuleType("rclpy")
    fake.Context = lambda: object()
    fake.init = lambda context=None: None
    fake.shutdown = lambda context=None: None
    fake.create_node = lambda name, context=None: _FakeNode(names)
    return fake


def test_recorder_in_graph_finds_the_real_recorder_node_name(monkeypatch) -> None:
    monkeypatch.setitem(sys.modules, "rclpy", _fake_rclpy(["a", "rosbag2_recorder"]))
    assert _module().recorder_in_graph(0.2) is True


def test_recorder_in_graph_times_out_without_one(monkeypatch) -> None:
    monkeypatch.setitem(sys.modules, "rclpy", _fake_rclpy(["a", "safety_node"]))
    assert _module().recorder_in_graph(0.2) is False


@pytest.mark.parametrize("topic", ["/pnp/local_track", "/pnp/planner/status", "/pnp/plan"])
def test_recorder_regex_takes_the_pnp_topics(topic) -> None:
    regex = _load("car_teleop.launch.py")._RECORDED_TOPIC_REGEX
    assert re.search(regex, topic), topic


@pytest.mark.parametrize("topic", ["/pnpx", "/pnp", "/xpnp/plan"])
def test_recorder_regex_leaves_out_lookalikes(topic) -> None:
    regex = _load("car_teleop.launch.py")._RECORDED_TOPIC_REGEX
    assert re.search(regex, topic) is None, topic


def _bundle(tmp_path: Path, v_max: float = 0.9) -> str:
    (tmp_path / "manifest.yaml").write_text(
        f"io:\n  validity: {{v_min_mps: 0.0, v_max_mps: {v_max}}}\n"
    )
    return str(tmp_path)


def test_lap_memory_is_off_by_default(monkeypatch) -> None:
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    actions = m._make_actions(_context())
    assert not [a for a in actions if getattr(a, "node_package", "") == "racer_laps"]
    planner = next(a for a in actions if getattr(a, "node_package", "") == "racer_planner")
    assert "use_lap_guidance" not in _flat_params(planner)


def test_lap_memory_adds_the_node_with_the_bundle_validity_as_ceiling(
    monkeypatch, tmp_path
) -> None:
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    ctx = _context(lap_memory="true", dynamics_bundle=_bundle(tmp_path, 0.9), v_max_mps="0.4")
    actions = m._make_actions(ctx)
    laps = [a for a in actions if getattr(a, "node_package", "") == "racer_laps"]
    assert len(laps) == 1 and laps[0].node_executable == "lap_memory_node"
    p = _flat_params(laps[0])
    assert float(p["v_cap_ceiling_mps"]) == 0.9 and float(p["v_cap_start_mps"]) == 0.4
    planner = next(a for a in actions if getattr(a, "node_package", "") == "racer_planner")
    assert _flat_params(planner)["use_lap_guidance"] is True
    assert len([a for a in actions if isinstance(a, RegisterEventHandler)]) == 3


def test_lap_memory_start_cap_never_exceeds_the_ceiling(tmp_path) -> None:
    params = _module().lap_memory_parameters(_bundle(tmp_path, 0.5), 0.8)
    assert params == {"v_cap_ceiling_mps": 0.5, "v_cap_start_mps": 0.5}


def test_lap_memory_refuses_a_bundle_without_validity(monkeypatch, tmp_path) -> None:
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    (tmp_path / "manifest.yaml").write_text("io: {}\n")
    with pytest.raises(RuntimeError, match="validity"):
        m._make_actions(_context(lap_memory="true", dynamics_bundle=str(tmp_path)))
    with pytest.raises(RuntimeError, match="validity"):
        m._make_actions(_context(lap_memory="true", dynamics_bundle=str(tmp_path / "none")))
