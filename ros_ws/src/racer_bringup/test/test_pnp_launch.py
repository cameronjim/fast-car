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
        "v_lap1_mps": "",
        "lap_mode": "v0",
        "lap_source": "start_scan",
        "lap_counter": "false",
        "require_lap_counter": "false",
        "shadow_perception_bundle": "",
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
        "v_lap1_mps": "",
        "lap_mode": "v0",
        "lap_source": "start_scan",
        "lap_counter": "true",
        "require_lap_counter": "false",
        "shadow_perception_bundle": "",
        "params_file": "",
    }
    assert declared["backend"].choices == ["net", "geometric"]
    assert declared["lap_mode"].choices == ["v0", "v1"]
    assert declared["lap_source"].choices == ["start_scan", "map_pose"]


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


def test_lap_memory_adds_the_node_with_the_cap_range_and_the_planner_fallback(
    monkeypatch, tmp_path
) -> None:
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    ctx = _context(
        lap_memory="true",
        dynamics_bundle=_bundle(tmp_path, 0.9),
        v_max_mps="0.8",
        v_lap1_mps="0.4",
        lap_mode="v1",
    )
    actions = m._make_actions(ctx)
    laps = [a for a in actions if getattr(a, "node_package", "") == "racer_laps"]
    assert len(laps) == 1 and laps[0].node_executable == "lap_memory_node"
    p = _flat_params(laps[0])
    assert float(p["v_cap_ceiling_mps"]) == 0.8 and float(p["v_cap_start_mps"]) == 0.4
    assert p["lap_mode"] == "v1" and p["lap_source"] == "start_scan"
    planner = next(a for a in actions if getattr(a, "node_package", "") == "racer_planner")
    q = _flat_params(planner)
    assert q["use_lap_guidance"] is True
    assert float(q["v_max_mps"]) == 0.8
    assert float(q["v_no_guidance_mps"]) == 0.4  # a dead lap node leaves lap 1's cap, not 0.8
    assert len([a for a in actions if isinstance(a, RegisterEventHandler)]) == 3


def test_v_lap1_defaults_to_v_max_so_a_launch_without_lap_memory_is_as_before(
    monkeypatch, tmp_path
) -> None:
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    plain = m._make_actions(_context(v_max_mps="0.7", v_lap1_mps="0.3"))
    planner = next(a for a in plain if getattr(a, "node_package", "") == "racer_planner")
    assert "v_no_guidance_mps" not in _flat_params(planner)  # no lap memory: untouched
    assert float(_flat_params(planner)["v_max_mps"]) == 0.7
    ctx = _context(lap_memory="true", dynamics_bundle=_bundle(tmp_path, 0.9), v_max_mps="0.7")
    actions = m._make_actions(ctx)
    laps = next(a for a in actions if getattr(a, "node_package", "") == "racer_laps")
    assert float(_flat_params(laps)["v_cap_start_mps"]) == 0.7  # empty v_lap1_mps: equal to v_max


def test_lap_memory_ceiling_is_the_lower_of_v_max_and_the_bundle_validity(tmp_path) -> None:
    m = _module()
    params = m.lap_memory_parameters(_bundle(tmp_path, 0.5), 0.8)
    assert params["v_cap_ceiling_mps"] == 0.5 and params["v_cap_start_mps"] == 0.5
    params = m.lap_memory_parameters(_bundle(tmp_path, 1.5), 0.8, 0.3)
    assert params["v_cap_ceiling_mps"] == 0.8 and params["v_cap_start_mps"] == 0.3


@pytest.mark.parametrize(
    ("kwargs", "match"),
    [
        ({"v_lap1_mps": 0.9, "v_max_mps": 0.8}, "v_lap1_mps"),
        ({"lap_mode": "v1", "lap_source": "map_pose"}, "start_scan"),
        ({"lap_mode": "v2"}, "lap_mode"),
        ({"lap_source": "gps"}, "lap_source"),
    ],
)
def test_lap_memory_refusals(tmp_path, kwargs, match) -> None:
    m = _module()
    args = {"v_max_mps": 0.8, "v_lap1_mps": None, "lap_mode": "v0", "lap_source": "start_scan"}
    args.update(kwargs)
    with pytest.raises(RuntimeError, match=match):
        m.lap_memory_parameters(_bundle(tmp_path, 1.5), **args)


def test_the_launch_refuses_v_lap1_above_v_max(monkeypatch, tmp_path) -> None:
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    ctx = _context(
        lap_memory="true", dynamics_bundle=_bundle(tmp_path, 1.5), v_max_mps="0.5", v_lap1_mps="0.6"
    )
    with pytest.raises(RuntimeError, match="v_lap1_mps"):
        m._make_actions(ctx)
    # without lap memory v_lap1_mps is unused and not checked
    assert m._make_actions(_context(v_max_mps="0.5", v_lap1_mps="0.6"))


def test_lap_memory_refuses_a_bundle_without_validity(monkeypatch, tmp_path) -> None:
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    (tmp_path / "manifest.yaml").write_text("io: {}\n")
    with pytest.raises(RuntimeError, match="validity"):
        m._make_actions(_context(lap_memory="true", dynamics_bundle=str(tmp_path)))
    with pytest.raises(RuntimeError, match="validity"):
        m._make_actions(_context(lap_memory="true", dynamics_bundle=str(tmp_path / "none")))


def test_the_planner_declares_the_parameter_the_launch_sets() -> None:
    """The launch hands the planner `v_no_guidance_mps` (the lap-1 cap used when the guidance is
    missing, stale or malformed). The planner must DECLARE it: brief 25 adds it. Until that is on
    main this test is red on purpose: with lap_memory:=true and no fallback, a dead lap node would
    release the ceiling."""
    path = _LAUNCH_DIR.parents[1] / "racer_planner" / "src" / "mppi_planner_node.cpp"
    assert re.search(r"""["']v_no_guidance_mps["']""", path.read_text()), (
        "mppi_planner_node.cpp does not declare v_no_guidance_mps"
    )


# -- the camera lap counter ----------------------------------------------------------------------


def _types():
    from launch.actions import ExecuteProcess, IncludeLaunchDescription, LogInfo, TimerAction

    return ExecuteProcess, IncludeLaunchDescription, LogInfo, TimerAction


def _shutdown_handlers(actions) -> list:
    return [
        h
        for h in actions
        if isinstance(h, RegisterEventHandler)
        and any(
            isinstance(x, EmitEvent)
            for x in getattr(h.event_handler, "_OnActionEventBase__actions_on_event", [])
        )
    ]


def test_the_lap_counter_is_started_and_reset_by_default_and_the_driver_is_untouched(
    monkeypatch,
) -> None:
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    monkeypatch.setattr(m, "lap_counter_in_graph", lambda *a, **k: False)
    execute, include, _log, timer = _types()
    actions = m._make_actions(_context(lap_counter="true"))
    includes = [a for a in actions if isinstance(a, include)]
    assert len(includes) == 1
    source = includes[0]._IncludeLaunchDescription__launch_description_source
    location = "".join(t.text for t in source._LaunchDescriptionSource__location)
    assert location.endswith("launch/lap_counter.launch.py")
    resets = [a for a in actions if type(a) is execute]
    assert len(resets) == 1
    cmd = [str(c[0].text) if isinstance(c, (list, tuple)) else str(c) for c in resets[0].cmd]
    assert cmd[0] == "timeout" and cmd[2:6] == ["ros2", "service", "call", "/lap_counter/reset"]
    assert "std_srvs/srv/Trigger" in cmd
    assert not [a for a in actions if isinstance(a, timer)]
    # the driving nodes are exactly as before, and only they shut the launch down
    nodes = [a for a in actions if hasattr(a, "node_package")]
    assert [n.node_package for n in nodes] == ["racer_track_perception", "racer_planner"]
    assert len(_shutdown_handlers(actions)) == 2


def test_a_lap_counter_already_in_the_graph_is_not_started_again_but_is_reset(monkeypatch) -> None:
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    monkeypatch.setattr(m, "lap_counter_in_graph", lambda *a, **k: True)
    execute, include, _log, _timer = _types()
    actions = m._make_actions(_context(lap_counter="true"))
    assert not [a for a in actions if isinstance(a, include)]
    assert len([a for a in actions if type(a) is execute]) == 1


def test_lap_counter_false_starts_and_resets_nothing(monkeypatch) -> None:
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)

    def boom(*a, **k):
        raise AssertionError("the graph was probed for a counter although lap_counter is false")

    monkeypatch.setattr(m, "lap_counter_in_graph", boom)
    execute, include, _log, _timer = _types()
    actions = m._make_actions(_context(lap_counter="false"))
    assert not [a for a in actions if isinstance(a, include) or type(a) is execute]


def test_a_missing_racer_perception_is_one_logged_error_and_the_driver_keeps_running(
    monkeypatch,
) -> None:
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    monkeypatch.setattr(m, "lap_counter_in_graph", lambda *a, **k: False)
    from ament_index_python import packages

    def missing(name):
        raise packages.PackageNotFoundError(name)

    monkeypatch.setattr(packages, "get_package_share_directory", missing)
    _execute, include, log, _timer = _types()
    actions = m._make_actions(_context(lap_counter="true"))
    assert not [a for a in actions if isinstance(a, include)]
    assert len([a for a in actions if isinstance(a, log)]) == 1
    assert len([a for a in actions if hasattr(a, "node_package")]) == 2
    assert len(_shutdown_handlers(actions)) == 2


def test_the_counter_exiting_with_code_3_logs_the_opencv_error_and_does_not_shut_down(
    monkeypatch,
) -> None:
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    monkeypatch.setattr(m, "lap_counter_in_graph", lambda *a, **k: False)
    _execute, _include, log, _timer = _types()
    actions = m._make_actions(_context(lap_counter="true"))
    handlers = [
        h
        for h in actions
        if isinstance(h, RegisterEventHandler)
        and getattr(h.event_handler, "_OnActionEventBase__action_matcher", None) is None
    ]
    assert len(handlers) == 1
    on_exit = handlers[0].event_handler._OnActionEventBase__on_event
    assert callable(on_exit)
    event = types.SimpleNamespace(process_name="lap_counter_node-3", returncode=3)
    out = on_exit(event, LaunchContext())
    assert len(out) == 1 and isinstance(out[0], log)
    assert not any(isinstance(x, EmitEvent) for x in out)
    text = "".join(getattr(t, "text", str(t)) for t in out[0].msg)
    assert "OpenCV" in text and "keeps running" in text
    other = types.SimpleNamespace(process_name="mppi_planner_node-2", returncode=1)
    assert on_exit(other, LaunchContext()) == []


def test_require_lap_counter_with_the_counter_off_refuses_when_none_is_in_the_graph(
    monkeypatch,
) -> None:
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    waited = []
    monkeypatch.setattr(m, "lap_counter_in_graph", lambda t=0.0: waited.append(t) or False)
    with pytest.raises(RuntimeError, match="no lap_counter_node"):
        m._make_actions(_context(lap_counter="false", require_lap_counter="true"))
    assert waited == [m.LAP_COUNTER_WAIT_S]  # it waited the five seconds
    monkeypatch.setattr(m, "lap_counter_in_graph", lambda t=0.0: True)
    assert m._make_actions(_context(lap_counter="false", require_lap_counter="true"))


def test_require_lap_counter_with_the_counter_on_checks_five_seconds_after_start(
    monkeypatch,
) -> None:
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    monkeypatch.setattr(m, "lap_counter_in_graph", lambda *a, **k: False)
    _execute, _include, log, timer = _types()
    actions = m._make_actions(_context(lap_counter="true", require_lap_counter="true"))
    timers = [a for a in actions if isinstance(a, timer)]
    assert len(timers) == 1 and float(timers[0].period) == m.LAP_COUNTER_WAIT_S
    # absent after the wait: one error and a shutdown; present: nothing
    out = m.require_counter_check(LaunchContext())
    assert any(isinstance(x, EmitEvent) and isinstance(x.event, Shutdown) for x in out)
    assert any(isinstance(x, log) for x in out)
    monkeypatch.setattr(m, "lap_counter_in_graph", lambda *a, **k: True)
    assert m.require_counter_check(LaunchContext()) == []
    # without require there is no timer
    plain = m._make_actions(_context(lap_counter="true", require_lap_counter="false"))
    assert not [a for a in plain if isinstance(a, timer)]


@pytest.mark.parametrize(
    "topic", ["/lap/event", "/lap/status", "/pnp/lap/event", "/pnp/shadow/local_track"]
)
def test_recorder_regex_takes_the_lap_and_shadow_topics(topic) -> None:
    regex = _load("car_teleop.launch.py")._RECORDED_TOPIC_REGEX
    assert re.search(regex, topic), topic


# -- the shadow perception node ------------------------------------------------------------------


def test_shadow_perception_is_a_second_node_with_remapped_outputs_that_does_not_stop_the_launch(
    monkeypatch,
) -> None:
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    actions = m._make_actions(_context(shadow_perception_bundle="/b/shadow", backend="geometric"))
    nodes = [a for a in actions if hasattr(a, "node_package")]
    assert [n.node_package for n in nodes] == [
        "racer_track_perception",
        "racer_planner",
        "racer_track_perception",
    ]
    shadow = nodes[2]
    assert shadow.node_executable == "track_perception_node"
    assert "track_perception_shadow" in str(shadow._Node__node_name)
    p = _flat_params(shadow)
    assert p["backend"] == "net" and p[m.PERCEPTION_BUNDLE_PARAM] == "/b/shadow"
    remaps = [
        ("".join(t.text for t in a[0]), "".join(t.text for t in a[1]))
        for a in shadow._Node__remappings
    ]
    assert remaps == [
        ("/pnp/local_track", "/pnp/shadow/local_track"),
        ("/pnp/perception/status", "/pnp/shadow/perception/status"),
    ]
    # the driving perception node is the geometric one and is NOT remapped
    assert _flat_params(nodes[0])["backend"] == "geometric"
    assert nodes[0]._Node__remappings is None or list(nodes[0]._Node__remappings) == []
    # the shadow's exit logs and does not shut the launch down; the others still do
    assert len(_shutdown_handlers(actions)) == 2
    watching_shadow = [
        h
        for h in actions
        if isinstance(h, RegisterEventHandler)
        and h.event_handler._OnActionEventBase__action_matcher == shadow
    ]
    assert len(watching_shadow) == 1
    assert callable(watching_shadow[0].event_handler._OnActionEventBase__on_event)


def test_no_shadow_node_by_default(monkeypatch) -> None:
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    nodes = [a for a in m._make_actions(_context()) if hasattr(a, "node_package")]
    assert len(nodes) == 2


def test_nothing_in_the_planner_the_lap_memory_or_safety_reads_a_shadow_topic() -> None:
    """The shadow outputs are recorded, never read: no mention of /pnp/shadow/ in the planner,
    the lap memory or the safety node."""
    src = _LAUNCH_DIR.parents[1]
    for rel in (
        "racer_planner/src/mppi_planner_node.cpp",
        "racer_laps/src/racer_laps/lap_memory_node.py",
        "racer_safety/src/safety_node.cpp",
    ):
        path = src / rel
        assert path.is_file(), path
        assert "/pnp/shadow" not in path.read_text(), rel
