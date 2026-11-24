"""L1 check of brief 52 (rt_sched) in car_teleop.launch.py and pnp.launch.py: launch-description
inspection and fixture text, no process started.

What it pins:
  * the launch defaults tables: every car_teleop argument and its default (rt_sched added, default
    off), the node list, the safety_node prefix and parameters;
  * rt_sched off is today's launch: no prefix on either node (the global launch-prefix is kept),
    no entity added to pnp's action list, car_teleop gets one inert OpaqueFunction;
  * rt_sched on adds exactly the two prefixes with the values of config/scheduling.yaml (read, not
    typed here) and the handlers that check and refuse;
  * a bad rt_sched value is refused;
  * the /proc reader and the thread check on fixture text.
"""

from __future__ import annotations

import importlib.util
import shlex
from pathlib import Path

import pytest

pytest.importorskip("launch_ros")

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

_LAUNCH_DIR = Path(__file__).resolve().parents[1] / "launch"
_REPO = Path(__file__).resolve().parents[4]


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _car():
    return _load(_LAUNCH_DIR / "car_teleop.launch.py", "car_teleop_rt_test")


def _pnp():
    return _load(_LAUNCH_DIR / "pnp.launch.py", "pnp_rt_test")


def _rt():
    return _load(_LAUNCH_DIR / "rt_sched.py", "rt_sched_test")


def _sched():
    return _load(_REPO / "config" / "scheduling.py", "scheduling_rt_test").load()


def _ctx(**configs: str) -> LaunchContext:
    context = LaunchContext()
    context.launch_configurations.update(configs)
    return context


def _prefix(node) -> str:
    return perform_substitutions(_ctx(), node._ExecuteLocal__process_description.prefix)


def _prefix_with(node, **configs: str) -> str:
    return perform_substitutions(_ctx(**configs), node._ExecuteLocal__process_description.prefix)


def _entities():
    return _car().generate_launch_description().entities


def _safety(entities):
    (node,) = [e for e in entities if isinstance(e, Node) and e.node_executable == "safety_node"]
    return node


# -- car_teleop: the defaults table --------------------------------------------------------------

CAR_DEFAULTS = {
    "start_teleop": "false",
    "browser_teleop": "false",
    "teleop_cmd_vel_topic": "/teleop/cmd_vel",
    "twist_timeout_s": "0.5",
    "allow_reverse": "false",
    "teleop_speed_step_mps": "0.18",
    "teleop_min_speed_mps": "0.57",
    "viz": "true",
    "sysfs_root": "/sys/class/pwm",
    "steering_pwmchip": "0",
    "steering_pwm_channel": "0",
    "throttle_pwmchip": "2",
    "throttle_pwm_channel": "0",
    "drive_timeout_s": "0.1",
    "safety_cycle_record": "true",
    "rt_sched": "off",
    "record": "true",
    "bag_dir": "/workspace/data/bags",
    "bag_storage": "auto",
    "rail_voltage": "true",
    "ina3221_root": "/sys/bus/i2c/drivers/ina3221",
    "lidar": "false",
    "lidar_serial_port": "/dev/ttyUSB0",
    "vesc": "false",
    "vesc_serial_port": "/dev/ttyTHS1",
    "vesc_imu": "false",
    "cameras": "false",
    "camera_usb_device": "/dev/video0",
    "mapping": "false",
    "localization": "false",
    "map": "",
}


def test_car_teleop_argument_defaults_table() -> None:
    declared = {e.name: e for e in _entities() if isinstance(e, DeclareLaunchArgument)}
    defaults = {n: perform_substitutions(_ctx(), a.default_value) for n, a in declared.items()}
    assert defaults == CAR_DEFAULTS
    assert declared["rt_sched"].choices == ["off", "on"]


def test_car_teleop_nodes_are_the_same_with_rt_sched_added() -> None:
    entities = _entities()
    nodes = [
        (e.node_package, e.node_executable, e._Node__node_name)
        for e in entities
        if isinstance(e, Node)
    ]
    assert nodes == [
        ("racer_drivers", "rail_voltage_node", "rail_voltage_node"),
        ("racer_safety", "safety_node", "safety_node"),
        ("racer_drivers", "pwm_output_node", "pwm_output_node"),
        ("racer_tools", "keyboard_teleop_node", "keyboard_teleop"),
        ("racer_tools", "twist_teleop_adapter_node", "twist_teleop_adapter"),
        ("foxglove_bridge", "foxglove_bridge", "foxglove_bridge"),
    ]
    # one inert OpaqueFunction was added, and nothing else, for rt_sched
    assert len(entities) == 43
    params = _safety(entities)._Node__parameters
    assert len(params) == 1
    assert [perform_substitutions(_ctx(), k) for k in params[0]] == ["publish_cycle_record"]


def test_off_leaves_safety_node_without_a_prefix_and_adds_no_handlers() -> None:
    entities = _entities()
    node = _safety(entities)
    for context in (_ctx(), _ctx(rt_sched="off")):
        assert perform_substitutions(context, node._ExecuteLocal__process_description.prefix) == ""
    (expand,) = [
        e
        for e in entities
        if isinstance(e, OpaqueFunction) and e._OpaqueFunction__function.__name__ == "expand"
    ]
    assert expand.execute(_ctx(rt_sched="off")) in (None, [])


def test_off_keeps_a_global_launch_prefix_as_with_no_prefix_at_all() -> None:
    node = _safety(_entities())
    assert _prefix_with(node, **{"launch-prefix": "nice -n 5", "rt_sched": "off"}) == "nice -n 5"


def test_on_prefixes_safety_node_with_the_yaml_value_and_adds_the_handlers() -> None:
    entities = _entities()
    node = _safety(entities)
    sched = _sched()
    assert shlex.split(_prefix_with(node, rt_sched="on")) == [
        "chrt",
        "-f",
        str(sched.safety_priority),
    ]
    (expand,) = [
        e
        for e in entities
        if isinstance(e, OpaqueFunction) and e._OpaqueFunction__function.__name__ == "expand"
    ]
    added = expand._OpaqueFunction__function(_ctx(rt_sched="on"))
    assert len(added) == 2 and all(isinstance(a, RegisterEventHandler) for a in added)
    # before the node, so the start handler exists when the node starts
    assert entities.index(expand) < entities.index(node)


def test_a_bad_rt_sched_value_is_refused() -> None:
    node = _safety(_entities())
    with pytest.raises(RuntimeError, match="rt_sched must be one of"):
        _prefix_with(node, rt_sched="maybe")
    with pytest.raises(RuntimeError, match="rt_sched must be one of"):
        _rt().check_mode("")


# -- pnp: nothing changes with off, the prefix with on -------------------------------------------


def test_pnp_nodes_for_without_a_prefix_is_as_before() -> None:
    perception, planner = _pnp().nodes_for({}, {}, "")
    for node in (perception, planner):
        assert perform_substitutions(_ctx(), node._ExecuteLocal__process_description.prefix) == ""


def test_pnp_nodes_for_with_the_planner_prefix() -> None:
    rt = _rt()
    perception, planner = _pnp().nodes_for({}, {}, "", None, rt.prefix_for("mppi_planner_node"))
    sched = _sched()
    assert shlex.split(_prefix_with(planner, rt_sched="on")) == [
        "chrt",
        "-f",
        str(sched.planner_priority),
    ]
    # the perception node is never raised
    assert _prefix_with(perception, rt_sched="on") == ""


def _pnp_context(**overrides: str) -> LaunchContext:
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
        "lap_length_ratio_min": "0.0",
        "lap_length_ratio_max": "0.0",
        "lap_counter": "false",
        "lap_rate_hz": "15.0",
        "require_lap_counter": "false",
        "deskew": "false",
        "shadow_perception_bundle": "",
        "params_file": "",
    }
    configs.update(overrides)
    return _ctx(**configs)


def _pnp_actions(**overrides: str) -> list:
    module = _pnp()
    module.recorder_in_graph = lambda *a, **k: True
    return module._make_actions(_pnp_context(**overrides))


def test_pnp_off_adds_nothing_to_the_action_list() -> None:
    unset = _pnp_actions()
    off = _pnp_actions(rt_sched="off")
    assert [type(a) for a in off] == [type(a) for a in unset]
    planner = [a for a in off if isinstance(a, Node) and a.node_executable == "mppi_planner_node"]
    assert len(planner) == 1 and _prefix(planner[0]) == ""
    # perception, planner, two shutdown handlers
    assert [type(a).__name__ for a in off] == [
        "Node",
        "Node",
        "RegisterEventHandler",
        "RegisterEventHandler",
    ]


def test_pnp_on_prefixes_the_planner_only_and_adds_the_handlers_first() -> None:
    on = _pnp_actions(rt_sched="on")
    assert len(on) == 6
    assert [type(a).__name__ for a in on[:2]] == ["RegisterEventHandler"] * 2
    nodes = [a for a in on if isinstance(a, Node)]
    by_exec = {n.node_executable: n for n in nodes}
    assert shlex.split(_prefix_with(by_exec["mppi_planner_node"], rt_sched="on")) == [
        "chrt",
        "-f",
        str(_sched().planner_priority),
    ]
    assert _prefix_with(by_exec["track_perception_node"], rt_sched="on") == ""


def test_pnp_refuses_a_bad_rt_sched_value() -> None:
    with pytest.raises(RuntimeError, match="rt_sched must be one of"):
        _pnp_actions(rt_sched="maybe")


# -- the /proc reader and the thread check, on fixture text ----------------------------------------


#: /proc/<pid>/task/<tid>/stat: field 41 is policy, 40 is rt_priority (proc(5)); comm has a space
#: and a parenthesis to show the parse counts from the LAST ")".
def _stat(tid: int, comm: str, policy: int, prio: int) -> str:
    after = ["S", "1"] + ["0"] * 35 + [str(prio), str(policy)] + ["0"] * 11
    return f"{tid} ({comm}) " + " ".join(after) + "\n"


def test_parse_stat_reads_policy_and_priority_after_an_awkward_comm() -> None:
    assert _rt().parse_stat(_stat(7, "a b) c", 1, 50)) == (7, 1, 50, "a b) c")
    assert _rt().parse_stat(_stat(8, "safety_node", 0, 0)) == (8, 0, 0, "safety_node")


def _proc(tmp_path: Path, pid: int, threads: list[tuple[int, int, int]]) -> str:
    for tid, policy, prio in threads:
        d = tmp_path / str(pid) / "task" / str(tid)
        d.mkdir(parents=True)
        (d / "stat").write_text(_stat(tid, "t", policy, prio))
    return str(tmp_path)


def test_all_fifo_threads_pass(tmp_path) -> None:
    rt = _rt()
    root = _proc(tmp_path, 10, [(10, 1, 50), (11, 1, 50), (12, 1, 50)])
    ok, line = rt.evaluate("safety_node", 10, rt.read_threads(10, root), 50, "on")
    assert ok and "3 threads" in line and "all FIFO 50" in line


def test_one_other_thread_fails_with_on_and_is_only_logged_with_off(tmp_path) -> None:
    rt = _rt()
    root = _proc(tmp_path, 10, [(10, 1, 50), (11, 0, 0)])
    threads = rt.read_threads(10, root)
    ok, line = rt.evaluate("safety_node", 10, threads, 50, "on")
    assert not ok and "not FIFO 50: 11:OTHER/0" in line
    ok, line = rt.evaluate("safety_node", 10, threads, 50, "off")
    assert ok and line.startswith("rt_sched off:")


def test_wrong_priority_fails(tmp_path) -> None:
    rt = _rt()
    root = _proc(tmp_path, 10, [(10, 1, 40)])
    ok, line = rt.evaluate("safety_node", 10, rt.read_threads(10, root), 50, "on")
    assert not ok and "10:FIFO/40" in line


def test_a_process_that_is_gone(tmp_path) -> None:
    rt = _rt()
    assert rt.read_threads(99, str(tmp_path)) is None
    assert rt.evaluate("planner", 99, None, 40, "on")[0] is False
    assert rt.evaluate("planner", 99, None, 40, "off")[0] is True


class _Exit:
    def __init__(self, returncode: int) -> None:
        self.returncode = returncode


def test_an_early_non_zero_exit_is_the_registered_refusal_and_raises() -> None:
    rt = _rt()
    on_exit = rt.exit_callback("safety_node", {"at": __import__("time").monotonic()})
    with pytest.raises(RuntimeError) as exc:
        on_exit(_Exit(1), _ctx(rt_sched="on"))
    assert str(exc.value).startswith(rt.refusal_prefix())
    # never started: also a refusal
    with pytest.raises(RuntimeError, match="refusing to start"):
        rt.exit_callback("safety_node", {})(_Exit(1), _ctx(rt_sched="on"))


def test_a_late_or_clean_exit_shuts_down_and_off_does_nothing() -> None:
    rt = _rt()
    long_ago = {"at": __import__("time").monotonic() - 3600}
    out = rt.exit_callback("safety_node", long_ago)(_Exit(1), _ctx(rt_sched="on"))
    assert [type(a).__name__ for a in out] == ["EmitEvent"]
    out = rt.exit_callback("safety_node", {"at": 0.0})(_Exit(0), _ctx(rt_sched="on"))
    assert [type(a).__name__ for a in out] == ["EmitEvent"]
    # the pnp planner: its own handler shuts down, this one only logs
    assert rt.exit_callback("p", long_ago, emit_shutdown=False)(_Exit(1), _ctx(rt_sched="on")) == []
    assert rt.exit_callback("safety_node", {})(_Exit(1), _ctx(rt_sched="off")) == []
