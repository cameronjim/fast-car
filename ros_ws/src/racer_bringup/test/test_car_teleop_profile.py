"""L1 check of car_teleop.launch.py's first-drive keyboard teleop profile (GitHub issue #72).

Loads the SOURCE launch file as a module and inspects the launch description it returns; no
process is started and no ROS graph is needed. What it pins: the `teleop_speed_step_mps`
argument exists and defaults to the documented FIRST_DRIVE_SPEED_STEP_MPS, and the keyboard
node is handed that argument as its `speed_step_mps` parameter (a float), so the profile is
not just a comment. The resulting pulse per tap is checked in racer_drivers' gtests.
"""

from __future__ import annotations

import importlib.util
import pathlib

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

_LAUNCH_FILE = pathlib.Path(__file__).resolve().parents[1] / "launch" / "car_teleop.launch.py"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("car_teleop_launch", _LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _entities():
    return _load_launch_module().generate_launch_description().entities


def test_first_drive_speed_step_is_the_documented_value():
    assert _load_launch_module().FIRST_DRIVE_SPEED_STEP_MPS == 0.18


def test_speed_step_argument_defaults_to_the_first_drive_profile():
    module = _load_launch_module()
    args = {e.name: e for e in _entities() if isinstance(e, DeclareLaunchArgument)}
    assert "teleop_speed_step_mps" in args
    context = LaunchContext()
    default = "".join(s.perform(context) for s in args["teleop_speed_step_mps"].default_value)
    assert float(default) == module.FIRST_DRIVE_SPEED_STEP_MPS


def test_keyboard_node_receives_the_speed_step_as_a_float_parameter():
    keyboard = [
        e
        for e in _entities()
        if isinstance(e, Node) and e.node_executable == "keyboard_teleop_node"
    ]
    assert len(keyboard) == 1
    context = LaunchContext()
    context.launch_configurations["teleop_speed_step_mps"] = "0.18"
    context.launch_configurations["allow_reverse"] = "false"
    # launch_ros keeps the raw parameter dicts until the node is executed; evaluate the one
    # entry this test is about the way launch_ros will.
    params = keyboard[0]._Node__parameters
    merged = {}
    for entry in params:
        for key, value in entry.items():
            name = "".join(k.perform(context) for k in key) if isinstance(key, tuple) else key
            merged[name] = value
    assert "speed_step_mps" in merged
    value = merged["speed_step_mps"].evaluate(context)
    assert isinstance(value, float)
    assert value == 0.18


def test_first_drive_min_speed_is_the_documented_value():
    assert _load_launch_module().FIRST_DRIVE_MIN_SPEED_MPS == 0.57


def test_min_speed_argument_defaults_to_the_first_drive_profile():
    module = _load_launch_module()
    args = {e.name: e for e in _entities() if isinstance(e, DeclareLaunchArgument)}
    assert "teleop_min_speed_mps" in args
    context = LaunchContext()
    default = "".join(s.perform(context) for s in args["teleop_min_speed_mps"].default_value)
    assert float(default) == module.FIRST_DRIVE_MIN_SPEED_MPS


def test_keyboard_node_receives_the_min_speed_as_a_float_parameter():
    keyboard = [
        e
        for e in _entities()
        if isinstance(e, Node) and e.node_executable == "keyboard_teleop_node"
    ]
    assert len(keyboard) == 1
    context = LaunchContext()
    context.launch_configurations["teleop_min_speed_mps"] = "0.57"
    merged = {}
    for entry in keyboard[0]._Node__parameters:
        for key, value in entry.items():
            name = "".join(k.perform(context) for k in key) if isinstance(key, tuple) else key
            merged[name] = value
    assert "min_speed_mps" in merged
    value = merged["min_speed_mps"].evaluate(context)
    assert isinstance(value, float)
    assert value == 0.57


def _safety_params(context):
    safety = [e for e in _entities() if isinstance(e, Node) and e.node_executable == "safety_node"]
    assert len(safety) == 1
    merged = {}
    for entry in safety[0]._Node__parameters:
        for key, value in entry.items():
            name = "".join(k.perform(context) for k in key) if isinstance(key, tuple) else key
            merged[name] = value
    return merged


def test_safety_cycle_record_argument_defaults_true():
    args = {e.name: e for e in _entities() if isinstance(e, DeclareLaunchArgument)}
    assert "safety_cycle_record" in args
    default = "".join(s.perform(LaunchContext()) for s in args["safety_cycle_record"].default_value)
    assert default == "true"


def test_safety_node_gets_publish_cycle_record_at_construction_as_a_bool():
    for text, expected in (("true", True), ("false", False)):
        context = LaunchContext()
        context.launch_configurations["safety_cycle_record"] = text
        merged = _safety_params(context)
        assert "publish_cycle_record" in merged
        value = merged["publish_cycle_record"].evaluate(context)
        assert isinstance(value, bool)
        assert value is expected


def test_recorder_topic_list_includes_safety_cycle_and_keeps_the_rest():
    import re

    regex = re.compile(_load_launch_module()._RECORDED_TOPIC_REGEX)
    for topic in ("/safety/cycle", "/safety/events", "/drive", "/drive_raw", "/pnp/plan"):
        assert regex.match(topic), topic
    assert not regex.match("/safety/cycle/extra")


def test_recorder_regex_takes_the_lap_similarity_and_thumb_topics_and_stays_anchored():
    import re

    regex = re.compile(_load_launch_module()._RECORDED_TOPIC_REGEX)
    for topic in ("/lap/event", "/lap/status", "/lap/similarity", "/lap/thumb"):
        assert regex.match(topic), topic
    for topic in ("/lap/thumb/extra", "/lap/other", "/lap/similarity_x"):
        assert not regex.match(topic), topic


def test_recorder_gets_time_to_close_a_large_bag_before_launch_escalates(tmp_path):
    """sigterm/sigkill of 10 s killed the recorder before a multi-GB bag was finalised (no
    metadata.yaml). They must be at least the racer app's stop wait, stop_timeout_stack_s."""
    module = _load_launch_module()
    assert module._RECORDER_CLOSE_TIMEOUT_S >= 600
    context = LaunchContext()
    context.launch_configurations["bag_dir"] = str(tmp_path)
    context.launch_configurations["bag_storage"] = "sqlite3"
    actions = module._bag_actions(context)
    recorder = next(a for a in actions if type(a).__name__ == "ExecuteProcess")
    for attr in ("_ExecuteLocal__sigterm_timeout", "_ExecuteLocal__sigkill_timeout"):
        value = "".join(s.perform(context) for s in getattr(recorder, attr))
        assert float(value) == module._RECORDER_CLOSE_TIMEOUT_S, attr
