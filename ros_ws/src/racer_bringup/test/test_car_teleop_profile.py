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
    assert _load_launch_module().FIRST_DRIVE_SPEED_STEP_MPS == 0.25


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
    context.launch_configurations["teleop_speed_step_mps"] = "0.25"
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
    assert value == 0.25
