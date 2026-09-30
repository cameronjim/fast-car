"""rclpy-level tests for keyboard_teleop_node's tap-size parameters (GitHub issue #72).

The step/clamp arithmetic itself is L1-tested in test_keymap.py; this pins the plumbing: the
two ROS parameters exist, default to the vehicle_params derivation, and an override reaches
the config the key handler uses. Same pytest.importorskip-guarded shape as
test_keyboard_teleop_node_main.py: skipped under the bare `uv run pytest` L1 run (no rclpy),
runs for real under `colcon test` in the ros-dev image. The constructor needs no TTY.
"""

from __future__ import annotations

import pytest

pytest.importorskip("rclpy")

import rclpy
from racer_tools.keyboard_teleop_node import KeyboardTeleopNode
from racer_tools.keymap import derived_steps
from racer_tools.vehicle_params_loader import load_vehicle_params


def _make_node(ros_args: list[str]) -> KeyboardTeleopNode:
    if rclpy.ok():
        rclpy.shutdown()
    rclpy.init(args=["--ros-args", *ros_args] if ros_args else [])
    return KeyboardTeleopNode()


def _teardown(node: KeyboardTeleopNode) -> None:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_steps_default_to_the_vehicle_params_derivation():
    node = _make_node([])
    try:
        steering_step, speed_step = derived_steps(load_vehicle_params(), 50.0)
        assert node.get_parameter("speed_step_mps").value == pytest.approx(speed_step)
        assert node.get_parameter("steering_step_rad").value == pytest.approx(steering_step)
        assert node._config.speed_step_mps == pytest.approx(speed_step)
        assert node._config.steering_step_rad == pytest.approx(steering_step)
    finally:
        _teardown(node)


def test_overrides_reach_the_key_handler():
    node = _make_node(["-p", "speed_step_mps:=0.25", "-p", "steering_step_rad:=0.1"])
    try:
        assert node._config.speed_step_mps == 0.25
        assert node._config.steering_step_rad == 0.1
        node.handle_raw_input("w")
        assert node._state.speed_mps == 0.25
        node.handle_raw_input("a")
        assert node._state.steering_angle_rad == 0.1
    finally:
        _teardown(node)


def test_a_non_positive_override_refuses_to_start():
    if rclpy.ok():
        rclpy.shutdown()
    rclpy.init(args=["--ros-args", "-p", "speed_step_mps:=0.0"])
    try:
        with pytest.raises(ValueError, match="speed_step_mps"):
            KeyboardTeleopNode()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_min_speed_defaults_to_disabled():
    node = _make_node([])
    try:
        assert node.get_parameter("min_speed_mps").value == 0.0
        assert node._config.min_speed_mps == 0.0
        node.handle_raw_input("w")
        assert node._state.speed_mps == pytest.approx(node._config.speed_step_mps)
    finally:
        _teardown(node)


def test_min_speed_override_reaches_the_key_handler():
    node = _make_node(["-p", "min_speed_mps:=0.8", "-p", "speed_step_mps:=0.25"])
    try:
        assert node._config.min_speed_mps == 0.8
        node.handle_raw_input("w")
        assert node._state.speed_mps == 0.8
        node.handle_raw_input("w")
        assert node._state.speed_mps == pytest.approx(1.05)
        node.handle_raw_input("s")
        node.handle_raw_input("s")
        assert node._state.speed_mps == 0.0
    finally:
        _teardown(node)


@pytest.mark.parametrize("bad", ["-0.1", "1000.0"])
def test_an_invalid_min_speed_refuses_to_start(bad):
    if rclpy.ok():
        rclpy.shutdown()
    rclpy.init(args=["--ros-args", "-p", f"min_speed_mps:={bad}"])
    try:
        with pytest.raises(ValueError, match="min_speed_mps"):
            KeyboardTeleopNode()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
