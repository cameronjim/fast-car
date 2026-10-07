"""L1 checks of vesc_telemetry.launch.py, car_teleop.launch.py's `vesc` argument, and the
read-only patch the car image applies to the f1tenth vesc_driver.

Loads the SOURCE launch files as modules and inspects the launch descriptions they return; no
process is started, so vesc_driver (built only in the car image, see vesc_telemetry.launch.py's
docstring) does not need to be installed. What it pins:

  * the launch arguments and their defaults (/dev/ttyTHS1, 115200) and that they reach the
    driver's port / baud parameters, baud as an integer;
  * READ-ONLY, from the launch side: no node in the file remaps anything at all, so no
    commands/* topic of the driver can be connected to a publisher in this graph, and nothing
    from vesc_ackermann (which turns /drive into VESC commands) is started;
  * vesc_odometry_node listens on the topic the namespaced driver publishes;
  * car_teleop.launch.py declares `vesc` defaulting to false, includes vesc_telemetry.launch.py
    only when it is true, and its recorder regex keeps /odom/wheel, /telemetry/vesc/* and the
    raw /vesc/sensors/core;
  * READ-ONLY, from the driver side: docker/car/patches/vesc_driver-readonly.patch removes all
    six command subscriptions, adds none back, and turns every set-command send into a no-op.
    CI cannot build the driver (no vesc_driver in ros-dev), so this is checked on the patch
    text, and docker/car/Dockerfile re-checks the patched source at image build time.
"""

from __future__ import annotations

import importlib.util
import pathlib
import re

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

_LAUNCH_DIR = pathlib.Path(__file__).resolve().parents[1] / "launch"
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_PATCH = _REPO_ROOT / "docker" / "car" / "patches" / "vesc_driver-readonly.patch"
_DOCKERFILE = _REPO_ROOT / "docker" / "car" / "Dockerfile"

#: The command topics upstream vesc_driver subscribes (relative names, f1tenth/vesc ros2).
_DRIVER_COMMAND_TOPICS = (
    "commands/motor/duty_cycle",
    "commands/motor/current",
    "commands/motor/brake",
    "commands/motor/speed",
    "commands/motor/position",
    "commands/servo/position",
)


def _load(name: str):
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), _LAUNCH_DIR / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _perform(value, context):
    if hasattr(value, "evaluate"):
        return value.evaluate(context)
    if hasattr(value, "perform"):
        return value.perform(context)
    if isinstance(value, list | tuple) and value and all(hasattr(v, "perform") for v in value):
        return "".join(v.perform(context) for v in value)
    return value


def _defaults(entities):
    context = LaunchContext()
    return {
        e.name: "".join(s.perform(context) for s in e.default_value)
        for e in entities
        if isinstance(e, DeclareLaunchArgument)
    }


def _context(**configs) -> LaunchContext:
    context = LaunchContext()
    context.launch_configurations.update(configs)
    return context


def _node_parameters(node: Node, context: LaunchContext) -> dict:
    merged = {}
    for entry in evaluate_parameters(context, node._Node__parameters):
        merged.update(entry)
    return merged


def _nodes(entities) -> list[Node]:
    return [e for e in entities if isinstance(e, Node)]


_VESC_DEFAULTS = {"serial_port": "/dev/ttyTHS1", "baud": "115200"}


# --- vesc_telemetry.launch.py ----------------------------------------------------------


def test_vesc_launch_declares_port_and_baud_with_jetson_uart_defaults():
    entities = _load("vesc_telemetry.launch.py").generate_launch_description().entities
    assert _defaults(entities) == _VESC_DEFAULTS


def test_vesc_launch_starts_exactly_the_driver_and_the_odometry_node():
    entities = _load("vesc_telemetry.launch.py").generate_launch_description().entities
    nodes = _nodes(entities)
    assert [(n.node_package, n.node_executable) for n in nodes] == [
        ("vesc_driver", "vesc_driver_node"),
        ("racer_drivers", "vesc_odometry_node"),
    ]
    # vesc_ackermann's ackermann_to_vesc would turn /drive into VESC commands.
    assert not any(n.node_package == "vesc_ackermann" for n in nodes)


def test_driver_parameters_come_from_the_launch_arguments():
    module = _load("vesc_telemetry.launch.py")
    driver, _ = module.vesc_nodes()
    params = _node_parameters(driver, _context(serial_port="/dev/ttyTHS1", baud="115200"))
    assert params == {"port": "/dev/ttyTHS1", "baud": 115200}
    assert isinstance(params["baud"], int)
    # Fresh nodes: launch_ros' ParameterValue caches its first evaluation.
    driver, _ = module.vesc_nodes()
    params = _node_parameters(driver, _context(serial_port="/dev/ttyACM0", baud="230400"))
    assert params == {"port": "/dev/ttyACM0", "baud": 230400}


def test_no_node_remaps_anything_so_no_command_topic_is_connected():
    """READ-ONLY, launch side. Upstream's driver subscribes the relative commands/* topics; in
    the /vesc namespace they would be /vesc/commands/*, which nothing in this repo publishes.
    A remap is the only way this file could wire one of them to a real publisher (for example
    /drive), so the file is required to have none at all, not merely none that look like a
    command."""
    entities = _load("vesc_telemetry.launch.py").generate_launch_description().entities
    context = _context(**_VESC_DEFAULTS)
    for node in _nodes(entities):
        remaps = [
            (_perform(src, context), _perform(dst, context))
            for src, dst in (node._Node__remappings or [])
        ]
        assert remaps == [], f"{node.node_executable} remaps {remaps}"
        for argument in node._Node__arguments or []:
            text = _perform(argument, context)
            assert ":=" not in str(text), f"{node.node_executable} has a remap argument {text}"


def test_odometry_listens_where_the_namespaced_driver_publishes():
    module = _load("vesc_telemetry.launch.py")
    driver, odometry = module.vesc_nodes()
    context = _context(**_VESC_DEFAULTS)
    namespace = _perform(driver._Node__node_namespace, context)
    assert namespace == module.VESC_NAMESPACE
    # The driver publishes the relative name sensors/core.
    assert module.STATE_TOPIC == f"/{namespace}/sensors/core"
    assert _node_parameters(odometry, context) == {"state_topic": module.STATE_TOPIC}


# --- car_teleop.launch.py `vesc` argument ------------------------------------------------


def _vesc_include(entities) -> IncludeLaunchDescription:
    found = []
    for e in entities:
        if isinstance(e, IncludeLaunchDescription):
            e.launch_description_source.get_launch_description(LaunchContext())
            if pathlib.Path(e.launch_description_source.location).name == (
                "vesc_telemetry.launch.py"
            ):
                found.append(e)
    assert len(found) == 1
    return found[0]


def test_car_teleop_vesc_defaults_off():
    defaults = _defaults(_load("car_teleop.launch.py").generate_launch_description().entities)
    assert defaults["vesc"] == "false"
    assert defaults["vesc_serial_port"] == "/dev/ttyTHS1"


def test_car_teleop_includes_vesc_launch_only_when_asked():
    entities = _load("car_teleop.launch.py").generate_launch_description().entities
    include = _vesc_include(entities)
    assert pathlib.Path(include.launch_description_source.location).is_file()
    assert include.condition.evaluate(_context(vesc="false")) is False
    assert include.condition.evaluate(_context(vesc="true")) is True
    forwarded = {
        _perform(k, LaunchContext()): _perform(v, _context(vesc_serial_port="/dev/ttyTHS0"))
        for k, v in include.launch_arguments
    }
    assert forwarded == {"serial_port": "/dev/ttyTHS0"}


@pytest.mark.parametrize(
    "topic",
    [
        "/odom/wheel",
        "/telemetry/vesc/voltage_v",
        "/telemetry/vesc/temp_motor_degc",
        "/telemetry/vesc/fault",
        "/vesc/sensors/core",
        # Unchanged entries still match.
        "/drive",
        "/scan",
        "/telemetry/rail_voltage_v",
    ],
)
def test_recorder_regex_keeps_the_vesc_topics(topic):
    regex = _load("car_teleop.launch.py")._RECORDED_TOPIC_REGEX
    assert re.fullmatch(regex, topic), topic


@pytest.mark.parametrize("topic", ["/odom", "/odom/wheel_extra", "/vesc/sensors/imu"])
def test_recorder_regex_is_still_anchored(topic):
    regex = _load("car_teleop.launch.py")._RECORDED_TOPIC_REGEX
    assert not re.fullmatch(regex, topic), topic


# --- the read-only patch -----------------------------------------------------------------


def _patch_lines(sign: str) -> list[str]:
    return [
        line[1:]
        for line in _PATCH.read_text(encoding="utf-8").splitlines()
        if line.startswith(sign) and not line.startswith(sign * 3)
    ]


def test_patch_removes_all_six_command_subscriptions_and_adds_none():
    removed = "\n".join(_patch_lines("-"))
    added = "\n".join(_patch_lines("+"))
    for topic in _DRIVER_COMMAND_TOPICS:
        assert f'"{topic}"' in removed, f"patch does not remove the {topic} subscription"
        assert f'"{topic}"' not in added, f"patch re-adds {topic}"
    assert removed.count("create_subscription") == 6
    assert "create_subscription" not in added


def test_patch_turns_every_set_command_send_into_a_no_op():
    removed = "\n".join(_patch_lines("-"))
    added = "\n".join(_patch_lines("+"))
    for packet in (
        "VescPacketSetDuty",
        "VescPacketSetCurrent(",
        "VescPacketSetCurrentBrake",
        "VescPacketSetRPM",
        "VescPacketSetPos",
        "VescPacketSetServoPos",
    ):
        assert f"send({packet}" in removed, f"patch does not remove send({packet}...)"
    assert "send(" not in added


def test_patch_turns_off_hardware_flow_control():
    assert "FlowControl::HARDWARE" in "\n".join(_patch_lines("-"))
    assert "FlowControl::NONE" in "\n".join(_patch_lines("+"))


def test_dockerfile_applies_the_patch_and_checks_the_result():
    text = _DOCKERFILE.read_text(encoding="utf-8")
    assert "vesc_driver-readonly.patch" in text
    assert "create_subscription" in text, "the build-time no-subscription check is missing"
    assert "--packages-select vesc_msgs vesc_driver" in text, (
        "the image must build only vesc_msgs and vesc_driver, never vesc_ackermann"
    )
