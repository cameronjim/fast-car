"""L1 checks of lidar.launch.py and car_teleop.launch.py's `lidar` argument (roadmap 2.3).

Loads the SOURCE launch files as modules and inspects the launch descriptions they return; no
process is started, so sllidar_ros2 (built only in the car image, see lidar.launch.py's
docstring) does not need to be installed. What it pins:

  * lidar.launch.py's C1 launch arguments and their defaults;
  * the static transform comes from vehicle_params sensors.lidar and the launch refuses on a
    null mount field instead of inventing one;
  * the driver is handed vehicle_params' nominal scan rate and publishes on /scan;
  * car_teleop.launch.py declares `lidar` defaulting to false and includes lidar.launch.py
    only when it is true.
"""

from __future__ import annotations

import importlib.util
import pathlib
from types import SimpleNamespace

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

_LAUNCH_DIR = pathlib.Path(__file__).resolve().parents[1] / "launch"


def _load(name: str):
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), _LAUNCH_DIR / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _perform(value, context):
    """Resolve a launch value (substitution list, ParameterValue, plain value) to Python."""
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


def _node_parameters(node: Node, context: LaunchContext) -> dict:
    """The node's parameters as launch_ros will hand them over: final Python values."""
    merged = {}
    for entry in evaluate_parameters(context, node._Node__parameters):
        merged.update(entry)
    return merged


def _context(**configs) -> LaunchContext:
    context = LaunchContext()
    context.launch_configurations.update(configs)
    return context


_LIDAR_DEFAULTS = {
    "serial_port": "/dev/ttyUSB0",
    "serial_baudrate": "460800",
    "frame_id": "laser",
    "angle_compensate": "true",
    "scan_mode": "Standard",
}

_MOUNT = SimpleNamespace(mount_x_m=0.12, mount_y_m=-0.01, mount_z_m=0.2, mount_yaw_rad=3.0)
_SPEC = SimpleNamespace(nominal_scan_rate_hz=7.5)


# --- lidar.launch.py --------------------------------------------------------------------


def test_lidar_launch_declares_the_c1_arguments_with_slamtec_defaults():
    entities = _load("lidar.launch.py").generate_launch_description().entities
    assert _defaults(entities) == _LIDAR_DEFAULTS
    # The nodes are built at launch time from vehicle_params, not at description time.
    assert any(isinstance(e, OpaqueFunction) for e in entities)


def test_static_transform_arguments_come_from_the_mount_fields():
    module = _load("lidar.launch.py")
    args = module.static_transform_arguments(_MOUNT, "base_link", "laser")
    assert args == [
        "--x", "0.12",
        "--y", "-0.01",
        "--z", "0.2",
        "--yaw", "3.0",
        "--pitch", "0.0",
        "--roll", "0.0",
        "--frame-id", "base_link",
        "--child-frame-id", "laser",
    ]  # fmt: skip


@pytest.mark.parametrize("field", ["mount_x_m", "mount_y_m", "mount_z_m", "mount_yaw_rad"])
def test_static_transform_refuses_a_null_mount_field(field):
    module = _load("lidar.launch.py")
    mount = SimpleNamespace(**{**vars(_MOUNT), field: None})
    with pytest.raises(RuntimeError, match=field):
        module.static_transform_arguments(mount, "base_link", "laser")


def test_static_transform_names_every_null_field():
    module = _load("lidar.launch.py")
    mount = SimpleNamespace(mount_x_m=None, mount_y_m=0.0, mount_z_m=None, mount_yaw_rad=0.0)
    with pytest.raises(RuntimeError, match="mount_x_m, mount_z_m"):
        module.static_transform_arguments(mount, "base_link", "laser")


def test_lidar_nodes_driver_parameters_and_topic():
    module = _load("lidar.launch.py")
    driver, static_tf = module.lidar_nodes(_MOUNT, _SPEC, "laser")
    assert (driver.node_package, driver.node_executable) == ("sllidar_ros2", "sllidar_node")
    context = _context(**_LIDAR_DEFAULTS)
    params = _node_parameters(driver, context)
    assert params["channel_type"] == "serial"
    assert params["serial_port"] == "/dev/ttyUSB0"
    assert params["serial_baudrate"] == 460800
    assert isinstance(params["serial_baudrate"], int)
    assert params["angle_compensate"] is True
    assert params["scan_mode"] == "Standard"
    assert params["frame_id"] == "laser"
    assert params["scan_frequency"] == pytest.approx(7.5)
    remaps = [
        (_perform(src, context), _perform(dst, context)) for src, dst in driver._Node__remappings
    ]
    assert remaps == [("scan", "/scan")]

    assert (static_tf.node_package, static_tf.node_executable) == (
        "tf2_ros",
        "static_transform_publisher",
    )
    tf_args = [_perform(a, context) for a in static_tf._Node__arguments]
    assert tf_args == module.static_transform_arguments(_MOUNT, "base_link", "laser")


def test_lidar_nodes_follow_a_non_default_frame_id():
    module = _load("lidar.launch.py")
    driver, static_tf = module.lidar_nodes(_MOUNT, _SPEC, "lidar_top")
    context = _context(**_LIDAR_DEFAULTS)
    assert _node_parameters(driver, context)["frame_id"] == "lidar_top"
    tf_args = [_perform(a, context) for a in static_tf._Node__arguments]
    assert tf_args[-2:] == ["--child-frame-id", "lidar_top"]


def test_launch_setup_uses_the_committed_vehicle_params():
    loader = pytest.importorskip("racer_tools.vehicle_params_loader")
    lidar = loader.load_vehicle_params().sensors.lidar
    spec = loader.load_vehicle_params().sensors.lidar_spec
    module = _load("lidar.launch.py")
    nodes = module._launch_setup(_context(**_LIDAR_DEFAULTS))
    driver, static_tf = nodes
    context = _context(**_LIDAR_DEFAULTS)
    tf_args = [_perform(a, context) for a in static_tf._Node__arguments]
    assert tf_args == module.static_transform_arguments(lidar, "base_link", "laser")
    assert _node_parameters(driver, context)["scan_frequency"] == pytest.approx(
        spec.nominal_scan_rate_hz
    )


# --- car_teleop.launch.py `lidar` argument -----------------------------------------------


def test_car_teleop_lidar_defaults_off():
    entities = _load("car_teleop.launch.py").generate_launch_description().entities
    defaults = _defaults(entities)
    assert defaults["lidar"] == "false"
    assert defaults["lidar_serial_port"] == "/dev/ttyUSB0"


def test_car_teleop_includes_lidar_launch_only_when_asked():
    entities = _load("car_teleop.launch.py").generate_launch_description().entities
    # cameras.launch.py is included too (test_camera_launch.py); pick the LiDAR include.
    # Loading each description resolves its location and proves the file imports cleanly.
    includes = []
    for e in entities:
        if isinstance(e, IncludeLaunchDescription):
            e.launch_description_source.get_launch_description(LaunchContext())
            if pathlib.Path(e.launch_description_source.location).name == "lidar.launch.py":
                includes.append(e)
    assert len(includes) == 1
    include = includes[0]
    source_path = include.launch_description_source.location
    assert pathlib.Path(source_path).name == "lidar.launch.py"
    assert pathlib.Path(source_path).is_file()
    assert include.condition.evaluate(_context(lidar="false")) is False
    assert include.condition.evaluate(_context(lidar="true")) is True
    forwarded = {
        _perform(k, LaunchContext()): _perform(v, _context(lidar_serial_port="/dev/lidar"))
        for k, v in include.launch_arguments
    }
    assert forwarded == {"serial_port": "/dev/lidar"}
