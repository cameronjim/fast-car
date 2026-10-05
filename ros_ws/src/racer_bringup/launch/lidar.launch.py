"""lidar.launch.py -- the RPLIDAR C1 driver plus the base_link -> laser static transform.

Roadmap 2.3. Starts two nodes:

    sllidar_ros2/sllidar_node     reads the head over its CP210x USB-UART, publishes /scan
                                  (sensor_msgs/LaserScan, claude-docs/04-architecture.md)
    tf2_ros/static_transform_publisher
                                  base_link -> <frame_id>, from config/vehicle_params.yaml

Run it on its own for the bench check (docs/notes/first-boot-runbook.md "LiDAR first
power-up"), or through car_teleop.launch.py with lidar:=true so /scan lands in the run's bag.

THE DRIVER. sllidar_ros2 is Slamtec's own ROS 2 driver, built from source at a pinned commit
in docker/car/Dockerfile (there is no Humble apt package of it, and the apt rplidar_ros
package was not verified to support the C1). It is NOT in ros_ws/src and NOT an
<exec_depend> of racer_bringup: CI's ros-dev image does not carry it and rosdep has no key
for it, the same reason racer_gym_bridge is left out of package.xml. So this file only RUNS
in the car image. Its L1 test inspects the launch description and never starts the node.

PARAMETERS. The C1 settings are launch arguments with the values Slamtec's own
sllidar_c1_launch.py uses (460800 baud, angle_compensate true, scan_mode Standard), because
they are per-machine configuration (claude-docs/10-conventions.md), not physical constants.
The two things that ARE physical come from vehicle_params through the generated binding
(CLAUDE.md invariant 2):

  * sensors.lidar mount_x/y/z/yaw -> the static transform. Flat mount, so roll and pitch are
    zero. This launch REFUSES to start while any of the four is null: a made-up transform is
    worse than no LiDAR, because everything downstream would trust it.
  * sensors.lidar_spec.nominal_scan_rate_hz -> the driver's scan_frequency, which it uses to
    size the angle-compensated beam array (720 beams for the C1). racer_tools' lidar_check
    computes the expected beam count from the same field, so the two cannot drift apart.

QoS. sllidar_node publishes /scan reliable, KeepLast(10) (driver default, not configurable).
racer_safety/safety_node and racer_tools/lidar_check subscribe best_effort, which matches a
reliable publisher.
"""

from __future__ import annotations

from typing import Any

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

#: REP-105 vehicle frame the transform hangs the LiDAR off.
BASE_FRAME = "base_link"

#: The four vehicle_params sensors.lidar fields the transform needs, in --x/--y/--z/--yaw order.
_MOUNT_FIELDS = ("mount_x_m", "mount_y_m", "mount_z_m", "mount_yaw_rad")

#: Launch argument name -> (default, description). Defaults match Slamtec's
#: sllidar_c1_launch.py at the pinned commit, except frame_id, which is "laser" to match the
#: frame the sim bridge already uses (docs/notes/milestone-2-sim-viz.md).
LAUNCH_ARGUMENTS = {
    "serial_port": (
        "/dev/ttyUSB0",
        (
            "Serial device of the C1's CP210x adapter. /dev/ttyUSB0 on the Jetson when it is "
            "the only USB serial adapter; /dev/lidar with the udev rule in tools/udev/."
        ),
    ),
    "serial_baudrate": ("460800", "UART baud rate. The C1 runs at 460800."),
    "frame_id": (
        "laser",
        "frame_id stamped on /scan and the child frame of the base_link static transform.",
    ),
    "angle_compensate": (
        "true",
        (
            "Bin each revolution into a fixed-size array (720 beams for the C1). lidar_check's "
            "angle_compensate parameter must match."
        ),
    ),
    "scan_mode": ("Standard", "C1 scan mode. The driver logs the supported modes if wrong."),
}


def static_transform_arguments(mount: Any, parent_frame: str, child_frame: str) -> list[str]:
    """tf2_ros static_transform_publisher arguments for vehicle_params' sensors.lidar.

    Raises RuntimeError naming every null mount field: the launch must not come up with an
    invented transform.
    """
    missing = [name for name in _MOUNT_FIELDS if getattr(mount, name) is None]
    if missing:
        raise RuntimeError(
            "lidar.launch.py: config/vehicle_params.yaml sensors.lidar has null "
            f"{', '.join(missing)}. Measure the LiDAR mount and fill them in (they may be "
            "marked PROVISIONAL) before launching; refusing to publish a made-up "
            f"{parent_frame} -> {child_frame} transform."
        )
    x, y, z, yaw = (float(getattr(mount, name)) for name in _MOUNT_FIELDS)
    return [
        "--x", repr(x),
        "--y", repr(y),
        "--z", repr(z),
        "--yaw", repr(yaw),
        "--pitch", "0.0",
        "--roll", "0.0",
        "--frame-id", parent_frame,
        "--child-frame-id", child_frame,
    ]  # fmt: skip


def lidar_nodes(lidar_mount: Any, lidar_spec: Any, frame_id: str) -> list[Node]:
    """The driver node and the static transform node, from vehicle_params sections."""
    tf_args = static_transform_arguments(lidar_mount, BASE_FRAME, frame_id)
    driver = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": LaunchConfiguration("serial_port"),
                # ParameterValue with an explicit type: a bare LaunchConfiguration is text,
                # and sllidar_node declares these as int / bool.
                "serial_baudrate": ParameterValue(
                    LaunchConfiguration("serial_baudrate"), value_type=int
                ),
                "frame_id": frame_id,
                "angle_compensate": ParameterValue(
                    LaunchConfiguration("angle_compensate"), value_type=bool
                ),
                "scan_mode": LaunchConfiguration("scan_mode"),
                "scan_frequency": float(lidar_spec.nominal_scan_rate_hz),
            }
        ],
        # The driver publishes on the relative name "scan"; pin it to the interface topic in
        # claude-docs/04-architecture.md whatever namespace this is included under.
        remappings=[("scan", "/scan")],
    )
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_laser_tf",
        output="screen",
        arguments=tf_args,
    )
    return [driver, static_tf]


def _launch_setup(context, *args, **kwargs):
    # Imported here, not at module top, so the launch description (and its L1 test) can be
    # built without loading vehicle_params; the binding is only needed once the launch runs.
    from racer_tools.vehicle_params_loader import load_vehicle_params

    sensors = load_vehicle_params().sensors
    frame_id = LaunchConfiguration("frame_id").perform(context)
    return lidar_nodes(sensors.lidar, sensors.lidar_spec, frame_id)


def generate_launch_description() -> LaunchDescription:
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (default, description) in LAUNCH_ARGUMENTS.items()
    ]
    return LaunchDescription([*declared, OpaqueFunction(function=_launch_setup)])
