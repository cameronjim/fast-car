"""Minimal launch file for wall_follow_node (GitHub issue 26).

Lives in racer_control/launch alongside tracker.launch.py, for the same reason given there
(claude-docs/10-conventions.md puts launch files in racer_bringup, which has no launch
infrastructure for these controllers yet). The node subscribes /scan and publishes /drive_raw
only: run it with racer_safety's safety_node, which gates /drive_raw -> /drive. Nothing here
remaps /drive_raw onto /drive.

Every tuning parameter is a declared ROS parameter with a default (see
src/wall_follow_node.cpp). `max_speed_mps` is exposed here because it is the one most often
changed per session; `params_file` takes a ROS parameter YAML for the rest (for example the
left-wall ray bearings).

laser_yaw_from_vehicle_params (default true) must stay true on the real car, where the LiDAR
yaw comes only from vehicle_params sensors.lidar.mount_yaw_rad. Set it false (in the params
file) ONLY for the simulator and synthetic-scan tests, whose /scan is aligned to the vehicle
(yaw 0); the node then uses laser_yaw_offset_rad as given and logs that it ignored the binding.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

_NODE = "wall_follow_node"
_DEFAULT_MAX_SPEED_MPS = "1.5"


def generate_launch_description() -> LaunchDescription:
    max_speed_arg = DeclareLaunchArgument(
        "max_speed_mps",
        default_value=_DEFAULT_MAX_SPEED_MPS,
        description="Speed on a straight in m/s (range-limited to vehicle_params' global cap).",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value="",
        description="Optional ROS parameter YAML for the remaining tuning parameters.",
    )
    max_speed = {"max_speed_mps": LaunchConfiguration("max_speed_mps")}
    has_params_file = PythonExpression(["'", LaunchConfiguration("params_file"), "' != ''"])
    with_file = Node(
        package="racer_control",
        executable=_NODE,
        name=_NODE,
        output="screen",
        parameters=[LaunchConfiguration("params_file"), max_speed],
        condition=IfCondition(has_params_file),
    )
    without_file = Node(
        package="racer_control",
        executable=_NODE,
        name=_NODE,
        output="screen",
        parameters=[max_speed],
        condition=UnlessCondition(has_params_file),
    )
    return LaunchDescription([max_speed_arg, params_file_arg, with_file, without_file])
