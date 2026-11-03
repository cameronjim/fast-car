"""mapping.launch.py -- build a map with slam_toolbox (online async) on top of the EKF odometry.

    odometry.launch.py (odometry:=true)  -> /odom, TF odom -> base_link
    slam_toolbox async_slam_toolbox_node -> /map, TF map -> odom

Needs /scan (lidar:=true on the car) and /odom/wheel (vesc:=true on the car). Drive the track
slowly, then save the map while this is still running:

    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \\
        "{name: {data: '/data/maps/<track_id>'}}"

which writes <track_id>.yaml and <track_id>.pgm for localization.launch.py's map argument.
Parameters: racer_state/config/slam_toolbox.yaml. See docs/notes/localization-2026-11-03.md and
docs/notes/first-boot-runbook.md "First mapping session".
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_ODOMETRY_LAUNCH_FILE = str(Path(__file__).resolve().parent / "odometry.launch.py")


def generate_launch_description() -> LaunchDescription:
    odometry_arg = DeclareLaunchArgument(
        "odometry",
        default_value="true",
        description="Also start odometry.launch.py (wheel_twist_node + ekf_node). Default true.",
    )
    slam_params_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("racer_state"), "config", "slam_toolbox.yaml"]
        ),
        description="slam_toolbox parameters (racer_state/config/slam_toolbox.yaml).",
    )

    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_ODOMETRY_LAUNCH_FILE),
        condition=IfCondition(LaunchConfiguration("odometry")),
    )
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[LaunchConfiguration("slam_params_file")],
    )
    return LaunchDescription([odometry_arg, slam_params_arg, odometry_launch, slam_toolbox_node])
