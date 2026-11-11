"""offline_map.launch.py -- the car's mapping stack on a replayed bag (use_sim_time).

    twist_source:=feeder (default): bag_feeder.py publishes /odom/wheel_twist itself (measured
        steering), so only the robot_localization ekf_node and slam_toolbox run here.
    twist_source:=node: racer_bringup's odometry.launch.py, the real wheel_twist_node with
        vehicle_params' symmetric steering limits, plus slam_toolbox (what the car runs today).

Parameters are the car's own (racer_state/config/ekf.yaml, slam_toolbox.yaml); only the clock
differs. Started by tools/track_session/map_from_bag.sh.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    twist_source = LaunchConfiguration("twist_source")
    odometry_launch = str(
        Path(FindPackageShare("racer_bringup").find("racer_bringup"))
        / "launch"
        / "odometry.launch.py"
    )
    state_config = PathJoinSubstitution([FindPackageShare("racer_state"), "config"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("twist_source", default_value="feeder"),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=PathJoinSubstitution([state_config, "slam_toolbox.yaml"]),
            ),
            SetParameter(name="use_sim_time", value=True),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(odometry_launch),
                condition=IfCondition(PythonExpression(["'", twist_source, "' == 'node'"])),
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_node",
                output="screen",
                parameters=[PathJoinSubstitution([state_config, "ekf.yaml"])],
                remappings=[("odometry/filtered", "/odom")],
                condition=IfCondition(PythonExpression(["'", twist_source, "' != 'node'"])),
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[LaunchConfiguration("slam_params_file")],
            ),
        ]
    )
