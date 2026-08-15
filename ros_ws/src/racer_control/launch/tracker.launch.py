"""Minimal launch file for tracker_node (roadmap task S.2).

claude-docs/10-conventions.md says launch files belong in racer_bringup only, but that
package has no code yet (still a `.gitkeep` placeholder, claude-docs/01-roadmap.md task
0.1) -- same rationale sim/bridge/racer_gym_bridge/launch/bridge.launch.py already used.
This moves into racer_bringup the first time that package actually lands with real launch
infrastructure. `raceline_path` has no default: the caller must supply one (see
tracker_node.cpp: the node refuses to start without it).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    raceline_path_arg = DeclareLaunchArgument(
        "raceline_path",
        description="Path to a tools/raceline-generated raceline CSV (required).",
    )
    tracker_node = Node(
        package="racer_control",
        executable="tracker_node",
        name="tracker_node",
        output="screen",
        parameters=[{"raceline_path": LaunchConfiguration("raceline_path")}],
    )
    return LaunchDescription([raceline_path_arg, tracker_node])
