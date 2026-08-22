"""Minimal launch file for the gym<->ROS bridge (roadmap task 0.5).

claude-docs/10-conventions.md says launch files belong in racer_bringup
only, but ros_ws/src/racer_bringup has no code yet (still a `.gitkeep`
placeholder per claude-docs/01-roadmap.md task 0.1) and wiring
cross-package launch infrastructure for a single node right now would be
more sprawl than value. This file moves into racer_bringup the first time
that package actually lands with real launch infrastructure.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bridge_node = Node(
        package="racer_gym_bridge",
        executable="bridge_node",
        name="bridge_node",
        output="screen",
    )
    return LaunchDescription([bridge_node])
