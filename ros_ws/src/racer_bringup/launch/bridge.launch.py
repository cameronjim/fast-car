"""Minimal launch file for the gym<->ROS bridge (roadmap task 0.5).

Moved here from sim/bridge/racer_gym_bridge/launch/bridge.launch.py (milestone 1): that
file's own docstring said it would move into racer_bringup "the first time that package
actually lands with real launch infrastructure" (claude-docs/10-conventions.md: "Launch
files in racer_bringup only"), which sim_teleop.launch.py now does.
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
