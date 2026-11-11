"""lap_counter.launch.py -- lap_counter_node (camera lap counter). OPTIONAL, default not started.

Run it next to the camera (the car-camera container, docker/car/README.md "Cameras") or in
car-stack, wherever the camera's CompressedImage topic and /odom/wheel are visible:

    ros2 launch racer_perception lap_counter.launch.py
    ros2 launch racer_perception lap_counter.launch.py similarity_threshold:=0.85 min_lap_time_s:=10.0

or together with the cameras:  ros2 launch racer_bringup cameras.launch.py lap_counter:=true

It publishes /lap/event and /lap/status (JSON Strings); car_teleop.launch.py's recorder regex
takes both, so a lap count is always reconstructable from the run's bag. Nothing here touches
the command path. docs/notes/lap-counter-2026-11-12.md explains the parameters and tuning from
a bag (tools/lap_replay.py).

The defaults below are LapConfig's (the test pins that); no physical constants live here.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

#: Launch argument -> (default, description, parameter type). The argument name IS the node
#: parameter name.
ARGUMENTS = {
    "image_topic": (
        "/camera/usb/image_raw/compressed",
        "CompressedImage topic of the camera that sees the start line.",
        str,
    ),
    "odom_topic": ("/odom/wheel", "Odometry for speed and path length.", str),
    "yaw_rate_topic": (
        "/odom",
        "Odometry whose twist.angular.z feeds the heading-turn diagnostic ('' to disable).",
        str,
    ),
    "rate_hz": ("15.0", "Maximum frames processed per second.", float),
    "similarity_threshold": ("0.8", "Count a lap at a similarity peak at or above this.", float),
    "rearm_threshold": ("0.55", "Similarity must drop below this before the next lap.", float),
    "min_lap_time_s": ("6.0", "Shortest possible lap, seconds.", float),
    "min_lap_distance_m": ("3.0", "Shortest possible lap, metres of odometry.", float),
    "start_speed_mps": ("0.3", "Capture the reference at the first frame at this speed.", float),
}


def generate_launch_description() -> LaunchDescription:
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (default, description, _type) in ARGUMENTS.items()
    ]
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, (_default, _description, value_type) in ARGUMENTS.items()
    }
    node = Node(
        package="racer_perception",
        executable="lap_counter_node",
        name="lap_counter_node",
        output="screen",
        parameters=[parameters],
    )
    return LaunchDescription([*declared, node])
