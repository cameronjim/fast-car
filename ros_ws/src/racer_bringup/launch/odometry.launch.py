"""odometry.launch.py -- the car's odometry: wheel speed + modelled steering through an EKF.

    /odom/wheel (VESC, racer_drivers/vesc_odometry_node; or racer_gym_bridge in the sim)
    /drive      (safety_node's output, read only)
        -> racer_state/wheel_twist_node -> /odom/wheel_twist
        -> robot_localization ekf_node  -> /odom (nav_msgs/Odometry) + TF odom -> base_link

Included by mapping.launch.py and localization.launch.py (both default odometry:=true), which
are in turn included by car_teleop.launch.py (mapping:=true / localization:=true) and the sim
launch. Nothing here publishes on the command path: wheel_twist_node only READS /drive.

Parameters: racer_state/config/ekf.yaml (override with ekf_params_file:=...). Physical constants
(wheelbase, steering limits and rate) come from the generated vehicle_params binding inside
wheel_twist_node, never from this file. See docs/notes/localization-2026-11-03.md.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ekf_params_arg = DeclareLaunchArgument(
        "ekf_params_file",
        default_value=PathJoinSubstitution([FindPackageShare("racer_state"), "config", "ekf.yaml"]),
        description="robot_localization ekf_node parameters (racer_state/config/ekf.yaml).",
    )
    wheel_odom_topic_arg = DeclareLaunchArgument(
        "wheel_odom_topic",
        default_value="/odom/wheel",
        description="Wheel speed input of wheel_twist_node (vesc_odometry_node's /odom/wheel).",
    )

    wheel_twist_node = Node(
        package="racer_state",
        executable="wheel_twist_node",
        name="wheel_twist_node",
        output="screen",
        parameters=[{"wheel_odom_topic": LaunchConfiguration("wheel_odom_topic")}],
    )
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        output="screen",
        parameters=[LaunchConfiguration("ekf_params_file")],
        # claude-docs/04-architecture.md: /odom comes from the EKF.
        remappings=[("odometry/filtered", "/odom")],
    )
    return LaunchDescription([ekf_params_arg, wheel_odom_topic_arg, wheel_twist_node, ekf_node])
