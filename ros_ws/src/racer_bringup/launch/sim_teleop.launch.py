"""sim_teleop.launch.py -- milestone 1: keyboard-driven remote control of the simulated car
through the REAL command path (claude-docs/04-architecture.md):

    keyboard_teleop_node -> /drive_raw -> safety_node -> /drive -> bridge_node

racer_safety/safety_node is the ONLY node this (or any) launch file may start that publishes
/drive (claude-docs/05-safety.md). No test-only remaps here, unlike tests/l5_tracker_lap or
tests/e2e_sim_safety -- this is the real graph shape, unmodified.

Starts racer_gym_bridge/bridge_node and racer_safety/safety_node. keyboard_teleop_node needs
a real interactive TTY (raw terminal mode, see racer_tools/keyboard_teleop_node.py), which a
`ros2 launch`-managed subprocess does not reliably provide -- launch multiplexes several
processes' stdio, and only one process can meaningfully own the terminal for raw single-key
input. The documented, supported way to drive the car is to run THIS launch file in one
terminal and `ros2 run racer_tools keyboard_teleop_node` in a SEPARATE second terminal (see
docs/notes/milestone-1-sim-teleop.md for the exact commands). The `start_teleop` launch
argument (default false) can also start keyboard_teleop_node in this same launch for
convenience in an environment where that stdio caveat happens not to matter, but that is not
the supported path.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    start_teleop_arg = DeclareLaunchArgument(
        "start_teleop",
        default_value="false",
        description=(
            "Also start keyboard_teleop_node in this launch. NOT the supported/documented "
            "path (teleop needs a real interactive TTY a ros2-launch-managed subprocess does "
            "not reliably provide) -- run it in a separate terminal instead, see "
            "docs/notes/milestone-1-sim-teleop.md. Default false."
        ),
    )

    bridge_node = Node(
        package="racer_gym_bridge",
        executable="bridge_node",
        name="bridge_node",
        output="screen",
    )
    safety_node = Node(
        package="racer_safety",
        executable="safety_node",
        name="safety_node",
        output="screen",
    )
    teleop_node = Node(
        package="racer_tools",
        executable="keyboard_teleop_node",
        name="keyboard_teleop",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_teleop")),
    )

    return LaunchDescription([start_teleop_arg, bridge_node, safety_node, teleop_node])
