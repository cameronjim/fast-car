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

Milestone 2 adds the `viz` launch argument (default true): starts `foxglove_bridge` on port
8765 alongside bridge_node/safety_node, so the owner can watch the simulated car (map, TF,
/scan, pose) live in the Foxglove app instead of driving headless -- see
docs/notes/milestone-2-sim-viz.md for the exact end-to-end demo procedure, including the
`docker run -p 8765:8765` port publish this needs.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_FOXGLOVE_BRIDGE_PORT = 8765


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
    viz_arg = DeclareLaunchArgument(
        "viz",
        default_value="true",
        description=(
            "Start foxglove_bridge (ros-humble-foxglove-bridge, docker/ros-dev/Dockerfile) "
            f"on port {_FOXGLOVE_BRIDGE_PORT} so the Foxglove app can connect and show the "
            "sim live (map, TF, /scan, pose). Default true; see "
            "docs/notes/milestone-2-sim-viz.md for the demo procedure."
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
    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{"port": _FOXGLOVE_BRIDGE_PORT}],
        condition=IfCondition(LaunchConfiguration("viz")),
    )

    return LaunchDescription(
        [start_teleop_arg, viz_arg, bridge_node, safety_node, teleop_node, foxglove_bridge_node]
    )
