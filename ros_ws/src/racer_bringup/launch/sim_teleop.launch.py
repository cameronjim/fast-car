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

Milestone 5 adds `browser_teleop` (default TRUE) and `racer_tools/twist_teleop_adapter_node`:
subscribes `geometry_msgs/Twist` on `teleop_cmd_vel_topic` (default `/teleop/cmd_vel`, the
topic Foxglove Studio's built-in Teleop panel publishes -- see
`config/foxglove_sim_viz.layout.json`, which now wires a Teleop panel to it, and
docs/notes/milestone-5-browser-teleop.md for the demo procedure) and republishes
`/drive_raw`, so the owner can drive entirely from the Foxglove window -- no second
`docker exec` terminal in raw tty mode needed, unlike keyboard_teleop_node.

ONE-PUBLISHER-AT-A-TIME SCHEME (read before changing either default): `start_teleop` and
`browser_teleop` both start a node that publishes `/drive_raw`, and nothing arbitrates
between two simultaneous publishers of the same topic -- `safety_node` gates whatever it
last received, from whichever source published most recently, which is not a defined or
useful behavior if both are actually running. The chosen default scheme is BROWSER TELEOP ON,
KEYBOARD TELEOP OFF: `browser_teleop` defaults to `true` (driving from Foxglove works out of
the box with just this one launch file) and `start_teleop` keeps its milestone-1 default of
`false` (keyboard teleop still needs the documented two-terminal procedure, and does not also
need a THIRD flag flipped just to avoid conflicting with the new default). To drive by
keyboard instead, launch with `browser_teleop:=false start_teleop:=true` (or run
keyboard_teleop_node in its own terminal per docs/notes/milestone-1-sim-teleop.md, still with
`browser_teleop:=false`). Do not set both to `true`.
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
            "docs/notes/milestone-1-sim-teleop.md. Default false. Do not set this AND "
            "browser_teleop to true at once -- see this file's module docstring."
        ),
    )
    browser_teleop_arg = DeclareLaunchArgument(
        "browser_teleop",
        default_value="true",
        description=(
            "Start racer_tools/twist_teleop_adapter_node so the car can be driven from "
            "Foxglove's Teleop panel with no separate terminal (roadmap milestone 5). Default "
            "true. Do not set this AND start_teleop to true at once -- see this file's module "
            "docstring."
        ),
    )
    teleop_cmd_vel_topic_arg = DeclareLaunchArgument(
        "teleop_cmd_vel_topic",
        default_value="/teleop/cmd_vel",
        description=(
            "Topic twist_teleop_adapter_node subscribes geometry_msgs/Twist on -- must match "
            "the Teleop panel's configured topic in whatever Foxglove layout is in use "
            "(config/foxglove_sim_viz.layout.json's committed default)."
        ),
    )
    twist_timeout_s_arg = DeclareLaunchArgument(
        "twist_timeout_s",
        default_value="0.5",
        description=(
            "twist_teleop_adapter_node: seconds since the last received Twist after which it "
            "commands (and keeps commanding) zero -- see that node's module docstring for the "
            "no-Twist-timeout design."
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
    twist_teleop_adapter_node = Node(
        package="racer_tools",
        executable="twist_teleop_adapter_node",
        name="twist_teleop_adapter",
        output="screen",
        parameters=[
            {
                "input_topic": LaunchConfiguration("teleop_cmd_vel_topic"),
                "twist_timeout_s": LaunchConfiguration("twist_timeout_s"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("browser_teleop")),
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
        [
            start_teleop_arg,
            browser_teleop_arg,
            teleop_cmd_vel_topic_arg,
            twist_timeout_s_arg,
            viz_arg,
            bridge_node,
            safety_node,
            teleop_node,
            twist_teleop_adapter_node,
            foxglove_bridge_node,
        ]
    )
