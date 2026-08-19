"""sim_autopilot.launch.py -- milestone 3: the classical (non-learned) stack drives the
simulated car around the sim track ITSELF, through the REAL command path
(claude-docs/04-architecture.md):

    tracker_node -> /drive_raw -> safety_node -> /drive -> bridge_node (racer_gym)

racer_safety/safety_node remains the ONLY node in this graph that publishes /drive
(claude-docs/05-safety.md) -- tracker_node's /drive_raw is gated exactly like
keyboard_teleop_node's was in sim_teleop.launch.py; no node here bypasses that gate.

Pose source (read before touching this file): tracker_node subscribes /odom, which
claude-docs/04-architecture.md reserves for the real fused estimator (racer_state, roadmap
Phase 2) -- that estimator does not exist yet. In sim the only pose available is
bridge_node's ground truth (/sim/ground_truth_odom, the SAME ground truth
sim_teleop.launch.py/foxglove_sim_viz.layout.json already display). Per this milestone's
explicit instruction ("do not rename architecture topics; prefer a remap"), this launch
file remaps ONLY tracker_node's own subscription -- local name /odom -> actual topic
/sim/ground_truth_odom -- leaving bridge_node's real topic name untouched (bridge_node
still genuinely publishes /sim/ground_truth_odom, exactly as milestone 2's Foxglove layout
and the architecture doc's topic table expect). This is a sim-only pose-source adapter,
not a stand-in for real localization: real localization (racer_state's EKF/particle
filter) replaces this remap in Phase 2, and this is called out again in
docs/notes/milestone-3-sim-autopilot.md.

Both bridge_node and tracker_node are pointed at the SAME committed raceline
(`raceline_path` argument, default ../config/tracks/gym_oval/raceline.csv -- roadmap task
S.2's tools/raceline output) so bridge_node builds the matching closed-loop track
tracker_node is trying to follow -- the same wiring tests/l5_tracker_lap already proved
out for tracker_node + bridge_node alone, just now with the real safety_node in the loop
instead of that test's test-only remap shim. Relative raceline paths are resolved against
the launching process's current working directory: the default is relative to `ros_ws`
(one level up to the repo root, then into `config/tracks/`), matching every documented demo
procedure in this repo, which builds/sources ros_ws and runs `ros2 launch` from inside it
(docs/notes/milestone-1-sim-teleop.md, milestone-2-sim-viz.md, milestone-3-sim-autopilot.md)
-- NOT from the repo root itself.

`raceline_publisher_node` (racer_tools) publishes that same raceline once as a latched
nav_msgs/Path on /sim/raceline so Foxglove can show the line the tracker is following,
alongside everything milestone 2 already shows -- see
config/foxglove_sim_autopilot.layout.json.

`viz` (default true) starts foxglove_bridge, same as sim_teleop.launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_FOXGLOVE_BRIDGE_PORT = 8765
_DEFAULT_RACELINE_PATH = "../config/tracks/gym_oval/raceline.csv"


def generate_launch_description() -> LaunchDescription:
    raceline_path_arg = DeclareLaunchArgument(
        "raceline_path",
        default_value=_DEFAULT_RACELINE_PATH,
        description=(
            "Path to the tools/raceline-generated raceline CSV both bridge_node and "
            "tracker_node track (claude-docs/02-repo-layout.md: "
            "config/tracks/<venue>_<layout>/raceline.csv). Relative paths resolve against "
            "the launching process's cwd -- the default assumes `ros2 launch` is run from "
            "inside `ros_ws` (see docs/notes/milestone-3-sim-autopilot.md's demo procedure); "
            "pass an absolute path if running from somewhere else."
        ),
    )
    viz_arg = DeclareLaunchArgument(
        "viz",
        default_value="true",
        description=(
            "Start foxglove_bridge (ros-humble-foxglove-bridge) on port "
            f"{_FOXGLOVE_BRIDGE_PORT}, same as sim_teleop.launch.py. Default true."
        ),
    )

    raceline_path = LaunchConfiguration("raceline_path")

    bridge_node = Node(
        package="racer_gym_bridge",
        executable="bridge_node",
        name="bridge_node",
        output="screen",
        parameters=[{"raceline_path": raceline_path}],
    )
    safety_node = Node(
        package="racer_safety",
        executable="safety_node",
        name="safety_node",
        output="screen",
    )
    tracker_node = Node(
        package="racer_control",
        executable="tracker_node",
        name="tracker_node",
        output="screen",
        parameters=[{"raceline_path": raceline_path}],
        # Sim-only pose-source adapter remap -- see this file's module docstring. Only
        # tracker_node's own subscription is remapped; bridge_node's real topic name
        # (/sim/ground_truth_odom) is untouched.
        remappings=[("/odom", "/sim/ground_truth_odom")],
    )
    raceline_publisher_node = Node(
        package="racer_tools",
        executable="raceline_publisher_node",
        name="raceline_publisher_node",
        output="screen",
        parameters=[{"raceline_path": raceline_path}],
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
            raceline_path_arg,
            viz_arg,
            bridge_node,
            safety_node,
            tracker_node,
            raceline_publisher_node,
            foxglove_bridge_node,
        ]
    )
