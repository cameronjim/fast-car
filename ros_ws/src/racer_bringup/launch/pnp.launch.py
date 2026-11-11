"""pnp.launch.py -- the Plug-and-Play mapless driver (claude-docs/13-plug-and-play.md).

    ros2 launch racer_bringup pnp.launch.py \
        perception_bundle:=/workspace/data/pnp/models/<perception model_id> \
        dynamics_bundle:=/workspace/data/pnp/models/<dynamics model_id> v_max_mps:=0.5

Started in a second terminal next to the running stack (`car_teleop.launch.py lidar:=true
vesc:=true`, recorder on), exactly like racer_control's gap_follow.launch.py. It starts two nodes
and nothing else:

    racer_track_perception/track_perception_node   /scan -> /pnp/local_track
    racer_planner/mppi_planner_node                /pnp/local_track + /odom/wheel -> /drive_raw

mppi_planner_node is the only publisher of /drive_raw and nothing here publishes /drive or remaps
anything onto it: the command still goes through safety_node and the hardware mux (CLAUDE.md
invariant 1). One driver at a time: never start this next to gap_follow or the tracker.

Refusals (a launch that does not start is the safe outcome):

* `require_recorder` (default true): the launch refuses to start unless a rosbag2 recorder node
  is in the ROS graph (CLAUDE.md invariant 5: every run is logged). The check waits up to 5 s.
  Only sim tests pass false.
* `dynamics_bundle` is required, and `perception_bundle` is required with `backend:=net`.
* If either node exits for any reason the whole launch shuts down, so a node that refuses its
  bundle leaves nothing running; safety_node's watchdog then brakes on the silent /drive_raw.
"""

from __future__ import annotations

import time

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BACKENDS = ("net", "geometric")
RECORDER_NODE_SUBSTRING = "rosbag2_recorder"
RECORDER_WAIT_S = 5.0
RECORDER_REFUSAL = "refusing to drive: no rosbag recorder in the graph (every run is logged)"

# Parameter names the two nodes declare for their bundle directories.
PERCEPTION_BUNDLE_PARAM = "bundle_dir"
DYNAMICS_BUNDLE_PARAM = "dynamics_bundle_dir"


def recorder_in_graph(timeout_s: float = RECORDER_WAIT_S) -> bool:
    """True once a node whose name contains `rosbag2_recorder` is in the ROS graph, polling for
    up to `timeout_s`. Uses a short-lived rclpy node in its own context, so it neither disturbs
    nor depends on the launch's own rclpy state."""
    import rclpy

    context = rclpy.Context()
    rclpy.init(context=context)
    try:
        node = rclpy.create_node("pnp_launch_recorder_probe", context=context)
        try:
            deadline = time.monotonic() + timeout_s
            while True:
                if any(RECORDER_NODE_SUBSTRING in name for name in node.get_node_names()):
                    return True
                if time.monotonic() >= deadline:
                    return False
                time.sleep(0.1)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown(context=context)


def parameters_for(
    perception_bundle: str,
    dynamics_bundle: str,
    backend: str,
    v_max_mps: float,
    laser_yaw_from_vehicle_params: bool,
) -> tuple[dict, dict]:
    """(perception parameters, planner parameters) as plain dicts, so they can be checked.
    Raises, so the launch fails before any node starts, on a missing bundle or a bad backend."""
    if backend not in BACKENDS:
        raise RuntimeError(
            f"pnp.launch.py: backend must be one of {list(BACKENDS)}, got {backend!r}"
        )
    if not dynamics_bundle:
        raise RuntimeError(
            "pnp.launch.py: dynamics_bundle:=<dynamics bundle directory> is required"
        )
    if backend == "net" and not perception_bundle:
        raise RuntimeError(
            "pnp.launch.py: perception_bundle:=<track_perception bundle directory> is required "
            "with backend:=net"
        )
    perception: dict = {
        "backend": backend,
        "laser_yaw_from_vehicle_params": laser_yaw_from_vehicle_params,
    }
    if perception_bundle:
        perception[PERCEPTION_BUNDLE_PARAM] = perception_bundle
    planner: dict = {
        DYNAMICS_BUNDLE_PARAM: dynamics_bundle,
        "v_max_mps": float(v_max_mps),
        "laser_yaw_from_vehicle_params": laser_yaw_from_vehicle_params,
    }
    return perception, planner


def nodes_for(perception_params: dict, planner_params: dict, params_file: str) -> tuple[Node, Node]:
    # A params file comes first so the explicit launch values above it win.
    base = [params_file] if params_file else []
    perception = Node(
        package="racer_track_perception",
        executable="track_perception_node",
        name="track_perception_node",
        output="screen",
        parameters=[*base, perception_params],
    )
    planner = Node(
        package="racer_planner",
        executable="mppi_planner_node",
        name="mppi_planner_node",
        output="screen",
        parameters=[*base, planner_params],
    )
    return perception, planner


def _bool(value: str, name: str) -> bool:
    value = value.strip().lower()
    if value not in ("true", "false"):
        raise RuntimeError(f"pnp.launch.py: {name} must be true or false, got {value!r}")
    return value == "true"


def _make_actions(context: LaunchContext) -> list:
    def arg(name: str) -> str:
        return LaunchConfiguration(name).perform(context).strip()

    perception_params, planner_params = parameters_for(
        arg("perception_bundle"),
        arg("dynamics_bundle"),
        arg("backend"),
        float(arg("v_max_mps")),
        _bool(arg("laser_yaw_from_vehicle_params"), "laser_yaw_from_vehicle_params"),
    )
    if _bool(arg("require_recorder"), "require_recorder") and not recorder_in_graph():
        raise RuntimeError(RECORDER_REFUSAL)
    perception, planner = nodes_for(perception_params, planner_params, arg("params_file"))
    stop_all = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[EmitEvent(event=Shutdown(reason=f"{name} exited"))],
            )
        )
        for node, name in ((perception, "track_perception_node"), (planner, "mppi_planner_node"))
    ]
    return [perception, planner, *stop_all]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "perception_bundle",
                default_value="",
                description="track_perception bundle directory. Required with backend:=net.",
            ),
            DeclareLaunchArgument(
                "dynamics_bundle",
                default_value="",
                description="dynamics bundle directory (required).",
            ),
            DeclareLaunchArgument(
                "backend",
                default_value="net",
                choices=list(BACKENDS),
                description="Track perception backend: net or geometric.",
            ),
            DeclareLaunchArgument(
                "v_max_mps",
                default_value="0.5",
                description="Speed cap in m/s passed to the planner.",
            ),
            DeclareLaunchArgument(
                "require_recorder",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Refuse to start unless a rosbag recorder node is in the ROS graph. "
                    "false only in sim tests."
                ),
            ),
            DeclareLaunchArgument(
                "laser_yaw_from_vehicle_params",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "true on the car: the LiDAR mount yaw comes from vehicle_params. false only "
                    "in the simulator, whose scan is already in the vehicle frame."
                ),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value="",
                description="Optional ROS parameter YAML applied to both nodes.",
            ),
            OpaqueFunction(function=_make_actions),
        ]
    )
