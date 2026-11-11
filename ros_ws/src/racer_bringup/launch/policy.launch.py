"""policy.launch.py -- the tracker plus a learned residual on top of it (roadmap 5.2 scaffolding).

    ros2 launch racer_bringup policy.launch.py \
        policy_dir:=/workspace/data/policies/<name> raceline:=/workspace/data/maps/<t>/raceline.csv

Starts two nodes:

    tracker_node  with drive_topic:=/tracker/command   (the tracker's HANDOFF mode)
    policy_node   subscribes to it, adds the bounded residual, publishes /drive_raw

policy_node is then the ONLY publisher of /drive_raw and nothing in this file publishes /drive:
the command still goes through safety_node and the hardware mux exactly as before (CLAUDE.md
invariant 1, claude-docs/04-architecture.md). Why a handoff topic and not a second publisher on
/drive_raw: two publishers would interleave their commands at the safety node, and a mux node
would be one more thing on the command path; a renamed output on the one node that already
exists changes nothing else about it.

The tracker runs with the lookahead the policy was trained on (read from the policy
directory's policy_meta.yaml, `must_match_at_deploy.base_controller`), because the residual
learned to correct THAT base. policy_node refuses to start on any contract or sidecar mismatch
(racer_policy.deploy). When policy_node exits for any reason the whole launch shuts down, so a
refused or crashed policy never leaves a tracker running that nobody is listening to, and never
falls back to driving by itself: /drive_raw goes quiet and safety_node's watchdog brakes. Run
the tracker alone if that is what you want.

THIS DOES NOT PASS GATE G3. The tracker has not driven the real track yet; this launch exists
so the first wheels-off-ground trial is one command (docs/notes/residual-v1-2026-11-11.md).
"""

from pathlib import Path

import yaml
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

TRACKER_HANDOFF_TOPIC = "/tracker/command"


def base_controller_settings(policy_dir: Path) -> dict[str, float]:
    """The lookahead the policy trained against, from its sidecar. Raises (so the launch fails
    before any node starts) if the sidecar is missing or lacks them."""
    meta_path = policy_dir / "policy_meta.yaml"
    if not meta_path.is_file():
        raise RuntimeError(f"policy.launch.py: {meta_path} not found; not a policy directory")
    meta = yaml.safe_load(meta_path.read_text(encoding="utf-8"))
    try:
        base = meta["must_match_at_deploy"]["base_controller"]
        return {
            "lookahead_min_m": float(base["lookahead_min_m"]),
            "lookahead_max_m": float(base["lookahead_max_m"]),
            "lookahead_curvature_ref_1pm": float(base["lookahead_curvature_ref_1pm"]),
        }
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(
            f"policy.launch.py: {meta_path} has no usable must_match_at_deploy.base_controller"
        ) from exc


def parameters_for(
    policy_dir: str, raceline: str, pose_frame: str, odom_topic: str
) -> tuple[dict, dict]:
    """(tracker parameters, policy_node parameters): plain dicts, so they can be checked."""
    if not policy_dir:
        raise RuntimeError("policy.launch.py: policy_dir:=<policy directory> is required")
    if not raceline:
        raise RuntimeError("policy.launch.py: raceline:=<raceline csv> is required")
    lookahead = base_controller_settings(Path(policy_dir))
    tracker = {
        **lookahead,
        "raceline_path": raceline,
        "pose_frame": pose_frame,
        "odom_topic": odom_topic,
        "drive_topic": TRACKER_HANDOFF_TOPIC,
    }
    policy = {
        "policy_dir": policy_dir,
        "raceline_path": raceline,
        "pose_frame": pose_frame,
        "odom_topic": odom_topic,
        "tracker_command_topic": TRACKER_HANDOFF_TOPIC,
    }
    return tracker, policy


def nodes_for(
    policy_dir: str, raceline: str, pose_frame: str, odom_topic: str
) -> tuple[Node, Node]:
    tracker_params, policy_params = parameters_for(policy_dir, raceline, pose_frame, odom_topic)
    tracker = Node(
        package="racer_control",
        executable="tracker_node",
        name="tracker_node",
        output="screen",
        parameters=[tracker_params],
    )
    policy = Node(
        package="racer_policy",
        executable="policy_node",
        name="policy_node",
        output="screen",
        parameters=[policy_params],
    )
    return tracker, policy


def _make_nodes(context: LaunchContext) -> list:
    tracker, policy = nodes_for(
        LaunchConfiguration("policy_dir").perform(context).strip(),
        LaunchConfiguration("raceline").perform(context).strip(),
        LaunchConfiguration("pose_frame").perform(context).strip(),
        LaunchConfiguration("odom_topic").perform(context).strip(),
    )
    stop_all = RegisterEventHandler(
        OnProcessExit(
            target_action=policy,
            on_exit=[EmitEvent(event=Shutdown(reason="policy_node exited"))],
        )
    )
    return [tracker, policy, stop_all]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "policy_dir", default_value="", description="Policy directory (required)."
            ),
            DeclareLaunchArgument(
                "raceline", default_value="", description="Raceline CSV (required)."
            ),
            DeclareLaunchArgument(
                "pose_frame",
                default_value="map",
                description=(
                    "map: follow /odom composed with the localiser's map -> odom (the car). "
                    "Empty: use odom_topic's pose as it is (ground truth in the sim)."
                ),
            ),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            OpaqueFunction(function=_make_nodes),
        ]
    )
