"""spiderbot.launch.py -- waypoint patrol with a pose-triggered stop (roadmap 5.3). OFF BY DEFAULT.

    ros2 launch racer_apps spiderbot.launch.py patrol:=/workspace/data/patrols/loop.yaml

Run it in car-stack, next to car_teleop.launch.py with `localization:=true map:=...` (it needs
the map frame and a live pose, exactly like the tracker), with the perception launch running in
the car-perception container in POSE mode (`perception.launch.py mode:=pose`, docker/car/
README.md "Perception"). car_teleop.launch.py does not include this file.

Starts:
  * racer_control's tracker_node on a raceline generated from the patrol waypoints, with its
    /drive_raw remapped to /spiderbot/patrol_cmd (so nothing reaches the safety node except
    through spiderbot_node), pose_frame:=map.
  * spiderbot_node, the ONLY thing here that publishes /drive_raw (never /drive).

The waypoint file is refused, before anything starts, if the route cannot be driven (see
racer_apps/spiderbot/patrol.py): too few or coincident waypoints, or a corner tighter than the
car's steering allows. The curvature limit and the speed ceiling come from vehicle_params.

`odom_topic` and `pose_frame` exist for the simulator, whose bridge has no localisation: there
`odom_topic:=/sim/ground_truth_odom pose_frame:=` (empty), the same adapter sim_autopilot.launch
uses. On the car leave both alone.
"""

from __future__ import annotations

import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#: Hard ceiling on the patrol speed argument, on top of vehicle_params' own limits: this
#: behaviour drives near people. Raise it deliberately, in code review, not on the command line.
MAX_PATROL_SPEED_MPS = 1.0

#: Launch argument -> (default, description). Forwarded to spiderbot_node as parameters of the
#: same name, typed by SPIDERBOT_TYPES.
SPIDERBOT_ARGUMENTS = {
    "rule": ("arms_raised", "Pose rule: person_present, arms_raised, one_arm_raised, arm_swing."),
    "rule_frames": ("3", "Consecutive frames the rule must hold before the car reacts."),
    "rule_min_score": ("0.5", "Minimum detector score for a person to count."),
    "hold_s": ("10.0", "Seconds stopped after the alert before the patrol resumes."),
    "cooldown_s": ("20.0", "Seconds after resuming during which triggers are ignored."),
    "face_enabled": ("true", "Turn to face the person (a short forward creep)."),
    "face_speed_mps": ("0.3", "Creep speed while facing."),
    "require_perception": ("true", "Stop if the detector goes quiet."),
    "output_dir": ("~/spiderbot", "Where the event log and snapshots go."),
    "objects_topic": ("/perception/objects", "Fused people input."),
    "image_topic": ("/camera/usb/image_raw/compressed", "Camera for the snapshot."),
    "speed_topic": ("/odom/wheel", "Odometry carrying the wheel speed (optional)."),
}
SPIDERBOT_TYPES = {
    "rule_frames": int,
    "rule_min_score": float,
    "hold_s": float,
    "cooldown_s": float,
    "face_enabled": bool,
    "face_speed_mps": float,
    "require_perception": bool,
}


def _launch_nodes(context):
    from racer_apps.spiderbot.patrol import (
        PatrolError,
        build_raceline,
        load_patrol,
        max_curvature_from_vehicle,
        write_raceline_csv,
    )
    from racer_tools.vehicle_params_loader import load_vehicle_params

    patrol_path = LaunchConfiguration("patrol").perform(context)
    if not patrol_path:
        raise RuntimeError("spiderbot.launch.py: patrol:=<patrol.yaml> is required")
    patrol = load_patrol(patrol_path)
    vp = load_vehicle_params()
    speed_arg = LaunchConfiguration("speed_mps").perform(context)
    speed = float(speed_arg) if speed_arg else (patrol.speed_mps or 0.5)
    ceiling = min(MAX_PATROL_SPEED_MPS, float(vp.actuation.throttle_full_scale_mps))
    if not 0.0 < speed <= ceiling:
        raise PatrolError(f"patrol speed {speed} m/s is outside (0, {ceiling}] m/s")
    curvature = max_curvature_from_vehicle(
        float(vp.chassis.wheelbase_m),
        min(abs(float(vp.steering.max_angle_rad)), abs(float(vp.steering.min_angle_rad))),
    )
    points = build_raceline(patrol.waypoints, speed, curvature)
    out_dir = tempfile.mkdtemp(prefix="spiderbot_")
    raceline_path = os.path.join(out_dir, "patrol_raceline.csv")
    write_raceline_csv(raceline_path, points, patrol.name)

    tracker_params = {"raceline_path": raceline_path}
    odom_topic = LaunchConfiguration("odom_topic").perform(context)
    pose_frame = LaunchConfiguration("pose_frame").perform(context)
    tracker_params["odom_topic"] = odom_topic
    if pose_frame:
        tracker_params["pose_frame"] = pose_frame
    tracker = Node(
        package="racer_control",
        executable="tracker_node",
        name="spiderbot_tracker",
        output="screen",
        parameters=[tracker_params],
        remappings=[("/drive_raw", "/spiderbot/patrol_cmd")],
    )
    from launch_ros.parameter_descriptions import ParameterValue

    params = {
        name: ParameterValue(LaunchConfiguration(name), value_type=SPIDERBOT_TYPES.get(name, str))
        for name in SPIDERBOT_ARGUMENTS
    }
    spiderbot = Node(
        package="racer_apps",
        executable="spiderbot_node",
        name="spiderbot_node",
        output="screen",
        parameters=[params],
    )
    return [tracker, spiderbot]


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("patrol", default_value="", description="Patrol YAML (required)."),
        DeclareLaunchArgument(
            "speed_mps",
            default_value="",
            description=f"Patrol speed, m/s (default: the file's, else 0.5; at most {MAX_PATROL_SPEED_MPS}).",
        ),
        DeclareLaunchArgument(
            "odom_topic", default_value="/odom", description="Tracker pose input (sim only)."
        ),
        DeclareLaunchArgument(
            "pose_frame", default_value="map", description="Tracker pose frame (sim: empty)."
        ),
    ]
    args += [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (default, description) in SPIDERBOT_ARGUMENTS.items()
    ]
    return LaunchDescription([*args, OpaqueFunction(function=_launch_nodes)])
