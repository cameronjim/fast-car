"""track_follow.launch.py -- tracker_node on a raceline, for the car and for the sim.

    ros2 launch racer_control track_follow.launch.py raceline:=/data/maps/<track>/raceline_ccw.csv

tracker_node follows a tools/raceline CSV with pure pursuit and publishes /drive_raw; safety_node
gates it into /drive like every other source (CLAUDE.md invariant 1). Run it next to
car_teleop.launch.py with localization:=true map:=<yaml> (AMCL on the track's map, the EKF's /odom
composed with AMCL's map -> odom; `pose_frame` map) once the owner has put the car on the start
pose and sent /initialpose, or next to a sim with ground truth (pose_frame empty, odom_topic
/sim/ground_truth_rear_odom). docs/notes/first-boot-runbook.md, "New track session", is the whole
routine; this is its last step.

THE CAR CAN DRIVE ITSELF WITH THIS. Clear the lane, hold the RC kill switch, and make the first
raceline a slow one (tools/track_session raceline --v-max): the speeds in the CSV are what gets
commanded, nothing here scales them.

STEERING AS MEASURED (profile `calib-2026-11-10`, the default for `profile`). On the car as of
2026-11-10 a full command (0.4189 rad, vehicle_params steering.max_angle_rad) turns the wheels
0.504 rad to the left and 0.449 rad to the right (rear-axle turning radius 0.60 and 0.69 m), where
the software assumes 0.74 m both ways. The profile tells tracker_node the true limits and the
command each radian of true angle needs (0.831 left, 0.933 right), so it steers the angle its
geometry asks for. The speeds in the CSV are commands: until the tyre radius and gear ratio are calibrated the car
moves at 1 / 1.401 of them, so make the raceline with `--commands-read-high 1.401` to get true
speeds (tools/track_session). When the calibration PR puts the measured
numbers in config/vehicle_params.yaml, run with profile:=none and delete this profile
(CLAUDE.md invariant 2: one source of truth for physical constants).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_NODE = "tracker_node"

# The 2026-11-10 floor measurements (tools/track_session/real_car_2026-11-10.yaml, which holds the
# same numbers for the offline tools; test_tracker_launch_profile.py keeps the two in step).
CALIB_2026_11_10_PROFILE: dict[str, float] = {
    "max_steering_left_rad": 0.504,
    "max_steering_right_rad": 0.449,
    "steering_command_per_rad_left": 0.4189 / 0.504,
    "steering_command_per_rad_right": 0.4189 / 0.449,
}
"""See the module docstring. Delete when vehicle_params carries the calibration."""

PROFILES: dict[str, dict[str, float]] = {
    "none": {},
    "calib-2026-11-10": CALIB_2026_11_10_PROFILE,
}

# The pure pursuit lookahead that suits a 1 m lane (the node's default 0.4 to 1.5 m cuts the
# corners of a lane this narrow into the inside wall): see docs/notes/track-session-2026-11-11.md.
LANE_LOOKAHEAD: dict[str, float] = {
    "lookahead_min_m": 0.35,
    "lookahead_max_m": 0.7,
    "lookahead_curvature_ref_1pm": 1.25,
}

_OPTIONAL_FLOAT_ARGS = {
    "lookahead_min_m": "Pure pursuit lookahead in tight turns (m). Empty = profile / default.",
    "lookahead_max_m": "Pure pursuit lookahead on straights (m). Empty = profile / default.",
    "lookahead_curvature_ref_1pm": (
        "Curvature at which the lookahead reaches its minimum (1/m). Empty = profile / default."
    ),
    "speed_rate_limit_margin_fraction": "Fraction of the acceleration limit. Empty = default.",
}


def _make_node(context: LaunchContext) -> list[Node]:
    name = LaunchConfiguration("profile").perform(context).strip()
    if name not in PROFILES:
        raise ValueError(f"profile must be one of {sorted(PROFILES)}, got {name!r}")
    raceline = LaunchConfiguration("raceline").perform(context).strip()
    if not raceline:
        raise RuntimeError("track_follow.launch.py: raceline:=<raceline csv> is required")
    parameters: dict = {**LANE_LOOKAHEAD, **PROFILES[name], "raceline_path": raceline}
    pose_frame = LaunchConfiguration("pose_frame").perform(context).strip()
    parameters["pose_frame"] = pose_frame
    parameters["odom_topic"] = LaunchConfiguration("odom_topic").perform(context).strip()
    for arg in _OPTIONAL_FLOAT_ARGS:
        value = LaunchConfiguration(arg).perform(context).strip()
        if value:
            parameters[arg] = float(value)
    return [
        Node(
            package="racer_control",
            executable=_NODE,
            name=_NODE,
            output="screen",
            parameters=[parameters],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "raceline", default_value="", description="Raceline CSV (required)."
            ),
            DeclareLaunchArgument(
                "profile",
                default_value="calib-2026-11-10",
                choices=list(PROFILES),
                description=(
                    "none (vehicle_params only) or calib-2026-11-10 (the measured steering, "
                    "until the calibration is in vehicle_params). See the module docstring."
                ),
            ),
            DeclareLaunchArgument(
                "pose_frame",
                default_value="map",
                description=(
                    "map: follow /odom composed with AMCL's map -> odom (the car). Empty: use "
                    "odom_topic's pose as it is (ground truth in the sim)."
                ),
            ),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            *[
                DeclareLaunchArgument(arg, default_value="", description=text)
                for arg, text in _OPTIONAL_FLOAT_ARGS.items()
            ],
            OpaqueFunction(function=_make_node),
        ]
    )
