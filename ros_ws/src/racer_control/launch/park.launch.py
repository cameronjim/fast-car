"""Launch file for park_node (roadmap 2.9): parallel park and three-point turn.

Same shape as gap_follow.launch.py. park_node subscribes /scan, /odom/wheel and the gated /drive
and publishes /drive_raw only: run it next to the car stack (racer_bringup's
car_teleop.launch.py with lidar:=true vesc:=true), whose safety_node gates /drive_raw -> /drive
and whose recorder logs the run. Nothing here remaps /drive_raw onto /drive.

THE NODE DRIVES THE CAR, INCLUDING IN REVERSE, ON ITS OWN once started. It comes up IDLE and
publishes nothing until a service call:

    ros2 service call /park_node/start std_srvs/srv/Trigger              # parallel park
    ros2 service call /park_node/three_point_turn std_srvs/srv/Trigger   # three-point turn
    ros2 service call /park_node/abort std_srvs/srv/Trigger              # stop whatever runs

`profile` picks a named parameter set from PROFILES below: `none` (default) passes nothing extra,
so the node defaults apply; `floor-2026-10-07` is FLOOR_2026_10_07_PROFILE, the first floor
settings. Precedence, lowest to highest: node defaults, `params_file`, the profile, then any
launch argument given explicitly here (empty = not given). See docs/notes/parking-2026-10-07.md.

laser_yaw_from_vehicle_params (default true) must stay true on the car; only synthetic-scan
fixtures set it false (in a params file).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_NODE = "park_node"

FLOOR_2026_10_07_PROFILE: dict[str, float | int | bool | str] = {
    "park_side": "right",
    "search_speed_mps": 0.5,
    "park_speed_mps": -0.5,
    "settle_s": 0.3,
    "require_odometry": True,
    "margin_m": 0.1,
    "park_steering_fraction": 0.95,
    "blocked_abort_s": 2.0,
    "final_forward_m": 0.15,
}
"""First floor settings, 2026-10-07 (NOT yet driven on the car).

0.5 m/s both ways: the slowest the VESC speed loop reliably drives (it does nothing below about
0.44 m/s, s_pid_min_erpm). settle_s 0.3 s: the servo slews at most 3.2 rad/s
(steering.max_rate_rad_per_s), so full lock to full lock (0.80 rad at the 0.95 fraction) takes
0.25 s, and park_node ramps it at half that rate first. require_odometry true: the arcs are
measured with the VESC's wheel odometry (/odom/wheel), never guessed from the command on the car.
0.1 m margins: the planned swept body keeps 0.1 m from every face the LiDAR saw, and the parked
car's near side sits 0.1 m inside the row line. The swept-body check uses the PROVISIONAL
overhangs (vehicle_params chassis.front_overhang_m / rear_overhang_m): measure them first.

Record a later tuned set as a new dated profile rather than editing this one.
"""

PROFILES: dict[str, dict[str, float | int | bool | str]] = {
    "none": {},
    "floor-2026-10-07": FLOOR_2026_10_07_PROFILE,
}

_OPTIONAL_FLOAT_ARGS = {
    "search_speed_mps": "SEARCH and every forward move (m/s). Empty: profile or node default 0.5.",
    "park_speed_mps": "Every reverse move (m/s, negative). Empty: profile or node default -0.5.",
    "park_steering_fraction": (
        "Arc steering as a fraction of steering.max_angle_rad. Empty: profile or node default 0.95."
    ),
    "margin_m": "Planning clearance (m). Empty: profile or node default 0.1.",
    "settle_s": "Standstill after a steering change (s). Empty: profile or node default 0.3.",
    "final_forward_m": "Largest forward centring move (m). Empty: profile or node default 0.15.",
    "blocked_abort_s": (
        "ABORT after safety_node holds the car this long (s). Empty: profile or node default 2.0."
    ),
}
_OPTIONAL_BOOL_ARGS = {
    "require_odometry": (
        "true or false: refuse to run without /odom/wheel. Empty: profile or node default (true). "
        "false is for simulator fixtures only."
    ),
}
_OPTIONAL_CHOICE_ARGS = {
    "park_side": ("right", "left"),
    "turn_direction": ("left", "right"),
}


def _profile(context: LaunchContext) -> dict[str, float | int | bool | str]:
    name = LaunchConfiguration("profile").perform(context).strip()
    if name not in PROFILES:
        raise ValueError(f"profile must be one of {sorted(PROFILES)}, got {name!r}")
    return dict(PROFILES[name])


def _make_node(context: LaunchContext) -> list[Node]:
    parameters: list = []
    params_file = LaunchConfiguration("params_file").perform(context)
    if params_file:
        parameters.append(params_file)
    profile = _profile(context)
    if profile:
        parameters.append(profile)
    overrides: dict = {}
    for name in _OPTIONAL_FLOAT_ARGS:
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            # float() so "0" is not handed to the node as an integer.
            overrides[name] = float(value)
    for name in _OPTIONAL_BOOL_ARGS:
        value = LaunchConfiguration(name).perform(context).strip().lower()
        if value:
            if value not in ("true", "false"):
                raise ValueError(f"{name} must be true or false, got {value!r}")
            overrides[name] = value == "true"
    for name, choices in _OPTIONAL_CHOICE_ARGS.items():
        value = LaunchConfiguration(name).perform(context).strip().lower()
        if value:
            if value not in choices:
                raise ValueError(f"{name} must be one of {choices}, got {value!r}")
            overrides[name] = value
    if overrides:
        parameters.append(overrides)
    return [
        Node(
            package="racer_control",
            executable=_NODE,
            name=_NODE,
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    profile_arg = DeclareLaunchArgument(
        "profile",
        default_value="none",
        choices=list(PROFILES),
        description=(
            "Named parameter set: none (node defaults) or floor-2026-10-07 (first floor settings). "
            "Explicit launch arguments override it; it overrides params_file."
        ),
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value="",
        description="Optional ROS parameter YAML for the remaining parameters.",
    )
    optional_args = [
        DeclareLaunchArgument(name, default_value="", description=description)
        for name, description in {**_OPTIONAL_FLOAT_ARGS, **_OPTIONAL_BOOL_ARGS}.items()
    ]
    choice_args = [
        DeclareLaunchArgument(
            name,
            default_value="",
            description=f"One of {', '.join(choices)}. Empty: profile or node default.",
        )
        for name, choices in _OPTIONAL_CHOICE_ARGS.items()
    ]
    return LaunchDescription(
        [
            profile_arg,
            params_file_arg,
            *optional_args,
            *choice_args,
            OpaqueFunction(function=_make_node),
        ]
    )
