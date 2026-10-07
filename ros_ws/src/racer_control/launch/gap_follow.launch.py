"""Minimal launch file for gap_follow_node (GitHub issue 26).

Lives in racer_control/launch alongside tracker.launch.py, for the same reason given there
(claude-docs/10-conventions.md puts launch files in racer_bringup, which has no launch
infrastructure for these controllers yet). The node subscribes /scan and publishes /drive_raw
only: run it with racer_safety's safety_node, which gates /drive_raw -> /drive. Nothing here
remaps /drive_raw onto /drive.

Every tuning parameter is a declared ROS parameter with a default (see
src/gap_follow_node.cpp). `max_speed_mps` is exposed here because it is the one most often
changed per session; `params_file` takes a ROS parameter YAML for the rest.

`profile` picks a named parameter set from PROFILES below: `none` (default) passes nothing
extra, so the node defaults apply as before; `floor-2026-10-06` is FLOOR_2026_10_06_PROFILE,
the parameters of the first working floor laps (a checkpoint, not a tuned optimum).
Precedence, lowest to highest: node defaults, `params_file`, the profile, then any launch
argument given explicitly here. So `profile:=floor-2026-10-06 max_speed_mps:=0.7` runs the
profile at 0.7 m/s.

`max_speed_mps` defaults to empty: the profile's value if it has one, otherwise 2.0, which
(as before this argument could be empty) is passed to the node and overrides the params file.

`forward_preference` and `gap_switch_margin` (gap selection weighting and hysteresis, see
include/racer_control/gap_follow.hpp GapPreference; both [0, 1]) are launch arguments too,
for floor tuning. Their launch defaults are empty, meaning "not set here": the profile value,
the params file value or the node default (0.0, off) applies. A value given here is passed as
a float and overrides all of those. The node range-checks them and refuses to start outside
[0, 1].

`swept_path_clamp` (true/false) and `swept_path_lookahead_m` (the swept-path steering clamp,
see include/racer_control/gap_follow.hpp clamp_steering_to_swept_path) follow the same rule:
empty leaves the profile, params file or node default (on, 1.0 m). The node refuses to start
while vehicle_params sensors.lidar.mount_x_m / mount_y_m is null.

`speed_time_constant_s` (low-pass on the speed command before the rate limiter, [0, 5] s,
0 = off) and `target_range_median_scans` (median of the last N scans' target ranges, integer
[1, 15], 1 = off) follow the same rule too; see include/racer_control/reactive_speed.hpp.

`centering_gain` (lane centring, [0, 5] rad, 0 = off; see include/racer_control/gap_follow.hpp
measure_lane_walls) follows the same rule. The profile turns it on at 0.6.

`reverse_escape` (true/false) and `escape_probe_distance_m` (see
include/racer_control/reverse_escape.hpp) follow the same rule. WITH reverse_escape TRUE THE CAR
CAN REVERSE ON ITS OWN: when safety_node has refused the forward request for escape_after_s and
the follower sees no clear forward arc, gap_follow_node requests reverse through /drive_raw
(gated by safety_node's rear corridor). The profile turns it on; `reverse_escape:=false` turns
it off for a session.

laser_yaw_from_vehicle_params (default true) must stay true on the real car, where the LiDAR
yaw comes only from vehicle_params sensors.lidar.mount_yaw_rad. Set it false (in the params
file) ONLY for the simulator and synthetic-scan tests, whose /scan is aligned to the vehicle
(yaw 0); the node then uses laser_yaw_offset_rad as given and logs that it ignored the binding.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_NODE = "gap_follow_node"
_DEFAULT_MAX_SPEED_MPS = 2.0

FLOOR_2026_10_06_PROFILE: dict[str, float | int | bool] = {
    "min_speed_mps": 0.5,
    "max_speed_mps": 0.9,
    "k_steer": 0.4,
    "free_space_threshold_m": 0.6,
    "steering_gain": 1.6,
    "steering_time_constant_s": 0.15,
    "disparity_threshold_m": 0.3,
    "cone_half_angle_rad": 1.2,
    "forward_preference": 0.0,
    "gap_switch_margin": 0.0,
    "swept_path_clamp": True,
    "swept_path_lookahead_m": 0.6,
    "corner_sector_inner_rad": 0.4,
    "corner_sector_outer_rad": 1.6,
    "corner_min_clearance_m": 0.35,
    "speed_rate_limit_margin_fraction": 0.1,
    "target_deepest_ray": False,
    # Speed smoothing added after the laps, for the surging the owner reported on them.
    "speed_time_constant_s": 0.5,
    "target_range_median_scans": 5,
    # Lane centring added after the night's bags (2026-10-06T23-19-41 and 23-25-38) showed the
    # follower hugging the edges. 0.6 rad on the 1.0 to 1.2 m lane: an offset e from the lane
    # centre pushes 2 * 0.6 / W, about 1.1 rad per metre at W = 1.1 m, which is the steering a
    # pure-pursuit point about 0.8 m ahead would ask for (2 L e / Ld^2 with L = 0.33 m), about
    # the turning radius and the swept-path lookahead. Not yet driven on the floor.
    "centering_gain": 0.6,
    "centering_sector_half_angle_rad": 1.0,
    "centering_max_range_m": 1.5,
    # Reverse escape added after the same bags showed the car stuck nose-in to corners tighter
    # than its turning circle. THE CAR CAN REVERSE ON ITS OWN WITH THIS PROFILE. The escape_*
    # timings are the node defaults (1.5 s, -0.5 m/s, 0.4 m, 2.0 s, retry 3.0 s, 3 attempts).
    # Not yet driven on the floor.
    "reverse_escape": True,
}
"""Checkpoint 2026-10-06: the first working floor laps.

gap_follow_node completed its first working laps on the floor of the owner's living-room lane
on the evening of 2026-10-06 with every value above up to and including target_deepest_ray.
The rest were added later the same night, at the owner's request, and have NOT been driven on
the floor yet: the speed smoothing (the speed surged and slowed with the scan-to-scan flicker
of the target range), the lane centring (the follower hugged the edges in the night's bags)
and the reverse escape (it ended nose-in to corners tighter than its turning circle; WITH IT
ON THE CAR CAN REVERSE ON ITS OWN). To drive exactly the laps' parameters, override them back
off: `speed_time_constant_s:=0 target_range_median_scans:=1 centering_gain:=0
reverse_escape:=false`. See
docs/notes/reactive-control-port-2026-10-05.md, "Checkpoint 2026-10-06: first working floor
laps", for the known limits.

This is a CHECKPOINT to get back to a known working state, NOT a tuned optimum: it is one
evening's hand tuning on one lane at 0.5 to 0.9 m/s. Record a later tuned set as a new profile
with its own date rather than editing the values above; try new values with launch arguments
or a params file on top of it.
"""

PROFILES: dict[str, dict[str, float | int | bool]] = {
    "none": {},
    "floor-2026-10-06": FLOOR_2026_10_06_PROFILE,
}

# Launch arguments forwarded to the node only when set (empty = profile, params file or node
# default).
_OPTIONAL_FLOAT_ARGS = {
    "forward_preference": (
        "Gap selection weighting toward forward, [0, 1]. Empty (default) leaves the profile "
        "value, the params file value or the node default 0.0 (plain widest gap)."
    ),
    "gap_switch_margin": (
        "Gap switching hysteresis fraction, [0, 1]. Empty (default) leaves the profile value, "
        "the params file value or the node default 0.0 (off)."
    ),
    "swept_path_lookahead_m": (
        "Arc length (m) within which returns constrain the swept-path clamp. Empty (default) "
        "leaves the profile value, the params file value or the node default 1.0."
    ),
    "speed_time_constant_s": (
        "Low-pass time constant on the speed command (s), before the rate limiter, [0, 5]. "
        "Empty (default) leaves the profile value, the params file value or the node default "
        "0.0 (off)."
    ),
    "centering_gain": (
        "Lane centring gain (rad), [0, 5]: a push away from the nearer side wall, added before "
        "the steering clamp. Empty (default) leaves the profile value, the params file value or "
        "the node default 0.0 (off)."
    ),
    "escape_probe_distance_m": (
        "Reverse escape: the forward path is blocked when no steering gives this much clear "
        "travel (m), [0.02, 3]. Empty (default) leaves the profile value, the params file value "
        "or the node default 0.3."
    ),
}
_OPTIONAL_INT_ARGS = {
    "target_range_median_scans": (
        "Median of the last N scans' target ranges feeds the speed law, integer [1, 15]. Empty "
        "(default) leaves the profile value, the params file value or the node default 1 (off)."
    ),
}
_OPTIONAL_BOOL_ARGS = {
    "swept_path_clamp": (
        "true or false: reduce the steering so the car's swept area clears returns beside it "
        "on the turn-in side. Empty (default) leaves the profile value, the params file value "
        "or the node default (true)."
    ),
    "reverse_escape": (
        "true or false: let gap_follow_node REVERSE ON ITS OWN out of a corner it cannot drive "
        "round (through safety_node's rear corridor). Empty (default) leaves the profile value "
        "(true in floor-2026-10-06), the params file value or the node default (false)."
    ),
}


def _profile(context: LaunchContext) -> dict[str, float | int | bool]:
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
    max_speed = LaunchConfiguration("max_speed_mps").perform(context).strip()
    if max_speed:
        overrides["max_speed_mps"] = float(max_speed)
    elif "max_speed_mps" not in profile:
        overrides["max_speed_mps"] = _DEFAULT_MAX_SPEED_MPS
    for name in _OPTIONAL_FLOAT_ARGS:
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            # float() so "0" or "1" is not handed to the node as an integer, which a double
            # parameter would reject.
            overrides[name] = float(value)
    for name in _OPTIONAL_INT_ARGS:
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            # int() so "5" is not handed over as a string, and "5.0" is refused here rather
            # than silently truncated.
            overrides[name] = int(value)
    for name in _OPTIONAL_BOOL_ARGS:
        value = LaunchConfiguration(name).perform(context).strip().lower()
        if value:
            if value not in ("true", "false"):
                raise ValueError(f"{name} must be true or false, got {value!r}")
            overrides[name] = value == "true"
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
            "Named parameter set: none (node defaults) or floor-2026-10-06 (the first working "
            "floor laps, a checkpoint, not a tuned optimum). Explicit launch arguments override "
            "it; it overrides params_file."
        ),
    )
    max_speed_arg = DeclareLaunchArgument(
        "max_speed_mps",
        default_value="",
        description=(
            "Upper speed bound in m/s (range-limited to vehicle_params' global cap). Empty "
            f"(default): the profile's value, or {_DEFAULT_MAX_SPEED_MPS} without one."
        ),
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value="",
        description="Optional ROS parameter YAML for the remaining tuning parameters.",
    )
    optional_args = [
        DeclareLaunchArgument(name, default_value="", description=description)
        for name, description in {
            **_OPTIONAL_FLOAT_ARGS,
            **_OPTIONAL_INT_ARGS,
            **_OPTIONAL_BOOL_ARGS,
        }.items()
    ]
    return LaunchDescription(
        [
            profile_arg,
            max_speed_arg,
            params_file_arg,
            *optional_args,
            OpaqueFunction(function=_make_node),
        ]
    )
