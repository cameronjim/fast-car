"""Minimal launch file for gap_follow_node (GitHub issue 26).

Lives in racer_control/launch alongside tracker.launch.py, for the same reason given there
(claude-docs/10-conventions.md puts launch files in racer_bringup, which has no launch
infrastructure for these controllers yet). The node subscribes /scan and publishes /drive_raw
only: run it with racer_safety's safety_node, which gates /drive_raw -> /drive. Nothing here
remaps /drive_raw onto /drive.

Every tuning parameter is a declared ROS parameter with a default (see
src/gap_follow_node.cpp). `max_speed_mps` is exposed here because it is the one most often
changed per session; `params_file` takes a ROS parameter YAML for the rest.

`forward_preference` and `gap_switch_margin` (gap selection weighting and hysteresis, see
include/racer_control/gap_follow.hpp GapPreference; both [0, 1]) are launch arguments too,
for floor tuning. Their launch defaults are empty, meaning "not set here": the node default
(0.0, off) or the params file value applies. A value given here is passed as a float and
overrides the params file. The node range-checks them and refuses to start outside [0, 1].

`swept_path_clamp` (true/false) and `swept_path_lookahead_m` (the swept-path steering clamp,
see include/racer_control/gap_follow.hpp clamp_steering_to_swept_path) follow the same rule:
empty leaves the node default (on, 1.0 m) or the params file value. The node refuses to start
while vehicle_params sensors.lidar.mount_x_m / mount_y_m is null.

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
_DEFAULT_MAX_SPEED_MPS = "2.0"
# Launch arguments forwarded to the node only when set (empty = node default or params file).
_OPTIONAL_FLOAT_ARGS = {
    "forward_preference": (
        "Gap selection weighting toward forward, [0, 1]. Empty (default) leaves the node "
        "default 0.0 (plain widest gap) or the params file value."
    ),
    "gap_switch_margin": (
        "Gap switching hysteresis fraction, [0, 1]. Empty (default) leaves the node default "
        "0.0 (off) or the params file value."
    ),
    "swept_path_lookahead_m": (
        "Arc length (m) within which returns constrain the swept-path clamp. Empty (default) "
        "leaves the node default 1.0 or the params file value."
    ),
}
_OPTIONAL_BOOL_ARGS = {
    "swept_path_clamp": (
        "true or false: reduce the steering so the car's swept area clears returns beside it "
        "on the turn-in side. Empty (default) leaves the node default (true) or the params "
        "file value."
    ),
}


def _make_node(context: LaunchContext) -> list[Node]:
    parameters: list = []
    params_file = LaunchConfiguration("params_file").perform(context)
    if params_file:
        parameters.append(params_file)
    overrides: dict = {"max_speed_mps": LaunchConfiguration("max_speed_mps")}
    for name in _OPTIONAL_FLOAT_ARGS:
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            # float() so "0" or "1" is not handed to the node as an integer, which a double
            # parameter would reject.
            overrides[name] = float(value)
    for name in _OPTIONAL_BOOL_ARGS:
        value = LaunchConfiguration(name).perform(context).strip().lower()
        if value:
            if value not in ("true", "false"):
                raise ValueError(f"{name} must be true or false, got {value!r}")
            overrides[name] = value == "true"
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
    max_speed_arg = DeclareLaunchArgument(
        "max_speed_mps",
        default_value=_DEFAULT_MAX_SPEED_MPS,
        description="Upper speed bound in m/s (range-limited to vehicle_params' global cap).",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value="",
        description="Optional ROS parameter YAML for the remaining tuning parameters.",
    )
    optional_args = [
        DeclareLaunchArgument(name, default_value="", description=description)
        for name, description in {**_OPTIONAL_FLOAT_ARGS, **_OPTIONAL_BOOL_ARGS}.items()
    ]
    return LaunchDescription(
        [max_speed_arg, params_file_arg, *optional_args, OpaqueFunction(function=_make_node)]
    )
