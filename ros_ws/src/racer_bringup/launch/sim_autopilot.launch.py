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

Milestone 5 adds the `track` launch argument (default `gym_oval`, UNCHANGED default
behavior): `raceline_path`'s own default is now derived from it
(`../config/tracks/<track>/raceline.csv`), so a second committed track (e.g.
`track:=oschersleben`, roadmap milestone 5's twistier second track, see
docs/notes/milestone-5-browser-teleop.md) can be selected with one argument instead of
spelling out the whole path. Passing `raceline_path` directly still works and overrides
`track` entirely (it always did; `track` only changes what `raceline_path` DEFAULTS to).

`raceline_publisher_node` (racer_tools) publishes that same raceline once as a latched
nav_msgs/Path on /sim/raceline so Foxglove can show the line the tracker is following,
alongside everything milestone 2 already shows -- see
config/foxglove_sim_autopilot.layout.json.

`viz` (default true) starts foxglove_bridge, same as sim_teleop.launch.py.

MAPPING AND LOCALISATION IN THE SIM (2026-11-03, docs/notes/localization-2026-11-03.md), both
off by default, which leaves everything above exactly as it was:

  * `mapping:=true`: the bridge stops broadcasting its ground-truth map -> base_link, puts
    base_link at the rear axle like the car, publishes /odom/wheel at 50 Hz and casts /scan like
    the car's C1 (vehicle_params sensors.lidar, 10 Hz, 12 m); racer_bringup's mapping.launch.py
    (wheel_twist_node, the EKF, slam_toolbox) runs on that. tracker_node still drives on ground
    truth, so the map gets built whatever the odometry does. Give the track walls
    (`track_half_width_m:=0.8`), or slam_toolbox has nothing to see.
  * `localization:=true map:=<yaml>`: the same bridge setup, localization.launch.py (EKF, map
    server, AMCL), and tracker_node on the EKF's /odom composed with AMCL's map -> odom
    (pose_frame map). The map must be in the sim's frame, the frame the raceline is written in
    (tests/l5_localization writes the bridge's own map as one). AMCL waits for an initial pose:
    publish one from Foxglove's pose estimate tool (/initialpose).
  * `sensor_noise:=true` adds the canaries' wheel speed, scan and steering trim errors.

safety_node keeps its car settings in these modes (laser yaw from vehicle_params), because the
bridge then casts its scan from the car's real mount.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_FOXGLOVE_BRIDGE_PORT = 8765
_DEFAULT_TRACK = "gym_oval"
_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))

#: The sensor errors sensor_noise:=true turns on; the same values as tests/l5_localization's
#: canaries (see l5_localization_common.NOISE for where each comes from).
SIM_SENSOR_NOISE = {
    "wheel_speed_scale": 1.02,
    "wheel_speed_noise_std_mps": 0.02,
    "wheel_slip_s": 0.01,
    "scan_noise_std_m": 0.02,
    "steering_offset_rad": 0.01,
}


def car_like_bridge_parameters(vehicle_params, sensor_noise: bool) -> dict:
    """Bridge parameters that make the sim look like the car to the localisation stack: no
    ground-truth TF, base_link at the rear axle, the VESC's 50 Hz /odom/wheel, the C1's scan."""
    import math

    lidar = vehicle_params.sensors.lidar
    spec = vehicle_params.sensors.lidar_spec
    beams = round(2.0 * math.pi / spec.angular_resolution_rad)
    lr = vehicle_params.chassis.cg_to_rear_axle_m
    params = {
        "publish_ground_truth_tf": False,
        "base_link_offset_x_m": -lr,
        "publish_wheel_odom": True,
        "wheel_odom_rate_hz": 50.0,
        "lidar_offset_x_m": lidar.mount_x_m - lr,
        "lidar_yaw_rad": lidar.mount_yaw_rad,
        "lidar_fov_rad": 2.0 * math.pi * (beams - 1) / beams,
        "lidar_num_beams": int(beams),
        "scan_rate_hz": spec.nominal_scan_rate_hz,
        "scan_range_max_m": spec.range_max_m,
    }
    if sensor_noise:
        params.update(SIM_SENSOR_NOISE)
    return params


def _flag(name: str, context) -> bool:
    return LaunchConfiguration(name).perform(context).lower() == "true"


def generate_launch_description() -> LaunchDescription:
    track_arg = DeclareLaunchArgument(
        "track",
        default_value=_DEFAULT_TRACK,
        description=(
            "Track id under config/tracks/<track>/ (claude-docs/02-repo-layout.md) whose "
            "raceline.csv both bridge_node and tracker_node track by default -- only used to "
            "compute raceline_path's own default (below); passing raceline_path directly "
            f"overrides this entirely. Default '{_DEFAULT_TRACK}' (unchanged from milestones "
            "3-4). See docs/notes/milestone-5-browser-teleop.md for the committed tracks and "
            "how to add another."
        ),
    )
    raceline_path_arg = DeclareLaunchArgument(
        "raceline_path",
        default_value=["../config/tracks/", LaunchConfiguration("track"), "/raceline.csv"],
        description=(
            "Path to the tools/raceline-generated raceline CSV both bridge_node and "
            "tracker_node track (claude-docs/02-repo-layout.md: "
            "config/tracks/<venue>_<layout>/raceline.csv). Relative paths resolve against "
            "the launching process's cwd -- the default assumes `ros2 launch` is run from "
            "inside `ros_ws` (see docs/notes/milestone-3-sim-autopilot.md's demo procedure); "
            "pass an absolute path if running from somewhere else. Defaults to "
            "`../config/tracks/<track>/raceline.csv` from the `track` argument above; passing "
            "this directly overrides `track` entirely."
        ),
    )
    mapping_arg = DeclareLaunchArgument(
        "mapping",
        default_value="false",
        description="Car-like sensors + mapping.launch.py (slam_toolbox). See the docstring.",
    )
    localization_arg = DeclareLaunchArgument(
        "localization",
        default_value="false",
        description=(
            "Car-like sensors + localization.launch.py (AMCL on map:=...) and tracker_node on "
            "the localised pose. See the docstring."
        ),
    )
    map_arg = DeclareLaunchArgument(
        "map", default_value="", description="Map yaml for localization:=true (sim frame)."
    )
    track_half_width_arg = DeclareLaunchArgument(
        "track_half_width_m",
        default_value="0.0",
        description=(
            "bridge_node's walled corridor half width (m); 0.0 (default) = no walls, as before. "
            "Mapping and localisation need walls (e.g. 0.8)."
        ),
    )
    sensor_noise_arg = DeclareLaunchArgument(
        "sensor_noise",
        default_value="false",
        description="With mapping/localization: the canaries' sensor errors (SIM_SENSOR_NOISE).",
    )
    viz_arg = DeclareLaunchArgument(
        "viz",
        default_value="true",
        description=(
            "Start foxglove_bridge (ros-humble-foxglove-bridge) on port "
            f"{_FOXGLOVE_BRIDGE_PORT}, same as sim_teleop.launch.py. Default true."
        ),
    )

    return_nodes = OpaqueFunction(function=_autopilot_nodes)
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
            track_arg,
            raceline_path_arg,
            viz_arg,
            mapping_arg,
            localization_arg,
            map_arg,
            track_half_width_arg,
            sensor_noise_arg,
            return_nodes,
            foxglove_bridge_node,
        ]
    )


def _autopilot_nodes(context):
    """bridge_node, safety_node, tracker_node and raceline_publisher_node; with mapping or
    localization also the car-like bridge parameters and the localisation stack."""
    mapping = _flag("mapping", context)
    localization = _flag("localization", context)
    if mapping and localization:
        raise RuntimeError("sim_autopilot.launch.py: pick one of mapping:=true, localization:=true")
    car_like = mapping or localization
    raceline_path = LaunchConfiguration("raceline_path")

    bridge_params = {
        "raceline_path": raceline_path,
        "track_half_width_m": float(LaunchConfiguration("track_half_width_m").perform(context)),
    }
    if car_like:
        from racer_tools.vehicle_params_loader import load_vehicle_params

        bridge_params.update(
            car_like_bridge_parameters(load_vehicle_params(), _flag("sensor_noise", context))
        )
    bridge_node = Node(
        package="racer_gym_bridge",
        executable="bridge_node",
        name="bridge_node",
        output="screen",
        parameters=[bridge_params],
    )
    safety_node = Node(
        package="racer_safety",
        executable="safety_node",
        name="safety_node",
        output="screen",
        # racer_gym_bridge publishes /scan aligned to the vehicle (laser yaw 0), not mounted
        # like the real car's LiDAR (vehicle_params sensors.lidar.mount_yaw_rad = pi), so the
        # sim must not apply the car's yaw to safety_node's forward sector. Sim only: the car's
        # launch files keep the default (true). The car-like modes cast from the real mount.
        parameters=[{"laser_yaw_from_vehicle_params": car_like}],
    )
    if localization:
        tracker_params = {
            "raceline_path": raceline_path,
            "odom_topic": "/odom",
            "pose_frame": "map",
        }
        tracker_remaps = []
    else:
        tracker_params = {"raceline_path": raceline_path}
        # Sim-only pose-source adapter remap -- see this file's module docstring. Only
        # tracker_node's own subscription is remapped; bridge_node's real topic name
        # (/sim/ground_truth_odom) is untouched.
        tracker_remaps = [("/odom", "/sim/ground_truth_odom")]
    tracker_node = Node(
        package="racer_control",
        executable="tracker_node",
        name="tracker_node",
        output="screen",
        parameters=[tracker_params],
        remappings=tracker_remaps,
    )
    raceline_publisher_node = Node(
        package="racer_tools",
        executable="raceline_publisher_node",
        name="raceline_publisher_node",
        output="screen",
        parameters=[{"raceline_path": raceline_path}],
    )
    actions = [bridge_node, safety_node, tracker_node, raceline_publisher_node]
    if mapping:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(_LAUNCH_DIR, "mapping.launch.py"))
            )
        )
    if localization:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(_LAUNCH_DIR, "localization.launch.py")),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "set_initial_pose": "false",
                }.items(),
            )
        )
    return actions
