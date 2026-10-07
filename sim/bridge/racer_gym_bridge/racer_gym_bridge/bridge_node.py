"""ROS 2 node wrapping the pinned f1tenth_gym env (roadmap task 0.5).

Bridges the gym simulator to the standard racer topics
(claude-docs/04-architecture.md):

  - publishes ``/scan`` (sensor_msgs/LaserScan, best_effort) from the gym's
    LiDAR model.
  - publishes ``/sim/ground_truth_odom`` (nav_msgs/Odometry, reliable) from
    gym ground truth -- NOT ``/odom``, which claude-docs/04-architecture.md
    reserves for the real EKF (racer_state).
  - subscribes to ``/drive`` (ackermann_msgs/AckermannDriveStamped,
    reliable) and steps the sim from the latest received command.
  - offers ``/sim/reset`` (std_srvs/Trigger) so tests and tooling can reset
    the episode deterministically.
  - (roadmap 2.9, all off by default) for park_node's L5 scenario: a POCKET cut into
    one corridor wall (``pocket_*``), ``/odom/wheel`` standing in for
    racer_drivers/vesc_odometry_node (``publish_wheel_odom``), a /scan cast from a
    LiDAR mounted like the real car's (``lidar_*``), and the gym's speed controller
    mirrored for reverse (``mirror_reverse_speed_control``). See the parameter
    descriptions below.
  - (milestone 2) publishes ``/sim/map`` (nav_msgs/OccupancyGrid, latched
    via transient_local) once, built from the env's own ``Track``
    (``track.occupancy_map`` / ``track.spec``) -- see
    ``racer_gym_bridge.conversions.build_occupancy_grid_fields`` -- and
    broadcasts ``map`` -> ``base_link`` on ``/tf`` every step from the same
    ground truth pose already used for ``/sim/ground_truth_odom``, plus a
    static ``base_link`` -> ``laser`` transform so ``/scan`` renders
    correctly aligned in Foxglove/RViz. REP-105 naming
    (claude-docs/04-architecture.md): ``map`` and ``base_link`` are real
    REP-105 frames; ``map`` -> ``base_link`` direct from sim ground truth is
    a documented simplification for visualization only -- there is no
    localization stack yet (roadmap Phase 2), so this is not standing in
    for a real ``map`` -> ``odom`` -> ``base_link`` chain.

Vehicle physical parameters (mass, wheelbase, friction, ...) are the
f1tenth_gym env's own defaults. ``config/vehicle_params.yaml`` (roadmap
task 0.7) does not exist yet; when it lands, this node's env ``params``
config should be built from it instead of the gym defaults, per CLAUDE.md
hard invariant 2 (one source of truth for physical constants). Track data
(``config/tracks/``, also task 0.7-adjacent) does not exist yet either, so
this node builds a synthetic, network-free reference-line track -- the
same approach ``docker/sim-cpu/smoke_test.py`` uses -- rather than a named
map, which would fetch from api.f1tenth.org on first use.
"""

from __future__ import annotations

import dataclasses
import math

import gymnasium as gym
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from f1tenth_gym.envs.track import Track
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from racer_gym_bridge.conversions import (
    build_occupancy_grid_fields,
    build_odom_fields,
    build_scan_fields,
    drive_cmd_to_action,
    mirrored_speed_to_accel,
    yaw_to_quaternion,
)
from racer_gym_bridge.track_loader import (
    build_corridor_occupancy,
    carve_pocket,
    load_raceline_xy_speed,
)

_MAP_FRAME_ID = "map"
_BASE_LINK_FRAME_ID = "base_link"
_LASER_FRAME_ID = "laser"

_ODOM_FRAME_ID = "odom"
_GYM_ENV_ID = "f1tenth_gym:f1tenth-v0"
# f1tenth_gym's default control input is ["speed", "steering_angle"]; with
# mirror_reverse_speed_control the bridge computes the acceleration itself.
_ACCEL_CONTROL_INPUT = ["accl", "steering_angle"]
_EGO_AGENT_ID = "agent_0"
_OBSERVATION_FEATURES = [
    "scan",
    "pose_x",
    "pose_y",
    "pose_theta",
    "linear_vel_x",
    "linear_vel_y",
    "ang_vel_z",
]


def build_synthetic_track() -> Track:
    """A deterministic, network-free reference-line track.

    Identical construction to ``docker/sim-cpu/smoke_test.py``'s
    ``build_env()``: a named map (e.g. "Spielberg") would fetch from
    api.f1tenth.org on first use, which this bridge -- meant to run
    headlessly in CI and on a laptop with no network -- must not depend on.
    Real venue tracks land with ``config/tracks/`` (roadmap task 0.7 area).
    """
    xs = np.linspace(0, 50, 100)
    ys = np.sin(xs / 3.0) * 3.0
    velxs = np.full_like(xs, 3.0)
    return Track.from_refline(x=xs, y=ys, velx=velxs)


@dataclasses.dataclass(frozen=True)
class Pocket:
    """A pocket cut into one corridor wall (roadmap 2.9, ``track_loader.carve_pocket``)."""

    start_s_m: float
    length_m: float
    depth_m: float
    side: str


def build_track_from_raceline(
    raceline_path: str,
    track_half_width_m: float = 0.0,
    reverse_direction: bool = False,
    pocket: Pocket | None = None,
) -> Track:
    """A closed-loop track from a committed raceline file (roadmap task S.2).

    Reuses the raceline's own x/y centerline and target-speed columns as the reference
    line f1tenth_gym's ``Track.from_refline`` needs -- see
    ``sim/bridge/racer_gym_bridge/racer_gym_bridge/track_loader.py``'s docstring for the
    shared CSV format (also parsed independently by
    ``ros_ws/src/racer_control/include/racer_control/raceline.hpp`` in C++). Unlike
    ``build_synthetic_track``'s open reference line, a raceline generated by
    ``tools/raceline`` for a closed-loop track (e.g. the committed ``config/tracks/gym_oval``)
    produces an actual closed loop, which is what f1tenth_gym's lap-counting (crossing the
    start/finish gate) needs to mean anything.

    ``track_half_width_m`` (GitHub issue 26): 0 (the default) keeps ``Track.from_refline``'s
    occupancy map, which is free EVERYWHERE: no walls, so ``/scan`` sees nothing but the
    map edge and the gym's collision flag cannot fire. That is fine for the odometry-driven
    tracker canary and is unchanged. A positive value replaces the map with a walled
    corridor of that half width around the raceline
    (``track_loader.build_corridor_occupancy``), at the same 0.05 m resolution
    ``from_refline`` uses, so a LiDAR-driven controller has real walls to react to. The
    corridor width is a property of the simulated TRACK, not of the vehicle, so it is a
    bridge parameter rather than a vehicle_params field.

    ``reverse_direction`` (2026-10-06 night, the reactive canary in both lap directions):
    False (the default) keeps the raceline's own order. True reverses the waypoint order before
    the track is built. f1tenth_gym's reset (the default ``rl_grid_static`` reset function)
    puts the car on the raceline's FIRST waypoint facing along the raceline, so reversing the
    order is what turns the car round: it starts at the old last waypoint (next to the old
    first one on a closed loop) facing the other way round the loop. The corridor walls are
    built from the same points, so they do not change.

    ``pocket`` (roadmap 2.9, park_node's L5 scenario; needs ``track_half_width_m`` > 0): a
    recess in one wall, measured along the raceline AS DRIVEN (after ``reverse_direction``)
    from its first point (``track_loader.carve_pocket``). The map border grows to fit it.
    """
    x, y, velx = load_raceline_xy_speed(raceline_path)
    if reverse_direction:
        x, y, velx = x[::-1].copy(), y[::-1].copy(), velx[::-1].copy()
    track = Track.from_refline(x=x, y=y, velx=velx)
    if pocket is not None and track_half_width_m <= 0.0:
        raise ValueError("a pocket needs a walled corridor (track_half_width_m > 0)")
    if track_half_width_m > 0.0:
        border_m = 1.0 if pocket is None else max(1.0, pocket.depth_m + 0.5)
        occupancy, origin = build_corridor_occupancy(
            x, y, track_half_width_m, track.spec.resolution, border_m=border_m
        )
        if pocket is not None:
            occupancy = carve_pocket(
                occupancy,
                origin,
                track.spec.resolution,
                x,
                y,
                track_half_width_m,
                pocket.start_s_m,
                pocket.length_m,
                pocket.depth_m,
                pocket.side,
            )
        track.occupancy_map = occupancy
        track.spec = dataclasses.replace(track.spec, origin=origin)
    return track


def build_env(
    seed: int,
    raceline_path: str = "",
    track_half_width_m: float = 0.0,
    reverse_direction: bool = False,
    pocket: Pocket | None = None,
    accel_control: bool = False,
) -> gym.Env:
    """Construct the pinned f1tenth_gym env: single ego agent, headless.

    ``raceline_path`` is optional: empty (the default) keeps this node's original
    network-free synthetic reference line (``build_synthetic_track``), unchanged from
    roadmap task 0.5. When set (roadmap task S.2's L5 tracker lap canary is the only
    current caller -- see its launch file), the env's map is built from that raceline
    file instead via ``build_track_from_raceline``, giving a real closed-loop track a
    tracker can complete laps of.
    """
    track = (
        build_track_from_raceline(raceline_path, track_half_width_m, reverse_direction, pocket)
        if raceline_path
        else build_synthetic_track()
    )
    config = {
        "seed": seed,
        "map": track,
        "num_agents": 1,
        "ego_idx": 0,
        "observation_config": {"type": "features", "features": _OBSERVATION_FEATURES},
    }
    if accel_control:
        config["control_input"] = list(_ACCEL_CONTROL_INPUT)
    return gym.make(_GYM_ENV_ID, config=config, render_mode=None)


class BridgeNode(Node):
    """Steps a pinned f1tenth_gym env and bridges it to ROS topics."""

    def __init__(self) -> None:
        super().__init__("bridge_node")

        seed_descriptor = ParameterDescriptor(
            description="Seed used for env construction and every /sim/reset call.",
            integer_range=[IntegerRange(from_value=0, to_value=2**31 - 1, step=1)],
        )
        self._seed = int(self.declare_parameter("seed", 42, seed_descriptor).value)

        raceline_path_descriptor = ParameterDescriptor(
            description=(
                "Optional path to a tools/raceline-generated raceline CSV (roadmap task "
                "S.2). Empty (default) keeps the original synthetic, network-free "
                "reference line from roadmap task 0.5; when set, builds a closed-loop "
                "track from that raceline instead (see build_track_from_raceline)."
            ),
        )
        self._raceline_path = str(
            self.declare_parameter("raceline_path", "", raceline_path_descriptor).value
        )

        track_half_width_descriptor = ParameterDescriptor(
            description=(
                "GitHub issue 26: half width (m) of a walled corridor built around the "
                "raceline so /scan sees real track walls. 0.0 (default) keeps the original "
                "wall-free map. Only used with raceline_path."
            ),
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=50.0, step=0.0)],
        )
        self._track_half_width_m = float(
            self.declare_parameter("track_half_width_m", 0.0, track_half_width_descriptor).value
        )

        reverse_direction_descriptor = ParameterDescriptor(
            description=(
                "Drive the raceline the other way round: false (default) keeps its order; true "
                "reverses the waypoints before the track is built, so the reset pose (first "
                "waypoint, facing along the raceline) faces the other way. Only used with "
                "raceline_path."
            ),
        )
        self._reverse_direction = bool(
            self.declare_parameter("reverse_direction", False, reverse_direction_descriptor).value
        )

        pocket = self._declare_pocket()
        self._declare_park_scenario_parameters()

        self.env = build_env(
            self._seed,
            self._raceline_path,
            self._track_half_width_m,
            self._reverse_direction,
            pocket,
            self._mirror_reverse_speed_control,
        )

        scan_sim = self.env.unwrapped.sim.agents[0].scan_simulator
        self._fov_rad = float(scan_sim.fov)
        self._range_min = 0.0
        self._range_max = float(scan_sim.max_range)
        self._env_timestep = float(self.env.unwrapped.timestep)
        # The mounted LiDAR (roadmap 2.9): a second ScanSimulator2D on the same map, cast from the
        # mount instead of the gym pose. None keeps the gym's own scan.
        self._mounted_scan = None
        if self._lidar_mounted:
            from f1tenth_gym.envs.laser_models import ScanSimulator2D

            fov = self._lidar_fov_rad if self._lidar_fov_rad > 0.0 else self._fov_rad
            beams = self._lidar_num_beams if self._lidar_num_beams > 0 else int(scan_sim.num_beams)
            self._mounted_scan = ScanSimulator2D(beams, fov, max_range=self._range_max)
            self._mounted_scan.set_map(self.env.unwrapped.track)
            self._mounted_scan_rng = np.random.default_rng(self._seed)
            self._fov_rad = float(fov)
        self._wheel_distance_m = 0.0

        step_rate_descriptor = ParameterDescriptor(
            description=(
                "Sim step rate in Hz. 0.0 (default) means 'use the gym env's own "
                "physics timestep' (1 / env.timestep) rather than a second "
                "hand-typed number."
            ),
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1000.0, step=0.0)],
        )
        requested_rate = float(
            self.declare_parameter("step_rate_hz", 0.0, step_rate_descriptor).value
        )
        self._step_rate_hz = requested_rate if requested_rate > 0.0 else 1.0 / self._env_timestep

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        drive_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # transient_local + explicit depth (claude-docs/10-conventions.md: "QoS: ... explicit
        # depth -- never default"): the map is published once and never changes for the life
        # of an episode, so a late-joining subscriber (e.g. Foxglove connecting after
        # bridge_node has been running a while) must still receive it -- that is what
        # "latched" means for a ROS 2 publisher.
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._scan_pub = self.create_publisher(LaserScan, "/scan", scan_qos)
        self._odom_pub = self.create_publisher(Odometry, "/sim/ground_truth_odom", odom_qos)
        # /odom/wheel only when asked for: on the car it belongs to vesc_odometry_node.
        self._wheel_odom_pub = (
            self.create_publisher(Odometry, "/odom/wheel", odom_qos)
            if self._publish_wheel_odom
            else None
        )
        self._map_pub = self.create_publisher(OccupancyGrid, "/sim/map", map_qos)
        self._drive_sub = self.create_subscription(
            AckermannDriveStamped, "/drive", self._on_drive, drive_qos
        )
        self._reset_srv = self.create_service(Trigger, "/sim/reset", self._on_reset)

        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._broadcast_static_laser_tf()

        self._latest_steering_angle = 0.0
        self._latest_speed = 0.0
        self._warned_terminated = False

        self._reset_env()
        self._publish_map_once()

        self._timer = self.create_timer(1.0 / self._step_rate_hz, self._on_timer)

        self.get_logger().info(
            f"racer_gym_bridge up: stepping at {self._step_rate_hz:.1f} Hz "
            f"(env timestep {self._env_timestep:.4f} s), scan {scan_sim.num_beams} beams "
            f"over {self._fov_rad:.3f} rad fov, range_max {self._range_max:.1f} m."
        )

    # -- ROS callbacks ---------------------------------------------------

    def _on_drive(self, msg: AckermannDriveStamped) -> None:
        self._latest_steering_angle = msg.drive.steering_angle
        self._latest_speed = msg.drive.speed

    def _on_reset(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request  # std_srvs/Trigger takes no fields
        self._reset_env()
        response.success = True
        response.message = "sim reset"
        return response

    def _on_timer(self) -> None:
        longitudinal = self._latest_speed
        if self._mirror_reverse_speed_control:
            params = self.env.unwrapped.params
            longitudinal = mirrored_speed_to_accel(
                self._latest_speed,
                float(self.env.unwrapped.sim.agents[0].state[3]),
                float(params["a_max"]),
                float(params["v_max"]),
                float(params["v_min"]),
            )
        action = drive_cmd_to_action(self._latest_steering_angle, longitudinal)
        obs, _reward, terminated, truncated, _info = self.env.step(action)
        if terminated or truncated:
            if not self._warned_terminated:
                self.get_logger().warn(
                    f"sim episode ended (terminated={terminated} truncated={truncated}); "
                    "continuing to step in place -- call /sim/reset to start a new episode."
                )
                self._warned_terminated = True
        else:
            self._warned_terminated = False
        self._publish(obs)

    # -- helpers -----------------------------------------------------------

    def _reset_env(self) -> None:
        obs, _info = self.env.reset(seed=self._seed)
        self._warned_terminated = False
        self._publish(obs)

    def _publish(self, obs: dict) -> None:
        agent_obs = obs[_EGO_AGENT_ID]
        now = self.get_clock().now().to_msg()

        ranges = agent_obs["scan"]
        if self._mounted_scan is not None:
            yaw = float(agent_obs["pose_theta"])
            lidar_pose = np.array(
                [
                    float(agent_obs["pose_x"]) + self._lidar_offset_x_m * math.cos(yaw),
                    float(agent_obs["pose_y"]) + self._lidar_offset_x_m * math.sin(yaw),
                    yaw + self._lidar_yaw_rad,
                ]
            )
            ranges = self._mounted_scan.scan(lidar_pose, self._mounted_scan_rng)
        scan_fields = build_scan_fields(
            ranges=ranges,
            fov_rad=self._fov_rad,
            range_min=self._range_min,
            range_max=self._range_max,
        )
        scan_msg = LaserScan()
        scan_msg.header.stamp = now
        scan_msg.header.frame_id = _LASER_FRAME_ID
        scan_msg.angle_min = scan_fields.angle_min
        scan_msg.angle_max = scan_fields.angle_max
        scan_msg.angle_increment = scan_fields.angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 1.0 / self._step_rate_hz
        scan_msg.range_min = scan_fields.range_min
        scan_msg.range_max = scan_fields.range_max
        scan_msg.ranges = scan_fields.ranges
        self._scan_pub.publish(scan_msg)

        odom_fields = build_odom_fields(
            pose_x=agent_obs["pose_x"],
            pose_y=agent_obs["pose_y"],
            yaw_rad=agent_obs["pose_theta"],
            vx=agent_obs["linear_vel_x"],
            vy=agent_obs["linear_vel_y"],
            yaw_rate=agent_obs["ang_vel_z"],
        )
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = _MAP_FRAME_ID
        odom_msg.child_frame_id = _BASE_LINK_FRAME_ID
        position = odom_msg.pose.pose.position
        position.x, position.y, position.z = odom_fields.position
        orientation = odom_msg.pose.pose.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = odom_fields.orientation
        linear = odom_msg.twist.twist.linear
        linear.x, linear.y, linear.z = odom_fields.linear
        angular = odom_msg.twist.twist.angular
        angular.x, angular.y, angular.z = odom_fields.angular
        self._odom_pub.publish(odom_msg)

        if self._wheel_odom_pub is not None:
            self._publish_wheel_odom_msg(now, float(agent_obs["linear_vel_x"]))

        # Milestone 2: map -> base_link from the same ground-truth pose, every step (no
        # localization stack exists yet -- see this module's docstring).
        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = _MAP_FRAME_ID
        transform.child_frame_id = _BASE_LINK_FRAME_ID
        transform.transform.translation.x, transform.transform.translation.y, _ = (
            odom_fields.position
        )
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ) = odom_fields.orientation
        self._tf_broadcaster.sendTransform(transform)

    def _publish_wheel_odom_msg(self, stamp, speed_mps: float) -> None:
        """/odom/wheel in vesc_odometry_node's shape (roadmap 2.9): twist.linear.x the speed,
        pose.position.x the SIGNED along-track distance since the node started (not an x
        coordinate). The speed is the gym's longitudinal body speed, ``linear_vel_x`` (v cos beta at
        the gym's reference point); in the gym's kinematic regime that is exactly the rear axle's
        speed, the distance park_node's arcs are measured in. Integrated per step over the env
        timestep; /sim/reset does not zero it (the real driver does not either)."""
        self._wheel_distance_m += speed_mps * self._env_timestep
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = _ODOM_FRAME_ID
        msg.child_frame_id = _BASE_LINK_FRAME_ID
        msg.pose.pose.position.x = self._wheel_distance_m
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = speed_mps
        self._wheel_odom_pub.publish(msg)

    def _declare_pocket(self) -> Pocket | None:
        descriptor = ParameterDescriptor(
            description=(
                "Roadmap 2.9: a pocket (parking slot) cut into one corridor wall, starting this "
                "far (m) along the raceline as driven from its first point. Used only when "
                "pocket_length_m > 0 (needs raceline_path and track_half_width_m)."
            ),
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1000.0, step=0.0)],
        )
        start = float(self.declare_parameter("pocket_start_m", 0.0, descriptor).value)
        descriptor = ParameterDescriptor(
            description="Pocket length along the raceline (m); 0.0 (default) = no pocket.",
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=100.0, step=0.0)],
        )
        length = float(self.declare_parameter("pocket_length_m", 0.0, descriptor).value)
        descriptor = ParameterDescriptor(
            description="How far the pocket pushes the wall back beyond the corridor (m).",
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=20.0, step=0.0)],
        )
        depth = float(self.declare_parameter("pocket_depth_m", 0.5, descriptor).value)
        descriptor = ParameterDescriptor(
            description="Which wall, relative to the direction of travel: right or left."
        )
        side = str(self.declare_parameter("pocket_side", "right", descriptor).value)
        if length <= 0.0:
            return None
        if not self._raceline_path or self._track_half_width_m <= 0.0:
            raise ValueError("pocket_length_m > 0 needs raceline_path and track_half_width_m > 0")
        return Pocket(start_s_m=start, length_m=length, depth_m=depth, side=side)

    def _declare_park_scenario_parameters(self) -> None:
        """The rest of the roadmap 2.9 parameters; every default keeps the old behaviour."""
        self._publish_wheel_odom = bool(
            self.declare_parameter(
                "publish_wheel_odom",
                False,
                ParameterDescriptor(
                    description=(
                        "Publish /odom/wheel like racer_drivers/vesc_odometry_node (twist.linear.x "
                        "the speed, pose.position.x the signed along-track distance). SIM ONLY: on "
                        "the car that topic is the VESC's. Default false."
                    )
                ),
            ).value
        )
        self._mirror_reverse_speed_control = bool(
            self.declare_parameter(
                "mirror_reverse_speed_control",
                False,
                ParameterDescriptor(
                    description=(
                        "Drive the gym in acceleration mode with its forward speed-controller "
                        "gains mirrored for reverse (conversions.mirrored_speed_to_accel): the "
                        "gym's own controller brakes a reversing car 20 times more weakly than a "
                        "forward one. Default false (the gym's controller, unchanged)."
                    )
                ),
            ).value
        )
        self._lidar_offset_x_m = float(
            self.declare_parameter(
                "lidar_offset_x_m",
                0.0,
                ParameterDescriptor(
                    description=(
                        "Cast /scan from this far ahead of the gym's pose (m; the gym's pose is "
                        "its centre of gravity, chassis.cg_to_rear_axle_m ahead of the rear "
                        "axle). The caller passes it from vehicle_params; nothing physical is "
                        "typed here."
                    ),
                    floating_point_range=[
                        FloatingPointRange(from_value=-2.0, to_value=2.0, step=0.0)
                    ],
                ),
            ).value
        )
        self._lidar_yaw_rad = float(
            self.declare_parameter(
                "lidar_yaw_rad",
                0.0,
                ParameterDescriptor(
                    description=(
                        "Mounting yaw of the cast /scan (rad; pi = facing backwards, like the "
                        "car's sensors.lidar.mount_yaw_rad)."
                    ),
                    floating_point_range=[
                        FloatingPointRange(
                            from_value=-2.0 * math.pi, to_value=2.0 * math.pi, step=0.0
                        )
                    ],
                ),
            ).value
        )
        self._lidar_fov_rad = float(
            self.declare_parameter(
                "lidar_fov_rad",
                0.0,
                ParameterDescriptor(
                    description="Field of view of the cast /scan (rad); 0.0 = the gym's own.",
                    floating_point_range=[
                        FloatingPointRange(from_value=0.0, to_value=2.0 * math.pi, step=0.0)
                    ],
                ),
            ).value
        )
        self._lidar_num_beams = int(
            self.declare_parameter(
                "lidar_num_beams",
                0,
                ParameterDescriptor(
                    description="Beams of the cast /scan; 0 = the gym's own.",
                    integer_range=[IntegerRange(from_value=0, to_value=10000, step=1)],
                ),
            ).value
        )
        self._lidar_mounted = (
            self._lidar_offset_x_m != 0.0
            or self._lidar_yaw_rad != 0.0
            or self._lidar_fov_rad > 0.0
            or self._lidar_num_beams > 0
        )

    def _publish_map_once(self) -> None:
        """Publish ``/sim/map`` once (transient_local latches it for later subscribers).

        Built from the env's own ``Track`` (``track.occupancy_map`` / ``track.spec``), never
        hand-copied -- see ``racer_gym_bridge.conversions.build_occupancy_grid_fields``. The
        map is fixed for the life of an episode (the track never changes underneath a running
        bridge_node), so publishing once at startup is sufficient; a late-joining subscriber
        gets it from the transient_local publisher history, not a live re-publish.
        """
        track: Track = self.env.unwrapped.track
        fields = build_occupancy_grid_fields(
            occupancy_map=track.occupancy_map,
            resolution=track.spec.resolution,
            origin=track.spec.origin,
            negate=bool(track.spec.negate),
            occupied_thresh=track.spec.occupied_thresh,
            free_thresh=track.spec.free_thresh,
        )
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = _MAP_FRAME_ID
        msg.info.resolution = fields.resolution
        msg.info.width = fields.width
        msg.info.height = fields.height
        origin_position = msg.info.origin.position
        origin_position.x, origin_position.y, origin_position.z = fields.origin_position
        origin_orientation = msg.info.origin.orientation
        (
            origin_orientation.x,
            origin_orientation.y,
            origin_orientation.z,
            origin_orientation.w,
        ) = fields.origin_orientation
        msg.data = fields.data
        self._map_pub.publish(msg)

    def _broadcast_static_laser_tf(self) -> None:
        """Static ``base_link`` -> ``laser`` transform so ``/scan`` renders aligned.

        Identity, not an invented offset: f1tenth_gym's ``ScanSimulator2D`` has no separate
        LiDAR extrinsic of its own (it raycasts directly from the car's pose), and
        ``config/vehicle_params.yaml``'s ``sensors.lidar`` mount offsets are still ``null``
        pending a real Phase 2 measurement (roadmap 2.3) -- CLAUDE.md hard invariant 2
        forbids hand-writing a substitute number in their place, so identity is the only
        transform that is both correct for the current sim model and not a fabricated
        physical constant.
        """
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = _BASE_LINK_FRAME_ID
        transform.child_frame_id = _LASER_FRAME_ID
        # The mounted LiDAR's pose relative to the gym's pose when one is configured (roadmap
        # 2.9); identity otherwise.
        transform.transform.translation.x = self._lidar_offset_x_m
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ) = yaw_to_quaternion(self._lidar_yaw_rad)
        self._static_tf_broadcaster.sendTransform(transform)

    def destroy_node(self) -> bool:
        self.env.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        # rclpy installs its own SIGINT handler that already calls
        # rclpy.try_shutdown() -- launch_testing's post-shutdown check
        # (and a plain Ctrl-C from a launch file) delivers exactly that
        # signal, so an unconditional rclpy.shutdown() here double-shuts
        # the context and raises RCLError, which turns a clean exit into
        # a nonzero one. Only shut down if it is still our job to do so.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
