#!/usr/bin/env python3
"""Driver benchmark on the track family: one track, one driver, one JSON result.

Like reactive_bench_runner.py (same metrics, same real command path: driver -> /drive_raw ->
safety_node -> /drive -> bridge_node, nothing remapped) but for any of the bridge's synthetic
tracks including the family tracks (stadium, rounded_rect, and `map`, a saved SLAM map as the
world), with the car as measured and three drivers:

  gap_follow    gap_follow_node on a launch profile (default lane-2026-10-25) plus parameters
  tracker       tracker_node on a raceline made from the track's map by tools/track_session
                (the same pipeline a track session runs), following GROUND TRUTH pose
  tracker_loc   the same raceline, following the LOCALISED pose: the car's odometry (wheel
                speed, modelled steering, EKF) and AMCL on the track's map, sensor errors on
  tracker_policy the tracker in HANDOFF mode (drive_topic /tracker/command, lookahead from the
                policy directory's policy_meta.yaml) plus racer_policy's policy_node, which adds
                the bounded residual and is the only publisher of /drive_raw (--policy-dir)
  pnp           the Plug-and-Play driver, no map and no pose: racer_track_perception's
                track_perception_node and racer_planner's mppi_planner_node (--backend,
                --perception-bundle, --dynamics-bundle, --v-max-mps), with the car's LiDAR,
                scan noise and 50 Hz /odom/wheel; parameters come from pnp.launch.py

The car (--car): `params` (default) gives the simulator's steering the limits of vehicle_params
(0.449 rad each way, schema 0.14.0: commands are true angles and true speeds); `gym` is the
simulator's own 0.4189 rad. The LiDAR is cast from the real mount.

Run with the workspace sourced (racer_gym_bridge, racer_safety, racer_control, racer_state,
racer_bringup, racer_tools built), e.g. in the ros-dev container:

    python3 tests/l5_reactive_bench/track_family_runner.py --track stadium \\
        --track-params-json '{"lane_width_m": 1.0, "straight_length_m": 2.0, "island_radius_m": 0.0}' \\
        --driver tracker --stop-after-laps 3 --out result.json

Exit status: 0 with a result, 3 when a node died (the JSON carries "error").
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO_ROOT / "tools" / "track_session"))

import reactive_bench_runner as base

TRACKS = (
    "lane_square",
    "lane_round",
    "lane_open_side",
    "chicane",
    "stadium",
    "rounded_rect",
    "map",
)
DRIVERS = ("gap_follow", "tracker", "tracker_loc", "tracker_policy", "pnp")
CARS = ("gym", "params")
PNP_SCAN_BEAMS = 720
LOCALISATION_LAUNCH = REPO_ROOT / "ros_ws" / "src" / "racer_bringup" / "launch"
# The sensor errors the localisation canaries use (sim_autopilot.launch.py SIM_SENSOR_NOISE).
LANE_TRACKER_DEFAULTS = {
    "lookahead_min_m": 0.35,
    "lookahead_max_m": 0.7,
    "lookahead_curvature_ref_1pm": 1.25,
}
_STRING_TRACK_KEYS = {
    "map_yaml": "synthetic_map_yaml",
    "map_centreline_csv": "synthetic_map_centreline",
}
_FLOAT_TRACK_KEYS = {
    **base._BRIDGE_TRACK_KEYS,
    "straight_length_m": "synthetic_straight_length_m",
    "island_width_m": "synthetic_island_width_m",
    "island_radius_m": "synthetic_island_radius_m",
}


def car_parameters(car: str) -> dict:
    """Bridge parameters for the car model: `params` gives the gym the steering limits of
    vehicle_params (0.449 rad each way), `gym` leaves its own 0.4189."""
    from pipeline import steering_limits

    if car != "params":
        return {}
    left, right = steering_limits()
    return {"steering_limit_left_rad": left, "steering_limit_right_rad": right}


def bridge_track_parameters(track: str, track_params: dict, seed: int, reverse: bool) -> dict:
    unknown = set(track_params) - set(_FLOAT_TRACK_KEYS) - set(_STRING_TRACK_KEYS)
    if unknown:
        raise ValueError(f"unknown track parameter(s) {sorted(unknown)}")
    params: dict = {"seed": seed, "synthetic_track": track, "reverse_direction": reverse}
    for key, value in track_params.items():
        if key in _STRING_TRACK_KEYS:
            params[_STRING_TRACK_KEYS[key]] = str(value)
        else:
            params[_FLOAT_TRACK_KEYS[key]] = float(value)
    return params


def build_world(track: str, track_params: dict, work: Path):
    """The track, its map yaml (surfaces only, for AMCL and the raceline tool) and the
    centreline CSV the `map` track needs. Returns (synthetic, map_yaml, track_params with the
    generated centreline path filled in)."""
    from pipeline import raceline_from_map, write_surface_map
    from racer_gym_bridge import track_loader

    params = dict(track_params)
    if track == "map":
        map_yaml = Path(params["map_yaml"])
        if "map_centreline_csv" not in params:
            centre_csv = work / "map_centreline.csv"
            raceline_from_map(
                map_yaml, work / "unused_line.csv", centre_only=True, centre_csv=centre_csv
            )
            params["map_centreline_csv"] = str(centre_csv)
        synthetic = track_loader.build_synthetic_track(track, **params)
        return synthetic, map_yaml, params
    synthetic = track_loader.build_synthetic_track(track, **params)
    occupancy, origin = track_loader.rasterize_walls(
        synthetic.walls, synthetic.bounds(), synthetic.occupancy_resolution_m
    )
    map_yaml = write_surface_map(
        occupancy, (origin[0], origin[1]), synthetic.occupancy_resolution_m, work / "world"
    )
    return synthetic, map_yaml, params


def make_raceline(args, synthetic, map_yaml: Path, work: Path) -> tuple[Path, dict]:
    """tools/track_session's pipeline on the track's own map, in the driving direction."""
    from pipeline import raceline_from_map

    options = json.loads(args.raceline_json)
    if args.clearances:
        options["clearance_candidates"] = [float(v) for v in args.clearances.split(",")]
    if "clearance_candidates" in options:
        options["clearance_candidates"] = tuple(options["clearance_candidates"])
    seed_xy = (float(synthetic.centreline_x[0]), float(synthetic.centreline_y[0]))
    if args.track == "map":
        seed_xy = (0.0, 0.0)
    out = work / "raceline.csv"
    result = raceline_from_map(
        map_yaml,
        out,
        direction="cw" if args.reverse else "ccw",
        seed_xy=seed_xy,
        **options,
    )
    return out, result.report


def run_benchmark(args: argparse.Namespace) -> dict:
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    import bench_metrics as bm
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from racer_msgs.msg import SafetyEvent
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener

    work = Path(args.work_dir) if args.work_dir else Path(tempfile.mkdtemp(prefix="family_bench_"))
    work.mkdir(parents=True, exist_ok=True)
    track_params = json.loads(args.track_params_json)
    overrides = json.loads(args.params_json)
    tracker_overrides = json.loads(args.tracker_params_json)
    result: dict = {
        "track": args.track,
        "track_params": track_params,
        "reverse": args.reverse,
        "driver": args.driver,
        "car": args.car,
        "seed": args.seed,
        "lidar": args.lidar,
    }
    synthetic, map_yaml, track_params = build_world(args.track, track_params, work)
    walls_synthetic = synthetic
    if args.reverse:
        synthetic = synthetic.reversed()
    vehicle = yaml.safe_load(base.VEHICLE_PARAMS_PATH.read_text())
    box = bm.BodyBox.from_vehicle_params(vehicle)
    cg_to_rear = float(vehicle["chassis"]["cg_to_rear_axle_m"])
    lap_tracker = bm.LapTracker(synthetic.centreline_x, synthetic.centreline_y)
    result["loop_length_m"] = round(lap_tracker.length_m, 3)

    localised = args.driver == "tracker_loc"
    bridge = bridge_track_parameters(args.track, track_params, args.seed, args.reverse)
    bridge.update(car_parameters(args.car))
    mounted = args.lidar == "mounted" or localised
    if mounted:
        from racer_tools.vehicle_params_loader import load_vehicle_params

        helpers = base._load_module(
            "sim_autopilot_launch", LOCALISATION_LAUNCH / "sim_autopilot.launch.py"
        )
        car_like = helpers.car_like_bridge_parameters(load_vehicle_params(), args.noise)
        if localised:
            bridge.update(car_like)
            bridge.update({k: float(v) for k, v in (kv.split("=") for kv in args.noise_set)})
        else:
            lidar_keys = (
                "lidar_offset_x_m",
                "lidar_yaw_rad",
                "lidar_fov_rad",
                "lidar_num_beams",
                "scan_rate_hz",
                "scan_range_max_m",
            )
            bridge.update({k: car_like[k] for k in lidar_keys})
    if args.driver in ("tracker", "tracker_policy"):
        bridge["ground_truth_rear_odom_offset_x_m"] = -cg_to_rear
    pnp_launch = None
    if args.driver == "pnp":
        pnp_launch = base._load_module("pnp_launch", LOCALISATION_LAUNCH / "pnp.launch.py")
        bridge.update({k: car_like[k] for k in ("publish_wheel_odom", "wheel_odom_rate_hz")})
        bridge["scan_noise_std_m"] = helpers.SIM_SENSOR_NOISE["scan_noise_std_m"]
        # The C1 as it runs on the car: 720 beams over the full circle.
        bridge["lidar_num_beams"] = PNP_SCAN_BEAMS
        bridge["lidar_fov_rad"] = 2.0 * math.pi * (PNP_SCAN_BEAMS - 1) / PNP_SCAN_BEAMS

    nodes: dict[str, dict] = {
        "bridge_node": bridge,
        "safety_node": {"laser_yaw_from_vehicle_params": mounted},
    }
    stack = [
        ("bridge_node", [base._executable("racer_gym_bridge", "bridge_node")]),
        ("safety_node", [base._executable("racer_safety", "safety_node")]),
    ]
    if args.driver == "gap_follow":
        profiles = base.load_profiles()
        gap = base.gap_follow_parameters(args.profile, overrides, profiles)
        gap["laser_yaw_from_vehicle_params"] = mounted
        nodes["gap_follow_node"] = gap
        stack.append(("gap_follow_node", [base._executable("racer_control", "gap_follow_node")]))
        result.update({"profile": args.profile, "overrides": overrides, "gap_follow_params": gap})
    elif args.driver == "pnp":
        perception, planner = pnp_launch.parameters_for(
            args.perception_bundle, args.dynamics_bundle, args.backend, args.v_max_mps, mounted
        )
        nodes["track_perception_node"], nodes["mppi_planner_node"] = perception, planner
        for package, name in (
            ("racer_track_perception", "track_perception_node"),
            ("racer_planner", "mppi_planner_node"),
        ):
            stack.append((name, [base._executable(package, name)]))
        result.update({"backend": args.backend, "v_max_mps": args.v_max_mps, "pnp": nodes})
    else:
        raceline_csv, report = make_raceline(args, walls_synthetic, map_yaml, work)
        tracker = dict(LANE_TRACKER_DEFAULTS)
        tracker.update(tracker_overrides)
        tracker["raceline_path"] = str(raceline_csv)
        if localised:
            tracker.update({"odom_topic": "/odom", "pose_frame": "map"})
        else:
            tracker["odom_topic"] = "/sim/ground_truth_rear_odom"
        if args.driver == "tracker_policy":
            if not args.policy_dir:
                raise ValueError("--driver tracker_policy needs --policy-dir")
            policy_dir = Path(args.policy_dir)
            meta = yaml.safe_load((policy_dir / "policy_meta.yaml").read_text())
            base_settings = meta["must_match_at_deploy"]["base_controller"]
            tracker.update(
                {
                    k: base_settings[k]
                    for k in ("lookahead_min_m", "lookahead_max_m", "lookahead_curvature_ref_1pm")
                }
            )
            tracker["drive_topic"] = "/tracker/command"
            nodes["policy_node"] = {
                "policy_dir": str(policy_dir),
                "raceline_path": str(raceline_csv),
                "odom_topic": tracker["odom_topic"],
                "pose_frame": "",
                "tracker_command_topic": "/tracker/command",
                "laser_yaw_from_vehicle_params": mounted,
            }
        nodes["tracker_node"] = tracker
        stack.append(("tracker_node", [base._executable("racer_control", "tracker_node")]))
        if args.driver == "tracker_policy":
            stack.append(("policy_node", [base._executable("racer_policy", "policy_node")]))
            result["policy_dir"] = str(args.policy_dir)
        result.update({"tracker_params": tracker, "raceline": report})
        if args.track == "map":
            # The imported map's start pose comes from its centreline file: give it the raceline
            # (already in the driving direction), so the car starts on the line it follows.
            from racer_gym_bridge import track_loader

            track_params["map_centreline_csv"] = str(raceline_csv)
            synthetic = track_loader.build_synthetic_track(args.track, **track_params)
            bridge["synthetic_map_centreline"] = str(raceline_csv)
            bridge["reverse_direction"] = False
            lap_tracker = bm.LapTracker(synthetic.centreline_x, synthetic.centreline_y)
            result["loop_length_m"] = round(lap_tracker.length_m, 3)
        if localised:
            stack.append(
                (
                    "localization",
                    [
                        "ros2",
                        "launch",
                        str(LOCALISATION_LAUNCH / "localization.launch.py"),
                        f"map:={map_yaml}",
                        "set_initial_pose:=false",
                        *([f"amcl_params_file:={args.amcl_params}"] if args.amcl_params else []),
                        *([f"ekf_params_file:={args.ekf_params}"] if args.ekf_params else []),
                    ],
                )
            )
    params_file = work / "params.yaml"
    base._write_params_file(params_file, nodes)

    processes: list[tuple[str, subprocess.Popen]] = []
    for name, command in stack:
        log = (work / f"{name}.log").open("w")
        if name == "localization":
            cmd = command
        else:
            cmd = [*command, "--ros-args", "--params-file", str(params_file)]
        processes.append(
            (
                name,
                subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy()),
            )
        )

    def died() -> str | None:
        for name, p in processes:
            if p.poll() is not None:
                tail = (work / f"{name}.log").read_text()[-1500:]
                return f"{name} exited with {p.returncode}: {tail}"
        return None

    duration_s = args.duration
    result["requested_duration_s"] = duration_s
    rclpy.init()
    node = rclpy.create_node("track_family_runner")
    recording = bm.Recording()
    reliable = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=200
    )
    rear_pose: list = [None]
    loc_errors: list[float] = []
    loc_series: list[tuple] = []
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node, spin_thread=False)

    def odom_yaw() -> float:
        """Heading of the EKF's odom -> base_link (the odometry alone, before AMCL's correction)."""
        if not tf_buffer.can_transform("odom", "base_link", rclpy.time.Time()):
            return float("nan")
        q = tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time()).transform.rotation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_odom(msg) -> None:
        q = msg.pose.pose.orientation
        yaw = float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        recording.odom.append(
            (base._stamp_s(msg.header.stamp), x, y, yaw, msg.twist.twist.linear.x)
        )
        rear_pose[0] = (
            x - cg_to_rear * math.cos(yaw),
            y - cg_to_rear * math.sin(yaw),
            yaw,
        )
        if (
            localised
            and len(recording.odom) % 5 == 0
            and tf_buffer.can_transform("map", "base_link", rclpy.time.Time())
        ):
            tf = tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            tq = tf.transform.rotation
            est_yaw = math.atan2(
                2.0 * (tq.w * tq.z + tq.x * tq.y), 1.0 - 2.0 * (tq.y * tq.y + tq.z * tq.z)
            )
            ex, ey = tf.transform.translation.x, tf.transform.translation.y
            gx, gy, gyaw = rear_pose[0]
            dx, dy = ex - gx, ey - gy
            loc_errors.append(math.hypot(dx, dy))
            loc_series.append(
                (
                    base._stamp_s(msg.header.stamp),
                    math.cos(gyaw) * dx + math.sin(gyaw) * dy,  # along the car's heading
                    -math.sin(gyaw) * dx + math.cos(gyaw) * dy,  # across it, left positive
                    math.atan2(math.sin(est_yaw - gyaw), math.cos(est_yaw - gyaw)),
                    gyaw,
                    odom_yaw(),
                )
            )

    def on_drive_raw(msg) -> None:
        recording.drive_raw.append(
            (base._stamp_s(msg.header.stamp), msg.drive.speed, msg.drive.steering_angle)
        )

    def on_drive(msg) -> None:
        recording.drive.append(
            (base._stamp_s(msg.header.stamp), msg.drive.speed, msg.drive.steering_angle)
        )

    event_details: list[dict] = []

    def on_event(msg) -> None:
        recording.events.append((base._stamp_s(msg.stamp), msg.source, msg.phase, msg.severity))
        if msg.phase == 0 and len(event_details) < 40:
            event_details.append(
                {"t_s": base._stamp_s(msg.stamp), "source": msg.source, "detail": msg.detail}
            )

    node.create_subscription(Odometry, "/sim/ground_truth_odom", on_odom, reliable)
    node.create_subscription(AckermannDriveStamped, "/drive_raw", on_drive_raw, reliable)
    node.create_subscription(AckermannDriveStamped, "/drive", on_drive, reliable)
    node.create_subscription(SafetyEvent, "/safety/events", on_event, reliable)
    policy_status: dict = {}
    policy_events: list[dict] = []
    if args.driver == "tracker_policy":
        from std_msgs.msg import String

        def on_policy_status(msg) -> None:
            policy_status.clear()
            policy_status.update(json.loads(msg.data))

        def on_policy_event(msg) -> None:
            policy_events.append(json.loads(msg.data))

        node.create_subscription(String, "/policy/status", on_policy_status, reliable)
        node.create_subscription(String, "/policy/events", on_policy_event, reliable)
    planner_states: list[int] = []
    if args.driver == "pnp":
        from racer_pnp_msgs.msg import PlannerStatus

        node.create_subscription(
            PlannerStatus, "/pnp/planner/status", lambda m: planner_states.append(m.state), reliable
        )
    reset_client = node.create_client(Trigger, "/sim/reset")
    initial_pose_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", reliable)

    def publish_initial_pose() -> None:
        if rear_pose[0] is None:
            return
        x, y, yaw = rear_pose[0]
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = (
            math.sin(yaw / 2),
            math.cos(yaw / 2),
        )
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05**2
        msg.pose.covariance[35] = 0.05**2
        initial_pose_pub.publish(msg)

    stopped_because = "duration"
    try:
        deadline = time.monotonic() + base.STARTUP_TIMEOUT_S
        while not (recording.odom and (recording.drive_raw or localised)):
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() > deadline or (why := died()):
                raise RuntimeError(why or f"stack not up after {base.STARTUP_TIMEOUT_S} s")
        if not reset_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("/sim/reset not available")

        def request_reset():
            return reset_client.call_async(Trigger.Request())

        future = request_reset()
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
        if future.result() is None:
            raise RuntimeError("/sim/reset did not complete")
        if localised:
            # AMCL starts from the true pose, as an operator's click in Foxglove would, and
            # tracker_node cannot move until map -> odom exists, so the car waits here.
            end = time.monotonic() + 60.0
            next_pub = 0.0
            while time.monotonic() < end:
                if time.monotonic() >= next_pub:
                    publish_initial_pose()
                    next_pub = time.monotonic() + 1.0
                rclpy.spin_once(node, timeout_sec=0.05)
                if tf_buffer.can_transform("map", "base_link", rclpy.time.Time()):
                    break
            else:
                raise RuntimeError("AMCL published no map -> odom within 60 s of /initialpose")
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.05)
        recording.clear()
        loc_errors.clear()
        loc_series.clear()
        pending_reset = None
        initial_pose_until = 0.0
        consumed = 0
        previous = None
        cooldown_until = 0.0
        start_stamp = None
        wall_deadline = time.monotonic() + 3.0 * duration_s + 60.0
        last_check = time.monotonic()
        next_initial = 0.0
        last_motion = None
        while True:
            rclpy.spin_once(node, timeout_sec=0.01)
            if pending_reset is not None and pending_reset.done():
                pending_reset = None
            if (
                localised
                and time.monotonic() < initial_pose_until
                and time.monotonic() >= next_initial
            ):
                publish_initial_pose()
                next_initial = time.monotonic() + 0.4
            while consumed < len(recording.odom):
                sample = recording.odom[consumed]
                consumed += 1
                t, x, y, yaw, v = sample
                if start_stamp is None:
                    start_stamp = t
                if previous is not None and bm.is_teleport(previous[1:3], (x, y)):
                    lap_tracker.reset()
                    cooldown_until = 0.0
                    if localised:
                        initial_pose_until = time.monotonic() + 3.0
                elif previous is not None and t >= cooldown_until:
                    reason = bm.gym_contact_signature(previous[3], yaw, previous[4], v)
                    if (
                        reason is None
                        and consumed % base.CLEARANCE_CHECK_STRIDE == 0
                        and bm.body_clearance_m(
                            x, y, yaw, box, synthetic.walls, synthetic.wall_thickness_m
                        )
                        <= 0.0
                    ):
                        reason = "body_overlaps_wall"
                    if reason is not None:
                        recording.contacts.append((t, x, y, reason))
                        cooldown_until = t + base.CONTACT_COOLDOWN_S
                        if pending_reset is None:
                            pending_reset = request_reset()
                if t >= cooldown_until:
                    lap_tracker.update(x, y, t)
                if abs(v) > 0.05 or last_motion is None:
                    last_motion = t
                previous = sample
            if start_stamp is not None and previous is not None:
                if previous[0] - start_stamp >= duration_s:
                    break
                if args.stop_after_laps and lap_tracker.laps >= args.stop_after_laps:
                    stopped_because = "laps"
                    break
                if args.stop_after_contacts and len(recording.contacts) >= args.stop_after_contacts:
                    stopped_because = "contacts"
                    break
                if (
                    args.stall_s
                    and last_motion is not None
                    and previous[0] - last_motion > args.stall_s
                ):
                    stopped_because = "stalled"
                    break
            if time.monotonic() > wall_deadline:
                stopped_because = "wall_timeout"
                break
            if time.monotonic() - last_check > 1.0:
                last_check = time.monotonic()
                if why := died():
                    raise RuntimeError(why)
        metrics = bm.compute_metrics(
            recording,
            box=box,
            walls=synthetic.walls,
            lap_tracker=lap_tracker,
            wall_thickness_m=synthetic.wall_thickness_m,
        )
        result.update(metrics)
        result["stopped_because"] = stopped_because
        if args.driver == "tracker_policy":
            for _ in range(150):  # let the 1 Hz status of the last second arrive
                rclpy.spin_once(node, timeout_sec=0.01)
            result["policy_status"] = dict(policy_status)
            # CLAUDE.md invariant 1 / 04-architecture.md: policy_node is the only publisher of
            # /drive_raw in a policy run, safety_node the only publisher of /drive.
            result["publishers"] = {
                "/drive_raw": node.count_publishers("/drive_raw"),
                "/drive": node.count_publishers("/drive"),
                "/tracker/command": node.count_publishers("/tracker/command"),
            }
            result["policy_events"] = list(policy_events)
        if args.driver == "pnp":
            result["planner_states"] = {
                str(s): planner_states.count(s) for s in set(planner_states)
            }
            # Every /safety/events PHASE_ENGAGE with SEVERITY_BRAKE, whatever gate raised it.
            result["safety_brakes"] = sum(1 for e in recording.events if e[2] == 0 and e[3] == 2)
            # The command path: only the planner on /drive_raw, only safety_node on /drive.
            result["publisher_nodes"] = {
                topic: sorted(i.node_name for i in node.get_publishers_info_by_topic(topic))
                for topic in ("/drive_raw", "/drive")
            }
        t_first = recording.odom[0][0] if recording.odom else 0.0
        result["safety_events"] = [
            {**e, "t_s": round(e["t_s"] - t_first, 2)} for e in event_details
        ]
        if loc_errors:
            err = np.array(loc_errors)
            result["localisation_error_m"] = {
                "mean": round(float(err.mean()), 4),
                "p95": round(float(np.percentile(err, 95)), 4),
                "max": round(float(err.max()), 4),
            }
            ser = np.array(loc_series)
            result["localisation_error_split"] = {
                "along_mean_m": round(float(ser[:, 1].mean()), 4),
                "along_p95_abs_m": round(float(np.percentile(np.abs(ser[:, 1]), 95)), 4),
                "cross_mean_m": round(float(ser[:, 2].mean()), 4),
                "cross_p95_abs_m": round(float(np.percentile(np.abs(ser[:, 2]), 95)), 4),
                "heading_p95_abs_rad": round(float(np.percentile(np.abs(ser[:, 3]), 95)), 4),
            }
        if args.trace_out:
            np.savez_compressed(
                args.trace_out,
                odom=np.array(recording.odom),
                drive_raw=np.array(recording.drive_raw),
                drive=np.array(recording.drive),
                loc=np.array(loc_series),
                events=np.array([e["t_s"] for e in event_details]),
            )
    except RuntimeError as e:
        result["error"] = str(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        base._stop([p for _, p in processes])
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--track", required=True, choices=TRACKS)
    parser.add_argument("--track-params-json", default="{}")
    parser.add_argument("--reverse", action="store_true", help="drive the loop clockwise")
    parser.add_argument("--driver", default="gap_follow", choices=DRIVERS)
    parser.add_argument("--policy-dir", help="policy directory (driver tracker_policy)")
    parser.add_argument("--perception-bundle", default="", help="track_perception bundle (pnp)")
    parser.add_argument("--dynamics-bundle", default="", help="dynamics bundle (pnp)")
    parser.add_argument("--backend", default="geometric", choices=("net", "geometric"))
    parser.add_argument("--v-max-mps", type=float, default=0.6, help="pnp planner speed cap")
    parser.add_argument("--profile", default="lane-2026-10-25", help="gap_follow profile")
    parser.add_argument("--params-json", default="{}", help="gap_follow parameters on top")
    parser.add_argument("--tracker-params-json", default="{}", help="tracker_node parameters")
    parser.add_argument(
        "--raceline-json",
        default="{}",
        help="tools/track_session pipeline.raceline_from_map options (margin, v_max_mps, ...)",
    )
    parser.add_argument("--car", default="params", choices=CARS)
    parser.add_argument("--lidar", default="mounted", choices=("mounted", "aligned"))
    parser.add_argument("--noise", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--noise-set",
        action="append",
        default=[],
        help="override one bridge sensor noise parameter on a localised run, name=value "
        "(wheel_speed_scale=1.0, steering_offset_rad=0 ...); repeatable",
    )
    parser.add_argument(
        "--clearances",
        help="body clearance candidates for the raceline, widest first, comma separated "
        "(overrides clearance_candidates in --raceline-json)",
    )
    parser.add_argument("--amcl-params", help="amcl_params_file for a localised run (tuning)")
    parser.add_argument("--ekf-params", help="ekf_params_file for a localised run (tuning)")
    parser.add_argument("--duration", type=float, default=150.0, help="run time cap (s)")
    parser.add_argument("--stop-after-laps", type=int, default=3)
    parser.add_argument("--stop-after-contacts", type=int, default=0)
    parser.add_argument(
        "--stall-s",
        type=float,
        default=12.0,
        help="end the run once the car has been still this long (s), e.g. latched by safety_node "
        "with a driver that has no way out; 0 = never",
    )
    parser.add_argument("--seed", type=int, default=base.SEED)
    parser.add_argument("--domain-id", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "0")))
    parser.add_argument("--work-dir", type=Path, help="keep the generated map, raceline and logs")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--trace-out", type=Path)
    args = parser.parse_args(argv)
    result = run_benchmark(args)
    text = json.dumps(result, indent=2, sort_keys=True, default=str)
    print(text)
    if args.out:
        args.out.write_text(text + "\n")
    return 3 if "error" in result else 0


if __name__ == "__main__":
    sys.exit(main())
