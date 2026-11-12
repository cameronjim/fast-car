#!/usr/bin/env python3
"""Replays a recorded mapping run (extract_bag.py's npz) into the real mapping stack.

Publishes /clock, /scan, /odom/wheel (speed divided by --speed-divisor, the VESC's tyre radius
error) and /drive, plus the static base_link -> laser transform from vehicle_params (the
LiDAR mount, 0.285 m ahead of the rear axle, yaw pi), so `offline_map.launch.py` (slam_toolbox
and the EKF, use_sim_time) sees what it sees on the car. With --twist-source feeder it also
publishes /odom/wheel_twist itself, from the same kinematics as racer_state/wheel_twist_node but
with the true wheel angle per command radian given by --steer-scale-left/right (1.0 for a bag
from the calibrated car; 1.203 / 1.072 for the 2026-11-10 bags, whose commands were uncalibrated:
the stock node cannot represent that, its servo model clamps at the symmetric limit).

Idle stretches (no wheel speed, no speed command) are cut out and the clock closed up, so a
recording with minutes of parked car maps in the time it takes to drive. When the replay ends the
feeder calls /slam_toolbox/save_map (and serialize_map for the pose graph), then exits.

Run inside the ros-dev container with the workspace sourced; tools/track_session/map_from_bag.sh
does that.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mapping_eval.deskew import deskew_scan

VEHICLE_PARAMS = Path(__file__).resolve().parents[2] / "config" / "vehicle_params.yaml"

# Same defaults as racer_state's WheelTwistConfig (wheel_twist.hpp).
SPEED_STD_FLOOR = 0.02
SPEED_STD_FRACTION = 0.03
STEERING_STD = 0.03
YAW_RATE_STD_FLOOR = 0.02
LATERAL_STD = 0.05


def to_stamp(t: float) -> Time:
    sec = math.floor(t)
    return Time(sec=int(sec), nanosec=int((t - sec) * 1e9))


def active_mask_spans(
    odom_t: np.ndarray, odom_v: np.ndarray, drive_t: np.ndarray, drive_speed: np.ndarray
) -> list[tuple[float, float]]:
    """Time spans (start, end) of idle stretches longer than 4 s: no wheel speed above 1 cm/s and
    no speed command above 1 cm/s. The spans returned are shrunk by 1.5 s on each side, so the
    scan matcher sees the car stop and start."""
    times = np.concatenate([odom_t[np.abs(odom_v) > 0.01], drive_t[np.abs(drive_speed) > 0.01]])
    times.sort()
    spans: list[tuple[float, float]] = []
    if times.size == 0:
        return spans
    edges = [(float(odom_t[0]) - 1.0, float(times[0]))]
    gaps = np.where(np.diff(times) > 4.0)[0]
    for i in gaps:
        edges.append((float(times[i]), float(times[i + 1])))
    edges.append((float(times[-1]), float(odom_t[-1]) + 1.0))
    for a, b in edges:
        if b - a > 4.0:
            spans.append((a + 1.5, b - 1.5))
    return spans


def build_events(data, spans, speed_divisor):
    """Time-ordered events (t_warped, kind, payload); kind in scan, odom, drive."""
    events = []

    def in_idle(t: float) -> bool:
        return any(a <= t <= b for a, b in spans)

    def warp(t: float) -> float:
        cut = 0.0
        for a, b in spans:
            if t >= b:
                cut += b - a
            elif t > a:
                cut += t - a
        return t - cut

    for t, r in zip(data["scan_t"], data["scan_ranges"], strict=True):
        if not in_idle(t):
            events.append((warp(t), 1, "scan", r))
    for t, v in zip(data["odom_t"], data["odom_v"], strict=True):
        if not in_idle(t):
            events.append((warp(t), 0, "odom", v / speed_divisor))
    for t, s in zip(data["drive_t"], data["drive_steer"], strict=True):
        if not in_idle(t):
            events.append((warp(t), 0, "drive", s))
    events.sort(key=lambda e: (e[0], e[1]))
    return events


class Feeder(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("bag_feeder")
        self.args = args
        data = np.load(args.npz)
        meta = data["scan_meta"]
        self.angle_min, self.angle_inc, self.range_min, self.range_max = (float(x) for x in meta)
        self.n_beams = data["scan_ranges"].shape[1]
        self.left_scale = args.steer_scale_left
        self.right_scale = args.steer_scale_right
        vehicle = yaml.safe_load(VEHICLE_PARAMS.read_text())
        self.wheelbase = float(vehicle["chassis"]["wheelbase_m"])
        lidar = vehicle["sensors"]["lidar"]
        self.mount = (
            float(lidar["mount_x_m"]),
            float(lidar["mount_y_m"]),
            float(lidar["mount_yaw_rad"]),
        )
        spans = active_mask_spans(
            data["odom_t"], data["odom_v"], data["drive_t"], data["drive_speed"]
        )
        self.idle_cut_s = sum(b - a for a, b in spans)
        self.events = build_events(data, spans, args.speed_divisor)
        self.get_logger().info(
            f"{len(self.events)} events, {self.idle_cut_s:.0f} s of idle cut, steering scale "
            f"left {self.left_scale:.3f} right {self.right_scale:.3f}, wheelbase {self.wheelbase}"
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
        )
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50
        )
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", sensor_qos)
        self.odom_pub = self.create_publisher(Odometry, "/odom/wheel", sensor_qos)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", reliable)
        self.twist_pub = self.create_publisher(
            TwistWithCovarianceStamped, "/odom/wheel_twist", reliable
        )
        self.static_tf = StaticTransformBroadcaster(self)
        self.publish_static()

    def publish_static(self) -> None:
        tf = TransformStamped()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = "laser"
        tf.transform.translation.x = self.mount[0]
        tf.transform.translation.y = self.mount[1]
        tf.transform.rotation.z = math.sin(self.mount[2] / 2.0)
        tf.transform.rotation.w = math.cos(self.mount[2] / 2.0)
        self.static_tf.sendTransform(tf)

    def effective_steering(self, command: float) -> float:
        return command * (self.left_scale if command >= 0.0 else self.right_scale)

    def play(self) -> None:
        t0 = self.events[0][0]
        wall0 = time.monotonic()
        latest_steer = 0.0
        latest_v = 0.0
        last_report = 0.0
        # Let the stack come up and see the static transform before the first message.
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            self.clock_pub.publish(Clock(clock=to_stamp(t0 - 1.0)))
            rclpy.spin_once(self, timeout_sec=0.1)
        events = self.events
        if self.args.limit_s > 0.0:
            events = [e for e in events if e[0] - t0 <= self.args.limit_s]
            self.events = events
        for t, _, kind, payload in events:
            target = wall0 + 3.0 + (t - t0) / self.args.rate
            while True:
                delay = target - time.monotonic()
                if delay <= 0.0:
                    break
                rclpy.spin_once(self, timeout_sec=min(delay, 0.01))
            self.clock_pub.publish(Clock(clock=to_stamp(t)))
            if kind == "scan":
                msg = LaserScan()
                msg.header.stamp = to_stamp(t)
                msg.header.frame_id = "laser"
                msg.angle_min = self.angle_min
                msg.angle_max = self.angle_min + self.angle_inc * (self.n_beams - 1)
                msg.angle_increment = self.angle_inc
                msg.scan_time = 0.1
                msg.range_min = self.range_min
                msg.range_max = self.range_max
                ranges = np.asarray(payload, dtype=float)
                if self.args.deskew != "none":
                    stamp_at, direction = self.args.deskew.split("_")
                    delta = self.effective_steering(latest_steer)
                    ranges = deskew_scan(
                        ranges,
                        self.angle_min,
                        self.angle_inc,
                        latest_v,
                        latest_v * math.tan(delta) / self.wheelbase,
                        0.1,
                        stamp_at,
                        direction == "fwd",
                        self.mount,
                    )
                msg.ranges = [float(x) for x in ranges]
                self.scan_pub.publish(msg)
            elif kind == "drive":
                latest_steer = float(payload)
                msg = AckermannDriveStamped()
                msg.header.stamp = to_stamp(t)
                msg.drive.steering_angle = latest_steer
                self.drive_pub.publish(msg)
            else:
                latest_v = float(payload)
                odom = Odometry()
                odom.header.stamp = to_stamp(t)
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_link"
                odom.pose.pose.orientation.w = 1.0
                odom.twist.twist.linear.x = float(payload)
                self.odom_pub.publish(odom)
                if self.args.twist_source == "feeder":
                    self.publish_twist(t, float(payload), latest_steer)
            if t - last_report > 30.0:
                last_report = t
                self.get_logger().info(
                    f"replay t = {t - t0:.0f} s of {self.events[-1][0] - t0:.0f}"
                )
            rclpy.spin_once(self, timeout_sec=0.0)

    def tail(self, seconds: float, out_prefix: str) -> None:
        """Park the car: the last scan again at 10 Hz and zero wheel speed at 50 Hz, the map saved
        (the SaveMap service only) halfway, the way the L5 mapping canary leaves slam_toolbox."""
        scan_t = self.events[-1][0]
        last_scan = next(e[3] for e in reversed(self.events) if e[2] == "scan")
        end = time.monotonic() + seconds
        saved = False
        step = 0
        while time.monotonic() < end:
            scan_t += 0.02
            step += 1
            self.clock_pub.publish(Clock(clock=to_stamp(scan_t)))
            odom = Odometry()
            odom.header.stamp = to_stamp(scan_t)
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.orientation.w = 1.0
            self.odom_pub.publish(odom)
            if self.args.twist_source == "feeder":
                self.publish_twist(scan_t, 0.0, 0.0)
            if step % 5 == 0:
                msg = LaserScan()
                msg.header.stamp = to_stamp(scan_t)
                msg.header.frame_id = "laser"
                msg.angle_min = self.angle_min
                msg.angle_max = self.angle_min + self.angle_inc * (self.n_beams - 1)
                msg.angle_increment = self.angle_inc
                msg.scan_time = 0.1
                msg.range_min = self.range_min
                msg.range_max = self.range_max
                msg.ranges = [float(x) for x in last_scan]
                self.scan_pub.publish(msg)
            if not saved and time.monotonic() > end - seconds / 2.0:
                from slam_toolbox.srv import SaveMap

                client = self.create_client(SaveMap, "/slam_toolbox/save_map")
                if client.wait_for_service(timeout_sec=10.0):
                    request = SaveMap.Request()
                    request.name.data = out_prefix
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
                saved = True
            rclpy.spin_once(self, timeout_sec=0.02)

    def publish_twist(self, t: float, v: float, command: float) -> None:
        delta = self.effective_steering(command)
        tan_d, cos_d = math.tan(delta), math.cos(delta)
        length = self.wheelbase
        speed_std = SPEED_STD_FLOOR + SPEED_STD_FRACTION * abs(v)
        var_vx = speed_std**2
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = to_stamp(t)
        msg.header.frame_id = "base_link"
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = v * tan_d / length
        d_wz_d_delta = v / (length * cos_d * cos_d)
        var_wz = (
            (d_wz_d_delta * STEERING_STD) ** 2
            + (tan_d / length) ** 2 * var_vx
            + YAW_RATE_STD_FLOOR**2
        )
        cov = [0.0] * 36
        cov[0], cov[7], cov[35] = var_vx, LATERAL_STD**2, var_wz
        cov[14] = cov[21] = cov[28] = 1e6
        msg.twist.covariance = cov
        self.twist_pub.publish(msg)


def save_maps(node: Node, out_prefix: str) -> bool:
    from slam_toolbox.srv import SaveMap, SerializePoseGraph

    ok = True
    for srv_type, name, field in (
        (SaveMap, "/slam_toolbox/save_map", "name"),
        (SerializePoseGraph, "/slam_toolbox/serialize_map", "filename"),
    ):
        client = node.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=20.0):
            node.get_logger().error(f"{name} not available")
            ok = False
            continue
        request = srv_type.Request()
        if field == "name":
            request.name.data = out_prefix
        else:
            request.filename = out_prefix
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=60.0)
        result = future.result()
        node.get_logger().info(f"{name}: {result}")
        ok = ok and result is not None
    return ok


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--npz", required=True, type=Path)
    parser.add_argument("--out-prefix", required=True, help="map path without extension")
    parser.add_argument(
        "--speed-divisor",
        type=float,
        default=1.0,
        help="divide /odom/wheel speeds by this (reading over true)",
    )
    parser.add_argument("--steer-scale-left", type=float, default=1.0)
    parser.add_argument("--steer-scale-right", type=float, default=1.0)
    parser.add_argument("--twist-source", choices=("feeder", "node"), default="feeder")
    parser.add_argument("--rate", type=float, default=1.0, help="playback speed")
    parser.add_argument(
        "--limit-s", type=float, default=0.0, help="replay only the first N seconds (0 = all)"
    )
    parser.add_argument(
        "--tail-s",
        type=float,
        default=0.0,
        help="after the replay keep publishing the last scan and zero speed for this long (s), "
        "saving the map halfway: the parked car of the L5 mapping canary, for shutdown tests",
    )
    parser.add_argument(
        "--deskew",
        default="none",
        choices=("none", "start_fwd", "start_rev", "end_fwd", "end_rev"),
        help="correct the scan's motion distortion with the wheel odometry: where the stamp sits "
        "in the 0.1 s sweep and whether beam time rises with the beam index",
    )
    args, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = Feeder(args)
    try:
        node.play()
        if args.tail_s > 0.0:
            node.tail(args.tail_s, args.out_prefix)
            rc = 0
        else:
            time.sleep(2.0)
            rc = 0 if save_maps(node, args.out_prefix) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
