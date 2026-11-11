"""policy_node: the learned residual on top of the tracker (roadmap 5.2 scaffolding).

    tracker_node --(/tracker/command)--> policy_node --(/drive_raw)--> safety_node --> /drive

The tracker is launched in HANDOFF mode (`drive_topic:=/tracker/command`): it computes its
command exactly as before but publishes it here instead of on /drive_raw, so the only
publisher of /drive_raw is this node and nothing is double-published or muxed. This node never
publishes /drive (CLAUDE.md invariant 1; claude-docs/04-architecture.md); the safety node,
the hardware mux and everything downstream are unchanged. If this node dies, /drive_raw goes
quiet and safety_node's watchdog brakes, which is the intended failure.

One decision per tracker command (50 Hz, driven by that message, no second timer):
observation as in training (`live_observation`), network, bounded residual, layer-4 envelope,
OOD / stale-input fallback to the plain tracker command (`policy_core`).

Refusal (CLAUDE.md invariant 3): the policy directory goes through `load_deployment` (contract
checksum and version, vehicle_params version, observation schema, the sidecar settings) and the
node exits non-zero with the reason on ANY failure, before it subscribes to anything. There is
no parameter that skips a check.

Published, besides /drive_raw:
  /policy/status  std_msgs/String, JSON, 1 Hz: state, reason, counters, the inference-latency
                  and loop-period jitter histograms.
  /policy/events  std_msgs/String, JSON, on each entry to / exit from fallback.

This is scaffolding. It does NOT deploy on hardware and does not pass gate G3.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

from racer_policy.live_observation import Array, compose_pose, pool_scan
from racer_policy.policy_core import StepResult
from racer_policy.startup import (
    StartupRefusal,
    build_core,
    clamp_for_float32,
    load_vehicle_params,
    yaw_from_quaternion,
)

TRACKER_COMMAND_TOPIC = "/tracker/command"
DRIVE_RAW_TOPIC = "/drive_raw"


def _make_node_class() -> Any:
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    class PolicyNode(Node):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__("policy_node")
            policy_dir = self.declare_parameter("policy_dir", "").value
            raceline_path = self.declare_parameter("raceline_path", "").value
            self._tracker_topic = self.declare_parameter(
                "tracker_command_topic", TRACKER_COMMAND_TOPIC
            ).value
            odom_topic = self.declare_parameter("odom_topic", "/odom").value
            self._pose_frame = self.declare_parameter("pose_frame", "").value
            self._pose_frame_timeout_s = float(
                self.declare_parameter("pose_frame_timeout_s", 1.0).value
            )
            scan_topic = self.declare_parameter("scan_topic", "/scan").value
            scan_timeout_s = float(self.declare_parameter("scan_timeout_s", 0.5).value)
            self._odom_timeout_s = float(self.declare_parameter("odom_timeout_s", 0.3).value)
            self._command_timeout_s = float(self.declare_parameter("command_timeout_s", 0.3).value)
            self._yaw_from_params = bool(
                self.declare_parameter("laser_yaw_from_vehicle_params", True).value
            )
            self._yaw_offset = float(self.declare_parameter("laser_yaw_offset_rad", 0.0).value)
            self._status_period_s = float(self.declare_parameter("status_period_s", 1.0).value)

            try:
                vehicle_params = load_vehicle_params()
            except Exception as exc:
                raise StartupRefusal(f"vehicle_params could not be loaded: {exc}") from exc
            self._core, self._deployment = build_core(
                str(policy_dir), str(raceline_path), vehicle_params, scan_timeout_s
            )
            self._vehicle_params = vehicle_params
            mount_yaw = vehicle_params.sensors.lidar.mount_yaw_rad
            if self._yaw_from_params:
                if mount_yaw is None:
                    raise StartupRefusal(
                        "sensors.lidar.mount_yaw_rad is null and laser_yaw_from_vehicle_params "
                        "is true: the scan cannot be put in the vehicle frame"
                    )
                self._mount_yaw = float(mount_yaw)
            else:
                self._mount_yaw = self._yaw_offset
            self._range_min = float(vehicle_params.sensors.lidar_spec.range_min_m)
            self._steer_limit = float(vehicle_params.steering.max_angle_rad)
            self._speed_low = float(vehicle_params.limits.min_velocity_mps)
            self._speed_high = float(self._deployment.envelope.speed_cap_mps)

            self._steady = Clock(clock_type=ClockType.STEADY_TIME)
            self._odom: Any = None
            self._odom_t = -1e9
            self._beams: Array | None = None
            self._scan_t = -1e9
            self._last_command_t: float | None = None
            self._tf_buffer: Any = None
            if self._pose_frame:
                import tf2_ros

                self._tf_buffer = tf2_ros.Buffer()
                self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

            reliable = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
            )
            sensor = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5
            )
            self.create_subscription(Odometry, odom_topic, self._on_odom, reliable)
            self.create_subscription(LaserScan, scan_topic, self._on_scan, sensor)
            self.create_subscription(
                AckermannDriveStamped, self._tracker_topic, self._on_command, reliable
            )
            self._drive_pub = self.create_publisher(
                AckermannDriveStamped, DRIVE_RAW_TOPIC, reliable
            )
            self._status_pub = self.create_publisher(String, "/policy/status", reliable)
            self._event_pub = self.create_publisher(String, "/policy/events", reliable)
            self.create_timer(self._status_period_s, self._publish_status)
            self._String = String
            self._AckermannDriveStamped = AckermannDriveStamped
            self.get_logger().info(
                f"policy_node up: policy {Path(str(policy_dir)).name} "
                f"(sha256 {self._deployment.contract.policy.checksum_sha256[:12]}), "
                f"tracker handoff on {self._tracker_topic}, publishing {DRIVE_RAW_TOPIC} only"
            )

        # -- inputs -----------------------------------------------------------------------

        def _steady_s(self) -> float:
            return float(self._steady.now().nanoseconds) * 1e-9

        def _on_odom(self, msg: Any) -> None:
            self._odom = msg
            self._odom_t = self._steady_s()

        def _on_scan(self, msg: Any) -> None:
            self._beams = pool_scan(
                msg.ranges,
                msg.angle_min,
                msg.angle_increment,
                self._mount_yaw,
                max(self._range_min, float(msg.range_min)),
            )
            self._scan_t = self._steady_s()

        def _pose(self) -> tuple[float, float, float] | None:
            """The pose the tracker follows: /odom, composed into `pose_frame` by TF when set
            (the localised case), exactly as tracker_node does."""
            odom = self._odom
            p = odom.pose.pose
            pose = (
                p.position.x,
                p.position.y,
                yaw_from_quaternion(
                    p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z
                ),
            )
            if not self._pose_frame:
                return pose
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._pose_frame, odom.header.frame_id, Time()
                )
            except Exception:  # noqa: BLE001 - tf2 raises several exception types
                return None
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            odom_stamp = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9
            if odom_stamp - stamp > self._pose_frame_timeout_s:
                return None
            q = tf.transform.rotation
            frame = (
                tf.transform.translation.x,
                tf.transform.translation.y,
                yaw_from_quaternion(q.w, q.x, q.y, q.z),
            )
            return compose_pose(frame, pose)

        # -- the control step -------------------------------------------------------------

        def _on_command(self, msg: Any) -> None:
            now = self._steady_s()
            # A gap in the tracker's commands (it stops on stale odometry) restarts the
            # envelope's rate-limit memory, like training's episode reset.
            if self._last_command_t is not None and now - self._last_command_t > (
                self._command_timeout_s
            ):
                self._core.reset()
            self._last_command_t = now
            if self._odom is None or now - self._odom_t > self._odom_timeout_s:
                # No fresh odometry: do not invent a command. The tracker is on the same
                # watchdog; /drive_raw goes quiet and safety_node brakes.
                return
            pose = self._pose()
            if pose is None:
                return
            twist = self._odom.twist.twist
            res: StepResult = self._core.step(
                now_s=now,
                velocity_mps=float(twist.linear.x),
                yaw_rate_rad_s=float(twist.angular.z),
                x_m=pose[0],
                y_m=pose[1],
                yaw_rad=pose[2],
                base_steering_rad=float(msg.drive.steering_angle),
                base_speed_mps=float(msg.drive.speed),
                beams=self._beams,
                scan_age_s=(now - self._scan_t) if self._beams is not None else None,
            )
            out = self._AckermannDriveStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = "base_link"
            out.drive.steering_angle = clamp_for_float32(
                res.steering_rad, -self._steer_limit, self._steer_limit
            )
            out.drive.speed = clamp_for_float32(res.speed_mps, self._speed_low, self._speed_high)
            self._drive_pub.publish(out)
            if res.transition is not None:
                self._publish_event(res)

        def _publish_event(self, res: StepResult) -> None:
            record = {
                "event": f"fallback_{res.transition}",
                "reason": res.reason if res.transition == "engage" else self._core.last_reason,
                "duration_s": res.fallback_duration_s,
            }
            self._event_pub.publish(self._String(data=json.dumps(record)))
            self.get_logger().warn(f"policy fallback {res.transition}: {record['reason'] or 'ok'}")

        def _publish_status(self) -> None:
            self._status_pub.publish(self._String(data=json.dumps(self._core.snapshot())))

    return PolicyNode


def main(argv: list[str] | None = None) -> int:
    import rclpy

    rclpy.init(args=argv)
    try:
        node_cls = _make_node_class()
        node = node_cls()
    except StartupRefusal as exc:
        print(f"policy_node: REFUSING TO START: {exc}", file=sys.stderr, flush=True)
        rclpy.shutdown()
        return 2
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
