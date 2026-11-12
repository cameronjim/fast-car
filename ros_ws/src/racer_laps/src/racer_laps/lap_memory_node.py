"""lap_memory_node: the rclpy plumbing around `racer_laps.engine.LapMemory`.

Reads the pose from TF `map -> base_link` at `pose_rate_hz` (20), the planner status, the safety
events, the wheel odometry and `/drive_raw`. Publishes `/pnp/lap/event` once per completed lap,
`/pnp/lap/guidance` at 2 Hz and `/pnp/lap/status` (JSON, 1 Hz) so a missing or stale transform is
visible. Writes the session's safe set to `<output_dir>/safe_set_<start time>.npz` after every
completed lap.

It never publishes `/drive_raw` or `/drive`: it only advises a speed cap, and the planner takes
the minimum of that and its own `v_max_mps`.

All callbacks run on one thread (default single-threaded executor), so the engine needs no lock.
"""

from __future__ import annotations

import json
import logging
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
import tf2_ros
from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Time as TimeMsg
from nav_msgs.msg import Odometry
from racer_msgs.msg import SafetyEvent
from racer_pnp_msgs.msg import LapEvent, LapGuidance, PlannerStatus
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String

from racer_laps.engine import LapConfig, LapMemory, LapResult

_DEFAULTS = LapConfig()


def _qos(reliable: bool, depth: int) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
    )


def _time_msg(t_s: float) -> TimeMsg:
    sec = math.floor(t_s)
    return TimeMsg(sec=int(sec), nanosec=round((t_s - sec) * 1e9) % 1_000_000_000)


def _yaw(q: Any) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _json_safe(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class LapMemoryNode(Node):
    def __init__(self) -> None:
        super().__init__("lap_memory_node")

        def declare(name: str, default: Any, description: str, lo: float, hi: float) -> Any:
            rng = FloatingPointRange(from_value=lo, to_value=hi, step=0.0)
            self.declare_parameter(
                name,
                default,
                ParameterDescriptor(description=description, floating_point_range=[rng]),
            )
            return self.get_parameter(name).value

        v_start = declare(
            "v_cap_start_mps", _DEFAULTS.v_cap_start_mps, "Speed cap of lap 1.", 0.05, 5.0
        )
        v_step = declare(
            "v_cap_step_mps", _DEFAULTS.v_cap_step_mps, "Cap change per lap.", 0.01, 1.0
        )
        v_ceiling = declare(
            "v_cap_ceiling_mps",
            _DEFAULTS.v_cap_ceiling_mps,
            "Highest cap (the launch passes the dynamics bundle's validity.v_max_mps).",
            0.05,
            10.0,
        )
        clearance = declare(
            "min_clearance_clean_m",
            _DEFAULTS.min_clearance_clean_m,
            "A lap is clean only if the plan clearance stayed at least this.",
            0.0,
            1.0,
        )
        min_len = declare(
            "min_lap_length_m",
            _DEFAULTS.min_lap_length_m,
            "Path length needed since the last crossing before a crossing counts.",
            0.5,
            200.0,
        )
        line_len = declare(
            "start_line_length_m",
            _DEFAULTS.start_line_length_m,
            "Length of the start line, centred on the car.",
            0.2,
            10.0,
        )
        heading_tol = declare(
            "start_heading_tolerance_rad",
            _DEFAULTS.start_heading_tolerance_rad,
            "Largest heading difference to the start heading for a crossing to count.",
            0.1,
            1.5,
        )
        horizon = declare(
            "guidance_horizon_m",
            _DEFAULTS.guidance_horizon_m,
            "How far ahead of the car the guidance arrays reach.",
            0.1,
            50.0,
        )
        self._pose_period_s = 1.0 / float(
            declare("pose_rate_hz", 20.0, "TF sampling rate.", 1.0, 200.0)
        )
        self._pose_timeout_s = float(
            declare("pose_timeout_s", 0.5, "A transform older than this is stale.", 0.05, 10.0)
        )
        guidance_period_s = 1.0 / float(
            declare("guidance_rate_hz", 2.0, "Guidance publish rate.", 0.1, 50.0)
        )
        status_period_s = float(declare("status_period_s", 1.0, "Status period.", 0.1, 60.0))
        self.declare_parameter(
            "output_dir",
            "/workspace/data/pnp/laps",
            ParameterDescriptor(description="Where safe_set_<start time>.npz is written."),
        )
        self.declare_parameter(
            "map_frame", "map", ParameterDescriptor(description="Pose parent frame.")
        )
        self.declare_parameter(
            "base_frame", "base_link", ParameterDescriptor(description="Pose child frame.")
        )
        self._output_dir = Path(str(self.get_parameter("output_dir").value))
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)

        cfg = LapConfig(
            v_cap_start_mps=float(v_start),
            v_cap_step_mps=float(v_step),
            v_cap_ceiling_mps=float(v_ceiling),
            min_clearance_clean_m=float(clearance),
            min_lap_length_m=float(min_len),
            start_line_length_m=float(line_len),
            start_heading_tolerance_rad=float(heading_tol),
            guidance_horizon_m=float(horizon),
        )
        # A bad cap schedule (ceiling below start, ...) raises here and the node exits non-zero.
        self._mem = LapMemory(cfg)
        self._safe_set_path = self._output_dir / (
            "safe_set_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".npz"
        )
        self._tf_ok = False
        self._tf_age_s = math.nan
        self._tf_error = ""
        self._save_error = ""

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._event_pub = self.create_publisher(LapEvent, "/pnp/lap/event", _qos(True, 50))
        self._guidance_pub = self.create_publisher(LapGuidance, "/pnp/lap/guidance", _qos(True, 5))
        self._status_pub = self.create_publisher(String, "/pnp/lap/status", _qos(True, 10))
        self.create_subscription(
            PlannerStatus, "/pnp/planner/status", self._on_planner, _qos(True, 50)
        )
        self.create_subscription(SafetyEvent, "/safety/events", self._on_safety, _qos(True, 50))
        self.create_subscription(Odometry, "/odom/wheel", self._on_odom, _qos(False, 10))
        self.create_subscription(
            AckermannDriveStamped, "/drive_raw", self._on_drive_raw, _qos(False, 10)
        )
        self.create_timer(self._pose_period_s, self._on_pose_timer)
        self.create_timer(guidance_period_s, self._publish_guidance)
        self.create_timer(status_period_s, self._publish_status)
        self.get_logger().info(
            f"lap memory up: cap {cfg.v_cap_start_mps} + {cfg.v_cap_step_mps} per clean lap up "
            f"to {cfg.v_cap_ceiling_mps} m/s, safe set -> {self._safe_set_path}"
        )

    # -- inputs ----------------------------------------------------------------------------

    def _on_planner(self, msg: PlannerStatus) -> None:
        self._mem.on_planner_status(int(msg.state), float(msg.plan_min_clearance_m))

    def _on_safety(self, msg: SafetyEvent) -> None:
        self._mem.on_safety_event(int(msg.phase), int(msg.severity))

    def _on_odom(self, msg: Odometry) -> None:
        self._mem.on_wheel_speed(float(msg.twist.twist.linear.x))

    def _on_drive_raw(self, msg: AckermannDriveStamped) -> None:
        self._mem.on_drive_raw(float(msg.drive.speed), float(msg.drive.steering_angle))

    def _on_pose_timer(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        try:
            tf = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, Time())
        except tf2_ros.TransformException as error:
            self._tf_ok = False
            self._tf_age_s = math.nan
            self._tf_error = f"{type(error).__name__}"
            self._mem.on_pose_missing()
            return
        stamp_s = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        self._tf_age_s = now_s - stamp_s
        if self._tf_age_s > self._pose_timeout_s:
            self._tf_ok = False
            self._tf_error = "stale"
            self._mem.on_pose_missing()
            return
        self._tf_ok = True
        self._tf_error = ""
        t = tf.transform.translation
        result = self._mem.on_pose(stamp_s, t.x, t.y, _yaw(tf.transform.rotation))
        if result is not None:
            self._on_lap(result)

    # -- outputs ---------------------------------------------------------------------------

    def _on_lap(self, r: LapResult) -> None:
        msg = LapEvent()
        msg.header.stamp = _time_msg(r.t_cross_s)
        msg.header.frame_id = self._map_frame
        msg.lap_index = r.lap_index
        msg.lap_time_s = float(r.lap_time_s)
        msg.lap_length_m = float(r.length_m)
        msg.v_cap_mps = float(r.cap_mps)
        msg.safety_brakes = min(int(r.safety_brakes), 65535)
        msg.min_clearance_m = float(r.min_clearance_m)
        msg.clean = bool(r.clean)
        self._event_pub.publish(msg)
        self.get_logger().info(
            f"lap {r.lap_index}: {r.lap_time_s:.2f} s, {r.length_m:.2f} m, cap {r.cap_mps:.2f}, "
            f"{'clean' if r.clean else 'UNCLEAN'}, brakes {r.safety_brakes}, "
            f"next cap {r.next_cap_mps:.2f}"
        )
        try:
            self._mem.safe_set.save(self._safe_set_path)
            self._save_error = ""
        except OSError as error:
            self._save_error = f"{type(error).__name__}: {error}"
            self.get_logger().error(f"could not write {self._safe_set_path}: {error}")

    def _publish_guidance(self) -> None:
        g = self._mem.guidance()
        msg = LapGuidance()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.lap_index = int(g.lap_index)
        msg.v_cap_mps = float(g.v_cap_mps)
        msg.s_m = [float(v) for v in g.s_m]
        msg.v_ref_mps = [float(v) for v in g.v_ref_mps]
        msg.cost_to_go_s = [float(v) for v in g.cost_to_go_s]
        self._guidance_pub.publish(msg)

    def _publish_status(self) -> None:
        status: dict[str, Any] = {k: _json_safe(v) for k, v in self._mem.status().items()}
        status["tf_ok"] = self._tf_ok
        status["tf_age_s"] = _json_safe(self._tf_age_s)
        status["tf_error"] = self._tf_error
        status["save_error"] = self._save_error
        status["safe_set_path"] = str(self._safe_set_path)
        status["wall_time_s"] = time.time()
        msg = String()
        msg.data = json.dumps(status, separators=(",", ":"))
        self._status_pub.publish(msg)


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    try:
        node = LapMemoryNode()
    except ValueError as error:  # a refusal (bad parameters) is a non-zero exit, not a warning
        logging.getLogger("lap_memory_node").error("refused to start: %s", error)
        rclpy.try_shutdown()
        return 2
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
