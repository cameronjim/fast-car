"""lap_memory_node: the rclpy plumbing around `racer_laps.engine.LapMemory`.

Default lap source `start_scan` (no map after lap 1): reads `/scan`, the wheel odometry
(`/odom/wheel`), the gated steering (`/drive`), the planner status, the safety events and
`/drive_raw`. The planner's first DRIVING report makes the next scan the start scan; every scan
while the gate is armed is matched against it (see racer_laps.gate). With TF `map -> base_link`
(the stack running `mapping:=true`) the pose detector also runs on lap 1 as a cross-check, and at
the first closure the node asks `/slam_toolbox/save_map` once for the lap-1 map (`save_lap1_map`;
failure is logged, never fatal). Lap source `map_pose` is the earlier behaviour, kept for sim
comparisons.

Publishes `/pnp/lap/event` once per closed lap (`header.frame_id` is `start_scan`, or `map` for
`map_pose`), `/pnp/lap/guidance` at `guidance_rate_hz` (10 Hz) and `/pnp/lap/status` (JSON, 1 Hz,
says why no lap has closed). Writes the session's laps to `<output_dir>/station_set_<start>.npz`
(`safe_set_<start>.npz` for `map_pose`) after every lap. Lap mode `v0` is the cap schedule with
empty guidance arrays; `v1` adds the station set (see racer_laps.station_set).

It never publishes `/drive_raw` or `/drive`: it only advises a speed cap and, in `v1`, a speed
bound per station, and the planner (parameter `use_lap_guidance`) takes the minimum of that and
its own limits. All callbacks run on one thread (default single-threaded executor), so the engine
needs no lock.
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
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from racer_laps.engine import LAP_MODES, LAP_SOURCES, LapConfig, LapMemory, LapResult
from racer_laps.gate import GateConfig
from racer_laps.scan_match import Body, Mount, scan_to_points
from racer_laps.vehicle import LapVehicle, VehicleParamsNull, from_generated

_DEFAULTS = LapConfig()
_GATE = GateConfig(wheelbase_m=1.0)


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


def _stamp_s(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


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

        def declare_int(name: str, default: int, description: str, lo: int, hi: int) -> int:
            rng = IntegerRange(from_value=lo, to_value=hi, step=1)
            self.declare_parameter(
                name, default, ParameterDescriptor(description=description, integer_range=[rng])
            )
            return int(self.get_parameter(name).value)

        def declare_str(name: str, default: str, description: str) -> str:
            self.declare_parameter(name, default, ParameterDescriptor(description=description))
            return str(self.get_parameter(name).value)

        lap_source = declare_str(
            "lap_source", "start_scan", f"One of {list(LAP_SOURCES)}: how a lap closes."
        )
        lap_mode = declare_str("lap_mode", "v0", f"One of {list(LAP_MODES)}: D v0 or D v1.")
        v_start = declare(
            "v_cap_start_mps", _DEFAULTS.v_cap_start_mps, "Speed cap of lap 1.", 0.05, 5.0
        )
        v_step = declare(
            "v_cap_step_mps", _DEFAULTS.v_cap_step_mps, "Cap change per lap (D v0).", 0.01, 1.0
        )
        v_ceiling = declare(
            "v_cap_ceiling_mps",
            _DEFAULTS.v_cap_ceiling_mps,
            "Highest cap (the launch passes the dynamics bundle's validity.v_max_mps).",
            0.05,
            10.0,
        )
        explore = declare(
            "explore_dv_mps",
            _DEFAULTS.explore_dv_mps,
            "D v1: speed added to the best speed already driven at a station.",
            0.0,
            1.0,
        )
        caution_before = declare(
            "caution_before_m",
            _DEFAULTS.caution_before_m,
            "D v1: length of a caution zone, metres of station before the event.",
            0.1,
            5.0,
        )
        caution_laps = declare_int(
            "caution_laps", _DEFAULTS.caution_laps, "D v1: laps a caution zone stays active.", 0, 20
        )
        min_filter = declare(
            "min_filter_half_m",
            _DEFAULTS.min_filter_half_m,
            "D v1: half width of the MIN filter over the stored speed profile.",
            0.0,
            2.0,
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
            "Path length needed since the last crossing before a crossing counts (lap 1 for the "
            "start-scan gate).",
            0.5,
            200.0,
        )
        line_len = declare(
            "start_line_length_m",
            _DEFAULTS.start_line_length_m,
            "map_pose only: length of the start line, centred on the car.",
            0.2,
            10.0,
        )
        heading_tol = declare(
            "start_heading_tolerance_rad",
            _DEFAULTS.start_heading_tolerance_rad,
            "map_pose only: largest heading difference for a crossing to count.",
            0.1,
            1.5,
        )
        horizon = declare(
            "guidance_horizon_m",
            6.0,
            "How far ahead of the car the guidance arrays reach (the planner accepts 2 to 128 "
            "stations of 0.10 m).",
            0.2,
            12.0,
        )
        gate = GateConfig(
            wheelbase_m=1.0,  # replaced below once vehicle_params is loaded
            min_lap_length_m=float(min_len),
            arm_fraction=declare(
                "arm_fraction",
                _GATE.arm_fraction,
                "Share of the reference lap length driven before the gate arms (laps 2 and on).",
                0.1,
                1.0,
            ),
            heading_slack_rad=declare(
                "heading_slack_rad",
                _GATE.heading_slack_rad,
                "The gate arms once the integrated heading has turned 2 pi minus this.",
                0.0,
                3.0,
            ),
            rms_max_m=declare(
                "match_rms_max_m",
                _GATE.rms_max_m,
                "Largest ICP RMS of an accepted match.",
                0.005,
                0.2,
            ),
            inlier_min_share=declare(
                "match_inlier_min_share",
                _GATE.inlier_min_share,
                "Smallest inlier share of an accepted match.",
                0.1,
                1.0,
            ),
            inlier_dist_m=declare(
                "match_inlier_dist_m", _GATE.inlier_dist_m, "Inlier distance of a match.", 0.01, 0.3
            ),
            lateral_max_m=declare(
                "match_lateral_max_m",
                _GATE.lateral_max_m,
                "Largest lateral offset from the start pose of an accepted match.",
                0.1,
                3.0,
            ),
            yaw_max_rad=declare(
                "match_yaw_max_rad",
                _GATE.yaw_max_rad,
                "Largest relative yaw to the start pose of an accepted match.",
                0.1,
                1.5,
            ),
            max_match_gap_s=declare(
                "max_match_gap_s",
                _GATE.max_match_gap_s,
                "Two accepted matches further apart than this are not a crossing.",
                0.05,
                5.0,
            ),
            missed_factor=declare(
                "missed_factor",
                _GATE.missed_factor,
                "More reference laps than this of wheel distance without a crossing: station "
                "uncertain.",
                1.05,
                5.0,
            ),
            min_along_support=declare_int(
                "min_along_support",
                _GATE.min_along_support,
                "Inlier matches that must pin the along-lane offset for a crossing (0 = off).",
                0,
                200,
            ),
        )
        self._pose_period_s = 1.0 / float(
            declare("pose_rate_hz", 20.0, "TF sampling rate (lap-1 cross-check).", 1.0, 200.0)
        )
        self._pose_timeout_s = float(
            declare("pose_timeout_s", 0.5, "A transform older than this is stale.", 0.05, 10.0)
        )
        guidance_period_s = 1.0 / float(
            declare("guidance_rate_hz", 10.0, "Guidance publish rate.", 0.1, 50.0)
        )
        status_period_s = float(declare("status_period_s", 1.0, "Status period.", 0.1, 60.0))
        yaw_gain_override = float(
            declare(
                "yaw_gain_override",
                0.0,
                "Yaw-rate gain of the gate's heading integral: 0 uses vehicle_params' "
                "steering.yaw_rate_gain_table (the car). The sim's car turns as commanded: 1.0.",
                0.0,
                3.0,
            )
        )
        self.declare_parameter(
            "save_lap1_map",
            True,
            ParameterDescriptor(
                description="At the first closure call /slam_toolbox/save_map once, if it exists."
            ),
        )
        self.declare_parameter(
            "output_dir",
            "/workspace/data/pnp/laps",
            ParameterDescriptor(description="Where station_set_<start time>.npz is written."),
        )
        self.declare_parameter(
            "map_frame", "map", ParameterDescriptor(description="Pose parent frame.")
        )
        self.declare_parameter(
            "base_frame", "base_link", ParameterDescriptor(description="Pose child frame.")
        )
        self.declare_parameter(
            "laser_yaw_from_vehicle_params",
            True,
            ParameterDescriptor(
                description="true (the car): LiDAR mount from vehicle_params. false: simulator "
                "and synthetic scans only, laser_yaw_offset_rad is used as the mount yaw"
            ),
        )
        self.declare_parameter(
            "laser_yaw_offset_rad",
            0.0,
            ParameterDescriptor(description="Mount yaw when not taken from vehicle_params."),
        )
        self._output_dir = Path(str(self.get_parameter("output_dir").value))
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._save_lap1_map = bool(self.get_parameter("save_lap1_map").value)
        if lap_source not in LAP_SOURCES or lap_mode not in LAP_MODES:
            raise ValueError(
                f"lap_source {lap_source!r} / lap_mode {lap_mode!r} not in {LAP_SOURCES} / {LAP_MODES}"
            )

        self._vehicle = self._load_vehicle()
        mount = self._vehicle.mount
        if not bool(self.get_parameter("laser_yaw_from_vehicle_params").value):
            mount = Mount(
                mount.x_m, mount.y_m, float(self.get_parameter("laser_yaw_offset_rad").value)
            )
            self.get_logger().warn(
                f"laser_yaw_from_vehicle_params is false: LiDAR mount yaw {mount.yaw_rad} rad "
                "(simulator and synthetic scans only)"
            )
        self._mount: Mount = mount
        self._body: Body = self._vehicle.body
        table = self._vehicle.yaw_gain_table
        if yaw_gain_override > 0.0:
            table = ((0.0, yaw_gain_override),)
        gate = GateConfig(
            wheelbase_m=self._vehicle.wheelbase_m,
            min_lap_length_m=gate.min_lap_length_m,
            arm_fraction=gate.arm_fraction,
            heading_slack_rad=gate.heading_slack_rad,
            rms_max_m=gate.rms_max_m,
            inlier_min_share=gate.inlier_min_share,
            inlier_dist_m=gate.inlier_dist_m,
            lateral_max_m=gate.lateral_max_m,
            yaw_max_rad=gate.yaw_max_rad,
            max_match_gap_s=gate.max_match_gap_s,
            missed_factor=gate.missed_factor,
            yaw_gain_table=table,
            min_along_support=gate.min_along_support,
        )
        cfg = LapConfig(
            v_cap_start_mps=float(v_start),
            v_cap_step_mps=float(v_step),
            v_cap_ceiling_mps=float(v_ceiling),
            min_clearance_clean_m=float(clearance),
            min_lap_length_m=float(min_len),
            start_line_length_m=float(line_len),
            start_heading_tolerance_rad=float(heading_tol),
            guidance_horizon_m=float(horizon),
            lap_source=lap_source,
            lap_mode=lap_mode,
            explore_dv_mps=float(explore),
            caution_before_m=float(caution_before),
            caution_laps=int(caution_laps),
            min_filter_half_m=float(min_filter),
            gate=gate if lap_source == "start_scan" else None,
        )
        # A bad cap schedule (ceiling below start, ...) raises here and the node exits non-zero.
        self._mem = LapMemory(cfg)
        self._cfg = cfg
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        kind = "station_set" if lap_source == "start_scan" else "safe_set"
        self._save_path = self._output_dir / f"{kind}_{stamp}.npz"
        self._tf_ok = False
        self._tf_age_s = math.nan
        self._tf_error = ""
        self._save_error = ""
        self._map_save = "not_requested"
        self._map_client: Any = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._event_pub = self.create_publisher(LapEvent, "/pnp/lap/event", _qos(True, 50))
        self._guidance_pub = self.create_publisher(LapGuidance, "/pnp/lap/guidance", _qos(True, 5))
        self._status_pub = self.create_publisher(String, "/pnp/lap/status", _qos(True, 10))
        self.create_subscription(
            PlannerStatus, "/pnp/planner/status", self._on_planner, _qos(True, 50)
        )
        self.create_subscription(SafetyEvent, "/safety/events", self._on_safety, _qos(True, 50))
        self.create_subscription(Odometry, "/odom/wheel", self._on_odom, _qos(False, 50))
        self.create_subscription(
            AckermannDriveStamped, "/drive_raw", self._on_drive_raw, _qos(False, 10)
        )
        self.create_subscription(AckermannDriveStamped, "/drive", self._on_drive, _qos(False, 10))
        self.create_subscription(LaserScan, "/scan", self._on_scan, _qos(False, 5))
        self.create_timer(self._pose_period_s, self._on_pose_timer)
        self.create_timer(guidance_period_s, self._publish_guidance)
        self.create_timer(status_period_s, self._publish_status)
        self.get_logger().info(
            f"lap memory up: source {lap_source}, mode {lap_mode}, lap 1 cap "
            f"{cfg.v_cap_start_mps} m/s, ceiling {cfg.v_cap_ceiling_mps} m/s, "
            f"laps -> {self._save_path}"
        )

    def _load_vehicle(self) -> LapVehicle:
        from racer_tools.vehicle_params_loader import load_vehicle_params

        try:
            return from_generated(load_vehicle_params())
        except VehicleParamsNull as exc:
            raise ValueError(str(exc)) from exc

    # -- inputs ----------------------------------------------------------------------------

    def _on_planner(self, msg: PlannerStatus) -> None:
        self._mem.on_planner_status(int(msg.state), float(msg.plan_min_clearance_m))

    def _on_safety(self, msg: SafetyEvent) -> None:
        self._mem.on_safety_event(int(msg.phase), int(msg.severity), str(msg.source))

    def _on_odom(self, msg: Odometry) -> None:
        stamp = _stamp_s(msg.header.stamp)
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9
        self._mem.on_wheel_odom(stamp, float(msg.twist.twist.linear.x))

    def _on_drive(self, msg: AckermannDriveStamped) -> None:
        self._mem.on_drive(float(msg.drive.steering_angle))

    def _on_drive_raw(self, msg: AckermannDriveStamped) -> None:
        self._mem.on_drive_raw(float(msg.drive.speed), float(msg.drive.steering_angle))

    def _on_scan(self, msg: LaserScan) -> None:
        if self._cfg.lap_source != "start_scan":
            return
        stamp = _stamp_s(msg.header.stamp)
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9
        sweep = float(msg.scan_time)
        if not (math.isfinite(sweep) and sweep > 0.0):
            sweep = float(msg.time_increment) * max(len(msg.ranges) - 1, 0)
        pts = scan_to_points(
            msg.ranges,
            float(msg.angle_min),
            float(msg.angle_increment),
            float(msg.range_min),
            float(msg.range_max),
            self._mount,
            self._body,
        )
        result = self._mem.on_scan(stamp + 0.5 * sweep, pts)
        if result is not None:
            self._on_lap(result)

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
        stamp_s = _stamp_s(tf.header.stamp)
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
        msg.header.frame_id = "start_scan" if r.source == "start_scan" else self._map_frame
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
            f"{'clean' if r.clean else 'UNCLEAN'}{' (missed gate, not a lap)' if r.missed_gate else ''}"
            f", brakes {r.safety_brakes} (obstacle {r.obstacle_brakes}, timing {r.timing_brakes}), "
            f"next cap {r.next_cap_mps:.2f}"
        )
        if r.lap_index == 1 and not r.missed_gate and self._save_lap1_map:
            self._request_map_save()
        try:
            if r.source == "start_scan":
                self._mem.station_set.save(self._save_path)
            else:
                self._mem.safe_set.save(self._save_path)
            self._save_error = ""
        except OSError as error:
            self._save_error = f"{type(error).__name__}: {error}"
            self.get_logger().error(f"could not write {self._save_path}: {error}")

    def _request_map_save(self) -> None:
        """Ask slam_toolbox for the lap-1 map, once. Never fatal."""
        try:
            from slam_toolbox.srv import SaveMap
            from std_msgs.msg import String as StringMsg

            if self._map_client is None:
                self._map_client = self.create_client(SaveMap, "/slam_toolbox/save_map")
            if not self._map_client.service_is_ready():
                self._map_save = "service_absent"
                self.get_logger().info("no /slam_toolbox/save_map: lap-1 map not saved")
                return
            self._output_dir.mkdir(parents=True, exist_ok=True)
            name = self._output_dir / self._save_path.stem.replace("station_set", "lap1_map")
            request = SaveMap.Request()
            request.name = StringMsg(data=str(name))
            future = self._map_client.call_async(request)
            self._map_save = "requested"

            def done(fut: Any) -> None:
                try:
                    ok = fut.result() is not None
                except Exception as error:  # noqa: BLE001 - a failed save is logged, not fatal
                    ok = False
                    self.get_logger().warn(f"lap-1 map save failed: {error}")
                self._map_save = "saved" if ok else "failed"
                if ok:
                    self.get_logger().info(f"lap-1 map saved as {name}")

            future.add_done_callback(done)
        except Exception as error:  # noqa: BLE001 - never fatal: the map is a by-product
            self._map_save = "failed"
            self.get_logger().warn(f"lap-1 map save not possible: {error}")

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
        status["lap1_map_save"] = self._map_save
        status["safe_set_path"] = str(self._save_path)
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
