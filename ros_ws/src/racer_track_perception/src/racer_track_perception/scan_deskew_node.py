"""scan_deskew_node: /scan in, /pnp/scan_deskewed out (P1.4, claude-docs/13-plug-and-play.md 15.6).

A relay. Every decision (which odometry a sweep gets, when it passes through, the numbers of the
status) lives in the ROS-free `deskew_relay` and `deskew` modules; this file is plumbing.

Inputs: /scan (best effort, like the other consumers), /odom/wheel (speed) and /drive (the gated
steering, read only). Outputs: /pnp/scan_deskewed (LaserScan on the same frame and angle grid,
reliable keep-last 10 like the driver's /scan so a reliable and a best effort consumer both
match) and /pnp/deskew/status (JSON, 1 Hz). Stamp = input stamp plus the offset that makes it the
reference time, scan_time = 0 and time_increment = 0 (the sweep is gone, so a consumer that adds
half a sweep to the stamp, like lap_memory_node, adds nothing), intensities dropped.

Exactly one output per input and in the same order: with no fresh wheel speed, at standstill, or
on any exception the input message is republished UNCHANGED (original stamp and timing fields)
and the reason is counted. The node never drops, delays or reorders a scan. It never publishes
/scan, /drive_raw or /drive; safety_node keeps reading the raw /scan.

`stamp_is` and `beam_order` have no default: they are what the measurement M-DSK chose on real
bags (docs/notes/preregistration-pnp-v0.3.md section 4), the launch passes them, and the node
refuses to start without them (exit 2) before it creates a publisher.
"""

from __future__ import annotations

import sys

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from racer_track_perception import deskew, deskew_relay
from racer_track_perception.vehicle import VehicleParamsNull, from_generated

SCAN_TOPIC = "/scan"
ODOM_TOPIC = "/odom/wheel"
DRIVE_TOPIC = "/drive"
OUT_TOPIC = "/pnp/scan_deskewed"
STATUS_TOPIC = "/pnp/deskew/status"
EXIT_REFUSED = 2


class StartupRefused(Exception):
    """The node refuses to start (missing or invalid parameter, null vehicle_params)."""


def _stamp_ns(stamp) -> int:  # type: ignore[no-untyped-def]
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class ScanDeskewNode(Node):
    def __init__(self) -> None:
        super().__init__("scan_deskew_node")
        stamp_is = self.declare_parameter(
            "stamp_is",
            Parameter.Type.STRING,
            ParameterDescriptor(
                description="sweep_start or sweep_end: what header.stamp of /scan means. "
                "No default: the value M-DSK chose."
            ),
        ).value
        beam_order = self.declare_parameter(
            "beam_order",
            Parameter.Type.STRING,
            ParameterDescriptor(
                description="index_ascending or index_descending: whether time runs with the "
                "beam index. No default: the value M-DSK chose."
            ),
        ).value
        reference = self.declare_parameter(
            "reference",
            "sweep_end",
            ParameterDescriptor(description="sweep_start, sweep_mid or sweep_end"),
        ).value
        max_age = self.declare_parameter(
            "max_twist_age_s",
            0.10,
            ParameterDescriptor(
                description="newest wheel speed older than this (s): pass the scan through",
                floating_point_range=[FloatingPointRange(from_value=0.01, to_value=1.0, step=0.0)],
            ),
        ).value

        if not stamp_is:
            raise StartupRefused("the stamp_is parameter is required (sweep_start or sweep_end)")
        if not beam_order:
            raise StartupRefused("the beam_order parameter is required")
        try:
            cfg = deskew.DeskewConfig(
                stamp_is=str(stamp_is),
                beam_order=str(beam_order),
                reference=str(reference),
                max_twist_age_s=float(max_age),
            )
        except ValueError as exc:
            raise StartupRefused(str(exc)) from exc
        vp = self._load_params()
        try:
            live = from_generated(vp)
        except VehicleParamsNull as exc:
            raise StartupRefused(str(exc)) from exc
        table = tuple(
            (float(p.abs_angle_rad), float(p.gain)) for p in vp.steering.yaw_rate_gain_table
        )
        self._relay = deskew_relay.DeskewRelay(cfg, live.mount, live.body, table)
        self._stats = deskew_relay.DeskewStats()

        # Publishers and subscriptions only now: a refusal above leaves none.
        qos_in = lambda depth: QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=depth
        )
        reliable = lambda depth: QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=depth
        )
        self._out_pub = self.create_publisher(LaserScan, OUT_TOPIC, reliable(10))
        self._status_pub = self.create_publisher(String, STATUS_TOPIC, reliable(10))
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, qos_in(50))
        self.create_subscription(AckermannDriveStamped, DRIVE_TOPIC, self._on_drive, qos_in(10))
        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, qos_in(10))
        self.create_timer(1.0, self._on_status_timer)
        self.get_logger().info(
            f"scan_deskew_node: stamp_is {cfg.stamp_is}, beam_order {cfg.beam_order}, "
            f"reference {cfg.reference}, max_twist_age_s {cfg.max_twist_age_s}"
        )

    def _load_params(self):  # type: ignore[no-untyped-def]
        from racer_tools.vehicle_params_loader import load_vehicle_params

        return load_vehicle_params()

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg: Odometry) -> None:
        self._relay.on_odom(
            self._now_s(), _stamp_ns(msg.header.stamp) * 1e-9, float(msg.twist.twist.linear.x)
        )

    def _on_drive(self, msg: AckermannDriveStamped) -> None:
        self._relay.on_drive(
            self._now_s(), _stamp_ns(msg.header.stamp) * 1e-9, float(msg.drive.steering_angle)
        )

    def _on_scan(self, scan: LaserScan) -> None:
        now = self._now_s()
        self._stats.scan_in(now)
        try:
            result = self._relay.process(
                now,
                _stamp_ns(scan.header.stamp) * 1e-9,
                scan.ranges,
                float(scan.angle_min),
                float(scan.angle_increment),
                float(scan.range_min),
                float(scan.range_max),
                float(scan.scan_time),
                float(scan.time_increment),
            )
            out = scan if result.ranges is None else self._deskewed_message(scan, result)
        except Exception:  # noqa: BLE001 - never lose a scan
            result = deskew_relay.RelayResult(None, 0.0, "exception", None, None, 0.0)
            out = scan
        self._out_pub.publish(out)
        self._stats.scan_out(result)

    def _deskewed_message(self, scan: LaserScan, result) -> LaserScan:  # type: ignore[no-untyped-def]
        out = LaserScan()
        out.header.frame_id = scan.header.frame_id
        ns = _stamp_ns(scan.header.stamp) + round(result.stamp_offset_s * 1e9)
        out.header.stamp.sec = ns // 1_000_000_000
        out.header.stamp.nanosec = ns % 1_000_000_000
        out.angle_min = scan.angle_min
        out.angle_max = scan.angle_max
        out.angle_increment = scan.angle_increment
        out.time_increment = 0.0
        out.scan_time = 0.0
        out.range_min = scan.range_min
        out.range_max = scan.range_max
        out.ranges = [float(r) for r in result.ranges]
        out.intensities = []
        return out

    def _on_status_timer(self) -> None:
        msg = String()
        msg.data = self._stats.as_json(
            self._now_s(), self._relay.odom.rejected, self._relay.drive.rejected
        )
        self._status_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ScanDeskewNode()
        rclpy.spin(node)
    except StartupRefused as exc:
        print(f"scan_deskew_node: refusing to start: {exc}", file=sys.stderr, flush=True)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        sys.exit(EXIT_REFUSED)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None and rclpy.ok():
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
