"""lidar_check: listen to /scan for a while, report rate and shape, exit non-zero on a bad stream.

Roadmap 2.3 ("LiDAR mounted, driver up, scan rate verified"). Run it next to a running
racer_bringup/launch/lidar.launch.py (or car_teleop.launch.py lidar:=true):

    ros2 run racer_tools lidar_check
    ros2 run racer_tools lidar_check --ros-args -p duration_s:=30.0

It subscribes for `duration_s` seconds of wall time, then prints scan rate (from header
stamps), beam count, declared field of view, the declared and observed range band and the
fraction of invalid returns, and exits 0 on PASS, 1 on FAIL, 2 if fewer than two scans
arrived. Expectations come from config/vehicle_params.yaml's sensors.lidar_spec through the
generated binding (CLAUDE.md invariant 2); see racer_tools/scan_check.py for the rules.

This file is thin rclpy plumbing. All statistics and pass/fail logic live in
racer_tools/scan_check.py and are unit-tested without ROS (claude-docs/12-testing.md L1).

QoS: best_effort, KeepLast with an explicit depth (claude-docs/10-conventions.md: sensor data
best_effort, explicit depth). sllidar_ros2 publishes reliable; a best_effort subscriber
matches a reliable publisher, which is also how racer_safety/safety_node subscribes.
"""

from __future__ import annotations

import sys
import time

import rclpy
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from racer_tools.scan_check import (
    ScanSample,
    compute_stats,
    evaluate,
    expectations_from_spec,
    format_report,
)
from racer_tools.vehicle_params_loader import load_vehicle_params

EXIT_PASS = 0
EXIT_FAIL = 1
EXIT_NO_DATA = 2

# Enough to absorb a burst at any plausible LiDAR rate between spins; the check itself keeps
# every scan it is handed.
_SCAN_QOS_DEPTH = 50


class LidarCheckNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_check")
        self.declare_parameter(
            "topic",
            "/scan",
            ParameterDescriptor(description="LaserScan topic to check."),
        )
        self.declare_parameter(
            "duration_s",
            10.0,
            ParameterDescriptor(
                description="Seconds of wall time to listen before reporting.",
                floating_point_range=[FloatingPointRange(from_value=1.0, to_value=600.0, step=0.0)],
            ),
        )
        self.declare_parameter(
            "min_rate_hz",
            -1.0,
            ParameterDescriptor(
                description=(
                    "Minimum acceptable scan rate, Hz. <= 0 (the default) uses "
                    "vehicle_params sensors.lidar_spec.min_scan_rate_hz."
                ),
                floating_point_range=[
                    FloatingPointRange(from_value=-1.0, to_value=100.0, step=0.0)
                ],
            ),
        )
        self.declare_parameter(
            "max_invalid_fraction",
            0.5,
            ParameterDescriptor(
                description=(
                    "Fail if more than this fraction of all returns is invalid (non-finite or "
                    "outside the message's range band). A check threshold, not a sensor "
                    "property: an open room with far walls legitimately shows some."
                ),
                floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.0)],
            ),
        )
        self.declare_parameter(
            "angle_compensate",
            True,
            ParameterDescriptor(
                description=(
                    "Must match the driver's angle_compensate (lidar.launch.py default true). "
                    "Decides whether the beam count is checked exactly or loosely."
                ),
            ),
        )

        self.topic = self.get_parameter("topic").value
        self.duration_s = float(self.get_parameter("duration_s").value)
        min_rate = float(self.get_parameter("min_rate_hz").value)
        spec = load_vehicle_params().sensors.lidar_spec
        self.model = spec.model
        self.expectations = expectations_from_spec(
            spec,
            angle_compensate=bool(self.get_parameter("angle_compensate").value),
            max_invalid_fraction=float(self.get_parameter("max_invalid_fraction").value),
            min_rate_hz=min_rate if min_rate > 0.0 else None,
        )

        self.samples: list[ScanSample] = []
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=_SCAN_QOS_DEPTH,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(LaserScan, self.topic, self._on_scan, qos)

    def _on_scan(self, msg: LaserScan) -> None:
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.samples.append(
            ScanSample(
                stamp_s=stamp_s,
                angle_min_rad=msg.angle_min,
                angle_max_rad=msg.angle_max,
                angle_increment_rad=msg.angle_increment,
                range_min_m=msg.range_min,
                range_max_m=msg.range_max,
                ranges=tuple(msg.ranges),
            )
        )


def run(node: LidarCheckNode) -> int:
    node.get_logger().info(
        f"listening to {node.topic} for {node.duration_s:.1f} s, expecting a {node.model}"
    )
    deadline = time.monotonic() + node.duration_s
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if len(node.samples) < 2:
        print(
            f"lidar_check: received {len(node.samples)} scan(s) on {node.topic} in "
            f"{node.duration_s:.1f} s. Is lidar.launch.py running and the head spinning?",
            file=sys.stderr,
        )
        return EXIT_NO_DATA

    stats = compute_stats(node.samples)
    failures = evaluate(stats, node.expectations)
    print(f"lidar_check: {node.model} on {node.topic}")
    print(format_report(stats, node.expectations, failures))
    return EXIT_FAIL if failures else EXIT_PASS


def main(args: list | None = None) -> None:
    rclpy.init(args=args)
    node: LidarCheckNode | None = None
    code = EXIT_FAIL
    try:
        node = LidarCheckNode()
        code = run(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
