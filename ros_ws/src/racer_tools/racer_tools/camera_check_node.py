"""camera_check: listen to a compressed image topic for a while, report frame rate and size.

Cameras are an optional subsystem, outside the project thesis. Run it next to a running
racer_bringup/launch/camera_usb.launch.py, camera_csi.launch.py or cameras.launch.py:

    ros2 run racer_tools camera_check
    ros2 run racer_tools camera_check --ros-args -p topic:=/camera/csi0/image_raw/compressed \\
        -p min_rate_hz:=54.0 -p expected_width:=1280 -p expected_height:=720

It subscribes for `duration_s` seconds of wall time, then prints the frame rate (from header
stamps, and from arrival times as a cross-check), the resolution read out of the JPEG/PNG
bytes, the mean frame size and the payload rate (the bag growth for that topic), and exits 0
on PASS, 1 on FAIL, 2 if fewer than two frames arrived. The thresholds are parameters, not
physical constants; camera mount poses are not needed for this check and are not read.

This file is thin rclpy plumbing. All statistics and pass/fail logic live in
racer_tools/frame_check.py and are unit-tested without ROS (claude-docs/12-testing.md L1).

QoS: best_effort, KeepLast with an explicit depth (claude-docs/10-conventions.md: sensor data
best_effort, explicit depth). A best_effort subscriber matches both a reliable publisher
(usb_cam) and a best_effort one (gscam with use_sensor_data_qos).
"""

from __future__ import annotations

import sys
import time

import rclpy
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from racer_tools.frame_check import (
    Expectations,
    FrameSample,
    compute_stats,
    evaluate,
    format_report,
    image_dimensions,
)

EXIT_PASS = 0
EXIT_FAIL = 1
EXIT_NO_DATA = 2

#: Enough to absorb a burst at 120 fps between spins; the check keeps every frame it is handed.
_IMAGE_QOS_DEPTH = 50


class CameraCheckNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_check")
        self.declare_parameter(
            "topic",
            "/camera/usb/image_raw/compressed",
            ParameterDescriptor(description="sensor_msgs/CompressedImage topic to check."),
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
            54.0,
            ParameterDescriptor(
                description=(
                    "Fail below this frame rate, Hz. Default 54 = 90 percent of the 60 fps the "
                    "camera launch files default to; lower it when the camera runs slower."
                ),
                floating_point_range=[
                    FloatingPointRange(from_value=0.1, to_value=1000.0, step=0.0)
                ],
            ),
        )
        self.declare_parameter(
            "expected_width",
            0,
            ParameterDescriptor(
                description="Fail unless every frame is this wide, pixels. 0 skips the check.",
                integer_range=[IntegerRange(from_value=0, to_value=16384, step=1)],
            ),
        )
        self.declare_parameter(
            "expected_height",
            0,
            ParameterDescriptor(
                description="Fail unless every frame is this tall, pixels. 0 skips the check.",
                integer_range=[IntegerRange(from_value=0, to_value=16384, step=1)],
            ),
        )

        self.topic = self.get_parameter("topic").value
        self.duration_s = float(self.get_parameter("duration_s").value)
        width = int(self.get_parameter("expected_width").value)
        height = int(self.get_parameter("expected_height").value)
        self.expectations = Expectations(
            min_rate_hz=float(self.get_parameter("min_rate_hz").value),
            expected_width=width if width > 0 else None,
            expected_height=height if height > 0 else None,
        )

        self.samples: list[FrameSample] = []
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=_IMAGE_QOS_DEPTH,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(CompressedImage, self.topic, self._on_image, qos)

    def _on_image(self, msg: CompressedImage) -> None:
        data = bytes(msg.data)
        size = image_dimensions(data)
        self.samples.append(
            FrameSample(
                stamp_s=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                receipt_s=time.monotonic(),
                size_bytes=len(data),
                width=None if size is None else size[0],
                height=None if size is None else size[1],
            )
        )


def run(node: CameraCheckNode) -> int:
    node.get_logger().info(f"listening to {node.topic} for {node.duration_s:.1f} s")
    deadline = time.monotonic() + node.duration_s
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    if len(node.samples) < 2:
        print(
            f"camera_check: received {len(node.samples)} frame(s) on {node.topic} in "
            f"{node.duration_s:.1f} s. Is the camera launch running, and is this the "
            "compressed topic (ros2 topic list | grep camera)?",
            file=sys.stderr,
        )
        return EXIT_NO_DATA

    stats = compute_stats(node.samples)
    failures = evaluate(stats, node.expectations)
    print(f"camera_check: {node.topic}")
    print(format_report(stats, node.expectations, failures))
    return EXIT_FAIL if failures else EXIT_PASS


def main(args: list | None = None) -> None:
    rclpy.init(args=args)
    node: CameraCheckNode | None = None
    code = EXIT_FAIL
    try:
        node = CameraCheckNode()
        code = run(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
