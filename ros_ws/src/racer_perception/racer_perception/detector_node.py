"""detector_node: YOLO object detection on a CompressedImage topic (roadmap 5.1).

    ros2 run racer_perception detector_node
    ros2 launch racer_perception perception.launch.py        (preferred, in car-camera)

Subscribes `image_topic` (default /camera/usb/image_raw/compressed), runs the detector on at
most `rate_hz` frames per second (default 10, the rest are dropped, newest frame wins) and
publishes vision_msgs/Detection2DArray on /perception/detections: one array per processed
frame, EMPTY when nothing was found, so a consumer can tell "nothing there" from "detector
dead". Each detection carries the class name (hypothesis.class_id), the score and the box in
image pixels; the header is the image's header (stamp and frame_id), unchanged.

Optionally (`publish_debug_image`, default false) a JPEG with the boxes drawn goes out on
`debug_image_topic` for Foxglove.

Needs `ultralytics` and PyTorch, which only the INSTALL_PERCEPTION=required car image has. If
they are missing, or device:=cuda has no CUDA, this logs ONE error and exits with code 3
(yolo_backend.EXIT_DETECTOR_UNAVAILABLE). Weights are downloaded on first run into
`model_cache_dir`, never committed.

Thin rclpy plumbing: the detector is behind racer_perception.yolo_backend (faked in tests),
the rate limiting is racer_perception.throttle, the fusion maths are in fusion.py.

QoS: the image subscription is best_effort, KeepLast(1): usb_cam publishes reliable and a
best_effort subscriber matches it; depth 1 means a slow detector skips frames instead of
queueing them. Detections are published reliable, KeepLast(10), so any subscriber matches.
"""

from __future__ import annotations

import sys
import time

import rclpy
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from racer_perception.fusion import RawDetection
from racer_perception.throttle import FrameThrottle
from racer_perception.yolo_backend import (
    DEFAULT_CACHE_DIR,
    EXIT_DETECTOR_UNAVAILABLE,
    DetectorBackend,
    DetectorUnavailable,
    load_yolo_backend,
)

_STATS_PERIOD_S = 10.0


def to_detection_array(header, detections: list[RawDetection]) -> Detection2DArray:
    """Pack RawDetection records into a Detection2DArray with the image's header."""
    out = Detection2DArray()
    out.header = header
    for d in detections:
        det = Detection2D()
        det.header = header
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = d.class_name
        hyp.hypothesis.score = float(d.score)
        det.results.append(hyp)
        det.bbox.center.position.x = float(d.cx_px)
        det.bbox.center.position.y = float(d.cy_px)
        det.bbox.size_x = float(d.width_px)
        det.bbox.size_y = float(d.height_px)
        out.detections.append(det)
    return out


class DetectorNode(Node):
    def __init__(self, backend: DetectorBackend | None = None, **node_kwargs) -> None:
        super().__init__("detector_node", **node_kwargs)

        def declare(name, default, description, **kw):
            self.declare_parameter(
                name, default, ParameterDescriptor(description=description, **kw)
            )

        declare("image_topic", "/camera/usb/image_raw/compressed", "CompressedImage input.")
        declare("detections_topic", "/perception/detections", "Detection2DArray output.")
        declare("model", "yolov8n.pt", "Model file: a bare name lives in model_cache_dir.")
        declare("device", "cpu", "'cpu', 'cuda' or 'cuda:N'. No silent fallback.")
        declare("model_cache_dir", DEFAULT_CACHE_DIR, "Where weights are cached/downloaded.")
        declare(
            "rate_hz",
            10.0,
            "Maximum frames processed per second; the rest are dropped. <= 0: no limit.",
            floating_point_range=[FloatingPointRange(from_value=-1.0, to_value=240.0, step=0.0)],
        )
        declare(
            "score_threshold",
            0.25,
            "Detections below this confidence are not published.",
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.0)],
        )
        declare("publish_debug_image", False, "Publish a JPEG with boxes drawn.")
        declare(
            "debug_image_topic",
            "/perception/debug_image/compressed",
            "CompressedImage with boxes drawn (only when publish_debug_image).",
        )

        def get(name):
            return self.get_parameter(name).value

        self._publish_debug = bool(get("publish_debug_image"))
        self._throttle = FrameThrottle(float(get("rate_hz")))
        if backend is None:
            backend = load_yolo_backend(
                str(get("model")),
                str(get("device")),
                str(get("model_cache_dir")),
                float(get("score_threshold")),
            )
        self._backend = backend

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        out_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE
        )
        self._pub = self.create_publisher(Detection2DArray, str(get("detections_topic")), out_qos)
        self._debug_pub = None
        if self._publish_debug:
            self._debug_pub = self.create_publisher(
                CompressedImage, str(get("debug_image_topic")), image_qos
            )
        self.create_subscription(
            CompressedImage, str(get("image_topic")), self._on_image, image_qos
        )

        self._infer_s_total = 0.0
        self._infer_count = 0
        self._failures = 0
        self._last_stats_s = time.monotonic()
        self.create_timer(_STATS_PERIOD_S, self._log_stats)
        self.get_logger().info(
            f"detector up: {get('model')} on {get('device')}, {get('rate_hz')} Hz max, "
            f"{get('image_topic')} -> {get('detections_topic')}"
        )

    def _on_image(self, msg: CompressedImage) -> None:
        if not self._throttle.accept(time.monotonic()):
            return
        started = time.monotonic()
        try:
            detections = self._backend.infer(bytes(msg.data))
        except Exception as exc:  # noqa: BLE001  a bad frame must not kill the node
            self._failures += 1
            self.get_logger().error(f"inference failed ({self._failures} so far): {exc}")
            return
        self._infer_s_total += time.monotonic() - started
        self._infer_count += 1
        self._pub.publish(to_detection_array(msg.header, detections))
        if self._debug_pub is not None:
            try:
                debug = CompressedImage()
                debug.header = msg.header
                debug.format = "jpeg"
                debug.data = self._backend.annotate(bytes(msg.data), detections)
                self._debug_pub.publish(debug)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"debug image failed: {exc}")

    def _log_stats(self) -> None:
        now = time.monotonic()
        window = max(now - self._last_stats_s, 1e-6)
        mean_ms = 1e3 * self._infer_s_total / self._infer_count if self._infer_count else 0.0
        self.get_logger().info(
            f"{self._infer_count / window:.1f} frames/s processed (mean inference "
            f"{mean_ms:.0f} ms), {self._throttle.dropped} dropped, {self._failures} failures"
        )
        self._infer_s_total, self._infer_count, self._last_stats_s = 0.0, 0, now
        self._throttle.dropped = 0


def main(args: list | None = None) -> None:
    rclpy.init(args=args)
    node: DetectorNode | None = None
    code = 0
    try:
        node = DetectorNode()
        rclpy.spin(node)
    except DetectorUnavailable as exc:
        get_logger("detector_node").error(str(exc))
        code = EXIT_DETECTOR_UNAVAILABLE
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
