"""camera_record_node: the camera stream that goes into the bag, small and only while driving.

    ros2 launch racer_bringup cameras.launch.py          (starts one per camera)
    ros2 run racer_perception camera_record_node --ros-args \
        -p image_topic:=/camera/usb/image_raw/compressed -p output_topic:=/camera/usb/record/compressed

Why: the full-rate compressed streams are about 6 MB/s per camera, and the run's recorder used to
take them for as long as the stack was up. 25 idle minutes with both cameras on made a 9 GB bag.
car_teleop.launch.py's recorder now takes /camera/<name>/record/compressed instead of
image_raw/compressed, and this node is what makes that topic:

  * Rate and size: at most `rate_hz` (default 5) frames per second, each shrunk to fit inside
    `max_width_px` x `max_height_px` (default 640 x 360, never upscaled) at `jpeg_quality`
    (default 60). The decode is libjpeg's reduced-size decode, so a 1280 x 720 frame costs a
    fraction of a full decode.
  * Only while driving: frames are published only while a /drive_raw message has been seen in
    the last `gate_hold_s` seconds (default 5), i.e. while a driving launch (gap follow, wall
    follow, tracker, park, policy, pnp, teleop) is up. With `gate_topic` empty the gate is
    always open (bench and debugging).
  * Idle costs nothing: the input subscription exists only while the gate is open. With no
    subscriber on the full-rate topic, image_transport's compressed plugin does not even
    encode, so an idle stack runs no camera JPEG encoding for this node.

The full-rate topics are untouched, so Foxglove, the racer app thumbnails, lap_counter_node and
the detector keep working exactly as before. Everything non-camera keeps being recorded by the
main recorder all the time; this node only decides about the camera frames.

Subscribes
  image_topic  sensor_msgs/CompressedImage              best_effort, KeepLast(1), gate open only
  gate_topic   ackermann_msgs/AckermannDriveStamped     default /drive_raw, read only
Publishes
  output_topic sensor_msgs/CompressedImage              reliable KeepLast(5), same header, JPEG

Nothing here publishes to /drive or /drive_raw: the gate topic is only ever read.

If OpenCV (cv2) is missing the node logs ONE error and exits with code 3, like lap_counter_node.
"""

from __future__ import annotations

import sys
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

from racer_perception.camera_record import ActivityGate, shrink_jpeg
from racer_perception.lap_frames import FrameDecodeError, cv2_available
from racer_perception.throttle import FrameThrottle

#: Exit code when OpenCV is missing (same as lap_counter_node and the detector).
EXIT_UNAVAILABLE = 3
_STATS_PERIOD_S = 30.0
_GATE_CHECK_PERIOD_S = 0.5


class CameraRecordUnavailable(RuntimeError):
    pass


class CameraRecordNode(Node):
    def __init__(self, **node_kwargs) -> None:
        super().__init__("camera_record_node", **node_kwargs)
        if not cv2_available():
            raise CameraRecordUnavailable(
                "camera_record_node needs OpenCV (python3-opencv / cv2) to shrink the JPEG frames"
            )

        def declare(name, default, description, **kw):
            self.declare_parameter(
                name, default, ParameterDescriptor(description=description, **kw)
            )

        declare("image_topic", "/camera/usb/image_raw/compressed", "Full-rate CompressedImage in.")
        declare("output_topic", "/camera/usb/record/compressed", "Recording-only stream out.")
        declare("rate_hz", 5.0, "Maximum frames per second recorded. <= 0: every frame.")
        declare(
            "max_width_px",
            640,
            "Frames are fitted inside this width, never upscaled.",
            integer_range=[IntegerRange(from_value=16, to_value=7680, step=0)],
        )
        declare(
            "max_height_px",
            360,
            "Frames are fitted inside this height, never upscaled.",
            integer_range=[IntegerRange(from_value=16, to_value=4320, step=0)],
        )
        declare(
            "jpeg_quality",
            60,
            "JPEG quality (1-100) of the recorded frames.",
            integer_range=[IntegerRange(from_value=1, to_value=100, step=0)],
        )
        declare(
            "gate_topic",
            "/drive_raw",
            "Record only while this AckermannDriveStamped topic is being published. '' = always.",
        )
        declare(
            "gate_hold_s",
            5.0,
            "Keep recording this long after the last gate message.",
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=3600.0, step=0.0)],
        )

        def get(name):
            return self.get_parameter(name).value

        self._image_topic = str(get("image_topic"))
        self._max_w = int(get("max_width_px"))
        self._max_h = int(get("max_height_px"))
        self._quality = int(get("jpeg_quality"))
        if not 1 <= self._quality <= 100:
            raise ValueError(f"jpeg_quality must be 1..100, got {self._quality}")
        if self._max_w <= 0 or self._max_h <= 0:
            raise ValueError("max_width_px and max_height_px must be positive")
        self._throttle = FrameThrottle(float(get("rate_hz")))
        self._always_open = str(get("gate_topic")) == ""
        self._gate = ActivityGate(float(get("gate_hold_s")))

        self._image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        out_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.RELIABLE
        )
        self._pub = self.create_publisher(CompressedImage, str(get("output_topic")), out_qos)
        self._image_sub = None
        if self._always_open:
            self._ensure_subscribed()
        else:
            gate_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
            )
            # raw=True: only the arrival matters, so the message is never deserialized.
            self.create_subscription(
                AckermannDriveStamped, str(get("gate_topic")), self._on_gate, gate_qos, raw=True
            )
            self.create_timer(_GATE_CHECK_PERIOD_S, self._check_gate)

        self._published = 0
        self._failures = 0
        self._bytes_in = 0
        self._bytes_out = 0
        self._cpu_s = 0.0
        self._last_stats_s = time.monotonic()
        self.create_timer(_STATS_PERIOD_S, self._log_stats)
        self.get_logger().info(
            f"camera record up: {self._image_topic} -> {get('output_topic')} at {get('rate_hz')} "
            f"Hz max, fit {self._max_w}x{self._max_h}, JPEG {self._quality}, "
            + ("always on" if self._always_open else f"while {get('gate_topic')} is active ")
            + ("" if self._always_open else f"(+{get('gate_hold_s')} s)")
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @property
    def recording(self) -> bool:
        """True while the full-rate input is subscribed (the gate is, or just was, open)."""
        return self._image_sub is not None

    def _ensure_subscribed(self) -> None:
        if self._image_sub is None:
            # raw=True: the callback gets the serialized bytes, and only the frames that pass
            # the throttle are deserialized. At 60 Hz in and 5 Hz out the other 55 frames a
            # second are never turned into Python messages, which is most of this node's CPU.
            self._image_sub = self.create_subscription(
                CompressedImage, self._image_topic, self._on_image, self._image_qos, raw=True
            )
            self.get_logger().info("drive active: recording camera frames")

    def _ensure_unsubscribed(self) -> None:
        if self._image_sub is not None:
            self.destroy_subscription(self._image_sub)
            self._image_sub = None
            self.get_logger().info("no drive: camera frames are not being recorded")

    def _on_gate(self, _serialized: bytes) -> None:
        self._gate.note(self._now_s())
        self._ensure_subscribed()

    def _check_gate(self) -> None:
        if not self._gate.is_open(self._now_s()):
            self._ensure_unsubscribed()

    def _on_image(self, serialized: bytes) -> None:
        now = self._now_s()
        if not self._always_open and not self._gate.is_open(now):
            return
        if not self._throttle.accept(now):
            return
        started = time.process_time()
        try:
            msg = deserialize_message(serialized, CompressedImage)
            data, _size = shrink_jpeg(msg.data, self._max_w, self._max_h, self._quality)
        except (FrameDecodeError, ValueError, RuntimeError) as exc:  # a bad frame must not kill us
            self._failures += 1
            self.get_logger().error(f"frame skipped ({self._failures} so far): {exc}")
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = "jpeg"
        out.data = data
        self._pub.publish(out)
        self._cpu_s += time.process_time() - started
        self._published += 1
        self._bytes_in += len(msg.data)
        self._bytes_out += len(data)

    def _log_stats(self) -> None:
        window = max(time.monotonic() - self._last_stats_s, 1e-6)
        if self._published or self._failures:
            self.get_logger().info(
                f"{self._published / window:.1f} frames/s recorded, "
                f"{self._bytes_out / window / 1e3:.0f} kB/s out (in {self._bytes_in / window / 1e3:.0f}"
                f" kB/s of the frames used), {1e3 * self._cpu_s / max(self._published, 1):.1f} ms "
                f"CPU each, {self._failures} failures"
            )
        self._published = self._failures = self._bytes_in = self._bytes_out = 0
        self._cpu_s = 0.0
        self._last_stats_s = time.monotonic()


def main(args: list | None = None) -> None:
    rclpy.init(args=args)
    node: CameraRecordNode | None = None
    code = 0
    try:
        node = CameraRecordNode()
        rclpy.spin(node)
    except CameraRecordUnavailable as exc:
        get_logger("camera_record_node").error(str(exc))
        code = EXIT_UNAVAILABLE
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
