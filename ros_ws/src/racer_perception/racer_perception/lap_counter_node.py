"""lap_counter_node: count laps by recognising the starting view again (camera only, no markers).

    ros2 run racer_perception lap_counter_node
    ros2 launch racer_perception lap_counter.launch.py        (preferred)
    ros2 launch racer_bringup cameras.launch.py lap_counter:=true

How it works is in racer_perception/lap_counter.py (cosine similarity of small mean-removed
gray thumbnails against the frame captured when the run started) and docs/notes/lap-counter-
2026-11-12.md (every parameter, and how to tune them from a bag with tools/lap_replay.py).

Subscribes
  image_topic     sensor_msgs/CompressedImage  default /camera/usb/image_raw/compressed
  odom_topic      nav_msgs/Odometry            default /odom/wheel   speed and along-track path length
  yaw_rate_topic  nav_msgs/Odometry            default /odom         twist.angular.z, diagnostic only
Publishes
  /lap/event   std_msgs/String  JSON, one per counted lap: lap, lap_time_s, best_lap_s,
                                peak_similarity, crossing_time_s, distance_m, heading_turns
  /lap/status  std_msgs/String  JSON at status_period_s (1 s): state, similarity, laps, last_lap_s,
                                best_lap_s, armed, speed_mps, distance_m, heading_turns, odom_seen
Serves
  /lap_counter/reset  std_srvs/Trigger  forget the reference and the count; the next frame
                      becomes the new reference (call it with the car at the start line).

Frames are throttled to `rate_hz` (the rest are dropped, newest wins) and decoded at 1/4 scale,
so the per-frame cost is a reduced JPEG decode plus a few array operations on 64 x 48 pixels. All
times are this node's ROS clock (use_sim_time works), not image stamps, so a camera restart with
a jumped stamp cannot confuse the lap times.

Nothing here publishes to /drive or /drive_raw, and nothing reads this node's output on the
command path: it is a measurement, recorded in the bag (car_teleop.launch.py's recorder regex).

If OpenCV (cv2) is missing the node logs ONE error and exits with code 3.

QoS: the image and odometry subscriptions are best_effort (match reliable and best_effort
publishers alike), images KeepLast(1) so a slow frame skips rather than queues; outputs are
reliable, KeepLast(10).
"""

from __future__ import annotations

import json
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from racer_tools.spin_guard import spin_until_shutdown
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger

from racer_perception.lap_counter import LapConfig, LapCounter
from racer_perception.lap_frames import REDUCTIONS, FrameDecodeError, cv2_available, decode_gray
from racer_perception.throttle import FrameThrottle

#: Exit code when OpenCV is missing (the detector's is 3 too: "a needed library is absent").
EXIT_UNAVAILABLE = 3
_STATS_PERIOD_S = 10.0

_DEFAULTS = LapConfig()


class LapCounterUnavailable(RuntimeError):
    pass


def _json(payload: dict) -> str:
    return json.dumps(payload, separators=(",", ":"), allow_nan=False)


class LapCounterNode(Node):
    def __init__(self, **node_kwargs) -> None:
        super().__init__("lap_counter_node", **node_kwargs)
        if not cv2_available():
            raise LapCounterUnavailable(
                "lap_counter_node needs OpenCV (python3-opencv / cv2) to decode the JPEG frames"
            )

        def declare(name, default, description, **kw):
            self.declare_parameter(
                name, default, ParameterDescriptor(description=description, **kw)
            )

        unit = [FloatingPointRange(from_value=0.0, to_value=1.0, step=0.0)]
        declare("image_topic", "/camera/usb/image_raw/compressed", "CompressedImage input.")
        declare(
            "odom_topic", "/odom/wheel", "Odometry: twist.linear.x speed, pose.position.x path."
        )
        declare("yaw_rate_topic", "/odom", "Odometry whose twist.angular.z feeds heading_turns.")
        declare("event_topic", "/lap/event", "JSON String, one per counted lap.")
        declare("status_topic", "/lap/status", "JSON String, every status_period_s.")
        declare("status_period_s", 1.0, "Seconds between /lap/status messages.")
        declare("rate_hz", 15.0, "Maximum frames processed per second. <= 0: no limit.")
        declare(
            "decode_reduction",
            4,
            f"JPEG decode scale 1/N, N in {list(REDUCTIONS)}.",
            integer_range=[IntegerRange(from_value=1, to_value=8, step=0)],
        )
        declare(
            "similarity_threshold",
            _DEFAULTS.similarity_threshold,
            "Count a lap at a similarity peak at or above this (cosine similarity, 0..1).",
            floating_point_range=unit,
        )
        declare(
            "rearm_threshold",
            _DEFAULTS.rearm_threshold,
            "After a lap (or the start) similarity must first fall below this.",
            floating_point_range=unit,
        )
        declare("min_lap_time_s", _DEFAULTS.min_lap_time_s, "Shortest possible lap, seconds.")
        declare(
            "min_lap_distance_m",
            _DEFAULTS.min_lap_distance_m,
            "Shortest possible lap, metres of odometry path length.",
        )
        declare(
            "start_speed_mps",
            _DEFAULTS.start_speed_mps,
            "Capture the reference at the first frame at or above this speed. <= 0: first frame.",
        )
        declare("thumb_width_px", _DEFAULTS.thumb_width_px, "Thumbnail width, pixels.")
        declare("thumb_height_px", _DEFAULTS.thumb_height_px, "Thumbnail height, pixels.")
        declare("blur", _DEFAULTS.blur, "3x3 blur of the thumbnail before comparing.")
        declare(
            "peak_window_max_s",
            _DEFAULTS.peak_window_max_s,
            "A bright window still open after this long is closed at its peak.",
        )
        declare(
            "min_reference_std",
            _DEFAULTS.min_reference_std,
            "Refuse a reference frame whose pixel std (0..255) is below this (blank frame).",
        )

        def get(name):
            return self.get_parameter(name).value

        cfg = LapConfig(
            similarity_threshold=float(get("similarity_threshold")),
            rearm_threshold=float(get("rearm_threshold")),
            min_lap_time_s=float(get("min_lap_time_s")),
            min_lap_distance_m=float(get("min_lap_distance_m")),
            start_speed_mps=float(get("start_speed_mps")),
            thumb_width_px=int(get("thumb_width_px")),
            thumb_height_px=int(get("thumb_height_px")),
            blur=bool(get("blur")),
            peak_window_max_s=float(get("peak_window_max_s")),
            min_reference_std=float(get("min_reference_std")),
        )
        cfg.validate()  # bad thresholds are a hard refusal, not a warning
        self._reduction = int(get("decode_reduction"))
        if self._reduction not in REDUCTIONS:
            raise ValueError(f"decode_reduction must be one of {list(REDUCTIONS)}")
        self._counter = LapCounter(cfg)
        self._throttle = FrameThrottle(float(get("rate_hz")))

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        out_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE
        )
        self._event_pub = self.create_publisher(String, str(get("event_topic")), out_qos)
        self._status_pub = self.create_publisher(String, str(get("status_topic")), out_qos)
        self.create_subscription(
            CompressedImage, str(get("image_topic")), self._on_image, sensor_qos
        )
        self.create_subscription(Odometry, str(get("odom_topic")), self._on_odom, odom_qos)
        if str(get("yaw_rate_topic")):
            self.create_subscription(
                Odometry, str(get("yaw_rate_topic")), self._on_yaw_odom, odom_qos
            )
        self.create_service(Trigger, "/lap_counter/reset", self._on_reset)
        self.create_timer(max(float(get("status_period_s")), 0.05), self._publish_status)

        self._process_s_total = 0.0
        self._process_count = 0
        self._failures = 0
        self._last_stats_s = time.monotonic()
        self.create_timer(_STATS_PERIOD_S, self._log_stats)
        self.get_logger().info(
            f"lap counter up: {get('image_topic')} at {get('rate_hz')} Hz max, similarity >= "
            f"{cfg.similarity_threshold} (rearm < {cfg.rearm_threshold}), lap >= "
            f"{cfg.min_lap_time_s} s and {cfg.min_lap_distance_m} m; reference at the first "
            f"frame at {cfg.start_speed_mps} m/s or more, or on /lap_counter/reset"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg: Odometry) -> None:
        self._counter.update_odom(
            self._now_s(), float(msg.twist.twist.linear.x), float(msg.pose.pose.position.x)
        )

    def _on_yaw_odom(self, msg: Odometry) -> None:
        self._counter.update_yaw_rate(self._now_s(), float(msg.twist.twist.angular.z))

    def _on_image(self, msg: CompressedImage) -> None:
        now = self._now_s()
        if not self._throttle.accept(now):
            return
        started = time.monotonic()
        before = self._counter.state
        try:
            gray = decode_gray(msg.data, self._reduction)
            event = self._counter.update_frame(now, gray)
        except (FrameDecodeError, ValueError) as exc:  # a bad frame must not kill the node
            self._failures += 1
            self.get_logger().error(f"frame skipped ({self._failures} so far): {exc}")
            return
        self._process_s_total += time.monotonic() - started
        self._process_count += 1
        if before != self._counter.state:
            self.get_logger().info("reference captured: counting laps from here")
        if event is not None:
            self._event_pub.publish(String(data=_json(event.to_dict())))
            self.get_logger().info(
                f"lap {event.lap}: {event.lap_time_s:.2f} s (best {event.best_lap_s:.2f} s), "
                f"similarity {event.peak_similarity:.3f}, {event.distance_m:.2f} m"
            )

    def _on_reset(self, _request, response):
        self._counter.reset(capture_next_frame=True)
        response.success = True
        response.message = "reset: the next frame becomes the reference, laps back to 0"
        self.get_logger().info(response.message)
        return response

    def _publish_status(self) -> None:
        self._status_pub.publish(String(data=_json(self._counter.status())))

    def _log_stats(self) -> None:
        window = max(time.monotonic() - self._last_stats_s, 1e-6)
        mean_ms = 1e3 * self._process_s_total / self._process_count if self._process_count else 0.0
        status = self._counter.status()
        sim = status["similarity"]
        self.get_logger().info(
            f"{self._process_count / window:.1f} frames/s processed ({mean_ms:.1f} ms each), "
            f"{self._throttle.dropped} dropped, {self._failures} failures; state "
            f"{status['state']}, laps {status['laps']}, similarity "
            f"{'--' if sim is None else f'{sim:.3f}'}"
        )
        self._process_s_total, self._process_count = 0.0, 0
        self._last_stats_s = time.monotonic()
        self._throttle.dropped = 0


def main(args: list | None = None) -> None:
    rclpy.init(args=args)
    node: LapCounterNode | None = None
    code = 0
    try:
        node = LapCounterNode()
        spin_until_shutdown(node)
    except LapCounterUnavailable as exc:
        get_logger("lap_counter_node").error(str(exc))
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
