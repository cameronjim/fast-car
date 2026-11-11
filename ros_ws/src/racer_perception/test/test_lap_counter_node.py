"""L3 tests for lap_counter_node.

The node runs in-process in its own DDS domain and is driven from a helper node on mocked
topics: JPEG frames (made here with OpenCV from synthetic textures), wheel odometry and the
EKF odometry for yaw. What is under test is the plumbing; the lap logic itself is covered
without ROS by test_lap_counter.py.

Skipped under the bare `uv run pytest` L1 run (no rclpy / cv2); runs for real under `colcon test`
in the ros-dev image.
"""

from __future__ import annotations

import json
import time

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
pytest.importorskip("rclpy")
pytest.importorskip("nav_msgs")
pytest.importorskip("std_srvs")

import rclpy
from nav_msgs.msg import Odometry
from racer_perception.lap_counter_node import LapCounterNode
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger


def _jpeg(img: np.ndarray, quality: int = 90) -> bytes:
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    assert ok
    return buf.tobytes()


def _texture(seed: int, width: int = 320, height: int = 240) -> np.ndarray:
    rng = np.random.default_rng(seed)
    small = rng.uniform(0, 255, size=(height // 16, width // 16)).astype(np.float32)
    big = cv2.resize(small, (width, height), interpolation=cv2.INTER_CUBIC)
    return np.clip(big, 0, 255).astype(np.uint8)


_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
)
_BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
)
IMAGE_TOPIC = "/camera/usb/image_raw/compressed"


class Harness:
    def __init__(self, domain_id: int, **params):
        self.context = Context()
        rclpy.init(context=self.context, domain_id=domain_id)
        overrides = [Parameter(k, value=v) for k, v in params.items()]
        try:
            self.node = LapCounterNode(context=self.context, parameter_overrides=overrides)
        except BaseException:
            rclpy.shutdown(context=self.context)
            raise
        self.helper = rclpy.create_node("lap_test_helper", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.executor.add_node(self.helper)
        self.events: list[dict] = []
        self.statuses: list[dict] = []
        self.helper.create_subscription(
            String, "/lap/event", lambda m: self.events.append(json.loads(m.data)), _RELIABLE
        )
        self.helper.create_subscription(
            String, "/lap/status", lambda m: self.statuses.append(json.loads(m.data)), _RELIABLE
        )
        self.image_pub = self.helper.create_publisher(CompressedImage, IMAGE_TOPIC, _BEST_EFFORT)
        self.odom_pub = self.helper.create_publisher(Odometry, "/odom/wheel", _BEST_EFFORT)
        self.yaw_pub = self.helper.create_publisher(Odometry, "/odom", _BEST_EFFORT)

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)

    def spin_until(self, predicate, timeout_s: float = 10.0) -> bool:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)
            if predicate():
                return True
        return False

    def connected(self) -> bool:
        return self.spin_until(
            lambda: (
                self.image_pub.get_subscription_count() > 0
                and self.odom_pub.get_subscription_count() > 0
            )
        )

    def odom(self, speed: float, x: float) -> None:
        msg = Odometry()
        msg.twist.twist.linear.x = speed
        msg.pose.pose.position.x = x
        self.odom_pub.publish(msg)

    def frame(self, img: np.ndarray, settle_s: float = 0.04) -> None:
        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = _jpeg(img)
        self.image_pub.publish(msg)
        self.spin_for(settle_s)

    def close(self) -> None:
        self.executor.shutdown()
        self.node.destroy_node()
        self.helper.destroy_node()
        rclpy.shutdown(context=self.context)


@pytest.fixture
def make_harness():
    made: list[Harness] = []

    def factory(domain_id, **params):
        h = Harness(domain_id, **params)
        made.append(h)
        return h

    yield factory
    for h in made:
        h.close()


#: Gates off, so a lap is just "the reference view came back"; real gates are unit tested.
_NO_GATES = {
    "rate_hz": 0.0,
    "min_lap_time_s": 0.0,
    "min_lap_distance_m": 0.0,
    "decode_reduction": 2,
    "status_period_s": 0.1,
}


def test_a_lap_event_is_published_when_the_start_view_comes_back(make_harness):
    h = make_harness(131, **_NO_GATES)
    assert h.connected()
    a, b, c = _texture(10), _texture(11), _texture(12)
    h.odom(1.0, 0.0)
    h.spin_for(0.1)
    h.frame(a)  # the run starts: speed is above start_speed_mps
    for i, img in enumerate((b, c, b, c)):
        h.odom(1.0, 0.2 * (i + 1))
        h.frame(img)
    h.frame(a)  # back at the start line
    h.frame(b)  # window closes
    assert h.spin_until(lambda: len(h.events) >= 1)
    event = h.events[0]
    assert event["lap"] == 1
    assert event["peak_similarity"] > 0.95
    assert event["lap_time_s"] > 0.0 and event["best_lap_s"] == event["lap_time_s"]
    assert event["distance_m"] == pytest.approx(0.8, abs=0.05)
    assert event["heading_turns"] is None  # no yaw source published
    assert len(h.events) == 1


def test_heading_turns_come_from_the_yaw_rate_topic(make_harness):
    h = make_harness(132, **_NO_GATES)
    assert h.connected()
    h.odom(1.0, 0.0)
    h.spin_for(0.1)
    h.frame(_texture(10))
    for i in range(4):
        odom = Odometry()
        odom.twist.twist.angular.z = 3.0
        h.yaw_pub.publish(odom)
        h.frame(_texture(20 + i), settle_s=0.1)
    h.frame(_texture(10))
    h.frame(_texture(21))
    assert h.spin_until(lambda: len(h.events) >= 1)
    turns = h.events[0]["heading_turns"]
    assert turns is not None and turns > 0.0


def test_status_is_published_while_waiting_and_before_any_frame(make_harness):
    h = make_harness(133, **_NO_GATES)
    assert h.spin_until(lambda: len(h.statuses) >= 2)
    status = h.statuses[-1]
    assert status["state"] == "waiting" and status["laps"] == 0
    assert status["similarity"] is None and status["odom_seen"] is False


def test_no_reference_while_the_car_stands_still(make_harness):
    h = make_harness(134, **_NO_GATES)
    assert h.connected()
    h.odom(0.0, 0.0)
    h.spin_for(0.1)
    for i in range(3):
        h.frame(_texture(i))
    assert h.spin_until(lambda: len(h.statuses) >= 1)
    assert h.statuses[-1]["state"] == "waiting"


def test_reset_service_clears_the_count_and_takes_the_next_frame(make_harness):
    h = make_harness(135, **_NO_GATES)
    assert h.connected()
    h.odom(0.0, 0.0)  # standing still: only reset can start the run
    h.spin_for(0.1)
    client = h.helper.create_client(Trigger, "/lap_counter/reset")
    assert client.wait_for_service(timeout_sec=5.0)
    future = client.call_async(Trigger.Request())
    assert h.spin_until(future.done)
    assert future.result().success
    h.frame(_texture(30))
    n = len(h.statuses)
    assert h.spin_until(lambda: len(h.statuses) > n + 1)
    assert h.statuses[-1]["state"] == "running"


def test_a_corrupt_frame_is_skipped_and_the_node_keeps_working(make_harness):
    h = make_harness(136, **_NO_GATES)
    assert h.connected()
    h.odom(1.0, 0.0)
    h.spin_for(0.1)
    junk = CompressedImage()
    junk.format = "jpeg"
    junk.data = b"not-really-a-jpeg"
    h.image_pub.publish(junk)
    h.spin_for(0.1)
    h.frame(_texture(40))
    assert h.spin_until(lambda: h.statuses and h.statuses[-1]["state"] == "running")


def test_frames_above_rate_hz_are_dropped(make_harness):
    h = make_harness(137, **{**_NO_GATES, "rate_hz": 2.0})
    assert h.connected()
    h.odom(1.0, 0.0)
    h.spin_for(0.1)
    # With the throttle at 2 Hz only the first frame of a burst is processed, and the first
    # frame is the reference; a different frame right after must not have been looked at.
    h.frame(_texture(50), settle_s=0.0)
    h.frame(_texture(51), settle_s=0.0)
    h.spin_for(0.3)
    sims = [s["similarity"] for s in h.statuses if s["state"] == "running"]
    assert sims and all(s == 1.0 for s in sims)  # still only the reference was seen


def test_the_node_only_publishes_the_lap_topics(make_harness):
    h = make_harness(138, **_NO_GATES)
    h.spin_for(0.3)
    published = {
        name
        for name, _types in h.helper.get_publisher_names_and_types_by_node("lap_counter_node", "/")
    }
    assert {"/lap/event", "/lap/status"} <= published
    assert "/drive" not in published and "/drive_raw" not in published


def test_bad_thresholds_are_a_hard_refusal(make_harness):
    with pytest.raises(ValueError):
        make_harness(139, similarity_threshold=0.5, rearm_threshold=0.9)
