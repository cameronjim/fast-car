"""L3 tests for camera_record_node.

The node runs in-process in its own DDS domain next to a helper node that publishes mocked
full-rate JPEG frames and /drive_raw, and listens on the recording topic. What is under test: no
frame is recorded while nothing drives, frames flow (shrunk and throttled) while /drive_raw is
active and for gate_hold_s after, the full-rate subscription only exists while the gate is open,
and a bad frame does not kill the node.

Skipped under the bare `uv run pytest` L1 run (no rclpy / cv2 / ackermann_msgs); runs for real
under `colcon test` in the ros-dev image.
"""

from __future__ import annotations

import time

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
pytest.importorskip("rclpy")
pytest.importorskip("ackermann_msgs")

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from racer_perception.camera_record_node import CameraRecordNode
from rclpy.context import Context
from rclpy.exceptions import InvalidParameterValueException
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
)
_BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
)
IN_TOPIC = "/camera/usb/image_raw/compressed"
OUT_TOPIC = "/camera/usb/record/compressed"


def _jpeg(width: int = 1280, height: int = 720, seed: int = 1) -> bytes:
    rng = np.random.default_rng(seed)
    small = rng.uniform(0, 255, size=(height // 16, width // 16, 3)).astype(np.float32)
    img = cv2.resize(small, (width, height), interpolation=cv2.INTER_CUBIC)
    ok, buf = cv2.imencode(".jpg", np.clip(img, 0, 255).astype(np.uint8))
    assert ok
    return buf.tobytes()


class Harness:
    def __init__(self, domain_id: int, **params):
        self.context = Context()
        rclpy.init(context=self.context, domain_id=domain_id)
        overrides = [
            Parameter("image_topic", value=IN_TOPIC),
            Parameter("output_topic", value=OUT_TOPIC),
            *[Parameter(k, value=v) for k, v in params.items()],
        ]
        try:
            self.node = CameraRecordNode(context=self.context, parameter_overrides=overrides)
        except BaseException:
            rclpy.shutdown(context=self.context)
            raise
        self.helper = rclpy.create_node("camera_record_test_helper", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.executor.add_node(self.helper)
        self.recorded: list[CompressedImage] = []
        self.helper.create_subscription(CompressedImage, OUT_TOPIC, self.recorded.append, _RELIABLE)
        self.image_pub = self.helper.create_publisher(CompressedImage, IN_TOPIC, _BEST_EFFORT)
        self.drive_pub = self.helper.create_publisher(
            AckermannDriveStamped, "/drive_raw", _RELIABLE
        )

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

    def drive(self) -> None:
        self.drive_pub.publish(AckermannDriveStamped())

    def frame(self, data: bytes, settle_s: float = 0.05) -> None:
        msg = CompressedImage()
        msg.header.frame_id = "camera_usb"
        msg.header.stamp.sec = 42
        msg.format = "jpeg"
        msg.data = data
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


#: Every frame passes the throttle, so frame counts are exact.
_FAST = {"rate_hz": 0.0, "gate_hold_s": 0.6}


def test_nothing_is_recorded_and_nothing_subscribes_while_idle(make_harness):
    h = make_harness(220, **_FAST)
    h.spin_for(0.5)
    # No /drive_raw: the full-rate topic has no subscriber, so no encoder runs for it either.
    assert h.image_pub.get_subscription_count() == 0
    assert h.node.recording is False
    for _ in range(5):
        h.frame(_jpeg())
    assert h.recorded == []


def test_frames_are_recorded_shrunk_while_drive_raw_is_active(make_harness):
    h = make_harness(221, **_FAST)
    source = _jpeg()
    h.drive()
    assert h.spin_until(lambda: h.image_pub.get_subscription_count() > 0)
    for _ in range(5):
        h.drive()
        h.frame(source)
    assert h.spin_until(lambda: len(h.recorded) >= 1)
    msg = h.recorded[0]
    assert msg.format == "jpeg"
    assert msg.header.frame_id == "camera_usb" and msg.header.stamp.sec == 42
    decoded = cv2.imdecode(np.frombuffer(bytes(msg.data), dtype=np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape[:2] == (360, 640)
    assert len(msg.data) < len(source) / 2


def test_recording_stops_gate_hold_s_after_the_last_drive_raw(make_harness):
    h = make_harness(222, **_FAST)
    h.drive()
    assert h.spin_until(lambda: h.image_pub.get_subscription_count() > 0)
    h.frame(_jpeg())
    assert h.spin_until(lambda: len(h.recorded) >= 1)
    # gate_hold_s is 0.6 s: after it, the subscription is dropped and frames are not recorded.
    assert h.spin_until(lambda: h.image_pub.get_subscription_count() == 0, timeout_s=5.0)
    assert h.node.recording is False
    count = len(h.recorded)
    for _ in range(3):
        h.frame(_jpeg())
    assert len(h.recorded) == count


def test_recording_resumes_on_the_next_drive(make_harness):
    h = make_harness(223, **_FAST)
    h.drive()
    assert h.spin_until(lambda: h.image_pub.get_subscription_count() > 0)
    assert h.spin_until(lambda: h.image_pub.get_subscription_count() == 0, timeout_s=5.0)
    h.drive()
    assert h.spin_until(lambda: h.image_pub.get_subscription_count() > 0)
    h.frame(_jpeg())
    assert h.spin_until(lambda: len(h.recorded) >= 1)


def test_the_rate_is_throttled(make_harness):
    h = make_harness(224, rate_hz=5.0, gate_hold_s=30.0)
    h.drive()
    assert h.spin_until(lambda: h.image_pub.get_subscription_count() > 0)
    source = _jpeg(320, 180)
    started = time.monotonic()
    while time.monotonic() - started < 2.0:  # about 100 Hz in
        h.frame(source, settle_s=0.01)
    h.spin_for(0.3)
    # 5 Hz over about 2 s is about 10 frames, never anywhere near the 100 offered.
    assert 5 <= len(h.recorded) <= 14


def test_an_empty_gate_topic_records_all_the_time(make_harness):
    h = make_harness(225, gate_topic="", rate_hz=0.0)
    assert h.spin_until(lambda: h.image_pub.get_subscription_count() > 0)
    h.frame(_jpeg(320, 180))
    assert h.spin_until(lambda: len(h.recorded) >= 1)


def test_a_bad_frame_is_skipped_and_the_node_keeps_going(make_harness):
    h = make_harness(226, **{**_FAST, "gate_hold_s": 30.0})
    h.drive()
    assert h.spin_until(lambda: h.image_pub.get_subscription_count() > 0)
    h.frame(b"this is not a jpeg")
    h.frame(b"")
    h.frame(_jpeg(320, 180))
    assert h.spin_until(lambda: len(h.recorded) == 1)


def test_the_recording_topic_is_the_only_thing_published(make_harness):
    h = make_harness(227, **_FAST)
    published = {
        name
        for name, _types in h.node.get_publisher_names_and_types_by_node("camera_record_node", "/")
        if not name.startswith(("/rosout", "/parameter_events"))
    }
    assert published == {OUT_TOPIC}


def test_bad_parameters_are_refused(make_harness):
    with pytest.raises(InvalidParameterValueException):
        Harness(228, jpeg_quality=0)
