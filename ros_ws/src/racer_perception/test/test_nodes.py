"""L3 node tests for racer_perception: detector_node (fake backend) and fusion_node.

Builds each node in-process in its own DDS domain (no cross-talk between cases or with other
packages' tests) and drives it from a helper node on a mocked topic, the same shape as
racer_tools' node tests. The detector runs with a FAKE backend: the real one needs ultralytics,
torch and weights (not in CI), and what is under test here is the plumbing: rate limiting, the
Detection2DArray content, the empty-array contract, QoS, and fusion's bearing / range / flags /
target hold-over on the wire. The maths are covered without ROS by test_fusion.py.

pytest.importorskip-guarded: skipped under the bare `uv run pytest` L1 run, runs for real under
`colcon test` in the ros-dev image.
"""

from __future__ import annotations

import math
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("vision_msgs")
pytest.importorskip("racer_msgs")

import rclpy
from racer_msgs.msg import ObjectDetection, ObjectDetectionArray
from racer_perception.detector_node import DetectorNode
from racer_perception.fusion import RawDetection
from racer_perception.fusion_node import FusionNode
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, LaserScan
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

DEG = math.pi / 180.0
_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
)
_BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
)


class Harness:
    """One DDS domain, one executor, the node under test and a helper node."""

    def __init__(self, domain_id: int, make_node):
        self.context = Context()
        rclpy.init(context=self.context, domain_id=domain_id)
        try:
            self.node = make_node(self.context)
        except BaseException:
            rclpy.shutdown(context=self.context)
            raise
        self.helper = rclpy.create_node("perception_test_helper", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.executor.add_node(self.helper)

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.02)

    def spin_until(self, predicate, timeout_s: float = 10.0) -> bool:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return True
        return False

    def close(self) -> None:
        self.executor.shutdown()
        self.node.destroy_node()
        self.helper.destroy_node()
        rclpy.shutdown(context=self.context)


@pytest.fixture
def make_harness():
    made: list[Harness] = []

    def factory(domain_id, make_node):
        h = Harness(domain_id, make_node)
        made.append(h)
        return h

    yield factory
    for h in made:
        h.close()


# ------------------------------------------------------------------ detector


class FakeBackend:
    def __init__(self, detections=(), fail=False):
        self.detections = list(detections)
        self.fail = fail
        self.calls = 0
        self.annotated = 0

    def infer(self, jpeg):
        self.calls += 1
        if self.fail:
            raise RuntimeError("boom")
        return list(self.detections)

    def annotate(self, jpeg, detections):
        self.annotated += 1
        return b"debug-jpeg"


def _image(index: int) -> CompressedImage:
    msg = CompressedImage()
    msg.header.stamp.sec = 100 + index
    msg.header.frame_id = "camera_usb"
    msg.format = "jpeg"
    msg.data = b"not-really-a-jpeg"
    return msg


def _detector(backend, **params):
    def make(context):
        overrides = [Parameter(k, value=v) for k, v in params.items()]
        return DetectorNode(backend=backend, context=context, parameter_overrides=overrides)

    return make


def _collect(h, msg_type, topic, qos=_RELIABLE):
    got: list = []
    h.helper.create_subscription(msg_type, topic, got.append, qos)
    return got


def test_detector_publishes_class_score_and_box_with_the_image_header(make_harness):
    backend = FakeBackend([RawDetection("person", 0.87, 640.0, 360.0, 100.0, 300.0)])
    h = make_harness(101, _detector(backend))
    got = _collect(h, Detection2DArray, "/perception/detections")
    pub = h.helper.create_publisher(CompressedImage, "/camera/usb/image_raw/compressed", _RELIABLE)
    assert h.spin_until(lambda: pub.get_subscription_count() > 0)
    pub.publish(_image(0))
    assert h.spin_until(lambda: len(got) >= 1)
    msg = got[0]
    assert msg.header.frame_id == "camera_usb" and msg.header.stamp.sec == 100
    assert len(msg.detections) == 1
    d = msg.detections[0]
    assert d.results[0].hypothesis.class_id == "person"
    assert d.results[0].hypothesis.score == pytest.approx(0.87)
    assert (d.bbox.center.position.x, d.bbox.center.position.y) == (640.0, 360.0)
    assert (d.bbox.size_x, d.bbox.size_y) == (100.0, 300.0)


def test_detector_publishes_an_empty_array_when_nothing_is_found(make_harness):
    h = make_harness(102, _detector(FakeBackend([])))
    got = _collect(h, Detection2DArray, "/perception/detections")
    pub = h.helper.create_publisher(CompressedImage, "/camera/usb/image_raw/compressed", _RELIABLE)
    assert h.spin_until(lambda: pub.get_subscription_count() > 0)
    pub.publish(_image(0))
    assert h.spin_until(lambda: len(got) >= 1)
    assert got[0].detections == []


def test_detector_drops_frames_above_rate_hz(make_harness):
    backend = FakeBackend([])
    h = make_harness(103, _detector(backend, rate_hz=2.0))
    got = _collect(h, Detection2DArray, "/perception/detections")
    pub = h.helper.create_publisher(CompressedImage, "/camera/usb/image_raw/compressed", _RELIABLE)
    assert h.spin_until(lambda: pub.get_subscription_count() > 0)
    start = time.monotonic()
    index = 0
    while time.monotonic() - start < 1.2:  # about 60 frames in 1.2 s
        pub.publish(_image(index))
        index += 1
        h.spin_for(0.02)
    h.spin_for(0.3)
    assert index >= 30
    assert 2 <= len(got) <= 4, f"{len(got)} of {index} frames processed at 2 Hz"
    assert backend.calls == len(got)


def test_detector_survives_a_failing_backend(make_harness):
    backend = FakeBackend(fail=True)
    h = make_harness(104, _detector(backend))
    got = _collect(h, Detection2DArray, "/perception/detections")
    pub = h.helper.create_publisher(CompressedImage, "/camera/usb/image_raw/compressed", _RELIABLE)
    assert h.spin_until(lambda: pub.get_subscription_count() > 0)
    pub.publish(_image(0))
    h.spin_for(0.5)
    assert backend.calls == 1 and got == []
    backend.fail = False
    h.spin_for(0.6)  # rate_hz default 10 Hz: the next frame is accepted
    pub.publish(_image(1))
    assert h.spin_until(lambda: len(got) == 1)


def test_debug_image_is_off_by_default_and_on_when_asked(make_harness):
    off_backend = FakeBackend([])
    h = make_harness(105, _detector(off_backend))
    assert "/perception/debug_image/compressed" not in dict(h.node.get_topic_names_and_types())

    on_backend = FakeBackend([RawDetection("person", 0.9, 10.0, 10.0, 5.0, 5.0)])
    h2 = make_harness(106, _detector(on_backend, publish_debug_image=True))
    got = _collect(h2, CompressedImage, "/perception/debug_image/compressed", _BEST_EFFORT)
    pub = h2.helper.create_publisher(CompressedImage, "/camera/usb/image_raw/compressed", _RELIABLE)
    assert h2.spin_until(lambda: pub.get_subscription_count() > 0)
    pub.publish(_image(0))
    assert h2.spin_until(lambda: len(got) >= 1)
    assert bytes(got[0].data) == b"debug-jpeg" and got[0].format == "jpeg"
    assert off_backend.annotated == 0


def test_detector_image_subscription_matches_a_reliable_camera(make_harness):
    # usb_cam publishes reliable; the detector's best_effort subscription must still connect.
    h = make_harness(107, _detector(FakeBackend([])))
    pub = h.helper.create_publisher(CompressedImage, "/camera/usb/image_raw/compressed", _RELIABLE)
    assert h.spin_until(lambda: pub.get_subscription_count() > 0)


# -------------------------------------------------------------------- fusion


def _fusion(**params):
    def make(context):
        overrides = [Parameter(k, value=v) for k, v in params.items()]
        return FusionNode(context=context, parameter_overrides=overrides)

    return make


def _scan(stamp_s: float, near_vehicle_bearing_deg: float, range_m: float, width_deg=10):
    """1 degree beams in the laser frame, the real C1's yaw of pi, background 6 m."""
    msg = LaserScan()
    msg.header.stamp.sec = int(stamp_s)
    msg.header.stamp.nanosec = round((stamp_s - int(stamp_s)) * 1e9)
    msg.header.frame_id = "laser"
    msg.angle_min = -math.pi
    msg.angle_max = math.pi - DEG
    msg.angle_increment = DEG
    msg.range_min = 0.05
    msg.range_max = 12.0
    ranges = [6.0] * 360
    for k in range(-width_deg, width_deg + 1):
        vehicle_deg = near_vehicle_bearing_deg + k
        laser_deg = vehicle_deg - 180.0  # vehicle bearing = laser bearing + pi
        ranges[round(laser_deg + 180.0) % 360] = range_m
    msg.ranges = ranges
    return msg


def _detections(stamp_s: float, boxes):
    msg = Detection2DArray()
    msg.header.stamp.sec = int(stamp_s)
    msg.header.stamp.nanosec = round((stamp_s - int(stamp_s)) * 1e9)
    msg.header.frame_id = "camera_usb"
    for name, score, cx, width in boxes:
        det = Detection2D()
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id, hyp.hypothesis.score = name, score
        det.results.append(hyp)
        det.bbox.center.position.x, det.bbox.center.position.y = cx, 360.0
        det.bbox.size_x, det.bbox.size_y = width, 300.0
        msg.detections.append(det)
    return msg


def _fusion_io(h):
    objects = _collect(h, ObjectDetectionArray, "/perception/objects")
    targets = _collect(h, ObjectDetection, "/perception/target")
    scan_pub = h.helper.create_publisher(LaserScan, "/scan", _BEST_EFFORT)
    det_pub = h.helper.create_publisher(Detection2DArray, "/perception/detections", _RELIABLE)
    assert h.spin_until(
        lambda: scan_pub.get_subscription_count() > 0 and det_pub.get_subscription_count() > 0
    )
    return objects, targets, scan_pub, det_pub


def test_fusion_uses_the_vehicle_params_yaw_and_reports_bearing_and_range(make_harness):
    h = make_harness(110, _fusion())
    objects, targets, scan_pub, det_pub = _fusion_io(h)
    scan_pub.publish(_scan(100.0, near_vehicle_bearing_deg=0.0, range_m=2.0))
    h.spin_for(0.3)
    det_pub.publish(_detections(100.05, [("person", 0.9, 640.0, 200.0)]))
    assert h.spin_until(lambda: len(objects) >= 1 and len(targets) >= 1)
    obj = objects[0].detections[0]
    assert obj.header.frame_id == "base_link"
    assert obj.class_name == "person"
    assert obj.bearing_rad == pytest.approx(0.0, abs=1e-6)
    assert obj.range_valid and obj.range_status == "ok"
    assert obj.range_m == pytest.approx(2.0)
    assert obj.range_samples > 0
    assert (obj.box_cx_px, obj.box_width_px) == (640.0, 200.0)
    assert targets[0].range_m == pytest.approx(2.0)


def test_fusion_flags_no_scan_and_still_publishes_the_bearing(make_harness):
    h = make_harness(111, _fusion())
    objects, _targets, _scan_pub, det_pub = _fusion_io(h)
    det_pub.publish(_detections(100.0, [("person", 0.9, 200.0, 100.0)]))
    assert h.spin_until(lambda: len(objects) >= 1)
    obj = objects[0].detections[0]
    assert obj.range_status == "no_scan" and not obj.range_valid
    assert math.isnan(obj.range_m)
    assert obj.bearing_rad > 0.0  # left of centre


def test_fusion_flags_a_stale_scan(make_harness):
    h = make_harness(112, _fusion())
    objects, _targets, scan_pub, det_pub = _fusion_io(h)
    scan_pub.publish(_scan(100.0, 0.0, 2.0))
    h.spin_for(0.3)
    det_pub.publish(_detections(105.0, [("person", 0.9, 640.0, 200.0)]))
    assert h.spin_until(lambda: len(objects) >= 1)
    assert objects[0].detections[0].range_status == "stale_scan"


def test_fusion_publishes_an_empty_array_for_an_empty_detection_array(make_harness):
    h = make_harness(113, _fusion())
    objects, targets, _scan_pub, det_pub = _fusion_io(h)
    det_pub.publish(_detections(100.0, []))
    assert h.spin_until(lambda: len(objects) >= 1)
    assert objects[0].detections == [] and targets == []


def test_fusion_target_is_the_nearest_person_and_is_held_then_dropped(make_harness):
    h = make_harness(114, _fusion(target_hold_s=0.6))
    _objects, targets, scan_pub, det_pub = _fusion_io(h)
    scan_pub.publish(_scan(100.0, 0.0, 1.5))
    h.spin_for(0.3)
    boxes = [("chair", 0.9, 640.0, 200.0), ("person", 0.8, 640.0, 200.0)]
    det_pub.publish(_detections(100.0, boxes))
    assert h.spin_until(lambda: len(targets) >= 1)
    assert targets[-1].class_name == "person"

    det_pub.publish(_detections(100.1, [("chair", 0.9, 640.0, 200.0)]))
    assert h.spin_until(lambda: len(targets) >= 2)  # held over
    assert targets[-1].header.stamp.sec == 100 and targets[-1].header.stamp.nanosec == 0

    h.spin_for(0.8)  # past target_hold_s
    count = len(targets)
    det_pub.publish(_detections(100.2, [("chair", 0.9, 640.0, 200.0)]))
    h.spin_for(0.5)
    assert len(targets) == count, "no target must be published once the hold-over expired"


def test_fusion_refuses_a_laser_yaw_that_disagrees_with_vehicle_params(make_harness):
    with pytest.raises(ValueError, match="mount_yaw_rad"):
        make_harness(115, _fusion(laser_yaw_rad=1.0))


def test_fusion_simulator_mode_ignores_the_binding(make_harness):
    h = make_harness(116, _fusion(yaw_from_vehicle_params=False))
    assert h.node._laser_yaw == 0.0
