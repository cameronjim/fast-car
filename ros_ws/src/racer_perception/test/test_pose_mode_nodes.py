"""L3: detector pose mode and fusion's poses input (fake backend, mocked topics).

Same harness as test_nodes.py (own DDS domain per case). The real pose model is covered by
test_yolo_images.py, which needs ultralytics and weights and is skipped elsewhere.
"""

from __future__ import annotations

import importlib.util
import math
import pathlib

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("vision_msgs")
pytest.importorskip("racer_msgs")

from racer_msgs.msg import (
    Keypoint2D,
    ObjectDetectionArray,
    PoseDetection,
    PoseDetectionArray,
)
from racer_perception.fusion import RawDetection
from racer_perception.keypoints import Keypoint
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2DArray

_spec = importlib.util.spec_from_file_location(
    "perception_test_nodes", pathlib.Path(__file__).with_name("test_nodes.py")
)
_nodes = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_nodes)

make_harness = _nodes.make_harness
_collect = _nodes._collect
_detector = _nodes._detector
_fusion = _nodes._fusion
_image = _nodes._image
_scan = _nodes._scan
_RELIABLE = _nodes._RELIABLE
_BEST_EFFORT = _nodes._BEST_EFFORT


class PoseBackend(_nodes.FakeBackend):
    image_size = (640, 480)


def _person_with_keypoints():
    kps = tuple(Keypoint(10.0 * i, 5.0 * i, 0.1 * (i % 10)) for i in range(17))
    return RawDetection("person", 0.9, 320.0, 240.0, 100.0, 300.0, kps)


def _pose_msg(stamp_s, cx, width, width_px=0, with_kp=True):
    msg = PoseDetectionArray()
    msg.header.stamp.sec = int(stamp_s)
    msg.header.stamp.nanosec = round((stamp_s - int(stamp_s)) * 1e9)
    msg.image_width_px = width_px
    det = PoseDetection()
    det.class_name, det.score = "person", 0.9
    det.box_cx_px, det.box_cy_px, det.box_width_px, det.box_height_px = cx, 360.0, width, 300.0
    if with_kp:
        det.keypoint_layout = "coco17"
        for i in range(17):
            det.keypoints.append(Keypoint2D(x_px=cx + i, y_px=300.0 + i, confidence=0.8))
    msg.detections.append(det)
    return msg


def test_pose_mode_publishes_keypoints_and_the_plain_boxes(make_harness):
    backend = PoseBackend([_person_with_keypoints()])
    h = make_harness(201, _detector(backend, mode="pose"))
    poses = _collect(h, PoseDetectionArray, "/perception/poses")
    boxes = _collect(h, Detection2DArray, "/perception/detections")
    pub = h.helper.create_publisher(CompressedImage, "/camera/usb/image_raw/compressed", _RELIABLE)
    assert h.spin_until(lambda: pub.get_subscription_count() > 0)
    pub.publish(_image(0))
    assert h.spin_until(lambda: len(poses) >= 1 and len(boxes) >= 1)
    arr = poses[0]
    assert arr.header.frame_id == "camera_usb" and arr.header.stamp.sec == 100
    assert (arr.image_width_px, arr.image_height_px) == (640, 480)
    (det,) = arr.detections
    assert det.class_name == "person" and det.keypoint_layout == "coco17"
    assert len(det.keypoints) == 17
    assert det.keypoints[4].x_px == pytest.approx(40.0)
    assert det.keypoints[4].confidence == pytest.approx(0.4)
    assert (det.box_cx_px, det.box_height_px) == (320.0, 300.0)
    assert len(boxes[0].detections) == 1  # the box consumers still get their array


def test_pose_mode_publishes_an_empty_array_for_nobody(make_harness):
    h = make_harness(202, _detector(PoseBackend([]), mode="pose"))
    poses = _collect(h, PoseDetectionArray, "/perception/poses")
    pub = h.helper.create_publisher(CompressedImage, "/camera/usb/image_raw/compressed", _RELIABLE)
    assert h.spin_until(lambda: pub.get_subscription_count() > 0)
    pub.publish(_image(0))
    assert h.spin_until(lambda: len(poses) >= 1)
    assert poses[0].detections == []


def test_detect_mode_publishes_no_poses_topic(make_harness):
    h = make_harness(203, _detector(PoseBackend([_person_with_keypoints()])))
    assert h.node.count_publishers("/perception/poses") == 0
    assert h.node.count_publishers("/perception/detections") == 1


def test_bad_detector_mode_is_refused():
    import rclpy
    from racer_perception.detector_node import DetectorNode
    from racer_perception.yolo_backend import DetectorUnavailable
    from rclpy.context import Context

    context = Context()
    rclpy.init(context=context, domain_id=204)
    try:
        with pytest.raises(DetectorUnavailable, match="mode"):
            DetectorNode(
                backend=PoseBackend(),
                context=context,
                parameter_overrides=[Parameter("mode", value="segment")],
            )
    finally:
        rclpy.shutdown(context=context)


def test_fusion_poses_input_carries_keypoints_through_with_bearing_and_range(make_harness):
    h = make_harness(210, _fusion(input="poses"))
    objects = _collect(h, ObjectDetectionArray, "/perception/objects")
    scan_pub = h.helper.create_publisher(_nodes.LaserScan, "/scan", _BEST_EFFORT)
    pose_pub = h.helper.create_publisher(PoseDetectionArray, "/perception/poses", _RELIABLE)
    assert h.spin_until(
        lambda: scan_pub.get_subscription_count() > 0 and pose_pub.get_subscription_count() > 0
    )
    scan_pub.publish(_scan(100.0, near_vehicle_bearing_deg=0.0, range_m=2.0))
    h.spin_for(0.3)
    pose_pub.publish(_pose_msg(100.05, 640.0, 200.0, width_px=1280))
    assert h.spin_until(lambda: len(objects) >= 1)
    (obj,) = objects[0].detections
    assert obj.bearing_rad == pytest.approx(0.0, abs=1e-6)
    assert obj.range_valid and obj.range_m == pytest.approx(2.0)
    assert obj.keypoint_layout == "coco17" and len(obj.keypoints) == 17
    assert obj.keypoints[3].x_px == pytest.approx(643.0)
    assert obj.keypoints[3].confidence == pytest.approx(0.8)


def test_fusion_uses_the_pose_arrays_image_width_for_the_bearing(make_harness):
    h = make_harness(211, _fusion(input="poses", image_width_px=1280))
    objects = _collect(h, ObjectDetectionArray, "/perception/objects")
    pose_pub = h.helper.create_publisher(PoseDetectionArray, "/perception/poses", _RELIABLE)
    assert h.spin_until(lambda: pose_pub.get_subscription_count() > 0)
    # A box at pixel 80 in a 320 px wide image is 80 px left of its centre; with the parameter's
    # 1280 px it would be 560 px left of centre, a much larger angle.
    pose_pub.publish(_pose_msg(100.0, 80.0, 20.0, width_px=320))
    pose_pub.publish(_pose_msg(100.1, 80.0, 20.0, width_px=0))  # unknown: the parameter is used
    assert h.spin_until(lambda: len(objects) >= 2)
    narrow = objects[0].detections[0].bearing_rad
    param = objects[1].detections[0].bearing_rad
    assert narrow == pytest.approx(math.atan(80.0 / ((320 / 2) / math.tan(1.48 / 2))))
    assert param == pytest.approx(math.atan(560.0 / ((1280 / 2) / math.tan(1.48 / 2))))
    assert narrow < param


def test_fusion_poses_input_keeps_the_nearest_person_target_with_keypoints(make_harness):
    h = make_harness(212, _fusion(input="poses"))
    targets = _collect(h, _nodes.ObjectDetection, "/perception/target")
    pose_pub = h.helper.create_publisher(PoseDetectionArray, "/perception/poses", _RELIABLE)
    assert h.spin_until(lambda: pose_pub.get_subscription_count() > 0)
    pose_pub.publish(_pose_msg(100.0, 640.0, 200.0, width_px=1280))
    assert h.spin_until(lambda: len(targets) >= 1)
    assert len(targets[0].keypoints) == 17


def test_fusion_refuses_an_unknown_input(make_harness):
    with pytest.raises(ValueError, match="input"):
        make_harness(213, _fusion(input="lidar"))


def test_fusion_detections_input_has_no_keypoints(make_harness):
    h = make_harness(214, _fusion())
    objects = _collect(h, ObjectDetectionArray, "/perception/objects")
    det_pub = h.helper.create_publisher(Detection2DArray, "/perception/detections", _RELIABLE)
    assert h.spin_until(lambda: det_pub.get_subscription_count() > 0)
    det_pub.publish(_nodes._detections(100.0, [("person", 0.9, 640.0, 200.0)]))
    assert h.spin_until(lambda: len(objects) >= 1)
    assert objects[0].detections[0].keypoints == []
    assert objects[0].detections[0].keypoint_layout == ""
