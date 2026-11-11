"""L3 check of racer_msgs/ObjectDetection and ObjectDetectionArray (roadmap 5.1).

Pins the fields racer_perception and the behaviours to come rely on, and that a message
survives serialization (the default NaN range and the status string included).
"""

from __future__ import annotations

import math

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("racer_msgs")

from racer_msgs.msg import (
    Keypoint2D,
    ObjectDetection,
    ObjectDetectionArray,
    PoseDetection,
    PoseDetectionArray,
    SpiderbotAlert,
)
from rclpy.serialization import deserialize_message, serialize_message


def test_object_detection_fields_and_types():
    fields = ObjectDetection.get_fields_and_field_types()
    assert fields == {
        "header": "std_msgs/Header",
        "class_name": "string",
        "score": "float",
        "bearing_rad": "float",
        "bearing_half_width_rad": "float",
        "range_m": "float",
        "range_valid": "boolean",
        "range_status": "string",
        "range_samples": "uint16",
        "box_cx_px": "float",
        "box_cy_px": "float",
        "box_width_px": "float",
        "box_height_px": "float",
        "keypoint_layout": "string",
        "keypoints": "sequence<racer_msgs/Keypoint2D>",
    }


def test_object_detection_round_trips_including_nan_range():
    msg = ObjectDetection()
    msg.header.frame_id = "base_link"
    msg.header.stamp.sec = 12
    msg.class_name = "person"
    msg.score = 0.5
    msg.bearing_rad = -0.25
    msg.range_m = math.nan
    msg.range_valid = False
    msg.range_status = "no_returns"
    msg.box_width_px = 100.0
    back = deserialize_message(serialize_message(msg), ObjectDetection)
    assert back.class_name == "person" and back.range_status == "no_returns"
    assert math.isnan(back.range_m) and back.range_valid is False
    assert back.bearing_rad == pytest.approx(-0.25) and back.header.stamp.sec == 12


def test_array_holds_detections_and_a_header():
    arr = ObjectDetectionArray()
    arr.header.frame_id = "base_link"
    arr.detections.append(ObjectDetection())
    arr.detections[0].class_name = "chair"
    back = deserialize_message(serialize_message(arr), ObjectDetectionArray)
    assert back.header.frame_id == "base_link"
    assert [d.class_name for d in back.detections] == ["chair"]
    assert ObjectDetectionArray().detections == []


def test_keypoint_and_pose_message_fields():
    assert Keypoint2D.get_fields_and_field_types() == {
        "x_px": "float",
        "y_px": "float",
        "confidence": "float",
    }
    assert PoseDetection.get_fields_and_field_types() == {
        "class_name": "string",
        "score": "float",
        "box_cx_px": "float",
        "box_cy_px": "float",
        "box_width_px": "float",
        "box_height_px": "float",
        "keypoint_layout": "string",
        "keypoints": "sequence<racer_msgs/Keypoint2D>",
    }
    assert PoseDetectionArray.get_fields_and_field_types() == {
        "header": "std_msgs/Header",
        "image_width_px": "uint32",
        "image_height_px": "uint32",
        "detections": "sequence<racer_msgs/PoseDetection>",
    }


def test_pose_array_round_trips_with_17_keypoints():
    arr = PoseDetectionArray()
    arr.image_width_px, arr.image_height_px = 1280, 720
    det = PoseDetection()
    det.class_name, det.keypoint_layout = "person", "coco17"
    for i in range(17):
        det.keypoints.append(Keypoint2D(x_px=float(i), y_px=2.0 * i, confidence=0.5))
    arr.detections.append(det)
    back = deserialize_message(serialize_message(arr), PoseDetectionArray)
    assert back.image_width_px == 1280 and len(back.detections[0].keypoints) == 17
    assert back.detections[0].keypoints[16].y_px == pytest.approx(32.0)


def test_object_detection_without_keypoints_has_empty_defaults():
    msg = ObjectDetection()
    assert msg.keypoints == [] and msg.keypoint_layout == ""


def test_spiderbot_alert_fields_and_round_trip():
    fields = SpiderbotAlert.get_fields_and_field_types()
    assert list(fields) == [
        "header",
        "event_id",
        "rule",
        "held_frames",
        "score",
        "bearing_rad",
        "range_m",
        "range_valid",
        "facing_ok",
        "final_bearing_rad",
        "pose_valid",
        "pose_x_m",
        "pose_y_m",
        "pose_yaw_rad",
        "snapshot_path",
    ]
    msg = SpiderbotAlert()
    msg.event_id, msg.rule, msg.facing_ok = "e-1", "arms_raised", True
    msg.range_m = math.nan
    back = deserialize_message(serialize_message(msg), SpiderbotAlert)
    assert back.event_id == "e-1" and back.facing_ok is True and math.isnan(back.range_m)
