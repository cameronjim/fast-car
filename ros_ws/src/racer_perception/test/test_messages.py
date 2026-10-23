"""L3 check of racer_msgs/ObjectDetection and ObjectDetectionArray (roadmap 5.1).

Pins the fields racer_perception and the behaviours to come rely on, and that a message
survives serialization (the default NaN range and the status string included).
"""

from __future__ import annotations

import math

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("racer_msgs")

from racer_msgs.msg import ObjectDetection, ObjectDetectionArray
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
