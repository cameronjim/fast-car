"""L1: the benchmark's statistics and keypoints surviving the pure fusion step."""

from __future__ import annotations

import pytest
from racer_perception.bench import percentile
from racer_perception.fusion import CameraModel, RawDetection, fuse_detection
from racer_perception.keypoints import Keypoint


def test_percentile_matches_hand_computed_values():
    values = [10.0, 20.0, 30.0, 40.0, 50.0]
    assert percentile(values, 0) == 10.0
    assert percentile(values, 50) == 30.0
    assert percentile(values, 100) == 50.0
    assert percentile(values, 95) == pytest.approx(48.0)  # 40 + 0.8 * (50 - 40)
    assert percentile([7.0], 95) == 7.0
    assert percentile([3.0, 1.0, 2.0], 50) == 2.0  # unsorted input
    with pytest.raises(ValueError):
        percentile([], 50)


def test_fusion_carries_keypoints_through_unchanged():
    kps = (Keypoint(1.0, 2.0, 0.9), Keypoint(3.0, 4.0, 0.1))
    det = RawDetection("person", 0.9, 640.0, 360.0, 100.0, 300.0, kps)
    fused = fuse_detection(det, 10.0, CameraModel(1280, 1.48), None, 0.5)
    assert fused.keypoints == kps
    assert fused.range_status == "no_scan"


def test_plain_detections_have_no_keypoints():
    det = RawDetection("chair", 0.5, 640.0, 360.0, 100.0, 300.0)
    assert det.keypoints == ()
    assert fuse_detection(det, 10.0, CameraModel(1280, 1.48), None, 0.5).keypoints == ()
