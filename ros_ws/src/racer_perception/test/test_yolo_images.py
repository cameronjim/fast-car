"""Real YOLO and YOLO-pose on the committed test photographs (opt-in, CPU is enough).

Needs ultralytics and torch AND the pinned weights (RACER_WEIGHTS_DIR, which the car image sets
to /opt/racer_weights, or the model cache): skipped everywhere else, so CI never downloads
anything. Run it in the perception image:

    tools/perception_cpu_tests.sh

Compared with the keypoints recorded from the same model on the same images
(test/fixtures/keypoints/yolov8n_pose_images.json) with stated tolerances, never exact floats:
scores within 0.05, box centres and sizes within 8 px, keypoints within 10 px when both the
recording and the new run are at least 0.5 confident. Those tolerances absorb the difference
between CPU thread counts and torch builds, not a different model: a changed weights file is
refused by the sha256 pin before it gets here.
"""

from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(
    importlib.util.find_spec("ultralytics") is None, reason="ultralytics is not installed"
)

from racer_perception.weights import WeightsError, locate_weights
from racer_perception.yolo_backend import load_yolo_backend

HERE = Path(__file__).resolve().parent
IMAGES = HERE / "fixtures" / "images"
GOLDEN = json.loads((HERE / "fixtures" / "keypoints" / "yolov8n_pose_images.json").read_text())
CACHE = os.environ.get("RACER_TEST_WEIGHTS_CACHE", "~/.cache/racer_perception")

SCORE_TOL = 0.05
BOX_TOL_PX = 8.0
KEYPOINT_TOL_PX = 10.0


def _backend(task):
    model = {"pose": "yolov8n-pose.pt", "detect": "yolov8n.pt"}[task]
    try:
        locate_weights(model, CACHE)
    except WeightsError as exc:
        pytest.skip(f"pinned weights not available offline: {exc}")
    return load_yolo_backend("", "cpu", CACHE, 0.25, task)


@pytest.fixture(scope="module")
def pose_backend():
    return _backend("pose")


@pytest.mark.parametrize("name", sorted(GOLDEN))
def test_pose_matches_the_recorded_keypoints(pose_backend, name):
    got = pose_backend.infer((IMAGES / name).read_bytes())
    want = GOLDEN[name]
    assert len(got) == len(want)
    order = lambda items, key: sorted(items, key=key)
    got = order(got, lambda d: d.cx_px)
    want = order(want, lambda d: d["box_cxcywh_px"][0])
    for g, w in zip(got, want, strict=True):
        assert g.class_name == w["class_name"] == "person"
        assert abs(g.score - w["score"]) <= SCORE_TOL
        for have, expect in zip((g.cx_px, g.cy_px, g.width_px, g.height_px), w["box_cxcywh_px"]):
            assert abs(have - expect) <= BOX_TOL_PX
        assert len(g.keypoints) == 17
        for k, (x, y, c) in zip(g.keypoints, w["keypoints"], strict=True):
            if k.confidence >= 0.5 and c >= 0.5:
                assert abs(k.x_px - x) <= KEYPOINT_TOL_PX and abs(k.y_px - y) <= KEYPOINT_TOL_PX


def test_pose_finds_the_hiker_with_arms_up_and_the_four_people(pose_backend):
    hiker = pose_backend.infer((IMAGES / "arms_raised_hiker.jpg").read_bytes())
    assert len(hiker) == 1
    kp = hiker[0].keypoints
    assert kp[9].y_px < kp[5].y_px and kp[10].y_px < kp[6].y_px  # both wrists above shoulders
    group = pose_backend.infer((IMAGES / "four_people_standing.jpg").read_bytes())
    assert len(group) == 4


def test_plain_detector_finds_the_people_too():
    backend = _backend("detect")
    dets = backend.infer((IMAGES / "four_people_standing.jpg").read_bytes())
    assert sum(d.class_name == "person" for d in dets) >= 4
    assert all(d.keypoints == () for d in dets)


def test_annotate_returns_a_jpeg_with_a_skeleton(pose_backend):
    jpeg = (IMAGES / "arms_raised_hiker.jpg").read_bytes()
    out = pose_backend.annotate(jpeg, pose_backend.infer(jpeg))
    assert out[:2] == b"\xff\xd8" and out != jpeg
