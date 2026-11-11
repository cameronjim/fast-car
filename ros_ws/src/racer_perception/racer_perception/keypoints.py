"""Body keypoint types and the COCO-17 layout (roadmap 5.1 pose mode, 5.3). No ROS, no numpy.

The YOLO pose models (yolov8n-pose and friends) output the 17 COCO keypoints per person, in
this order. `left_*` and `right_*` are the PERSON's own left and right, not the image's.

Image convention (same as racer_msgs/Keypoint2D): pixels, origin top-left, x right, y DOWN, so a
raised wrist has a SMALLER y than its shoulder. A keypoint the model could not place comes out
with confidence 0 and position (0, 0): consumers must test confidence, never position.
"""

from __future__ import annotations

from dataclasses import dataclass

LAYOUT_COCO17 = "coco17"

COCO17_NAMES: tuple[str, ...] = (
    "nose",
    "left_eye",
    "right_eye",
    "left_ear",
    "right_ear",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
    "left_hip",
    "right_hip",
    "left_knee",
    "right_knee",
    "left_ankle",
    "right_ankle",
)

COCO17_INDEX: dict[str, int] = {name: i for i, name in enumerate(COCO17_NAMES)}

NOSE = COCO17_INDEX["nose"]
LEFT_SHOULDER = COCO17_INDEX["left_shoulder"]
RIGHT_SHOULDER = COCO17_INDEX["right_shoulder"]
LEFT_ELBOW = COCO17_INDEX["left_elbow"]
RIGHT_ELBOW = COCO17_INDEX["right_elbow"]
LEFT_WRIST = COCO17_INDEX["left_wrist"]
RIGHT_WRIST = COCO17_INDEX["right_wrist"]
LEFT_HIP = COCO17_INDEX["left_hip"]
RIGHT_HIP = COCO17_INDEX["right_hip"]


@dataclass(frozen=True)
class Keypoint:
    x_px: float
    y_px: float
    confidence: float


def keypoints_from_flat(xy: list[list[float]], conf: list[float] | None) -> tuple[Keypoint, ...]:
    """Build Keypoints from the detector's (N, 2) positions and optional (N,) confidences.

    A model that gives no per-keypoint confidence (conf is None) is treated as fully
    confident (1.0): there is nothing better to say, and the rules still apply their own
    geometry checks.
    """
    if conf is not None and len(conf) != len(xy):
        raise ValueError(f"{len(xy)} keypoint positions but {len(conf)} confidences")
    return tuple(
        Keypoint(float(p[0]), float(p[1]), 1.0 if conf is None else float(conf[i]))
        for i, p in enumerate(xy)
    )
