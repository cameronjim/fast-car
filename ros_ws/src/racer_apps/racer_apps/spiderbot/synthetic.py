"""Synthetic people for tests and the fake detector: COCO-17 keypoints from a simple body model.

NOT a model of real pose detector output (that has noise, occlusion and odd limbs; the rule tests
also run on keypoints recorded from yolov8n-pose on real photographs, see
racer_perception/test/fixtures/keypoints). This is for building exact, controllable cases:
"both wrists 0.3 body heights above the head", "a wrist covered", "a fist thrown over 0.2 s".

Body model, in units of the person's pixel height `h` and a head-top at `top_px`:
shoulders at 0.18 h (half width 0.12 h), hips at 0.52 h (half width 0.07 h), knees at 0.74 h,
ankles at 0.97 h, arms 0.33 h from shoulder to wrist (elbow halfway). Image y points DOWN.
"""

from __future__ import annotations

from racer_perception.keypoints import COCO17_NAMES, Keypoint

POSES = ("standing", "arms_raised", "one_arm_raised", "arms_wide", "guard")

_ARM = 0.33
_SHOULDER_Y = 0.18
_SHOULDER_HALF = 0.12


def _arm(pose: str, side: int, reach: float) -> tuple[float, float]:
    """Wrist offset (dx, dy) in body heights from the same-side shoulder. side: +1 left, -1 right.

    In the IMAGE the person's left is on the image's right (they face the camera), so the
    left arm lives at larger x.
    """
    if pose == "arms_raised" or (pose == "one_arm_raised" and side == 1):
        return (side * 0.05, -_ARM)
    if pose == "arms_wide":
        return (side * _ARM, -0.03)
    if pose == "guard":
        # Wrist in front of the chin, extending outward and level with the shoulder by `reach`.
        return (side * (0.02 + reach * (_ARM - 0.02)), -0.06 + reach * 0.06)
    return (side * 0.03, _ARM - 0.02)  # standing, or the lowered arm of one_arm_raised


def synthetic_keypoints(
    pose: str,
    cx_px: float,
    top_px: float,
    height_px: float,
    confidence: float = 0.9,
    reach: float = 0.0,
    punching_side: int = 0,
) -> tuple[Keypoint, ...]:
    """17 COCO keypoints for `pose` (see POSES).

    `reach` only matters for pose "guard": 0 is a guard with the wrists by the chin, 1 is the
    wrists fully extended. `punching_side` (+1 left, -1 right, 0 none) limits the reach to that
    arm, so a single fist can be thrown.
    """
    if pose not in POSES:
        raise ValueError(f"pose must be one of {POSES}, got {pose!r}")
    h = height_px

    def at(dx: float, dy: float) -> tuple[float, float]:
        return (cx_px + dx * h, top_px + dy * h)

    pts: dict[str, tuple[float, float]] = {
        "nose": at(0.0, 0.05),
        "left_eye": at(0.015, 0.04),
        "right_eye": at(-0.015, 0.04),
        "left_ear": at(0.03, 0.05),
        "right_ear": at(-0.03, 0.05),
        "left_hip": at(0.07, 0.52),
        "right_hip": at(-0.07, 0.52),
        "left_knee": at(0.07, 0.74),
        "right_knee": at(-0.07, 0.74),
        "left_ankle": at(0.07, 0.97),
        "right_ankle": at(-0.07, 0.97),
    }
    for side, name in ((1, "left"), (-1, "right")):
        shoulder = at(side * _SHOULDER_HALF, _SHOULDER_Y)
        arm_reach = reach if punching_side in (0, side) else 0.0
        dx, dy = _arm(pose, side, arm_reach)
        wrist = (shoulder[0] + dx * h, shoulder[1] + dy * h)
        pts[f"{name}_shoulder"] = shoulder
        pts[f"{name}_wrist"] = wrist
        pts[f"{name}_elbow"] = ((shoulder[0] + wrist[0]) / 2.0, (shoulder[1] + wrist[1]) / 2.0)
    return tuple(Keypoint(pts[n][0], pts[n][1], confidence) for n in COCO17_NAMES)


def with_confidence(
    keypoints: tuple[Keypoint, ...], names: tuple[str, ...], confidence: float
) -> tuple[Keypoint, ...]:
    """Copy of `keypoints` with the named joints' confidence replaced (0 = not seen)."""
    wanted = {COCO17_NAMES.index(n) for n in names}
    return tuple(
        Keypoint(k.x_px, k.y_px, confidence) if i in wanted else k for i, k in enumerate(keypoints)
    )
