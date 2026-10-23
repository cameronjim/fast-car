"""Pure camera-to-LiDAR fusion maths (roadmap 5.1). No ROS, no hardware, no numpy.

Turns a detection box (pixels) into a vehicle-frame bearing with a pinhole model, then looks
up a LiDAR scan inside the box's angular window to get a robust range. racer_perception/
fusion_node.py is the thin ROS plumbing around this module; everything here is unit-tested
without rclpy (claude-docs/12-testing.md L1).

CONVENTIONS (SI, claude-docs/06-vehicle-params.md): metres, radians, seconds. Bearings are in
the VEHICLE frame (base_link), LEFT positive, 0 straight ahead (REP-103), wrapped to (-pi, pi].
A pixel LEFT of the image centre is therefore a POSITIVE camera bearing.

PINHOLE MODEL. For an image `image_width_px` wide and horizontal field of view `hfov_rad`,
the focal length in pixels is f = (W / 2) / tan(hfov / 2) and the camera-frame bearing of
pixel column u is atan((W / 2 - u) / f). Lens distortion is ignored (the ELP's roughly
85 degree lens is mildly distorted at the edges; see docs/notes/perception-2026-10-23.md).
`yaw_rad` is the camera's yaw relative to base_link, added to the camera bearing. No camera
mount pose is measured yet, so the default is 0 and it is a launch parameter. The LiDAR
mount yaw (CLAUDE.md invariant 2) comes from vehicle_params through the generated binding in
fusion_node, never from this file.

LASER vs VEHICLE BEARING. A LaserScan ray i has laser bearing angle_min + i * angle_increment
in the laser frame. vehicle bearing = wrap(laser bearing + mount yaw), exactly the rule
racer_safety and racer_control use (mount yaw is pi on the real C1: a box straight ahead of
the nose reads at a laser bearing near 180 degrees).

ROBUST RANGE. A box around a person also contains wall behind them, and the window can clip
an arm or a chair leg. The range reported is the median of the closest `closest_fraction`
(default 30 percent, at least one) of the valid returns in the window. Closest-biased on
purpose: the thing the detector boxed is in front of its background.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

#: range_status values (mirrored in racer_msgs/ObjectDetection.msg).
STATUS_OK = "ok"
STATUS_NO_SCAN = "no_scan"
STATUS_STALE_SCAN = "stale_scan"
STATUS_NO_RETURNS = "no_returns"

#: Yaw consistency tolerance, same as racer_safety's resolve_laser_yaw_rad.
_YAW_MATCH_TOL_RAD = 1e-9


def wrap_angle(angle_rad: float) -> float:
    """Wrap to (-pi, pi]."""
    wrapped = math.fmod(angle_rad + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


@dataclass(frozen=True)
class CameraModel:
    """Pinhole camera, horizontal axis only (bearing needs no vertical model)."""

    image_width_px: int
    hfov_rad: float
    yaw_rad: float = 0.0

    def __post_init__(self) -> None:
        if self.image_width_px <= 0:
            raise ValueError(f"image_width_px must be positive, got {self.image_width_px}")
        if not (math.isfinite(self.hfov_rad) and 0.0 < self.hfov_rad < math.pi):
            raise ValueError(f"hfov_rad must be in (0, pi), got {self.hfov_rad}")
        if not math.isfinite(self.yaw_rad):
            raise ValueError(f"yaw_rad must be finite, got {self.yaw_rad}")

    @property
    def focal_px(self) -> float:
        return (self.image_width_px / 2.0) / math.tan(self.hfov_rad / 2.0)


def camera_bearing_rad(model: CameraModel, u_px: float) -> float:
    """Bearing of pixel column u in the CAMERA frame (left positive), no yaw offset."""
    return math.atan((model.image_width_px / 2.0 - u_px) / model.focal_px)


def pixel_bearing_rad(model: CameraModel, u_px: float) -> float:
    """Bearing of pixel column u in the VEHICLE frame: camera bearing plus camera yaw."""
    return wrap_angle(camera_bearing_rad(model, u_px) + model.yaw_rad)


def box_bearing_and_half_width(
    model: CameraModel, cx_px: float, width_px: float
) -> tuple[float, float]:
    """Return (vehicle bearing of the box centre, half the box's angular width).

    The angular width comes from the two box edges, not from width * (hfov / W), so it is
    right off-axis where pixels subtend less angle. Edges are clamped to the image so a box
    clipped by the frame does not claim to see beyond it.
    """
    half_w = max(width_px, 0.0) / 2.0
    left = camera_bearing_rad(model, max(cx_px - half_w, 0.0))
    right = camera_bearing_rad(model, min(cx_px + half_w, float(model.image_width_px)))
    return pixel_bearing_rad(model, cx_px), abs(left - right) / 2.0


def resolve_laser_yaw_rad(
    binding_rad: float | None, parameter_rad: float, from_vehicle_params: bool
) -> float | None:
    """Pick the LiDAR mount yaw, same rules as racer_safety/racer_control.

    * from_vehicle_params true (the real car), binding unset: parameter_rad is used.
    * true, binding set: the binding wins; parameter_rad must be 0 or equal the binding
      (within 1e-9 rad), anything else returns None and the node must refuse to start.
    * false: the binding is ignored and parameter_rad is used. Only for scans that are not
      the real car's LiDAR (the simulator's /scan is aligned to the vehicle, yaw 0).
    A non-finite parameter, or a non-finite binding that would be used, returns None.
    """
    if not math.isfinite(parameter_rad):
        return None
    if not from_vehicle_params:
        return parameter_rad
    if binding_rad is None:
        return parameter_rad
    if not math.isfinite(binding_rad):
        return None
    if parameter_rad != 0.0 and abs(wrap_angle(parameter_rad - binding_rad)) > _YAW_MATCH_TOL_RAD:
        return None
    return binding_rad


@dataclass(frozen=True)
class ScanView:
    """The parts of one sensor_msgs/LaserScan the fusion needs."""

    stamp_s: float
    angle_min_rad: float
    angle_increment_rad: float
    range_min_m: float
    range_max_m: float
    ranges: Sequence[float]
    laser_yaw_rad: float


def valid_returns_in_window(
    scan: ScanView, centre_bearing_rad: float, half_width_rad: float
) -> list[float]:
    """Valid ranges whose VEHICLE bearing lies within +/- half_width of centre_bearing.

    A return is valid if it is finite and inside the scan's own [range_min, range_max] band
    (the rule safety_node and lidar_check use; sllidar reports no-return as +inf). The window
    is widened to at least one beam spacing so a box narrower than the beam pitch still
    selects the beam nearest its centre. The comparison wraps, so a window straddling +/- pi
    works.
    """
    step = abs(scan.angle_increment_rad)
    half = max(half_width_rad, step / 2.0)
    out: list[float] = []
    for i, value in enumerate(scan.ranges):
        if not math.isfinite(value) or not scan.range_min_m <= value <= scan.range_max_m:
            continue
        bearing = wrap_angle(scan.angle_min_rad + i * scan.angle_increment_rad + scan.laser_yaw_rad)
        if abs(wrap_angle(bearing - centre_bearing_rad)) <= half:
            out.append(float(value))
    return out


def robust_range_m(values: Sequence[float], closest_fraction: float = 0.3) -> float | None:
    """Median of the closest `closest_fraction` of values (at least one); None if empty."""
    if not 0.0 < closest_fraction <= 1.0:
        raise ValueError(f"closest_fraction must be in (0, 1], got {closest_fraction}")
    if not values:
        return None
    ordered = sorted(values)
    keep = max(1, math.ceil(closest_fraction * len(ordered)))
    near = ordered[:keep]
    mid = len(near) // 2
    if len(near) % 2:
        return near[mid]
    return 0.5 * (near[mid - 1] + near[mid])


@dataclass(frozen=True)
class RawDetection:
    """One detector output: class, score and a pixel box (centre and size)."""

    class_name: str
    score: float
    cx_px: float
    cy_px: float
    width_px: float
    height_px: float


@dataclass(frozen=True)
class FusedObject:
    """A detection with a vehicle-frame bearing and a LiDAR range (or why there is none)."""

    stamp_s: float
    class_name: str
    score: float
    bearing_rad: float
    bearing_half_width_rad: float
    range_m: float  # NaN when range_valid is False
    range_valid: bool
    range_status: str
    range_samples: int
    cx_px: float
    cy_px: float
    width_px: float
    height_px: float


def fuse_detection(
    det: RawDetection,
    stamp_s: float,
    camera: CameraModel,
    scan: ScanView | None,
    max_scan_age_s: float,
    closest_fraction: float = 0.3,
) -> FusedObject:
    """Fuse one detection (image stamp `stamp_s`) with the newest scan, if any.

    Staleness is |image stamp - scan stamp| > max_scan_age_s, in either direction: a detector
    has latency, so the newest scan is normally slightly NEWER than the image it is paired
    with. The bearing is always computed; only the range depends on the scan.
    """
    bearing, half_width = box_bearing_and_half_width(camera, det.cx_px, det.width_px)
    range_m = math.nan
    samples = 0
    if scan is None:
        status = STATUS_NO_SCAN
    elif not math.isfinite(scan.stamp_s - stamp_s) or abs(scan.stamp_s - stamp_s) > max_scan_age_s:
        status = STATUS_STALE_SCAN
    else:
        window = valid_returns_in_window(scan, bearing, half_width)
        robust = robust_range_m(window, closest_fraction)
        if robust is None:
            status = STATUS_NO_RETURNS
        else:
            status = STATUS_OK
            range_m = robust
            samples = len(window)
    return FusedObject(
        stamp_s=stamp_s,
        class_name=det.class_name,
        score=det.score,
        bearing_rad=bearing,
        bearing_half_width_rad=half_width,
        range_m=range_m,
        range_valid=status == STATUS_OK,
        range_status=status,
        range_samples=samples,
        cx_px=det.cx_px,
        cy_px=det.cy_px,
        width_px=det.width_px,
        height_px=det.height_px,
    )
