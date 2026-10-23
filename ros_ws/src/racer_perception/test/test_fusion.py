"""L1 tests for racer_perception.fusion: bearing, window, robust range, yaw, edge cases.

Pure Python, hand-computed cases. Sign convention: LEFT positive, 0 straight ahead.
"""

from __future__ import annotations

import math

import pytest
from racer_perception import fusion
from racer_perception.fusion import (
    STATUS_NO_RETURNS,
    STATUS_NO_SCAN,
    STATUS_OK,
    STATUS_STALE_SCAN,
    CameraModel,
    RawDetection,
    ScanView,
    box_bearing_and_half_width,
    fuse_detection,
    pixel_bearing_rad,
    resolve_laser_yaw_rad,
    robust_range_m,
    valid_returns_in_window,
    wrap_angle,
)

HFOV = 1.48
CAM = CameraModel(image_width_px=1280, hfov_rad=HFOV)
DEG = math.pi / 180.0
INF = math.inf


def make_scan(
    bearings_deg_to_range: dict[int, float],
    *,
    yaw: float = 0.0,
    stamp: float = 100.0,
    n: int = 360,
    fill: float = INF,
) -> ScanView:
    """1 degree beams over the full circle in the LASER frame; given laser bearing -> range."""
    ranges = [fill] * n
    for deg, value in bearings_deg_to_range.items():
        ranges[(deg + 180) % n] = value  # beam i is at laser bearing (i - 180) degrees
    return ScanView(
        stamp_s=stamp,
        angle_min_rad=-math.pi,
        angle_increment_rad=DEG,
        range_min_m=0.05,
        range_max_m=12.0,
        ranges=ranges,
        laser_yaw_rad=yaw,
    )


def det(cx=640.0, width=100.0, name="person", score=0.9):
    return RawDetection(name, score, cx, 360.0, width, 300.0)


# ---------------------------------------------------------------- wrap_angle


@pytest.mark.parametrize(
    "angle, expected",
    [
        (0.0, 0.0),
        (math.pi, math.pi),
        (-math.pi, math.pi),
        (3 * math.pi / 2, -math.pi / 2),
        (-3 * math.pi / 2, math.pi / 2),
        (2 * math.pi, 0.0),
        (7.0, 7.0 - 2 * math.pi),
    ],
)
def test_wrap_angle(angle, expected):
    assert wrap_angle(angle) == pytest.approx(expected, abs=1e-12)


# ------------------------------------------------------------ pinhole bearing


def test_image_centre_is_straight_ahead():
    assert pixel_bearing_rad(CAM, 640.0) == pytest.approx(0.0, abs=1e-12)


def test_image_edges_are_half_the_fov_and_left_is_positive():
    assert pixel_bearing_rad(CAM, 0.0) == pytest.approx(HFOV / 2)
    assert pixel_bearing_rad(CAM, 1280.0) == pytest.approx(-HFOV / 2)


def test_bearing_matches_hand_computation_off_axis():
    # f = 640 / tan(0.74); pixel 320 px right of centre -> atan(-320 / f)
    f = 640.0 / math.tan(HFOV / 2)
    assert pixel_bearing_rad(CAM, 960.0) == pytest.approx(math.atan(-320.0 / f))
    assert pixel_bearing_rad(CAM, 960.0) < 0.0


def test_bearing_is_not_linear_in_pixels():
    # The pinhole model compresses the edges: half-way to the edge is MORE than half the FOV/2.
    assert pixel_bearing_rad(CAM, 320.0) > HFOV / 4


def test_camera_yaw_offset_is_added_and_wrapped():
    yawed = CameraModel(1280, HFOV, yaw_rad=0.3)
    assert pixel_bearing_rad(yawed, 640.0) == pytest.approx(0.3)
    behind = CameraModel(1280, HFOV, yaw_rad=math.pi)
    assert pixel_bearing_rad(behind, 640.0) == pytest.approx(math.pi)
    assert pixel_bearing_rad(behind, 0.0) == pytest.approx(wrap_angle(math.pi + HFOV / 2))


@pytest.mark.parametrize(
    "kwargs",
    [
        {"image_width_px": 0, "hfov_rad": 1.0},
        {"image_width_px": -5, "hfov_rad": 1.0},
        {"image_width_px": 640, "hfov_rad": 0.0},
        {"image_width_px": 640, "hfov_rad": math.pi},
        {"image_width_px": 640, "hfov_rad": math.nan},
        {"image_width_px": 640, "hfov_rad": 1.0, "yaw_rad": math.inf},
    ],
)
def test_camera_model_rejects_bad_parameters(kwargs):
    with pytest.raises(ValueError):
        CameraModel(**kwargs)


def test_box_half_width_symmetric_on_axis():
    bearing, half = box_bearing_and_half_width(CAM, 640.0, 200.0)
    f = 640.0 / math.tan(HFOV / 2)
    assert bearing == pytest.approx(0.0, abs=1e-12)
    assert half == pytest.approx(math.atan(100.0 / f))


def test_box_half_width_off_axis_is_narrower_than_on_axis_for_same_pixels():
    _, centre = box_bearing_and_half_width(CAM, 640.0, 200.0)
    _, edge = box_bearing_and_half_width(CAM, 1100.0, 200.0)
    assert edge < centre


def test_box_clipped_by_the_frame_does_not_see_past_it():
    # Box centred on the left image edge: its left half is outside the image and is clamped
    # to column 0, so the half width is half the angle between column 0 and column 100.
    bearing, half = box_bearing_and_half_width(CAM, 0.0, 200.0)
    f = 640.0 / math.tan(HFOV / 2)
    assert bearing == pytest.approx(HFOV / 2)
    assert half == pytest.approx((HFOV / 2 - math.atan(540.0 / f)) / 2)


def test_zero_and_negative_width_boxes_have_zero_half_width():
    assert box_bearing_and_half_width(CAM, 640.0, 0.0)[1] == 0.0
    assert box_bearing_and_half_width(CAM, 640.0, -10.0)[1] == 0.0


# ------------------------------------------------------------------ yaw rules


def test_yaw_binding_used_when_parameter_zero():
    assert resolve_laser_yaw_rad(math.pi, 0.0, True) == math.pi


def test_yaw_binding_unset_falls_back_to_parameter():
    assert resolve_laser_yaw_rad(None, 0.7, True) == 0.7
    assert resolve_laser_yaw_rad(None, 0.0, True) == 0.0


def test_yaw_parameter_must_agree_with_binding():
    assert resolve_laser_yaw_rad(math.pi, math.pi, True) == math.pi
    assert resolve_laser_yaw_rad(math.pi, -math.pi, True) == math.pi  # same angle, wrapped
    assert resolve_laser_yaw_rad(math.pi, 1.0, True) is None


def test_yaw_binding_ignored_for_simulator():
    assert resolve_laser_yaw_rad(math.pi, 0.0, False) == 0.0
    assert resolve_laser_yaw_rad(math.pi, 0.4, False) == 0.4


def test_yaw_non_finite_is_refused():
    assert resolve_laser_yaw_rad(math.pi, math.nan, True) is None
    assert resolve_laser_yaw_rad(math.pi, math.inf, False) is None
    assert resolve_laser_yaw_rad(math.nan, 0.0, True) is None


# -------------------------------------------------------------- window select


def test_window_selects_by_vehicle_bearing_with_mount_yaw_pi():
    # Real C1: a box straight ahead reads at laser bearing 180 degrees. Vehicle bearing 0
    # therefore sits at laser bearing +/-180.
    scan = make_scan({180: 2.0, -179: 2.1, 0: 9.0}, yaw=math.pi)
    got = valid_returns_in_window(scan, 0.0, 3 * DEG)
    assert sorted(got) == [2.0, 2.1]  # the laser-forward 9 m return is BEHIND the car


def test_window_left_is_positive_with_mount_yaw_pi():
    # Bench result in vehicle_params: a box on the driver's left read at laser bearing -100 deg,
    # which is vehicle bearing +80 degrees.
    scan = make_scan({-100: 1.5, 100: 7.0}, yaw=math.pi)
    assert valid_returns_in_window(scan, 80 * DEG, 2 * DEG) == [1.5]
    assert valid_returns_in_window(scan, -80 * DEG, 2 * DEG) == [7.0]


def test_window_with_yaw_zero_is_the_laser_frame():
    scan = make_scan({0: 3.0, 180: 4.0}, yaw=0.0)
    assert valid_returns_in_window(scan, 0.0, 1 * DEG) == [3.0]


def test_window_straddling_plus_minus_pi_wraps():
    scan = make_scan({178: 1.0, 179: 1.1, -179: 1.2, -178: 1.3}, yaw=0.0)
    got = valid_returns_in_window(scan, math.pi, 2.5 * DEG)
    assert sorted(got) == [1.0, 1.1, 1.2, 1.3]


def test_window_boundary_is_inclusive_and_outside_is_excluded():
    scan = make_scan({10: 2.0, 11: 3.0, 12: 4.0})
    assert valid_returns_in_window(scan, 10 * DEG, 1 * DEG + 1e-9) == [2.0, 3.0]
    assert valid_returns_in_window(scan, 10 * DEG, 0.5 * DEG) == [2.0]


def test_window_narrower_than_a_beam_still_selects_the_nearest_beam():
    scan = make_scan({10: 2.0, 11: 3.0})
    assert valid_returns_in_window(scan, 10.2 * DEG, 0.0) == [2.0]


def test_window_drops_invalid_returns():
    scan = make_scan(
        {10: INF, 11: math.nan, 12: 0.01, 13: 13.0, 14: -1.0, 15: 0.05, 16: 12.0, 17: 2.0}
    )
    got = valid_returns_in_window(scan, 13.5 * DEG, 4 * DEG)
    # 0.05 and 12.0 sit exactly on the band edges and count; 0.01 / 13.0 / -1.0 do not.
    assert sorted(got) == [0.05, 2.0, 12.0]


def test_window_on_empty_scan():
    scan = ScanView(0.0, -math.pi, DEG, 0.05, 12.0, [], 0.0)
    assert valid_returns_in_window(scan, 0.0, 1.0) == []


# --------------------------------------------------------------- robust range


def test_robust_range_empty_is_none():
    assert robust_range_m([]) is None


def test_robust_range_single_value():
    assert robust_range_m([2.5]) == 2.5


def test_robust_range_median_of_closest_30_percent_odd_count():
    values = [5.0, 1.0, 9.0, 1.2, 8.0, 1.1, 7.0, 6.0, 4.0, 3.0]
    # 10 returns -> ceil(3.0) = 3 closest: 1.0, 1.1, 1.2 -> median 1.1
    assert robust_range_m(values) == pytest.approx(1.1)


def test_robust_range_even_count_averages_the_middle_pair():
    values = [1.0, 2.0, 3.0, 4.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0]
    # 13 returns -> ceil(3.9) = 4 closest: 1, 2, 3, 4 -> (2 + 3) / 2
    assert robust_range_m(values) == pytest.approx(2.5)


def test_robust_range_ignores_far_background_behind_the_object():
    person = [2.0, 2.05, 1.95, 2.1, 1.9]
    wall = [6.0] * 12
    assert robust_range_m(person + wall) == pytest.approx(2.0, abs=0.1)


def test_robust_range_always_keeps_at_least_one():
    assert robust_range_m([3.0, 1.0], closest_fraction=0.01) == 1.0


def test_robust_range_fraction_one_is_the_plain_median():
    assert robust_range_m([1.0, 2.0, 9.0], closest_fraction=1.0) == 2.0


@pytest.mark.parametrize("fraction", [0.0, -0.1, 1.5, math.nan])
def test_robust_range_rejects_bad_fraction(fraction):
    with pytest.raises(ValueError):
        robust_range_m([1.0], fraction)


# ------------------------------------------------------------- fuse_detection


def full_ring(range_m: float, **kw) -> ScanView:
    return make_scan({d: range_m for d in range(-180, 180)}, **kw)


def test_fuse_straight_ahead_with_real_mount_yaw():
    # An object 2 m in front of the nose: laser bearings near +/-180 at 2 m, 6 m elsewhere.
    returns = {d: 6.0 for d in range(-180, 180)}
    for d in list(range(170, 180)) + list(range(-180, -169)):
        returns[d] = 2.0
    scan = make_scan(returns, yaw=math.pi)
    out = fuse_detection(det(cx=640.0, width=200.0), 100.0, CAM, scan, max_scan_age_s=0.5)
    assert out.range_status == STATUS_OK and out.range_valid
    assert out.range_m == pytest.approx(2.0)
    assert out.bearing_rad == pytest.approx(0.0, abs=1e-12)
    assert out.range_samples >= 7


def test_fuse_object_on_the_left_gets_positive_bearing_and_its_own_range():
    # Left half of the image -> positive bearing. Put a 1.2 m object at vehicle +30 deg,
    # i.e. laser bearing 30 - 180 = -150 deg with yaw pi.
    f = 640.0 / math.tan(HFOV / 2)
    u = 640.0 - f * math.tan(30 * DEG)
    returns = {d: 5.0 for d in range(-180, 180)}
    for d in range(-155, -144):
        returns[d] = 1.2
    scan = make_scan(returns, yaw=math.pi)
    out = fuse_detection(det(cx=u, width=120.0), 100.0, CAM, scan, 0.5)
    assert out.bearing_rad == pytest.approx(30 * DEG, abs=1e-9)
    assert out.range_m == pytest.approx(1.2)


def test_fuse_no_scan_still_gives_bearing_and_nan_range():
    out = fuse_detection(det(cx=300.0), 100.0, CAM, None, 0.5)
    assert out.range_status == STATUS_NO_SCAN
    assert not out.range_valid
    assert math.isnan(out.range_m)
    assert out.range_samples == 0
    assert out.bearing_rad > 0.0


def test_fuse_stale_scan_in_either_direction():
    scan = full_ring(3.0, stamp=100.0)
    older = fuse_detection(det(), 100.6, CAM, scan, 0.5)
    newer = fuse_detection(det(), 99.4, CAM, scan, 0.5)
    assert older.range_status == newer.range_status == STATUS_STALE_SCAN
    assert math.isnan(older.range_m) and math.isnan(newer.range_m)


def test_fuse_scan_exactly_at_the_age_limit_is_fresh():
    scan = full_ring(3.0, stamp=100.0)
    assert fuse_detection(det(), 100.5, CAM, scan, 0.5).range_status == STATUS_OK


def test_fuse_scan_with_nan_stamp_is_stale():
    scan = full_ring(3.0, stamp=math.nan)
    assert fuse_detection(det(), 100.0, CAM, scan, 0.5).range_status == STATUS_STALE_SCAN


def test_fuse_no_returns_in_window_flags_it():
    scan = make_scan({}, yaw=math.pi)  # all inf
    out = fuse_detection(det(), 100.0, CAM, scan, 0.5)
    assert out.range_status == STATUS_NO_RETURNS
    assert not out.range_valid and math.isnan(out.range_m)
    assert out.range_samples == 0


def test_fuse_returns_only_outside_the_window_flags_no_returns():
    scan = make_scan({0: 3.0}, yaw=math.pi)  # laser-forward = behind the car
    out = fuse_detection(det(cx=640.0, width=100.0), 100.0, CAM, scan, 0.5)
    assert out.range_status == STATUS_NO_RETURNS


def test_fuse_carries_class_score_and_box_through():
    out = fuse_detection(det(cx=700.0, width=80.0, name="chair", score=0.42), 7.5, CAM, None, 0.5)
    assert (out.class_name, out.score, out.stamp_s) == ("chair", 0.42, 7.5)
    assert (out.cx_px, out.cy_px, out.width_px, out.height_px) == (700.0, 360.0, 80.0, 300.0)


def test_module_status_strings_match_the_message_documentation():
    assert {STATUS_OK, STATUS_NO_SCAN, STATUS_STALE_SCAN, STATUS_NO_RETURNS} == {
        "ok",
        "no_scan",
        "stale_scan",
        "no_returns",
    }
    assert fusion.FusedObject  # exported
