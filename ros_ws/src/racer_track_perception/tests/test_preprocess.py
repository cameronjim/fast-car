from __future__ import annotations

import dataclasses
import math

import numpy as np
from racer_track_perception import preprocess as pp
from racer_track_perception.vehicle import LiveVehicle

RANGE_MIN = 0.05
RANGE_MAX = 12.0


def tensor(live: LiveVehicle, ranges, angle_min, inc=0.01, mount=None):
    return pp.scan_to_tensor(
        ranges, angle_min, inc, RANGE_MIN, RANGE_MAX, mount or live.mount, live.body
    )


def test_one_beam_straight_ahead_is_bin_180(live: LiveVehicle) -> None:
    # Vehicle bearing 0 is laser bearing -mount_yaw (pi for the live car).
    t = tensor(live, [1.0], -live.mount.yaw_rad)
    assert t.shape == (1, 2, 360)
    assert t.dtype == np.float32
    assert np.flatnonzero(t[0, 1]).tolist() == [180]
    assert abs(t[0, 0, 180] - 1.0 / 6.0) < 1e-6


def test_straight_ahead_with_live_yaw_pi_exactly(live: LiveVehicle) -> None:
    assert abs(live.mount.yaw_rad - math.pi) < 1e-5  # the contract's example mount
    t = tensor(live, [1.0], math.pi)
    assert np.flatnonzero(t[0, 1]).tolist() == [180]


def test_beam_to_the_left_is_above_180(live: LiveVehicle) -> None:
    # Vehicle bearing +90 degrees (left): laser bearing = pi/2 - mount_yaw.
    t = tensor(live, [1.0], math.pi / 2 - live.mount.yaw_rad)
    assert np.flatnonzero(t[0, 1]).tolist() == [270]
    t = tensor(live, [1.0], -math.pi / 2 - live.mount.yaw_rad)
    assert np.flatnonzero(t[0, 1]).tolist() == [90]


def test_behind_is_bin_0_and_359_edges(live: LiveVehicle) -> None:
    mount = dataclasses.replace(live.mount, yaw_rad=0.0)
    # Vehicle bearing exactly -pi lands in bin 0; just under +pi in bin 359.
    assert np.flatnonzero(tensor(live, [1.0], -math.pi, mount=mount)[0, 1]).tolist() == [0]
    last = math.pi - 1e-6
    assert np.flatnonzero(tensor(live, [1.0], last, mount=mount)[0, 1]).tolist() == [359]
    # +pi wraps to -pi.
    assert np.flatnonzero(tensor(live, [1.0], math.pi, mount=mount)[0, 1]).tolist() == [0]


def test_bin_edge_belongs_to_the_upper_bin(live: LiveVehicle) -> None:
    mount = dataclasses.replace(live.mount, yaw_rad=0.0)
    width = 2 * math.pi / 360
    inside = tensor(live, [1.0], 10 * width + 1e-4, mount=mount)
    assert np.flatnonzero(inside[0, 1]).tolist() == [190]
    below = tensor(live, [1.0], 10 * width - 1e-4, mount=mount)
    assert np.flatnonzero(below[0, 1]).tolist() == [189]


def test_min_pooling_and_clip(live: LiveVehicle) -> None:
    mount = dataclasses.replace(live.mount, yaw_rad=0.0)
    # Three beams in one bin (spacing 1e-4 rad): the nearest valid wins.
    t = tensor(live, [3.0, 1.5, 2.0], 0.002, inc=1e-4, mount=mount)
    assert np.flatnonzero(t[0, 1]).tolist() == [180]
    assert abs(t[0, 0, 180] - 1.5 / 6.0) < 1e-6
    # A far valid beam is clipped to 1.0 but still counts as valid.
    far = tensor(live, [9.0], 0.002, mount=mount)
    assert far[0, 0, 180] == 1.0 and far[0, 1, 180] == 1.0


def test_no_return_bins_are_one_and_zero(live: LiveVehicle) -> None:
    t = tensor(live, [1.0], 0.002, mount=dataclasses.replace(live.mount, yaw_rad=0.0))
    empty = np.ones(360, dtype=bool)
    empty[180] = False
    assert np.all(t[0, 0][empty] == 1.0)
    assert np.all(t[0, 1][empty] == 0.0)


def test_invalid_beams_are_not_returns(live: LiveVehicle) -> None:
    mount = dataclasses.replace(live.mount, yaw_rad=0.0)
    ranges = [math.nan, math.inf, -1.0, 0.0, RANGE_MIN / 2, RANGE_MAX * 2]
    t = tensor(live, ranges, 0.0, inc=0.2, mount=mount)
    assert t[0, 1].sum() == 0.0


def test_body_filter_drops_beams_on_the_body(live: LiveVehicle) -> None:
    mount = dataclasses.replace(live.mount, yaw_rad=0.0)
    # A beam from the head straight back at 0.1 m ends at x = mount_x - 0.1, inside the body.
    inside = tensor(live, [0.1], math.pi, mount=mount)
    assert inside[0, 1].sum() == 0.0
    pts = pp.scan_to_points([0.1], math.pi, 0.01, RANGE_MIN, RANGE_MAX, mount, live.body)
    assert pts.shape == (0, 2)
    # Beyond the rear bumper plus margin it is kept.
    reach = live.mount.x_m + live.body.rear_overhang_m + pp.BODY_FILTER_MARGIN_M + 0.01
    outside = tensor(live, [reach], math.pi, mount=mount)
    assert outside[0, 1].sum() == 1.0
    just_inside = live.mount.x_m + live.body.rear_overhang_m + pp.BODY_FILTER_MARGIN_M - 0.01
    assert tensor(live, [just_inside], math.pi, mount=mount)[0, 1].sum() == 0.0


def test_ranges_are_from_the_head_not_base_link(live: LiveVehicle) -> None:
    mount = dataclasses.replace(live.mount, yaw_rad=0.0)
    t = tensor(live, [2.0], 0.002, mount=mount)
    assert abs(t[0, 0, 180] - 2.0 / 6.0) < 1e-6  # not (2 + mount_x) / 6
    p = pp.scan_to_points([2.0], 0.0, 0.01, RANGE_MIN, RANGE_MAX, mount, live.body)
    assert abs(p[0, 0] - (mount.x_m + 2.0)) < 1e-5
    assert abs(p[0, 1] - mount.y_m) < 1e-6


def test_points_use_vehicle_bearing(live: LiveVehicle) -> None:
    # Left of the car (vehicle +90 degrees) is +y in base_link.
    p = pp.scan_to_points(
        [1.0], math.pi / 2 - live.mount.yaw_rad, 0.01, RANGE_MIN, RANGE_MAX, live.mount, live.body
    )
    assert abs(p[0, 0] - live.mount.x_m) < 1e-5
    assert abs(p[0, 1] - (live.mount.y_m + 1.0)) < 1e-5


def test_degenerate_inputs_do_not_raise(live: LiveVehicle) -> None:
    for ranges in ([], [math.nan] * 5, np.zeros(0), None, "abc", [[1.0, 2.0]]):
        t = tensor(live, ranges, 0.0)
        assert t.shape == (1, 2, 360) and np.isfinite(t).all()
    assert pp.scan_to_points([1.0], math.nan, 0.1, 0.0, 1.0, live.mount, live.body).shape == (0, 2)
    assert pp.scan_to_points([1.0], 0.0, 0.1, "x", 1.0, live.mount, live.body).shape == (0, 2)
