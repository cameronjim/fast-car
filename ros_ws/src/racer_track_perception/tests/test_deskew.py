"""L1 (hand cases and a round trip against an independent generator) and L2 (properties) of the
scan deskew core, racer_track_perception.deskew.

The round trip is the test that matters: deskew_sim.py casts a ray per beam from the pose the
car has at that beam's time (a closed form of its own, nothing shared with the module), the core
deskews the result, and the wall points are judged against the walls from where the LiDAR really
is at the reference time.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest
from deskew_sim import ROOM, reference_truth_distance, sweep
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st
from racer_track_perception import deskew
from racer_track_perception.deskew import DeskewConfig, Twist
from racer_track_perception.vehicle import LiveVehicle

N = 720
INC = 2.0 * math.pi / 719.0
DT = 1.0e-4 * 10 / 7.2  # time_increment of the generator, about 0.1 s over 720 beams
SWEEP_S = DT * (N - 1)
WALL_TOLERANCE_M = 0.02  # brief 33, L1 round trip
# The wrong stamp_is must give an error at least this many times the right one's (and not less
# than 5 cm; measured: 25 to 45 times): this is what lets step A tell the two apart.
WRONG_STAMP_FACTOR = 10.0


def cfg(stamp_is: str = "sweep_start", order: str = "index_ascending", **kw) -> DeskewConfig:
    return DeskewConfig(stamp_is=stamp_is, beam_order=order, **kw)


def origin_mount(live: LiveVehicle):
    return dataclasses.replace(live.mount, x_m=0.0, y_m=0.0, yaw_rad=0.0)


# -- config and timing ---------------------------------------------------------------------------


def test_config_refuses_unknown_values() -> None:
    with pytest.raises(ValueError, match="stamp_is"):
        DeskewConfig(stamp_is="middle", beam_order="index_ascending")
    with pytest.raises(ValueError, match="beam_order"):
        DeskewConfig(stamp_is="sweep_start", beam_order="up")
    with pytest.raises(ValueError, match="reference"):
        cfg(reference="whenever")
    with pytest.raises(ValueError, match="max_twist_age_s"):
        cfg(max_twist_age_s=0.0)


def test_stamp_is_and_beam_order_have_no_default() -> None:
    with pytest.raises(TypeError):
        DeskewConfig()  # type: ignore[call-arg]
    with pytest.raises(TypeError):
        DeskewConfig(stamp_is="sweep_start")  # type: ignore[call-arg]


def test_beam_times_use_the_messages_own_fields() -> None:
    t = deskew.beam_times(5, 0.5, 0.01)
    assert np.allclose(t, [0.0, 0.01, 0.02, 0.03, 0.04])  # time_increment wins
    t = deskew.beam_times(5, 0.5, 0.0)
    assert np.allclose(t, np.arange(5) * 0.1)  # scan_time / n
    assert np.all(deskew.beam_times(5, 0.0, 0.0) == 0.0)
    assert np.all(deskew.beam_times(5, math.nan, math.nan) == 0.0)
    assert deskew.beam_times(0, 0.1, 0.0).size == 0


@pytest.mark.parametrize(
    ("stamp_is", "reference", "expected"),
    [
        ("sweep_start", "sweep_end", 0.1),
        ("sweep_start", "sweep_mid", 0.05),
        ("sweep_start", "sweep_start", 0.0),
        ("sweep_end", "sweep_end", 0.0),
        ("sweep_end", "sweep_mid", -0.05),
        ("sweep_end", "sweep_start", -0.1),
    ],
)
def test_stamp_offset_for_both_stamp_meanings(stamp_is, reference, expected) -> None:
    t = np.linspace(0.0, 0.1, 11)
    assert deskew.stamp_offset(t, cfg(stamp_is, reference=reference)) == pytest.approx(expected)


# -- deskew_points hand cases ---------------------------------------------------------------------


def test_standstill_returns_the_input_exactly(live: LiveVehicle) -> None:
    ranges = sweep(live.mount, 0.0, 0.0)
    out, offset = deskew.deskew_scan(
        ranges,
        -math.pi,
        INC,
        SWEEP_S,
        DT,
        Twist(0.0, 0.0),
        live.mount,
        cfg(),
        range_min=0.05,
        range_max=16.0,
    )
    assert np.array_equal(out, ranges)
    assert offset == pytest.approx(SWEEP_S, rel=1e-6)


def test_pure_forward_motion_shifts_points_by_speed_times_time_to_the_reference(live) -> None:
    mount = origin_mount(live)
    n, v = 11, 1.2
    t = np.linspace(0.0, 0.1, n)
    # beams all pointing straight ahead at a wall 3 m away, seen at their own times
    out = deskew.deskew_points(np.full(n, 3.0), 0.0, 0.0, t, Twist(v, 0.0), mount, cfg())
    assert np.allclose(out[:, 1], 0.0, atol=1e-12)
    assert np.allclose(out[:, 0], 3.0 - v * (0.1 - t), atol=1e-12)


def test_reference_sweep_start_moves_points_the_other_way(live) -> None:
    mount = origin_mount(live)
    t = np.linspace(0.0, 0.1, 11)
    out = deskew.deskew_points(
        np.full(11, 3.0), 0.0, 0.0, t, Twist(1.0, 0.0), mount, cfg(reference="sweep_start")
    )
    assert np.allclose(out[:, 0], 3.0 + 1.0 * t, atol=1e-12)


def test_pure_rotation_keeps_a_circle_a_circle_and_shifts_bearings(live) -> None:
    mount = origin_mount(live)
    n, w, radius = 90, 1.5, 2.0
    a = np.arange(n) * (2 * math.pi / n)
    t = np.linspace(0.0, 0.1, n)
    out = deskew.deskew_points(
        np.full(n, radius), 0.0, 2 * math.pi / n, t, Twist(0.0, w), mount, cfg()
    )
    assert np.allclose(np.hypot(out[:, 0], out[:, 1]), radius, atol=1e-12)
    bearing = np.arctan2(out[:, 1], out[:, 0])
    expected = np.angle(np.exp(1j * (a - w * (0.1 - t))))
    assert np.allclose(np.angle(np.exp(1j * (bearing - expected))), 0.0, atol=1e-12)


def test_the_arc_is_exact_not_a_small_angle_approximation(live) -> None:
    mount = origin_mount(live)
    v, w, dt = 1.5, 2.0, 0.1
    out = deskew.deskew_points(
        np.array([2.0]),
        0.0,
        0.0,
        np.array([0.0, 0.0])[:1],
        Twist(v, w),
        mount,
        cfg(reference="sweep_end"),
    )
    # one beam: reference time is its own time, so it does not move
    assert np.allclose(out, [[2.0, 0.0]])
    # two beams: the first is carried over dt = 0.1 s
    t = np.array([0.0, dt])
    out = deskew.deskew_points(np.array([2.0, 2.0]), 0.0, 0.0, t, Twist(v, w), mount, cfg())
    th = w * dt
    tx, ty = v / w * math.sin(th), v / w * (1.0 - math.cos(th))
    c, s = math.cos(th), math.sin(th)
    want = [c * (2.0 - tx) + s * (0.0 - ty), -s * (2.0 - tx) + c * (0.0 - ty)]
    assert np.allclose(out[0], want, atol=1e-12)
    assert np.allclose(out[1], [2.0, 0.0], atol=1e-12)
    # the chord of a small-angle approximation (translation v dt, rotation w dt) is not this
    assert not np.allclose(out[0], [c * (2.0 - v * dt), -s * (2.0 - v * dt)], atol=1e-4)


def test_mount_yaw_of_pi_and_the_x_offset(live) -> None:
    mount = dataclasses.replace(live.mount, x_m=0.285, y_m=0.0, yaw_rad=math.pi)
    # laser bearing 0 (straight out of the head) points to the REAR of the car: base (0.285 - r, 0)
    out = deskew.deskew_points(
        np.array([1.0]), 0.0, 0.0, np.array([0.0]), Twist(0.0, 0.0), mount, cfg()
    )
    assert np.allclose(out, [[0.285 - 1.0, 0.0]], atol=1e-12)
    # laser bearing pi/2 is to the car's RIGHT with this mount (vehicle bearing 3 pi/2)
    out = deskew.deskew_points(
        np.array([1.0]), math.pi / 2, 0.0, np.array([0.0]), Twist(0.0, 0.0), mount, cfg()
    )
    assert np.allclose(out, [[0.285, -1.0]], atol=1e-12)


def test_the_yaw_rotates_about_the_rear_axle_not_the_lidar(live) -> None:
    mount = dataclasses.replace(live.mount, x_m=0.285, y_m=0.0, yaw_rad=0.0)
    w, dt = 2.0, 0.1
    t = np.array([0.0, dt])
    # the beam at t=0 sees a point at the LiDAR's own position (range 1e-3 straight ahead)
    out = deskew.deskew_points(np.array([1e-3, 1e-3]), 0.0, 0.0, t, Twist(0.0, w), mount, cfg())
    th = w * dt
    # base (0.286, 0) rotated by -th about the origin, since v = 0
    assert np.allclose(out[0], [0.286 * math.cos(th), -0.286 * math.sin(th)], atol=1e-9)


def test_invalid_ranges_stay_invalid(live) -> None:
    r = np.array([1.0, np.nan, np.inf, -1.0, 0.0, 2.0])
    t = np.linspace(0.0, 0.1, 6)
    out = deskew.deskew_points(r, 0.0, 0.1, t, Twist(1.0, 0.5), live.mount, cfg())
    assert np.isfinite(out[[0, 5]]).all()
    assert np.isnan(out[[1, 2, 3, 4]]).all()


def test_invalid_input_stays_inf_after_deskew_scan(live) -> None:
    r = sweep(live.mount, 1.0, 0.5)
    r[::7] = np.nan
    r[3::11] = -1.0
    out, _ = deskew.deskew_scan(
        r,
        -math.pi,
        INC,
        SWEEP_S,
        DT,
        Twist(1.0, 0.5),
        live.mount,
        cfg(),
        range_min=0.05,
        range_max=16.0,
    )
    assert not np.isnan(out).any() and (out > 0).all()


def test_a_twist_that_is_not_finite_is_refused(live) -> None:
    with pytest.raises(ValueError, match="finite"):
        deskew.deskew_scan(np.ones(4), 0.0, 0.1, 0.1, 0.0, Twist(math.nan, 0.0), live.mount, cfg())


def test_empty_and_untimed_scans(live) -> None:
    out, offset = deskew.deskew_scan(
        np.zeros(0), 0.0, 0.1, 0.1, 0.0, Twist(1.0, 1.0), live.mount, cfg()
    )
    assert out.size == 0 and offset == 0.0
    r = np.array([1.0, 2.0, np.inf])
    out, offset = deskew.deskew_scan(r, 0.0, 0.1, 0.0, 0.0, Twist(1.0, 1.0), live.mount, cfg())
    assert np.array_equal(out, r) and offset == 0.0  # no timing: nothing to compensate


# -- rebin ------------------------------------------------------------------------------------------


def test_rebin_takes_the_nearest_return_per_bin(live) -> None:
    mount = origin_mount(live)
    # three points in the same 0.1 rad bin of bin 5 (angle 0.5): ranges 3, 1, 2; bearing +-0.02
    b = 0.5
    pts = np.array(
        [
            [3 * math.cos(b), 3 * math.sin(b)],
            [1 * math.cos(b + 0.02), 1 * math.sin(b + 0.02)],
            [2 * math.cos(b - 0.02), 2 * math.sin(b - 0.02)],
        ]
    )
    out = deskew.rebin(pts, mount, 0.0, 0.1, 10, 0.05, 16.0)
    assert out[5] == pytest.approx(1.0)
    assert np.isinf(np.delete(out, [4, 5, 6])).all()


def test_rebin_fills_a_one_bin_hole_with_the_smaller_neighbour_and_only_that(live) -> None:
    mount = origin_mount(live)
    inc = 0.1

    def point(bin_: int, r: float):
        return [r * math.cos(bin_ * inc), r * math.sin(bin_ * inc)]

    pts = np.array([point(2, 3.0), point(4, 1.5), point(8, 2.0), point(11, 2.0)])
    out = deskew.rebin(pts, mount, 0.0, inc, 14, 0.05, 16.0)
    assert out[3] == pytest.approx(1.5)  # the hole between 3.0 and 1.5 takes 1.5
    assert np.isinf(out[[5, 6, 7, 9, 10]]).all()  # holes of one bin only: 5 6 7 and 9 10 stay
    assert out[2] == pytest.approx(3.0) and out[4] == pytest.approx(1.5)


def test_rebin_drops_points_out_of_range_and_nan(live) -> None:
    mount = origin_mount(live)
    pts = np.array([[0.01, 0.0], [20.0, 0.0], [np.nan, np.nan], [2.0, 0.0]])
    out = deskew.rebin(pts, mount, 0.0, 0.1, 6, 0.05, 16.0)
    assert out[0] == pytest.approx(2.0) and np.isinf(out[1:]).all()


def test_rebin_wraps_the_bearing_for_a_negative_increment(live) -> None:
    mount = origin_mount(live)
    inc = -0.1
    pts = np.array([[math.cos(-0.3), math.sin(-0.3)]])  # bearing -0.3 = bin 3 of a -0.1 grid
    out = deskew.rebin(pts, mount, 0.0, inc, 20, 0.05, 16.0)
    assert out[3] == pytest.approx(1.0) and np.isinf(np.delete(out, 3)).all()


# -- round trip against the independent generator ------------------------------------------------------

MOTIONS = [(0.5, 0.3), (1.0, 1.0), (1.5, 2.0), (1.5, -2.0), (1.0, 0.0), (-0.6, 0.8)]


def _round_trip(live, v, w, stamp_is, order, inc_sign=1, claim_stamp_is=None, claim_order=None):
    """(max error to the true walls, claimed reference time check) of a deskewed generated sweep.
    The truth is the sweep recorded with the REAL (stamp_is, order); the config under test may
    claim another."""
    inc = INC * inc_sign
    a_min = -math.pi if inc_sign > 0 else math.pi
    t_start = 5.0
    ranges = sweep(
        live.mount, v, w, angle_min=a_min, angle_increment=inc, time_order=order, t_start=t_start
    )
    stamp = t_start if stamp_is == "sweep_start" else t_start + SWEEP_S
    config = cfg(claim_stamp_is or stamp_is, claim_order or order)
    out, offset = deskew.deskew_scan(
        ranges,
        a_min,
        inc,
        SWEEP_S,
        DT,
        Twist(v, w),
        live.mount,
        config,
        range_min=0.05,
        range_max=16.0,
    )
    dist = reference_truth_distance(
        out, live.mount, v, w, stamp + offset, a_min, inc, ROOM, t_start=t_start
    )
    return dist.max() if dist.size else math.nan, int(np.isfinite(out).sum())


@pytest.mark.parametrize("order", ["index_ascending", "index_descending"])
@pytest.mark.parametrize("stamp_is", ["sweep_start", "sweep_end"])
@pytest.mark.parametrize(("v", "w"), MOTIONS)
def test_round_trip_wall_points_land_within_2_cm_of_the_true_walls(
    live, v, w, stamp_is, order
) -> None:
    err, n_finite = _round_trip(live, v, w, stamp_is, order)
    assert n_finite > 600
    assert err <= WALL_TOLERANCE_M, (v, w, stamp_is, order, err)


@pytest.mark.parametrize(("v", "w"), [(1.0, 1.0), (1.5, -2.0)])
def test_round_trip_with_a_negative_angle_increment(live, v, w) -> None:
    err, n_finite = _round_trip(live, v, w, "sweep_start", "index_ascending", inc_sign=-1)
    assert n_finite > 600 and err <= WALL_TOLERANCE_M, err


@pytest.mark.parametrize(("v", "w"), [(1.0, 1.0), (1.5, 2.0), (1.5, -2.0)])
@pytest.mark.parametrize("stamp_is", ["sweep_start", "sweep_end"])
def test_the_wrong_stamp_is_is_worse_by_a_stated_factor(live, v, w, stamp_is) -> None:
    wrong = "sweep_end" if stamp_is == "sweep_start" else "sweep_start"
    right_err, _ = _round_trip(live, v, w, stamp_is, "index_ascending")
    wrong_err, _ = _round_trip(live, v, w, stamp_is, "index_ascending", claim_stamp_is=wrong)
    assert right_err <= WALL_TOLERANCE_M
    assert wrong_err >= 0.05
    assert wrong_err >= WRONG_STAMP_FACTOR * right_err, (right_err, wrong_err)


@pytest.mark.parametrize(("v", "w"), [(1.0, 1.0), (1.5, 2.0), (1.5, -2.0)])
def test_the_wrong_beam_order_is_worse_than_the_right_one(live, v, w) -> None:
    right_err, _ = _round_trip(live, v, w, "sweep_start", "index_descending")
    wrong_err, _ = _round_trip(
        live, v, w, "sweep_start", "index_descending", claim_order="index_ascending"
    )
    assert right_err <= WALL_TOLERANCE_M
    assert wrong_err >= WRONG_STAMP_FACTOR * right_err and wrong_err >= 0.03


def test_raw_is_worse_than_deskewed_in_a_fast_corner(live) -> None:
    v, w = 1.0, 1.5
    ranges = sweep(live.mount, v, w)
    raw = reference_truth_distance(ranges, live.mount, v, w, SWEEP_S, -math.pi, INC, ROOM)
    out, _ = deskew.deskew_scan(
        ranges,
        -math.pi,
        INC,
        SWEEP_S,
        DT,
        Twist(v, w),
        live.mount,
        cfg(),
        range_min=0.05,
        range_max=16.0,
    )
    deskewed = reference_truth_distance(out, live.mount, v, w, SWEEP_S, -math.pi, INC, ROOM)
    assert deskewed.max() <= WALL_TOLERANCE_M
    assert raw.max() > 4 * deskewed.max()


# -- the car's own parts -------------------------------------------------------------------------------


def test_body_beams_flags_returns_from_inside_the_body_rectangle(live) -> None:
    n = 4
    # the mount looks backwards (yaw pi): laser bearing 0 is the rear, 0.285 m from the rear
    # axle's x-forward point, so range 0.17 lands at x = 0.115, inside the body
    r = np.array([0.17, 3.0, np.nan, 0.17])
    flags = deskew.body_beams(r, 0.0, 0.0, live.mount, live.body)
    assert flags.tolist() == [True, False, False, True]
    assert n == len(flags)


def test_static_beams_are_not_carried(live) -> None:
    r = np.array([0.17, 3.0])
    t = np.array([0.0, 0.1])
    static = np.array([True, False])
    out = deskew.deskew_points(r, 0.0, 0.0, t, Twist(1.0, 1.0), live.mount, cfg(), static)
    body_point = [live.mount.x_m - 0.17, 0.0]
    assert np.allclose(out[0], body_point, atol=1e-6)  # the yaml holds pi to 7 digits
    free = deskew.deskew_points(r, 0.0, 0.0, t, Twist(1.0, 1.0), live.mount, cfg())
    assert not np.allclose(free[0], body_point, atol=1e-3)


def test_static_returns_survive_a_deskew_scan_on_their_own_bins(live) -> None:
    r = sweep(live.mount, 1.0, 1.0)
    r[343:353] = 0.17  # the car's own parts, as on the real car
    static = deskew.body_beams(r, -math.pi, INC, live.mount, live.body)
    assert static.sum() >= 10
    out, _ = deskew.deskew_scan(
        r,
        -math.pi,
        INC,
        SWEEP_S,
        DT,
        Twist(1.0, 1.0),
        live.mount,
        cfg(),
        range_min=0.05,
        range_max=16.0,
        static=static,
    )
    assert np.allclose(out[343:353], 0.17, atol=1e-6)


# -- L2: properties ---------------------------------------------------------------------------------------

range_values = st.one_of(
    st.floats(min_value=-5.0, max_value=40.0),
    st.just(math.nan),
    st.just(math.inf),
    st.just(-math.inf),
    st.just(0.0),
)


@settings(max_examples=200, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(
    ranges=st.lists(range_values, min_size=0, max_size=60),
    v=st.floats(min_value=-3.0, max_value=3.0),
    w=st.floats(min_value=-6.0, max_value=6.0),
    angle_min=st.floats(min_value=-4.0, max_value=4.0),
    inc=st.one_of(
        st.floats(min_value=-0.2, max_value=-1e-3), st.floats(min_value=1e-3, max_value=0.2)
    ),
    scan_time=st.one_of(st.floats(min_value=0.0, max_value=0.3), st.just(math.nan)),
    stamp_is=st.sampled_from(deskew.STAMP_IS_VALUES),
    order=st.sampled_from(deskew.BEAM_ORDERS),
    reference=st.sampled_from(deskew.REFERENCES),
)
def test_any_ranges_and_any_twist_give_a_well_formed_output(
    live, ranges, v, w, angle_min, inc, scan_time, stamp_is, order, reference
) -> None:
    range_min, range_max = 0.05, 16.0
    out, offset = deskew.deskew_scan(
        np.array(ranges, dtype=np.float64),
        angle_min,
        inc,
        scan_time,
        0.0,
        Twist(v, w),
        live.mount,
        cfg(stamp_is, order, reference=reference),
        range_min=range_min,
        range_max=range_max,
    )
    assert out.shape == (len(ranges),)
    assert not np.isnan(out).any()
    finite = out[np.isfinite(out)]
    assert (finite >= range_min).all() and (finite <= range_max).all()
    assert math.isfinite(offset)


@settings(max_examples=100, deadline=None)
@given(
    values=st.lists(st.floats(min_value=0.1, max_value=10.0), min_size=1, max_size=50),
    v=st.floats(min_value=-3.0, max_value=3.0),
    w=st.floats(min_value=-6.0, max_value=6.0),
)
def test_a_zero_sweep_time_never_changes_the_scan(live, values, v, w) -> None:
    r = np.array(values)
    out, offset = deskew.deskew_scan(r, 0.0, 0.05, 0.0, 0.0, Twist(v, w), live.mount, cfg())
    assert np.array_equal(out, r) and offset == 0.0
