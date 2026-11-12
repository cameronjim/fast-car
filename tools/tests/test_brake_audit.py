"""L1 tests for tools/safety_replay/counterfactual.py and brake_audit.py (the brake audit of
docs/notes/safety-race-retune-2026-11-12.md): synthetic walls and known dynamics, so every number
the audit reports from a bag has a hand-computed counterpart here.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from follower_replay.bag import DriveRecord, EventRecord, OdomRecord, ScanRecord
from safety_replay import brake_audit as ba
from safety_replay.counterfactual import (
    BodyBox,
    Plant,
    body_clearance_m,
    simulate,
    swept_distance_m,
)
from safety_replay.motion import Mount

WHEELBASE = 0.3302
BODY = BodyBox(WHEELBASE + 0.11, 0.14, 0.155)  # front 0.4402 m from the rear axle


def wall(x: float, y_lo: float = -1.0, y_hi: float = 1.0, n: int = 80) -> np.ndarray:
    """A wall across the path at x (rear-axle frame), as scan returns."""
    return np.stack([np.full(n, x), np.linspace(y_lo, y_hi, n)], axis=1)


# -- geometry -------------------------------------------------------------------------------------


def test_clearance_is_the_distance_to_the_body_rectangle():
    pts = wall(1.0)
    assert body_clearance_m(pts, 0.0, 0.0, 0.0, BODY) == pytest.approx(1.0 - BODY.front_x_m)
    # Driven forward 0.5 m the gap shrinks by 0.5; turned to face sideways the front is at y.
    assert body_clearance_m(pts, 0.5, 0.0, 0.0, BODY) == pytest.approx(1.0 - 0.5 - BODY.front_x_m)
    assert body_clearance_m(np.zeros((0, 2)), 0.0, 0.0, 0.0, BODY) == math.inf
    # A point on the body is a contact, not a negative distance.
    assert body_clearance_m(np.array([[0.1, 0.0]]), 0.0, 0.0, 0.0, BODY) == 0.0


def test_swept_distance_straight_is_the_gap_to_the_bumper():
    pts = wall(1.0)
    d = swept_distance_m(pts, 0.0, WHEELBASE, BODY, 2.0)
    assert d == pytest.approx(1.0 - BODY.front_x_m, abs=0.011)  # one step of resolution
    assert swept_distance_m(pts, 0.0, WHEELBASE, BODY, 0.3) == math.inf  # beyond the horizon
    # With a margin the contact is that much sooner.
    assert swept_distance_m(pts, 0.0, WHEELBASE, BODY, 2.0, margin_m=0.1) == pytest.approx(
        d - 0.1, abs=0.011
    )


def test_swept_distance_turns_away_from_a_side_wall_and_into_it():
    side = np.stack([np.linspace(0.0, 3.0, 120), np.full(120, 0.45)], axis=1)  # a wall to the left
    straight = swept_distance_m(side, 0.0, WHEELBASE, BODY, 3.0)
    left = swept_distance_m(side, 0.449, WHEELBASE, BODY, 3.0)
    right = swept_distance_m(side, -0.449, WHEELBASE, BODY, 3.0)
    assert straight == math.inf  # the body is 0.155 wide: 0.45 clears it by 0.295
    assert left < 1.0
    assert right == math.inf


# -- the unbraked plan -----------------------------------------------------------------------------


def plant(**kw) -> Plant:
    base = {
        "wheelbase_m": WHEELBASE,
        "speed_delay_s": 0.0,
        "speed_tau_s": 0.05,
        "steer_tau_s": 0.0,
        "max_decel_mps2": 100.0,
        "max_accel_mps2": 100.0,
    }
    base.update(kw)
    return Plant(**base)


def test_simulation_hits_a_wall_the_car_drives_into():
    traj = simulate(wall(1.0), lambda t: 1.0, lambda t: 0.0, 1.0, 0.0, plant(), BODY, horizon_s=1.0)
    assert traj.first_hit_s is not None and traj.min_clearance_m == 0.0
    # 0.56 m to cover at 1 m/s: contact at about 0.56 s.
    assert traj.first_hit_s == pytest.approx(1.0 - BODY.front_x_m, abs=0.03)
    assert traj.travel_m > 0.5


def test_simulation_that_slows_in_time_ends_short_of_the_wall():
    # Commanded to stop at once; first-order lag 0.05 s and v0 1 m/s: about v0 * tau = 0.05 m
    # (0.04 with the explicit step, the speed updated before the position).
    traj = simulate(wall(1.0), lambda t: 0.0, lambda t: 0.0, 1.0, 0.0, plant(), BODY, horizon_s=1.0)
    assert traj.first_hit_s is None
    assert traj.travel_m == pytest.approx(0.05, abs=0.015)
    assert traj.min_clearance_m == pytest.approx(1.0 - BODY.front_x_m - 0.05, abs=0.015)


def test_simulation_delay_keeps_the_old_speed_for_that_long():
    traj = simulate(
        wall(2.0),
        lambda t: 1.0 if t < 0.0 else 0.0,  # the car was being asked for 1.0 until t = 0
        lambda t: 0.0,
        1.0,
        0.0,
        plant(speed_delay_s=0.2),
        BODY,
        horizon_s=0.5,
    )
    assert traj.travel_m == pytest.approx(0.2 + 0.05, abs=0.02)


def test_simulation_steering_follows_the_commanded_arc_and_the_tyre_gain():
    straight = simulate(wall(5.0), lambda t: 1.0, lambda t: 0.0, 1.0, 0.0, plant(), BODY, 1.0)
    left = simulate(wall(5.0), lambda t: 1.0, lambda t: 0.4, 1.0, 0.4, plant(), BODY, 1.0)
    assert abs(straight.y_m[-1]) < 1e-9 and straight.theta_rad[-1] == 0.0
    assert left.y_m[-1] > 0.2 and left.theta_rad[-1] > 0.5  # a left turn, positive yaw
    kappa = math.tan(0.4) / WHEELBASE
    assert left.theta_rad[-1] == pytest.approx(left.travel_m * kappa, rel=0.02)
    weak = simulate(
        wall(5.0),
        lambda t: 1.0,
        lambda t: 0.4,
        1.0,
        0.4,
        plant(curvature_gain=lambda d: 0.5),
        BODY,
        1.0,
    )
    assert weak.theta_rad[-1] == pytest.approx(0.5 * left.theta_rad[-1], rel=0.03)


def test_simulation_speed_gain_scales_the_wheel_speed_to_the_true_speed():
    traj = simulate(
        wall(5.0),
        lambda t: 1.0,
        lambda t: 0.0,
        0.6,
        0.0,
        plant(speed_gain=lambda d: 0.6),
        BODY,
        1.0,
    )
    # Wheel speed 1.0 (v0 / gain), true 0.6: 0.6 m in 1 s.
    assert traj.travel_m == pytest.approx(0.6, abs=0.02)


def test_simulation_rate_limits_hold_the_speed():
    held = simulate(
        wall(5.0),
        lambda t: 0.0,
        lambda t: 0.0,
        1.0,
        0.0,
        plant(max_decel_mps2=0.0, max_accel_mps2=0.0),
        BODY,
        1.0,
    )
    assert held.travel_m == pytest.approx(1.0, abs=0.02)
    assert simulate(
        wall(5.0), lambda t: 0.0, lambda t: 0.0, 1.0, 0.0, plant(speed_tau_s=0.0), BODY, 0.5
    ).travel_m == pytest.approx(0.0, abs=0.02)


# -- engagements and classes -------------------------------------------------------------------------


def event(t, severity, phase, source="ttc", detail="time-to-collision 0.5s <= x"):
    return EventRecord(
        stamp_s=t,
        severity=severity,
        phase=phase,
        duration_s=0.0,
        source=source,
        detail=detail,
        log_time_s=t,
    )


def test_engagements_pair_each_engage_with_its_release_and_name_the_kind():
    events = [
        event(10.0, 0, 0),  # advisory
        event(10.4, 0, 1),
        event(11.0, 2, 0),  # brake
        event(11.3, 2, 1),
        event(12.0, 1, 0, source="rate_limit"),  # not a TTC engagement
        event(13.0, 0, 0, detail="steering held while obstacle-latched: x"),  # not a warning
        event(14.0, 2, 0, source="watchdog"),
        event(15.0, 2, 0),  # a brake still engaged at the end of the bag
    ]
    got = ba.ttc_engagements(events)
    assert [(e.kind, e.t_s) for e in got] == [("advisory", 10.0), ("brake", 11.0), ("brake", 15.0)]
    assert got[0].duration_s == pytest.approx(0.4)
    assert got[1].duration_s == pytest.approx(0.3)
    assert got[2].release_s is None and got[2].duration_s is None


@pytest.mark.parametrize(
    ("clearance", "label"),
    [
        (0.0, "necessary"),
        (0.0399, "necessary"),
        (0.04, "marginal"),
        (0.0999, "marginal"),
        (0.10, "false alarm"),
        (math.inf, "false alarm"),
    ],
)
def test_classification_thresholds_are_the_pre_registered_ones(clearance, label):
    assert ba.NECESSARY_CLEARANCE_M == 0.04 and ba.MARGINAL_CLEARANCE_M == 0.10
    assert ba.classify(clearance) == label


# -- the measured tables -------------------------------------------------------------------------------


def pair_rows(n, v_true, wheel, kappa_gain, steering):
    """Scan-matched pairs of a car at true speed v_true and the steering angle given."""
    rows = []
    kin = math.tan(steering) / WHEELBASE
    for i in range(n):
        t = i * 0.1
        dist = v_true * 0.1
        theta = kappa_gain * kin * dist
        rows.append(
            (t, t + 0.1, dist * math.cos(theta / 2), dist * math.sin(theta / 2), theta, 0.01)
        )
    return np.array(rows), np.full(n, wheel), np.full(n, steering)


def test_slip_table_recovers_known_speed_and_curvature_ratios():
    a, wa, sa = pair_rows(40, 0.8, 1.0, 1.0, 0.05)  # straight: true = 0.8 wheel
    b, wb, sb = pair_rows(40, 0.64, 1.0, 1.04, -0.44)  # full lock: 0.64, curvature 1.04
    pairs = np.vstack([a, b])
    table = ba.build_slip_table(
        pairs, np.concatenate([wa, wb]), np.concatenate([sa, sb]), WHEELBASE
    )
    assert table.speed_gain(0.02) == pytest.approx(0.8, abs=0.01)
    assert table.speed_gain(-0.44) == pytest.approx(0.64, abs=0.01)
    assert table.curvature_gain(-0.44) == pytest.approx(1.04, abs=0.03)
    # Bins with no data take the nearest bin below, then above, then 1.0.
    assert table.speed_gain(0.25) == pytest.approx(0.8, abs=0.01)
    empty = ba.build_slip_table(a[:0].reshape(0, 6), np.zeros(0), np.zeros(0), WHEELBASE)
    assert empty.speed_gain(0.3) == 1.0 and empty.curvature_gain(0.3) == 1.0


def test_fit_speed_plant_recovers_gain_and_acceleration_bound():
    t = np.arange(0.0, 30.0, 0.02)
    cmd = 1.0 + 0.5 * np.sign(np.sin(2 * np.pi * t / 6.0))  # 0.5 / 1.5 steps
    v = np.empty_like(t)
    v[0] = 0.9
    k = 4  # 0.08 s delay
    for i in range(1, len(t)):
        a = (0.9 * cmd[max(i - 1 - k, 0)] - v[i - 1]) / 0.2
        v[i] = v[i - 1] + max(-6.0, min(2.5, a)) * 0.02
    delay, tau, accel, decel, gain = ba.fit_speed_plant(t, cmd, v)
    assert gain == pytest.approx(0.9, abs=0.05)
    assert tau == pytest.approx(0.2, abs=0.1) and delay == pytest.approx(0.08, abs=0.05)
    assert 1.5 <= accel <= 4.0 and decel >= 2.5


# -- stopping ----------------------------------------------------------------------------------------------


def test_stop_response_recovers_latency_and_deceleration():
    t = np.arange(0.0, 3.0, 0.02)
    v = np.where(t < 1.1, 1.0, np.maximum(1.0 - 2.0 * (t - 1.1), 0.0))
    s = ba.stop_response(t, v, t_trip=1.0)
    assert s["v0"] == pytest.approx(1.0)
    assert s["latency_s"] == pytest.approx(0.15, abs=0.03)  # 10 per cent drop: 0.05 s after 1.1
    assert s["mean_decel_mps2"] == pytest.approx(2.0, abs=0.05)
    assert s["peak_decel_mps2"] == pytest.approx(2.0, abs=0.05)
    assert s["v_min"] == pytest.approx(0.0, abs=1e-9)
    # Travel from the trip to the lowest speed: 0.1 s at 1.0 then 0.25 m of ramp.
    assert s["wheel_travel_m"] == pytest.approx(0.35, abs=0.03)


def test_stop_response_with_a_release_and_without_data():
    t = np.arange(0.0, 3.0, 0.02)
    v = np.where(t < 1.1, 1.0, np.where(t < 1.3, 0.6, 1.0))  # a short dip, never to 20 per cent
    s = ba.stop_response(t, v, 1.0)
    assert math.isnan(s["mean_decel_mps2"]) and s["v_min"] == pytest.approx(0.6)
    assert ba.stop_response(t, v, -5.0) == {}
    assert ba.stop_response(t, np.zeros_like(t), 1.0) == {"v0": 0.0}


def test_model_stop_travel_is_latency_times_speed_plus_the_braking_distance():
    assert ba.model_stop_travel_m(1.0, 0.26, 1.0) == pytest.approx(0.26 + 0.5)
    assert ba.model_stop_travel_m(0.0, 0.26, 1.0) == 0.0


# -- one engagement end to end ------------------------------------------------------------------------------

MOUNT = Mount(0.285, 0.0, math.pi)


def make_scan(t_stamp: float, t_arrival: float, wall_x: float) -> ScanRecord:
    """A wall across the path at wall_x (rear-axle frame), seen from the head at x = 0.285 with the
    laser yawed pi: bearing b in the laser frame is the vehicle bearing b + pi."""
    n = 720
    inc = 2 * math.pi / n
    ranges = []
    for k in range(n):
        vehicle_bearing = -math.pi + k * inc + math.pi
        head_x = wall_x - MOUNT.x_m
        c = math.cos(vehicle_bearing)
        ranges.append(head_x / c if c > 0.2 and head_x / c < 12.0 else math.inf)
    return ScanRecord(t_stamp, -math.pi, inc, 0.05, 12.0, tuple(ranges), t_arrival)


def make_streams(wall_x: float, request_speed: float, wheel: float, steering: float = 0.0):
    t0 = 100.0
    scans = [make_scan(t0 - 0.1 + 0.1 * k, t0 + 0.1 * k, wall_x - 0.0) for k in range(-2, 12)]
    raw = [
        DriveRecord(t0 + 0.02 * k, steering, request_speed, t0 + 0.02 * k) for k in range(-100, 200)
    ]
    drive = list(raw)
    odom = [OdomRecord(t0 + 0.02 * k, wheel, t0 + 0.02 * k) for k in range(-100, 200)]
    return t0, ba.Streams.of(scans, raw, drive, odom)


def model(slip=None) -> ba.AuditModel:
    slip = slip or ba.SlipTable(ba.STEERING_BINS, (1.0,) * 5, (1.0,) * 5, (10,) * 5)
    return ba.AuditModel(
        BODY,
        MOUNT,
        {
            "delay_s": 0.04,
            "tau_s": 0.2,
            "max_accel": 1.5,
            "max_decel": 4.0,
            "cmd_gain": 1.0,
            "steer_tau_s": 0.0,
        },
        slip,
        WHEELBASE,
    )


def test_a_brake_into_a_close_wall_at_speed_is_necessary():
    t0, streams = make_streams(wall_x=0.9, request_speed=1.0, wheel=1.0)
    row = ba.analyse_engagement(ba.Engagement(t0, 2, "x", None, "brake"), streams, model())
    assert row.classification == "necessary"
    assert row.cf_min_clearance_m < ba.NECESSARY_CLEARANCE_M
    assert row.cf_hit_s is not None
    assert not row.corner
    assert row.body_clearance_m == pytest.approx(0.9 - BODY.front_x_m, abs=0.1)


def test_a_brake_far_from_the_wall_is_a_false_alarm_and_the_bracket_agrees():
    t0, streams = make_streams(wall_x=3.0, request_speed=0.6, wheel=0.6)
    row = ba.analyse_engagement(ba.Engagement(t0, 2, "x", None, "brake"), streams, model())
    assert row.classification == "false alarm"
    assert not row.sensitive
    assert row.cf_hold_clearance_m >= row.cf_min_clearance_m - 1e-9
    assert row.cf_min_clearance_m > ba.MARGINAL_CLEARANCE_M


def test_the_hold_bracket_never_has_more_clearance_than_the_nominal_plant():
    # The car is asked to slow but would keep its speed under 'hold': never farther from the wall.
    t0, streams = make_streams(wall_x=1.3, request_speed=0.2, wheel=1.0)
    row = ba.analyse_engagement(ba.Engagement(t0, 2, "x", None, "brake"), streams, model())
    assert row.cf_hold_clearance_m <= row.cf_min_clearance_m + 1e-9
    assert row.cf_agile_clearance_m >= row.cf_min_clearance_m - 1e-9


def test_a_request_in_a_hard_turn_is_a_corner():
    t0, streams = make_streams(wall_x=3.0, request_speed=0.5, wheel=0.5, steering=-0.44)
    row = ba.analyse_engagement(ba.Engagement(t0, 2, "x", None, "brake"), streams, model())
    assert row.corner and row.request_steering_rad == pytest.approx(-0.44)


def test_an_engagement_with_no_scan_yet_is_left_empty():
    t0, streams = make_streams(wall_x=3.0, request_speed=0.5, wheel=0.5)
    row = ba.analyse_engagement(ba.Engagement(t0 - 50.0, 2, "x", None, "brake"), streams, model())
    assert row.classification == "" and math.isnan(row.cf_min_clearance_m)
