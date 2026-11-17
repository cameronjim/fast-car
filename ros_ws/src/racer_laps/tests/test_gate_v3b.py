"""L1, brief 50b: the pair centre, the winding rate that comes from it, and the start attitude."""

from __future__ import annotations

import math
from dataclasses import replace

import pytest
import racer_laps.gate as gate_module
from racer_laps.scan_match import MatchResult
from synth import Stadium
from test_gate_v3 import feed_gate
from test_gate_v3_ring import straight_gate
from v3_support import RING

CENTRE_R_M = RING.centre_r_m  # 0.95 m


def pair_after(stadium: Stadium, travel_m: float, clockwise: bool = False, shift_m: float = 0.0):
    """A gate that drove `travel_m` of the stadium; its pair centre after one more update, with the
    displaced scan's recorded pose moved `shift_m` along the odometry x axis (another seed)."""
    gate, _pts = feed_gate(stadium, travel_m, gate_kw={"ring_enabled": True}, clockwise=clockwise)
    now = gate._at(gate._hist[-1].t)
    rec = gate._displaced(now.total_m)
    assert rec is not None
    if shift_m:
        gate._scans = type(gate._scans)(
            replace(r, x=r.x + shift_m) if r is rec else r for r in gate._scans
        )
    gate._pair = None
    gate._source_thin = gate._scans[-1].thin
    gate._source_dense = gate._scans[-1].dense
    gate._update_pair(now)
    return gate


@pytest.mark.parametrize("shift", [-0.15, 0.0, 0.15])
@pytest.mark.parametrize("clockwise", [False, True])
def test_a_steady_ring_gives_the_track_centre_whatever_the_seed(
    shift: float, clockwise: bool
) -> None:
    gate = pair_after(RING, 3.0, clockwise, shift)
    assert gate._pair is not None
    cx, cy, _t = gate._pair
    sign = -1.0 if clockwise else 1.0
    assert cx == pytest.approx(0.0, abs=0.05)
    assert cy == pytest.approx(sign * CENTRE_R_M, abs=0.05)
    # the turn rate it gives: v / R, the position going round the island
    assert abs(cy / (cx * cx + cy * cy)) == pytest.approx(1.0 / CENTRE_R_M, rel=0.06)


def test_a_straight_gives_no_valid_centre() -> None:
    straight = Stadium(straight_m=6.0, lane_m=1.0, island_r_m=0.4)
    gate = pair_after(straight, 2.0)
    assert gate._pair is None  # |yaw| of the match is under pair_min_yaw_rad


def feed_odom(g: gate_module.StartScanGate, t0: float, seconds: float, v: float = 1.0) -> float:
    t = t0
    for _ in range(round(seconds * 50)):
        t += 0.02
        g.on_odom(t, v)
    return t


def test_the_winding_uses_the_steering_rate_inside_launch_steer_m_and_the_pair_rate_after_it() -> (
    None
):
    g = straight_gate()
    g.on_steering(0.0)  # the steering says straight, the pair centre says a 0.5 m circle
    t = 0.0
    g._pair = (0.0, 0.5, 0.0)
    for _ in range(50):  # the first metre: inside launch_steer_m (2.0 m)
        t += 0.02
        g._pair = (0.0, 0.5, t)
        g.on_odom(t, 1.0)
    inside = g._hist[-1].psi
    assert inside == pytest.approx(0.0, abs=1e-9)  # steering rate: straight
    for _ in range(150):  # on past 2 m
        t += 0.02
        g._pair = (0.0, 0.5, t)
        g.on_odom(t, 1.0)
    # 2 m per second of the pair rate v / R = 2 rad/s from about 2 m on
    assert g._hist[-1].psi == pytest.approx(2.0 * (g.total_m - 2.0), rel=0.05)
    assert 0.0 < g._psi_pair_m < g._psi_all_m


def test_a_pair_centre_older_than_pair_max_age_s_is_not_used() -> None:
    g = straight_gate()
    g.on_steering(0.0)
    t = feed_odom(g, 0.0, 2.5)  # past the launch stretch
    g._pair = (0.0, 0.5, t)
    psi0 = g._hist[-1].psi
    t = feed_odom(g, t, 0.4)  # fresh for 0.4 s: the pair rate
    assert g._hist[-1].psi - psi0 == pytest.approx(2.0 * 0.4, rel=0.1)
    psi1 = g._hist[-1].psi
    feed_odom(g, t, 0.6)  # older than 0.5 s after 0.1 s more: the steering rate (zero)
    assert g._hist[-1].psi - psi1 < 2.0 * 0.15


def attitude_gate(c_car: tuple[float, float], c_start: tuple[float, float], yaws: list[float]):
    g = straight_gate()
    for k, yaw in enumerate(yaws):
        # a car whose turn centre is c_car in its own frame and c_start in the start frame
        px = c_start[0] - (math.cos(yaw) * c_car[0] - math.sin(yaw) * c_car[1])
        py = c_start[1] - (math.sin(yaw) * c_car[0] + math.cos(yaw) * c_car[1])
        res = MatchResult(px, py, yaw, 0.02, 0.9, 200)
        now = gate_module._Sample(10.0 + k, 3.0, 0.0, 0.0, 0.0, 0.0)
        g._pair = (c_car[0], c_car[1], now.t)
        g._note_attitude(res, now)
    return g


def test_the_centre_in_the_start_frame_is_the_same_point_for_every_alignment() -> None:
    g = attitude_gate((0.0, 0.89), (0.22, 0.95), [0.5, 1.7, 3.0, 4.2, 5.5])
    bearings = g._att_s
    assert len(bearings) == 5
    assert max(bearings) - min(bearings) < 1e-9
    assert bearings[0] == pytest.approx(math.atan2(0.95, 0.22))
    assert g._d_beta == pytest.approx(math.pi / 2 - math.atan2(0.95, 0.22), abs=1e-9)  # +0.23


def test_under_attitude_min_scans_matches_d_beta_is_zero_and_the_clockwise_sign_flips() -> None:
    few = attitude_gate((0.0, 0.89), (0.22, 0.95), [0.5, 1.7, 3.0, 4.2])
    assert few._d_beta == 0.0  # 4 scans, 5 needed
    cw = attitude_gate((0.0, -0.89), (0.22, -0.95), [0.5, 1.7, 3.0, 4.2, 5.5])
    assert cw._d_beta == pytest.approx(-(math.pi / 2 - math.atan2(0.95, 0.22)), abs=1e-9)
    off = straight_gate(ring_enabled=False)
    off._pair = (0.0, 0.89, 10.0)
    off._note_attitude(
        MatchResult(0.0, 0.0, 0.0, 0.02, 0.9, 200), gate_module._Sample(10.0, 3.0, 0, 0, 0, 0)
    )
    assert off._att_s == []  # the ring off: nothing is collected
    later = attitude_gate((0.0, 0.89), (0.22, 0.95), [0.5])
    later._laps = 1
    later._note_attitude(
        MatchResult(0.0, 0.0, 0.0, 0.02, 0.9, 200), gate_module._Sample(11.0, 3.0, 0, 0, 0, 0)
    )
    assert len(later._att_s) == 1  # only lap 1 collects
