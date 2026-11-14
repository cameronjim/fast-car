"""L1 regression and L2 properties of gate v2 on a synthetic stadium with the faults that made gate
v1 close laps in the wrong place on long straights (brief 29): odometry scale error, yaw gain
error, a weaving car, straights of 2, 3 and 5 m, speed rising lap over lap.

Gate v1 stays out of the code. Its failure is reproduced on a real simulator input dump
(`test_l4_alias_fixture.py`); the synthetic world here is the regression the fix must keep.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st
from racer_laps.engine import LapConfig, LapMemory
from racer_laps.gate import GateConfig
from racer_laps.lap_record import STATE_DRIVING
from racer_laps.scan_match import scan_to_points
from synth import BODY, MOUNT, RANGE_MAX_M, WHEELBASE_M, Stadium, drive_stadium, feed, scan_at

LAPS = 8
LANE_M = 0.9
ISLAND_R_M = 0.5


def make_mem(gain: float = 1.1, **gate_kw: object) -> LapMemory:
    cfg = LapConfig(
        lap_source="start_scan",
        gate=GateConfig(
            wheelbase_m=WHEELBASE_M,
            yaw_gain_table=((0.0, gain),),
            lidar_x_m=MOUNT.x_m,
            lidar_y_m=MOUNT.y_m,
            **gate_kw,  # type: ignore[arg-type]
        ),
        v_cap_start_mps=0.5,
        v_cap_ceiling_mps=1.5,
    )
    mem = LapMemory(cfg)
    mem.on_planner_status(STATE_DRIVING, 0.3)
    return mem


def truth_lap_times(stadium: Stadium, speed: float, step: float) -> list[float]:
    return [stadium.length_m / (speed + step * k) for k in range(LAPS)]


@pytest.mark.parametrize("straight_m", [2.0, 3.0, 5.0])
@pytest.mark.parametrize("clockwise", [False, True])
@pytest.mark.parametrize("scale", [0.95, 1.0, 1.05])
@pytest.mark.parametrize("rising", [False, True], ids=["0.5mps", "rising-to-1.2mps"])
def test_lap_count_and_times_are_right_on_long_straights_with_odometry_errors(
    straight_m: float, clockwise: bool, scale: float, rising: bool
) -> None:
    """8 laps: the count is right, no lap is missed, every lap time is within 0.3 s of truth and
    the wheel distance of every lap is the loop length (to the odometry scale)."""
    stadium = Stadium(straight_m=straight_m, lane_m=LANE_M, island_r_m=ISLAND_R_M)
    step = 0.1 if rising else 0.0
    mem = make_mem(gain=1.1)
    laps = feed(
        mem,
        stadium,
        laps=LAPS + 0.2,
        speed_mps=0.5,
        seed=int(10 * straight_m) + int(clockwise),
        clockwise=clockwise,
        odom_scale=scale,
        speed_step_per_lap_mps=step,
        wobble_m=0.1,
    )
    assert [r.lap_index for r in laps] == list(range(1, LAPS + 1))  # type: ignore[attr-defined]
    truth = truth_lap_times(stadium, 0.5, step)
    for r, t in zip(laps, truth, strict=True):
        assert not r.missed_gate  # type: ignore[attr-defined]
        assert abs(r.lap_time_s - t) < 0.3, (r.lap_index, r.lap_time_s, t)  # type: ignore[attr-defined]
        assert r.length_m / scale == pytest.approx(stadium.length_m, abs=0.35)  # type: ignore[attr-defined]


@pytest.mark.parametrize("clockwise", [False, True])
@pytest.mark.parametrize("scale", [1.0, 1.05])
def test_an_under_reading_yaw_gain_of_ten_percent_still_closes_every_lap(
    clockwise: bool, scale: float
) -> None:
    stadium = Stadium(straight_m=3.0, lane_m=LANE_M, island_r_m=ISLAND_R_M)
    mem = make_mem(gain=0.9)
    laps = feed(
        mem,
        stadium,
        laps=4.2,
        speed_mps=0.5,
        seed=5,
        clockwise=clockwise,
        odom_scale=scale,
        wobble_m=0.1,
    )
    assert [r.lap_index for r in laps] == [1, 2, 3, 4]  # type: ignore[attr-defined]
    for r in laps:
        assert abs(r.lap_time_s - stadium.length_m / 0.5) < 0.3  # type: ignore[attr-defined]


def test_the_closure_reports_how_many_candidates_agreed() -> None:
    stadium = Stadium(straight_m=3.0, lane_m=LANE_M, island_r_m=ISLAND_R_M)
    mem = make_mem()
    laps = feed(mem, stadium, laps=3.2, speed_mps=0.5, seed=2)
    assert len(laps) == 3
    for r in laps:
        assert r.n_candidates >= 1 and r.spread_m < 0.6  # type: ignore[attr-defined]


# -- L2: properties over random odometry and scans -----------------------------------------------


@settings(
    max_examples=10,
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow, HealthCheck.data_too_large],
)
@given(
    scale=st.floats(0.9, 1.1),
    gain=st.floats(0.95, 1.2),
    drop=st.floats(0.0, 0.6),
    speed=st.floats(0.6, 1.6),
    laps=st.floats(1.5, 3.5),
    clockwise=st.booleans(),
    seed=st.integers(0, 10_000),
    jitter=st.floats(0.0, 0.2),
)
def test_gate_invariants_for_any_odometry_and_scans(
    scale: float,
    gain: float,
    drop: float,
    speed: float,
    laps: float,
    clockwise: bool,
    seed: int,
    jitter: float,
) -> None:
    """Whatever the odometry and the scans: no two closures within the minimum lap length, lap
    indices rise by one, a closure is never in the future and never before the previous one, the
    station is never negative."""
    stadium = Stadium()
    mem = make_mem(gain=gain)
    gate = mem.gate
    assert gate is not None
    min_len = gate.cfg.min_lap_length_m
    rng = np.random.default_rng(seed)
    walls = stadium.walls()
    closures: list[tuple[float, float, int]] = []  # t_cross, length, lap index
    last_scan_t = -math.inf
    for tick in drive_stadium(
        stadium, laps, speed, clockwise=clockwise, odom_scale=scale, wobble_m=jitter * 0.5
    ):
        if tick.kind == "drive":
            mem.on_drive(tick.payload[0] * (1.0 + rng.normal(0.0, 0.02)))
        elif tick.kind == "odom":
            mem.on_wheel_odom(tick.t, tick.payload[0])
            assert gate.station_m >= -1e-9
        else:
            if rng.random() < drop:
                continue
            x, y, yaw, stamp = tick.payload
            ranges, a0, da = scan_at(
                walls, x + rng.normal(0, jitter * 0.3), y, yaw + rng.normal(0, jitter * 0.1), rng
            )
            pts = scan_to_points(ranges, a0, da, 0.05, RANGE_MAX_M, MOUNT, BODY)
            t_eff = stamp + 0.04
            last_scan_t = t_eff
            r = mem.on_scan(t_eff, pts)
            if r is not None:
                assert r.t_cross_s <= t_eff + 1e-9  # never in the future
                closures.append((r.t_cross_s, r.length_m, r.lap_index))
    assert last_scan_t > -math.inf
    for k, (t, length, index) in enumerate(closures):
        assert index == k + 1
        assert length >= min_len - 1e-6  # no two closures within the minimum lap length
        if k > 0:
            assert t > closures[k - 1][0]  # never before the previous closure
    assert gate.laps_completed == len(closures)
