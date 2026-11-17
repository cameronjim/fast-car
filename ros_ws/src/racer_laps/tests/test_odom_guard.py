"""L1: the lap gate does not place a crossing from a scan whose wheel odometry is stale.

`StartScanGate._at` extrapolates a scan's pose at most `MAX_EXTRAPOLATION_S` past the last odometry
sample. If the node serves a scan before the odometry queued behind it (a starved host), the wheel
total at the scan is under-read and every candidate line, and the crossing, is early: the 0.3 to
0.46 s early crossing 3 of three B4 runs (brief 48 item 5, `docs/notes/lap-gate-lap3-2026-11.md`),
reproduced there by delivering the odometry 0.7 s late. The gate now skips such a scan."""

from __future__ import annotations

import numpy as np
import pytest
from racer_laps.engine import LapConfig, LapMemory
from racer_laps.gate import MAX_EXTRAPOLATION_S, GateConfig
from racer_laps.lap_record import STATE_DRIVING
from racer_laps.scan_match import scan_to_points
from synth import BODY, MOUNT, RANGE_MAX_M, WHEELBASE_M, Stadium, drive_stadium, scan_at

SPEED = 1.0
LAPS = 4.2


def run(lag_window: tuple[float, float] | None, lag_s: float = 0.7):
    """Drive a stadium; wheel odometry inside `lag_window` (seconds after the start) is delivered
    `lag_s` late (after the scans stamped then). Returns (LapResults, t0, lap time)."""
    stadium = Stadium(straight_m=3.0, lane_m=0.9, island_r_m=0.5)
    cfg = LapConfig(
        lap_source="start_scan",
        gate=GateConfig(wheelbase_m=WHEELBASE_M, lidar_x_m=MOUNT.x_m, lidar_y_m=MOUNT.y_m),
        v_cap_start_mps=0.5,
        v_cap_ceiling_mps=1.5,
    )
    mem = LapMemory(cfg)
    mem.on_planner_status(STATE_DRIVING, 0.3)
    rng = np.random.default_rng(3)
    walls = stadium.walls()
    events = []
    t0 = None
    for tick in drive_stadium(stadium, LAPS, speed_mps=SPEED, t0=0.0):
        t0 = 0.0
        late = 0.0
        if tick.kind == "odom" and lag_window and lag_window[0] <= tick.t <= lag_window[1]:
            late = lag_s
        events.append((tick.t + late, 0 if tick.kind != "scan" else 1, tick))
    events.sort(key=lambda e: (e[0], e[1]))
    laps = []
    for _t, _k, tick in events:
        if tick.kind == "drive":
            mem.on_drive(tick.payload[0])
        elif tick.kind == "odom":
            mem.on_wheel_odom(tick.t, tick.payload[0])
        else:
            x, y, yaw, stamp = tick.payload
            ranges, a0, da = scan_at(walls, x, y, yaw, rng)
            pts = scan_to_points(ranges, a0, da, 0.05, RANGE_MAX_M, MOUNT, BODY)
            r = mem.on_scan(stamp + 0.04, pts)
            if r is not None:
                laps.append(r)
    return laps, t0, stadium.length_m / SPEED


def crossing_errors(laps, lap_s: float) -> list[float]:
    # truth: the start line is the pose of the start scan (about t = 0.04), crossed every lap_s
    return [r.t_cross_s - (0.0 + k * lap_s) for k, r in enumerate(laps, start=1)]


def test_without_late_odometry_every_crossing_is_on_time() -> None:
    laps, _t0, lap_s = run(None)
    assert [r.lap_index for r in laps] == [1, 2, 3, 4]
    assert all(abs(e) < 0.2 for e in crossing_errors(laps, lap_s))


def test_a_burst_of_0_7_s_late_odometry_over_the_closure_of_lap_3_does_not_place_it_early() -> None:
    _, _, lap_s = run(None)
    window = (2.0 * lap_s - 1.0, 3.0 * lap_s + 1.5)  # covers the approach and closure of lap 3
    laps, _t0, _ = run(window)
    assert [r.lap_index for r in laps] == [1, 2, 3, 4]
    errors = crossing_errors(laps, lap_s)
    # an early crossing from stale odometry is 0.4 s or more (v * (lag - 0.2) = 0.5 m at 1 m/s)
    assert all(abs(e) < 0.2 for e in errors), errors


def test_a_scan_with_stale_odometry_is_skipped_and_counted() -> None:
    stadium = Stadium()
    mem = LapMemory(
        LapConfig(
            lap_source="start_scan",
            gate=GateConfig(wheelbase_m=WHEELBASE_M, lidar_x_m=MOUNT.x_m, lidar_y_m=MOUNT.y_m),
        )
    )
    mem.on_planner_status(STATE_DRIVING, 0.3)
    x, y, yaw, _ = stadium.pose_at(0.0)
    ranges, a0, da = scan_at(stadium.walls(), x, y, yaw, np.random.default_rng(1))
    pts = scan_to_points(ranges, a0, da, 0.05, RANGE_MAX_M, MOUNT, BODY)
    mem.on_wheel_odom(0.0, 0.0)
    mem.on_scan(0.0, pts)
    gate = mem.gate
    assert gate is not None and gate.started
    for k in range(1, 6):
        mem.on_wheel_odom(0.02 * k, 1.0)
    assert gate.stale_odometry_scans == 0
    mem.on_scan(0.1 + MAX_EXTRAPOLATION_S + 0.05, pts)  # odometry ends at 0.10 s
    assert gate.stale_odometry_scans == 1
    assert mem.status()["stale_odometry_scans"] == 1
    mem.on_scan(0.1 + MAX_EXTRAPOLATION_S - 0.01, pts)  # inside the limit: not skipped
    assert gate.stale_odometry_scans == 1


@pytest.mark.parametrize("age", [0.0, 0.1, MAX_EXTRAPOLATION_S])
def test_odometry_at_or_inside_the_limit_is_not_stale(age: float) -> None:
    mem = LapMemory(LapConfig(lap_source="start_scan", gate=GateConfig(wheelbase_m=WHEELBASE_M)))
    assert mem.gate is not None
    mem.gate.on_odom(1.0, 1.0)
    assert not mem.gate.odometry_stale(1.0 + age)
    assert mem.gate.odometry_stale(1.0 + MAX_EXTRAPOLATION_S + 0.001)
