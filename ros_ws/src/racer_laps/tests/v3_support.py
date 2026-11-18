"""Helpers of the gate v3 tests: a ring, a ring with one box and a stadium driven through
`LapMemory`, with and without late wheel odometry. Works against gate v2.1 too (no v3 argument is
passed unless a test asks for one), so the F tests can be seen red on main."""

from __future__ import annotations

import numpy as np
from racer_laps.engine import LapConfig, LapMemory, LapResult
from racer_laps.gate import GateConfig
from racer_laps.lap_record import STATE_DRIVING
from racer_laps.scan_match import scan_to_points
from synth import (
    BODY,
    MOUNT,
    RANGE_MAX_M,
    WHEELBASE_M,
    Stadium,
    Tick,
    drive_stadium,
    feed,
    scan_at,
)

RING = Stadium(straight_m=0.0, lane_m=1.1, island_r_m=0.4)  # P4-like: 5.97 m round
RING_SPEED = 0.5
STADIUM = Stadium(straight_m=3.0, lane_m=0.9, island_r_m=0.5)
OFF = {"ring_enabled": False, "unique_shift_m": 0.0}


def make_mem(**gate_kw: object) -> LapMemory:
    cfg = LapConfig(
        lap_source="start_scan",
        gate=GateConfig(
            wheelbase_m=WHEELBASE_M, lidar_x_m=MOUNT.x_m, lidar_y_m=MOUNT.y_m, **gate_kw
        ),
        v_cap_start_mps=0.5,
        v_cap_ceiling_mps=1.5,
    )
    mem = LapMemory(cfg)
    mem.on_planner_status(STATE_DRIVING, 0.3)
    return mem


def run(
    stadium: Stadium, laps: float, speed: float, gate_kw: dict | None = None, **drive_kw: float
) -> tuple[LapMemory, list[LapResult]]:
    mem = make_mem(**(gate_kw or {}))
    out = feed(mem, stadium, laps, speed_mps=speed, t0=0.0, **drive_kw)
    return mem, [r for r in out if isinstance(r, LapResult)]


def lap_s(stadium: Stadium, speed: float) -> float:
    return stadium.length_m / speed


def run_late(window: tuple[float, float] | None, lag_s: float = 0.35, laps: float = 4.2):
    cfg_kw: dict = {}
    mem = make_mem(**cfg_kw)
    rng = np.random.default_rng(3)
    walls = STADIUM.walls()
    events = []
    for tick in drive_stadium(STADIUM, laps, speed_mps=1.0, t0=0.0):
        late = 0.0
        if tick.kind == "odom" and window and window[0] <= tick.t <= window[1]:
            late = lag_s
        events.append((tick.t + late, 0 if tick.kind != "scan" else 1, tick))
    events.sort(key=lambda e: (e[0], e[1]))
    out: list[LapResult] = []
    for _t, _k, tick in events:
        deliver(mem, tick, walls, rng, out)
    return mem, out


def deliver(
    mem: LapMemory, tick: Tick, walls: np.ndarray, rng: np.random.Generator, out: list[LapResult]
) -> None:
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
            out.append(r)
