"""Helpers of the gate v2 tests that need no hypothesis, so test_gate_v3d.py (and the L3 job's
system python, which has no hypothesis) can import them without importing test_gate_v2.py."""

from __future__ import annotations

from racer_laps.engine import LapConfig, LapMemory
from racer_laps.gate import GateConfig
from racer_laps.lap_record import STATE_DRIVING
from synth import MOUNT, WHEELBASE_M, Stadium, feed

__all__ = ["ISLAND_R_M", "LANE_M", "LAPS", "feed", "make_mem", "truth_lap_times"]

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
