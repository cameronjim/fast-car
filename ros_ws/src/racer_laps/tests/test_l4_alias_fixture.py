"""L4, brief 29 step 0: the start-scan gate on the simulator inputs of a run where gate v1 closed
lap 1 on a false line 3 m before the start (T2, 5.0 m straight, clockwise, seed 12, condition a).

`fixtures/t2_alias_scans.npz` holds the lap node's inputs from the start scan to a little over a
lap (see `fixtures/make_alias_fixture.py`). The loop is 15.65 m; correct laps of that run measure
15.3 to 15.45 m of wheel distance and 34.0 to 34.4 s.

Gate v1 (commit f45fd09) on this input (this test's config): lap 1 closes at 12.16 m, 3.2 m early.
Gate v2 with the design's first violation limit (0.04): 13.61 m. This test fails on those and
passes with the limit chosen on the development data (0.02).

The config is built so that the same test runs against gate v1 on main (which has no LiDAR
position in its config): it then fails on the lap length, not on an unknown argument.
"""

from __future__ import annotations

import inspect
import math
from pathlib import Path

import numpy as np
import pytest
from racer_laps.engine import LapConfig, LapMemory
from racer_laps.gate import GateConfig
from racer_laps.lap_record import STATE_DRIVING
from racer_laps.scan_match import scan_to_points
from synth import BODY, MOUNT, WHEELBASE_M

FIXTURE = Path(__file__).resolve().parent / "fixtures" / "t2_alias_scans.npz"
#: The simulator's scan: 720 beams over 2 pi, first beam at -(719/720) pi (bridge_node).
ANGLE_MIN = -(2.0 * math.pi * 719.0 / 720.0) / 2.0
ANGLE_INC = 2.0 * math.pi / 720.0
#: The yaw gain the runner gives the gate for the sim plant (track_family_runner SIM_YAW_GAIN_*).
SIM_YAW_GAIN_LEFT = 0.903
SIM_YAW_GAIN_RIGHT = 1.008
LOOP_WHEEL_M = 15.40
TRUE_LOOP_M = 15.654


def replay(**gate_kw: float) -> list:
    z = np.load(FIXTURE)
    scan_t, scan_mm, odom, drive = z["scan_t"], z["scan_mm"], z["odom"], z["drive"]
    start = float(z["start_t"][0])
    kw: dict = {
        "wheelbase_m": WHEELBASE_M,
        "yaw_gain_table": ((0.0, SIM_YAW_GAIN_RIGHT),),  # clockwise: right turns
        **gate_kw,
    }
    if "lidar_x_m" in inspect.signature(GateConfig).parameters:
        kw["lidar_x_m"], kw["lidar_y_m"] = MOUNT.x_m, MOUNT.y_m
    mem = LapMemory(LapConfig(lap_source="start_scan", gate=GateConfig(**kw)))
    events = [(t, 2, i) for i, t in enumerate(scan_t)]
    events += [(r[0], 0, i) for i, r in enumerate(odom)]
    events += [(r[0], 1, i) for i, r in enumerate(drive)]
    events.sort()
    started = False
    laps = []
    for _t, kind, i in events:
        if kind == 0:
            mem.on_wheel_odom(odom[i][0], odom[i][1])
        elif kind == 1:
            mem.on_drive(drive[i][1])
        else:
            if not started and scan_t[i] >= start - 1e-3:
                mem.on_planner_status(STATE_DRIVING, 0.3)
                started = True
            ranges = scan_mm[i].astype(np.float64) / 1000.0
            ranges[ranges == 0.0] = np.nan
            pts = scan_to_points(ranges, ANGLE_MIN, ANGLE_INC, 0.05, 16.0, MOUNT, BODY)
            r = mem.on_scan(float(scan_t[i]), pts)
            if r is not None:
                laps.append(r)
    return laps


def test_the_fixture_is_small_and_complete() -> None:
    assert FIXTURE.stat().st_size < 500_000
    z = np.load(FIXTURE)
    assert z["scan_mm"].shape[1] == 720 and z["scan_mm"].dtype == np.uint16
    assert z["scan_t"][-1] - z["start_t"][0] > 38.0


def test_lap_1_closes_at_the_start_line_not_on_the_alias_two_metres_before_it() -> None:
    laps = replay()
    assert laps, "no lap closed in a little over one lap of inputs"
    first = laps[0]
    assert first.lap_index == 1 and not first.missed_gate
    assert first.length_m == pytest.approx(LOOP_WHEEL_M, abs=0.45), first.length_m
    assert first.lap_time_s == pytest.approx(34.3, abs=0.6), first.lap_time_s
    assert TRUE_LOOP_M - first.length_m < 0.6


def test_the_design_starting_limit_is_too_loose_on_this_input() -> None:
    """Development evidence for the limit: with 0.04 the same input closes lap 1 about 2 m early
    (the false line shows a violation share of 0.032 to 0.075, the true line 0.000)."""
    if "strong_max_violation" not in inspect.signature(GateConfig).parameters:
        pytest.skip("gate v1 has no violation limit")
    laps = replay(strong_max_violation=0.04)
    assert laps and laps[0].length_m < LOOP_WHEEL_M - 1.0
