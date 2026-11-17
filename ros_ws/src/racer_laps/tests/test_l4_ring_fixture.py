"""L4, brief 50: the lap node's inputs of a simulator run on the ring P4 (stadium with no straight:
lane 1.1 m, island radius 0.40 m, 0.5 m/s), through the lap memory of the checked-out gate.

`fixtures_ring/p4_ccw_11_three_laps.npz` was cut by `fixtures_ring/make_ring_fixture.py` from the
development run `a_P4_ccw_11` (counter-clockwise, seed 11, condition a). Gate v2.1 closes lap 1 of
that input about a second early (a ring gives the start scan as a STRONG match everywhere); gate v3
counts three laps by winding and says so. Golden: `fixtures_ring/golden.json`, compared with a
tolerance of 0.02 s and never refreshed to get a test green. The golden is a record of what the gate
does, not a pass of D8-S0: lap 1 of this input is 0.64 s (5 percent of the lap) longer than the
simulator's truth, which is outside the 4 percent of the aliased class (note
docs/notes/lap-gate-v3-dev-2026-11.md, A5)."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("yaml")
pytest.importorskip("jsonschema")

from racer_laps.engine import LapConfig, LapMemory, LapResult
from racer_laps.gate import GateConfig
from racer_laps.lap_record import STATE_DRIVING
from racer_laps.scan_match import scan_to_points
from racer_laps.vehicle import load_live_vehicle

FIXTURES = Path(__file__).parent / "fixtures_ring"
FIXTURE = FIXTURES / "p4_ccw_11_three_laps.npz"
#: The simulator's scan: 720 beams over 2 pi, first beam at -(719/720) pi (bridge_node).
ANGLE_MIN = -(2.0 * math.pi * 719.0 / 720.0) / 2.0
ANGLE_INC = 2.0 * math.pi / 720.0


def replay(**gate_kw: object) -> list[LapResult]:
    z = np.load(FIXTURE)
    scan_t, scan_mm, odom, drive = z["scan_t"], z["scan_mm"], z["odom"], z["drive"]
    left, right = (float(x) for x in z["yaw_gains"])
    start = float(z["start_t"][0])
    vehicle = load_live_vehicle()
    mem = LapMemory(
        LapConfig(
            lap_source="start_scan",
            gate=GateConfig(
                wheelbase_m=vehicle.wheelbase_m,
                yaw_gain_table=((0.0, left),),
                yaw_gain_right_scale=right / left,
                lidar_x_m=vehicle.mount.x_m,
                lidar_y_m=vehicle.mount.y_m,
                **gate_kw,  # type: ignore[arg-type]
            ),
            v_cap_start_mps=0.5,
            v_cap_ceiling_mps=0.5,
        )
    )
    events = [(float(t), 2, i) for i, t in enumerate(scan_t)]
    events += [(float(r[0]), 0, i) for i, r in enumerate(odom)]
    events += [(float(r[0]), 1, i) for i, r in enumerate(drive)]
    events.sort()
    started = False
    laps: list[LapResult] = []
    for _t, kind, i in events:
        if kind == 0:
            mem.on_wheel_odom(float(odom[i][0]), float(odom[i][1]))
        elif kind == 1:
            mem.on_drive(float(drive[i][1]))
        else:
            if not started and scan_t[i] >= start - 1e-3:
                mem.on_planner_status(STATE_DRIVING, 0.3)
                started = True
            ranges = scan_mm[i].astype(np.float64) / 1000.0
            ranges[ranges == 0.0] = np.nan
            pts = scan_to_points(
                ranges, ANGLE_MIN, ANGLE_INC, 0.05, 16.0, vehicle.mount, vehicle.body
            )
            r = mem.on_scan(float(scan_t[i]), pts)
            if r is not None:
                laps.append(r)
    return laps


def test_the_fixture_is_small_and_complete() -> None:
    assert FIXTURE.stat().st_size < 2_000_000
    z = np.load(FIXTURE)
    assert z["scan_mm"].shape[1] == 720 and z["scan_mm"].dtype == np.uint16
    assert len(z["truth_lap_s"]) == 3


def test_gate_v3_counts_three_laps_by_winding_and_the_times_are_the_golden_ones() -> None:
    golden = json.loads((FIXTURES / "golden.json").read_text())
    laps = replay()
    assert len(laps) == golden["n_laps"] == 3
    assert [r.line_source for r in laps] == golden["line_sources"] == ["winding"] * 3
    assert not any(r.missed_gate for r in laps)
    for got, want in zip(laps, golden["lap_times_s"], strict=True):
        assert got.lap_time_s == pytest.approx(want, abs=golden["tolerance_s"])


def test_gate_v21_closes_lap_1_of_the_same_input_early() -> None:
    """The premise on simulator data: with the v3 additions off the first lap is short."""
    truth = np.load(FIXTURE)["truth_lap_s"]
    laps = replay(ring_enabled=False, unique_shift_m=0.0)
    assert laps and laps[0].lap_time_s < 0.95 * float(truth[1])
    assert all(r.line_source == "start_scan" for r in laps)
