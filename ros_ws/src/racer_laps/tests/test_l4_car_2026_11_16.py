"""L4: two slices of the car runs of 2026-11-16 through the start-scan lap memory (brief 48).

* `gp3r1_laps_14_to_17.npz` (GP3-R1, laps 14 to 17 of the 20 lap run): 4 laps of a 5.2 m loop at
  1 m/s. The gate of PR 201 flagged the last one missed (the closure is decided at station 5.7 m
  or more after a crossing at 5.3 m: the live station passed the estimate plus the band while the
  closure was pending) and the D v0 cap fell. Now: 4 laps, none missed, all clean, the cap never
  falls.
* `t0218_last_lap_parked.npz` (the earlier 20 lap attempt): the car parks 0.15 m past the line
  with a candidate on every scan and the last lap never closed (16 to 17 live candidates arrived
  every 1.5 s for minutes). Now: the lap closes within a second of standstill.

How the slices were cut and why those windows: `fixtures_car_2026_11_16/make_car_fixtures.py`.
The pinned numbers are the first record of these slices under gate v2.1 and are never refreshed to
get a test green: a change needs a stated reason in the PR."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("yaml")
pytest.importorskip("jsonschema")

from racer_laps.engine import LapMemory, LapResult
from racer_laps.gate import STANDSTILL_MPS
from racer_laps.replay import default_start_scan_config
from racer_laps.scan_match import scan_to_points
from racer_laps.vehicle import load_live_vehicle

FIXTURES = Path(__file__).parent / "fixtures_car_2026_11_16"
QUANTUM_M = 0.00025
#: A scan is delivered when its sweep has ended: the recorder's log time is stored with it.


def replay_slice(name: str) -> tuple[list[LapResult], list[float], dict[str, np.ndarray]]:
    """The node's logic over a stored slice, every message in log order. Returns the laps, the
    log time of the scan that closed each one and the arrays."""
    z = dict(np.load(FIXTURES / name))
    vehicle = load_live_vehicle()
    mem = LapMemory(default_start_scan_config(vehicle))
    a0, da, rmin, rmax, sweep = (float(x) for x in z["scan_meta"])
    ranges = z["scan_q"].astype(np.float64) * QUANTUM_M
    ranges[ranges == 0.0] = np.nan
    order: list[tuple[float, int, int]] = []
    order += [(float(row[0]), 0, i) for i, row in enumerate(z["odom"])]
    order += [(float(row[0]), 1, i) for i, row in enumerate(z["drive"])]
    order += [(float(row[0]), 2, i) for i, row in enumerate(z["planner"])]
    order += [(float(row[0]), 3, i) for i, row in enumerate(z["events"])]
    order += [(float(t), 4, i) for i, t in enumerate(z["scan_log"])]
    order.sort()
    laps: list[LapResult] = []
    decided: list[float] = []
    for t_log, kind, i in order:
        if kind == 0:
            mem.on_wheel_odom(float(z["odom"][i][1]), float(z["odom"][i][2]))
        elif kind == 1:
            mem.on_drive(float(z["drive"][i][1]))
        elif kind == 2:
            mem.on_planner_status(int(z["planner"][i][1]), float(z["planner"][i][2]))
        elif kind == 3:
            ev = z["events"][i]
            mem.on_safety_event(int(ev[1]), int(ev[2]), str(z["event_source"][i]))
        else:
            pts = scan_to_points(ranges[i], a0, da, rmin, rmax, vehicle.mount, vehicle.body)
            r = mem.on_scan(float(z["scan_stamp"][i]) + 0.5 * sweep, pts)
            if r is not None:
                laps.append(r)
                decided.append(t_log)
    return laps, decided, z


def test_gp3r1_laps_14_to_17_are_four_clean_laps_and_the_cap_never_falls() -> None:
    laps, _decided, z = replay_slice("gp3r1_laps_14_to_17.npz")
    assert [lap.lap_index for lap in laps] == [1, 2, 3, 4]
    assert [lap.missed_gate for lap in laps] == [False] * 4
    assert [lap.clean for lap in laps] == [True] * 4
    # the cap schedule rises one step per clean lap and never falls
    assert [round(lap.cap_mps, 2) for lap in laps] == [0.5, 0.6, 0.7, 0.8]
    assert [round(lap.next_cap_mps, 2) for lap in laps] == [0.6, 0.7, 0.8, 0.9]
    # first record of this slice, laps 14 to 17 of the full run (5.63 5.74 5.52 5.78 s from
    # its own start line) to within the slice's own line position
    for lap, want in zip(laps, (5.68, 5.73, 5.53, 5.78), strict=True):
        assert lap.lap_time_s == pytest.approx(want, abs=0.06)
    assert all(5.0 < lap.length_m < 5.5 for lap in laps)
    # the slice holds no safety event that could make a lap unclean
    assert len(z["events"]) == 0


def test_the_last_lap_closes_within_a_second_of_the_car_standing_still() -> None:
    laps, decided, z = replay_slice("t0218_last_lap_parked.npz")
    assert [lap.lap_index for lap in laps] == [1, 2], "the lap of the parked car never closed"
    assert [lap.missed_gate for lap in laps] == [False, False]
    odom = z["odom"]
    moving = [k for k, row in enumerate(odom) if row[2] >= STANDSTILL_MPS]
    assert moving and moving[-1] + 1 < len(odom)
    standing_from = float(odom[moving[-1] + 1][0])  # log time of the first sample standing
    assert decided[-1] - standing_from <= 1.0, decided[-1] - standing_from
    assert decided[-1] >= standing_from - 0.2  # it was a car still rolling that was decided
    # the crossing is on the line: the lap is as long as the one before
    assert laps[1].length_m == pytest.approx(laps[0].length_m, abs=0.1)
    assert laps[1].lap_time_s == pytest.approx(5.66, abs=0.06)
    assert math.isfinite(laps[1].t_cross_s)
    # 5 s parked: nothing closes a third lap
    assert decided[-1] < float(z["odom"][-1][0]) - 3.0
