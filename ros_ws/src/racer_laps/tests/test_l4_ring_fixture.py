"""L4, briefs 50 and 50b: the lap node's inputs of four simulator runs through the lap memory of the
checked-out gate. Development data, cut by `fixtures_ring/make_ring_fixture.py`:

* `p4_ccw_11`, `p4_cw_11`: development case P4, a ring (stadium with no straight: lane 1.1 m, island
  radius 0.40 m), condition a (0.5 m/s), seed 11, 4 truth laps;
* `nd_ccw_11`, `nd_cw_11`: development case ND, a near ring (straight 0.3 m, lane 1.0 m, island
  radius 0.5 m), condition c (D v0), seed 11, 13 truth laps.

Brief 50b item 4: on every one the gate closes every lap by `winding`, every lap is inside the
aliased tolerance (the larger of 0.3 s and 4 percent of the lap) and every lap error is within 0.10 s
of the designer's prototype (the table of the brief: lap time minus truth). Gate v2.1 on the same
input closes lap 1 of P4 early (the premise of brief 50)."""

from __future__ import annotations

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
#: The simulator's scan: 720 beams over 2 pi, first beam at -(719/720) pi (bridge_node).
ANGLE_MIN = -(2.0 * math.pi * 719.0 / 720.0) / 2.0
ANGLE_INC = 2.0 * math.pi / 720.0
TOLERANCE_SHARE = 0.04
TOLERANCE_FLOOR_S = 0.3
TABLE_SLACK_S = 0.10

#: Brief 50b, "revised winding" row of the prototype table: lap time minus truth, per lap (a list
#: of numbers for P4) or the first lap and the range of the others (ND).
PROTOTYPE = {
    "p4_ccw_11": [0.12, 0.09, -0.02, 0.21],
    "p4_cw_11": [0.08, 0.01, -0.08, 0.08],
    "nd_ccw_11": (0.19, -0.06, 0.10),
    "nd_cw_11": (0.16, -0.08, 0.09),
}


def replay(name: str, **gate_kw: object) -> list[LapResult]:
    z = np.load(FIXTURES / f"{name}.npz")
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
            v_cap_ceiling_mps=1.3,
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


def test_the_fixtures_are_small_and_complete() -> None:
    for name in PROTOTYPE:
        path = FIXTURES / f"{name}.npz"
        assert path.stat().st_size < 2_000_000, name
        z = np.load(path)
        assert z["scan_mm"].shape[1] == 720 and z["scan_mm"].dtype == np.uint16
        assert len(z["truth_lap_s"]) == (4 if name.startswith("p4") else 13)


@pytest.mark.parametrize("name", list(PROTOTYPE))
def test_every_lap_closes_by_winding_inside_the_tolerance_and_as_the_prototype_says(
    name: str,
) -> None:
    truth = np.load(FIXTURES / f"{name}.npz")["truth_lap_s"]
    laps = replay(name)
    assert len(laps) == len(truth)
    assert [r.line_source for r in laps] == ["winding"] * len(laps)
    assert not any(r.missed_gate for r in laps)
    errors = [r.lap_time_s - float(t) for r, t in zip(laps, truth, strict=True)]
    for err, t in zip(errors, truth, strict=True):
        assert abs(err) <= max(TOLERANCE_FLOOR_S, TOLERANCE_SHARE * float(t)), errors
    want = PROTOTYPE[name]
    if isinstance(want, list):
        for got, expected in zip(errors, want, strict=True):
            assert got == pytest.approx(expected, abs=TABLE_SLACK_S), errors
    else:
        first, lo, hi = want
        assert errors[0] == pytest.approx(first, abs=TABLE_SLACK_S), errors
        for got in errors[1:]:
            assert lo - TABLE_SLACK_S <= got <= hi + TABLE_SLACK_S, errors


def test_the_status_says_it_was_winding_and_why() -> None:
    z = np.load(FIXTURES / "p4_ccw_11.npz")
    assert len(z["truth_lap_s"]) == 4
    laps = replay("p4_ccw_11")
    assert laps[0].line_source == "winding"


def test_gate_v21_closes_lap_1_of_the_same_p4_input_early_or_late_but_not_by_winding() -> None:
    """The premise on simulator data: with the v3 additions off the line is a scan's, and a ring's
    scan does not know where along the lap the car is."""
    laps = replay("p4_ccw_11", ring_enabled=False, unique_shift_m=0.0)
    truth = np.load(FIXTURES / "p4_ccw_11.npz")["truth_lap_s"]
    assert laps and laps[0].lap_time_s < 0.95 * float(truth[1])
    assert all(r.line_source == "start_scan" for r in laps)
