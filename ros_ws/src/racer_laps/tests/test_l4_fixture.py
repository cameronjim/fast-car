"""L4: 18 s of a real recording (the mapped tracker at 1.5 m/s, segment T2a of the pre-registered
split, two lap closures) through the start-scan lap memory, against golden crossing times.

The fixture `fixtures/t2_scans.npz` holds the LiDAR scans (uint16 quarter millimetres, lossless),
the wheel odometry and the gated steering; `fixtures/t2_golden.json` holds the expected crossing
times and, as an independent reference, the map-pose detector's times on the same slice. How both
were made is in `fixtures/make_t2_fixture.py`. The golden is never refreshed to get a test green:
a change of it needs a stated reason in the PR."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("yaml")
pytest.importorskip("jsonschema")

from racer_laps.engine import LapMemory
from racer_laps.lap_record import STATE_DRIVING
from racer_laps.replay import default_start_scan_config
from racer_laps.scan_match import scan_to_points
from racer_laps.vehicle import load_live_vehicle

FIXTURES = Path(__file__).parent / "fixtures"
QUANTUM_M = 0.00025


def replay_fixture(path: Path) -> dict[str, object]:
    z = np.load(path)
    vehicle = load_live_vehicle()
    mem = LapMemory(default_start_scan_config(vehicle))
    mem.on_planner_status(STATE_DRIVING, math.nan)
    a0, da, rmin, rmax, sweep = (float(x) for x in z["scan_meta"])
    stamps = z["scan_stamp"]
    ranges = z["scan_q"].astype(np.float64) * QUANTUM_M
    ranges[ranges == 0.0] = np.nan
    events: list[tuple[float, int, int]] = []
    events += [(float(t), 0, i) for i, (t, _v) in enumerate(z["odom"])]
    events += [(float(t), 1, i) for i, (t, _s) in enumerate(z["drive"])]
    # a scan is delivered when its sweep has ended (the recorder's log time is about 0.1 s later)
    events += [(float(s) + 0.1, 2, i) for i, s in enumerate(stamps)]
    events.sort()
    crossings: list[float] = []
    for _t, kind, i in events:
        if kind == 0:
            mem.on_wheel_odom(float(z["odom"][i][0]), float(z["odom"][i][1]))
        elif kind == 1:
            mem.on_drive(float(z["drive"][i][1]))
        else:
            pts = scan_to_points(ranges[i], a0, da, rmin, rmax, vehicle.mount, vehicle.body)
            r = mem.on_scan(float(stamps[i]) + 0.5 * sweep, pts)
            if r is not None:
                crossings.append(r.t_cross_s)
    first = float(z["t0"][0])  # log time of the slice's first scan, the axis of the pose reference
    return {"crossings_s": [c - first for c in crossings], "laps": len(crossings)}


def test_the_recorded_slice_closes_two_laps_at_the_golden_times() -> None:
    golden = json.loads((FIXTURES / "t2_golden.json").read_text())
    got = replay_fixture(FIXTURES / "t2_scans.npz")
    expected = golden["expected_crossings_s"]
    assert got["laps"] == len(expected) == 2
    for have, want in zip(got["crossings_s"], expected):  # type: ignore[arg-type]
        assert abs(have - want) <= golden["tolerance_s"], (have, want)


def test_the_golden_agrees_with_the_independent_map_pose_reference() -> None:
    golden = json.loads((FIXTURES / "t2_golden.json").read_text())
    assert len(golden["pose_reference_s"]) == len(golden["expected_crossings_s"])
    for want, ref in zip(golden["expected_crossings_s"], golden["pose_reference_s"]):
        assert abs(want - ref) <= 0.15, (want, ref)


def test_the_fixture_is_small() -> None:
    total = sum(p.stat().st_size for p in FIXTURES.iterdir() if p.is_file())
    assert total < 500_000, total
