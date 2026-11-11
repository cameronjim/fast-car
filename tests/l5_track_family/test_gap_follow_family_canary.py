"""L5 canary: gap_follow laps the thin-island stadiums the raceline tracker cannot, both directions.

A 1.0 m lane round a thin island (radius 0: a wall) has an end whose centreline radius (0.5 m) is
under the car's turning radius (0.69 m, vehicle_params 0.14.0): the car cannot follow the middle
of it and cannot clear the outer wall at full lock in one sweep. The raceline tracker stalls
against safety_node there (docs/notes/track-session-2026-11-11.md); gap_follow with its reverse
escape gets round. This runs it at the settings the note recommends (lane-2026-10-25 with
lane_lookahead_m 0.8 and outside_entry_offset_m 0.25) on two such tracks, 0.64 m/s.

THRESHOLDS, from measured runs with vehicle_params 0.14.0:
  24 local runs of the 2 m straight, ccw, 2 laps, 1.5 CPUs per run: lap 23.13 to 23.55 s, brakes
      4 (23 runs) or 5 (1), escapes 4, wall contacts 0, minimum clearance 0.080 to 0.0995 m
  3 laps (the note's family table): 2 m cw 23.2 s, 6 brakes, 6 escapes, clearance 0.077; 1 m
      straights 19.7 to 19.8 s, 7 brakes, 6 escapes, clearance 0.095 to 0.104
About one escape per end, which is the geometry, not a defect. CI failed once with 7 brakes (the
limit was 6, set from 3-lap numbers) on a loaded 2 vCPU runner: the same four escapes, but
safety_node's TTC latch engaged more than once in some when the scheduler delayed a cycle. So:
brakes at most 10 (measured 4 to 5 locally, 7 in CI), escapes at most 8 (measured 4; each end
needs one and a second attempt is allowed), no wall contact, clearance at least 0.02 m
(measured 0.08 and up), mean lap time within a factor 1.5 of the measured one. Skipped without a
ROS workspace.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("f1tenth_gym")

RUNNER = Path(__file__).resolve().parents[1] / "l5_reactive_bench" / "track_family_runner.py"
LAPS = 2
PARAMS = {"max_speed_mps": 0.64, "lane_lookahead_m": 0.8, "outside_entry_offset_m": 0.25}

LONG = ({"lane_width_m": 1.0, "straight_length_m": 2.0, "island_radius_m": 0.0}, 23.3)
SHORT = ({"lane_width_m": 1.0, "straight_length_m": 1.0, "island_radius_m": 0.0}, 19.8)
MAX_BRAKES = 10
MAX_ESCAPES = 8
MIN_CLEARANCE_M = 0.02
LAP_TIME_FACTOR = 1.5


@pytest.mark.parametrize("reverse", [False, True], ids=["ccw", "cw"])
@pytest.mark.parametrize("case", [LONG, SHORT], ids=["straight_2m", "straight_1m"])
def test_gap_follow_gets_round_a_thin_island_without_touching_a_wall(case, reverse, tmp_path):
    params, measured_lap_s = case
    out = tmp_path / "result.json"
    command = [
        sys.executable,
        str(RUNNER),
        "--track",
        "stadium",
        "--track-params-json",
        json.dumps(params),
        "--driver",
        "gap_follow",
        "--params-json",
        json.dumps(PARAMS),
        "--stop-after-laps",
        str(LAPS),
        "--duration",
        "150",
        "--domain-id",
        "79",
        "--work-dir",
        str(tmp_path / "work"),
        "--out",
        str(out),
    ]
    if reverse:
        command.append("--reverse")
    proc = subprocess.run(
        command, env=dict(os.environ), capture_output=True, text=True, timeout=500, check=False
    )
    assert out.exists(), proc.stdout[-2000:] + proc.stderr[-2000:]
    result = json.loads(out.read_text())
    assert "error" not in result, result["error"]
    assert result["laps"] >= LAPS, result.get("stopped_because")
    assert result["wall_contacts"] == 0
    assert result["obstacle_brakes"] <= MAX_BRAKES
    assert result["escapes"] <= MAX_ESCAPES
    assert result["min_body_clearance_m"] >= MIN_CLEARANCE_M
    lap = result["lap_time_s"]
    assert measured_lap_s / LAP_TIME_FACTOR <= lap <= measured_lap_s * LAP_TIME_FACTOR, lap
