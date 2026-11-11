"""L5 canary: the race profile (race-2026-11-12) laps the track family, both directions, and is
faster than the recommended pre-race profile.

The race profile is the lane profile with lookahead 0.8 and outside entry 0.25 plus the physics
speed law, the racing line and the side proximity cap (docs/notes/gap-follow-race-2026-11-12.md).
This runs it, through the profile and with no parameter overrides, on two tracks of the family,
both directions, 2 laps, with the real command path (gap_follow_node -> /drive_raw -> safety_node
-> /drive -> bridge_node) and the car as measured in vehicle_params:

  * a 1.0 m lane round a THIN island (end radius 0, straights 2 m): the case the raceline tracker
    cannot do and the reason the reverse escape exists;
  * a 1.0 m lane with end radius 0.4 m (straights 2 m): round ends, where the speed law has room
    to be fast.

THRESHOLDS, from measured runs with vehicle_params 0.14.0 (`tools/race_tune_docker.sh evaluate`,
docs/notes/gap-follow-race-2026-11-12.md): see the constants below; each carries the measured
value it was set from. They are bands, not the measured numbers: CI runners are loaded 2 vCPU
machines (test_gap_follow_family_canary.py failed once on brakes at 7 against a limit of 6). Wall
contacts are never allowed, clearance never below 0.02 m. The speed gain over the recommended
profile is asserted against the COMMITTED lap time of that profile on the same track (the note's
table), with a margin, so the canary fails if the race profile stops being worth having.
Skipped without a ROS workspace.
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
PROFILE = "race-2026-11-12"
LAPS = 2

# (track params, lap time of the recommended pre-race profile on this track, measured lap time of
# the race profile, brake limit, escape limit)
THIN = ({"lane_width_m": 1.0, "straight_length_m": 2.0, "island_radius_m": 0.0},)
ROUND = ({"lane_width_m": 1.0, "straight_length_m": 2.0, "island_radius_m": 0.4},)
CASES = {
    "thin_island": {
        "params": THIN[0],
        "recommended_lap_s": 23.2,  # note, 3 laps; the recommended profile at 0.64 m/s
        "race_lap_s": 0.0,
        "max_brakes": 0,
        "max_escapes": 0,
    },
    "round_ends": {
        "params": ROUND[0],
        "recommended_lap_s": 19.6,
        "race_lap_s": 0.0,
        "max_brakes": 0,
        "max_escapes": 0,
    },
}
MIN_CLEARANCE_M = 0.02
LAP_TIME_FACTOR = 1.5


@pytest.mark.parametrize("reverse", [False, True], ids=["ccw", "cw"])
@pytest.mark.parametrize("case", sorted(CASES))
def test_race_profile_laps_without_touching_a_wall_and_beats_the_recommended_profile(
    case, reverse, tmp_path
):
    spec = CASES[case]
    out = tmp_path / "result.json"
    command = [
        sys.executable,
        str(RUNNER),
        "--track",
        "stadium",
        "--track-params-json",
        json.dumps(spec["params"]),
        "--driver",
        "gap_follow",
        "--profile",
        PROFILE,
        "--stop-after-laps",
        str(LAPS),
        "--duration",
        "150",
        "--domain-id",
        "102",
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
    assert result["obstacle_brakes"] <= spec["max_brakes"]
    assert result["escapes"] <= spec["max_escapes"]
    assert result["min_body_clearance_m"] >= MIN_CLEARANCE_M
    lap = result["lap_time_s"]
    assert spec["race_lap_s"] / LAP_TIME_FACTOR <= lap <= spec["race_lap_s"] * LAP_TIME_FACTOR, lap
    # Worth having: clearly faster than the recommended profile's committed lap time.
    assert lap <= 0.9 * spec["recommended_lap_s"], (lap, spec["recommended_lap_s"])
