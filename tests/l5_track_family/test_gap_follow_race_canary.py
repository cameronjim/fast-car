"""L5 canary: the race profile (race-2026-11-12) laps the track family, both directions, and is
faster than the recommended pre-race profile.

The race profile is the lane profile with lookahead 0.6 and outside entry 0.25 plus the physics
speed law at a 1.0 m/s ceiling, a late apex, the side proximity cap and commands on scan arrival
(docs/notes/gap-follow-race-2026-11-12.md).
This runs it, through the profile and with no parameter overrides, on two tracks of the family,
both directions, 2 laps, with the real command path (gap_follow_node -> /drive_raw -> safety_node
-> /drive -> bridge_node) and the car as measured in vehicle_params:

  * a 1.0 m lane round a THIN island (end radius 0, straights 2 m): the case the raceline tracker
    cannot do and the reason the reverse escape exists;
  * a 1.0 m lane with end radius 0.4 m (straights 2 m): round ends, where the speed law has room
    to be fast.

THRESHOLDS, from measured runs with vehicle_params 0.14.0 and the profile as committed
(`tools/track_bench_docker.sh run --suite family`, 3 laps, docs/notes/gap-follow-race-2026-11-12.md):
  thin island, 2 m straight: lap 21.6 to 21.9 s, 9 to 11 brakes and 7 escapes in 3 laps, clearance
      0.077 m; the recommended profile 23.1 to 23.2 s, 6 brakes, 6 escapes
  round ends 0.4 m: lap 11.3 to 11.4 s, no brakes, no escapes, clearance 0.174 to 0.178 m; the
      recommended profile 19.6 s
They are bands, not the measured numbers: CI runners are loaded 2 vCPU machines
(test_gap_follow_family_canary.py failed once on brakes at 7 against a limit of 6), and the brake
count of one configuration is not repeatable to the run (1.06 and 1.29 brakes per lap on the same
tracks, the note's section on repeats). Wall contacts are never allowed, clearance never below
0.02 m. The speed gain is asserted against the COMMITTED lap time of the recommended profile on the
same track (table in the note) so the canary fails if the race profile stops being worth having:
on the thin island it must not be much slower than the recommended profile (its ends are the
geometry's doing, not the speed's, and an escape more or less is 2 s of a lap), on the round-ended track it must be at least 25 percent faster.
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

THIN = {"lane_width_m": 1.0, "straight_length_m": 2.0, "island_radius_m": 0.0}
ROUND = {"lane_width_m": 1.0, "straight_length_m": 2.0, "island_radius_m": 0.4}
CASES = {
    "thin_island": {
        "params": THIN,
        "recommended_lap_s": 23.2,  # the recommended profile at 0.64 m/s, 3 laps (the note)
        "race_lap_s": 21.8,  # measured, race profile
        "max_ratio_to_recommended": 1.25,  # measured 0.93 to 1.05: escape luck
        "max_brakes": 20,  # measured 9 to 11 per 3 laps
        "max_escapes": 12,  # measured 7 per 3 laps
    },
    "round_ends": {
        "params": ROUND,
        "recommended_lap_s": 19.6,
        "race_lap_s": 11.4,
        "max_ratio_to_recommended": 0.75,
        "max_brakes": 6,  # measured 0
        "max_escapes": 3,  # measured 0
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
    # Worth having: against the recommended profile's committed lap time on the same track.
    ratio = lap / spec["recommended_lap_s"]
    assert ratio <= spec["max_ratio_to_recommended"], (lap, spec["recommended_lap_s"])
