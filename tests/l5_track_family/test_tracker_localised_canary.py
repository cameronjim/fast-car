"""L5 canary: the raceline tracker on the LOCALISED pose (the car's odometry and AMCL on the track's
map, sensor errors on) laps the family tracks without a safety brake, both directions.

This is the configuration the real car runs, and the one the ground-truth canary
(test_track_family_canary.py) cannot see: with vehicle_params 0.14.0 the tracker on the localised
pose braked 4 to 95 times in three laps on tracks where ground truth lapped cleanly (the note,
docs/notes/localised-tracker-2026-11-11.md). The cause was a raceline that passed 0.09 to 0.12 m
from the wall (the old widest body clearance asked for was 0.15 m, and the car does not hold the
line to better than the pose error), which with the scan noise and a 5 to 10 cm pose error puts
a wall inside safety_node's gate; the fix was a raceline asked for 0.25 m first (the widest the lane
allows), and AMCL's motion noise and update thresholds (racer_state/config/amcl.yaml). This test
fails on either: with the old 0.15 m ceiling or the old AMCL file the clockwise cases braked 28 and
95 times.

THRESHOLDS, from measured runs (3 laps; noise on; 0.64 m/s; vehicle_params 0.14.0):
  stadium 1.0 m lane, island radius 0.4 m:  ccw 14.8 s, cw 15.0 s; 0 brakes; minimum body
      clearance 0.194 / 0.199 m; localisation error mean 0.032 / 0.039 m, p95 0.051 / 0.077 m,
      heading p95 0.032 / 0.075 rad
  rounded rectangle 3 x 2 m, corner radius 0.4 m, 1.0 m lane:  ccw 19.1 s, cw 19.3 s; 0 brakes;
      clearance 0.198 / 0.192 m; error mean 0.034 / 0.038 m, p95 0.050 / 0.059 m, heading p95
      0.028 / 0.052 rad
Limits: the laps, no wall contact, at most 3 brakes (measured 0; the loaded runner of the gap_follow
canary showed that a TTC latch can engage a few times without a real problem; the failure being
caught is 28 to 95), clearance at least 0.12 m (measured 0.19 and up, the failing runs 0.07 to
0.09), mean localisation error at most 0.10 m (measured at most 0.04), heading p95 at most 0.15
rad (measured at most 0.075), a lap time within a factor 1.4 of the measured one. Skipped without
a ROS workspace.
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

STADIUM = (
    "stadium",
    {"lane_width_m": 1.0, "straight_length_m": 2.0, "island_radius_m": 0.4},
    14.9,
)
ROUNDED_RECT = (
    "rounded_rect",
    {
        "lane_width_m": 1.0,
        "straight_length_m": 3.0,
        "island_width_m": 2.0,
        "island_radius_m": 0.4,
    },
    19.2,
)
CLEARANCES = "0.25,0.20,0.15,0.12,0.10,0.08,0.06,0.05"
MAX_BRAKES = 3
MIN_CLEARANCE_M = 0.12
MAX_LOC_ERROR_MEAN_M = 0.10
MAX_HEADING_P95_RAD = 0.15
LAP_TIME_FACTOR = 1.4


@pytest.mark.parametrize("reverse", [False, True], ids=["ccw", "cw"])
@pytest.mark.parametrize("case", [STADIUM, ROUNDED_RECT], ids=["stadium", "rounded_rect"])
def test_localised_tracker_laps_without_a_safety_brake(case, reverse, tmp_path):
    track, params, measured_lap_s = case
    out = tmp_path / "result.json"
    command = [
        sys.executable,
        str(RUNNER),
        "--track",
        track,
        "--track-params-json",
        json.dumps(params),
        "--driver",
        "tracker_loc",
        "--stop-after-laps",
        str(LAPS),
        "--duration",
        "120",
        "--domain-id",
        "80",
        "--work-dir",
        str(tmp_path / "work"),
        "--raceline-json",
        json.dumps({"v_max_mps": 0.64}),
        "--clearances",
        CLEARANCES,
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
    assert result["raceline"]["curvature_violations"] == 0
    assert result["raceline"]["clearance_violations"] == 0
    assert result["laps"] >= LAPS, result.get("stopped_because")
    assert result["wall_contacts"] == 0
    assert result["obstacle_brakes"] <= MAX_BRAKES, result["safety_events"][:6]
    assert result["min_body_clearance_m"] >= MIN_CLEARANCE_M
    assert result["localisation_error_m"]["mean"] <= MAX_LOC_ERROR_MEAN_M
    assert result["localisation_error_split"]["heading_p95_abs_rad"] <= MAX_HEADING_P95_RAD
    lap = result["lap_time_s"]
    assert measured_lap_s / LAP_TIME_FACTOR <= lap <= measured_lap_s * LAP_TIME_FACTOR, lap
