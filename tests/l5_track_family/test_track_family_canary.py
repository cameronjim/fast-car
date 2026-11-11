"""L5 canary: the raceline tracker laps two family tracks, both directions.

Each case runs tests/l5_reactive_bench/track_family_runner.py: bridge_node on the track, safety_node
and tracker_node on the real command path (no remap), the car as measured on 2026-11-10 (left
lock 0.504 rad, right lock 0.449 rad, the command mapping the tracker compensates for), the
tracker on ground-truth pose, the raceline made from the track's own map by tools/track_session
(the pipeline a track session runs: centreline, minimum-curvature line inside the lane with the
body kept off the walls, speed profile at 0.64 m/s, tonight's top true speed).

THRESHOLDS are from measured runs (docs/notes/track-session-2026-11-11.md, tracker rows), with
margin, at 2 laps:
  stadium, 1.0 m lane, 2 m straights, island end radius 0.4 m   lap 15.1 to 15.3 s, 0 brakes,
      minimum body clearance 0.115 to 0.116 m, both directions
  rounded rectangle, 1.0 m lane, 3 x 2 m island, corner radius 0.4 m   lap 19.0 to 19.1 s, 0
      brakes, minimum body clearance 0.119 to 0.121 m, both directions
Limits: the laps, no wall contact, no safety_node brake, clearance at least 0.06 m, mean lap time
within a factor 1.4 either side of the measured one (a different controller or a bridge change
shows up there; wall-clock jitter does not, the runs are real time at a real-time factor of 1.0).
Skipped without a sourced ROS workspace.
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
    15.2,
)
ROUNDED_RECT = (
    "rounded_rect",
    {
        "lane_width_m": 1.0,
        "straight_length_m": 3.0,
        "island_width_m": 2.0,
        "island_radius_m": 0.4,
    },
    19.05,
)
MIN_CLEARANCE_M = 0.06
LAP_TIME_FACTOR = 1.4


@pytest.mark.parametrize("reverse", [False, True], ids=["ccw", "cw"])
@pytest.mark.parametrize("case", [STADIUM, ROUNDED_RECT], ids=["stadium", "rounded_rect"])
def test_tracker_laps_the_track_without_touching_a_wall_or_braking(case, reverse, tmp_path):
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
        "tracker",
        "--car",
        "today",
        "--stop-after-laps",
        str(LAPS),
        "--duration",
        "120",
        "--domain-id",
        "78",
        "--work-dir",
        str(tmp_path / "work"),
        "--raceline-json",
        json.dumps({"v_max_mps": 0.64, "body_clearance_m": 0.15}),
        "--out",
        str(out),
    ]
    if reverse:
        command.append("--reverse")
    proc = subprocess.run(
        command, env=dict(os.environ), capture_output=True, text=True, timeout=400, check=False
    )
    assert out.exists(), proc.stdout[-2000:] + proc.stderr[-2000:]
    result = json.loads(out.read_text())
    assert "error" not in result, result["error"]
    # Nothing in the line itself broke a limit.
    assert result["raceline"]["curvature_violations"] == 0
    assert result["raceline"]["clearance_violations"] == 0
    assert result["laps"] >= LAPS, result.get("stopped_because")
    assert result["wall_contacts"] == 0
    assert result["obstacle_brakes"] == 0
    assert result["escapes"] == 0
    assert result["min_body_clearance_m"] >= MIN_CLEARANCE_M
    lap = result["lap_time_s"]
    assert measured_lap_s / LAP_TIME_FACTOR <= lap <= measured_lap_s * LAP_TIME_FACTOR, lap
