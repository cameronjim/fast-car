"""L5 canary: the 2026-11-16 indoor loop refusals, reproduced in the sim, and the profile that fixes
them (docs/notes/gap-follow-loop-2026-11-16.md).

On the floor on 2026-11-16, gap_follow with race-2026-11-12 (a_lat_max 2.0, 1.0 and 1.3 m/s) carried
speed into the turn on the indoor loop, turned in late, and safety_node's TTC gate refused forward
3 to 4 times per run, each ending in a reverse escape. The sim check before it (stock f1tenth_gym
plant) had no brake at all. The difference is lag: the gym car steers with no lag and acts on a
command the moment it arrives, and its LiDAR scan has no age. With about 0.25 s between a scan and
the wheels answering (a 0.15 s first order steering lag and a 0.1 s command delay, the runner's plant
flags) the sim brakes the way the car did; the car's speed response
(fixtures/plant-car-2026-11-16, the dynamics bundle's io.nominal) is added for realism but does not
cause it on its own (the note's ablation).

The track is the synthetic copy of the loop: `rounded_rect`, a 1.5 m lane round a 0.44 x 0.40 m
island with square corners, so the outer wall is a 3.4 m rounded square (the 2026-11-13 SLAM map of
the loop: lane median 1.50 m, island 0.44 x 0.40 m, loop 5.8 m). The map itself is not committed.

  * REPRODUCTION. race-2026-11-12 at a_lat_max 2.0 and 1.3 m/s (the G 1.3 run's settings), counter-
    clockwise as on the floor. With the gym plant: no brake (the old sim check). With the car's
    actuation: at least one TTC brake. Measured on this track with the car's actuation: 4, 7, 7, 25
    brakes counter-clockwise (seeds 1 to 4) and 9 clockwise in 3 laps; with the gym plant 0.
  * FIX. race-2026-11-16 with the car's actuation, both directions, and with a slower steering
    assumption (0.25 s lag, 0.15 s delay) counter-clockwise: 3 laps, no safety_node brake, no
    reverse escape, no wall contact. Measured: 0 brakes in all of them, laps 6.75 to 7.88 s, body
    clearance 0.25 to 0.35 m.

Zero is the requirement, not a band. If the fix runs ever brake in CI, quarantine the test and file
an issue per claude-docs/12-testing.md; do not raise the bound. Skipped without a ROS workspace.
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

HERE = Path(__file__).resolve().parent
RUNNER = HERE.parents[0] / "l5_reactive_bench" / "track_family_runner.py"
PLANT = HERE / "fixtures" / "plant-car-2026-11-16"
LAPS = 3

LOOP = {
    "lane_width_m": 1.5,
    "straight_length_m": 0.44,
    "island_width_m": 0.4,
    "island_radius_m": 0.0,
}
# The car's actuation, and the slower assumption the fix must also survive.
CAR = ["--plant-bundle", str(PLANT), "--plant-steer-tau", "0.15", "--plant-command-delay", "0.1"]
CAR_SLOW = [
    "--plant-bundle",
    str(PLANT),
    "--plant-steer-tau",
    "0.25",
    "--plant-command-delay",
    "0.15",
]
# The 2026-11-16 floor run G 1.3: race-2026-11-12 with these launch arguments.
FLOOR_RUN = {"a_lat_max_mps2": 2.0, "max_speed_mps": 1.3}

FIX_LAP_S = 7.3  # measured 6.75 to 7.88 s over the three fix cases
LAP_TIME_FACTOR = 1.5
FIX_MIN_CLEARANCE_M = 0.1  # measured 0.25 to 0.35 m


def _run(tmp_path: Path, domain: int, profile: str, params: dict, plant: list, reverse: bool):
    out = tmp_path / "result.json"
    command = [
        sys.executable,
        str(RUNNER),
        "--track",
        "rounded_rect",
        "--track-params-json",
        json.dumps(LOOP),
        "--driver",
        "gap_follow",
        "--profile",
        profile,
        "--params-json",
        json.dumps(params),
        "--stop-after-laps",
        str(LAPS),
        "--duration",
        "90",
        "--domain-id",
        str(domain),
        "--work-dir",
        str(tmp_path / "work"),
        "--out",
        str(out),
        *plant,
    ]
    if reverse:
        command.append("--reverse")
    proc = subprocess.run(
        command, env=dict(os.environ), capture_output=True, text=True, timeout=400, check=False
    )
    assert out.exists(), proc.stdout[-2000:] + proc.stderr[-2000:]
    result = json.loads(out.read_text())
    assert "error" not in result, result["error"]
    return result


def test_the_floor_run_does_not_brake_on_the_gym_plant(tmp_path):
    """Why the sim check before the floor run passed: the gym car stops and steers at once."""
    result = _run(tmp_path, 111, "race-2026-11-12", FLOOR_RUN, [], reverse=False)
    assert result["laps"] >= LAPS, result.get("stopped_because")
    assert result["wall_contacts"] == 0
    assert result["obstacle_brakes"] == 0


def test_the_floor_run_brakes_with_the_cars_actuation(tmp_path):
    """The reproduction: the same run with the car's actuation brakes, as it did on the floor."""
    result = _run(tmp_path, 112, "race-2026-11-12", FLOOR_RUN, CAR, reverse=False)
    assert result["wall_contacts"] == 0
    assert result["obstacle_brakes"] >= 1, result


@pytest.mark.parametrize(
    ("plant", "reverse"),
    [(CAR, False), (CAR, True), (CAR_SLOW, False)],
    ids=["car-ccw", "car-cw", "slow-steering-ccw"],
)
def test_race_2026_11_16_laps_the_loop_with_no_refusal(plant, reverse, tmp_path):
    domain = 113 + (1 if reverse else 0) + (2 if plant is CAR_SLOW else 0)
    result = _run(tmp_path, domain, "race-2026-11-16", {}, plant, reverse)
    assert result["laps"] >= LAPS, result.get("stopped_because")
    assert result["wall_contacts"] == 0
    assert result["obstacle_brakes"] == 0, result
    assert result["escapes"] == 0, result
    assert result["min_body_clearance_m"] >= FIX_MIN_CLEARANCE_M
    lap = result["lap_time_s"]
    assert FIX_LAP_S / LAP_TIME_FACTOR <= lap <= FIX_LAP_S * LAP_TIME_FACTOR, lap
