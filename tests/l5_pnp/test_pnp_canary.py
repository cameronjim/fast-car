"""L5 canary: the Plug-and-Play driver laps tracks through the real command path.

Each case runs tests/l5_reactive_bench/track_family_runner.py with driver pnp: bridge_node on the
track with the car's LiDAR (mounted at the vehicle_params pose, 720 beams over 360 degrees,
10 Hz, 2 cm range noise) and a 50 Hz /odom/wheel, then track_perception_node,
mppi_planner_node and safety_node, nothing remapped. No map, no pose and no raceline: the driver
sees only the scan and the wheel speed.

Cases, each 3 laps at 0.6 m/s, car `params`, seed 11:
  stadium (lane 1.0 m, straight 2.0 m, island radius 0.4 m)   ccw and cw   geometric backend
  rounded rectangle (lane 1.0 m, 3 x 2 m island, radius 0.4 m)   ccw        geometric backend
  stadium, ccw, net backend with the released bundle from racer_track_perception's fixtures
      (skipped, with the reason, when that fixture directory is absent)

Pass criteria per case, never loosened to get green: 3 laps, 0 wall contacts, at most 3 safety
brake engagements (/safety/events PHASE_ENGAGE with SEVERITY_BRAKE), no rate_limit event from
safety_node (the planner's output rate limit stays inside it), the only publisher on
/drive_raw is mppi_planner_node, the only publisher on /drive is safety_node, and no STATE_FAULT in
/pnp/planner/status. Lap times and the brake count go to the test output. Skipped without a
sourced ROS workspace.
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

REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER = REPO_ROOT / "tests" / "l5_reactive_bench" / "track_family_runner.py"
DYNAMICS_BUNDLE = REPO_ROOT / "config" / "pnp" / "fixtures" / "dyn-nominal-v0"
PERCEPTION_BUNDLE = (
    REPO_ROOT / "ros_ws" / "src" / "racer_track_perception" / "test" / "fixtures" / "bundle_release"
)
LAPS = 3
V_MAX_MPS = 0.6
SEED = 11
MAX_SAFETY_BRAKES = 3
STATE_FAULT = 6

STADIUM = (
    "stadium",
    {"lane_width_m": 1.0, "straight_length_m": 2.0, "island_radius_m": 0.4},
)
ROUNDED_RECT = (
    "rounded_rect",
    {
        "lane_width_m": 1.0,
        "straight_length_m": 3.0,
        "island_width_m": 2.0,
        "island_radius_m": 0.4,
    },
)

CASES = [
    pytest.param(STADIUM, False, "geometric", id="stadium-ccw-geometric"),
    pytest.param(STADIUM, True, "geometric", id="stadium-cw-geometric"),
    pytest.param(ROUNDED_RECT, False, "geometric", id="rounded_rect-ccw-geometric"),
    pytest.param(STADIUM, False, "net", id="stadium-ccw-net"),
]


@pytest.mark.parametrize(("track", "reverse", "backend"), CASES)
def test_pnp_laps_the_track_through_the_real_command_path(
    track, reverse, backend, tmp_path, capsys
):
    if backend == "net" and not (PERCEPTION_BUNDLE / "manifest.yaml").is_file():
        pytest.skip(f"released perception bundle not committed yet: {PERCEPTION_BUNDLE}")
    assert (DYNAMICS_BUNDLE / "manifest.yaml").is_file(), DYNAMICS_BUNDLE
    name, params = track
    out = tmp_path / "result.json"
    command = [
        sys.executable,
        str(RUNNER),
        "--track",
        name,
        "--track-params-json",
        json.dumps(params),
        "--driver",
        "pnp",
        "--backend",
        backend,
        "--dynamics-bundle",
        str(DYNAMICS_BUNDLE),
        "--v-max-mps",
        str(V_MAX_MPS),
        "--car",
        "params",
        "--seed",
        str(SEED),
        "--stop-after-laps",
        str(LAPS),
        "--duration",
        "150",
        "--domain-id",
        "81",
        "--work-dir",
        str(tmp_path / "work"),
        "--out",
        str(out),
    ]
    if backend == "net":
        command += ["--perception-bundle", str(PERCEPTION_BUNDLE)]
    if reverse:
        command.append("--reverse")
    proc = subprocess.run(
        command, env=dict(os.environ), capture_output=True, text=True, timeout=500, check=False
    )
    assert out.exists(), proc.stdout[-2000:] + proc.stderr[-2000:]
    result = json.loads(out.read_text())
    assert "error" not in result, result["error"]
    with capsys.disabled():
        print(
            f"\nl5_pnp {name} {'cw' if reverse else 'ccw'} {backend}: "
            f"laps {result['laps']} lap_times_s {result['lap_times_s']} "
            f"safety_brakes {result['safety_brakes']} "
            f"rate_limit_events {result['rate_limit_events']} "
            f"wall_contacts {result['wall_contacts']} "
            f"min_clearance_m {result['min_body_clearance_m']} "
            f"planner_states {result['planner_states']}"
        )
    assert result["laps"] >= LAPS, result.get("stopped_because")
    assert result["wall_contacts"] == 0
    assert result["safety_brakes"] <= MAX_SAFETY_BRAKES
    assert result["rate_limit_events"] == 0, "safety_node rate-limited the planner's command"
    assert result["publisher_nodes"] == {
        "/drive_raw": ["mppi_planner_node"],
        "/drive": ["safety_node"],
    }
    assert str(STATE_FAULT) not in result["planner_states"], result["planner_states"]


# --- the car-like plant (2026-11-12) ----------------------------------------------------------
# The cases above run on the gym's own plant, which is nicer than the car: no speed dead zone and
# no steering lag. These run the same driver, perception and planner against a plant like the
# car's: the dynamics bundle's actuation (gains, accel and decel limits), a 0.20 m/s speed dead
# zone (a /drive speed under it leaves the car standing) and a 0.10 s steering lag. The planner's
# model knows the dead zone (the bundle's speed_deadband_mps, so its speed floor sits above it) and
# not the steering lag, which is the situation on the car since planner v0.1. Stadium
# with island radius 0.4 m and straight 2.0 m, lane 1.0 m and 1.3 m, both ways, geometric
# backend, 0.5 m/s, and the scan-only fault case. Same pass criteria as above.
#
# Measured on unmodified main (2026-11-12, v0 driver, car `params`, seed 11), per case:
#   lane 1.0 ccw  passes in 6 of 7 runs (0 to 2 safety brakes); the seventh stalled after one lap
#                 (94 brakes, BLOCKED and NO_ODOM), so this case can fail on a loaded machine
#   lane 1.0 cw   3 laps, 0 brakes, 0 contacts
#   lane 1.3 ccw  3 laps, 0 brakes, 0 contacts
#   lane 1.3 cw   3 laps, 0 brakes, 0 contacts
#   lane 1.3 ccw with --no-perception: 0 laps, planner_states NO_TRACK about 600 ticks (and NO_ODOM
#                 at the start), stalled (v0). Since planner v0.1 the planner drives on the scan
#                 alone and the case is an ordinary passing one.
CAR_LIKE_V_MAX_MPS = 0.5
CAR_LIKE_PLANT = [
    "--plant-bundle",
    str(DYNAMICS_BUNDLE),
    "--plant-speed-deadband",
    "0.20",
    "--plant-steer-tau",
    "0.10",
]


def stadium(lane_width_m: float) -> tuple[str, dict]:
    return (
        "stadium",
        {"lane_width_m": lane_width_m, "straight_length_m": 2.0, "island_radius_m": 0.4},
    )


CAR_LIKE_CASES = [
    pytest.param(stadium(1.0), False, True, id="car-like-stadium-lane1.0-ccw"),
    pytest.param(stadium(1.0), True, True, id="car-like-stadium-lane1.0-cw"),
    pytest.param(stadium(1.3), False, True, id="car-like-stadium-lane1.3-ccw"),
    pytest.param(stadium(1.3), True, True, id="car-like-stadium-lane1.3-cw"),
    pytest.param(stadium(1.3), False, False, id="car-like-stadium-lane1.3-ccw-no-perception"),
]


@pytest.mark.parametrize(("track", "reverse", "perception"), CAR_LIKE_CASES)
def test_pnp_laps_with_the_car_like_plant(track, reverse, perception, tmp_path, capsys):
    assert (DYNAMICS_BUNDLE / "manifest.yaml").is_file(), DYNAMICS_BUNDLE
    name, params = track
    out = tmp_path / "result.json"
    command = [
        sys.executable,
        str(RUNNER),
        "--track",
        name,
        "--track-params-json",
        json.dumps(params),
        "--driver",
        "pnp",
        "--backend",
        "geometric",
        "--dynamics-bundle",
        str(DYNAMICS_BUNDLE),
        "--v-max-mps",
        str(CAR_LIKE_V_MAX_MPS),
        "--car",
        "params",
        *CAR_LIKE_PLANT,
        "--seed",
        str(SEED),
        "--stop-after-laps",
        str(LAPS),
        "--duration",
        "150",
        "--domain-id",
        "82",
        "--work-dir",
        str(tmp_path / "work"),
        "--out",
        str(out),
    ]
    if not perception:
        command.append("--no-perception")
    if reverse:
        command.append("--reverse")
    proc = subprocess.run(
        command, env=dict(os.environ), capture_output=True, text=True, timeout=500, check=False
    )
    assert out.exists(), proc.stdout[-2000:] + proc.stderr[-2000:]
    result = json.loads(out.read_text())
    assert "error" not in result, result["error"]
    with capsys.disabled():
        print(
            f"\nl5_pnp car-like {name} lane {params['lane_width_m']} "
            f"{'cw' if reverse else 'ccw'}{'' if perception else ' no-perception'}: "
            f"laps {result['laps']} lap_times_s {result['lap_times_s']} "
            f"safety_brakes {result['safety_brakes']} "
            f"rate_limit_events {result['rate_limit_events']} "
            f"wall_contacts {result['wall_contacts']} "
            f"min_clearance_m {result['min_body_clearance_m']} "
            f"planner_states {result['planner_states']}"
        )
    assert result["laps"] >= LAPS, result.get("stopped_because")
    assert result["wall_contacts"] == 0
    assert result["safety_brakes"] <= MAX_SAFETY_BRAKES
    assert result["rate_limit_events"] == 0, "safety_node rate-limited the planner's command"
    assert result["publisher_nodes"] == {
        "/drive_raw": ["mppi_planner_node"],
        "/drive": ["safety_node"],
    }
    assert str(STATE_FAULT) not in result["planner_states"], result["planner_states"]
