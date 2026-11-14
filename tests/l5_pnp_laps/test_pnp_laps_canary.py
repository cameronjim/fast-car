"""L5 canary: lap learning (D) on the Plug-and-Play driver through the real command path.

tests/l5_reactive_bench/track_family_runner.py, driver pnp, with `--lap-memory v0`: bridge_node on
the tuning stadium (lane 1.0 m, straight 2.0 m, island radius 0.4 m) with the car's LiDAR and a
car-like plant (the fitted bundle's actuation, speed dead zone, steering lag), then
track_perception_node, mppi_planner_node, safety_node and racer_laps' lap_memory_node with the
START-SCAN lap source (no map, no pose: the lap position comes from the start scan and the wheel
odometry). Ground truth is used only to score.

Case 1 (counter-clockwise, seed 11, D v0, 6 laps from a standing start, lap-1 cap 0.5 m/s, ceiling
the planner bundle's validity): the system's lap count equals the ground-truth lap count, every
system lap time is within 0.3 s of the truth, 0 wall contacts, the cap rose at least once, the
only publisher on /drive_raw is mppi_planner_node and the only publisher on /drive is safety_node.

Case 3 (brief 29, development case T2: lane 1.0 m, straight 5.0 m, island radius 0.4 m, condition a,
D off at the lap-1 cap, 4 laps): rule D3-S0a of the v0.3 pre-registration: the system's lap count
equals the ground-truth count and every system lap time is within 0.3 s of the ground-truth lap
(differences of crossing stamps of the start line, `gt_lap_times_by_crossing_s`), 0 wall contacts.
Gate v1 closed laps metres early on straights like this one.

Case 2 (half a lap): the car is stopped after a little more than half a lap. A stadium looks the
same half a lap later, so a gate that only compared scans would close here; the heading condition
must keep it armed off. The system reports no lap.

When the planner's lap profile hook (brief 26) is on main the same case with D v1 is added below.
Skipped without a sourced ROS workspace and f1tenth_gym.
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
# The planner's model and, for the plant, the same fitted car: its actuation, its 0.22 m/s speed
# dead zone, its steering lag and command delay (io.nominal of the manifest).
BUNDLE = REPO_ROOT / "config" / "pnp" / "fixtures" / "dyn-car-2026-11-12"
TUNING_STADIUM = {"lane_width_m": 1.0, "straight_length_m": 2.0, "island_radius_m": 0.4}
LAP1_CAP_MPS = 0.5
LAPS = 6
SEED = 11
LAP_TIME_TOLERANCE_S = 0.3
MAX_SAFETY_BRAKES = 3
STATE_FAULT = 6


def plant_flags() -> list[str]:
    import yaml

    nominal = yaml.safe_load((BUNDLE / "manifest.yaml").read_text())["io"]["nominal"]
    return [
        "--plant-bundle",
        str(BUNDLE),
        "--plant-speed-deadband",
        str(nominal["speed_deadband_mps"]),
        "--plant-steer-tau",
        str(nominal["steer_tau_s"]),
        "--plant-command-delay",
        str(nominal["command_delay_s"]),
    ]


def ceiling_mps() -> float:
    import yaml

    return float(
        yaml.safe_load((BUNDLE / "manifest.yaml").read_text())["io"]["validity"]["v_max_mps"]
    )


def run_runner(
    tmp_path: Path,
    domain: int,
    mode: str,
    laps: int,
    duration_s: float,
    v_max_mps: float | None = None,
    track_params: dict | None = None,
) -> dict:
    out = tmp_path / "result.json"
    command = [
        sys.executable,
        str(RUNNER),
        "--track",
        "stadium",
        "--track-params-json",
        json.dumps(TUNING_STADIUM if track_params is None else track_params),
        "--driver",
        "pnp",
        "--backend",
        "geometric",
        "--dynamics-bundle",
        str(BUNDLE),
        *plant_flags(),
        "--v-max-mps",
        str(ceiling_mps() if v_max_mps is None else v_max_mps),
        "--v-lap1-mps",
        str(LAP1_CAP_MPS),
        "--lap-memory",
        mode,
        "--car",
        "params",
        "--seed",
        str(SEED),
        "--stop-after-laps",
        str(laps),
        "--duration",
        str(duration_s),
        "--domain-id",
        str(domain),
        "--work-dir",
        str(tmp_path / "work"),
        "--out",
        str(out),
    ]
    proc = subprocess.run(
        command, env=dict(os.environ), capture_output=True, text=True, timeout=900, check=False
    )
    assert out.exists(), proc.stdout[-2000:] + proc.stderr[-2000:]
    result = json.loads(out.read_text())
    assert "error" not in result, result["error"]
    return result


def check_laps(result: dict, mode: str, capsys: pytest.CaptureFixture[str]) -> None:
    gt = result["gt_lap_times_s"]
    system = [e["lap_time_s"] for e in result["system_lap_events"]]
    with capsys.disabled():
        print(
            f"\nl5_pnp_laps stadium ccw D {mode}: gt laps {gt} system laps {system} "
            f"caps {[round(e['cap_mps'], 2) for e in result['system_lap_events']]} "
            f"clean {[e['clean'] for e in result['system_lap_events']]} "
            f"safety_brakes {result['safety_brakes']} contacts {result['wall_contacts']} "
            f"planner_states {result['planner_states']}"
        )
    assert len(gt) >= LAPS, result.get("stopped_because")
    assert len(system) == len(gt), f"system {len(system)} laps, ground truth {len(gt)}"
    for k, (s, g) in enumerate(zip(system, gt, strict=True), start=1):
        assert abs(s - g) <= LAP_TIME_TOLERANCE_S, f"lap {k}: system {s:.2f} s, truth {g:.2f} s"
    assert result["wall_contacts"] == 0
    assert result["safety_brakes"] <= MAX_SAFETY_BRAKES
    assert result["publisher_nodes"] == {
        "/drive_raw": ["mppi_planner_node"],
        "/drive": ["safety_node"],
    }
    assert str(STATE_FAULT) not in result["planner_states"], result["planner_states"]
    caps = [e["cap_mps"] for e in result["system_lap_events"]]
    assert max(caps) > caps[0], f"the cap never rose: {caps}"
    assert caps[0] == pytest.approx(LAP1_CAP_MPS)
    assert max(caps) <= ceiling_mps() + 1e-6


def test_the_lap_memory_counts_the_laps_with_the_start_scan_source_and_the_cap_rises(
    tmp_path, capsys
):
    result = run_runner(tmp_path, domain=83, mode="v0", laps=LAPS, duration_s=300.0)
    assert all(e["frame_id"] == "start_scan" for e in result["system_lap_events"])
    check_laps(result, "v0", capsys)


def test_the_gate_does_not_close_at_half_a_lap(tmp_path, capsys):
    # D off at the lap-1 cap (condition a): the car drives about 0.45 m/s, so 15 s from the first
    # odometry sample is about 0.6 of the 9.65 m lap
    result = run_runner(
        tmp_path, domain=84, mode="off", laps=1, duration_s=15.0, v_max_mps=LAP1_CAP_MPS
    )
    loop = result["loop_length_m"]
    with capsys.disabled():
        print(
            f"\nl5_pnp_laps half lap: progress {result['progress_m']} of {loop} m, "
            f"system laps {result['system_laps']}, status {result['lap_status'].get('why_no_lap')}"
        )
    assert 0.5 * loop < result["progress_m"] < 0.95 * loop, "the run must pass half way, not finish"
    assert result["system_laps"] == 0
    assert result["lap_status"]["laps_completed"] == 0
    assert result["wall_contacts"] == 0


T2_STADIUM = {"lane_width_m": 1.0, "straight_length_m": 5.0, "island_radius_m": 0.4}
T2_LAPS = 4


def test_the_long_straight_stadium_t2_counts_every_lap_with_the_right_time(tmp_path, capsys):
    """D3-S0a on development case T2 (5.0 m straight), ccw, seed 11, condition a, 4 laps."""
    result = run_runner(
        tmp_path,
        domain=85,
        mode="off",
        laps=T2_LAPS,
        duration_s=400.0,
        v_max_mps=LAP1_CAP_MPS,
        track_params=T2_STADIUM,
    )
    gt = result["gt_lap_times_by_crossing_s"]
    system = [e["lap_time_s"] for e in result["system_lap_events"]]
    with capsys.disabled():
        print(
            f"\nl5_pnp_laps T2 ccw D off: gt laps {gt} system laps {system} "
            f"lengths {[round(e['length_m'], 2) for e in result['system_lap_events']]} "
            f"loop {result['loop_length_m']} contacts {result['wall_contacts']}"
        )
    assert len(gt) >= T2_LAPS, result.get("stopped_because")
    assert len(system) == len(gt), f"system {len(system)} laps, ground truth {len(gt)}"
    for k, (s, g) in enumerate(zip(system, gt, strict=True), start=1):
        assert abs(s - g) <= LAP_TIME_TOLERANCE_S, f"lap {k}: system {s:.2f} s, truth {g:.2f} s"
    assert result["wall_contacts"] == 0
