"""L5 canary (brief 40): the obstacle development case OD is lapped through the real command path.

tests/l5_reactive_bench/track_family_runner.py, driver pnp, simulated time, car-like plant, on
case OD of pre-registration v0.4 (lane 1.0 m, a ROUND donut: straight 0.0 m, island radius
0.5 m, one solid box on the outer wall, 0.25 m deep and 0.30 m long, at 0.30 of the loop), counter
clockwise, seed 11, 3 laps at the lap-1 cap, lap memory off:

  * the geometric backend: 3 laps, 0 wall contacts;
  * the same with `--backend oracle` (the ground truth LocalTrack): 3 laps, 0 wall contacts, and
    the message on /pnp/local_track comes from oracle_track_node, not track_perception_node;
  * both: one publisher each on /drive_raw and /drive, and the box is in the run's record.

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
BUNDLE = REPO_ROOT / "config" / "pnp" / "fixtures" / "dyn-car-2026-11-12"
OD = {
    "lane_width_m": 1.0,
    "straight_length_m": 0.0,
    "island_radius_m": 0.5,
    "obstacles": [{"at": 0.30, "side": "outer", "protrusion_m": 0.25, "length_m": 0.30}],
}
LAP1_CAP_MPS = 0.5
LAPS = 3
SEED = 11
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


def run_od(tmp_path: Path, domain: int, backend: str) -> dict:
    out = tmp_path / "result.json"
    command = [
        sys.executable,
        str(RUNNER),
        "--track",
        "stadium",
        "--track-params-json",
        json.dumps(OD),
        "--driver",
        "pnp",
        "--backend",
        backend,
        "--dynamics-bundle",
        str(BUNDLE),
        *plant_flags(),
        "--v-max-mps",
        str(LAP1_CAP_MPS),
        "--lap-memory",
        "off",
        "--car",
        "params",
        "--seed",
        str(SEED),
        "--stop-after-laps",
        str(LAPS),
        "--sim-time",
        "--duration",
        "300",
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


def check(result: dict, backend: str, capsys: pytest.CaptureFixture[str]) -> None:
    gt = result["gt_lap_times_by_crossing_s"]
    with capsys.disabled():
        print(
            f"\nl5_pnp_oracle OD ccw {backend}: gt laps {gt} contacts {result['wall_contacts']} "
            f"safety_brakes {result['safety_brakes']} planner_states {result['planner_states']} "
            f"ticks {result.get('pnp_ticks')} passes {result.get('obstacle_passes')}"
        )
    assert len(gt) >= LAPS, result.get("stopped_because")
    assert result["wall_contacts"] == 0
    assert result["publisher_nodes"] == {
        "/drive_raw": ["mppi_planner_node"],
        "/drive": ["safety_node"],
    }
    assert str(STATE_FAULT) not in result["planner_states"], result["planner_states"]
    assert result["track_params"]["obstacles"][0]["side"] == "outer"
    assert len(result["obstacle_passes"]) == 1
    assert result["obstacle_passes"][0]["ticks"] > 0, "the car never came near the box"
    assert result["pnp_ticks"]["driving_ticks"] > 100


def test_od_is_lapped_with_the_geometric_backend(tmp_path, capsys):
    result = run_od(tmp_path, domain=86, backend="geometric")
    assert "track_perception_node" in result["pnp"]
    check(result, "geometric", capsys)


def test_od_is_lapped_with_the_oracle_track(tmp_path, capsys):
    result = run_od(tmp_path, domain=87, backend="oracle")
    assert "oracle_track_node" in result["pnp"]
    assert "track_perception_node" not in result["pnp"]
    assert (tmp_path / "work" / "ground_truth_track.npz").exists()
    check(result, "oracle", capsys)
