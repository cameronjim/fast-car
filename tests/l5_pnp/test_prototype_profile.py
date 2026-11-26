"""L5: the prototype profile (owner decision 29) runs end to end in sim.

One short fixed scenario through tests/l5_reactive_bench/track_family_runner.py with
`--pnp-profile prototype`, so every learned part runs as `pnp.launch.py profile:=prototype` starts
it on the car: net perception (trackperc-v2), learned dynamics dyn-v0.7p with dynamics_fast_mlp,
D v1 variant K3s, the session 2 lap settings (lap 1 cap 0.5, ceiling 1.3, length check 0.75 to
1.5). The camera lap source has no sim counterpart: the simulator's lap oracle stands in for it
(tests/l5_reactive_bench/pnp_profile.py). The bundles are committed copies under
fixtures/prototype; the test fails when the profile names a bundle that is not there, so a
profile change brings its fixture with it.

Scenario: stadium lane 1.0 m, straight 2.0 m, island radius 0.4 m (B11's T), counter-clockwise,
seed 11, the car-like plant (dyn-car-2026-11-12 actuation, 0.22 m/s dead zone, steering lag,
reverse braking like forward), the planner's sim_guidance_clock_skew_s at 0.05 s (on simulated
time each node gets /clock on its own, PR 268),
5 laps on simulated time.

Pass, never loosened to get green: 5 laps, 0 wall contacts, every driver node still running when
the run ends, no ERROR / FATAL line in a driver node's log, both
nets publishing (every LocalTrack from trackperc-v2 and planner status naming dyn-v0.7p and
trackperc-v2, the perception status says backend net), the lap memory guiding (a station bound
published after lap 1), the only publisher on /drive_raw is mppi_planner_node, the only publisher
on /drive is safety_node, and no STATE_FAULT. Solve time, sample cuts, V and lap times go to the
test output. Skipped without a sourced ROS workspace.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("f1tenth_gym")

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tests" / "l5_reactive_bench"))
import pnp_profile
from test_pnp_canary import STATE_FAULT, run_judged

RUNNER = REPO_ROOT / "tests" / "l5_reactive_bench" / "track_family_runner.py"
LAUNCH = REPO_ROOT / "ros_ws" / "src" / "racer_bringup" / "launch" / "pnp.launch.py"
MODELS = Path(__file__).resolve().parent / "fixtures" / "prototype"
PLANT = REPO_ROOT / "config" / "pnp" / "fixtures" / "dyn-car-2026-11-12"
TRACK = {"lane_width_m": 1.0, "straight_length_m": 2.0, "island_radius_m": 0.4}
LAPS = 5
SEED = 11
SIM_CLOCK_SKEW_S = 0.05
DRIVER_NODES = ("track_perception_node", "mppi_planner_node", "lap_memory_node", "safety_node")


def profile_bundles() -> dict[str, str]:
    proto = pnp_profile.load_profiles(LAUNCH)["prototype"]
    return {k: Path(proto[k]).name for k in ("perception_bundle", "dynamics_bundle")}


def test_the_profile_bundles_are_the_fixtures():
    for key, name in profile_bundles().items():
        assert (MODELS / name / "manifest.yaml").is_file(), (
            f"profile:=prototype names {key} {name}; copy that bundle to {MODELS}"
        )


def test_prototype_profile(tmp_path, capsys):
    bundles = profile_bundles()
    for name in bundles.values():
        assert (MODELS / name / "manifest.yaml").is_file(), MODELS / name
    out = tmp_path / "result.json"
    command = [
        sys.executable,
        str(RUNNER),
        "--track",
        "stadium",
        "--track-params-json",
        json.dumps(TRACK),
        "--driver",
        "pnp",
        "--pnp-profile",
        "prototype",
        "--pnp-models-dir",
        str(MODELS),
        "--plant-bundle",
        str(PLANT),
        "--plant-speed-deadband",
        "0.22",
        "--plant-steer-tau",
        "0.02787697420940658",
        "--plant-command-delay",
        "0.0",
        "--plant-mirror-reverse",
        # simulated time only: a lap profile stamped one sim step ahead of the planner's clock is
        # judged at age 0 instead of refused (PR 268); the car keeps the age >= 0 rule
        "--planner-params-json",
        json.dumps({"sim_guidance_clock_skew_s": SIM_CLOCK_SKEW_S}),
        "--car",
        "params",
        "--seed",
        str(SEED),
        "--stop-after-laps",
        str(LAPS),
        "--duration",
        "240",
        "--stall-s",
        "30",
        "--domain-id",
        "83",
        "--work-dir",
        str(tmp_path / "work"),
        "--out",
        str(out),
    ]

    def report(result):
        system = result.get("system", {})
        return (
            f"\nl5_pnp prototype stadium ccw: laps {result.get('laps')} "
            f"lap_times_s {result.get('gt_lap_times_s')} contacts {result.get('wall_contacts')} "
            f"safety_brakes {result.get('safety_brakes')} solve_ms {system.get('solve_ms')} "
            f"samples {system.get('samples')} valid_count_mean "
            f"{system.get('valid_count', {}).get('mean')} V<2 "
            f"{system.get('valid_count', {}).get('unusable')} "
            f"caps {[v.get('last_cap_mps') for v in (result.get('guidance_caps_by_lap') or {}).values()]}"
        )

    def judge(result):
        assert result.get("pnp_profile", {}).get("profile") == "prototype"
        planner = result["pnp"]["mppi_planner_node"]
        assert planner.get("sim_guidance_clock_skew_s") == SIM_CLOCK_SKEW_S, planner
        assert result["laps"] >= LAPS, result.get("stopped_because")
        assert result["wall_contacts"] == 0
        logs = result.get("node_logs", {})
        for name in DRIVER_NODES:
            assert result["alive_at_end"].get(name) is True, (name, result["alive_at_end"])
            # a traceback printed while the node is stopped after the run is not counted (rclpy
            # can raise in take_message during shutdown); one during the run kills the node
            bad = {k: logs[name].get(k, 0) for k in ("ERROR", "FATAL")}
            assert not any(bad.values()), (name, bad, logs[name].get("first_errors"))
        system = result["system"]
        assert set(system["track_model_ids"]) == {bundles["perception_bundle"]}
        assert system["valid_count"]["frames"] > 0
        assert set(system["planner_model_ids"]) == {
            f"{bundles['dynamics_bundle']}|{bundles['perception_bundle']}"
        }
        assert system["perception_status"].get("backend") == "net"
        assert system["perception_status"].get("model_id") == bundles["perception_bundle"]
        caps = result["guidance_caps_by_lap"]
        assert any(int(k) >= 2 and v["max_stations"] > 0 for k, v in caps.items()), caps
        assert result["publisher_nodes"] == {
            "/drive_raw": ["mppi_planner_node"],
            "/drive": ["safety_node"],
        }
        assert str(STATE_FAULT) not in result["planner_states"], result["planner_states"]

    run_judged(command, out, report, judge, capsys, timeout_s=600)
