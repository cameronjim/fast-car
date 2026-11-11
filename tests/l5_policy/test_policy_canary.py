"""L5 canary: the learned-residual path laps a family stadium, both directions, and behaves like
the tracker it rides on.

Each case runs tests/l5_reactive_bench/track_family_runner.py twice on the same world: once
with the tracker alone (driver `tracker`) and once with `tracker_policy`: tracker_node in
HANDOFF mode (drive_topic /tracker/command), racer_policy's policy_node, and safety_node on the
real command path, ground-truth pose, the car as measured on 2026-11-10, 1.0 m lane, 2 m
straights, island end radius 0.4 m, raceline at 0.64 m/s (the same world as
tests/l5_track_family).

The network is a FIXTURE, not a trained policy: a tiny TorchScript module with fixed small
weights whose steering output depends on the observation (so the residual is non-zero and the
whole path, observation builder, inference, envelope, publication, is exercised) and whose
speed output is zero (a lap-time comparison should not be dominated by an untrained speed
residual). The directory is a complete one: contract.yaml, policy_meta.yaml and policy.pt, and
it goes through `load_deployment` against the live vehicle_params inside policy_node, so a
refusal would fail this test.

Assertions, each with its reason:
* both arms lap twice with no wall contact and no safety brake (the residual is bounded to
  about 0.022 rad of a 0.449 rad range, so it must not turn a lapping tracker into a crashing
  one);
* policy_node is the only publisher of /drive_raw and safety_node the only publisher of /drive
  (CLAUDE.md invariant 1);
* the residual was really applied (non-zero) and never exceeded the envelope bound, the node
  stayed active (not in fallback) for nearly every step, nothing was clipped;
* inference median under 5 ms and p99 under two 50 Hz periods (a Python node on a loaded runner;
  the Jetson figure is a separate measurement);
* mean lap time within 8% of the tracker alone, whose own margin run to run is about 1%.

Skipped without a sourced ROS workspace, f1tenth_gym or torch (the fixture network needs it;
torch is in the car image, not in ros-dev).
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
pytest.importorskip("torch")

REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "tests" / "l5_reactive_bench" / "track_family_runner.py"
sys.path.insert(0, str(REPO / "ros_ws" / "src" / "racer_policy" / "tests"))

from policy_fixture import build_policy_dir, torch_policy_bytes

LAPS = 2
TRACK = {"lane_width_m": 1.0, "straight_length_m": 2.0, "island_radius_m": 0.4}
RACELINE = {"v_max_mps": 0.64, "body_clearance_m": 0.15}
LAP_TIME_TOLERANCE = 0.08
# A CPU-bound python node on a loaded CI runner: the median is the stable number, the p99 only
# has to stay clear of two control periods (the real figures are in the residual-v1 note).
MAX_INFERENCE_P50_US = 5000.0
MAX_INFERENCE_P99_US = 40000.0
MAX_FALLBACK_FRACTION = 0.05
# The envelope bound on steering: 0.025 of the committed range (policy_fixture).
MAX_RESIDUAL_STEERING_RAD = 0.025 * 0.898 + 1e-6


def _policy_dir(tmp_path: Path) -> Path:
    from racer_tools.vehicle_params_loader import load_vehicle_params

    def meta(m: dict) -> None:
        base = m["must_match_at_deploy"]["base_controller"]
        base.update(lookahead_min_m=0.35, lookahead_max_m=0.7, lookahead_curvature_ref_1pm=1.25)
        m["must_match_at_deploy"]["envelope"]["ood_reference_state"][0] = 0.64
        m["must_match_at_deploy"]["envelope"]["speed_cap_mps"] = 0.8

    return build_policy_dir(
        tmp_path,
        load_vehicle_params(),
        policy_bytes=torch_policy_bytes(scale=0.5, speed_gain=0.0),
        mutate_meta=meta,
    )


def _run(driver: str, reverse: bool, work: Path, policy_dir: Path | None) -> dict:
    out = work / "result.json"
    command = [
        sys.executable,
        str(RUNNER),
        "--track",
        "stadium",
        "--track-params-json",
        json.dumps(TRACK),
        "--driver",
        driver,
        "--car",
        "today",
        "--stop-after-laps",
        str(LAPS),
        "--duration",
        "120",
        "--domain-id",
        "98",
        "--work-dir",
        str(work / "run"),
        "--raceline-json",
        json.dumps(RACELINE),
        "--out",
        str(out),
    ]
    if policy_dir is not None:
        command += ["--policy-dir", str(policy_dir)]
    if reverse:
        command.append("--reverse")
    proc = subprocess.run(
        command, env=dict(os.environ), capture_output=True, text=True, timeout=400, check=False
    )
    assert out.exists(), proc.stdout[-2000:] + proc.stderr[-2000:]
    result = json.loads(out.read_text())
    assert "error" not in result, result["error"]
    return result


@pytest.mark.parametrize("reverse", [False, True], ids=["ccw", "cw"])
def test_tracker_plus_policy_laps_like_the_tracker_alone(reverse, tmp_path):
    (tmp_path / "a").mkdir()
    (tmp_path / "b").mkdir()
    (tmp_path / "p").mkdir()
    policy_dir = _policy_dir(tmp_path / "p")
    alone = _run("tracker", reverse, tmp_path / "a", None)
    policy = _run("tracker_policy", reverse, tmp_path / "b", policy_dir)

    for name, result in (("tracker", alone), ("tracker_policy", policy)):
        assert result["laps"] >= LAPS, (name, result.get("stopped_because"))
        assert result["wall_contacts"] == 0, name
        assert result["obstacle_brakes"] == 0, name

    pubs = policy["publishers"]
    assert pubs["/drive_raw"] == 1, pubs  # policy_node, not the tracker
    assert pubs["/drive"] == 1, pubs  # safety_node
    assert pubs["/tracker/command"] == 1, pubs

    status = policy["policy_status"]
    c = status["counters"]
    assert c["steps"] > 500
    assert c["fallback_steps"] <= MAX_FALLBACK_FRACTION * c["steps"], status
    assert c["clipped_steps"] == 0
    assert c["active_abs_residual_steering_max_rad"] > 1e-3  # the residual really was applied
    assert c["active_abs_residual_steering_max_rad"] <= MAX_RESIDUAL_STEERING_RAD
    assert c["active_abs_residual_speed_max_mps"] == 0.0
    assert status["inference_us"]["p50"] <= MAX_INFERENCE_P50_US
    assert status["inference_us"]["p99"] <= MAX_INFERENCE_P99_US
    assert status["loop_period_ms"]["n"] > 400

    ratio = policy["lap_time_s"] / alone["lap_time_s"]
    if report := os.environ.get("L5_POLICY_REPORT"):
        with open(report, "a", encoding="utf-8") as handle:
            handle.write(
                json.dumps(
                    {
                        "direction": "cw" if reverse else "ccw",
                        "lap_time_tracker_s": alone["lap_time_s"],
                        "lap_time_tracker_plus_policy_s": policy["lap_time_s"],
                        "ratio": ratio,
                        "min_clearance_tracker_m": alone.get("min_body_clearance_m"),
                        "min_clearance_policy_m": policy.get("min_body_clearance_m"),
                        "policy_counters": c,
                        "inference_us": status["inference_us"],
                        "loop_period_ms": status["loop_period_ms"],
                    }
                )
                + "\n"
            )
    assert abs(ratio - 1.0) <= LAP_TIME_TOLERANCE, (alone["lap_time_s"], policy["lap_time_s"])
