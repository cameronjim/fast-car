"""L5 smoke test of the reactive benchmark runner: one short real run, the result's shape.

Runs reactive_bench_runner.py on lane_open_side for a few seconds with the floor profile (the
bridge, safety_node and gap_follow_node as child processes) and checks the JSON result has every
metric with a sane value. It does NOT judge the follower: how well gap_follow_node drives is what
the battery (tools/reactive_bench.py) measures. Skipped without a sourced ROS workspace.
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

RUNNER = Path(__file__).with_name("reactive_bench_runner.py")

EXPECTED_KEYS = {
    "track",
    "profile",
    "laps",
    "lap_time_s",
    "obstacle_brakes",
    "escapes",
    "min_body_clearance_m",
    "wall_contacts",
    "mean_abs_steering_rad",
    "steering_reversals_per_min",
    "duration_s",
    "real_time_factor",
    "distance_m",
    "stopped_because",
}


def test_runner_reports_every_metric_for_a_short_run(tmp_path):
    out = tmp_path / "result.json"
    env = dict(os.environ)
    proc = subprocess.run(
        [
            sys.executable,
            str(RUNNER),
            "--track",
            "lane_open_side",
            "--profile",
            "floor-2026-10-06",
            "--duration",
            "8",
            "--domain-id",
            "77",
            "--out",
            str(out),
        ],
        env=env,
        capture_output=True,
        text=True,
        timeout=200,
        check=False,
    )
    assert proc.returncode == 0, proc.stdout[-2000:] + proc.stderr[-2000:]
    result = json.loads(out.read_text())
    assert EXPECTED_KEYS <= set(result), EXPECTED_KEYS - set(result)
    assert result["track"] == "lane_open_side"
    assert result["laps"] == 0  # eight seconds is not a lap of a 19 m loop at 0.9 m/s
    assert result["duration_s"] >= 7.9
    assert result["distance_m"] > 1.0, "the follower never drove"
    assert 0.0 < result["min_body_clearance_m"] <= 1.5
    assert result["mean_abs_steering_rad"] is not None
    assert result["stopped_because"] == "duration"
