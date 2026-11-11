"""L1 tests for the pure parts of tools/gap_follow_latency_bench.py (the ROS half runs in the
ros-dev container and is exercised by the node launch tests, which read the same status topic)."""

from __future__ import annotations

import json

import gap_follow_latency_bench as bench


def test_latency_bench_writes_a_typed_parameter_file():
    text = bench.params_yaml({"max_speed_mps": 1.0, "physics_speed": True, "target_mode": "x"})
    loaded = json.loads(text)["gap_follow_node"]["ros__parameters"]
    assert loaded == {"max_speed_mps": 1.0, "physics_speed": True, "target_mode": "x"}
    assert isinstance(loaded["max_speed_mps"], float)  # a float stays a float for the node


def test_latency_bench_table_and_default_configurations():
    status = {
        "scans": 400,
        "latency_ms": {"n": 200, "p50": 1.5, "p99": 4.25, "max": 9.0},
        "scan_wall_us": {"n": 200, "p50": 700.0, "p99": 1500.0, "max": 2000.0},
        "scan_cpu_us": {"n": 200, "p50": 650.0, "p99": 1400.0, "max": 1900.0},
    }
    table = bench.format_table(
        {"race": {"status": status, "commands": 2000}, "dead": {"status": None, "commands": 0}}
    )
    assert "| race | 400 | 1.50 / 4.25 / 9.00 | 700 / 1500 | 650 / 1400 | 2000 |" in table
    assert "no status received" in table
    assert set(bench.DEFAULT_CONFIGS) == {"lane_rec", "race_timer", "race_event"}
    # The comparison is against the recommended pre-race setup, with the race profile twice.
    assert bench.DEFAULT_CONFIGS["lane_rec"]["params"] == {
        "lane_lookahead_m": 0.8,
        "outside_entry_offset_m": 0.25,
    }
    assert bench.DEFAULT_CONFIGS["race_event"]["params"] == {"compute_on_scan": True}
