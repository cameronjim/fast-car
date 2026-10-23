"""L1 tests for the runner's pure setup logic (reactive_bench_runner.py), no ROS.

How parameters reach the three nodes: the named profile with overrides on top (types kept, the
simulator's scan alignment forced), the bridge's track parameters, and the params file the
children are started with. The stack itself is covered by test_reactive_bench_smoke.py.
"""

from __future__ import annotations

import pytest
import reactive_bench_runner as runner
import yaml

PROFILES = {
    "none": {"max_speed_mps": 2.0},
    "floor": {"max_speed_mps": 0.9, "steering_gain": 1.6, "reverse_escape": True, "scans": 5},
}


def test_profile_with_overrides_on_top_and_the_sim_scan_alignment_forced():
    params = runner.gap_follow_parameters("floor", {"steering_gain": 2.4, "new": 1.0}, PROFILES)
    assert params["steering_gain"] == 2.4 and params["new"] == 1.0
    assert params["max_speed_mps"] == 0.9 and params["reverse_escape"] is True
    assert params["laser_yaw_from_vehicle_params"] is False


def test_an_integer_override_of_a_float_parameter_becomes_a_float():
    # A double parameter rejects an int; a bool or an int parameter keeps its type.
    params = runner.gap_follow_parameters(
        "floor", {"steering_gain": 2, "reverse_escape": False, "scans": 3}, PROFILES
    )
    assert params["steering_gain"] == 2.0 and isinstance(params["steering_gain"], float)
    assert params["reverse_escape"] is False
    assert params["scans"] == 3 and isinstance(params["scans"], int)


def test_the_profile_table_is_not_modified():
    runner.gap_follow_parameters("floor", {"steering_gain": 9.0}, PROFILES)
    assert PROFILES["floor"]["steering_gain"] == 1.6
    assert "laser_yaw_from_vehicle_params" not in PROFILES["floor"]


def test_unknown_profile_is_refused():
    with pytest.raises(ValueError, match="profile"):
        runner.gap_follow_parameters("nope", {}, PROFILES)


def test_bridge_parameters_map_track_options_to_bridge_names_as_floats():
    params = runner.bridge_parameters(
        "lane_open_side", {"box_gap_m": 1, "lane_width_m": 1.0}, seed=7, reverse=True
    )
    assert params == {
        "seed": 7,
        "synthetic_track": "lane_open_side",
        "reverse_direction": True,
        "synthetic_box_gap_m": 1.0,
        "synthetic_lane_width_m": 1.0,
    }
    assert isinstance(params["synthetic_box_gap_m"], float)


def test_bridge_parameters_refuse_an_unknown_track_option():
    with pytest.raises(ValueError, match="unknown track parameter"):
        runner.bridge_parameters("chicane", {"jog_size": 0.5}, seed=1, reverse=False)


def test_every_track_option_the_cli_documents_exists_on_the_bridge():
    # The mapping's targets are the bridge's declared parameter names.
    bridge_source = (runner.REPO_ROOT / "sim/bridge/racer_gym_bridge/racer_gym_bridge").joinpath(
        "bridge_node.py"
    )
    text = bridge_source.read_text()
    for name in runner._BRIDGE_TRACK_KEYS.values():
        assert f'"{name}"' in text, name


def test_params_file_round_trips_the_types_a_ros_parameter_needs(tmp_path):
    path = tmp_path / "params.yaml"
    runner._write_params_file(
        path,
        {"gap_follow_node": {"a": 0.5, "b": 1.0, "c": 3, "d": True, "e": False}},
    )
    loaded = yaml.safe_load(path.read_text())["gap_follow_node"]["ros__parameters"]
    assert loaded == {"a": 0.5, "b": 1.0, "c": 3, "d": True, "e": False}
    assert isinstance(loaded["b"], float) and isinstance(loaded["c"], int)


def test_the_default_durations_cover_every_track():
    assert set(runner.DEFAULT_DURATION_S) == set(runner.TRACKS)
    assert all(t > 0 for t in runner.DEFAULT_DURATION_S.values())
