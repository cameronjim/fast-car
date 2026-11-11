"""L1 check of gap_follow.launch.py's `profile` argument (2026-10-06 floor checkpoint).

Loads the SOURCE launch file as a module, runs its OpaqueFunction against a LaunchContext and
inspects the parameters the node would be started with; no process is started and no ROS
graph is needed (same approach as racer_bringup/test/test_car_teleop_profile.py). What it
pins:

  * `profile:=none` (the default) is the old behaviour: no profile entry, max_speed_mps 1.43.
  * `profile:=floor-2026-10-06` hands the node exactly the checkpoint parameter set, with the
    right types (a double parameter rejects an int, an integer parameter rejects a float).
  * any launch argument given explicitly overrides the profile value, the profile overrides
    params_file, and an unknown profile is refused.
"""

from __future__ import annotations

import importlib.util
import pathlib

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

_LAUNCH_FILE = pathlib.Path(__file__).resolve().parents[1] / "launch" / "gap_follow.launch.py"

# The checkpoint, typed out again on purpose: an edit to the profile in the launch file must
# also edit this test (and the checkpoint table in
# docs/notes/reactive-control-port-2026-10-05.md).
_FLOOR_2026_10_06 = {
    "min_speed_mps": 0.36,  # was 0.5 before the 2026-11-10 calibration (old 0.5 / 1.401)
    "max_speed_mps": 0.64,  # was 0.9 (old 0.9 / 1.401): speeds are true speeds since schema 0.14.0
    "k_steer": 0.4,
    "free_space_threshold_m": 0.6,
    "steering_gain": 1.6,
    "steering_time_constant_s": 0.15,
    "disparity_threshold_m": 0.3,
    "cone_half_angle_rad": 1.2,
    "forward_preference": 0.0,
    "gap_switch_margin": 0.0,
    "swept_path_clamp": True,
    "swept_path_lookahead_m": 0.6,
    "corner_sector_inner_rad": 0.4,
    "corner_sector_outer_rad": 1.6,
    "corner_min_clearance_m": 0.35,
    "speed_rate_limit_margin_fraction": 0.1,
    "target_deepest_ray": False,
    "speed_time_constant_s": 0.5,
    "target_range_median_scans": 5,
    "centering_gain": 0.6,
    "centering_sector_half_angle_rad": 1.0,
    "centering_max_range_m": 1.5,
    "reverse_escape": True,
    "escape_probe_distance_m": 0.5,
}
_INT_PARAMETERS = {"target_range_median_scans"}
_BOOL_PARAMETERS = {"swept_path_clamp", "target_deepest_ray", "reverse_escape"}


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("gap_follow_launch", _LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _declared_arguments(description) -> dict[str, DeclareLaunchArgument]:
    return {e.name: e for e in description.entities if isinstance(e, DeclareLaunchArgument)}


def _node_parameters(**launch_arguments: str) -> list:
    """Evaluated parameter entries (file paths and dicts, in order) for these arguments."""
    description = _load_launch_module().generate_launch_description()
    context = LaunchContext()
    for name, argument in _declared_arguments(description).items():
        default = "".join(s.perform(context) for s in argument.default_value)
        context.launch_configurations[name] = default
    context.launch_configurations.update(launch_arguments)
    opaque = [e for e in description.entities if isinstance(e, OpaqueFunction)]
    assert len(opaque) == 1
    nodes = [e for e in opaque[0].execute(context) if isinstance(e, Node)]
    assert len(nodes) == 1
    return list(evaluate_parameters(context, nodes[0]._Node__parameters))


def _merged(entries: list) -> dict:
    """What the node ends up with from the dict entries: later entries win, as in rclcpp."""
    merged: dict = {}
    for entry in entries:
        if isinstance(entry, dict):
            merged.update(entry)
    return merged


def test_profile_argument_defaults_to_none_with_both_choices():
    module = _load_launch_module()
    args = _declared_arguments(module.generate_launch_description())
    assert "profile" in args
    context = LaunchContext()
    assert "".join(s.perform(context) for s in args["profile"].default_value) == "none"
    names = {"none", "floor-2026-10-06", "lane-2026-10-25", "race-2026-11-12"}
    assert set(args["profile"].choices) == names
    assert set(module.PROFILES) == names


def test_floor_profile_constant_is_the_checkpoint():
    module = _load_launch_module()
    assert module.FLOOR_2026_10_06_PROFILE == _FLOOR_2026_10_06
    assert module.PROFILES["floor-2026-10-06"] is module.FLOOR_2026_10_06_PROFILE
    assert module.PROFILES["lane-2026-10-25"] is module.LANE_2026_10_25_PROFILE
    lane = module.LANE_2026_10_25_PROFILE
    assert lane["target_mode"] == "lane_centre"
    assert lane["lane_lookahead_m"] == 0.6
    assert lane["swept_path_clamp"] is True
    for key, value in module.FLOOR_2026_10_06_PROFILE.items():
        if key not in ("target_mode", "lane_lookahead_m"):
            assert lane[key] == value, key
    assert module.PROFILES["none"] == {}


def test_none_profile_is_the_old_behaviour():
    entries = _node_parameters()
    assert entries == [{"max_speed_mps": 1.43}]
    assert isinstance(entries[0]["max_speed_mps"], float)


def test_none_profile_with_params_file_and_explicit_max_speed():
    entries = _node_parameters(params_file="/tmp/p.yaml", max_speed_mps="1")
    assert len(entries) == 2
    assert str(entries[0]) == "/tmp/p.yaml"
    assert entries[1] == {"max_speed_mps": 1.0}
    assert isinstance(entries[1]["max_speed_mps"], float)


def test_floor_profile_reaches_the_node_with_the_right_types():
    merged = _merged(_node_parameters(profile="floor-2026-10-06"))
    assert merged == _FLOOR_2026_10_06
    for name, value in merged.items():
        if name in _INT_PARAMETERS:
            assert type(value) is int, name
        elif name in _BOOL_PARAMETERS:
            assert type(value) is bool, name
        else:
            assert type(value) is float, name


def test_explicit_launch_arguments_override_the_profile():
    merged = _merged(
        _node_parameters(
            profile="floor-2026-10-06",
            max_speed_mps="0.7",
            forward_preference="0.6",
            gap_switch_margin="0.2",
            swept_path_clamp="false",
            swept_path_lookahead_m="1",
            speed_time_constant_s="0",
            target_range_median_scans="3",
            centering_gain="0",
            reverse_escape="false",
            escape_probe_distance_m="0.8",
            target_mode="lane_centre",
            lane_lookahead_m="1",
            lane_default_half_width_m="0.5",
        )
    )
    expected = dict(_FLOOR_2026_10_06)
    expected.update(
        max_speed_mps=0.7,
        forward_preference=0.6,
        gap_switch_margin=0.2,
        swept_path_clamp=False,
        swept_path_lookahead_m=1.0,
        speed_time_constant_s=0.0,
        target_range_median_scans=3,
        centering_gain=0.0,
        reverse_escape=False,
        escape_probe_distance_m=0.8,
        target_mode="lane_centre",
        lane_lookahead_m=1.0,
        lane_default_half_width_m=0.5,
    )
    assert merged == expected
    assert type(merged["target_mode"]) is str
    assert type(merged["lane_lookahead_m"]) is float
    assert type(merged["lane_default_half_width_m"]) is float
    assert type(merged["swept_path_lookahead_m"]) is float
    assert type(merged["speed_time_constant_s"]) is float
    assert type(merged["target_range_median_scans"]) is int
    assert type(merged["centering_gain"]) is float
    assert type(merged["reverse_escape"]) is bool
    assert type(merged["escape_probe_distance_m"]) is float


def test_profile_comes_after_params_file_and_before_explicit_arguments():
    entries = _node_parameters(
        profile="floor-2026-10-06", params_file="/tmp/p.yaml", max_speed_mps="0.7"
    )
    assert str(entries[0]) == "/tmp/p.yaml"
    assert entries[1] == _FLOOR_2026_10_06
    assert entries[2] == {"max_speed_mps": 0.7}


def test_unknown_profile_is_refused():
    with pytest.raises(ValueError, match="profile"):
        _node_parameters(profile="floor-2026-10-07")


def test_median_scans_must_be_an_integer():
    with pytest.raises(ValueError):
        _node_parameters(target_range_median_scans="5.0")


def test_target_mode_must_be_one_of_the_three_names():
    for mode in ("gap_centre", "gap_deepest", "lane_centre"):
        assert _merged(_node_parameters(target_mode=mode))["target_mode"] == mode
    with pytest.raises(ValueError, match="target_mode"):
        _node_parameters(target_mode="lane_center")
    # Not set: nothing is passed, the profile or node default applies.
    assert "target_mode" not in _merged(_node_parameters())


# -- race profile and the race launch arguments (2026-11-12) ---------------------------------------

_RACE_ONLY_PARAMETERS = {
    "physics_speed",
    "a_lat_max_mps2",
    "stopping_margin_m",
    "side_cap_distance_m",
    "side_cap_min_speed_mps",
    "apex_offset_m",
    "track_out_offset_m",
    "compute_on_scan",
}


def test_old_profiles_carry_no_race_parameter_so_their_behaviour_is_unchanged():
    module = _load_launch_module()
    for name in ("none", "floor-2026-10-06", "lane-2026-10-25"):
        assert _RACE_ONLY_PARAMETERS.isdisjoint(module.PROFILES[name]), name
        merged = _merged(_node_parameters(profile=name))
        assert _RACE_ONLY_PARAMETERS.isdisjoint(merged), name
        assert "status_rate_hz" not in merged
    # The lane profile still has the values it had, and the old defaults are the old 0.6 / off.
    lane = module.LANE_2026_10_25_PROFILE
    assert lane["lane_lookahead_m"] == 0.6
    assert "outside_entry_offset_m" not in lane


def test_race_profile_is_the_lane_profile_with_the_recommended_settings_and_the_race_features():
    module = _load_launch_module()
    race = module.RACE_2026_11_12_PROFILE
    lane = module.LANE_2026_10_25_PROFILE
    assert module.PROFILES["race-2026-11-12"] is race
    for key, value in lane.items():
        if key not in ("lane_lookahead_m",):
            assert race[key] == value or key in race, key
    assert race["target_mode"] == "lane_centre"
    assert race["lane_lookahead_m"] == 0.8  # the recommended settings, now inside the profile
    assert race["outside_entry_offset_m"] == 0.25
    assert race["physics_speed"] is True
    assert race["speed_time_constant_s"] == 0.0  # a low-pass would delay the stopping limit
    assert race["swept_path_clamp"] is True and race["reverse_escape"] is True


def test_race_profile_reaches_the_node_with_the_right_types():
    merged = _merged(_node_parameters(profile="race-2026-11-12"))
    for name, value in merged.items():
        if name in _INT_PARAMETERS:
            assert type(value) is int, name
        elif name in _BOOL_PARAMETERS | {"physics_speed", "compute_on_scan"}:
            assert type(value) is bool, name
        elif name == "target_mode":
            assert type(value) is str
        else:
            assert type(value) is float, name


def test_race_launch_arguments_are_forwarded_with_their_types_and_override_the_profile():
    merged = _merged(
        _node_parameters(
            profile="race-2026-11-12",
            physics_speed="false",
            compute_on_scan="true",
            a_lat_max_mps2="3",
            stopping_margin_m="0.1",
            side_cap_distance_m="0",
            side_cap_min_speed_mps="1",
            apex_offset_m="0",
            track_out_offset_m="0.1",
            status_rate_hz="2",
        )
    )
    assert merged["physics_speed"] is False
    assert merged["compute_on_scan"] is True
    for name, value in (
        ("a_lat_max_mps2", 3.0),
        ("stopping_margin_m", 0.1),
        ("side_cap_distance_m", 0.0),
        ("side_cap_min_speed_mps", 1.0),
        ("apex_offset_m", 0.0),
        ("track_out_offset_m", 0.1),
        ("status_rate_hz", 2.0),
    ):
        assert merged[name] == value and type(merged[name]) is float, name


def test_physics_speed_and_compute_on_scan_must_be_true_or_false():
    for name in ("physics_speed", "compute_on_scan"):
        with pytest.raises(ValueError, match=name):
            _node_parameters(**{name: "yes"})
    assert "physics_speed" not in _merged(_node_parameters())
