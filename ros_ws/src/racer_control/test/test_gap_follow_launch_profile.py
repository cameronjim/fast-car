"""L1 check of gap_follow.launch.py's `profile` argument (2026-10-06 floor checkpoint).

Loads the SOURCE launch file as a module, runs its OpaqueFunction against a LaunchContext and
inspects the parameters the node would be started with; no process is started and no ROS
graph is needed (same approach as racer_bringup/test/test_car_teleop_profile.py). What it
pins:

  * `profile:=none` (the default) is the old behaviour: no profile entry, max_speed_mps 2.0.
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
    "min_speed_mps": 0.5,
    "max_speed_mps": 0.9,
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
}
_INT_PARAMETERS = {"target_range_median_scans"}
_BOOL_PARAMETERS = {"swept_path_clamp", "target_deepest_ray"}


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
    assert set(args["profile"].choices) == {"none", "floor-2026-10-06"}
    assert set(module.PROFILES) == {"none", "floor-2026-10-06"}


def test_floor_profile_constant_is_the_checkpoint():
    module = _load_launch_module()
    assert module.FLOOR_2026_10_06_PROFILE == _FLOOR_2026_10_06
    assert module.PROFILES["floor-2026-10-06"] is module.FLOOR_2026_10_06_PROFILE
    assert module.PROFILES["none"] == {}


def test_none_profile_is_the_old_behaviour():
    entries = _node_parameters()
    assert entries == [{"max_speed_mps": 2.0}]
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
    )
    assert merged == expected
    assert type(merged["swept_path_lookahead_m"]) is float
    assert type(merged["speed_time_constant_s"]) is float
    assert type(merged["target_range_median_scans"]) is int
    assert type(merged["centering_gain"]) is float


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
