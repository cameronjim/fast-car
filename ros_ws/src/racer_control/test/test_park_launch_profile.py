"""L1 check of park.launch.py's `profile` argument (roadmap 2.9).

Loads the SOURCE launch file as a module, runs its OpaqueFunction against a LaunchContext and
inspects the parameters park_node would be started with; no process is started (same approach
as test_gap_follow_launch_profile.py). What it pins:

  * `profile:=none` (the default) passes nothing: the node defaults apply.
  * `profile:=floor-2026-10-07` hands the node exactly the profile, with the right types (a
    double parameter rejects an int).
  * explicit launch arguments override the profile, the profile overrides params_file, and bad
    values are refused before the node starts.
"""

from __future__ import annotations

import importlib.util
import pathlib

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

_LAUNCH_FILE = pathlib.Path(__file__).resolve().parents[1] / "launch" / "park.launch.py"

# The profile, typed out again on purpose: an edit to the profile in the launch file must also
# edit this test (and the table in docs/notes/parking-2026-10-07.md).
_FLOOR_2026_10_07 = {
    "park_side": "right",
    "search_speed_mps": 0.5,
    "park_speed_mps": -0.5,
    "settle_s": 0.3,
    "require_odometry": True,
    "margin_m": 0.1,
    "park_steering_fraction": 0.95,
    "blocked_abort_s": 2.0,
    "final_forward_m": 0.15,
}
_BOOL_PARAMETERS = {"require_odometry"}
_STRING_PARAMETERS = {"park_side", "turn_direction"}


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("park_launch", _LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _declared_arguments(description) -> dict[str, DeclareLaunchArgument]:
    return {e.name: e for e in description.entities if isinstance(e, DeclareLaunchArgument)}


def _node_parameters(**launch_arguments: str) -> list:
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
    merged: dict = {}
    for entry in entries:
        if isinstance(entry, dict):
            merged.update(entry)
    return merged


def test_profile_argument_defaults_to_none():
    module = _load_launch_module()
    args = _declared_arguments(module.generate_launch_description())
    context = LaunchContext()
    assert "".join(s.perform(context) for s in args["profile"].default_value) == "none"
    assert set(args["profile"].choices) == {"none", "floor-2026-10-07"}
    assert module.PROFILES["floor-2026-10-07"] is module.FLOOR_2026_10_07_PROFILE
    assert module.PROFILES["none"] == {}


def test_floor_profile_constant():
    assert _load_launch_module().FLOOR_2026_10_07_PROFILE == _FLOOR_2026_10_07


def test_none_profile_passes_nothing():
    assert _node_parameters() == []


def test_floor_profile_reaches_the_node_with_the_right_types():
    merged = _merged(_node_parameters(profile="floor-2026-10-07"))
    assert merged == _FLOOR_2026_10_07
    for name, value in merged.items():
        if name in _BOOL_PARAMETERS:
            assert type(value) is bool, name
        elif name in _STRING_PARAMETERS:
            assert type(value) is str, name
        else:
            assert type(value) is float, name
    # The real car's profile refuses to run without odometry.
    assert merged["require_odometry"] is True


def test_explicit_arguments_override_the_profile_in_order():
    entries = _node_parameters(
        profile="floor-2026-10-07",
        params_file="/tmp/p.yaml",
        search_speed_mps="1",
        park_side="LEFT",
        turn_direction="right",
        require_odometry="false",
    )
    assert str(entries[0]) == "/tmp/p.yaml"
    assert entries[1] == _FLOOR_2026_10_07
    assert entries[2] == {
        "search_speed_mps": 1.0,
        "park_side": "left",
        "turn_direction": "right",
        "require_odometry": False,
    }
    assert type(entries[2]["search_speed_mps"]) is float


@pytest.mark.parametrize(
    ("argument", "value"),
    [("profile", "floor-2026-10-06"), ("park_side", "middle"), ("require_odometry", "yes")],
)
def test_bad_values_are_refused(argument, value):
    with pytest.raises(ValueError):
        _node_parameters(**{argument: value})
