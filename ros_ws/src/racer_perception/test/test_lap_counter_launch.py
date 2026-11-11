"""L1 checks of lap_counter.launch.py and the lap_counter pieces of cameras.launch.py.

Skipped where launch / launch_ros are absent (the bare `uv run pytest` run).
"""

from __future__ import annotations

import importlib.util
import pathlib

import pytest

pytest.importorskip("launch")
pytest.importorskip("launch_ros")

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node as LaunchNode
from launch_ros.utilities import evaluate_parameters
from racer_perception.lap_counter import LapConfig

_PKG = pathlib.Path(__file__).resolve().parents[1]
_CAMERAS = _PKG.parent / "racer_bringup" / "launch" / "cameras.launch.py"
_LAUNCH = _PKG / "launch" / "lap_counter.launch.py"


def _load(path: pathlib.Path):
    spec = importlib.util.spec_from_file_location(path.name.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _defaults(entities) -> dict[str, str]:
    context = LaunchContext()
    return {
        e.name: "".join(s.perform(context) for s in e.default_value)
        for e in entities
        if isinstance(e, DeclareLaunchArgument)
    }


def test_launch_defaults_equal_lapconfig_defaults():
    defaults = _defaults(_load(_LAUNCH).generate_launch_description().entities)
    cfg = LapConfig()
    for name in (
        "similarity_threshold",
        "rearm_threshold",
        "min_lap_time_s",
        "min_lap_distance_m",
        "start_speed_mps",
    ):
        assert float(defaults[name]) == getattr(cfg, name), name


def test_launch_starts_exactly_one_node_with_typed_parameters():
    entities = _load(_LAUNCH).generate_launch_description().entities
    nodes = [e for e in entities if isinstance(e, LaunchNode)]
    assert len(nodes) == 1
    node = nodes[0]
    assert (node.node_package, node.node_executable) == ("racer_perception", "lap_counter_node")
    context = LaunchContext()
    context.launch_configurations.update(
        {**_defaults(entities), "similarity_threshold": "0.9", "rate_hz": "5"}
    )
    merged = {}
    for entry in evaluate_parameters(context, node._Node__parameters):
        merged.update(entry)
    assert merged["similarity_threshold"] == 0.9 and merged["rate_hz"] == 5.0
    assert merged["image_topic"] == "/camera/usb/image_raw/compressed"


def test_cameras_launch_lap_defaults_equal_lapconfig_defaults():
    module = _load(_CAMERAS)
    cfg = LapConfig()
    for param, default, _desc, kind in module.LAP_FORWARDED.values():
        if kind is float:
            assert float(default) == getattr(cfg, param), param
