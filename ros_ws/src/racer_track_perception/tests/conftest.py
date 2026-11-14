"""Shared fixtures. The vehicle is the live config/vehicle_params.yaml through the generated
binding (never typed here); other mounts are derived from it with dataclasses.replace."""

from __future__ import annotations

import importlib.util

import pytest
from racer_track_perception.vehicle import LiveVehicle, load_live_vehicle

# hypothesis is a dev dependency (uv run pytest). colcon test runs this directory under the
# ros-dev image's system python3 where it may be absent; the property tests are then not collected.
collect_ignore = (
    []
    if importlib.util.find_spec("hypothesis")
    else ["test_property.py", "test_refine_property.py", "test_deskew_property.py"]
)


@pytest.fixture(scope="session")
def live() -> LiveVehicle:
    return load_live_vehicle()
