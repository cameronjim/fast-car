from __future__ import annotations

import pytest
from racer_track_perception.vehicle import LiveVehicle, load_live_vehicle


@pytest.fixture(scope="session")
def live() -> LiveVehicle:
    return load_live_vehicle()
