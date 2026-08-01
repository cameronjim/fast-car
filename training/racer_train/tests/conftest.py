"""Shared fixtures for racer_train tests."""

from __future__ import annotations

from pathlib import Path

import pytest
from racer_gym.params import load_vehicle_params

# tests/conftest.py -> tests/ -> racer_train/ -> training/ -> repo root.
REPO_ROOT = Path(__file__).resolve().parents[3]
FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"
SMOKE_CONFIG_PATH = FIXTURES_DIR / "smoke_config.yaml"


@pytest.fixture(scope="session")
def real_vehicle_params():
    """The generated VEHICLE_PARAMS instance from the committed config/vehicle_params.yaml
    (claude-docs/06-vehicle-params.md rule 3: bindings are generated, never hand-written)."""
    return load_vehicle_params()
