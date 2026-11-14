"""Shared test inputs.

`toy_vehicle` is an arbitrary vehicle with round numbers so hand-computed expectations stay
readable. It is a test input to pure functions, not a statement about the car: the car's own
numbers only ever come from config/vehicle_params.yaml (see test_fixture.py).
"""

from __future__ import annotations

import pytest
import racer_pnp_dynamics.bags  # noqa: F401  (puts tools/ on sys.path for the tests)
from racer_pnp_dynamics.model import Nominal
from racer_pnp_dynamics.vehicle import Vehicle


@pytest.fixture
def toy_vehicle() -> Vehicle:
    return Vehicle(
        wheelbase_m=0.5,
        steer_min_rad=-0.4,
        steer_max_rad=0.4,
        steer_rate_max_rad_per_s=2.0,
        throttle_full_scale_mps=2.0,
        schema_version="9.9.9",
        sysid_session_id="toy",
        physical_lock_left_rad=0.5,
        physical_lock_right_rad=-0.4,
        body_front_x_m=0.6,
        body_rear_x_m=0.1,
        body_half_width_m=0.15,
        lidar_x_m=0.3,
        lidar_y_m=0.0,
        lidar_yaw_rad=3.141593,
    )


@pytest.fixture
def toy_nominal() -> Nominal:
    return Nominal(
        speed_gain=1.0,
        speed_deadband_mps=0.0,
        speed_tau_s=0.25,
        accel_max_mps2=2.0,
        decel_max_mps2=1.5,
        steer_gain_left=1.0,
        steer_gain_right=1.0,
        steer_offset_rad=0.0,
        steer_tau_s=0.10,
        command_delay_s=0.08,
        understeer_s2_per_m2=0.0,
    )


@pytest.fixture(scope="module")
def toy_vehicle_module() -> Vehicle:
    """The same toy vehicle for fixtures that live for a whole test module."""
    return Vehicle(
        0.5, -0.4, 0.4, 2.0, 2.0, "9.9.9", "toy", 0.5, -0.4, 0.6, 0.1, 0.15, 0.3, 0.0, 3.141593
    )
