"""Integration test: EnvelopeConfig.from_vehicle_params against the REAL generated binding
of the committed config/vehicle_params.yaml (regenerated at test time via
tools/gen_params.py, never committed -- see tests/conftest.py). This is the proof that
envelope.params.VehicleParamsLike is actually satisfied by what gen_params.py emits, not
just by the hand-written stubs in test_params.py."""

from __future__ import annotations

from typing import Any

from envelope.ood import DistanceOODScorer
from envelope.params import EnvelopeConfig


def test_wires_steering_and_speed_ranges_from_the_real_params_file(
    real_vehicle_params: Any,
) -> None:
    # config/vehicle_params.yaml currently carries the gym-sourced provisional steering
    # limits and limits.envelope_fraction_speed = null (Phase 5 has not tuned it), so an
    # explicit override is required here -- this is exactly the "params wiring uses the
    # explicit constructor argument while the field is null" path the S.4 task calls for.
    config = EnvelopeConfig.from_vehicle_params(
        real_vehicle_params,
        DistanceOODScorer(reference=(0.0,)),
        residual_fraction_steering=0.2,
        residual_fraction_speed=0.15,
        max_delta_steering_rad=0.05,
        max_delta_speed_mps=0.5,
        speed_cap_mps=10.0,
        ood_threshold=3.0,
    )

    assert config.steering_range == (-0.4189, 0.4189)
    assert config.speed_range == (-5.0, 20.0)
    assert config.residual_fraction_speed == 0.15
