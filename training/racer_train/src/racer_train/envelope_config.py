"""Builds the layer-4 `envelope.EnvelopeConfig` for the training env.

claude-docs/05-safety.md layer 4 / claude-docs/02-repo-layout.md: "envelope/ is one library
consumed by BOTH training (sim env) and racer_policy (deployment). Never fork its logic."
This module never re-implements any envelope decision logic -- it only resolves the
`EnvelopeConfig` constructor's arguments, using `EnvelopeConfig.from_vehicle_params` (the
wiring point training/envelope/src/envelope/params.py already provides for exactly this)
for the base-command ranges, and CLAUDE.md invariant 2 (physical constants only from the
generated vehicle_params binding, never hand-typed) for the rate limits.
"""

from __future__ import annotations

from dataclasses import dataclass

from envelope import DistanceOODScorer, EnvelopeConfig


@dataclass(frozen=True)
class EnvelopeSettings:
    """Training-time tuning knobs for the layer-4 envelope. None of these are physical
    constants (see claude-docs/06-vehicle-params.md's EnvelopeConfig.from_vehicle_params
    docstring: residual fractions, rate limits, and the speed cap have no dedicated
    vehicle_params field yet, or are deliberately configurable per training run), so they
    live in the experiment config (training/configs/), not vehicle_params.yaml.
    """

    residual_fraction_steering: float
    residual_fraction_speed: float
    speed_cap_mps: float
    ood_threshold: float
    ood_reference_state: tuple[float, ...]
    ood_reference_scale: tuple[float, ...] | None = None


def build_envelope_config(
    settings: EnvelopeSettings, vehicle_params, timestep_s: float
) -> EnvelopeConfig:
    """`vehicle_params` is the generated `VEHICLE_PARAMS` instance (see
    `racer_gym.params.load_vehicle_params`). Rate limits are converted from the physical
    rad/s / (m/s)/s values vehicle_params already carries into per-`apply()`-call deltas at
    this call rate, exactly as `envelope.envelope`'s module docstring specifies ("a caller
    running the deploy loop at a fixed control rate converts a physical rad/s or (m/s)/s
    limit into a per-call delta once, at config-construction time")."""
    if timestep_s <= 0.0:
        raise ValueError(f"timestep_s must be > 0, got {timestep_s}")

    max_delta_steering_rad = vehicle_params.steering.max_rate_rad_per_s * timestep_s
    max_delta_speed_mps = vehicle_params.actuation.max_acceleration_mps2 * timestep_s

    ood_scorer = DistanceOODScorer(
        reference=settings.ood_reference_state, scale=settings.ood_reference_scale
    )

    return EnvelopeConfig.from_vehicle_params(
        vehicle_params,
        ood_scorer,
        residual_fraction_steering=settings.residual_fraction_steering,
        residual_fraction_speed=settings.residual_fraction_speed,
        max_delta_steering_rad=max_delta_steering_rad,
        max_delta_speed_mps=max_delta_speed_mps,
        speed_cap_mps=settings.speed_cap_mps,
        ood_threshold=settings.ood_threshold,
    )
