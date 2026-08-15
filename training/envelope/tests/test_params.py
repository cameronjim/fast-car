"""L1 unit tests for envelope.params: EnvelopeConfig validation and
EnvelopeConfig.from_vehicle_params wiring. Every branch, every boundary value
(claude-docs/12-testing.md): exactly at bound, epsilon over, NaN, inf, negative rates, zero
ranges."""

from __future__ import annotations

import math
from dataclasses import dataclass

import pytest
from envelope.ood import DistanceOODScorer
from envelope.params import EnvelopeConfig

EPS = 1e-9


def _config(**overrides: object) -> EnvelopeConfig:
    kwargs: dict[str, object] = {
        "steering_range": (-0.4189, 0.4189),
        "speed_range": (-5.0, 20.0),
        "residual_fraction_steering": 0.2,
        "residual_fraction_speed": 0.1,
        "max_delta_steering_rad": 0.05,
        "max_delta_speed_mps": 0.5,
        "speed_cap_mps": 10.0,
        "ood_threshold": 3.0,
        "ood_scorer": DistanceOODScorer(reference=(0.0,)),
    }
    kwargs.update(overrides)
    return EnvelopeConfig(**kwargs)  # type: ignore[arg-type]


def test_valid_config_constructs() -> None:
    config = _config()
    assert config.residual_fraction_speed == 0.1


def test_zero_width_steering_range_is_allowed() -> None:
    config = _config(steering_range=(0.0, 0.0))
    assert config.steering_range == (0.0, 0.0)


def test_zero_width_speed_range_is_allowed_with_matching_cap() -> None:
    config = _config(speed_range=(0.0, 0.0), speed_cap_mps=0.0)
    assert config.speed_range == (0.0, 0.0)


def test_steering_range_low_above_high_rejected() -> None:
    with pytest.raises(ValueError):
        _config(steering_range=(0.5, -0.5))


def test_speed_range_low_above_high_rejected() -> None:
    with pytest.raises(ValueError):
        _config(speed_range=(5.0, -5.0))


def test_steering_range_nan_rejected() -> None:
    with pytest.raises(ValueError):
        _config(steering_range=(math.nan, 0.4189))


def test_steering_range_inf_rejected() -> None:
    with pytest.raises(ValueError):
        _config(steering_range=(-math.inf, 0.4189))


def test_speed_range_nan_rejected() -> None:
    with pytest.raises(ValueError):
        _config(speed_range=(-5.0, math.nan))


def test_residual_fraction_steering_exactly_zero_allowed() -> None:
    assert _config(residual_fraction_steering=0.0).residual_fraction_steering == 0.0


def test_residual_fraction_steering_exactly_one_allowed() -> None:
    assert _config(residual_fraction_steering=1.0).residual_fraction_steering == 1.0


def test_residual_fraction_steering_epsilon_below_zero_rejected() -> None:
    with pytest.raises(ValueError):
        _config(residual_fraction_steering=-EPS)


def test_residual_fraction_steering_epsilon_above_one_rejected() -> None:
    with pytest.raises(ValueError):
        _config(residual_fraction_steering=1.0 + EPS)


def test_residual_fraction_speed_nan_rejected() -> None:
    with pytest.raises(ValueError):
        _config(residual_fraction_speed=math.nan)


def test_residual_fraction_speed_inf_rejected() -> None:
    with pytest.raises(ValueError):
        _config(residual_fraction_speed=math.inf)


def test_max_delta_steering_zero_allowed() -> None:
    assert _config(max_delta_steering_rad=0.0).max_delta_steering_rad == 0.0


def test_max_delta_steering_negative_rejected() -> None:
    with pytest.raises(ValueError):
        _config(max_delta_steering_rad=-EPS)


def test_max_delta_speed_negative_rejected() -> None:
    with pytest.raises(ValueError):
        _config(max_delta_speed_mps=-1.0)


def test_max_delta_steering_nan_rejected() -> None:
    with pytest.raises(ValueError):
        _config(max_delta_steering_rad=math.nan)


def test_speed_cap_negative_but_within_speed_range_allowed() -> None:
    # speed_cap_mps is not required to be >= 0 in isolation -- a config capping a
    # reverse-leaning speed range is legal as long as the cap does not fall below the
    # range's own low end (see test_speed_cap_below_speed_range_low_rejected).
    config = _config(speed_range=(-5.0, 20.0), speed_cap_mps=-EPS)
    assert config.speed_cap_mps == -EPS


def test_speed_cap_inf_rejected() -> None:
    with pytest.raises(ValueError):
        _config(speed_cap_mps=math.inf)


def test_speed_cap_below_speed_range_low_rejected() -> None:
    with pytest.raises(ValueError):
        _config(speed_range=(-5.0, 20.0), speed_cap_mps=-6.0)


def test_speed_cap_exactly_at_speed_range_low_allowed() -> None:
    config = _config(speed_range=(-5.0, 20.0), speed_cap_mps=-5.0)
    assert config.speed_cap_mps == -5.0


def test_ood_threshold_zero_allowed() -> None:
    assert _config(ood_threshold=0.0).ood_threshold == 0.0


def test_ood_threshold_negative_rejected() -> None:
    with pytest.raises(ValueError):
        _config(ood_threshold=-EPS)


def test_ood_threshold_nan_rejected() -> None:
    with pytest.raises(ValueError):
        _config(ood_threshold=math.nan)


def test_ood_scorer_without_score_method_rejected() -> None:
    with pytest.raises(TypeError):
        _config(ood_scorer=object())


# --- from_vehicle_params wiring -----------------------------------------------------------
#
# Lightweight stand-ins for the generated VehicleParams shape, structurally compatible with
# envelope.params.VehicleParamsLike, so wiring logic (including both the null and non-null
# branches of the currently-unset params fields) is fully exercised without depending on the
# real generated binding's current values. tests/test_from_vehicle_params_wiring.py covers
# the real generated binding end to end.


@dataclass(frozen=True)
class _StubSteering:
    min_angle_rad: float | None
    max_angle_rad: float | None


@dataclass(frozen=True)
class _StubLimits:
    global_speed_cap_mps: float
    min_velocity_mps: float
    envelope_fraction_speed: float | None


@dataclass(frozen=True)
class _StubVehicleParams:
    steering: _StubSteering
    limits: _StubLimits


def _stub_params(
    envelope_fraction_speed: float | None,
    min_angle_rad: float | None = -0.4189,
    max_angle_rad: float | None = 0.4189,
) -> _StubVehicleParams:
    return _StubVehicleParams(
        steering=_StubSteering(min_angle_rad=min_angle_rad, max_angle_rad=max_angle_rad),
        limits=_StubLimits(
            global_speed_cap_mps=20.0,
            min_velocity_mps=-5.0,
            envelope_fraction_speed=envelope_fraction_speed,
        ),
    )


def test_from_vehicle_params_uses_params_fraction_when_no_override_given() -> None:
    config = EnvelopeConfig.from_vehicle_params(
        _stub_params(envelope_fraction_speed=0.3),
        DistanceOODScorer(reference=(0.0,)),
        residual_fraction_steering=0.2,
        max_delta_steering_rad=0.05,
        max_delta_speed_mps=0.5,
        speed_cap_mps=10.0,
        ood_threshold=3.0,
    )
    assert config.residual_fraction_speed == 0.3
    assert config.steering_range == (-0.4189, 0.4189)
    assert config.speed_range == (-5.0, 20.0)


def test_from_vehicle_params_explicit_override_wins_over_params_value() -> None:
    config = EnvelopeConfig.from_vehicle_params(
        _stub_params(envelope_fraction_speed=0.3),
        DistanceOODScorer(reference=(0.0,)),
        residual_fraction_steering=0.2,
        residual_fraction_speed=0.7,
        max_delta_steering_rad=0.05,
        max_delta_speed_mps=0.5,
        speed_cap_mps=10.0,
        ood_threshold=3.0,
    )
    assert config.residual_fraction_speed == 0.7


def test_from_vehicle_params_null_fraction_requires_explicit_override() -> None:
    with pytest.raises(ValueError):
        EnvelopeConfig.from_vehicle_params(
            _stub_params(envelope_fraction_speed=None),
            DistanceOODScorer(reference=(0.0,)),
            residual_fraction_steering=0.2,
            max_delta_steering_rad=0.05,
            max_delta_speed_mps=0.5,
            speed_cap_mps=10.0,
            ood_threshold=3.0,
        )


def test_from_vehicle_params_null_fraction_with_override_succeeds() -> None:
    config = EnvelopeConfig.from_vehicle_params(
        _stub_params(envelope_fraction_speed=None),
        DistanceOODScorer(reference=(0.0,)),
        residual_fraction_steering=0.2,
        residual_fraction_speed=0.4,
        max_delta_steering_rad=0.05,
        max_delta_speed_mps=0.5,
        speed_cap_mps=10.0,
        ood_threshold=3.0,
    )
    assert config.residual_fraction_speed == 0.4


def test_from_vehicle_params_null_steering_range_refuses() -> None:
    with pytest.raises(ValueError):
        EnvelopeConfig.from_vehicle_params(
            _stub_params(envelope_fraction_speed=0.3, min_angle_rad=None),
            DistanceOODScorer(reference=(0.0,)),
            residual_fraction_steering=0.2,
            residual_fraction_speed=0.3,
            max_delta_steering_rad=0.05,
            max_delta_speed_mps=0.5,
            speed_cap_mps=10.0,
            ood_threshold=3.0,
        )


def test_from_vehicle_params_null_max_angle_refuses() -> None:
    with pytest.raises(ValueError):
        EnvelopeConfig.from_vehicle_params(
            _stub_params(envelope_fraction_speed=0.3, max_angle_rad=None),
            DistanceOODScorer(reference=(0.0,)),
            residual_fraction_steering=0.2,
            residual_fraction_speed=0.3,
            max_delta_steering_rad=0.05,
            max_delta_speed_mps=0.5,
            speed_cap_mps=10.0,
            ood_threshold=3.0,
        )
