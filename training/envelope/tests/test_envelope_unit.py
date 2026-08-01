"""L1 unit tests for envelope.envelope.apply: every branch, every boundary value
(claude-docs/12-testing.md) -- exactly at bound, epsilon over, NaN, inf, negative rates,
zero ranges."""

from __future__ import annotations

import math
from dataclasses import dataclass

import pytest
from envelope.envelope import _clip, _sanitize, apply
from envelope.params import EnvelopeConfig
from envelope.types import Command, EnvelopeState

EPS = 1e-9


@dataclass(frozen=True)
class _FixedScorer:
    """Deterministic OODScorer stub: always returns the same score, ignoring the state it is
    given, so ood_triggered's score-vs-threshold branch can be exercised independently of
    envelope.ood's own (separately tested) decision logic."""

    fixed_score: float

    def score(self, state: tuple[float, ...]) -> float:
        return self.fixed_score


def _config(**overrides: object) -> EnvelopeConfig:
    kwargs: dict[str, object] = {
        "steering_range": (-1.0, 1.0),
        "speed_range": (-2.0, 10.0),
        "residual_fraction_steering": 0.5,  # max_steering_delta = 0.5 * 2.0 = 1.0
        "residual_fraction_speed": 0.2,  # max_speed_delta = 0.2 * 12.0 = 2.4
        "max_delta_steering_rad": 0.3,
        "max_delta_speed_mps": 1.0,
        "speed_cap_mps": 8.0,  # tighter than speed_range high of 10.0
        "ood_threshold": 5.0,
        "ood_scorer": _FixedScorer(0.0),
    }
    kwargs.update(overrides)
    return EnvelopeConfig(**kwargs)  # type: ignore[arg-type]


def _state(steering: float = 0.0, speed: float = 0.0) -> EnvelopeState:
    return EnvelopeState(last_output=Command(steering_rad=steering, speed_mps=speed))


# --- _clip -----------------------------------------------------------------------------


def test_clip_value_below_low_returns_low() -> None:
    assert _clip(-5.0, -1.0, 1.0) == -1.0


def test_clip_value_above_high_returns_high() -> None:
    assert _clip(5.0, -1.0, 1.0) == 1.0


def test_clip_value_inside_range_unchanged() -> None:
    assert _clip(0.3, -1.0, 1.0) == 0.3


def test_clip_exactly_at_low_unchanged() -> None:
    assert _clip(-1.0, -1.0, 1.0) == -1.0


def test_clip_exactly_at_high_unchanged() -> None:
    assert _clip(1.0, -1.0, 1.0) == 1.0


def test_clip_epsilon_below_low_clamped() -> None:
    assert _clip(-1.0 - EPS, -1.0, 1.0) == -1.0


def test_clip_epsilon_above_high_clamped() -> None:
    assert _clip(1.0 + EPS, -1.0, 1.0) == 1.0


def test_clip_zero_width_range_pins_to_the_single_value() -> None:
    assert _clip(5.0, 0.0, 0.0) == 0.0
    assert _clip(-5.0, 0.0, 0.0) == 0.0
    assert _clip(0.0, 0.0, 0.0) == 0.0


# --- _sanitize ---------------------------------------------------------------------------


def test_sanitize_finite_value_unchanged() -> None:
    assert _sanitize(3.5) == 3.5


def test_sanitize_nan_becomes_zero() -> None:
    assert _sanitize(math.nan) == 0.0


def test_sanitize_positive_inf_becomes_zero() -> None:
    assert _sanitize(math.inf) == 0.0


def test_sanitize_negative_inf_becomes_zero() -> None:
    assert _sanitize(-math.inf) == 0.0


# --- apply: nominal path, no clipping, no rate limiting, no OOD --------------------------


def test_nominal_command_passes_through_unmodified() -> None:
    config = _config(max_delta_steering_rad=100.0, max_delta_speed_mps=100.0)
    result = apply(config, _state(0.0, 0.0), Command(0.1, 1.0), Command(0.05, 0.2), (0.0,))
    assert result.command.steering_rad == pytest.approx(0.15)
    assert result.command.speed_mps == pytest.approx(1.2)
    assert result.ood_triggered is False
    assert result.residual_clipped is False
    assert result.rate_limited is False
    assert result.next_state.last_output == result.command


# --- absolute bounds: applied regardless of source ---------------------------------------


def test_base_command_alone_outside_bounds_is_clamped() -> None:
    config = _config(max_delta_steering_rad=100.0, max_delta_speed_mps=100.0)
    result = apply(config, _state(0.9, 0.0), Command(50.0, 50.0), Command(0.0, 0.0), (0.0,))
    assert result.command.steering_rad == config.steering_range[1]
    assert result.command.speed_mps == config.speed_cap_mps


def test_speed_cap_tighter_than_speed_range_wins() -> None:
    config = _config()  # speed_range high = 10.0, speed_cap_mps = 8.0
    result = apply(config, _state(0.0, 7.9), Command(0.0, 6.0), Command(0.0, 4.0), (0.0,))
    assert result.command.speed_mps == 8.0


# --- residual bounds: fraction-of-range clipping ------------------------------------------


def test_residual_within_fraction_bound_is_not_clipped() -> None:
    config = _config(max_delta_steering_rad=100.0)  # isolate from rate limiting
    result = apply(config, _state(0.0, 0.0), Command(0.0, 0.0), Command(0.5, 0.0), (0.0,))
    assert result.command.steering_rad == 0.5
    assert result.residual_clipped is False


def test_residual_exactly_at_fraction_bound_is_not_clipped() -> None:
    config = _config(max_delta_steering_rad=100.0)  # max_steering_delta = 1.0 exactly
    result = apply(config, _state(0.0, 0.0), Command(0.0, 0.0), Command(1.0, 0.0), (0.0,))
    assert result.command.steering_rad == 1.0
    assert result.residual_clipped is False


def test_residual_epsilon_over_fraction_bound_is_clipped() -> None:
    config = _config(max_delta_steering_rad=100.0)
    result = apply(config, _state(0.0, 0.0), Command(0.0, 0.0), Command(1.0 + EPS, 0.0), (0.0,))
    assert result.residual_clipped is True
    assert result.command.steering_rad == 1.0


def test_negative_residual_is_clipped_symmetrically() -> None:
    config = _config(max_delta_steering_rad=100.0)
    result = apply(config, _state(0.0, 0.0), Command(0.0, 0.0), Command(-5.0, 0.0), (0.0,))
    assert result.residual_clipped is True
    assert result.command.steering_rad == -1.0


def test_only_steering_residual_clipped_flag() -> None:
    config = _config()
    result = apply(config, _state(0.0, 0.0), Command(0.0, 0.0), Command(5.0, 0.1), (0.0,))
    assert result.residual_clipped is True


def test_only_speed_residual_clipped_flag() -> None:
    config = _config()  # max_speed_delta = 2.4
    result = apply(config, _state(0.0, 0.0), Command(0.0, 0.0), Command(0.1, 5.0), (0.0,))
    assert result.residual_clipped is True


def test_both_channels_clipped_flag() -> None:
    config = _config()
    result = apply(config, _state(0.0, 0.0), Command(0.0, 0.0), Command(5.0, 5.0), (0.0,))
    assert result.residual_clipped is True


def test_zero_residual_fraction_disables_residual_entirely() -> None:
    config = _config(residual_fraction_steering=0.0, residual_fraction_speed=0.0)
    result = apply(config, _state(0.0, 0.0), Command(0.2, 1.0), Command(0.9, 0.9), (0.0,))
    assert result.command.steering_rad == 0.2
    assert result.command.speed_mps == 1.0
    assert result.residual_clipped is True


# --- OOD fallback trigger ------------------------------------------------------------------


def test_ood_score_below_threshold_does_not_trigger_fallback() -> None:
    config = _config(ood_scorer=_FixedScorer(4.9))
    result = apply(config, _state(0.0, 0.0), Command(0.1, 0.0), Command(0.2, 0.0), (0.0,))
    assert result.ood_triggered is False
    assert result.command.steering_rad == pytest.approx(0.3)


def test_ood_score_exactly_at_threshold_does_not_trigger_fallback() -> None:
    config = _config(ood_scorer=_FixedScorer(5.0))
    result = apply(config, _state(0.0, 0.0), Command(0.1, 0.0), Command(0.2, 0.0), (0.0,))
    assert result.ood_triggered is False


def test_ood_score_epsilon_over_threshold_triggers_fallback() -> None:
    config = _config(ood_scorer=_FixedScorer(5.0 + EPS))
    result = apply(config, _state(0.0, 0.0), Command(0.1, 0.0), Command(0.2, 0.0), (0.0,))
    assert result.ood_triggered is True
    assert result.command.steering_rad == 0.1
    assert result.residual_clipped is False


def test_ood_score_nan_treated_as_maximally_out_of_distribution() -> None:
    config = _config(ood_scorer=_FixedScorer(math.nan))
    result = apply(config, _state(0.0, 0.0), Command(0.1, 0.0), Command(0.2, 0.0), (0.0,))
    assert result.ood_triggered is True
    assert result.command.steering_rad == 0.1


def test_ood_score_inf_treated_as_maximally_out_of_distribution() -> None:
    config = _config(ood_scorer=_FixedScorer(math.inf))
    result = apply(config, _state(0.0, 0.0), Command(0.1, 0.0), Command(0.2, 0.0), (0.0,))
    assert result.ood_triggered is True


def test_nan_residual_steering_triggers_ood_fallback_regardless_of_score() -> None:
    config = _config(ood_scorer=_FixedScorer(0.0))  # score well under threshold
    result = apply(config, _state(0.0, 0.0), Command(0.1, 0.2), Command(math.nan, 0.0), (0.0,))
    assert result.ood_triggered is True
    assert result.command.steering_rad == 0.1
    assert result.command.speed_mps == 0.2


def test_inf_residual_speed_triggers_ood_fallback() -> None:
    config = _config(ood_scorer=_FixedScorer(0.0))
    result = apply(config, _state(0.0, 0.0), Command(0.1, 0.2), Command(0.0, math.inf), (0.0,))
    assert result.ood_triggered is True


def test_neg_inf_residual_triggers_ood_fallback() -> None:
    config = _config(ood_scorer=_FixedScorer(0.0))
    result = apply(config, _state(0.0, 0.0), Command(0.1, 0.2), Command(-math.inf, 0.0), (0.0,))
    assert result.ood_triggered is True


# --- fail-closed: non-finite base command ---------------------------------------------------


def test_nan_base_steering_replaced_with_zero() -> None:
    config = _config()
    result = apply(config, _state(0.0, 0.0), Command(math.nan, 1.0), Command(0.0, 0.0), (0.0,))
    assert result.command.steering_rad == 0.0
    assert result.command.speed_mps == 1.0


def test_inf_base_speed_replaced_with_zero() -> None:
    config = _config()
    result = apply(config, _state(0.0, 0.0), Command(0.1, math.inf), Command(0.0, 0.0), (0.0,))
    assert result.command.speed_mps == 0.0
    assert result.command.steering_rad == 0.1


def test_neg_inf_base_steering_replaced_with_zero() -> None:
    config = _config()
    result = apply(config, _state(0.0, 0.0), Command(-math.inf, 0.0), Command(0.0, 0.0), (0.0,))
    assert result.command.steering_rad == 0.0


def test_nan_base_and_nan_residual_together_still_produce_a_finite_command() -> None:
    config = _config()
    result = apply(
        config, _state(0.0, 0.0), Command(math.nan, math.nan), Command(math.nan, math.nan), (0.0,)
    )
    assert math.isfinite(result.command.steering_rad)
    assert math.isfinite(result.command.speed_mps)
    assert result.command.steering_rad == 0.0
    assert result.command.speed_mps == 0.0


# --- rate limiting -----------------------------------------------------------------------


def test_within_rate_limit_not_flagged() -> None:
    config = _config()  # max_delta_steering_rad = 0.3
    result = apply(config, _state(0.0, 0.0), Command(0.2, 0.0), Command(0.0, 0.0), (0.0,))
    assert result.rate_limited is False
    assert result.command.steering_rad == 0.2


def test_exactly_at_rate_limit_not_flagged() -> None:
    config = _config()
    result = apply(config, _state(0.0, 0.0), Command(0.3, 0.0), Command(0.0, 0.0), (0.0,))
    assert result.rate_limited is False
    assert result.command.steering_rad == 0.3


def test_epsilon_over_rate_limit_is_clamped_and_flagged() -> None:
    config = _config()
    result = apply(config, _state(0.0, 0.0), Command(0.3 + EPS, 0.0), Command(0.0, 0.0), (0.0,))
    assert result.rate_limited is True
    assert result.command.steering_rad == 0.3


def test_rate_limit_applies_in_negative_direction_too() -> None:
    config = _config()
    result = apply(config, _state(0.0, 0.0), Command(-1.0, 0.0), Command(0.0, 0.0), (0.0,))
    assert result.rate_limited is True
    assert result.command.steering_rad == -0.3


def test_zero_rate_limit_freezes_output_at_previous_value() -> None:
    config = _config(max_delta_steering_rad=0.0, max_delta_speed_mps=0.0)
    result = apply(config, _state(0.2, 1.0), Command(0.9, 5.0), Command(0.0, 0.0), (0.0,))
    assert result.command.steering_rad == 0.2
    assert result.command.speed_mps == 1.0
    assert result.rate_limited is True


def test_only_steering_rate_limited_flag() -> None:
    config = _config()
    result = apply(config, _state(0.0, 0.0), Command(1.0, 0.5), Command(0.0, 0.0), (0.0,))
    assert result.rate_limited is True


def test_only_speed_rate_limited_flag() -> None:
    config = _config()  # max_delta_speed_mps = 1.0
    result = apply(config, _state(0.0, 0.0), Command(0.1, 5.0), Command(0.0, 0.0), (0.0,))
    assert result.rate_limited is True


def test_both_channels_rate_limited_flag() -> None:
    config = _config()
    result = apply(config, _state(0.0, 0.0), Command(1.0, 5.0), Command(0.0, 0.0), (0.0,))
    assert result.rate_limited is True


# --- previous-state sanitization / legalization -------------------------------------------


def test_nan_previous_output_treated_as_zero_for_rate_limiting() -> None:
    config = _config()
    result = apply(config, _state(math.nan, math.nan), Command(1.0, 5.0), Command(0.0, 0.0), (0.0,))
    # rate-limit anchor becomes 0.0 -> output moves at most max_delta from zero.
    assert result.command.steering_rad == 0.3
    assert result.command.speed_mps == 1.0


def test_previous_output_far_outside_current_bounds_is_legalized_first() -> None:
    # An EnvelopeState carrying an out-of-range previous output (e.g. bounds tightened
    # between calls, or a corrupted carried state) must not make "stay within bounds" and
    # "stay within a rate-limit step of the previous output" simultaneously impossible: the
    # previous output is clamped into bounds BEFORE the rate window is built around it.
    config = _config()  # steering_range = (-1.0, 1.0), max_delta_steering_rad = 0.3
    result = apply(config, _state(1e6, 0.0), Command(0.0, 0.0), Command(0.0, 0.0), (0.0,))
    # legalized prev = clip(1e6, -1, 1) = 1.0; base+residual = 0.0, which is more than
    # max_delta_steering_rad (0.3) below the legalized prev, so the output is pulled up to
    # the bottom of the rate window: 1.0 - 0.3 = 0.7 -- NOT clamped to the range edge (1.0).
    assert result.command.steering_rad == pytest.approx(0.7)


def test_inf_previous_output_is_legalized_before_rate_limiting() -> None:
    config = _config()
    result = apply(config, _state(math.inf, 0.0), Command(0.0, 0.0), Command(0.0, 0.0), (0.0,))
    # sanitize(inf) -> 0.0, clip(0.0, -1, 1) -> 0.0 -> rate window is [-0.3, 0.3].
    assert result.command.steering_rad == 0.0


# --- zero-width ranges ----------------------------------------------------------------------


def test_zero_width_steering_range_pins_output_regardless_of_input() -> None:
    config = _config(steering_range=(0.0, 0.0), residual_fraction_steering=0.0)
    result = apply(config, _state(0.0, 0.0), Command(0.7, 0.0), Command(0.9, 0.0), (0.0,))
    assert result.command.steering_rad == 0.0


def test_zero_width_speed_range_pins_output_regardless_of_input() -> None:
    config = _config(speed_range=(0.0, 0.0), speed_cap_mps=0.0, residual_fraction_speed=0.0)
    result = apply(config, _state(0.0, 0.0), Command(0.0, 7.0), Command(0.0, 9.0), (0.0,))
    assert result.command.speed_mps == 0.0
