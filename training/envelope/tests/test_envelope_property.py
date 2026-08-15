"""L2 property test (claude-docs/12-testing.md): "for ANY input command and ANY internal
state, output is always within bounds and within rate limits of the previous output. This
is the single most important test in the repo -- the layer-4 safety claim rests on it."

hypothesis generates arbitrary floats -- including NaN and +-inf -- for every input:
base_command, residual, and the carried EnvelopeState. Nothing is excluded to make this
pass; envelope.envelope.apply's NaN/inf policy (see its module docstring) exists precisely
so this test holds unconditionally.

The "within rate limits of the previous output" half is checked against the LEGALIZED
previous output (sanitized non-finite -> 0.0, then clamped into the current bounds), which
is what apply() itself uses as the rate-limit anchor -- see the module docstring's
explanation of why an arbitrary/corrupted carried state would otherwise make the two halves
of this property mutually impossible to satisfy at the same time.
"""

from __future__ import annotations

import math

from envelope.envelope import _clip, _sanitize, apply
from envelope.ood import DistanceOODScorer
from envelope.params import EnvelopeConfig
from envelope.types import Command, EnvelopeState
from hypothesis import given, settings
from hypothesis import strategies as st

# allow_nan/allow_infinity are the whole point of this test; do not narrow this strategy to
# "well-behaved" floats to make the test easier to pass.
_any_float = st.floats(allow_nan=True, allow_infinity=True, width=64)

# Ranges, fractions, and rate limits are drawn from realistic-but-varied finite values;
# EnvelopeConfig.__post_init__ already has its own dedicated validation tests (test_params.py)
# for what makes a config invalid, so this strategy only generates configs that construct.
_finite_positive = st.floats(
    min_value=1e-6, max_value=1e3, allow_nan=False, allow_infinity=False, width=64
)
_fraction = st.floats(min_value=0.0, max_value=1.0, allow_nan=False, allow_infinity=False)


@st.composite
def _configs(draw: st.DrawFn) -> EnvelopeConfig:
    steering_width = draw(_finite_positive)
    speed_width = draw(_finite_positive)
    steering_low = draw(
        st.floats(min_value=-1e3, max_value=1e3, allow_nan=False, allow_infinity=False)
    )
    speed_low = draw(
        st.floats(min_value=-1e3, max_value=1e3, allow_nan=False, allow_infinity=False)
    )
    speed_high = speed_low + speed_width
    speed_cap_extra = draw(_finite_positive)  # cap sits at-or-above speed_low, may exceed high
    speed_cap = speed_low + speed_cap_extra

    return EnvelopeConfig(
        steering_range=(steering_low, steering_low + steering_width),
        speed_range=(speed_low, speed_high),
        residual_fraction_steering=draw(_fraction),
        residual_fraction_speed=draw(_fraction),
        max_delta_steering_rad=draw(_finite_positive),
        max_delta_speed_mps=draw(_finite_positive),
        speed_cap_mps=speed_cap,
        ood_threshold=draw(_finite_positive),
        ood_scorer=DistanceOODScorer(reference=(0.0, 0.0)),
    )


@given(
    config=_configs(),
    prev_steering=_any_float,
    prev_speed=_any_float,
    base_steering=_any_float,
    base_speed=_any_float,
    residual_steering=_any_float,
    residual_speed=_any_float,
    observed=st.tuples(_any_float, _any_float),
)
@settings(max_examples=500)
def test_output_always_in_bounds_and_within_rate_limit_of_legalized_previous_output(
    config: EnvelopeConfig,
    prev_steering: float,
    prev_speed: float,
    base_steering: float,
    base_speed: float,
    residual_steering: float,
    residual_speed: float,
    observed: tuple[float, float],
) -> None:
    state = EnvelopeState(last_output=Command(prev_steering, prev_speed))
    base_command = Command(base_steering, base_speed)
    residual = Command(residual_steering, residual_speed)

    result = apply(config, state, base_command, residual, observed)

    steering_low, steering_high = config.steering_range
    speed_low, speed_range_high = config.speed_range
    speed_high = min(speed_range_high, config.speed_cap_mps)

    # -- absolute bounds, unconditionally -------------------------------------------------
    assert math.isfinite(result.command.steering_rad)
    assert math.isfinite(result.command.speed_mps)
    assert steering_low - 1e-9 <= result.command.steering_rad <= steering_high + 1e-9
    assert speed_low - 1e-9 <= result.command.speed_mps <= speed_high + 1e-9

    # -- rate limit relative to the LEGALIZED previous output ------------------------------
    prev_legal_steering = _clip(_sanitize(prev_steering), steering_low, steering_high)
    prev_legal_speed = _clip(_sanitize(prev_speed), speed_low, speed_high)

    assert (
        abs(result.command.steering_rad - prev_legal_steering)
        <= config.max_delta_steering_rad + 1e-9
    )
    assert abs(result.command.speed_mps - prev_legal_speed) <= config.max_delta_speed_mps + 1e-9

    # -- next_state carries exactly the returned command, for the next call ----------------
    assert result.next_state.last_output == result.command
