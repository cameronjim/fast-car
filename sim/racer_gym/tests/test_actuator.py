"""L1/L2 tests for racer_gym.dynamics.actuator (first-order actuator dynamics --
claude-docs/07-sim-and-sysid.md requirement 3, claude-docs/12-testing.md)."""

from __future__ import annotations

import math

from hypothesis import given
from hypothesis import strategies as st
from racer_gym.dynamics.actuator import FirstOrderActuator

# --------------------------------------------------------------------------------------
# L1: hand-computed cases
# --------------------------------------------------------------------------------------


def test_step_response_63_percent_at_one_time_constant():
    tau_s = 0.5
    dt_s = 0.5  # exactly one time constant per step
    act = FirstOrderActuator(tau_s=tau_s, dt_s=dt_s, initial_output=0.0)
    output = act.step(target=1.0)
    assert math.isclose(output, 1.0 - math.exp(-1.0), rel_tol=1e-9)
    assert math.isclose(output, 0.6321, abs_tol=1e-3)


def test_step_response_approaches_target_over_many_small_steps():
    tau_s = 0.2
    dt_s = 0.01
    act = FirstOrderActuator(tau_s=tau_s, dt_s=dt_s, initial_output=0.0)
    for _ in range(int(tau_s / dt_s)):
        output = act.step(target=1.0)
    assert math.isclose(output, 1.0 - math.exp(-1.0), rel_tol=1e-6)


def test_instantaneous_when_tau_is_zero():
    """Documented fallback: tau_s <= 0 (unfitted vehicle_params) means no lag at all --
    identical to stock f1tenth_gym's instantaneous (rate-limited-only) actuation."""
    act = FirstOrderActuator(tau_s=0.0, dt_s=0.01, initial_output=0.0)
    assert act.step(target=5.0) == 5.0
    assert act.step(target=-3.0) == -3.0


def test_reset_sets_output_directly():
    act = FirstOrderActuator(tau_s=0.5, dt_s=0.01, initial_output=0.0)
    act.step(target=10.0)
    act.reset(output=2.0)
    assert act.output == 2.0


def test_negative_tau_raises():
    import pytest

    with pytest.raises(ValueError):
        FirstOrderActuator(tau_s=-0.1, dt_s=0.01)


def test_nonpositive_dt_raises():
    import pytest

    with pytest.raises(ValueError):
        FirstOrderActuator(tau_s=0.1, dt_s=0.0)


def test_output_converges_to_target_over_time():
    act = FirstOrderActuator(tau_s=0.3, dt_s=0.01, initial_output=0.0)
    for _ in range(2000):
        output = act.step(target=7.5)
    assert math.isclose(output, 7.5, rel_tol=1e-6)


# --------------------------------------------------------------------------------------
# L2: property-based tests (hypothesis)
# --------------------------------------------------------------------------------------


@given(
    tau_s=st.floats(min_value=0.0, max_value=5.0, allow_nan=False),
    dt_s=st.floats(min_value=1e-3, max_value=1.0, allow_nan=False),
    prev_output=st.floats(min_value=-100.0, max_value=100.0, allow_nan=False),
    target=st.floats(min_value=-100.0, max_value=100.0, allow_nan=False),
)
def test_output_always_between_previous_and_target(tau_s, dt_s, prev_output, target):
    """claude-docs/12-testing.md L2: 'actuator output always between previous state and
    command (monotone first-order)'."""
    act = FirstOrderActuator(tau_s=tau_s, dt_s=dt_s, initial_output=prev_output)
    new_output = act.step(target)
    lo, hi = sorted((prev_output, target))
    assert lo - 1e-9 <= new_output <= hi + 1e-9
