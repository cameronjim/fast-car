"""L1/L2 tests for racer_gym.dynamics.delay (transport delay -- claude-docs/07-sim-and-
sysid.md requirement 4, claude-docs/12-testing.md)."""

from __future__ import annotations

import pytest
from hypothesis import given
from hypothesis import strategies as st
from racer_gym.dynamics.delay import TransportDelay

# --------------------------------------------------------------------------------------
# L1: hand-computed cases
# --------------------------------------------------------------------------------------


def test_zero_delay_is_passthrough():
    delay = TransportDelay(delay_steps=0)
    for command in (1.0, 2.0, 3.0):
        assert delay.step(command) == command


def test_delay_applies_exactly_n_steps_late():
    n = 3
    delay = TransportDelay(delay_steps=n, initial_value=-1.0)
    commands = [10.0, 11.0, 12.0, 13.0, 14.0, 15.0]
    outputs = [delay.step(c) for c in commands]
    # first n outputs are the initial (pre-episode) value
    assert outputs[:n] == [-1.0] * n
    # from step n onward, output[k] == commands[k - n]
    for k in range(n, len(commands)):
        assert outputs[k] == commands[k - n]


def test_reset_clears_buffer_to_initial_value():
    delay = TransportDelay(delay_steps=2, initial_value=0.0)
    delay.step(5.0)
    delay.step(6.0)
    delay.reset()
    assert delay.step(99.0) == 0.0
    assert delay.step(99.0) == 0.0
    assert delay.step(99.0) == 99.0


def test_reset_with_new_initial_value():
    delay = TransportDelay(delay_steps=1, initial_value=0.0)
    delay.reset(initial_value=7.0)
    assert delay.step(1.0) == 7.0
    assert delay.step(2.0) == 1.0


def test_negative_delay_steps_raises():
    with pytest.raises(ValueError):
        TransportDelay(delay_steps=-1)


# --------------------------------------------------------------------------------------
# L2: property-based tests (hypothesis)
# --------------------------------------------------------------------------------------


@given(
    delay_steps=st.integers(min_value=0, max_value=20),
    commands=st.lists(
        st.floats(allow_nan=False, allow_infinity=False, width=32), min_size=1, max_size=50
    ),
)
def test_delay_never_reorders_commands(delay_steps, commands):
    """claude-docs/12-testing.md L2: 'delay buffer never reorders commands'. The sequence of
    non-initial-value outputs, in order, must equal a prefix of the input command sequence in
    the same order they were pushed (FIFO)."""
    delay = TransportDelay(delay_steps=delay_steps, initial_value=None or 0.0)
    outputs = [delay.step(c) for c in commands]
    if delay_steps == 0:
        assert outputs == commands
        return
    delayed_outputs = outputs[delay_steps:]
    expected = commands[: len(delayed_outputs)]
    assert delayed_outputs == expected
