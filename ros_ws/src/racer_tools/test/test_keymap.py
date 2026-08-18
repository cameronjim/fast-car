"""L1 unit tests for racer_tools.keymap (claude-docs/12-testing.md: "Teleop keymap logic
pytest"). No TTY, no ROS -- pure functions/dataclasses only.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest
from racer_tools.keymap import (
    TeleopConfig,
    TeleopState,
    apply_key,
    build_teleop_config,
    decode_key,
)

# --------------------------------------------------------------------------------------
# decode_key
# --------------------------------------------------------------------------------------


def test_decode_key_empty_string_is_none():
    assert decode_key("") is None


@pytest.mark.parametrize(
    "raw,expected",
    [
        ("\x1b[A", "UP"),
        ("\x1b[B", "DOWN"),
        ("\x1b[C", "RIGHT"),
        ("\x1b[D", "LEFT"),
    ],
)
def test_decode_key_recognizes_arrow_escape_sequences(raw, expected):
    assert decode_key(raw) == expected


@pytest.mark.parametrize("raw", ["w", "a", "s", "d", "q", "Q", " ", "x", "1", "\x1b"])
def test_decode_key_passes_through_single_characters(raw):
    """A single character passes through unchanged, including a bare ESC with no following
    bytes (a lone ESC keypress, not part of an arrow-key sequence) -- apply_key simply does
    not recognize it as any mapped key, so this is a harmless no-op end to end."""
    assert decode_key(raw) == raw


@pytest.mark.parametrize("raw", ["\x1b[Z", "\x1b[AB", "garbage", "\x1b[A\x1b[B"])
def test_decode_key_returns_none_for_unrecognized_or_incomplete_multi_char_sequences(raw):
    assert decode_key(raw) is None


# --------------------------------------------------------------------------------------
# build_teleop_config
# --------------------------------------------------------------------------------------


def _fake_vehicle_params():
    return SimpleNamespace(
        steering=SimpleNamespace(
            max_rate_rad_per_s=3.2, min_angle_rad=-0.4189, max_angle_rad=0.4189
        ),
        actuation=SimpleNamespace(max_acceleration_mps2=9.51),
        limits=SimpleNamespace(min_velocity_mps=-5.0, global_speed_cap_mps=20.0),
    )


def test_build_teleop_config_derives_step_sizes_from_vehicle_params_and_rate():
    config = build_teleop_config(_fake_vehicle_params(), control_rate_hz=50.0)
    assert config.steering_step_rad == pytest.approx(3.2 / 50.0)
    assert config.speed_step_mps == pytest.approx(9.51 / 50.0)
    assert config.steering_min_rad == -0.4189
    assert config.steering_max_rad == 0.4189
    assert config.speed_min_mps == -5.0
    assert config.speed_max_mps == 20.0


def test_build_teleop_config_scales_with_rate():
    slow = build_teleop_config(_fake_vehicle_params(), control_rate_hz=10.0)
    fast = build_teleop_config(_fake_vehicle_params(), control_rate_hz=100.0)
    assert slow.steering_step_rad == pytest.approx(fast.steering_step_rad * 10.0)


@pytest.mark.parametrize("bad_rate", [0.0, -1.0, -50.0])
def test_build_teleop_config_rejects_non_positive_rate(bad_rate):
    with pytest.raises(ValueError):
        build_teleop_config(_fake_vehicle_params(), control_rate_hz=bad_rate)


# --------------------------------------------------------------------------------------
# apply_key
# --------------------------------------------------------------------------------------

_CONFIG = TeleopConfig(
    steering_step_rad=0.1,
    speed_step_mps=1.0,
    steering_min_rad=-0.4189,
    steering_max_rad=0.4189,
    speed_min_mps=-5.0,
    speed_max_mps=20.0,
)


@pytest.mark.parametrize("key", ["w", "W", "UP"])
def test_throttle_up_keys_increase_speed(key):
    state = apply_key(_CONFIG, TeleopState(), key)
    assert state.speed_mps == pytest.approx(1.0)
    assert state.steering_angle_rad == 0.0
    assert not state.quit_requested


@pytest.mark.parametrize("key", ["s", "S", "DOWN"])
def test_throttle_down_keys_decrease_speed(key):
    state = apply_key(_CONFIG, TeleopState(), key)
    assert state.speed_mps == pytest.approx(-1.0)


def test_throttle_up_passes_within_bounds_no_clamp():
    state = TeleopState(speed_mps=18.5)
    result = apply_key(_CONFIG, state, "w")
    assert result.speed_mps == pytest.approx(19.5)


def test_throttle_up_marginal_exactly_at_cap_stays():
    state = TeleopState(speed_mps=20.0)
    result = apply_key(_CONFIG, state, "w")
    assert result.speed_mps == 20.0


def test_throttle_up_fails_past_cap_clamped():
    state = TeleopState(speed_mps=19.9)
    result = apply_key(_CONFIG, state, "w")
    assert result.speed_mps == 20.0


def test_throttle_down_fails_past_reverse_cap_clamped():
    state = TeleopState(speed_mps=-4.9)
    result = apply_key(_CONFIG, state, "s")
    assert result.speed_mps == -5.0


@pytest.mark.parametrize("key", ["a", "A", "LEFT"])
def test_steer_left_keys_increase_steering_angle_left_positive(key):
    """Sign convention (claude-docs/06-vehicle-params.md): LEFT positive."""
    state = apply_key(_CONFIG, TeleopState(), key)
    assert state.steering_angle_rad == pytest.approx(0.1)


@pytest.mark.parametrize("key", ["d", "D", "RIGHT"])
def test_steer_right_keys_decrease_steering_angle(key):
    state = apply_key(_CONFIG, TeleopState(), key)
    assert state.steering_angle_rad == pytest.approx(-0.1)


def test_steer_left_marginal_exactly_at_max_stays():
    state = TeleopState(steering_angle_rad=0.4189)
    result = apply_key(_CONFIG, state, "a")
    assert result.steering_angle_rad == 0.4189


def test_steer_left_fails_past_max_clamped():
    state = TeleopState(steering_angle_rad=0.35)
    result = apply_key(_CONFIG, state, "a")
    assert result.steering_angle_rad == 0.4189


def test_steer_right_fails_past_min_clamped():
    state = TeleopState(steering_angle_rad=-0.35)
    result = apply_key(_CONFIG, state, "d")
    assert result.steering_angle_rad == -0.4189


def test_stop_key_zeroes_both_but_does_not_quit():
    state = TeleopState(steering_angle_rad=0.2, speed_mps=5.0)
    result = apply_key(_CONFIG, state, " ")
    assert result.steering_angle_rad == 0.0
    assert result.speed_mps == 0.0
    assert not result.quit_requested


@pytest.mark.parametrize("key", ["q", "Q"])
def test_quit_key_zeroes_both_and_sets_quit_requested(key):
    state = TeleopState(steering_angle_rad=0.2, speed_mps=5.0)
    result = apply_key(_CONFIG, state, key)
    assert result.steering_angle_rad == 0.0
    assert result.speed_mps == 0.0
    assert result.quit_requested


def test_none_key_is_a_no_op():
    state = TeleopState(steering_angle_rad=0.2, speed_mps=5.0)
    result = apply_key(_CONFIG, state, None)
    assert result is state


@pytest.mark.parametrize("key", ["x", "1", "z", "\t"])
def test_unrecognized_key_is_a_no_op(key):
    state = TeleopState(steering_angle_rad=0.2, speed_mps=5.0)
    result = apply_key(_CONFIG, state, key)
    assert result.steering_angle_rad == 0.2
    assert result.speed_mps == 5.0
    assert not result.quit_requested


def test_throttle_and_steer_keys_combine_independently():
    # Not simultaneous in one call (one key per call), but two independent presses compose.
    state = apply_key(_CONFIG, TeleopState(), "w")
    state = apply_key(_CONFIG, state, "a")
    assert state.speed_mps == pytest.approx(1.0)
    assert state.steering_angle_rad == pytest.approx(0.1)
