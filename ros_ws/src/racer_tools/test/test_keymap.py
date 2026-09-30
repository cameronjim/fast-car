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
    derived_steps,
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
    assert config.speed_max_mps == 20.0


def test_build_teleop_config_disables_reverse_by_default():
    """Default allow_reverse=False clamps the speed floor at 0.0, not
    limits.min_velocity_mps. Added by the 2026-09-14 command-path review: what a below-neutral
    throttle pulse does is a VESC PPM control-type setting that has never been applied to this
    ESC, so the first drives do not emit one."""
    config = build_teleop_config(_fake_vehicle_params(), control_rate_hz=50.0)
    assert config.speed_min_mps == 0.0


def test_build_teleop_config_allow_reverse_restores_the_vehicle_params_floor():
    config = build_teleop_config(_fake_vehicle_params(), control_rate_hz=50.0, allow_reverse=True)
    assert config.speed_min_mps == -5.0


def test_throttle_down_from_rest_stays_at_zero_when_reverse_is_disabled():
    """The behavioural half: with the default config the throttle-down key decelerates to a
    stop and stops there, rather than spinning the motor backwards."""
    config = build_teleop_config(_fake_vehicle_params(), control_rate_hz=50.0)
    state = apply_key(config, TeleopState(), "s")
    assert state.speed_mps == 0.0
    state = apply_key(config, state, "DOWN")
    assert state.speed_mps == 0.0


def test_throttle_down_from_speed_decelerates_to_zero_and_stops_there():
    config = build_teleop_config(_fake_vehicle_params(), control_rate_hz=50.0)
    state = TeleopState(speed_mps=config.speed_step_mps * 1.5)
    state = apply_key(config, state, "s")
    assert state.speed_mps == pytest.approx(config.speed_step_mps * 0.5)
    state = apply_key(config, state, "s")
    assert state.speed_mps == 0.0
    state = apply_key(config, state, "s")
    assert state.speed_mps == 0.0


def test_build_teleop_config_scales_with_rate():
    slow = build_teleop_config(_fake_vehicle_params(), control_rate_hz=10.0)
    fast = build_teleop_config(_fake_vehicle_params(), control_rate_hz=100.0)
    assert slow.steering_step_rad == pytest.approx(fast.steering_step_rad * 10.0)


@pytest.mark.parametrize("bad_rate", [0.0, -1.0, -50.0])
def test_build_teleop_config_rejects_non_positive_rate(bad_rate):
    with pytest.raises(ValueError):
        build_teleop_config(_fake_vehicle_params(), control_rate_hz=bad_rate)


# --------------------------------------------------------------------------------------
# tap-size overrides (GitHub issue #72)
# --------------------------------------------------------------------------------------


def test_derived_steps_are_the_default_when_no_override_is_given():
    vp = _fake_vehicle_params()
    steering_step, speed_step = derived_steps(vp, 50.0)
    assert steering_step == pytest.approx(3.2 / 50.0)
    assert speed_step == pytest.approx(9.51 / 50.0)
    config = build_teleop_config(vp, 50.0, speed_step_mps=None, steering_step_rad=None)
    assert config.steering_step_rad == steering_step
    assert config.speed_step_mps == speed_step


def test_speed_step_override_replaces_only_the_speed_step():
    config = build_teleop_config(_fake_vehicle_params(), 50.0, speed_step_mps=0.25)
    assert config.speed_step_mps == 0.25
    assert config.steering_step_rad == pytest.approx(3.2 / 50.0)
    # Ranges are untouched by an override.
    assert config.speed_min_mps == 0.0
    assert config.speed_max_mps == 20.0


def test_steering_step_override_replaces_only_the_steering_step():
    config = build_teleop_config(_fake_vehicle_params(), 50.0, steering_step_rad=0.1)
    assert config.steering_step_rad == 0.1
    assert config.speed_step_mps == pytest.approx(9.51 / 50.0)
    assert config.steering_min_rad == -0.4189
    assert config.steering_max_rad == 0.4189


def test_overrides_are_independent_of_the_control_rate():
    slow = build_teleop_config(_fake_vehicle_params(), 10.0, speed_step_mps=0.25)
    fast = build_teleop_config(_fake_vehicle_params(), 100.0, speed_step_mps=0.25)
    assert slow.speed_step_mps == fast.speed_step_mps == 0.25


@pytest.mark.parametrize("bad", [0.0, -0.25, float("nan"), float("inf"), float("-inf")])
def test_speed_step_override_must_be_finite_and_positive(bad):
    with pytest.raises(ValueError, match="speed_step_mps"):
        build_teleop_config(_fake_vehicle_params(), 50.0, speed_step_mps=bad)


@pytest.mark.parametrize("bad", [0.0, -0.1, float("nan"), float("inf")])
def test_steering_step_override_must_be_finite_and_positive(bad):
    with pytest.raises(ValueError, match="steering_step_rad"):
        build_teleop_config(_fake_vehicle_params(), 50.0, steering_step_rad=bad)


def test_one_tap_with_the_first_drive_step_is_exactly_that_step():
    config = build_teleop_config(_fake_vehicle_params(), 50.0, speed_step_mps=0.25)
    state = apply_key(config, TeleopState(), "w")
    assert state.speed_mps == 0.25


def test_holding_the_key_still_ramps_one_step_per_event():
    """Hold-to-ramp is kept: each key-repeat event adds exactly one step, no acceleration."""
    config = build_teleop_config(_fake_vehicle_params(), 50.0, speed_step_mps=0.25)
    state = TeleopState()
    for n in range(1, 9):
        state = apply_key(config, state, "w")
        assert state.speed_mps == pytest.approx(0.25 * n)


def test_a_large_speed_step_override_is_clamped_to_the_cap_and_the_floor():
    config = build_teleop_config(_fake_vehicle_params(), 50.0, speed_step_mps=100.0)
    state = apply_key(config, TeleopState(), "w")
    assert state.speed_mps == 20.0  # limits.global_speed_cap_mps
    state = apply_key(config, state, "s")
    assert state.speed_mps == 0.0  # reverse disabled: floor is zero, never below


def test_a_large_speed_step_override_with_reverse_clamps_at_min_velocity():
    config = build_teleop_config(
        _fake_vehicle_params(), 50.0, allow_reverse=True, speed_step_mps=100.0
    )
    state = apply_key(config, TeleopState(), "s")
    assert state.speed_mps == -5.0  # limits.min_velocity_mps


def test_a_large_steering_step_override_is_clamped_to_the_angle_limits():
    config = build_teleop_config(_fake_vehicle_params(), 50.0, steering_step_rad=10.0)
    state = apply_key(config, TeleopState(), "a")
    assert state.steering_angle_rad == 0.4189
    state = apply_key(config, state, "d")
    state = apply_key(config, state, "d")
    assert state.steering_angle_rad == -0.4189


def test_speed_step_that_does_not_divide_the_cap_saturates_exactly_at_the_cap():
    config = build_teleop_config(_fake_vehicle_params(), 50.0, speed_step_mps=0.3)
    state = TeleopState(speed_mps=19.9)
    state = apply_key(config, state, "w")
    assert state.speed_mps == 20.0


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


# --------------------------------------------------------------------------------------
# min_speed_mps: minimum commanded speed while moving (0.0 = disabled)
# --------------------------------------------------------------------------------------


def _min_config(min_speed=0.8, step=0.25, allow_reverse=False):
    return build_teleop_config(
        _fake_vehicle_params(),
        50.0,
        allow_reverse=allow_reverse,
        speed_step_mps=step,
        min_speed_mps=min_speed,
    )


def _press(config, keys, speed=0.0):
    state = TeleopState(speed_mps=speed)
    for key in keys:
        state = apply_key(config, state, key)
    return state.speed_mps


def test_min_speed_defaults_to_disabled_in_config():
    assert build_teleop_config(_fake_vehicle_params(), 50.0).min_speed_mps == 0.0


def test_min_speed_from_rest_jumps_to_the_minimum():
    assert _press(_min_config(), "w") == 0.8


@pytest.mark.parametrize("key", ["w", "W", "UP"])
def test_min_speed_applies_to_every_throttle_up_key(key):
    assert _press(_min_config(), [key]) == 0.8


def test_min_speed_from_rest_uses_the_step_when_the_step_is_larger():
    assert _press(_min_config(min_speed=0.8, step=1.5), "w") == 1.5


def test_min_speed_above_the_minimum_adds_one_step_per_tap():
    assert _press(_min_config(), "ww") == pytest.approx(1.05)
    assert _press(_min_config(), "www") == pytest.approx(1.30)


def test_min_speed_throttle_down_above_the_minimum_subtracts_one_step():
    assert _press(_min_config(), "s", speed=1.30) == pytest.approx(1.05)
    assert _press(_min_config(), "s", speed=1.05) == pytest.approx(0.80)


def test_min_speed_throttle_down_below_the_minimum_goes_to_exactly_zero():
    config = _min_config()
    # 0.8 - 0.25 = 0.55 < 0.8: clean stop, no lingering crawl.
    assert _press(config, "s", speed=0.8) == 0.0
    assert _press(config, "wwss") == 0.0
    # The ladder works back down to zero and stays there (reverse disabled).
    assert _press(config, "wwwsssss") == 0.0


def test_min_speed_throttle_down_landing_exactly_on_the_minimum_stays():
    assert _press(_min_config(min_speed=0.8, step=0.5), "s", speed=1.3) == pytest.approx(0.8)


def test_min_speed_throttle_down_at_rest_stays_at_rest_without_reverse():
    assert _press(_min_config(), "s") == 0.0


def test_min_speed_clamps_at_the_maximum():
    config = _min_config()
    assert _press(config, "w" * 100) == 20.0
    assert _press(_min_config(min_speed=20.0), "w") == 20.0


def test_min_speed_equal_to_the_maximum_is_valid():
    assert _min_config(min_speed=20.0).min_speed_mps == 20.0


def test_min_speed_zero_is_the_original_behaviour():
    config = _min_config(min_speed=0.0)
    assert _press(config, "w") == 0.25
    assert _press(config, "ww") == 0.5
    assert _press(config, "s", speed=0.3) == pytest.approx(0.05)
    assert _press(config, "s", speed=0.1) == 0.0  # clamped at 0 (no reverse), as before


def test_min_speed_zero_with_reverse_matches_plain_add_subtract():
    config = _min_config(min_speed=0.0, allow_reverse=True)
    assert _press(config, "s") == -0.25
    assert _press(config, "s", speed=0.1) == pytest.approx(-0.15)


def test_min_speed_space_and_q_still_zero_the_speed():
    config = _min_config()
    for key in (" ", "q", "Q"):
        assert apply_key(config, TeleopState(speed_mps=3.0), key).speed_mps == 0.0


def test_min_speed_does_not_touch_steering():
    config = _min_config()
    state = apply_key(config, TeleopState(), "a")
    assert state.steering_angle_rad == pytest.approx(config.steering_step_rad)
    assert state.speed_mps == 0.0


def test_min_speed_negative_side_from_rest_jumps_to_minus_the_minimum():
    assert _press(_min_config(allow_reverse=True), "s") == -0.8


def test_min_speed_negative_side_from_rest_uses_the_step_when_larger():
    assert _press(_min_config(step=1.5, allow_reverse=True), "s") == -1.5


def test_min_speed_negative_side_grows_by_one_step():
    assert _press(_min_config(allow_reverse=True), "ss") == pytest.approx(-1.05)


def test_min_speed_negative_side_throttle_up_below_the_minimum_goes_to_zero():
    config = _min_config(allow_reverse=True)
    assert _press(config, "w", speed=-0.8) == 0.0
    assert _press(config, "w", speed=-1.05) == pytest.approx(-0.8)
    assert _press(config, "ssw") == pytest.approx(-0.8)
    assert _press(config, "sw") == 0.0


def test_min_speed_does_not_cross_zero_in_one_tap_either_way():
    config = _min_config(allow_reverse=True)
    # Forward 0.8, down: 0.55 < 0.8 -> 0, not -0.x. Reverse needs a further tap.
    assert _press(config, "s", speed=0.8) == 0.0
    assert _press(config, "ss", speed=0.8) == -0.8
    assert _press(config, "w", speed=-0.8) == 0.0
    assert _press(config, "ww", speed=-0.8) == 0.8


def test_min_speed_negative_side_clamps_at_the_reverse_limit():
    assert _press(_min_config(allow_reverse=True), "s" * 100) == -5.0


def test_min_speed_a_huge_step_from_rest_saturates_at_the_limits():
    config = _min_config(step=100.0, allow_reverse=True)
    assert _press(config, "w") == 20.0
    assert _press(config, "s") == -5.0


@pytest.mark.parametrize("bad", [-0.1, -1.0, float("nan"), float("inf"), float("-inf")])
def test_min_speed_must_be_finite_and_non_negative(bad):
    with pytest.raises(ValueError, match="min_speed_mps"):
        build_teleop_config(_fake_vehicle_params(), 50.0, min_speed_mps=bad)


def test_min_speed_above_the_maximum_speed_is_rejected():
    with pytest.raises(ValueError, match="maximum speed"):
        build_teleop_config(_fake_vehicle_params(), 50.0, min_speed_mps=20.5)


def test_min_speed_above_the_reverse_limit_is_rejected_only_when_reverse_is_allowed():
    build_teleop_config(_fake_vehicle_params(), 50.0, min_speed_mps=6.0)  # fine, no reverse
    with pytest.raises(ValueError, match="reverse limit"):
        build_teleop_config(_fake_vehicle_params(), 50.0, allow_reverse=True, min_speed_mps=6.0)
