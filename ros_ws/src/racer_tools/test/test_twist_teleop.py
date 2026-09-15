"""L1 unit tests for racer_tools.twist_teleop (claude-docs/12-testing.md L1). No ROS, no
subscriber -- pure functions/dataclasses only, same shape as test_keymap.py."""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest
from racer_tools.twist_teleop import (
    DriveCommand,
    TwistTeleopConfig,
    build_twist_teleop_config,
    convert_twist_to_command,
    should_use_zero_command,
)

# --------------------------------------------------------------------------------------
# build_twist_teleop_config
# --------------------------------------------------------------------------------------


def _fake_vehicle_params():
    return SimpleNamespace(
        chassis=SimpleNamespace(wheelbase_m=0.3302),
        steering=SimpleNamespace(min_angle_rad=-0.4189, max_angle_rad=0.4189),
        limits=SimpleNamespace(min_velocity_mps=-5.0, global_speed_cap_mps=20.0),
    )


def test_build_twist_teleop_config_reads_vehicle_params_fields():
    config = build_twist_teleop_config(_fake_vehicle_params())
    assert config.wheelbase_m == pytest.approx(0.3302)
    assert config.steering_min_rad == -0.4189
    assert config.steering_max_rad == 0.4189
    assert config.speed_max_mps == 20.0


def test_build_twist_teleop_config_disables_reverse_by_default():
    """Same default, same reason as keymap.build_teleop_config's (2026-09-14 command-path
    review): no below-neutral throttle pulse until the VESC's PPM control type is known."""
    config = build_twist_teleop_config(_fake_vehicle_params())
    assert config.speed_min_mps == 0.0


def test_build_twist_teleop_config_allow_reverse_restores_the_vehicle_params_floor():
    config = build_twist_teleop_config(_fake_vehicle_params(), allow_reverse=True)
    assert config.speed_min_mps == -5.0


def test_reverse_twist_is_a_zero_command_when_reverse_is_disabled():
    """The behavioural half: a driver pushing the Foxglove Teleop panel backwards gets a stop,
    not a reverse pulse."""
    config = build_twist_teleop_config(_fake_vehicle_params())
    assert convert_twist_to_command(config, linear_x=-2.0, angular_z=0.5) == DriveCommand(0.0, 0.0)


def test_reverse_twist_is_honoured_when_reverse_is_enabled():
    config = build_twist_teleop_config(_fake_vehicle_params(), allow_reverse=True)
    result = convert_twist_to_command(config, linear_x=-2.0, angular_z=0.5)
    assert result.speed_mps == pytest.approx(-2.0)


# --------------------------------------------------------------------------------------
# convert_twist_to_command
# --------------------------------------------------------------------------------------

_CONFIG = TwistTeleopConfig(
    wheelbase_m=0.3302,
    steering_min_rad=-0.4189,
    steering_max_rad=0.4189,
    speed_min_mps=-5.0,
    speed_max_mps=20.0,
)


def test_zero_twist_is_zero_command():
    result = convert_twist_to_command(_CONFIG, linear_x=0.0, angular_z=0.0)
    assert result == DriveCommand(0.0, 0.0)


def test_zero_linear_with_nonzero_angular_is_still_zero_steering():
    """A stationary Ackermann vehicle cannot achieve a nonzero yaw rate by steering alone --
    see this module's docstring's 'speed == 0.0 special case'."""
    result = convert_twist_to_command(_CONFIG, linear_x=0.0, angular_z=2.0)
    assert result == DriveCommand(0.0, 0.0)


def test_forward_no_turn_is_pass_through_speed_zero_steering():
    result = convert_twist_to_command(_CONFIG, linear_x=3.0, angular_z=0.0)
    assert result.speed_mps == pytest.approx(3.0)
    assert result.steering_angle_rad == pytest.approx(0.0)


def test_positive_angular_z_with_forward_speed_is_left_positive_steering():
    """Sign convention (claude-docs/06-vehicle-params.md, REP-103): angular.z > 0 (CCW,
    turn left) with linear.x > 0 (forward) -> steering_angle_rad > 0 (left positive)."""
    result = convert_twist_to_command(_CONFIG, linear_x=2.0, angular_z=1.0)
    assert result.steering_angle_rad > 0.0
    expected = math.atan(0.3302 * 1.0 / 2.0)
    assert result.steering_angle_rad == pytest.approx(expected)


def test_negative_angular_z_with_forward_speed_is_right_negative_steering():
    result = convert_twist_to_command(_CONFIG, linear_x=2.0, angular_z=-1.0)
    assert result.steering_angle_rad < 0.0


def test_reverse_speed_flips_steering_sign_for_the_same_angular_z():
    """Bicycle-model kinematics: achieving the same yaw rate while reversing needs the
    OPPOSITE steering lock direction from doing it while moving forward -- see this module's
    docstring's atan-vs-atan2 explanation."""
    forward = convert_twist_to_command(_CONFIG, linear_x=2.0, angular_z=1.0)
    reverse = convert_twist_to_command(_CONFIG, linear_x=-2.0, angular_z=1.0)
    assert forward.steering_angle_rad > 0.0
    assert reverse.steering_angle_rad < 0.0
    assert reverse.steering_angle_rad == pytest.approx(-forward.steering_angle_rad)


def test_speed_within_bounds_no_clamp():
    result = convert_twist_to_command(_CONFIG, linear_x=15.0, angular_z=0.0)
    assert result.speed_mps == pytest.approx(15.0)


def test_speed_marginal_exactly_at_cap_stays():
    result = convert_twist_to_command(_CONFIG, linear_x=20.0, angular_z=0.0)
    assert result.speed_mps == 20.0


def test_speed_fails_past_cap_clamped():
    result = convert_twist_to_command(_CONFIG, linear_x=25.0, angular_z=0.0)
    assert result.speed_mps == 20.0


def test_speed_fails_past_reverse_cap_clamped():
    result = convert_twist_to_command(_CONFIG, linear_x=-9.0, angular_z=0.0)
    assert result.speed_mps == -5.0


def test_steering_marginal_exactly_at_max_stays():
    # atan(0.3302 * omega / speed) == steering_max_rad exactly, solved for omega.
    omega = math.tan(_CONFIG.steering_max_rad) * 1.0 / _CONFIG.wheelbase_m
    result = convert_twist_to_command(_CONFIG, linear_x=1.0, angular_z=omega)
    assert result.steering_angle_rad == pytest.approx(_CONFIG.steering_max_rad)


def test_steering_fails_past_max_clamped():
    result = convert_twist_to_command(_CONFIG, linear_x=0.1, angular_z=50.0)
    assert result.steering_angle_rad == pytest.approx(_CONFIG.steering_max_rad)


def test_steering_fails_past_min_clamped():
    result = convert_twist_to_command(_CONFIG, linear_x=0.1, angular_z=-50.0)
    assert result.steering_angle_rad == pytest.approx(_CONFIG.steering_min_rad)


@pytest.mark.parametrize(
    "linear_x,angular_z",
    [
        (float("nan"), 0.0),
        (0.0, float("nan")),
        (float("inf"), 0.0),
        (0.0, float("-inf")),
        (float("nan"), float("nan")),
    ],
)
def test_non_finite_input_is_garbage_and_zeroes(linear_x, angular_z):
    """Fail closed on garbage input (claude-docs/05-safety.md), same stance
    racer_safety's gate logic takes for a non-finite /drive_raw command."""
    result = convert_twist_to_command(_CONFIG, linear_x=linear_x, angular_z=angular_z)
    assert result == DriveCommand(0.0, 0.0)


# --------------------------------------------------------------------------------------
# should_use_zero_command
# --------------------------------------------------------------------------------------


def test_no_twist_ever_received_uses_zero():
    assert should_use_zero_command(None, timeout_s=0.5) is True


def test_within_timeout_does_not_use_zero():
    assert should_use_zero_command(0.1, timeout_s=0.5) is False


def test_exactly_at_timeout_boundary_does_not_use_zero():
    assert should_use_zero_command(0.5, timeout_s=0.5) is False


def test_marginally_past_timeout_uses_zero():
    assert should_use_zero_command(0.500001, timeout_s=0.5) is True


def test_well_past_timeout_uses_zero():
    assert should_use_zero_command(5.0, timeout_s=0.5) is True


# A garbage elapsed measurement means "we do not know how stale the driver's input is",
# which must resolve to STOP, not GO (claude-docs/05-safety.md fail-closed; the same stance
# racer_safety's gate logic takes for a garbage drive_raw_age_s). Before this was handled, a
# plain `elapsed > timeout_s` test read a NEGATIVE elapsed time as "fresh", so a browser tab
# closing during a backwards clock step left twist_teleop_adapter_node republishing the
# driver's last non-zero command at 50 Hz until the clock caught up. The node now measures on
# time.monotonic() so it cannot produce these values itself; these cases keep the pure
# function honest for any future caller that does not.


def test_negative_elapsed_time_uses_zero():
    """A backwards clock step must read as STALE, never as fresh."""
    assert should_use_zero_command(-0.001, timeout_s=0.5) is True


def test_large_negative_elapsed_time_uses_zero():
    assert should_use_zero_command(-5.0, timeout_s=0.5) is True


def test_nan_elapsed_time_uses_zero():
    assert should_use_zero_command(float("nan"), timeout_s=0.5) is True


def test_positive_infinite_elapsed_time_uses_zero():
    assert should_use_zero_command(float("inf"), timeout_s=0.5) is True


def test_negative_infinite_elapsed_time_uses_zero():
    assert should_use_zero_command(float("-inf"), timeout_s=0.5) is True


def test_zero_elapsed_time_does_not_use_zero():
    """The boundary on the safe side: a genuinely just-arrived Twist is fresh."""
    assert should_use_zero_command(0.0, timeout_s=0.5) is False
