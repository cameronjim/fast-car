"""Pure keymap/step/clamp logic for keyboard_teleop_node (roadmap milestone 1,
claude-docs/12-testing.md L1: "unit-tested without a TTY"). No termios, no file descriptors,
no ROS -- keyboard_teleop_node.py's tty-reading loop is thin plumbing that calls
`decode_key` then `apply_key` and is not itself exercised by unit tests.

Sign convention (claude-docs/06-vehicle-params.md, REP-103): steering angle is the
road-wheel angle in radians, LEFT positive. Pressing the LEFT/'a' key therefore INCREASES
steering_angle_rad (more left); RIGHT/'d' DECREASES it (more right) -- verified against that
convention directly in test/test_keymap.py.

Step sizes come from the generated vehicle_params Python binding (CLAUDE.md invariant 2:
never hand-write a physical constant), not an invented "feels right" number: one keypress
moves steering/speed by what the vehicle could physically achieve in one control-loop period
at its own maximum rate (`steering.max_rate_rad_per_s` / `actuation.max_acceleration_mps2`),
scaled by `1 / control_rate_hz`. `control_rate_hz` itself is loop-rate tuning (like
tracker_node's lookahead gains), not a physical constant, so it is a plain function
parameter here (the ROS node declares it as a parameter, default 50 Hz per
claude-docs/04-architecture.md).

That derivation is the DEFAULT tap size, and it can be overridden (GitHub issue #72): a tap
is an operator-interface choice, not a physical constant, and the derived 0.19 m/s per tap
was too fine for a first drive. `build_teleop_config` takes optional `speed_step_mps` /
`steering_step_rad` overrides (the node's ROS parameters of the same names), which must be
finite and > 0. Whatever the step, every command is still clamped to the vehicle_params
ranges below, so a large step saturates at the limit rather than exceeding it, and holding a
key still ramps one step per key-repeat event.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

# --------------------------------------------------------------------------------------
# Raw terminal input -> symbolic key (pure; the tty plumbing hands this whatever bytes it
# already read, never touches termios/select itself).
# --------------------------------------------------------------------------------------

_ESCAPE_SEQUENCE_MAP = {
    "\x1b[A": "UP",
    "\x1b[B": "DOWN",
    "\x1b[C": "RIGHT",
    "\x1b[D": "LEFT",
}


def decode_key(raw: str) -> str | None:
    """Normalize a raw chunk of terminal input into the keys `apply_key` understands, or
    None for anything unrecognized (including empty input or an incomplete/unknown escape
    sequence -- a no-op, not an error, since raw terminal input is not a validated channel)."""
    if not raw:
        return None
    if raw in _ESCAPE_SEQUENCE_MAP:
        return _ESCAPE_SEQUENCE_MAP[raw]
    if len(raw) == 1:
        return raw
    return None


# --------------------------------------------------------------------------------------
# Keymap: WASD or arrows (roadmap milestone 1 instructions).
# --------------------------------------------------------------------------------------

THROTTLE_UP_KEYS = frozenset({"w", "W", "UP"})
THROTTLE_DOWN_KEYS = frozenset({"s", "S", "DOWN"})
STEER_LEFT_KEYS = frozenset({"a", "A", "LEFT"})
STEER_RIGHT_KEYS = frozenset({"d", "D", "RIGHT"})
STOP_KEYS = frozenset({" "})
QUIT_KEYS = frozenset({"q", "Q"})


@dataclass(frozen=True)
class TeleopConfig:
    steering_step_rad: float
    speed_step_mps: float
    steering_min_rad: float
    steering_max_rad: float
    speed_min_mps: float
    speed_max_mps: float
    # Minimum commanded |speed| while moving (0.0 = disabled, the original behaviour). See
    # `_step_speed`: a throttle key never produces a speed with 0 < |speed| < min_speed_mps.
    min_speed_mps: float = 0.0


@dataclass(frozen=True)
class TeleopState:
    steering_angle_rad: float = 0.0
    speed_mps: float = 0.0
    quit_requested: bool = False


def derived_steps(vehicle_params, control_rate_hz: float) -> tuple[float, float]:
    """The default (steering_step_rad, speed_step_mps): what the vehicle could achieve in one
    control period at its own maximum steering rate / acceleration (this module's docstring).
    The node uses these as the declared defaults of its `steering_step_rad` /
    `speed_step_mps` parameters."""
    if not (math.isfinite(control_rate_hz) and control_rate_hz > 0.0):
        raise ValueError("control_rate_hz must be > 0")
    period_s = 1.0 / control_rate_hz
    return (
        vehicle_params.steering.max_rate_rad_per_s * period_s,
        vehicle_params.actuation.max_acceleration_mps2 * period_s,
    )


def _checked_step(name: str, value: float) -> float:
    value = float(value)
    if not (math.isfinite(value) and value > 0.0):
        raise ValueError(f"{name} must be finite and > 0, got {value!r}")
    return value


def build_teleop_config(
    vehicle_params,
    control_rate_hz: float,
    allow_reverse: bool = False,
    speed_step_mps: float | None = None,
    steering_step_rad: float | None = None,
    min_speed_mps: float = 0.0,
) -> TeleopConfig:
    """Build a TeleopConfig from the generated vehicle_params binding
    (`racer_gym`/C++ pattern: `from vehicle_params_generated import VEHICLE_PARAMS`, see
    keyboard_teleop_node.py) and the node's own control-loop rate. Never hand-writes a
    physical constant (CLAUDE.md invariant 2) -- see this module's docstring for the step-size
    derivation.

    `allow_reverse` (default False) is the one place the reverse half of the speed range is
    turned on. With it False the lower clamp is 0.0 m/s instead of
    `vehicle_params.limits.min_velocity_mps` (-5.0), so the throttle-down key decelerates to a
    stop and stops there.

    Why False is the default, on a car nobody has driven: what a below-neutral pulse DOES is a
    property of the VESC's configured PPM control type, not of this code. In "Current" it is
    reverse drive current; in "Current No Reverse With Brake" it is proportional braking. That
    setting has never been applied to this ESC, so the honest position for the first drives is
    not to emit below-neutral pulses at all. It is a declared ROS parameter on the node
    (`allow_reverse`), not a constant, so the bench can turn it on deliberately once the VESC
    control type is known and recorded. See docs/notes/first-boot-runbook.md.

    `speed_step_mps` / `steering_step_rad` override the derived tap sizes when not None
    (GitHub issue #72). A non-finite or non-positive override raises ValueError: a zero step
    would make the key dead, a negative one would invert it, and neither is a tap size. There
    is no upper bound here because `apply_key` clamps every result to the ranges below.

    `min_speed_mps` (default 0.0 = disabled) is the smallest |speed| a throttle key will
    command while moving: a tap from rest jumps straight to it (or to one step if that is
    larger), and a throttle tap that would land below it stops the car instead of leaving a
    crawl. It must be finite, >= 0 and no larger than the speed range it applies to (the
    maximum speed, and the reverse limit too when `allow_reverse`), otherwise the clamp would
    silently produce a speed below the minimum, so it raises ValueError and the node refuses
    to start.
    """
    derived_steering_step, derived_speed_step = derived_steps(vehicle_params, control_rate_hz)
    min_speed_mps = float(min_speed_mps)
    speed_min_mps = vehicle_params.limits.min_velocity_mps if allow_reverse else 0.0
    speed_max_mps = vehicle_params.limits.global_speed_cap_mps
    if not (math.isfinite(min_speed_mps) and min_speed_mps >= 0.0):
        raise ValueError(f"min_speed_mps must be finite and >= 0, got {min_speed_mps!r}")
    if min_speed_mps > speed_max_mps:
        raise ValueError(
            f"min_speed_mps {min_speed_mps} exceeds the maximum speed {speed_max_mps} m/s"
        )
    if allow_reverse and min_speed_mps > -speed_min_mps:
        raise ValueError(
            f"min_speed_mps {min_speed_mps} exceeds the reverse limit {-speed_min_mps} m/s"
        )
    return TeleopConfig(
        steering_step_rad=(
            derived_steering_step
            if steering_step_rad is None
            else _checked_step("steering_step_rad", steering_step_rad)
        ),
        speed_step_mps=(
            derived_speed_step
            if speed_step_mps is None
            else _checked_step("speed_step_mps", speed_step_mps)
        ),
        steering_min_rad=vehicle_params.steering.min_angle_rad,
        steering_max_rad=vehicle_params.steering.max_angle_rad,
        speed_min_mps=speed_min_mps,
        speed_max_mps=speed_max_mps,
        min_speed_mps=min_speed_mps,
    )


def _clamp(value: float, lo: float, hi: float) -> float:
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def _step_speed(config: TeleopConfig, speed: float, direction: int) -> float:
    """One throttle tap: direction +1 (throttle-up) or -1 (throttle-down). With
    `min_speed_mps` == 0 this is the plain `speed + direction * step`, clamped. With a minimum
    m > 0, the rule is on |speed| and is symmetric about zero:

      * from rest, the tap sets |speed| = max(m, step) in that direction;
      * growing |speed| adds one step (clamped to the range);
      * shrinking |speed| subtracts one step, but if the result would be below m in
        magnitude (or cross zero) it is exactly 0: a clean stop, never a crawl below m.
        Crossing through zero to the other direction takes a further tap.
    """
    step = config.speed_step_mps
    minimum = config.min_speed_mps
    if speed == 0.0:
        candidate = direction * max(minimum, step)
    else:
        candidate = speed + direction * step
        shrinking = speed * direction < 0.0
        if shrinking and minimum > 0.0 and (candidate * speed <= 0.0 or abs(candidate) < minimum):
            candidate = 0.0
    return _clamp(candidate, config.speed_min_mps, config.speed_max_mps)


def apply_key(config: TeleopConfig, state: TeleopState, key: str | None) -> TeleopState:
    """Apply one decoded key to `state`, returning the new state. Unrecognized/None keys are
    a no-op (state returned unchanged) -- garbage/unmapped terminal input must never raise or
    silently do something surprising."""
    if key is None:
        return state
    if key in QUIT_KEYS:
        # "q quits after publishing a zero command" (milestone 1 instructions): zero the
        # command here so the node's next publish (before it exits) is the zero command,
        # rather than leaving whatever speed/steering was last commanded.
        return TeleopState(steering_angle_rad=0.0, speed_mps=0.0, quit_requested=True)
    if key in STOP_KEYS:
        return replace(state, steering_angle_rad=0.0, speed_mps=0.0)

    new_steering = state.steering_angle_rad
    new_speed = state.speed_mps

    if key in THROTTLE_UP_KEYS:
        new_speed = _step_speed(config, new_speed, +1)
    elif key in THROTTLE_DOWN_KEYS:
        new_speed = _step_speed(config, new_speed, -1)

    if key in STEER_LEFT_KEYS:
        # LEFT positive (claude-docs/06-vehicle-params.md) -- pressing left INCREASES the
        # steering angle.
        new_steering = _clamp(
            new_steering + config.steering_step_rad,
            config.steering_min_rad,
            config.steering_max_rad,
        )
    elif key in STEER_RIGHT_KEYS:
        new_steering = _clamp(
            new_steering - config.steering_step_rad,
            config.steering_min_rad,
            config.steering_max_rad,
        )

    if new_steering == state.steering_angle_rad and new_speed == state.speed_mps:
        return state
    return replace(state, steering_angle_rad=new_steering, speed_mps=new_speed)
