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
"""

from __future__ import annotations

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


@dataclass(frozen=True)
class TeleopState:
    steering_angle_rad: float = 0.0
    speed_mps: float = 0.0
    quit_requested: bool = False


def build_teleop_config(vehicle_params, control_rate_hz: float) -> TeleopConfig:
    """Build a TeleopConfig from the generated vehicle_params binding
    (`racer_gym`/C++ pattern: `from vehicle_params_generated import VEHICLE_PARAMS`, see
    keyboard_teleop_node.py) and the node's own control-loop rate. Never hand-writes a
    physical constant (CLAUDE.md invariant 2) -- see this module's docstring for the step-size
    derivation."""
    if control_rate_hz <= 0.0:
        raise ValueError("control_rate_hz must be > 0")
    period_s = 1.0 / control_rate_hz
    return TeleopConfig(
        steering_step_rad=vehicle_params.steering.max_rate_rad_per_s * period_s,
        speed_step_mps=vehicle_params.actuation.max_acceleration_mps2 * period_s,
        steering_min_rad=vehicle_params.steering.min_angle_rad,
        steering_max_rad=vehicle_params.steering.max_angle_rad,
        speed_min_mps=vehicle_params.limits.min_velocity_mps,
        speed_max_mps=vehicle_params.limits.global_speed_cap_mps,
    )


def _clamp(value: float, lo: float, hi: float) -> float:
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


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
        new_speed = _clamp(
            new_speed + config.speed_step_mps, config.speed_min_mps, config.speed_max_mps
        )
    elif key in THROTTLE_DOWN_KEYS:
        new_speed = _clamp(
            new_speed - config.speed_step_mps, config.speed_min_mps, config.speed_max_mps
        )

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
