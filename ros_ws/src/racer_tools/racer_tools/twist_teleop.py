"""Pure Twist -> AckermannDrive conversion + staleness logic for
twist_teleop_adapter_node (roadmap milestone 5, claude-docs/04-architecture.md).

No rclpy, no termios -- unit-tested without ROS, same split as racer_tools.keymap
(claude-docs/12-testing.md L1: "unit-tested without a TTY"; here, without a subscriber).
twist_teleop_adapter_node.py is thin rclpy plumbing that calls `convert_twist_to_command`
and `should_use_zero_command` and publishes whatever comes back.

Conversion: geometry_msgs/Twist's `linear.x` (m/s) and `angular.z` (rad/s, REP-103: positive
= counter-clockwise = left) are converted to an Ackermann (speed, steering_angle) command via
the standard bicycle-model inverse: for a desired forward speed ``v`` and yaw rate ``omega``,
the steering angle ``delta`` that produces them satisfies ``omega = v * tan(delta) / L``
(``L`` = wheelbase), so ``delta = atan(L * omega / v)``. This is the same relation
`racer_control`'s pure pursuit controller implicitly inverts the other way (steering ->
implied arc), and is the standard "twist to ackermann" conversion (e.g. ROS's
`twist_to_ackermann_drive`). Plain `atan` (not `atan2`) is used deliberately: dividing by the
(possibly negative, for reverse) commanded speed and taking `atan` of the ratio keeps the
sign flip that reversing implies -- turning the wheel to reproduce a given yaw rate while
backing up requires the opposite lock direction from doing it while moving forward, which is
exactly what the bicycle model says and exactly what plain `atan` (unlike `atan2`, which
would treat the numerator/denominator as a 2D point and wrap into the wrong quadrant for a
negative speed) reproduces.

`speed` is clamped FIRST (to `vehicle_params.limits.min_velocity_mps` /
`global_speed_cap_mps`, the SAME bounds `keyboard_teleop_node` uses), and the (possibly
clamped) speed is what the steering conversion divides by -- if a driver asks for more speed
than the car can do, the steering angle needed to hit the requested yaw rate at the LOWER,
actually-achievable speed is larger, which is the kinematically honest answer, not an
arbitrary one. `speed == 0.0` (exactly, after clamping) is a special case: a stationary
Ackermann vehicle cannot achieve any nonzero yaw rate by turning its wheels (unlike a
differential-drive base), so the steering angle is 0.0 rather than dividing by zero or
returning +/-90 degrees for a symbolically nonzero omega. This also gives "zero Twist means
zero command" for free: `linear.x=0, angular.z=0` clamps to `speed=0.0`, which short-circuits
straight to `DriveCommand(0.0, 0.0)`.

Sign convention (claude-docs/06-vehicle-params.md, REP-103): steering angle radians, LEFT
positive. `angular.z > 0` (turn left, CCW) with `linear.x > 0` (forward) yields
`steering_angle_rad > 0` (left) -- verified directly in test/test_twist_teleop.py.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class TwistTeleopConfig:
    wheelbase_m: float
    steering_min_rad: float
    steering_max_rad: float
    speed_min_mps: float
    speed_max_mps: float


@dataclass(frozen=True)
class DriveCommand:
    steering_angle_rad: float = 0.0
    speed_mps: float = 0.0


def build_twist_teleop_config(vehicle_params, allow_reverse: bool = False) -> TwistTeleopConfig:
    """Build a TwistTeleopConfig from the generated vehicle_params binding (CLAUDE.md
    invariant 2: never hand-write a physical constant) -- the SAME fields
    `racer_tools.keymap.build_teleop_config` reads for keyboard_teleop_node's own clamps, plus
    `chassis.wheelbase_m` for the bicycle-model conversion.

    `allow_reverse` (default False) has the same meaning and the same default as
    `racer_tools.keymap.build_teleop_config`'s: with it False the lower speed clamp is 0.0 m/s
    rather than `limits.min_velocity_mps`, so this adapter never emits a below-neutral throttle
    command. The two teleop sources are deliberately identical here -- this is the one a car
    launch file can actually start (`browser_teleop` in car_teleop.launch.py), so leaving
    reverse enabled on it while disabling it on the keyboard would disable nothing.
    """
    return TwistTeleopConfig(
        wheelbase_m=vehicle_params.chassis.wheelbase_m,
        steering_min_rad=vehicle_params.steering.min_angle_rad,
        steering_max_rad=vehicle_params.steering.max_angle_rad,
        speed_min_mps=vehicle_params.limits.min_velocity_mps if allow_reverse else 0.0,
        speed_max_mps=vehicle_params.limits.global_speed_cap_mps,
    )


def _clamp(value: float, lo: float, hi: float) -> float:
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def convert_twist_to_command(
    config: TwistTeleopConfig, linear_x: float, angular_z: float
) -> DriveCommand:
    """Convert one Twist sample to a clamped Ackermann DriveCommand. See this module's
    docstring for the bicycle-model derivation and the `speed == 0.0` special case.

    Non-finite (NaN/Inf) inputs are treated as garbage, the same "fail closed" stance
    `racer_safety`'s gate logic takes for a non-finite `/drive_raw` command
    (claude-docs/05-safety.md) -- rather than let a NaN propagate through `atan` (which
    returns NaN for a NaN input) into a published command, this returns a hard zero.
    """
    if not (math.isfinite(linear_x) and math.isfinite(angular_z)):
        return DriveCommand(steering_angle_rad=0.0, speed_mps=0.0)

    speed = _clamp(linear_x, config.speed_min_mps, config.speed_max_mps)
    if speed == 0.0:
        return DriveCommand(steering_angle_rad=0.0, speed_mps=0.0)

    steering = _clamp(
        math.atan(config.wheelbase_m * angular_z / speed),
        config.steering_min_rad,
        config.steering_max_rad,
    )
    return DriveCommand(steering_angle_rad=steering, speed_mps=speed)


def should_use_zero_command(elapsed_since_last_twist_s: float | None, timeout_s: float) -> bool:
    """True if the adapter should command zero instead of the last-converted Twist.

    `elapsed_since_last_twist_s` is `None` when no Twist has ever been received (from node
    startup) -- also zero, per "zero Twist means zero command" (there is nothing to be stale
    FROM yet, so there is no reason to wait out a timeout before commanding the only safe
    thing there is to command). Otherwise true iff the elapsed time exceeds `timeout_s` --
    see twist_teleop_adapter_node.py's module docstring for why this is "publish zero and
    keep publishing zero" rather than "stop publishing".

    Garbage elapsed values -- negative, NaN, or infinite -- are treated as STALE, not fresh,
    the same fail-closed stance racer_safety's gate logic takes for a garbage
    `drive_raw_age_s` (see gate_logic.cpp's watchdog step). A negative elapsed time means
    the clock the caller measured with went backwards; with a plain `> timeout_s` test that
    would read as "fresh", so a browser tab that closed during a backwards clock step would
    leave this node happily republishing the driver's LAST non-zero command until the clock
    caught up. `twist_teleop_adapter_node` now measures on `time.monotonic()`, which cannot
    produce a negative elapsed time -- this branch is defence in depth against a future
    caller that measures on a steppable clock, and it is what makes "no valid elapsed
    measurement" mean "stop the car".
    """
    if elapsed_since_last_twist_s is None:
        return True
    if not math.isfinite(elapsed_since_last_twist_s):
        return True
    if elapsed_since_last_twist_s < 0.0:
        return True
    return elapsed_since_last_twist_s > timeout_s
