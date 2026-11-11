"""Command shaping for the spiderbot: clamps, rate limits, the facing creep. Pure, no ROS.

Every number that is a property of the car arrives as an argument, built by the node from
config/vehicle_params.yaml through the generated binding (CLAUDE.md invariant 2). Nothing here
knows a wheelbase, a steering limit or an acceleration.

RATE LIMITS. racer_safety's gate rate-limits speed and steering against vehicle_params, and an
un-ramped command (patrol resuming from a stop, a stop from patrol speed) would trip it on
every transition. `RateLimiter` ramps the published command at a MARGIN below those bounds, the
same approach as racer_control's tracker, so the gate stays a bystander in normal operation.
It ramps both ways: a stop is a ramp to zero, a fraction of a second at patrol speeds.
Non-finite targets are treated as "stop, wheels straight", never passed on.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class MotionLimits:
    max_steering_rad: float  # magnitude, symmetric (the smaller of the left and right limits)
    max_speed_mps: float  # magnitude of the forward speed this node may ever publish
    max_speed_step_mps: float  # per control tick
    max_steering_step_rad: float  # per control tick

    def __post_init__(self) -> None:
        for name in ("max_steering_rad", "max_speed_mps", "max_speed_step_mps",
                     "max_steering_step_rad"):  # fmt: skip
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"{name} must be positive and finite, got {value}")


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def limits_from_vehicle(
    max_angle_rad: float,
    min_angle_rad: float,
    max_rate_rad_per_s: float,
    max_acceleration_mps2: float,
    full_scale_mps: float,
    control_rate_hz: float,
    margin_fraction: float = 0.5,
) -> MotionLimits:
    """Build MotionLimits from vehicle_params values.

    Steering is limited to the smaller of the two sides (the command is symmetric); the speed
    to the throttle full scale; the per-tick steps are `margin_fraction` of what the safety
    gate allows per tick.
    """
    if not 0.0 < margin_fraction <= 1.0:
        raise ValueError(f"margin_fraction must be in (0, 1], got {margin_fraction}")
    if not (math.isfinite(control_rate_hz) and control_rate_hz > 0.0):
        raise ValueError(f"control_rate_hz must be positive, got {control_rate_hz}")
    dt = 1.0 / control_rate_hz
    return MotionLimits(
        max_steering_rad=min(abs(max_angle_rad), abs(min_angle_rad)),
        max_speed_mps=full_scale_mps,
        max_speed_step_mps=margin_fraction * max_acceleration_mps2 * dt,
        max_steering_step_rad=margin_fraction * max_rate_rad_per_s * dt,
    )


class RateLimiter:
    """Moves the published (speed, steering) towards a target by at most one step per tick."""

    def __init__(self, limits: MotionLimits) -> None:
        self.limits = limits
        self.speed_mps = 0.0
        self.steering_rad = 0.0

    def reset(self, speed_mps: float = 0.0, steering_rad: float = 0.0) -> None:
        self.speed_mps = speed_mps
        self.steering_rad = steering_rad

    def step(self, target_speed_mps: float, target_steering_rad: float) -> tuple[float, float]:
        lim = self.limits
        if not (math.isfinite(target_speed_mps) and math.isfinite(target_steering_rad)):
            target_speed_mps, target_steering_rad = 0.0, 0.0
        speed = clamp(target_speed_mps, -lim.max_speed_mps, lim.max_speed_mps)
        steer = clamp(target_steering_rad, -lim.max_steering_rad, lim.max_steering_rad)
        self.speed_mps += clamp(
            speed - self.speed_mps, -lim.max_speed_step_mps, lim.max_speed_step_mps
        )
        self.steering_rad += clamp(
            steer - self.steering_rad, -lim.max_steering_step_rad, lim.max_steering_step_rad
        )
        return self.speed_mps, self.steering_rad


def face_command(
    bearing_rad: float, gain: float, creep_speed_mps: float, limits: MotionLimits
) -> tuple[float, float]:
    """(speed, steering) that creeps forward and turns towards `bearing_rad`.

    Bearing is vehicle frame, left positive, and steering is left positive, so a person on the
    left (positive bearing) gets a left (positive) steer: moving FORWARD with a left steer turns
    the nose left, towards them. Proportional with saturation at the steering limit; speed is
    the creep speed capped at the speed limit. A non-finite bearing is no command at all (0, 0).
    """
    if not math.isfinite(bearing_rad):
        return 0.0, 0.0
    steer = clamp(gain * bearing_rad, -limits.max_steering_rad, limits.max_steering_rad)
    return clamp(creep_speed_mps, 0.0, limits.max_speed_mps), steer
