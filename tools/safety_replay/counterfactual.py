"""What a recorded brake prevented: the unbraked plan swept through the scan the gate judged.

Pure numpy, no bag and no ROS. tools/audit_brakes.py feeds it the recorded scene, request and
measured dynamics; the unit tests feed it synthetic walls. Frames: rear axle at the origin, x
forward, y left (REP-103), the same as tools/safety_replay/motion.py.

Two questions, both on ONE scan (the scene is static over the 1 s horizon, which is how the gate
itself sees it, and the foam lane does not move):

* `swept_distance_m`: how far the rear axle can travel along a steering arc before the body
  rectangle (plus a margin) touches a return. An independent, geometric re-statement of what
  racer_safety's body distance measures; it is NOT the gate's code, so a disagreement between the two
  shows up in the audit instead of being hidden by sharing the implementation.
* `simulate`: drive the car along a command stream (steering and speed against time) through a
  first-order speed lag with a transport delay and a steering servo lag, and report the smallest
  distance from the body rectangle to any return over the horizon. That distance at or below
  zero is a hit.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class BodyBox:
    """The footprint about the rear axle, no margin."""

    front_x_m: float  # wheelbase + front overhang
    rear_x_m: float  # rear overhang, positive
    half_width_m: float


@dataclass(frozen=True)
class Plant:
    """The measured response the unbraked plan is run through (all SI)."""

    wheelbase_m: float
    speed_delay_s: float  # command to wheel speed: pure transport delay
    speed_tau_s: float  # then a first-order lag
    steer_tau_s: float  # steering servo lag (0 = instant)
    max_decel_mps2: float  # the lag cannot slow the car faster than this
    max_accel_mps2: float
    # Effective curvature as a share of the kinematic tan(delta) / L (tyre slip, understeer at
    # speed), a function of the steering angle; 1.0 is the bicycle model.
    curvature_gain: Callable[[float], float] = lambda delta: 1.0
    # True speed as a share of the wheel speed (the wheels slip at full lock), same argument.
    speed_gain: Callable[[float], float] = lambda delta: 1.0


def body_clearance_m(points: np.ndarray, x: float, y: float, theta: float, body: BodyBox) -> float:
    """Smallest distance from any point to the body rectangle posed at (x, y, theta); 0 on or
    inside it; +inf with no points."""
    if len(points) == 0:
        return math.inf
    c, s = math.cos(theta), math.sin(theta)
    dx = points[:, 0] - x
    dy = points[:, 1] - y
    bx = c * dx + s * dy  # into the body frame
    by = -s * dx + c * dy
    ex = np.maximum(np.maximum(-body.rear_x_m - bx, bx - body.front_x_m), 0.0)
    ey = np.maximum(np.abs(by) - body.half_width_m, 0.0)
    return float(np.hypot(ex, ey).min())


def _pose_on_arc(s_m: float, curvature: float) -> tuple[float, float, float]:
    if abs(curvature) < 1e-9:
        return s_m, 0.0, 0.0
    theta = s_m * curvature
    return math.sin(theta) / curvature, (1.0 - math.cos(theta)) / curvature, theta


def swept_distance_m(
    points: np.ndarray,
    steering_rad: float,
    wheelbase_m: float,
    body: BodyBox,
    horizon_m: float,
    margin_m: float = 0.0,
    step_m: float = 0.01,
) -> float:
    """Arc length the rear axle travels along `steering_rad` until the body (inflated by margin_m)
    reaches a return; +inf when nothing is reached within horizon_m. 0 when already touching."""
    curvature = math.tan(steering_rad) / wheelbase_m
    n = int(horizon_m / step_m)
    for k in range(n + 1):
        s = k * step_m
        x, y, th = _pose_on_arc(s, curvature)
        if body_clearance_m(points, x, y, th, body) <= margin_m:
            return s
    return math.inf


@dataclass(frozen=True)
class Trajectory:
    t_s: np.ndarray
    x_m: np.ndarray
    y_m: np.ndarray
    theta_rad: np.ndarray
    speed_mps: np.ndarray
    min_clearance_m: float  # smallest body-to-return distance over the horizon
    first_hit_s: float | None  # first time the clearance reached <= 0
    travel_m: float


def simulate(
    points: np.ndarray,
    speed_command: Callable[[float], float],
    steering_command: Callable[[float], float],
    v0_mps: float,
    delta0_rad: float,
    plant: Plant,
    body: BodyBox,
    horizon_s: float,
    dt_s: float = 0.01,
) -> Trajectory:
    """Run the car along commands given as functions of time since the start (t = 0 is the
    instant of the scan, the car at the origin, speed v0 along x and steering delta0).

    Speed: the command passes a transport delay speed_delay_s, then the wheel speed follows it with
    time constant speed_tau_s, bounded by max_accel / max_decel. The true speed is speed_gain(delta)
    times the wheel speed. Steering: first order, steer_tau_s. Pose: the bicycle model with the effective
    curvature gain. Clearance is the body rectangle's distance to the scan's returns at every step.
    """
    n = round(horizon_s / dt_s)
    t = np.arange(n + 1) * dt_s
    xs = np.zeros(n + 1)
    ys = np.zeros(n + 1)
    th = np.zeros(n + 1)
    vs = np.zeros(n + 1)
    g0 = plant.speed_gain(delta0_rad)
    wheel = v0_mps / g0 if g0 > 0.0 else v0_mps
    delta = delta0_rad
    vs[0] = v0_mps
    best = body_clearance_m(points, 0.0, 0.0, 0.0, body)
    hit: float | None = 0.0 if best <= 0.0 else None
    travel = 0.0
    for i in range(1, n + 1):
        t_cmd = t[i - 1] - plant.speed_delay_s
        u = speed_command(t_cmd)
        rate = (
            (u - wheel) / plant.speed_tau_s
            if plant.speed_tau_s > 0.0
            else math.copysign(math.inf, u - wheel)
        )
        rate = max(-plant.max_decel_mps2, min(plant.max_accel_mps2, rate))
        wheel += rate * dt_s
        target = steering_command(t[i - 1])
        if plant.steer_tau_s > 0.0:
            delta += (target - delta) * (1.0 - math.exp(-dt_s / plant.steer_tau_s))
        else:
            delta = target
        v = wheel * plant.speed_gain(delta)
        curvature = plant.curvature_gain(delta) * math.tan(delta) / plant.wheelbase_m
        mid = th[i - 1] + 0.5 * v * dt_s * curvature
        xs[i] = xs[i - 1] + v * dt_s * math.cos(mid)
        ys[i] = ys[i - 1] + v * dt_s * math.sin(mid)
        th[i] = th[i - 1] + v * dt_s * curvature
        vs[i] = v
        travel += abs(v) * dt_s
        c = body_clearance_m(points, xs[i], ys[i], th[i], body)
        best = min(best, c)
        if hit is None and c <= 0.0:
            hit = float(t[i])
    return Trajectory(t, xs, ys, th, vs, best, hit, travel)
