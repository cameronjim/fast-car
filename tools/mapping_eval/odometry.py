"""Wheel odometry from a bag's /odom/wheel speed and /drive steering, in numpy.

The same kinematics as racer_state's wheel_twist_node (servo rate limit, then
``wz = v tan(delta) / L``), so the offline replays and the yaw analysis see what the EKF's twist
input saw, plus the options needed to put a bag recorded before the 2026-11-10 calibration on
the calibrated car's scale and to try a corrected steering and ground-speed model.

Everything physical comes from config/vehicle_params.yaml (CLAUDE.md invariant 2); the options
here are corrections to a MEASURED bag, not constants of the car.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

VEHICLE_PARAMS = Path(__file__).resolve().parents[2] / "config" / "vehicle_params.yaml"


@dataclass(frozen=True)
class Vehicle:
    wheelbase_m: float
    min_angle_rad: float
    max_angle_rad: float
    max_rate_rad_per_s: float
    mount_xy_yaw: tuple[float, float, float]
    yaw_gain_table: tuple[tuple[float, float], ...] = ((0.0, 1.0),)


def load_vehicle(path: Path = VEHICLE_PARAMS) -> Vehicle:
    vp = yaml.safe_load(path.read_text())
    lidar = vp["sensors"]["lidar"]
    steering = vp["steering"]
    return Vehicle(
        wheelbase_m=float(vp["chassis"]["wheelbase_m"]),
        min_angle_rad=float(steering["min_angle_rad"]),
        max_angle_rad=float(steering["max_angle_rad"]),
        max_rate_rad_per_s=float(steering["max_rate_rad_per_s"]),
        yaw_gain_table=tuple(
            (float(k["abs_angle_rad"]), float(k["gain"])) for k in steering["yaw_rate_gain_table"]
        ),
        mount_xy_yaw=(
            float(lidar["mount_x_m"]),
            float(lidar["mount_y_m"]),
            float(lidar["mount_yaw_rad"]),
        ),
    )


@dataclass(frozen=True)
class BagScale:
    """Corrections that put a bag on the calibrated car's scale.

    speed_divisor: /odom/wheel speeds are divided by it (1.401 for the 2026-11-10 bags, recorded
    before the tyre radius and gear ratio were calibrated; 1.0 for a bag from the calibrated car).
    steer_scale_left / right: true wheel angle per command radian (1.203 / 1.072 for the
    2026-11-10 bags; 1.0 for the calibrated car, whose commands are true angles).
    """

    speed_divisor: float = 1.0
    steer_scale_left: float = 1.0
    steer_scale_right: float = 1.0


OLD_BAGS = BagScale(speed_divisor=1.401, steer_scale_left=1.203, steer_scale_right=1.072)


@dataclass(frozen=True)
class YawModel:
    """A model of how the true yaw rate and ground speed follow wheel speed and steering.

    steer_gain: a constant gain on the modelled yaw rate v tan(delta) / L. None takes
    vehicle_params' steering.yaw_rate_gain_table, piecewise linear in |delta| (what
    wheel_twist_node does); 1.0 is the model before 2026-11-12.
    ground_ratio_at_zero / ground_ratio_slope_per_rad: ground speed over wheel speed is
    ``ground_ratio_at_zero - ground_ratio_slope_per_rad * |delta|`` (a driven wheel over-reads
    the ground speed in a turn as the tyres slip); 1 and 0 leave the wheel speed as it is.
    yaw_scale: a plain multiplier on the yaw rate, for what-if replays only.
    """

    steer_gain: float | None = None
    ground_ratio_at_zero: float = 1.0
    ground_ratio_slope_per_rad: float = 0.0
    yaw_scale: float = 1.0


def yaw_gain(table: tuple[tuple[float, float], ...], abs_delta: np.ndarray) -> np.ndarray:
    """vehicle_params steering.yaw_rate_gain_table: piecewise linear in |delta|, held at the first
    and last gain outside the knots (racer_state's yaw_gain)."""
    xs, ys = zip(*table, strict=True)
    return np.interp(abs_delta, xs, ys)


def servo_angle_series(
    t: np.ndarray, command: np.ndarray, vehicle: Vehicle, max_rate: float | None = None
) -> np.ndarray:
    """Steering angle after the servo model: clamp to the limits, then the rate limit (no lag:
    steering.time_constant_s is not measured). t, command sampled at the same instants."""
    rate = vehicle.max_rate_rad_per_s if max_rate is None else max_rate
    target = np.clip(command, vehicle.min_angle_rad, vehicle.max_angle_rad)
    out = np.empty_like(target)
    angle = 0.0
    last = t[0]
    for i in range(len(t)):
        dt = max(t[i] - last, 0.0)
        last = t[i]
        angle += float(np.clip(target[i] - angle, -rate * dt, rate * dt))
        out[i] = angle
    return out


def wheel_twist_series(
    odom_t: np.ndarray,
    odom_v: np.ndarray,
    drive_t: np.ndarray,
    drive_steer: np.ndarray,
    vehicle: Vehicle,
    scale: BagScale | None = None,
    model: YawModel | None = None,
    clamp_to_vehicle_limits: bool = True,
) -> dict[str, np.ndarray]:
    """Speed, steering and yaw rate at every /odom/wheel stamp.

    Steering command -> (bag scale: true angle per command radian, per side) -> servo model
    (limits and rate) -> kinematics. ``clamp_to_vehicle_limits`` False lets the old bags'
    uncalibrated steering exceed the planning limit (the left lock is 0.504 rad)."""
    scale = scale or BagScale()
    model = model or YawModel()
    v_wheel = odom_v / scale.speed_divisor
    command = np.interp(odom_t, drive_t, drive_steer)
    command = np.where(
        command >= 0.0, command * scale.steer_scale_left, command * scale.steer_scale_right
    )
    if clamp_to_vehicle_limits:
        delta = servo_angle_series(odom_t, command, vehicle)
    else:
        wide = Vehicle(
            vehicle.wheelbase_m,
            -1.0,
            1.0,
            vehicle.max_rate_rad_per_s,
            vehicle.mount_xy_yaw,
            vehicle.yaw_gain_table,
        )
        delta = servo_angle_series(odom_t, command, wide)
    ratio = model.ground_ratio_at_zero - model.ground_ratio_slope_per_rad * np.abs(delta)
    v_ground = v_wheel * ratio
    gain = (
        yaw_gain(vehicle.yaw_gain_table, np.abs(delta))
        if model.steer_gain is None
        else model.steer_gain
    )
    wz = model.yaw_scale * gain * v_ground * np.tan(delta) / vehicle.wheelbase_m
    return {"t": odom_t, "v_wheel": v_wheel, "v": v_ground, "delta": delta, "wz": wz}


def integrate_pose(
    t: np.ndarray, v: np.ndarray, wz: np.ndarray, max_dt: float = 0.1
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Dead-reckoned (x, y, yaw) in the rear axle frame from forward speed and yaw rate
    (midpoint heading per step), starting at the origin."""
    dt = np.clip(np.diff(t, prepend=t[0]), 0.0, max_dt)
    yaw = np.cumsum(wz * dt)
    mid = yaw - 0.5 * wz * dt
    x = np.cumsum(v * np.cos(mid) * dt)
    y = np.cumsum(v * np.sin(mid) * dt)
    return x, y, yaw


def wrap(angle: np.ndarray | float) -> np.ndarray | float:
    return (np.asarray(angle) + math.pi) % (2.0 * math.pi) - math.pi
