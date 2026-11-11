"""Fit of the nominal parameters (io.nominal of the contract).

Speed channel: from bags with /odom/wheel, simulate the speed part of the model over 1 s windows
from the recorded (delayed) commands and minimise the multi-step wheel speed error. The command
delay is chosen on a grid (0.00 to 0.30 s, steps of 0.02 s) by the validation error of that fit.
Yaw channel: from the ICP yaw change per 0.1 s scan interval, integrate the model at 0.05 s
over the interval from the recorded commands and the wheel speed and minimise the yaw change
error. Bags with no odometry get their wheel speed from the fitted speed model.

Every parameter has physical bounds. A parameter within 0.5 percent of a bound, or one the data
cannot move (a zero sensitivity), is flagged in the result.
"""

from __future__ import annotations

import logging
from collections.abc import Sequence
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares

from .dataset import Segment, by_split
from .model import PLACEHOLDER_NOMINAL, Nominal
from .vehicle import Vehicle
from .windows import (
    SpeedWindows,
    predict_yaw_change,
    reconstruct_wheel_speed,
    speed_rollout,
    speed_windows,
    yaw_data,
)

LOG = logging.getLogger(__name__)

DELAY_GRID_S = tuple(round(0.02 * k, 2) for k in range(16))  # 0.00 .. 0.30
BOUNDS: dict[str, tuple[float, float]] = {
    "speed_gain": (0.5, 1.5),
    "speed_tau_s": (0.02, 1.0),
    "accel_max_mps2": (0.2, 20.0),
    "decel_max_mps2": (0.2, 20.0),
    "steer_gain_left": (0.5, 1.5),
    "steer_gain_right": (0.5, 1.5),
    "steer_offset_rad": (-0.1, 0.1),
    "steer_tau_s": (0.02, 1.0),
    "understeer_s2_per_m2": (0.0, 2.0),
}
SPEED_PARAMS = ("speed_gain", "speed_tau_s", "accel_max_mps2", "decel_max_mps2")
YAW_PARAMS = (
    "steer_gain_left",
    "steer_gain_right",
    "steer_offset_rad",
    "steer_tau_s",
    "understeer_s2_per_m2",
)
BOUND_TOLERANCE = 0.005  # of the bound range


@dataclass
class FitResult:
    nominal: Nominal
    flags: dict[str, str] = field(default_factory=dict)
    speed_val_rmse_mps: float = float("nan")  # all steps of the 1 s windows
    yaw_val_rmse_rad: float = float("nan")  # per 0.1 s interval
    delay_curve: dict[float, float] = field(default_factory=dict)  # delay -> speed val RMSE
    counts: dict[str, int] = field(default_factory=dict)


def _vector(p: Nominal, names: Sequence[str]) -> np.ndarray:
    return np.array([getattr(p, n) for n in names])


def _fit_params(
    names: Sequence[str],
    start: Nominal,
    residual_fn,
    flags: dict[str, str] | None = None,
    starts: Sequence[Nominal] | None = None,
) -> Nominal:
    """least_squares over `names` of a Nominal, from each start, keeping the best."""
    lo = np.array([BOUNDS[n][0] for n in names])
    hi = np.array([BOUNDS[n][1] for n in names])
    best = None
    for s in starts or [start]:
        x0 = np.clip(_vector(s, names), lo, hi)

        def fun(x: np.ndarray) -> np.ndarray:
            return residual_fn(start.replace(**dict(zip(names, x, strict=True))))

        sol = least_squares(fun, x0, bounds=(lo, hi), x_scale=hi - lo, diff_step=1e-4)
        if best is None or sol.cost < best.cost:
            best = sol
    assert best is not None
    x = best.x.copy()
    for i, n in enumerate(names):
        if np.linalg.norm(best.jac[:, i]) <= 1e-9:  # the data cannot move it: keep the start
            x[i] = np.clip(getattr(start, n), lo[i], hi[i])
            if flags is not None:
                flags[n] = "not identified by the data (zero sensitivity), kept at its start"
        elif flags is not None and min(x[i] - lo[i], hi[i] - x[i]) <= BOUND_TOLERANCE * (
            hi[i] - lo[i]
        ):
            flags[n] = "on a bound"
    out = start.replace(**dict(zip(names, x, strict=True)))
    return out


def _speed_residual(windows: SpeedWindows, vehicle: Vehicle):
    def fn(p: Nominal) -> np.ndarray:
        return (speed_rollout(windows, p, vehicle)[:, 1:] - windows.v_mps[:, 1:]).ravel()

    return fn


def speed_rmse(windows: SpeedWindows, p: Nominal, vehicle: Vehicle) -> float:
    if len(windows) == 0:
        return float("nan")
    return float(np.sqrt(np.mean(_speed_residual(windows, vehicle)(p) ** 2)))


def yaw_rmse(data, p: Nominal, vehicle: Vehicle) -> float:
    if len(data) == 0:
        return float("nan")
    return float(np.sqrt(np.mean((predict_yaw_change(p, vehicle, data) - data.dyaw_rad) ** 2)))


def fit_speed(
    train: Sequence[Segment],
    val: Sequence[Segment],
    vehicle: Vehicle,
    start: Nominal = PLACEHOLDER_NOMINAL,
    delays: Sequence[float] = DELAY_GRID_S,
) -> tuple[Nominal, dict[float, float], dict[str, str]]:
    """Speed parameters and the command delay (best validation error over the delay grid)."""
    curve: dict[float, float] = {}
    best: tuple[float, Nominal, dict[str, str]] | None = None
    # Two starts for the limits: the placeholder, and limits far above anything demanded.
    starts = [start, start.replace(accel_max_mps2=10.0, decel_max_mps2=10.0)]
    for d in delays:
        w_train = speed_windows(train, d)
        w_val = speed_windows(val, d)
        flags: dict[str, str] = {}
        p = _fit_params(
            SPEED_PARAMS,
            start.replace(command_delay_s=d),
            _speed_residual(w_train, vehicle),
            flags,
            [s.replace(command_delay_s=d) for s in starts],
        )
        curve[d] = speed_rmse(w_val, p, vehicle)
        if best is None or curve[d] < best[0]:
            best = (curve[d], p, flags)
    assert best is not None
    return best[1], curve, best[2]


def fit_yaw(
    train: Sequence[Segment],
    vehicle: Vehicle,
    nominal: Nominal,
    wheel_by_run: dict[str, np.ndarray] | None = None,
) -> tuple[Nominal, dict[str, str]]:
    data = yaw_data(train, nominal.command_delay_s, wheel_by_run)
    flags: dict[str, str] = {}

    def fn(p: Nominal) -> np.ndarray:
        return predict_yaw_change(p, vehicle, data) - data.dyaw_rad

    return _fit_params(YAW_PARAMS, nominal, fn, flags), flags


def wheel_speeds_for(
    segments: Sequence[Segment], nominal: Nominal, vehicle: Vehicle
) -> dict[str, np.ndarray]:
    """Run name -> wheel speed on the grid for every run with no odometry (from the speed model)."""
    out: dict[str, np.ndarray] = {}
    for seg in segments:
        if not seg.run.has_odom and seg.run.name not in out:
            out[seg.run.name] = reconstruct_wheel_speed(seg.run, nominal, vehicle)
    return out


def fit_nominal(
    segments: Sequence[Segment],
    vehicle: Vehicle,
    start: Nominal = PLACEHOLDER_NOMINAL,
    delays: Sequence[float] = DELAY_GRID_S,
) -> FitResult:
    """The whole fit: speed channel and delay (odometry bags), then the yaw channel (all bags)."""
    train, val = by_split(segments, "train"), by_split(segments, "val")
    speed_nominal, curve, flags = fit_speed(train, val, vehicle, start, delays)
    wheel = wheel_speeds_for([*train, *val], speed_nominal, vehicle)
    nominal, yaw_flags = fit_yaw(train, vehicle, speed_nominal, wheel)
    flags.update(yaw_flags)
    delay = nominal.command_delay_s
    w_val = speed_windows(val, delay)
    y_val = yaw_data(val, delay, wheel)
    y_train = yaw_data(train, delay, wheel)
    return FitResult(
        nominal=nominal,
        flags=flags,
        speed_val_rmse_mps=speed_rmse(w_val, nominal, vehicle),
        yaw_val_rmse_rad=yaw_rmse(y_val, nominal, vehicle),
        delay_curve=curve,
        counts={
            "speed_windows_train": len(speed_windows(train, delay)),
            "speed_windows_val": len(w_val),
            "yaw_intervals_train": len(y_train),
            "yaw_intervals_val": len(y_val),
            "yaw_left_train": int((y_train.cmds[:, -1, 1] >= 0.02).sum()),
            "yaw_right_train": int((y_train.cmds[:, -1, 1] <= -0.02).sum()),
        },
    )
