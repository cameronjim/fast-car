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
    delay_curve: dict[float, float] = field(default_factory=dict)  # delay -> speed RMSE
    delay_selection: str = ""
    yaw_training: str = ""
    sides_tied: bool = False
    alternative: Nominal | None = None  # the fit with the other tying choice, for the record
    yaw_cv_rmse_rad: dict[str, float] = field(default_factory=dict)
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


MIN_VAL_WINDOWS = 150  # fewer validation windows than this and the delay is cross-validated
CV_FOLDS = 4


def _fit_speed_at(
    windows: SpeedWindows, vehicle: Vehicle, start: Nominal, delay_s: float
) -> tuple[Nominal, dict[str, str]]:
    # Starts for the rate limits: a clip that is not active has no gradient, so start with tight,
    # the placeholder and loose limits and keep the best.
    starts = [
        start.replace(accel_max_mps2=a, decel_max_mps2=b)
        for a, b in ((0.5, 0.5), (start.accel_max_mps2, start.decel_max_mps2), (10.0, 10.0))
    ]
    flags: dict[str, str] = {}
    p = _fit_params(
        SPEED_PARAMS,
        start.replace(command_delay_s=delay_s),
        _speed_residual(windows, vehicle),
        flags,
        [s.replace(command_delay_s=delay_s) for s in starts],
    )
    return p, flags


def fit_speed(
    train: Sequence[Segment],
    val: Sequence[Segment],
    vehicle: Vehicle,
    start: Nominal = PLACEHOLDER_NOMINAL,
    delays: Sequence[float] = DELAY_GRID_S,
) -> tuple[Nominal, dict[float, float], dict[str, str], str]:
    """Speed parameters and the command delay, the best over the delay grid on validation.

    When the validation split holds fewer than MIN_VAL_WINDOWS speed windows (the 2026-11-11 bag
    has no driving in its validation tenth), the delay is chosen by blocked cross-validation over
    the training segments instead (segments dealt into CV_FOLDS groups). The fourth return value
    says which was used."""
    odom_train = [s for s in train if s.run.has_odom]
    use_val = len(speed_windows(val, delays[0])) >= MIN_VAL_WINDOWS
    k = min(CV_FOLDS, len(odom_train))
    folds = [odom_train[i::k] for i in range(k)] if not use_val and k >= 2 else []
    curve: dict[float, float] = {}
    for d in delays:
        if use_val:
            p, _ = _fit_speed_at(speed_windows(train, d), vehicle, start, d)
            curve[d] = speed_rmse(speed_windows(val, d), p, vehicle)
        elif folds:
            errors = []
            for j, held in enumerate(folds):
                rest = [s for i, f in enumerate(folds) if i != j for s in f]
                p, _ = _fit_speed_at(speed_windows(rest, d), vehicle, start, d)
                errors.append(speed_rmse(speed_windows(held, d), p, vehicle))
            curve[d] = float(np.mean(errors))
        else:  # nothing to validate on: the training error is all there is
            p, _ = _fit_speed_at(speed_windows(train, d), vehicle, start, d)
            curve[d] = speed_rmse(speed_windows(train, d), p, vehicle)
    best_delay = min(delays, key=lambda d: np.inf if np.isnan(curve[d]) else curve[d])
    p, flags = _fit_speed_at(speed_windows(train, best_delay), vehicle, start, best_delay)
    how = (
        "validation split"
        if use_val
        else f"{k}-fold cross-validation over training segments"
        if folds
        else "training error (no validation data)"
    )
    return p, curve, flags, how


MIN_SIDE_INTERVALS = 300  # fewer turning intervals than this on a side and the gains are tied
TIED_YAW_PARAMS = ("steer_gain_left", "steer_tau_s", "understeer_s2_per_m2")


def tie_sides(p: Nominal) -> Nominal:
    """One steering gain for both sides and no offset: what one sided data can support."""
    return p.replace(steer_gain_right=p.steer_gain_left, steer_offset_rad=0.0)


def fit_yaw(
    train: Sequence[Segment],
    vehicle: Vehicle,
    nominal: Nominal,
    wheel_by_run: dict[str, np.ndarray] | None = None,
    tied: bool = False,
) -> tuple[Nominal, dict[str, str]]:
    """Yaw channel. With `tied` the two gains are one parameter and the offset is zero."""
    data = yaw_data(train, nominal.command_delay_s, wheel_by_run)
    flags: dict[str, str] = {}

    def fn(p: Nominal) -> np.ndarray:
        p = tie_sides(p) if tied else p
        return predict_yaw_change(p, vehicle, data) - data.dyaw_rad

    if not tied:
        return _fit_params(YAW_PARAMS, nominal, fn, flags), flags
    out = _fit_params(TIED_YAW_PARAMS, tie_sides(nominal), fn, flags)
    return tie_sides(out), flags


def side_counts(data) -> tuple[int, int]:
    """(left, right) turning intervals: steering command of at least 0.02 rad at the interval."""
    steer = data.cmds[:, -1, 1]
    return int((steer >= 0.02).sum()), int((steer <= -0.02).sum())


def wheel_speeds_for(
    segments: Sequence[Segment], nominal: Nominal, vehicle: Vehicle
) -> dict[str, np.ndarray]:
    """Run name -> wheel speed on the grid for every run with no odometry (from the speed model)."""
    out: dict[str, np.ndarray] = {}
    for seg in segments:
        if not seg.run.has_odom and seg.run.name not in out:
            out[seg.run.name] = reconstruct_wheel_speed(seg.run, nominal, vehicle)
    return out


def choose_yaw_training(
    train: Sequence[Segment],
    vehicle: Vehicle,
    nominal: Nominal,
    wheel: dict[str, np.ndarray],
) -> tuple[str, dict[str, float]]:
    """Decide whether bags with no odometry may train the yaw channel, on training data only.

    The odometry bags are the car as it is now (the held-out data is that car too); the older
    bags are a different day and surface with a reconstructed wheel speed. Blocked cross
    validation over the odometry training segments compares a yaw fit on the odometry bags alone
    with one that also uses the older bags, and the lower yaw error on the held-back odometry
    segments wins. Returns ("all" or "odom_only", mean CV yaw RMSE per option)."""
    odom = [s for s in train if s.run.has_odom]
    other = [s for s in train if not s.run.has_odom]
    k = min(CV_FOLDS, len(odom))
    if not other or k < 2:
        return "odom_only" if not other else "all", {}
    folds = [odom[i::k] for i in range(k)]
    errors: dict[str, list[float]] = {"odom_only": [], "all": []}
    for j, held in enumerate(folds):
        rest = [s for i, f in enumerate(folds) if i != j for s in f]
        data = yaw_data(held, nominal.command_delay_s, wheel)
        if len(data) == 0:
            continue
        for mode, extra in (("odom_only", []), ("all", other)):
            p, _ = fit_yaw([*rest, *extra], vehicle, nominal, wheel)
            errors[mode].append(yaw_rmse(data, p, vehicle))
    means = {m: float(np.mean(e)) if e else float("inf") for m, e in errors.items()}
    return min(means, key=lambda m: means[m]), means


def fit_nominal(
    segments: Sequence[Segment],
    vehicle: Vehicle,
    start: Nominal = PLACEHOLDER_NOMINAL,
    delays: Sequence[float] = DELAY_GRID_S,
    yaw_training: str = "auto",
) -> FitResult:
    """The whole fit: speed channel and delay (odometry bags), then the yaw channel. `yaw_training`
    is "all", "odom_only" or "auto" (choose_yaw_training)."""
    train, val = by_split(segments, "train"), by_split(segments, "val")
    speed_nominal, curve, flags, how = fit_speed(train, val, vehicle, start, delays)
    wheel = wheel_speeds_for([*train, *val], speed_nominal, vehicle)
    cv: dict[str, float] = {}
    mode = yaw_training
    if mode == "auto":
        mode, cv = choose_yaw_training(train, vehicle, speed_nominal, wheel)
    yaw_train = train if mode == "all" else [s for s in train if s.run.has_odom]
    left_n, right_n = side_counts(yaw_data(yaw_train, speed_nominal.command_delay_s, wheel))
    tied = min(left_n, right_n) < MIN_SIDE_INTERVALS
    nominal, yaw_flags = fit_yaw(yaw_train, vehicle, speed_nominal, wheel, tied=tied)
    alternative, _ = fit_yaw(yaw_train, vehicle, speed_nominal, wheel, tied=not tied)
    flags.update(yaw_flags)
    if tied:
        flags["steer_gain_right"] = (
            f"tied to steer_gain_left: only {min(left_n, right_n)} training intervals on one "
            f"side (rule: fewer than {MIN_SIDE_INTERVALS})"
        )
        flags["steer_offset_rad"] = "fixed at 0 (one sided training data cannot separate it)"
    delay = nominal.command_delay_s
    if delay in (delays[0], delays[-1]):
        flags["command_delay_s"] = "on the edge of the delay grid"
    w_val = speed_windows(val, delay)
    y_val = yaw_data(val, delay, wheel)
    y_train = yaw_data(yaw_train, delay, wheel)
    return FitResult(
        nominal=nominal,
        flags=flags,
        speed_val_rmse_mps=speed_rmse(w_val, nominal, vehicle),
        yaw_val_rmse_rad=yaw_rmse(y_val, nominal, vehicle),
        delay_curve=curve,
        delay_selection=how,
        yaw_training=mode,
        sides_tied=tied,
        alternative=alternative,
        yaw_cv_rmse_rad=cv,
        counts={
            "speed_windows_train": len(speed_windows(train, delay)),
            "speed_windows_val": len(w_val),
            "yaw_intervals_train": len(y_train),
            "yaw_intervals_val": len(y_val),
            "yaw_left_train": side_counts(y_train)[0],
            "yaw_right_train": side_counts(y_train)[1],
        },
    )
