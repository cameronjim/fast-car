"""Fit of the nominal parameters (io.nominal of the contract).

Speed channel: from bags with /odom/wheel, simulate the speed part of the model over 1 s windows
from the recorded (delayed) commands and minimise the multi-step wheel speed error. The command
delay (0.00 to 0.30 s, steps of 0.02 s) is chosen on a grid, each cell with the continuous speed
parameters refit on the training windows, by the error on the HELD-OUT windows (the last 30
percent of each bag's moving data), falling back to blocked cross-validation over the training
segments when there are too few. The speed dead zone is the median launch command of the data
(`launch_deadband`, the launch side value; see `fit_speed` for why it is not an error argmin).
Yaw channel: from the ICP yaw change per 0.1 s scan interval, integrate the model at 0.05 s
over the interval from the recorded commands and the ICP ground speed of the interval and
minimise the yaw change error, so the steering gain is path curvature per steering command,
which is what the planner needs against walls. (Driven by wheel speed the gain came out 0.77
instead of 0.94, because the wheels over-read ground speed by up to 25 percent at full lock and
the gain absorbed that ratio, which made the planner's minimum radius 0.91 m instead of 0.74 m.)
The warm up of the steering lag still uses the wheel speed. Bags with no odometry get their
wheel speed from the fitted speed model.

Acceleration limit: kept only if the fitted limit fits the rolling windows better than the start
value (see `rolling`).

Every parameter has physical bounds. A parameter within 0.5 percent of a bound, or whose profile
(held-out error against the parameter, the others refit) is flat, or one the data cannot move (a
zero sensitivity), is reported in `FitResult.unidentified` (identify.py). If the delay is not
identified the grid minimum ships, not a hand-picked value.
"""

from __future__ import annotations

import logging
from collections.abc import Sequence
from dataclasses import dataclass, field
from functools import partial

import numpy as np
from scipy.optimize import least_squares

from .bags import Run
from .dataset import Segment, by_split
from .deadzone import RESOLVED_WIDTH_MPS, Launch, find_launches
from .identify import TAU_GRID_S, Verdict, assess, profile_table
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
DEADBAND_GRID_MPS = tuple(round(0.01 * k, 2) for k in range(41))  # 0.00 .. 0.40
DELAY_BOUNDS_S = (DELAY_GRID_S[0], DELAY_GRID_S[-1])
DEADBAND_BOUNDS_MPS = (0.0, 1.0)  # the schema range; the grid searches the lower part
BOUNDS: dict[str, tuple[float, float]] = {
    "speed_gain": (0.5, 1.5),
    "speed_tau_s": (0.005, 1.0),
    "accel_max_mps2": (0.2, 20.0),
    "decel_max_mps2": (0.2, 20.0),
    "steer_gain_left": (0.5, 1.5),
    "steer_gain_right": (0.5, 1.5),
    "steer_offset_rad": (-0.1, 0.1),
    "steer_tau_s": (0.005, 1.0),
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
START_INSIDE_BOUND = 1e-3  # of the bound range: where a start that is on a bound is moved to


@dataclass
class FitResult:
    nominal: Nominal
    flags: dict[str, str] = field(default_factory=dict)
    speed_heldout_rmse_mps: float = float("nan")  # all steps of the 1 s windows
    yaw_heldout_rmse_rad: float = float("nan")  # per 0.1 s interval
    delay_curve: dict[float, float] = field(default_factory=dict)  # delay -> speed RMSE
    delay_selection: str = ""
    yaw_training: str = ""
    sides_tied: bool = False
    alternative: Nominal | None = None  # the fit with the other tying choice, for the record
    yaw_cv_rmse_rad: dict[str, float] = field(default_factory=dict)
    accel_rule: dict[str, object] = field(default_factory=dict)
    counts: dict[str, int] = field(default_factory=dict)
    # parameter -> [{"value", "rmse", "over_min"}]: error against the parameter, others refit
    profiles: dict[str, list[dict[str, float]]] = field(default_factory=dict)
    profile_notes: dict[str, str] = field(default_factory=dict)
    unidentified: dict[str, str] = field(default_factory=dict)  # parameter -> why
    deadband_choice: dict[str, object] = field(default_factory=dict)


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
        # A start exactly on a bound does not leave it (least_squares stays at 2e-10 for a start
        # of 0.0 on [0, 2] although the optimum is 0.5), so it is nudged a thousandth of the range
        # inside. A start that is not on a bound is untouched. Found in brief 31.
        inner = START_INSIDE_BOUND * (hi - lo)
        x0 = np.where(x0 <= lo, lo + inner, np.where(x0 >= hi, hi - inner, x0))

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


def yaw_rmse(data, p: Nominal, vehicle: Vehicle, speed: str = "ground") -> float:
    if len(data) == 0:
        return float("nan")
    pred = predict_yaw_change(p, vehicle, data, speed=speed)
    return float(np.sqrt(np.mean((pred - data.dyaw_rad) ** 2)))


ROLLING_MPS = 0.15  # a window is rolling when the car and every command are at least this fast


def rolling(windows: SpeedWindows) -> SpeedWindows:
    """The windows that start moving and are commanded at least ROLLING_MPS throughout, so the
    launch dead zone (wheels still while the command ramps from 0) is not in them."""
    keep = (windows.v_mps[:, 0] >= ROLLING_MPS) & (windows.cmds[:, :, 0] >= ROLLING_MPS).all(axis=1)
    return SpeedWindows(windows.v_mps[keep], windows.cmds[keep])


MIN_VAL_WINDOWS = 150  # fewer held-out windows than this and the choice is cross-validated
CV_FOLDS = 4
MAX_ROUNDS = 3  # rounds of delay / dead zone coordinate descent
SPEED_TAU_STARTS_S = (0.05, 0.2, 0.5)


def _fit_speed_at(
    windows: SpeedWindows,
    vehicle: Vehicle,
    start: Nominal,
    delay_s: float,
    deadband_mps: float,
    fixed: dict[str, float] | None = None,
    warm: Nominal | None = None,
) -> tuple[Nominal, dict[str, str], dict[str, object]]:
    """Speed parameters at one delay and one dead zone. `fixed` pins some of SPEED_PARAMS (a
    profile); the accel rule is skipped for a pinned accel limit. `warm` (the solution of a
    neighbouring grid cell) is one more start, so a profile does not jump between local minima."""
    fixed = fixed or {}
    base = start.replace(command_delay_s=delay_s, speed_deadband_mps=deadband_mps, **fixed)
    names = [n for n in SPEED_PARAMS if n not in fixed]
    # Starts: a clip that is not active has no gradient, so start the rate limits tight, at the
    # placeholder and loose; the surface in the time constant has local minima too (a lag that is
    # too long and one that is too short both fit a step part way), so start it short, medium and
    # long. Keep the best of all combinations.
    taus = (start.speed_tau_s,) if "speed_tau_s" in fixed else SPEED_TAU_STARTS_S
    starts = [
        base.replace(
            **{
                k: v
                for k, v in (
                    ("accel_max_mps2", a),
                    ("decel_max_mps2", b),
                    ("speed_tau_s", tau),
                )
                if k not in fixed
            }
        )
        for tau in taus
        for a, b in ((0.5, 0.5), (start.accel_max_mps2, start.decel_max_mps2), (10.0, 10.0))
    ]
    if warm is not None:
        # a neighbouring cell's solution plus the plain start is enough: the full set of starts
        # was needed once, for the first cell
        starts = [
            base.replace(**{n: getattr(warm, n) for n in SPEED_PARAMS if n not in fixed}),
            starts[1 if len(starts) > 1 else 0],
        ]
    flags: dict[str, str] = {}
    p = _fit_params(names, base, _speed_residual(windows, vehicle), flags, starts)
    rule: dict[str, object] = {}
    if "accel_max_mps2" in fixed:
        return p, flags, rule
    r = rolling(windows)
    kept = p.replace(accel_max_mps2=start.accel_max_mps2)
    rule = {
        "rolling_windows": len(r),
        "rmse_fitted": speed_rmse(r, p, vehicle),
        "rmse_start": speed_rmse(r, kept, vehicle),
        "kept_start": False,
        "fitted_value": p.accel_max_mps2,
    }
    if len(r) >= MIN_VAL_WINDOWS and rule["rmse_start"] <= rule["rmse_fitted"]:
        p = kept
        rule["kept_start"] = True
        flags["accel_max_mps2"] = (
            "not supported while rolling (launch windows only), kept at its start"
        )
    return p, flags, rule


@dataclass
class SpeedFit:
    nominal: Nominal
    flags: dict[str, str]
    accel_rule: dict[str, object]
    how: str  # what scored the grid cells
    delay_curve: dict[float, float]  # delay -> held-out error, dead zone as shipped
    deadband_curve: dict[float, float]  # dead zone -> held-out error, delay as shipped
    deadband_estimate: dict[str, object]  # how the dead zone was chosen
    cells: dict[tuple[float, float], tuple[Nominal, float]]
    score_windows: int
    fixed_start: Nominal


DEADBAND_ROUND_MPS = 0.01
HELD_WIDTH_MPS = 0.01  # a launch interval narrower than this: the car was held under that command


def launches_in_segments(segments: Sequence[Segment], delay_s: float = 0.0) -> list[Launch]:
    """Launches from rest whose launch instant (the wheels first turning) lies inside one of the
    segments. The rest the detector looks back over (0.7 s plus the delay) may start before the
    segment; the split file leaves at least 2 s between a held-out segment and the train segment
    that follows it, so that look back never reaches held-out data."""
    out: list[Launch] = []
    by_run: dict[int, list[Segment]] = {}
    for seg in segments:
        if seg.run.has_odom:
            by_run.setdefault(id(seg.run), []).append(seg)
    for segs in by_run.values():
        for launch in find_launches(segs[0].run, delay_s):
            if any(s.t0_s <= launch.t_s <= s.t1_s for s in segs):
                out.append(launch)
    return out


def launch_deadband(
    runs: Sequence[Run], delay_s: float = 0.0, launches: Sequence[Launch] | None = None
) -> dict[str, object]:
    """The launch side dead zone: the median command at which the wheels first turned over every
    launch from rest (deadzone.find_launches), rounded to DEADBAND_ROUND_MPS. Launches whose
    command was already as high for the whole rest (zero interval: the car was held or blocked,
    not an observation of a threshold) are left out. A launch whose command jumped over the zone
    is an upper bound and stays in; that makes the estimate lean high when most launches are
    jumps. Zero (no dead zone) when no launch is left."""
    if launches is None:
        launches = [x for r in runs if r.has_odom for x in find_launches(r, delay_s)]
    kept = [x for x in launches if abs(x.width_mps) > HELD_WIDTH_MPS]
    resolved = [x for x in kept if 0.0 < x.width_mps <= RESOLVED_WIDTH_MPS]
    out: dict[str, object] = {
        "launches": len(launches),
        "launches_used": len(kept),
        "resolved_launches": len(resolved),
    }
    if not kept:
        return {**out, "value_mps": 0.0, "rule": "no launch from rest: no dead zone"}
    cmds = np.array([x.cmd_launch_mps for x in kept])
    return {
        **out,
        "value_mps": round(float(np.median(cmds)) / DEADBAND_ROUND_MPS) * DEADBAND_ROUND_MPS,
        "used_min_mps": float(cmds.min()),
        "used_max_mps": float(cmds.max()),
        "rule": "median command at which the wheels turned over the launches from rest",
    }


def fit_speed(
    train: Sequence[Segment],
    heldout: Sequence[Segment],
    vehicle: Vehicle,
    start: Nominal = PLACEHOLDER_NOMINAL,
    delays: Sequence[float] = DELAY_GRID_S,
    deadbands: Sequence[float] = DEADBAND_GRID_MPS,
    deadband: float | None = None,
    deadband_estimate: dict[str, object] | None = None,
) -> SpeedFit:
    """Speed parameters, the command delay and the dead zone.

    The DEAD ZONE is the launch side value `deadband` (see `launch_deadband`: the median command
    of the launches from rest), not the argmin of a speed error curve. The held-out
    windows of the real bags hold almost no command inside the dead zone band (a handful of 867
    windows), so the held-out error cannot identify it: the curve over the grid moves by a few
    percent with the local minimum each refit lands in, and the error over the windows that start
    at rest prefers 0.4 m/s because it is dominated by windows where the car was held still
    under 0.36 to 0.6 m/s. The brief says to take the launch side value then. The held-out
    curves are reported, and the dead zone is listed as not identified when its held-out
    profile is flat. With `deadband` None the grid is searched by held-out error instead (what
    a synthetic test with known truth does).

    The DELAY is searched on its grid; every cell refits the continuous speed parameters on the
    training windows and is scored on the held-out windows. With fewer than MIN_VAL_WINDOWS
    held-out windows the score is the mean error of blocked cross-validation over the training
    segments (segments dealt into CV_FOLDS groups) and, with no odometry segments at all, the
    training error."""
    odom_train = [s for s in train if s.run.has_odom]
    use_heldout = len(speed_windows(heldout, delays[0])) >= MIN_VAL_WINDOWS
    k = min(CV_FOLDS, len(odom_train))
    folds = [odom_train[i::k] for i in range(k)] if not use_heldout and k >= 2 else []
    cells: dict[tuple[float, float], tuple[Nominal, float]] = {}
    last: list[Nominal | None] = [None]  # the previous cell's solution warm starts the next

    def cell(d: float, db: float) -> float:
        if (d, db) in cells:
            return cells[(d, db)][1]
        p, _, _ = _fit_speed_at(speed_windows(train, d), vehicle, start, d, db, warm=last[0])
        last[0] = p
        if use_heldout:
            err = speed_rmse(speed_windows(heldout, d), p, vehicle)
        elif folds:
            errors = []
            for j, held in enumerate(folds):
                rest = [s for i, f in enumerate(folds) if i != j for s in f]
                pj, _, _ = _fit_speed_at(speed_windows(rest, d), vehicle, start, d, db)
                errors.append(speed_rmse(speed_windows(held, d), pj, vehicle))
            err = float(np.mean(errors))
        else:  # nothing to validate on: the training error is all there is
            err = speed_rmse(speed_windows(train, d), p, vehicle)
        cells[(d, db)] = (p, err)
        return err

    def best(values: Sequence[float], fn) -> float:
        errs = {v: fn(v) for v in values}
        return min(values, key=lambda v: np.inf if np.isnan(errs[v]) else errs[v])

    estimate = dict(deadband_estimate or {})
    if deadband is not None:
        db = float(deadband)
        d = best(delays, partial(lambda db_, x: cell(x, db_), db))
        estimate.setdefault("rule", "given")
    else:
        d, db = delays[0], deadbands[0]
        for _ in range(MAX_ROUNDS):
            d_new = best(delays, partial(lambda db_, x: cell(x, db_), db))
            db_new = best(deadbands, partial(cell, d_new))
            done = (d_new, db_new) == (d, db)
            d, db = d_new, db_new
            if done:
                break
        estimate["rule"] = "argmin of the held-out speed error over the grid"
    delay_curve = {x: cell(x, db) for x in delays}
    deadband_curve = {x: cell(d, x) for x in deadbands}
    p, flags, rule = _fit_speed_at(
        speed_windows(train, d), vehicle, start, d, db, warm=cells[(d, db)][0]
    )
    how = (
        "held-out windows"
        if use_heldout
        else f"{k}-fold cross-validation over training segments"
        if folds
        else "training error (no held-out data)"
    )
    n_score = len(speed_windows(heldout if use_heldout else train, d))
    estimate["fitted_mps"] = db
    return SpeedFit(
        p, flags, rule, how, delay_curve, deadband_curve, estimate, cells, n_score, start
    )  # fmt: skip


def speed_profile(
    fit: SpeedFit,
    train: Sequence[Segment],
    heldout: Sequence[Segment],
    vehicle: Vehicle,
    name: str,
    grid: Sequence[float],
) -> list[float]:
    """Held-out speed error with `name` (one of SPEED_PARAMS) pinned at each grid value, the
    other continuous speed parameters refit on the training windows, delay and dead zone at their
    chosen values."""
    d, db = fit.nominal.command_delay_s, fit.nominal.speed_deadband_mps
    score = heldout if len(speed_windows(heldout, d)) >= MIN_VAL_WINDOWS else train
    out = []
    for x in grid:
        p, _, _ = _fit_speed_at(speed_windows(train, d), vehicle, fit.fixed_start, d, db, {name: x})
        out.append(speed_rmse(speed_windows(score, d), p, vehicle))
    return out


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
    fixed: dict[str, float] | None = None,
) -> tuple[Nominal, dict[str, str]]:
    """Yaw channel. With `tied` the two gains are one parameter and the offset is zero. `fixed`
    pins some yaw parameters (a profile)."""
    fixed = fixed or {}
    nominal = nominal.replace(**fixed)
    data = yaw_data(train, nominal.command_delay_s, wheel_by_run)
    flags: dict[str, str] = {}

    def fn(p: Nominal) -> np.ndarray:
        p = tie_sides(p) if tied else p
        return predict_yaw_change(p, vehicle, data, speed="ground") - data.dyaw_rad

    if not tied:
        names = [n for n in YAW_PARAMS if n not in fixed]
        return _fit_params(names, nominal, fn, flags), flags
    names = [n for n in TIED_YAW_PARAMS if n not in fixed]
    out = _fit_params(names, tie_sides(nominal), fn, flags)
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


def _verdicts(
    nominal: Nominal,
    delay_curve: dict[float, float],
    deadband_curve: dict[float, float],
    tau_speed: tuple[Sequence[float], Sequence[float]],
    tau_steer: tuple[Sequence[float], Sequence[float]],
    flags: dict[str, str],
) -> tuple[dict[str, Verdict], dict[str, list[dict[str, float]]]]:
    """The identification verdict and the profile table of every parameter that has a profile,
    plus a verdict from the fit flags alone for the rest."""
    verdicts: dict[str, Verdict] = {}
    profiles: dict[str, list[dict[str, float]]] = {}
    items = (
        ("command_delay_s", nominal.command_delay_s, DELAY_BOUNDS_S,
         list(delay_curve), list(delay_curve.values()), False, True),
        ("speed_deadband_mps", nominal.speed_deadband_mps, (min(deadband_curve), max(deadband_curve)),
         list(deadband_curve), list(deadband_curve.values()), False, False),
        ("speed_tau_s", nominal.speed_tau_s, BOUNDS["speed_tau_s"],
         list(tau_speed[0]), list(tau_speed[1]), True, True),
        ("steer_tau_s", nominal.steer_tau_s, BOUNDS["steer_tau_s"],
         list(tau_steer[0]), list(tau_steer[1]), True, True),
    )  # fmt: skip
    for name, value, bounds, grid, errs, log_scale, ends in items:
        profiles[name] = profile_table(grid, errs)
        verdicts[name] = assess(value, bounds, grid, errs, log_scale, ends)
    for name, why in flags.items():
        if name in verdicts:
            if why:
                v = verdicts[name]
                verdicts[name] = Verdict(False, (*v.reasons, why))
        else:
            verdicts[name] = Verdict(False, (why,))
    return verdicts, profiles


def fit_nominal(
    segments: Sequence[Segment],
    vehicle: Vehicle,
    start: Nominal = PLACEHOLDER_NOMINAL,
    delays: Sequence[float] = DELAY_GRID_S,
    yaw_training: str = "auto",
    deadbands: Sequence[float] = DEADBAND_GRID_MPS,
    tau_grid: Sequence[float] = TAU_GRID_S,
    deadband: str | float | None = "launch",
    fit_split: str = "train",
    score_split: str = "heldout",
    launch_splits: Sequence[str] | None = None,
) -> FitResult:
    """The whole fit: speed channel, delay and dead zone (odometry bags), then the yaw channel,
    then the profiles that say which parameters the data identify. `yaw_training` is "all",
    "odom_only" or "auto" (choose_yaw_training). `deadband` is "launch" (the median launch
    command, `launch_deadband`), a number (given), or None (searched on the grid by held-out
    error).

    `fit_split` is the role the parameters are fitted on and `score_split` the role that scores
    the delay and the profiles (the dyn-v0.1 defaults: "train" and "heldout"; v0.2 fits on
    "train" and scores on its internal validation role "val", never on held-out). `launch_splits`
    limits the launches and steady bins of the dead zone estimates to those roles (None: every
    segment's whole run, the v0.1 behaviour)."""
    train, heldout = by_split(segments, fit_split), by_split(segments, score_split)
    estimate: dict[str, object] | None = None
    runs = list({id(s.run): s.run for s in segments if s.run.has_odom}.values())
    launch_segs = (
        None if launch_splits is None else [x for r in launch_splits for x in by_split(segments, r)]
    )

    def launches_for(delay_s: float) -> list[Launch] | None:
        return None if launch_segs is None else launches_in_segments(launch_segs, delay_s)

    if deadband == "launch":
        estimate = launch_deadband(runs, 0.0, launches_for(0.0))
        deadband = float(estimate["value_mps"])  # type: ignore[arg-type]
    speed = fit_speed(
        train,
        heldout,
        vehicle,
        start,
        delays,
        deadbands,
        deadband,
        estimate,  # type: ignore[arg-type]
    )
    if estimate is not None and speed.nominal.command_delay_s > 0.0:
        # The launch command is paired with the command the wheels were answering, so it needs
        # the delay; estimate again with the fitted delay and refit once.
        estimate = launch_deadband(
            runs, speed.nominal.command_delay_s, launches_for(speed.nominal.command_delay_s)
        )
        estimate["estimated_with_delay_s"] = speed.nominal.command_delay_s
        deadband = float(estimate["value_mps"])  # type: ignore[arg-type]
        speed = fit_speed(
            train,
            heldout,
            vehicle,
            start,
            delays,
            deadbands,
            deadband,
            estimate,  # type: ignore[arg-type]
        )
    speed_nominal, flags, how, accel_rule = speed.nominal, speed.flags, speed.how, speed.accel_rule
    # An unidentified delay ships the grid minimum, not the argmin of a flat curve.
    delay_verdict = assess(
        speed_nominal.command_delay_s,
        DELAY_BOUNDS_S,
        list(speed.delay_curve),
        list(speed.delay_curve.values()),
        False,
        True,
    )
    delay_note = ""
    if not delay_verdict.identified and speed_nominal.command_delay_s != delays[0]:
        delay_note = (
            f"not identified ({delay_verdict.text()}): the grid minimum {delays[0]:g} s ships "
            f"instead of the argmin {speed_nominal.command_delay_s:g} s"
        )
        speed_nominal, flags, accel_rule = _refit_speed_at_delay(
            train, vehicle, start, delays[0], speed_nominal.speed_deadband_mps
        )
        speed.nominal = speed_nominal
    wheel = wheel_speeds_for([*train, *heldout], speed_nominal, vehicle)
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
    w_held = speed_windows(heldout, delay)
    y_held = yaw_data(heldout, delay, wheel)
    y_train = yaw_data(yaw_train, delay, wheel)

    # Profiles of the two time constants, the others refit; held-out error (training when there
    # are no held-out windows).
    tau_speed = (
        list(tau_grid),
        speed_profile(speed, train, heldout, vehicle, "speed_tau_s", tau_grid),
    )
    y_score = y_held if len(y_held) else y_train
    tau_steer_errs = []
    for x in tau_grid:
        p, _ = fit_yaw(yaw_train, vehicle, nominal, wheel, tied=tied, fixed={"steer_tau_s": x})
        tau_steer_errs.append(yaw_rmse(y_score, p, vehicle))
    # The dead zone profile with the OTHER parameters FIXED at their shipped values is the clean
    # one: the held-out windows hold almost no command inside the dead zone band, so the refit
    # profile (speed.deadband_curve) moves more with the local minimum each cell's refit lands
    # in than with the dead zone. The verdict uses the clean one; both tables are reported.
    held_windows = speed_windows(heldout if len(w_held) else train, delay)
    fixed_curve = {
        x: speed_rmse(held_windows, nominal.replace(speed_deadband_mps=x), vehicle)
        for x in deadbands
    }
    verdicts, profiles = _verdicts(
        nominal, speed.delay_curve, fixed_curve, tau_speed,
        (list(tau_grid), tau_steer_errs), {k: "" for k in ()},
    )  # fmt: skip
    profiles["speed_deadband_mps_refit"] = profile_table(
        list(speed.deadband_curve), list(speed.deadband_curve.values())
    )
    unidentified = {k: v.text() for k, v in verdicts.items() if not v.identified}
    for name, why in flags.items():
        if why.startswith(("on a bound", "not identified")):
            unidentified[name] = "; ".join(x for x in (unidentified.get(name), why) if x)
    if delay_note:
        unidentified["command_delay_s"] = delay_note
    return FitResult(
        nominal=nominal,
        flags=flags,
        speed_heldout_rmse_mps=speed_rmse(w_held, nominal, vehicle),
        yaw_heldout_rmse_rad=yaw_rmse(y_held, nominal, vehicle),
        delay_curve=speed.delay_curve,
        delay_selection=how,
        yaw_training=mode,
        sides_tied=tied,
        alternative=alternative,
        yaw_cv_rmse_rad=cv,
        accel_rule=accel_rule,
        counts={
            "speed_windows_train": len(speed_windows(train, delay)),
            "speed_windows_heldout": len(w_held),
            "yaw_intervals_train": len(y_train),
            "yaw_intervals_heldout": len(y_held),
            "yaw_left_train": side_counts(y_train)[0],
            "yaw_right_train": side_counts(y_train)[1],
        },
        profiles=profiles,
        profile_notes={
            "speed_deadband_mps": (
                "held-out RMSE over all windows with the dead zone varied and every other "
                "parameter fixed at its shipped value; the shipped value is the launch side "
                "median (deadband_choice), not this argmin"
            ),
            "speed_deadband_mps_refit": (
                "the same with the continuous speed parameters refit at each value: moves with "
                "the local minimum of each refit more than with the dead zone"
            ),
            "command_delay_s": "dead zone at its chosen value, continuous speed parameters refit",
            "speed_tau_s": "gain, accel and decel refit; delay and dead zone at chosen values",
            "steer_tau_s": "yaw gains and understeer refit (yaw RMSE per 0.1 s interval)",
        },
        unidentified=unidentified,
        deadband_choice={
            **speed.deadband_estimate,
            "heldout_all_windows_argmin_mps_others_fixed": min(fixed_curve, key=fixed_curve.get),
            "heldout_all_windows_rmse_at_fitted": fixed_curve[nominal.speed_deadband_mps],
            "heldout_all_windows_rmse_min_others_fixed": min(fixed_curve.values()),
            "heldout_all_windows_argmin_mps_refit": min(
                speed.deadband_curve, key=lambda x: speed.deadband_curve[x]
            ),
            "heldout_all_windows_rmse_min_refit": min(speed.deadband_curve.values()),
            "heldout_windows_with_a_command_in_the_band": int(
                (
                    (held_windows.cmds[..., 0] >= 0.05)
                    & (held_windows.cmds[..., 0] < nominal.speed_deadband_mps)
                )
                .any(axis=1)
                .sum()
            ),
            "heldout_windows": len(held_windows),
            "grid_mps": [deadbands[0], deadbands[-1]],
        },
    )


def _refit_speed_at_delay(
    train: Sequence[Segment], vehicle: Vehicle, start: Nominal, delay_s: float, deadband: float
) -> tuple[Nominal, dict[str, str], dict[str, object]]:
    return _fit_speed_at(speed_windows(train, delay_s), vehicle, start, delay_s, deadband)
