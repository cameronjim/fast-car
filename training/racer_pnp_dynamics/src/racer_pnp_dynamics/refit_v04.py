"""N4, the classical refit of dyn-v0.1 on the v0.4 train data (pre-registration v0.4 section 2.4,
brief 37 section 3).

What is different from the v0.3 yaw refit Y: the yaw parameters are fitted on the OBJECTIVE THE GATE
SCORES, not on one-step ICP intervals. Y minimised the yaw change over single 0.1 s scan intervals
with the ICP ground speed as the speed, then was scored on 0.5 to 1.5 s open-loop rollouts from the
wheel speed. The ground speed is noisy and the noise in a regressor biases a gain; the rollout
objective never reads it.

* Windows: `evaluate.eval_set` of the train role, the very windows (starts every 0.25 s, start
  speed at least 0.25 m/s, strata) the gate builds. The model starts from the measured wheel speed
  and a steering state rebuilt from the command history, then runs on the recorded `/drive`
  commands only, delayed by the model's own `command_delay_s`.
* Objective: at 0.5 s and at 1.0 s the yaw error and the two position components (start frame), each
  divided by dyn-v0.1's own RMS of that term on the same windows (so yaw in radians and position in
  metres weigh the same), each window weighted by one over the number of windows of its stratum
  (so every stratum counts equally, as the gate's macro average does).
* Free parameters: `steer_gain_left`, `steer_gain_right`, `steer_offset_rad`, `steer_tau_s`,
  `understeer_s2_per_m2`, same bounds as dyn-v0.1 (`fit.BOUNDS`). The speed parameters, the command
  delay and the dead zone come from the existing speed fit (`fit.fit_nominal`) on the same data.
* No new parameter and no schema change.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field

import numpy as np

from . import fit
from .dataset import Segment, by_split
from .evaluate import EvalSet, eval_set, predict
from .icp_motion import SCAN_DT_S
from .identify import assess
from .model import DT_S, Nominal
from .strata import assign_strata
from .vehicle import Vehicle

YAW_FREE = fit.YAW_PARAMS
HORIZONS_S = (0.5, 1.0)  # brief 37 section 3
PROFILE_POINTS = 21
SPEED_FIELDS = (*fit.SPEED_PARAMS, "speed_deadband_mps", "command_delay_s")
EPS = 1e-12


@dataclass(frozen=True)
class Objective:
    """The rollout objective on one set of windows, with the normalisers fixed by `base`."""

    es: EvalSet
    delay_s: float
    vehicle: Vehicle
    sigmas: tuple[float, float]  # (yaw rad, position m) of the base on these windows
    weights: np.ndarray  # (E,) proportional to one over the stratum's window count, mean 1

    def residuals(self, p: Nominal) -> np.ndarray:
        """Weighted, normalised residual vector (E * len(HORIZONS_S) * 3)."""
        return self.raw(p, self.sigmas, self.weights)

    def raw(self, p: Nominal, sigmas: Sequence[float], weights: np.ndarray) -> np.ndarray:
        states = predict(self.es.data(self.delay_s), p, self.vehicle)
        sw = np.sqrt(weights)[:, None]
        cols = []
        for h in HORIZONS_S:
            k, j = round(h / DT_S), round(h / SCAN_DT_S)
            cols.append((states[:, k, 2] - self.es.poses[:, j, 2]) / sigmas[0])
            cols.append((states[:, k, 0] - self.es.poses[:, j, 0]) / sigmas[1])
            cols.append((states[:, k, 1] - self.es.poses[:, j, 1]) / sigmas[1])
        return (np.stack(cols, axis=1) * sw).ravel()

    def cost(self, p: Nominal) -> float:
        """Mean squared weighted normalised residual (dimensionless); 1.0 is the base's level on
        its own windows up to the weighting."""
        r = self.residuals(p)
        return float(np.mean(r**2))


def make_objective(es: EvalSet, base: Nominal, vehicle: Vehicle, delay_s: float) -> Objective:
    """Normalise by the RMS of `base` (the nominal being refitted, before the refit) on `es`."""
    if len(es) == 0:
        raise ValueError("no windows for the refit objective")
    strata = assign_strata(es.direction, es.start_speed)
    counts = np.array([strata.counts[s] for s in strata.label], dtype=float)
    weights = 1.0 / counts
    weights = weights * (len(es) / len(strata.names))  # every stratum sums to E / strata: mean 1
    states = predict(es.data(delay_s), base, vehicle)
    yaw, pos = [], []
    for h in HORIZONS_S:
        k, j = round(h / DT_S), round(h / SCAN_DT_S)
        yaw.append(states[:, k, 2] - es.poses[:, j, 2])
        pos.append(states[:, k, 0] - es.poses[:, j, 0])
        pos.append(states[:, k, 1] - es.poses[:, j, 1])
    s_yaw = float(np.sqrt(np.mean(np.square(yaw))))
    s_pos = float(np.sqrt(np.mean(np.square(pos))))
    return Objective(es, delay_s, vehicle, (max(s_yaw, EPS), max(s_pos, EPS)), weights)


def fit_yaw_rollout(
    es: EvalSet,
    vehicle: Vehicle,
    start: Nominal,
    starts: Sequence[Nominal] | None = None,
    fixed: dict[str, float] | None = None,
) -> tuple[Nominal, dict[str, str]]:
    """Fit the yaw parameters of `start` by the rollout objective. `fixed` pins some of them (a
    profile point). The command delay is `start`'s and is not fitted here."""
    fixed = fixed or {}
    start = start.replace(**fixed)
    obj = make_objective(es, start, vehicle, start.command_delay_s)
    names = [n for n in YAW_FREE if n not in fixed]
    flags: dict[str, str] = {}
    cand = [s.replace(**fixed) for s in (starts or [start])]
    fitted = fit._fit_params(names, start, obj.residuals, flags, starts=cand)
    return fitted, flags


@dataclass
class N4Fit:
    """The fitted nominal of N4 and what the report says about it."""

    base: Nominal
    speed_nominal: Nominal
    nominal: Nominal
    bounds: dict[str, tuple[float, float]]
    on_bound: list[str]
    flags: dict[str, str]
    unidentified: dict[str, str]
    profiles: dict[str, list[dict[str, float]]]
    cost: dict[str, float]  # rollout objective on the validation windows: base, speed fit only, N4
    counts: dict[str, int]
    speed_flags: dict[str, str] = field(default_factory=dict)
    objective: str = (
        "yaw and position of 0.5 s and 1.0 s open-loop rollouts, windows of the gate, strata "
        "weighted equally, each term divided by dyn-v0.1's RMS on the train windows"
    )

    def parameter_table(self) -> list[dict[str, object]]:
        rows = []
        for name, value in self.nominal.to_dict().items():
            lo, hi = fit.BOUNDS.get(name, (float("nan"), float("nan")))
            rows.append(
                {
                    "parameter": name,
                    "dyn_v0.1": self.base.to_dict()[name],
                    "n4": value,
                    "lower": lo,
                    "upper": hi,
                    "free_in_yaw_fit": name in YAW_FREE,
                    "on_bound": name in self.on_bound,
                    "flag": self.flags.get(name, self.speed_flags.get(name, "")),
                }
            )
        return rows


def fit_n4(
    segments: Sequence[Segment],
    vehicle: Vehicle,
    base: Nominal,
    speed_nominal: Nominal | None = None,
    speed_flags: dict[str, str] | None = None,
) -> N4Fit:
    """N4 on the train role of `segments` (validation role only for the profiles and the reported
    cost). `speed_nominal` is the result of the existing speed fit; with None it is computed here
    with `fit.fit_nominal` on the same segments as the v0.2 pipeline does."""
    train = [s for s in by_split(segments, "train") if s.run.has_odom and s.motion is not None]
    val = [s for s in by_split(segments, "val") if s.run.has_odom and s.motion is not None]
    es_train, es_val = eval_set(train), eval_set(val)
    if speed_nominal is None:
        result = fit.fit_nominal(
            segments, vehicle, fit_split="train", score_split="val",
            launch_splits=("train", "val"),
        )  # fmt: skip
        speed_nominal, speed_flags = result.nominal, dict(result.flags)
    carried = {n: getattr(speed_nominal, n) for n in SPEED_FIELDS}
    start = base.replace(**carried)
    starts = [start]
    if any(getattr(speed_nominal, n) != getattr(base, n) for n in YAW_FREE):
        starts.append(start.replace(**{n: getattr(speed_nominal, n) for n in YAW_FREE}))
    nominal, flags = fit_yaw_rollout(es_train, vehicle, start, starts)
    obj_val = make_objective(es_val, base, vehicle, start.command_delay_s) if len(es_val) else None
    bounds = {n: fit.BOUNDS[n] for n in YAW_FREE}
    profiles: dict[str, list[dict[str, float]]] = {}
    unidentified: dict[str, str] = {}
    score = (
        obj_val
        if obj_val is not None
        else make_objective(es_train, base, vehicle, start.command_delay_s)
    )
    for name in YAW_FREE:
        lo, hi = bounds[name]
        grid = np.linspace(lo, hi, PROFILE_POINTS)
        costs = []
        for x in grid:
            p, _ = fit_yaw_rollout(es_train, vehicle, nominal, fixed={name: float(x)})
            costs.append(score.cost(p))
        best = float(np.nanmin(costs))
        profiles[name] = [
            {"value": float(x), "cost": float(c), "over_min": float(c / best - 1.0)}
            for x, c in zip(grid, costs, strict=True)
        ]
        verdict = assess(getattr(nominal, name), bounds[name], list(grid), costs, False, True)
        if not verdict.identified:
            unidentified[name] = verdict.text()
    for name, why in flags.items():
        if why.startswith(("on a bound", "not identified")):
            unidentified[name] = "; ".join(x for x in (unidentified.get(name), why) if x)
    on_bound = [n for n in YAW_FREE if flags.get(n, "").startswith("on a bound")]
    return N4Fit(
        base=base,
        speed_nominal=start,
        nominal=nominal,
        bounds=bounds,
        on_bound=on_bound,
        flags=flags,
        unidentified=unidentified,
        profiles=profiles,
        cost={
            "dyn-v0.1": score.cost(base),
            "speed fit only": score.cost(start),
            "n4": score.cost(nominal),
        },
        counts={
            "windows_train": len(es_train),
            "windows_val": len(es_val),
            "train_segments": len(train),
            "val_segments": len(val),
        },
        speed_flags=speed_flags or {},
    )


def plot_profiles(path, refit: N4Fit, title: str) -> None:  # pragma: no cover - plotting
    """One panel per yaw parameter: the rollout objective on the validation windows against the
    parameter (the other four refit), N4's value and dyn-v0.1's value marked, bounds are the axis."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, len(YAW_FREE), figsize=(3.6 * len(YAW_FREE), 3.4))
    for ax, name in zip(axes, YAW_FREE, strict=True):
        rows = refit.profiles[name]
        ax.plot([r["value"] for r in rows], [r["cost"] for r in rows], marker="o", ms=3)
        ax.axvline(getattr(refit.nominal, name), color="C1", label="N4")
        ax.axvline(getattr(refit.base, name), color="C2", ls="--", label="dyn-v0.1")
        ax.set_xlabel(name)
        ax.set_ylabel("rollout objective (validation)")
        if name in refit.on_bound:
            ax.set_title("on a bound", color="C3")
    axes[0].legend()
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
