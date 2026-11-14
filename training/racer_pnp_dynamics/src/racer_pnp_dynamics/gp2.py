"""GP2 decisions of docs/notes/preregistration-pnp-v0.2.md section 3, as functions.

Every item of a decision is returned with its value, its threshold and whether it passed, so the
report can show all of them (a failed gate is reported in full, not as a bare "no"). The
thresholds are module constants quoting the pre-registration. They never change after a result
(rule 10 of the v0.2 briefs).
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np

from .model import DT_S, Nominal, Residual, rollout
from .vehicle import Vehicle

# 3.1 "The refit ships if P1(v0.2) is at most P1(v0.1), S1(v0.2) at most 1.05 times S1(v0.1) and
# Y1(v0.2) at most 1.05 times Y1(v0.1)."
REFIT_P1_MAX_RATIO = 1.00
REFIT_S1_MAX_RATIO = 1.05
REFIT_Y1_MAX_RATIO = 1.05

# 3.2 item 1 "P1(residual, shipping seed) at most 0.95 times P1(nominal), and the upper end of the
# bootstrap interval of that ratio is below 1.00."
RESIDUAL_P1_MAX_RATIO = 0.95
RESIDUAL_P1_CI_HI_BELOW = 1.00
# item 2 "Y1(residual) at most 1.00 times Y1(nominal) and S1(residual) at most 1.02 times S1(nominal)."
RESIDUAL_Y1_MAX_RATIO = 1.00
RESIDUAL_S1_MAX_RATIO = 1.02
# item 3 "Every seed: P1 at most 1.00 times P1(nominal)."
RESIDUAL_SEED_P1_MAX_RATIO = 1.00
# item 4 "Every stratum (after merging): position RMSE at 1.0 s at most 1.05 times the nominal's."
RESIDUAL_STRATUM_MAX_RATIO = 1.05
# item 5 "Rollouts stay finite and inside the bundle's state bounds on a 10^5 sample random sweep of
# the validity box" and brief 23: 30 steps, |delta| inside the steering limits, v at most 1.5
# times v_max_mps.
SWEEP_SAMPLES = 100_000
SWEEP_STEPS = 30
SWEEP_V_FACTOR = 1.5
SWEEP_SEED = 20261113


@dataclass(frozen=True)
class Item:
    name: str
    value: float
    comparison: str  # "<=" or "<"
    threshold: float
    passed: bool
    note: str = ""

    def to_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "value": self.value,
            "comparison": self.comparison,
            "threshold": self.threshold,
            "passed": self.passed,
            "note": self.note,
        }


@dataclass(frozen=True)
class Decision:
    gate: str
    items: tuple[Item, ...]

    @property
    def passed(self) -> bool:
        return all(i.passed for i in self.items)

    def to_dict(self) -> dict[str, object]:
        return {
            "gate": self.gate,
            "passed": self.passed,
            "items": [i.to_dict() for i in self.items],
        }


def _item(name: str, value: float, comparison: str, threshold: float, note: str = "") -> Item:
    if not np.isfinite(value):
        ok = False  # no number, no pass
    elif comparison == "<=":
        ok = value <= threshold
    elif comparison == "<":
        ok = value < threshold
    else:  # pragma: no cover - defensive
        raise ValueError(comparison)
    return Item(name, float(value), comparison, threshold, bool(ok), note)


def decide_refit(v02: Mapping[str, float], v01: Mapping[str, float]) -> Decision:
    """3.1: the nominal refit against dyn-v0.1 on the held-out segments. `v02` and `v01` hold the
    macro `P1`, `Y1`, `S1` (metres, radians, m/s at 1.0 s)."""
    return Decision(
        "3.1 refit",
        (
            _item("P1 ratio v0.2 / v0.1", v02["P1"] / v01["P1"], "<=", REFIT_P1_MAX_RATIO),
            _item("S1 ratio v0.2 / v0.1", v02["S1"] / v01["S1"], "<=", REFIT_S1_MAX_RATIO),
            _item("Y1 ratio v0.2 / v0.1", v02["Y1"] / v01["Y1"], "<=", REFIT_Y1_MAX_RATIO),
        ),
    )


@dataclass(frozen=True)
class SweepResult:
    samples: int
    steps: int
    nonfinite: int  # rollouts with a state that is not finite
    steering_violations: int  # rollouts with |delta| beyond the steering limits
    speed_violations: int  # rollouts with v above SWEEP_V_FACTOR * v_max_mps
    max_speed_mps: float
    max_abs_delta_rad: float

    @property
    def passed(self) -> bool:
        return self.nonfinite == 0 and self.steering_violations == 0 and self.speed_violations == 0

    def to_dict(self) -> dict[str, object]:
        return {**self.__dict__, "passed": self.passed}


def stability_sweep(
    nominal: Nominal,
    residual: Residual | None,
    vehicle: Vehicle,
    v_max_mps: float,
    samples: int = SWEEP_SAMPLES,
    steps: int = SWEEP_STEPS,
    seed: int = SWEEP_SEED,
    delta_limits: tuple[float, float] | None = None,
) -> SweepResult:
    """Item 5: `samples` uniformly random states and commands in the validity box, rolled `steps`
    steps (a new random command every step, harsher than a planner's smooth sequence). The box is
    speed 0..v_max, steering angle inside the steering limits, speed command 0..v_max and
    steering command inside the limits, yaw and position irrelevant to the model's dynamics."""
    rng = np.random.default_rng(seed)
    lo, hi = vehicle.steer_min_rad, vehicle.steer_max_rad
    d_lo, d_hi = delta_limits or (lo, hi)
    state = np.zeros((samples, 5))
    state[:, 2] = rng.uniform(-np.pi, np.pi, samples)
    state[:, 3] = rng.uniform(0.0, v_max_mps, samples)
    state[:, 4] = rng.uniform(lo, hi, samples)
    controls = np.stack(
        [rng.uniform(0.0, v_max_mps, (samples, steps)), rng.uniform(lo, hi, (samples, steps))],
        axis=-1,
    )
    with np.errstate(all="ignore"):
        states = rollout(state, controls, nominal, vehicle, residual, dt=DT_S)
        finite = np.isfinite(states).all(axis=(1, 2))
        v = np.where(np.isfinite(states[..., 3]), states[..., 3], np.inf)
        delta = np.where(np.isfinite(states[..., 4]), states[..., 4], np.inf)
        tol = 1e-12
        steer_bad = ((delta < d_lo - tol) | (delta > d_hi + tol)).any(axis=1)
        speed_bad = (v > SWEEP_V_FACTOR * v_max_mps).any(axis=1)
    return SweepResult(
        samples=samples,
        steps=steps,
        nonfinite=int((~finite).sum()),
        steering_violations=int((steer_bad & finite).sum()),
        speed_violations=int((speed_bad & finite).sum()),
        max_speed_mps=float(np.max(v[finite])) if finite.any() else float("inf"),
        max_abs_delta_rad=float(np.max(np.abs(delta[finite]))) if finite.any() else float("inf"),
    )


def decide_residual(
    nominal: Mapping[str, float],
    residual: Mapping[str, float],
    ratio_ci_hi: float,
    seed_p1: Mapping[int, float],
    stratum_nominal_pos: Mapping[str, float],
    stratum_residual_pos: Mapping[str, float],
    sweep: SweepResult,
) -> Decision:
    """3.2 items 1 to 5 on the held-out segments. `nominal` and `residual` hold the macro `P1`,
    `Y1`, `S1` of the nominal and of the shipping seed; `ratio_ci_hi` is the upper end of the 95
    percent bootstrap interval of the P1 ratio; `seed_p1` is the macro P1 of every seed;
    the two stratum maps are per stratum position RMSE at 1.0 s (after merging)."""
    items = [
        _item(
            "1a P1 ratio residual / nominal",
            residual["P1"] / nominal["P1"],
            "<=",
            RESIDUAL_P1_MAX_RATIO,
        ),
        _item(
            "1b upper end of the bootstrap interval of the P1 ratio",
            ratio_ci_hi,
            "<",
            RESIDUAL_P1_CI_HI_BELOW,
        ),
        _item(
            "2a Y1 ratio residual / nominal",
            residual["Y1"] / nominal["Y1"],
            "<=",
            RESIDUAL_Y1_MAX_RATIO,
        ),
        _item(
            "2b S1 ratio residual / nominal",
            residual["S1"] / nominal["S1"],
            "<=",
            RESIDUAL_S1_MAX_RATIO,
        ),
    ]
    for seed, p1 in sorted(seed_p1.items()):
        items.append(
            _item(
                f"3 seed {seed} P1 ratio to nominal",
                p1 / nominal["P1"],
                "<=",
                RESIDUAL_SEED_P1_MAX_RATIO,
            )
        )
    for name, nom in stratum_nominal_pos.items():
        items.append(
            _item(
                f"4 stratum {name} position RMSE ratio",
                stratum_residual_pos[name] / nom,
                "<=",
                RESIDUAL_STRATUM_MAX_RATIO,
            )
        )
    items.append(
        Item(
            "5 random sweep of the validity box: finite, inside the state bounds",
            float(sweep.nonfinite + sweep.steering_violations + sweep.speed_violations),
            "<=",
            0.0,
            sweep.passed,
            f"{sweep.samples} rollouts of {sweep.steps} steps; max v {sweep.max_speed_mps:.3f} m/s",
        )
    )
    return Decision("3.2 GP2a residual", tuple(items))
