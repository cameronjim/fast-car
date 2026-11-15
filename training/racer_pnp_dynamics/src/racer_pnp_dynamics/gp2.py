"""GP2 decisions of docs/notes/preregistration-pnp-v0.2.md section 3, as functions.

Every item of a decision is returned with its value, its threshold and whether it passed, so the
report can show all of them (a failed gate is reported in full, not as a bare "no"). The
thresholds are module constants quoting the pre-registration. They never change after a result
(rule 10 of the v0.2 briefs).
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
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
    evaluated: bool = (
        True  # False: the gate was not evaluated (a precondition failed), never a pass
    )

    @property
    def passed(self) -> bool:
        return self.evaluated and all(i.passed for i in self.items)

    def to_dict(self) -> dict[str, object]:
        out: dict[str, object] = {
            "gate": self.gate,
            "passed": self.passed,
            "items": [i.to_dict() for i in self.items],
        }
        if not self.evaluated:
            out["evaluated"] = False
        return out


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


# ---- GP2c, docs/notes/preregistration-pnp-v0.3.md section 2.3 ---------------------------------

# GP2c-Y item 1 "P1(Y) at most 1.00 times P1(v0.1) and the upper end of the bootstrap interval of
# the ratio below 1.00."
YAW_REFIT_P1_MAX_RATIO = 1.00
YAW_REFIT_P1_CI_HI_BELOW = 1.00
# item 2 "Y1(Y) at most 1.00 times Y1(v0.1)."
YAW_REFIT_Y1_MAX_RATIO = 1.00
# item 3 "Every stratum: position RMSE at 1.0 s at most 1.05 times dyn-v0.1's."
YAW_REFIT_STRATUM_MAX_RATIO = 1.05
# item 4 "Yp(Y) at most 1.00 times Yp(v0.1)."
YAW_REFIT_YP_MAX_RATIO = 1.00
# item 5 "S1(Y) equals S1(v0.1) within 1e-6 (the speed channel was not touched)."
S1_EQUAL_TOLERANCE_MPS = 1e-6
# item 6 "No refit parameter on a bound."
YAW_REFIT_MAX_ON_BOUND = 0

# GP2c-R item 1 "P1(R) at most 0.95 times P1(nominal) and the bootstrap upper end below 1.00."
YAW_RESIDUAL_P1_MAX_RATIO = 0.95
YAW_RESIDUAL_P1_CI_HI_BELOW = 1.00
# item 2 "Y1(R) at most 1.00 times Y1(nominal). S1(R) equals S1(nominal) within 1e-6."
YAW_RESIDUAL_Y1_MAX_RATIO = 1.00
# item 3 "Every seed: P1 at most 1.00 times P1(nominal)."
YAW_RESIDUAL_SEED_P1_MAX_RATIO = 1.00
# item 4 "Every stratum: position RMSE at 1.0 s at most 1.05 times the nominal's."
YAW_RESIDUAL_STRATUM_MAX_RATIO = 1.05
# item 6 "Yp(R) at most 1.00 times Yp(nominal)."
YAW_RESIDUAL_YP_MAX_RATIO = 1.00

YP_CANNOT_BE_APPLIED = "Yp cannot be applied: no usable /pose interval in G1b or T2b"


def _yp_item(name: str, candidate: float, nominal: float, limit: float) -> Item:
    if not (np.isfinite(candidate) and np.isfinite(nominal)):
        return Item(name, float("nan"), "<=", limit, False, YP_CANNOT_BE_APPLIED)
    return _item(name, candidate / nominal, "<=", limit)


def _s1_item(name: str, candidate: float, nominal: float) -> Item:
    diff = abs(candidate - nominal)
    return _item(name, diff, "<=", S1_EQUAL_TOLERANCE_MPS, "absolute difference in m/s")


def decide_yaw_refit(
    nominal: Mapping[str, float],
    candidate: Mapping[str, float],
    ratio_ci_hi: float,
    stratum_nominal_pos: Mapping[str, float],
    stratum_candidate_pos: Mapping[str, float],
    yp_nominal: float,
    yp_candidate: float,
    on_bound: Sequence[str],
) -> Decision:
    """GP2c-Y items 1 to 6 on the held-out segments under T2. `nominal` is dyn-v0.1 and
    `candidate` the yaw refit, each with the macro `P1`, `Y1`, `S1`; `ratio_ci_hi` is the upper
    end of the bootstrap interval of P1(Y) / P1(v0.1); `on_bound` lists the refit parameters that
    sit on a bound (unidentified)."""
    items = [
        _item(
            "1a P1 ratio Y / v0.1",
            candidate["P1"] / nominal["P1"],
            "<=",
            YAW_REFIT_P1_MAX_RATIO,
        ),
        _item(
            "1b upper end of the bootstrap interval of the P1 ratio",
            ratio_ci_hi,
            "<",
            YAW_REFIT_P1_CI_HI_BELOW,
        ),
        _item("2 Y1 ratio Y / v0.1", candidate["Y1"] / nominal["Y1"], "<=", YAW_REFIT_Y1_MAX_RATIO),
    ]
    for name, nom in stratum_nominal_pos.items():
        items.append(
            _item(
                f"3 stratum {name} position RMSE ratio",
                stratum_candidate_pos[name] / nom,
                "<=",
                YAW_REFIT_STRATUM_MAX_RATIO,
            )
        )
    items.append(_yp_item("4 Yp ratio Y / v0.1", yp_candidate, yp_nominal, YAW_REFIT_YP_MAX_RATIO))
    items.append(_s1_item("5 S1(Y) minus S1(v0.1)", candidate["S1"], nominal["S1"]))
    items.append(
        _item(
            "6 refit parameters on a bound",
            float(len(on_bound)),
            "<=",
            float(YAW_REFIT_MAX_ON_BOUND),
            ", ".join(on_bound) if on_bound else "none",
        )
    )
    return Decision("GP2c-Y yaw refit", tuple(items))


def decide_yaw_residual(
    nominal: Mapping[str, float],
    residual: Mapping[str, float],
    ratio_ci_hi: float,
    seed_p1: Mapping[int, float],
    stratum_nominal_pos: Mapping[str, float],
    stratum_residual_pos: Mapping[str, float],
    sweep: SweepResult,
    yp_nominal: float,
    yp_residual: float,
) -> Decision:
    """GP2c-R items 1 to 6 on the held-out segments under T2, against the shipping nominal."""
    items = [
        _item(
            "1a P1 ratio R / nominal",
            residual["P1"] / nominal["P1"],
            "<=",
            YAW_RESIDUAL_P1_MAX_RATIO,
        ),
        _item(
            "1b upper end of the bootstrap interval of the P1 ratio",
            ratio_ci_hi,
            "<",
            YAW_RESIDUAL_P1_CI_HI_BELOW,
        ),
        _item(
            "2a Y1 ratio R / nominal",
            residual["Y1"] / nominal["Y1"],
            "<=",
            YAW_RESIDUAL_Y1_MAX_RATIO,
        ),
        _s1_item("2b S1(R) minus S1(nominal)", residual["S1"], nominal["S1"]),
    ]
    for seed, p1 in sorted(seed_p1.items()):
        items.append(
            _item(
                f"3 seed {seed} P1 ratio to nominal",
                p1 / nominal["P1"],
                "<=",
                YAW_RESIDUAL_SEED_P1_MAX_RATIO,
            )
        )
    for name, nom in stratum_nominal_pos.items():
        items.append(
            _item(
                f"4 stratum {name} position RMSE ratio",
                stratum_residual_pos[name] / nom,
                "<=",
                YAW_RESIDUAL_STRATUM_MAX_RATIO,
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
    items.append(
        _yp_item("6 Yp ratio R / nominal", yp_residual, yp_nominal, YAW_RESIDUAL_YP_MAX_RATIO)
    )
    return Decision("GP2c-R yaw-only residual", tuple(items))


# ---- GP2d, docs/notes/preregistration-pnp-v0.4.md section 2.4 ---------------------------------
# Constants are quoted from the section; the yaml block `dynamics` of evaluation/prereg/
# pnp-v0.4.yaml holds the same numbers (`heading_outcome`, `fast_item`, `validity_cap_mps`) and a
# test reads the yaml and compares.

# GP2d-N item 1 "P1(N4) at most 1.00 times P1(v0.1) and the upper end of the bootstrap interval of
# the ratio below 1.00."
NOMINAL_P1_MAX_RATIO = 1.00
NOMINAL_P1_CI_HI_BELOW = 1.00
# item 2 "Y1(N4) at most 1.00 times Y1(v0.1) and S1(N4) at most 1.00 times S1(v0.1)."
NOMINAL_Y1_MAX_RATIO = 1.00
NOMINAL_S1_MAX_RATIO = 1.00
# item 3 "Every stratum: position RMSE at 1.0 s at most 1.05 times dyn-v0.1's."
NOMINAL_STRATUM_MAX_RATIO = 1.05
# item 4 "No refit parameter on a bound."
NOMINAL_MAX_ON_BOUND = 0
# item 5 "Yp(N4) at most 1.00 times Yp(v0.1) on G1b and T2b."
NOMINAL_YP_MAX_RATIO = 1.00


def decide_gp2d_nominal(
    v01: Mapping[str, float],
    n4: Mapping[str, float],
    ratio_ci_hi: float,
    stratum_v01_pos: Mapping[str, float],
    stratum_n4_pos: Mapping[str, float],
    on_bound: Sequence[str],
    yp_v01: float,
    yp_n4: float,
) -> Decision:
    """GP2d-N items 1 to 5 on the held-out segments under T2. `v01` and `n4` hold the macro `P1`,
    `Y1`, `S1`; `ratio_ci_hi` is the upper end of the bootstrap interval of P1(N4) / P1(v0.1)."""
    items = [
        _item("1a P1 ratio N4 / v0.1", n4["P1"] / v01["P1"], "<=", NOMINAL_P1_MAX_RATIO),
        _item(
            "1b upper end of the bootstrap interval of the P1 ratio",
            ratio_ci_hi,
            "<",
            NOMINAL_P1_CI_HI_BELOW,
        ),
        _item("2a Y1 ratio N4 / v0.1", n4["Y1"] / v01["Y1"], "<=", NOMINAL_Y1_MAX_RATIO),
        _item("2b S1 ratio N4 / v0.1", n4["S1"] / v01["S1"], "<=", NOMINAL_S1_MAX_RATIO),
    ]
    for name, nom in stratum_v01_pos.items():
        items.append(
            _item(
                f"3 stratum {name} position RMSE ratio",
                stratum_n4_pos[name] / nom,
                "<=",
                NOMINAL_STRATUM_MAX_RATIO,
            )
        )
    items.append(
        _item(
            "4 refit parameters on a bound",
            float(len(on_bound)),
            "<=",
            float(NOMINAL_MAX_ON_BOUND),
            ", ".join(on_bound) if on_bound else "none",
        )
    )
    items.append(_yp_item("5 Yp ratio N4 / v0.1", yp_n4, yp_v01, NOMINAL_YP_MAX_RATIO))
    return Decision("GP2d-N classical refit", tuple(items))


def decide_gp2d_residual(
    nominal: Mapping[str, float],
    residual: Mapping[str, float],
    ratio_ci_hi: float,
    seed_p1: Mapping[int, float],
    stratum_nominal_pos: Mapping[str, float],
    stratum_residual_pos: Mapping[str, float],
    sweep: SweepResult,
    yp_nominal: float,
    yp_residual: float,
) -> Decision:
    """GP2d-R: items 1 to 5 of GP2c-R with their thresholds unchanged (the thresholds are the
    GP2c-R constants above, there is no second copy) and Yp at most 1.00 times the nominal's, all
    against the shipping nominal."""
    inner = decide_yaw_residual(
        nominal,
        residual,
        ratio_ci_hi,
        seed_p1,
        stratum_nominal_pos,
        stratum_residual_pos,
        sweep,
        yp_nominal,
        yp_residual,
    )
    return Decision("GP2d-R yaw-only residual", inner.items)


# Named outcome (section 2.4), reported whether or not GP2d-R passes, NOT a deployment rule:
# "improves heading, not position" if Y1 ratio at most 0.80, the lateral position RMSE ratio at 1.0 s
# at most 0.90 with bootstrap upper end below 1.00, P1 ratio at most 1.00 and every stratum at most
# 1.05.
HEADING_Y1_MAX_RATIO = 0.80
HEADING_LATERAL_MAX_RATIO = 0.90
HEADING_LATERAL_CI_HI_BELOW = 1.00
HEADING_P1_MAX_RATIO = 1.00
HEADING_STRATUM_MAX_RATIO = 1.05
HEADING_LABEL = "improves heading, not position"
NO_NAMED_OUTCOME = "no named outcome"


@dataclass(frozen=True)
class HeadingOutcome:
    """A label and the items it rests on. It carries no deployment field on purpose: whether a
    model with this outcome may drive is the owner's decision (pre-registration v0.4 section 9)."""

    label: str
    items: tuple[Item, ...]

    def to_dict(self) -> dict[str, object]:
        return {"label": self.label, "items": [i.to_dict() for i in self.items]}


def heading_outcome(
    nominal: Mapping[str, float],
    candidate: Mapping[str, float],
    lateral_ratio: float,
    lateral_ci_hi: float,
    stratum_nominal_pos: Mapping[str, float],
    stratum_candidate_pos: Mapping[str, float],
) -> HeadingOutcome:
    """The named outcome of a candidate against its nominal on the held-out windows. `lateral_*` is
    the macro RMSE ratio of the across component of the position error at 1.0 s and the upper end
    of its bootstrap interval."""
    items = [
        _item("Y1 ratio", candidate["Y1"] / nominal["Y1"], "<=", HEADING_Y1_MAX_RATIO),
        _item(
            "lateral position RMSE ratio at 1.0 s", lateral_ratio, "<=", HEADING_LATERAL_MAX_RATIO
        ),
        _item(
            "upper end of the bootstrap interval of the lateral ratio",
            lateral_ci_hi,
            "<",
            HEADING_LATERAL_CI_HI_BELOW,
        ),
        _item("P1 ratio", candidate["P1"] / nominal["P1"], "<=", HEADING_P1_MAX_RATIO),
    ]
    for name, nom in stratum_nominal_pos.items():
        items.append(
            _item(
                f"stratum {name} position RMSE ratio",
                stratum_candidate_pos[name] / nom,
                "<=",
                HEADING_STRATUM_MAX_RATIO,
            )
        )
    label = HEADING_LABEL if all(i.passed for i in items) else NO_NAMED_OUTCOME
    return HeadingOutcome(label, tuple(items))


# ---- validity above 1.3 m/s (section 2.4) ------------------------------------------------------

# "may exceed dyn-v0.1's 1.3 only if the bundle's model passed its gate AND, on the held-out windows
# with a start speed of at least 1.15 m/s, in each direction with at least 30 such windows, its
# position RMSE at 1.0 s is at most 1.00 times dyn-v0.1's. A direction with fewer than 30 windows
# makes the item "cannot be applied" and the validity stays 1.3." Cap: yaml validity_cap_mps.
FAST_MIN_START_SPEED_MPS = 1.15
FAST_MIN_WINDOWS_PER_DIRECTION = 30
FAST_MAX_RATIO = 1.00
BASE_VALIDITY_MPS = 1.3
VALIDITY_CAP_MPS = 1.5
FAST_CANNOT_BE_APPLIED = "the fast item cannot be applied"


@dataclass(frozen=True)
class FastWindows:
    """Held-out windows with a start speed of at least FAST_MIN_START_SPEED_MPS in one direction:
    how many, and the position RMSE at 1.0 s of the bundle's model and of dyn-v0.1 on them."""

    direction: str
    windows: int
    model_p1: float
    v01_p1: float


@dataclass(frozen=True)
class ValidityDecision:
    v_max_mps: float
    rule_mps: float
    exceeds_base: bool  # the validity is above dyn-v0.1's 1.3
    applied: bool  # the fast item could be applied in both directions
    items: tuple[Item, ...]
    note: str

    def to_dict(self) -> dict[str, object]:
        return {
            "v_max_mps": self.v_max_mps,
            "rule_mps": self.rule_mps,
            "exceeds_base": self.exceeds_base,
            "applied": self.applied,
            "items": [i.to_dict() for i in self.items],
            "note": self.note,
        }


def decide_validity(
    rule_mps: float,
    gate_passed: bool,
    fast: Sequence[FastWindows],
    directions: Sequence[str] = ("ccw", "cw"),
) -> ValidityDecision:
    """`validity.v_max_mps` of a v0.4 bundle: `min(rule, 1.5)` if the model passed its gate and the
    fast item holds in every direction, otherwise `min(rule, 1.3)`. `rule_mps` is the pipeline's
    own validity rule on the new train data (nobody types a validity); a rule that is not a
    number gives no validity (NaN)."""
    rule = float(rule_mps)
    by_dir = {f.direction: f for f in fast}
    items: list[Item] = [
        Item("bundle model passed its gate", float(gate_passed), ">=", 1.0, bool(gate_passed))
    ]
    applied = True
    for d in directions:
        f = by_dir.get(d)
        n = 0 if f is None else f.windows
        if f is None or n < FAST_MIN_WINDOWS_PER_DIRECTION:
            applied = False
            items.append(
                Item(
                    f"fast {d}: windows with start speed >= {FAST_MIN_START_SPEED_MPS} m/s",
                    float(n),
                    ">=",
                    float(FAST_MIN_WINDOWS_PER_DIRECTION),
                    False,
                    FAST_CANNOT_BE_APPLIED,
                )
            )
            continue
        items.append(
            _item(
                f"fast {d}: P1 ratio to dyn-v0.1 ({n} windows)",
                f.model_p1 / f.v01_p1,
                "<=",
                FAST_MAX_RATIO,
            )
        )
    holds = all(i.passed for i in items)
    if not np.isfinite(rule):
        note, v_max = "the pipeline rule gave no number: no validity", float("nan")
    else:
        v_max = min(rule, VALIDITY_CAP_MPS if holds else BASE_VALIDITY_MPS)
        if not gate_passed:
            note = "the bundle's model failed its gate: validity held at min(rule, 1.3)"
        elif not applied:
            note = f"{FAST_CANNOT_BE_APPLIED}: validity held at min(rule, 1.3)"
        elif not holds:
            note = "the fast item failed: validity held at min(rule, 1.3)"
        else:
            note = "the fast item holds in both directions: validity min(rule, 1.5)"
    return ValidityDecision(
        v_max_mps=float(v_max),
        rule_mps=rule,
        exceeds_base=bool(np.isfinite(v_max) and v_max > BASE_VALIDITY_MPS),
        applied=applied,
        items=tuple(items),
        note=note,
    )


# ---- GP2e, docs/notes/preregistration-pnp-v0.4-addendum-1.md section 2.6 ----------------------
# Constants quoted from the section; the yaml blocks `gate_table` and `gate_learned` hold the same
# numbers and a test compares them.

# GP2e-T item 1 "P1(T4) at most 0.95 times P1(nominal) under T2, bootstrap upper end of the ratio
# below 1.00."
TABLE_P1_MAX_RATIO = 0.95
TABLE_P1_CI_HI_BELOW = 1.00
# item 2 "A1(T4) at most 1.00 times A1(nominal) and C1(T4) at most 1.00 times C1(nominal)."
TABLE_ALONG_MAX_RATIO = 1.00
TABLE_CROSS_MAX_RATIO = 1.00
# item 3 "Y1 and S1 equal to the nominal's within 1e-6."
CHANNEL_EQUAL_TOLERANCE = 1.0e-6
# item 4 "Every stratum: position RMSE at 1.0 s at most 1.05 times the nominal's."
TABLE_STRATUM_MAX_RATIO = 1.05
# item 5 "No supported table node on a bound."
TABLE_MAX_ON_BOUND = 0
# item 6 "Under T2s on the same windows: P1(T4) at most 1.00 times P1(nominal)."
TABLE_ALT_TRUTH_MAX_RATIO = 1.00
# item 7 "Pp(T4) at most 1.00 times Pp(nominal) on G1b and T2b."
TABLE_PP_MAX_RATIO = 1.00

# GP2e-L item 1 "P1(TL4) at most 0.95 times P1(T4) under T2, bootstrap upper end below 1.00."
LEARNED_P1_MAX_RATIO = 0.95
LEARNED_P1_CI_HI_BELOW = 1.00
# item 2 "Y1(TL4) at most 1.00 times Y1(T4). S1 equal within 1e-6."
LEARNED_Y1_MAX_RATIO = 1.00
# item 3 "Every seed: P1 at most 1.00 times P1(T4)."
LEARNED_SEED_P1_MAX_RATIO = 1.00
# item 4 "Every stratum at most 1.05 times T4's."
LEARNED_STRATUM_MAX_RATIO = 1.05
# item 6 "Under T2s: P1(TL4) at most 1.00 times P1(T4)."
LEARNED_ALT_TRUTH_MAX_RATIO = 1.00
# item 7 "Pp and Yp at most 1.00 times T4's on G1b and T2b."
LEARNED_PP_MAX_RATIO = 1.00
LEARNED_YP_MAX_RATIO = 1.00

PP_CANNOT_BE_APPLIED = "Pp cannot be applied: no usable /pose pair in G1b or T2b"
TABLE_WINS = "the table wins; the learned correction is not used"
ONE_SPEED_STAYS = "the one-speed contract stays"
TL4_CANDIDATE = "TL4 is the candidate"


def _pp_item(name: str, candidate: float, nominal: float, limit: float) -> Item:
    if not (np.isfinite(candidate) and np.isfinite(nominal)):
        return Item(name, float("nan"), "<=", limit, False, PP_CANNOT_BE_APPLIED)
    return _item(name, candidate / nominal, "<=", limit)


def _equal_item(name: str, candidate: float, nominal: float, note: str) -> Item:
    return _item(name, abs(candidate - nominal), "<=", CHANNEL_EQUAL_TOLERANCE, note)


def decide_gp2e_table(
    nominal: Mapping[str, float],
    table: Mapping[str, float],
    ratio_ci_hi: float,
    stratum_nominal_pos: Mapping[str, float],
    stratum_table_pos: Mapping[str, float],
    on_bound: Sequence[str],
    p1_alt_nominal: float,
    p1_alt_table: float,
    pp_nominal: float,
    pp_table: float,
) -> Decision:
    """GP2e-T items 1 to 7 on the held-out segments. `nominal` and `table` hold the macro `P1`,
    `A1`, `C1`, `Y1` and `S1`; `p1_alt_*` is P1 under T2s on the same windows."""
    items = [
        _item("1a P1 ratio T4 / nominal", table["P1"] / nominal["P1"], "<=", TABLE_P1_MAX_RATIO),
        _item(
            "1b upper end of the bootstrap interval of the P1 ratio",
            ratio_ci_hi,
            "<",
            TABLE_P1_CI_HI_BELOW,
        ),
        _item("2a A1 ratio T4 / nominal", table["A1"] / nominal["A1"], "<=", TABLE_ALONG_MAX_RATIO),
        _item("2b C1 ratio T4 / nominal", table["C1"] / nominal["C1"], "<=", TABLE_CROSS_MAX_RATIO),
        _equal_item("3a Y1(T4) minus Y1(nominal)", table["Y1"], nominal["Y1"], "absolute rad"),
        _equal_item("3b S1(T4) minus S1(nominal)", table["S1"], nominal["S1"], "absolute m/s"),
    ]
    for name, nom in stratum_nominal_pos.items():
        items.append(
            _item(
                f"4 stratum {name} position RMSE ratio",
                stratum_table_pos[name] / nom,
                "<=",
                TABLE_STRATUM_MAX_RATIO,
            )
        )
    items.append(
        _item(
            "5 supported table nodes on a bound",
            float(len(on_bound)),
            "<=",
            float(TABLE_MAX_ON_BOUND),
            ", ".join(on_bound) if on_bound else "none",
        )
    )
    items.append(
        _item(
            "6 P1 ratio T4 / nominal under T2s",
            p1_alt_table / p1_alt_nominal,
            "<=",
            TABLE_ALT_TRUTH_MAX_RATIO,
        )
    )
    items.append(_pp_item("7 Pp ratio T4 / nominal", pp_table, pp_nominal, TABLE_PP_MAX_RATIO))
    return Decision("GP2e-T ground motion table", tuple(items))


def decide_gp2e_learned(
    table_passed: bool,
    t4: Mapping[str, float] | None = None,
    tl4: Mapping[str, float] | None = None,
    ratio_ci_hi: float = float("nan"),
    seed_p1: Mapping[int, float] | None = None,
    stratum_t4_pos: Mapping[str, float] | None = None,
    stratum_tl4_pos: Mapping[str, float] | None = None,
    sweep=None,
    p1_alt_t4: float = float("nan"),
    p1_alt_tl4: float = float("nan"),
    pp_t4: float = float("nan"),
    pp_tl4: float = float("nan"),
    yp_t4: float = float("nan"),
    yp_tl4: float = float("nan"),
) -> Decision:
    """GP2e-L items 1 to 7 against T4 on the same base. Evaluated only if GP2e-T passed: with
    `table_passed` False the result is a Decision with `evaluated` False (never a pass) and no
    item, and every other argument is ignored (so none has to be computed)."""
    if not table_passed:
        return Decision("GP2e-L learned correction (not evaluated: GP2e-T failed)", (), False)
    assert t4 is not None and tl4 is not None and seed_p1 is not None and sweep is not None
    assert stratum_t4_pos is not None and stratum_tl4_pos is not None
    items = [
        _item("1a P1 ratio TL4 / T4", tl4["P1"] / t4["P1"], "<=", LEARNED_P1_MAX_RATIO),
        _item(
            "1b upper end of the bootstrap interval of the P1 ratio",
            ratio_ci_hi,
            "<",
            LEARNED_P1_CI_HI_BELOW,
        ),
        _item("2a Y1 ratio TL4 / T4", tl4["Y1"] / t4["Y1"], "<=", LEARNED_Y1_MAX_RATIO),
        _equal_item("2b S1(TL4) minus S1(T4)", tl4["S1"], t4["S1"], "absolute m/s"),
    ]
    for seed, p1 in sorted(seed_p1.items()):
        items.append(
            _item(f"3 seed {seed} P1 ratio to T4", p1 / t4["P1"], "<=", LEARNED_SEED_P1_MAX_RATIO)
        )
    for name, ref in stratum_t4_pos.items():
        items.append(
            _item(
                f"4 stratum {name} position RMSE ratio",
                stratum_tl4_pos[name] / ref,
                "<=",
                LEARNED_STRATUM_MAX_RATIO,
            )
        )
    n_bad = (
        sweep.nonfinite
        + sweep.alpha_violations
        + sweep.rho_violations
        + sweep.zero_speed_violations
    )
    items.append(
        Item(
            "5 random sweep of the validity box: finite, alpha and rho in bounds, rest is rest",
            float(n_bad),
            "<=",
            0.0,
            bool(sweep.passed),
            f"{sweep.samples} samples",
        )
    )
    items.append(
        _item(
            "6 P1 ratio TL4 / T4 under T2s",
            p1_alt_tl4 / p1_alt_t4,
            "<=",
            LEARNED_ALT_TRUTH_MAX_RATIO,
        )
    )
    items.append(_pp_item("7a Pp ratio TL4 / T4", pp_tl4, pp_t4, LEARNED_PP_MAX_RATIO))
    items.append(_yp_item("7b Yp ratio TL4 / T4", yp_tl4, yp_t4, LEARNED_YP_MAX_RATIO))
    return Decision("GP2e-L learned correction", tuple(items))


def gp2e_outcome(table: Decision, learned: Decision) -> str:
    """The outcome name of addendum 1 section 2.6. If GP2e-T fails TL4 is printed and decides
    nothing; if it passes and GP2e-L fails or was not evaluated the table wins."""
    if not table.passed:
        return ONE_SPEED_STAYS
    if learned.passed:
        return TL4_CANDIDATE
    return TABLE_WINS
