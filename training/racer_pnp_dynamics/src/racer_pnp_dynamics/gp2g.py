"""GP2g-R of brief 54 (`docs/notes/preregistration-pnp-v0.6-dynamics-yaw.md`): candidate R6, the
yaw-only residual on dyn-v0.1, against dyn-v0.1 with the residual off, on the held-out parts of
the 2026-11-22 runs under T2.

Every threshold is READ from the registered yaml (`ConstantsV06.gate`, block `gate_residual`, and
`metrics`); nothing is typed in. A test compares the unchanged ones with the GP2c-R / GP2d-R
constants of `gp2`. Every item is returned with its value, threshold and result. Item 0 (enough
data) not met gives INSUFFICIENT, which is not a pass. An item whose minimum window count is not
met is "cannot be applied" (`gp2f._na`): reported, never a failure.

Nothing here fits or tunes anything; the functions receive numbers computed on frozen models.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np

from . import gp2, gp2f
from .gp2 import Decision, Item, SweepResult
from .prereg_v06 import ConstantsV06
from .strata import Strata

GATE = "GP2g-R yaw-only residual R6"
INSUFFICIENT = "INSUFFICIENT"
PASS = "PASS"
FAIL = "FAIL"


@dataclass(frozen=True)
class SolveCost:
    """Item 12: `mppi_bench` p50 solve time (ms) with R6 and with dyn-v0.1."""

    candidate_p50_ms: float
    nominal_p50_ms: float
    note: str = ""


@dataclass(frozen=True)
class Canary:
    """Item 13: the L5 PnP canary run with the R6 bundle as the planner's model."""

    cases: int
    completed: int
    wall_contacts: int
    note: str = ""


@dataclass(frozen=True)
class GateResult:
    decision: Decision
    outcome: str  # PASS, FAIL or INSUFFICIENT
    enough_data: Decision

    def to_dict(self) -> dict[str, object]:
        return {
            "outcome": self.outcome,
            "enough_data": self.enough_data.to_dict(),
            "decision": self.decision.to_dict(),
        }


def enough_data(c: ConstantsV06, direction: np.ndarray, strata: Strata) -> Decision:
    """Item 0: at least 400 windows, 150 per direction, 4 strata with 30 windows."""
    g = c.gate["item_0_enough_data"]
    per_stratum = int(c.metrics["min_windows_per_stratum"])
    direction = np.asarray(direction, dtype=str)
    items = [
        gp2f._ge("0a windows", float(len(direction)), float(g["min_windows_total"])),
        *(
            gp2f._ge(
                f"0b windows {d}",
                float(np.sum(direction == d)),
                float(g["min_windows_per_direction"]),
            )
            for d in gp2f.DIRECTIONS
        ),
        gp2f._ge(
            f"0c strata with {per_stratum} windows",
            float(sum(1 for n in strata.names if strata.counts[n] >= per_stratum)),
            float(g["min_strata_with_30_windows"]),
        ),
    ]
    return Decision("GP2g-R item 0 enough data", tuple(items))


def _stratum_items(
    c: ConstantsV06, strata: Strata, ref: Mapping[str, float], cand: Mapping[str, float]
) -> list[Item]:
    minimum = int(c.metrics["min_windows_per_stratum"])
    out = []
    for name in strata.names:
        n = strata.counts[name]
        label = f"4 stratum {name} position RMSE ratio"
        if n < minimum:
            out.append(gp2f._na(label, n, minimum, "windows"))
        else:
            out.append(
                gp2f._ratio(
                    f"{label} ({n} windows)",
                    cand[name],
                    ref[name],
                    float(c.gate["max_stratum_ratio"]),
                )
            )
    return out


def _windowed(
    name: str, cand: tuple[int, float], ref: tuple[int, float], minimum: int, limit: float
):
    return gp2f._windowed(name, cand, ref, minimum, limit)


def decide_gp2g_r(
    c: ConstantsV06,
    direction: np.ndarray,
    strata: Strata,
    ref: gp2f.Metrics,
    cand: gp2f.Metrics,
    ratio_ci_hi: float,
    seed_p1: Mapping[int, float],
    sweep: SweepResult,
    saturation_share: float,
    solve: SolveCost,
    canary: Canary,
) -> GateResult:
    """GP2g-R items 0 to 13. `ref` is dyn-v0.1 and `cand` the shipping R6, both on the identical
    held-out windows (`direction` and `strata` are those windows'); `ratio_ci_hi` the upper end of
    the bootstrap interval of the P1 ratio; `seed_p1` P1 of every seed; `saturation_share` the
    share of held-out steps whose residual output is at or above the yaml level of its bound."""
    g = c.gate
    m = c.metrics
    zero = enough_data(c, direction, strata)
    items: list[Item] = [
        gp2f._ratio(
            "1a P1 ratio R6 / dyn-v0.1", cand.macro["P1"], ref.macro["P1"], float(g["max_p1_ratio"])
        ),
        gp2._item(
            "1b upper end of the bootstrap interval of the P1 ratio",
            ratio_ci_hi,
            "<",
            float(g["max_p1_ratio_bootstrap_upper"]),
        ),
        gp2f._ratio("2a Y1 ratio", cand.macro["Y1"], ref.macro["Y1"], float(g["max_y1_ratio"])),
        gp2f._equal(
            "2b S1(R6) minus S1(dyn-v0.1)",
            cand.macro["S1"],
            ref.macro["S1"],
            float(g["speed_equal_tolerance"]),
            "absolute difference in m/s",
        ),
    ]
    for seed, p1 in sorted(seed_p1.items()):
        items.append(
            gp2f._ratio(
                f"3 seed {seed} P1 ratio", p1, ref.macro["P1"], float(g["max_seed_p1_ratio"])
            )
        )
    items += _stratum_items(c, strata, ref.stratum_pos, cand.stratum_pos)
    items.append(
        Item(
            "5 random sweep of the validity box: finite, inside the state bounds",
            float(sweep.nonfinite + sweep.steering_violations + sweep.speed_violations),
            "<=",
            0.0,
            sweep.passed and sweep.samples >= int(g["sweep_samples"]),
            f"{sweep.samples} rollouts of {sweep.steps} steps; max v {sweep.max_speed_mps:.3f} m/s",
        )
    )
    items.append(
        gp2._yp_item(
            "6 Yp ratio (G1b, T2b; a re-read)", cand.yp, ref.yp, float(g["max_pose_yaw_ratio"])
        )
    )
    pd = g["per_direction"]
    for d in gp2f.DIRECTIONS:
        cd = cand.direction.get(d, {"P1": float("nan"), "Y1": float("nan")})
        rd = ref.direction.get(d, {"P1": float("nan"), "Y1": float("nan")})
        items.append(gp2f._ratio(f"7 {d} P1 ratio", cd["P1"], rd["P1"], float(pd["max_p1_ratio"])))
        items.append(gp2f._ratio(f"7 {d} Y1 ratio", cd["Y1"], rd["Y1"], float(pd["max_y1_ratio"])))
    items.append(
        gp2f._ratio("8a C1 ratio", cand.macro["C1"], ref.macro["C1"], float(g["max_cross_ratio"]))
    )
    items.append(
        gp2f._ratio("8b A1 ratio", cand.macro["A1"], ref.macro["A1"], float(g["max_along_ratio"]))
    )
    n_side = int(m["near_limit"]["min_windows_per_side"])
    for side in gp2f.SIDES:
        items.append(
            _windowed(
                f"9 near-limit {side} position RMSE ratio",
                cand.near_limit.get(side, (0, float("nan"))),
                ref.near_limit.get(side, (0, float("nan"))),
                n_side,
                float(g["max_near_limit_ratio"]),
            )
        )
    items.append(
        _windowed(
            "9 transition position RMSE ratio",
            cand.transition,
            ref.transition,
            int(m["transition"]["min_windows"]),
            float(g["max_transition_ratio"]),
        )
    )
    items.append(
        gp2f._ratio(
            "10 P1 ratio under T2s", cand.p1_alt, ref.p1_alt, float(g["max_p1_ratio_alt_truth"])
        )
    )
    sat = g["saturation"]
    items.append(
        gp2._item(
            f"11 share of held-out steps with the residual output at or above {sat['level']:g} of its bound",
            saturation_share,
            "<=",
            float(sat["max_heldout_step_share"]),
        )
    )
    items.append(
        gp2f._ratio(
            "12 mppi_bench p50 solve time ratio R6 / dyn-v0.1",
            solve.candidate_p50_ms,
            solve.nominal_p50_ms,
            float(g["solve_cost"]["max_p50_ratio_to_nominal"]),
            f"{solve.candidate_p50_ms:.3f} ms against {solve.nominal_p50_ms:.3f} ms. {solve.note}".strip(),
        )
    )
    cn = g["canary"]
    ok = (
        canary.cases > 0
        and canary.completed == canary.cases
        and canary.wall_contacts <= int(cn["contacts_allowed"])
    )
    items.append(
        Item(
            "13 L5 PnP canary with the R6 bundle: every case completes, 0 wall contacts",
            float(canary.wall_contacts + (canary.cases - canary.completed)),
            "<=",
            float(cn["contacts_allowed"]),
            bool(ok),
            f"{canary.completed} of {canary.cases} cases completed, {canary.wall_contacts} wall contacts. "
            f"{canary.note}".strip(),
        )
    )
    if not zero.passed:
        decision = Decision(GATE, tuple(items), evaluated=False)
        return GateResult(decision, INSUFFICIENT, zero)
    decision = Decision(GATE, tuple(items))
    return GateResult(decision, PASS if decision.passed else FAIL, zero)


def heading_outcome(
    c: ConstantsV06,
    ref: gp2f.Metrics,
    cand: gp2f.Metrics,
    lateral_ratio: float,
    lateral_ci_hi: float,
) -> gp2.HeadingOutcome:
    """The named outcome "improves heading, not position" (v0.4 section 2.4, unchanged; the yaml
    repeats its numbers and a test checks they are gp2's). Not a deployment rule."""
    return gp2.heading_outcome(
        dict(ref.macro),
        dict(cand.macro),
        lateral_ratio,
        lateral_ci_hi,
        ref.stratum_pos,
        cand.stratum_pos,
    )


def saturation_share(outputs: np.ndarray, bound: np.ndarray, level: float) -> float:
    """Share of steps (rows of `outputs`, (K, 2)) whose residual output is at or above `level`
    times its bound on any channel with a positive bound (the speed channel of a yaw-only residual
    has bound 0 and output 0 and is not counted)."""
    outputs = np.abs(np.asarray(outputs, dtype=float))
    bound = np.asarray(bound, dtype=float)
    live = bound > 0.0
    if not live.any() or len(outputs) == 0:
        return 0.0
    return float(np.mean((outputs[:, live] >= level * bound[live]).any(axis=1)))


def output_bound(residual) -> np.ndarray:
    """The bound of a residual's output per channel, implied by its weights: every hidden unit is a
    tanh, so |out_i| <= output_scale_i * (sum_j |w3_ij| + |b3_i|). It is at most the training cap
    of a capped config, so it is the tighter of the two."""
    w = residual.weights
    return np.abs(residual.output_scale) * (np.abs(w.w3).sum(axis=1) + np.abs(w.b3))
