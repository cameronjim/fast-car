"""GP2f decisions of docs/notes/preregistration-pnp-v0.5-dynamics.md section 6, as functions, and the
metrics of the one held-out pass that feed them (brief 47 step 6).

Every item of a decision is returned with its value, its threshold and whether it passed, so the
report shows all of them (a failed gate is reported in full, not as a bare "no"). Thresholds are
READ from the registered yaml (`ConstantsV05.doc`, blocks `gate_support`, `gate_table`,
`gate_learned`, `gate_validity`, `metrics`); nothing is typed in. An item whose minimum window count
is not met is "cannot be applied": it is printed with its count (comparison `n/a`) and does not fail
the gate, except in the validity decision where a missing item keeps the validity at 1.3.

Nothing here fits or tunes anything; the functions receive numbers computed on frozen models.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field

import numpy as np

from . import gp2
from .gp2 import Decision, Item, ValidityDecision
from .ground_motion_v21 import ConstantsV05

NA = "n/a"  # the comparison of an item that cannot be applied
DIRECTIONS = ("ccw", "cw")
SIDES = ("left", "right")
GATE_NOT_APPLIED = "cannot be applied"


def _ge(name: str, value: float, threshold: float, note: str = "") -> Item:
    ok = bool(np.isfinite(value) and value >= threshold)
    return Item(name, float(value), ">=", threshold, ok, note)


def _na(name: str, count: int, minimum: int, what: str) -> Item:
    """An item whose minimum count is not met: reported, never a failure."""
    return Item(
        name,
        float(count),
        NA,
        float(minimum),
        True,
        f"{GATE_NOT_APPLIED}: {count} {what}, at least {minimum} needed",
    )


def _equal(name: str, a: float, b: float, tol: float, note: str) -> Item:
    return gp2._item(name, abs(a - b), "<=", tol, note)


@dataclass(frozen=True)
class Metrics:
    """The numbers of one model on the pass windows (all under T2 unless the name says T2s)."""

    macro: Mapping[str, float]  # P1, A1, C1, Y1, S1
    stratum_pos: Mapping[str, float]
    direction: Mapping[str, Mapping[str, float]]  # direction -> {P1, Y1}
    yaw_bias: Mapping[str, float]  # direction -> mean (model - truth) / 1.0 s, rad/s
    near_limit: Mapping[str, tuple[int, float]]  # side -> (windows, position RMSE at 1.0 s)
    transition: tuple[int, float]  # (windows, position RMSE at 1.0 s)
    p1_alt: float  # P1 under T2s
    pp: float
    yp: float
    fast: Mapping[float, Mapping[str, tuple[int, float]]] = field(default_factory=dict)
    # fast[min start speed][direction] = (windows, position RMSE at 1.0 s)


# ---- GP2f-S ------------------------------------------------------------------------------------


def decide_gp2f_support(
    c: ConstantsV05,
    on_bound: Sequence[str],
    split_half_alpha_max: float,
    split_half_gamma_max: float,
    step_share_in_supported: float,
) -> Decision:
    """GP2f-S: the tables may be trusted where they are used. `on_bound` lists the supported nodes
    of either table within `bound_margin` of a bound."""
    g = c.doc["gate_support"]
    return Decision(
        "GP2f-S the tables may be trusted",
        (
            gp2._item(
                "1 supported nodes within 0.005 of a bound (alpha and gamma)",
                float(len(on_bound)),
                "<=",
                float(g["max_supported_nodes_on_bound"]),
                ", ".join(on_bound) if on_bound else "none",
            ),
            gp2._item(
                "2a split half: largest alpha difference at a supported node",
                split_half_alpha_max,
                "<=",
                float(g["max_split_half_difference_alpha"]),
            ),
            gp2._item(
                "2b split half: largest gamma difference at a supported node",
                split_half_gamma_max,
                "<=",
                float(g["max_split_half_difference_gamma"]),
            ),
            _ge(
                "3 share of held-out rollout steps in the cell of a supported node",
                step_share_in_supported,
                float(g["min_heldout_step_share_in_supported_cells"]),
            ),
        ),
    )


# ---- GP2f-T ------------------------------------------------------------------------------------


def _ratio(name: str, cand: float, ref: float, limit: float, note: str = "") -> Item:
    return gp2._item(name, cand / ref if ref else float("nan"), "<=", limit, note)


def _windowed(
    name: str, cand: tuple[int, float], ref: tuple[int, float], minimum: int, limit: float
) -> Item:
    n = cand[0]
    if n < minimum:
        return _na(name, n, minimum, "windows")
    return _ratio(f"{name} ({n} windows)", cand[1], ref[1], limit)


def _near_limit_items(
    c: ConstantsV05, cand: Metrics, ref: Metrics, limit: float, label: str
) -> list[Item]:
    minimum = int(c.doc["metrics"]["near_limit"]["min_windows_per_side"])
    return [
        _windowed(
            f"{label} near-limit {side} position RMSE ratio",
            cand.near_limit.get(side, (0, float("nan"))),
            ref.near_limit.get(side, (0, float("nan"))),
            minimum,
            limit,
        )
        for side in SIDES
    ]


def _transition_item(
    c: ConstantsV05, cand: Metrics, ref: Metrics, limit: float, label: str
) -> Item:
    minimum = int(c.doc["metrics"]["transition"]["min_windows"])
    return _windowed(
        f"{label} transition position RMSE ratio", cand.transition, ref.transition, minimum, limit
    )


def decide_gp2f_table(
    c: ConstantsV05,
    support_passed: bool,
    ref: Metrics,
    cand: Metrics,
    ratio_ci_hi: float,
) -> Decision:
    """GP2f-T items 1 to 10, T5 (`cand`) against dyn-v0.1 (`ref`) on identical held-out windows."""
    g = c.doc["gate_table"]
    items: list[Item] = [
        Item("0 GP2f-S passed", float(support_passed), ">=", 1.0, bool(support_passed)),
        _ratio(
            "1a P1 ratio T5 / dyn-v0.1", cand.macro["P1"], ref.macro["P1"], float(g["max_p1_ratio"])
        ),
        gp2._item(
            "1b upper end of the bootstrap interval of the P1 ratio",
            ratio_ci_hi,
            "<",
            float(g["max_p1_ratio_bootstrap_upper"]),
        ),
        _ratio("2a A1 ratio", cand.macro["A1"], ref.macro["A1"], float(g["max_along_ratio"])),
        _ratio("2b C1 ratio", cand.macro["C1"], ref.macro["C1"], float(g["max_cross_ratio"])),
        _ratio("3a Y1 ratio", cand.macro["Y1"], ref.macro["Y1"], float(g["max_y1_ratio"])),
        _equal(
            "3b S1(T5) minus S1(dyn-v0.1)",
            cand.macro["S1"],
            ref.macro["S1"],
            float(g["speed_equal_tolerance"]),
            "absolute m/s",
        ),
    ]
    for name, r in ref.stratum_pos.items():
        items.append(
            _ratio(
                f"4 stratum {name} position RMSE ratio",
                cand.stratum_pos.get(name, float("nan")),
                r,
                float(g["max_stratum_ratio"]),
            )
        )
    pd = g["per_direction"]
    for d in DIRECTIONS:
        cd, rd = cand.direction.get(d, {}), ref.direction.get(d, {})
        items.append(
            _ratio(
                f"5 {d} P1 ratio",
                cd.get("P1", float("nan")),
                rd.get("P1", float("nan")),
                float(pd["max_p1_ratio"]),
            )
        )
        items.append(
            _ratio(
                f"5 {d} Y1 ratio",
                cd.get("Y1", float("nan")),
                rd.get("Y1", float("nan")),
                float(pd["max_y1_ratio"]),
            )
        )
    yb = g["yaw_bias"]
    for d in DIRECTIONS:
        b, b_ref = cand.yaw_bias.get(d, float("nan")), ref.yaw_bias.get(d, float("nan"))
        limit = max(
            float(yb["max_abs_radps"]), float(yb["or_max_share_of_nominal_abs_bias"]) * abs(b_ref)
        )
        note = (
            f"at most {yb['max_abs_radps']} rad/s or {yb['or_max_share_of_nominal_abs_bias']} of "
            f"the dyn-v0.1 bias {b_ref:+.4f} rad/s"
        )
        items.append(gp2._item(f"6 {d} absolute yaw rate bias (rad/s)", abs(b), "<=", limit, note))
    items += _near_limit_items(c, cand, ref, float(g["max_near_limit_ratio"]), "7")
    items.append(_transition_item(c, cand, ref, float(g["max_transition_ratio"]), "8"))
    items.append(
        _ratio("9 P1 ratio under T2s", cand.p1_alt, ref.p1_alt, float(g["max_p1_ratio_alt_truth"]))
    )
    items.append(
        gp2._pp_item(
            "10a Pp ratio on G1b and T2b", cand.pp, ref.pp, float(g["max_pose_position_ratio"])
        )
    )
    items.append(
        gp2._yp_item("10b Yp ratio on G1b and T2b", cand.yp, ref.yp, float(g["max_pose_yaw_ratio"]))
    )
    return Decision("GP2f-T T5 is a candidate", tuple(items))


# ---- GP2f-L ------------------------------------------------------------------------------------


def decide_gp2f_learned(
    c: ConstantsV05,
    table_passed: bool,
    ref: Metrics | None = None,
    cand: Metrics | None = None,
    ratio_ci_hi: float = float("nan"),
    seed_p1: Mapping[int, float] | None = None,
    sweep_violations: int | None = None,
    sweep_samples: int = 0,
    saturation_share: float = float("nan"),
) -> Decision:
    """GP2f-L items 1 to 9, TL5 (`cand`) against T5 (`ref`). Evaluated only if GP2f-T passed: with
    `table_passed` False the result has `evaluated` False (never a pass) and no item, and every
    other argument is ignored (so none has to be computed)."""
    if not table_passed:
        return Decision("GP2f-L TL5 replaces T5 (not evaluated: GP2f-T failed)", (), False)
    assert ref is not None and cand is not None and seed_p1 is not None
    g = c.doc["gate_learned"]
    items: list[Item] = [
        _ratio("1a P1 ratio TL5 / T5", cand.macro["P1"], ref.macro["P1"], float(g["max_p1_ratio"])),
        gp2._item(
            "1b upper end of the bootstrap interval of the P1 ratio",
            ratio_ci_hi,
            "<",
            float(g["max_p1_ratio_bootstrap_upper"]),
        ),
        _ratio("2a Y1 ratio TL5 / T5", cand.macro["Y1"], ref.macro["Y1"], float(g["max_y1_ratio"])),
        _equal(
            "2b S1(TL5) minus S1(T5)",
            cand.macro["S1"],
            ref.macro["S1"],
            float(g["speed_equal_tolerance"]),
            "absolute m/s",
        ),
    ]
    for seed, p1 in sorted(seed_p1.items()):
        items.append(
            _ratio(
                f"3 seed {seed} P1 ratio to T5", p1, ref.macro["P1"], float(g["max_seed_p1_ratio"])
            )
        )
    for name, r in ref.stratum_pos.items():
        items.append(
            _ratio(
                f"4 stratum {name} position RMSE ratio",
                cand.stratum_pos.get(name, float("nan")),
                r,
                float(g["max_stratum_ratio"]),
            )
        )
    for d in DIRECTIONS:
        items.append(
            _ratio(
                f"4 {d} P1 ratio",
                cand.direction.get(d, {}).get("P1", float("nan")),
                ref.direction.get(d, {}).get("P1", float("nan")),
                float(g["per_direction"]["max_p1_ratio"]),
            )
        )
    items += _near_limit_items(c, cand, ref, float(g["max_near_limit_ratio"]), "5")
    items.append(_transition_item(c, cand, ref, float(g["max_transition_ratio"]), "5"))
    if sweep_violations is None:
        items.append(
            Item("6 random sweep of the validity box", float("nan"), "<=", 0.0, False, "not run")
        )
    else:
        items.append(
            gp2._item(
                "6 random sweep of the validity box: finite, alpha and rho in bounds, rest is rest",
                float(sweep_violations),
                "<=",
                0.0,
                f"{sweep_samples} samples",
            )
        )
    sat = g["saturation"]
    items.append(
        gp2._item(
            f"7 held-out steps with the network beyond {sat['level']} of a limit",
            saturation_share,
            "<=",
            float(sat["max_heldout_step_share"]),
        )
    )
    items.append(
        _ratio("8 P1 ratio under T2s", cand.p1_alt, ref.p1_alt, float(g["max_p1_ratio_alt_truth"]))
    )
    items.append(
        gp2._pp_item(
            "9a Pp ratio on G1b and T2b", cand.pp, ref.pp, float(g["max_pose_position_ratio"])
        )
    )
    items.append(
        gp2._yp_item("9b Yp ratio on G1b and T2b", cand.yp, ref.yp, float(g["max_pose_yaw_ratio"]))
    )
    return Decision("GP2f-L TL5 replaces T5", tuple(items))


# ---- GP2f-V ------------------------------------------------------------------------------------


def top_supported_speed(
    c: ConstantsV05, supported_alpha: np.ndarray, supported_gamma: np.ndarray
) -> float | None:
    """The highest speed node such that it and every lower node from 0.6 m/s up have, on each side
    of the steering axis, a supported node at share 0.5 or 1.0 (in absolute value) in BOTH tables.
    None when not even the 0.6 m/s node qualifies."""
    shares = np.array(c.steering_nodes)
    speeds = np.array(c.speed_nodes_mps)
    turning = [float(x) for x in c.doc["gate_validity"]["turning_nodes_abs_share"]]
    top: float | None = None
    for j, v in enumerate(speeds):
        if v < 0.6 - 1e-9:
            continue
        ok = True
        for table in (supported_alpha, supported_gamma):
            for sign in (-1.0, 1.0):
                nodes = [
                    i
                    for i, s in enumerate(shares)
                    if np.sign(s) == sign and any(abs(abs(s) - t) < 1e-9 for t in turning)
                ]
                ok &= any(bool(table[i, j]) for i in nodes)
        if not ok:
            break
        top = float(v)
    return top


def decide_gp2f_validity(
    c: ConstantsV05,
    gate_passed: bool,
    rule_mps: float,
    top_supported_mps: float | None,
    fast: Mapping[float, Mapping[str, tuple[int, float, float]]],
) -> ValidityDecision:
    """`v_max_mps` of a v0.5 bundle: `default_mps` (1.3) unless every item holds, and then the
    smallest of the cap (1.5), `rule_mps` (the pipeline's validity rule on train) and the top
    supported speed. `fast[min start speed][direction]` = (windows, model position RMSE at 1.0 s,
    dyn-v0.1 position RMSE). A direction with too few windows makes its item "cannot be applied";
    a missing item keeps the validity at the default."""
    g = c.doc["gate_validity"]
    cap, default = float(g["cap_mps"]), float(g["default_mps"])
    items: list[Item] = [
        Item(
            "1 the bundle's model passed its gate", float(gate_passed), ">=", 1.0, bool(gate_passed)
        )
    ]
    applied = True
    for k, spec in enumerate(g["fast_items"], start=2):
        speed = float(spec["min_start_speed_mps"])
        minimum = int(spec["min_windows_per_direction"])
        for d in DIRECTIONS:
            n, model, ref = fast.get(speed, {}).get(d, (0, float("nan"), float("nan")))
            if n < minimum:
                applied = False
                items.append(
                    Item(
                        f"{k} {d}: windows with start speed >= {speed:g} m/s",
                        float(n),
                        ">=",
                        float(minimum),
                        False,
                        gp2.FAST_CANNOT_BE_APPLIED,
                    )
                )
                continue
            items.append(
                gp2._item(
                    f"{k} {d} start speed >= {speed:g} m/s: P1 ratio to dyn-v0.1 ({n} windows)",
                    model / ref if ref else float("nan"),
                    "<=",
                    float(spec["max_ratio"]),
                )
            )
    items.append(
        Item(
            "4 a top supported speed exists",
            float("nan") if top_supported_mps is None else float(top_supported_mps),
            ">=",
            0.0,
            top_supported_mps is not None,
            "highest speed node with a supported turning node on each side in both tables",
        )
    )
    holds = all(i.passed for i in items)
    rule = float(rule_mps)
    if not holds:
        v_max = default
        if not gate_passed:
            note = f"the bundle's model failed its gate: validity held at {default:g}"
        elif not applied:
            note = f"{gp2.FAST_CANNOT_BE_APPLIED}: validity held at {default:g}"
        elif top_supported_mps is None:
            note = f"no top supported speed: validity held at {default:g}"
        else:
            note = f"a fast item failed: validity held at {default:g}"
    elif not np.isfinite(rule):
        v_max, note = float("nan"), "the pipeline rule gave no number: no validity"
    else:
        assert top_supported_mps is not None
        v_max = min(cap, rule, top_supported_mps)
        note = (
            f"every item holds: validity min(cap {cap:g}, rule {rule:.3g}, top supported speed "
            f"{top_supported_mps:g})"
        )
    return ValidityDecision(
        v_max_mps=float(v_max),
        rule_mps=rule,
        exceeds_base=bool(np.isfinite(v_max) and v_max > default),
        applied=applied,
        items=tuple(items),
        note=note,
    )


# ---- outcome -----------------------------------------------------------------------------------

T5_CANDIDATE = "T5 is the candidate"
TL5_CANDIDATE = "TL5 is the candidate"


def gp2f_outcome(support: Decision, table: Decision, learned: Decision) -> str:
    """Section 6, Outcomes. GP2f-S or GP2f-T fails: the one-speed contract stays. GP2f-T passes and
    GP2f-L fails or was not evaluated: T5 is the candidate. Both pass: TL5."""
    if not (support.passed and table.passed):
        return gp2.ONE_SPEED_STAYS
    return TL5_CANDIDATE if learned.passed else T5_CANDIDATE
