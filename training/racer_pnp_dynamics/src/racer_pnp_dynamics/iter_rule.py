"""The rule of a version of the dynamics iteration (dyn-v0.8 and later) against dyn-v0.7p.

Each version has its own rule file, `evaluation/prereg/dyn-iter/<model_id>.yaml`, written and
committed before its one pass. The items are GP2g-R's (brief 54) with dyn-v0.7p in place of
dyn-v0.1 and the GP2g-R thresholds unchanged, except two items that cannot apply as written to a
speed and yaw residual against a speed and yaw residual, which the file replaces and states:

* 2b: S1 at most 1.00 times dyn-v0.7p's (GP2g-R asks S1 equal within 1e-6 because R6 is yaw-only);
* 12: the planner solve time with `dynamics_fast_mlp` at most 1.00 times dyn-v0.7p's with it
  (GP2g-R asks at most 1.25 times dyn-v0.1's).

Item 13 (the L5 canary with the bundle) is not part of the held-out rule; it is a check before a
version is made the prototype's bundle. Every threshold is read from the file; nothing is typed in.
"""

from __future__ import annotations

import hashlib
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from . import gp2, gp2f, gp2g
from .gp2 import Decision, Item, SweepResult
from .strata import Strata

RULE_DIR = Path("evaluation") / "prereg" / "dyn-iter"
GATE = "dyn-iter rule against dyn-v0.7p"
PASS, FAIL, INSUFFICIENT = gp2g.PASS, gp2g.FAIL, gp2g.INSUFFICIENT
GP2G_UNCHANGED = (
    "max_p1_ratio",
    "max_p1_ratio_bootstrap_upper",
    "max_y1_ratio",
    "max_seed_p1_ratio",
    "max_stratum_ratio",
    "sweep_samples",
    "max_pose_yaw_ratio",
    "max_cross_ratio",
    "max_along_ratio",
    "max_near_limit_ratio",
    "max_transition_ratio",
    "max_p1_ratio_alt_truth",
)


class RuleInvalid(ValueError):
    """A rule file that loosens a GP2g-R threshold or lacks an item."""


@dataclass(frozen=True)
class Rule:
    sha256: str
    doc: Mapping[str, Any]

    @property
    def gate(self) -> Mapping[str, Any]:
        return self.doc["gate"]

    @property
    def metrics(self) -> Mapping[str, Any]:
        return self.doc["metrics"]


@dataclass(frozen=True)
class Solve:
    """Item 12: planner solve p50 (ms) with `dynamics_fast_mlp`, the version and dyn-v0.7p."""

    candidate_p50_ms: float
    reference_p50_ms: float
    note: str = ""


def rule_relative(model_id: str) -> Path:
    return RULE_DIR / f"{model_id}.yaml"


def rule_sha256(root: Path, model_id: str) -> str:
    return hashlib.sha256((root / rule_relative(model_id)).read_bytes()).hexdigest()


def parse_rule(blob: bytes, gp2g_gate: Mapping[str, Any] | None = None) -> Rule:
    """Read a rule; with `gp2g_gate` (GP2g-R's `gate_residual` block) refuse any threshold that
    differs from GP2g-R's for an item kept unchanged."""
    doc = yaml.safe_load(blob.decode("utf-8"))
    for key in ("gate", "metrics", "reference", "candidate"):
        if key not in doc:
            raise RuleInvalid(f"rule lacks {key!r}")
    g = doc["gate"]
    if gp2g_gate is not None:
        for key in GP2G_UNCHANGED:
            if g.get(key) != gp2g_gate[key]:
                raise RuleInvalid(f"{key}: {g.get(key)!r} is not GP2g-R's {gp2g_gate[key]!r}")
        for key in ("item_0_enough_data", "per_direction", "saturation"):
            if g.get(key) != gp2g_gate[key]:
                raise RuleInvalid(f"{key}: not GP2g-R's")
    if float(g["max_s1_ratio"]) > 1.0 or float(g["max_fast_solve_ratio_to_reference"]) > 1.0:
        raise RuleInvalid("the replaced items 2b and 12 may not exceed 1.00 times dyn-v0.7p")
    return Rule(hashlib.sha256(blob).hexdigest(), doc)


def load_rule(root: Path, model_id: str) -> Rule:  # pragma: no cover - real checkout
    from .stages_v06 import constants

    blob = (root / rule_relative(model_id)).read_bytes()
    return parse_rule(blob, constants().gate)


def select_cost_aware(candidates, width_tolerance: float):
    """The config with the lowest mean validation P1 over its seeds, unless a narrower config's
    mean is within `width_tolerance` (relative) of it: then the narrowest such config (ties: the
    lower mean). Within the config, the seed with the lowest validation P1."""
    by_cfg: dict[str, list] = {}
    for c in candidates:
        by_cfg.setdefault(c.config.name, []).append(c)
    table = {
        name: {
            "mean": float(np.mean([c.val_p1_m for c in cs])),
            "min": float(np.min([c.val_p1_m for c in cs])),
            "hidden": float(cs[0].config.hidden),
            **{f"seed{c.seed}": c.val_p1_m for c in cs},
        }
        for name, cs in by_cfg.items()
    }
    best = min(table.values(), key=lambda r: r["mean"])["mean"]
    ok = [n for n, r in table.items() if r["mean"] <= best * (1.0 + width_tolerance)]
    name = min(ok, key=lambda n: (table[n]["hidden"], table[n]["mean"]))
    ship = min(by_cfg[name], key=lambda c: c.val_p1_m)
    return ship.config, ship, table


def decide(
    rule: Rule,
    direction: np.ndarray,
    strata: Strata,
    ref: gp2f.Metrics,
    cand: gp2f.Metrics,
    ratio_ci_hi: float,
    seed_p1: Mapping[int, float],
    sweep: SweepResult,
    saturation_share: float,
    solve: Solve,
) -> gp2g.GateResult:
    """Items 0 to 12 of the rule; `ref` is dyn-v0.7p, `cand` the version's shipping seed."""
    g, m = rule.gate, rule.metrics
    zero_items = []
    g0 = g["item_0_enough_data"]
    zero_items.append(gp2f._ge("0a windows", float(len(direction)), float(g0["min_windows_total"])))
    for d in gp2f.DIRECTIONS:
        zero_items.append(
            gp2f._ge(
                f"0b windows {d}",
                float(np.sum(np.asarray(direction) == d)),
                float(g0["min_windows_per_direction"]),
            )
        )
    per = int(m["min_windows_per_stratum"])
    zero_items.append(
        gp2f._ge(
            f"0c strata with {per} windows",
            float(sum(1 for n in strata.names if strata.counts[n] >= per)),
            float(g0["min_strata_with_30_windows"]),
        )
    )
    zero = Decision("item 0 enough data", tuple(zero_items))
    items: list[Item] = [
        gp2f._ratio(
            "1a P1 ratio / dyn-v0.7p", cand.macro["P1"], ref.macro["P1"], float(g["max_p1_ratio"])
        ),
        gp2._item(
            "1b upper end of the bootstrap interval of the P1 ratio",
            ratio_ci_hi,
            "<",
            float(g["max_p1_ratio_bootstrap_upper"]),
        ),
        gp2f._ratio("2a Y1 ratio", cand.macro["Y1"], ref.macro["Y1"], float(g["max_y1_ratio"])),
        gp2f._ratio(
            "2b S1 ratio (replaces GP2g-R's equality)",
            cand.macro["S1"],
            ref.macro["S1"],
            float(g["max_s1_ratio"]),
        ),
    ]
    for seed, p1 in sorted(seed_p1.items()):
        items.append(
            gp2f._ratio(
                f"3 seed {seed} P1 ratio", p1, ref.macro["P1"], float(g["max_seed_p1_ratio"])
            )
        )
    for name in strata.names:
        n = strata.counts[name]
        label = f"4 stratum {name} position RMSE ratio"
        if n < per:
            items.append(gp2f._na(label, n, per, "windows"))
        else:
            items.append(
                gp2f._ratio(
                    f"{label} ({n} windows)",
                    cand.stratum_pos[name],
                    ref.stratum_pos[name],
                    float(g["max_stratum_ratio"]),
                )
            )
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
    nan = {"P1": float("nan"), "Y1": float("nan")}
    for d in gp2f.DIRECTIONS:
        cd, rd = cand.direction.get(d, nan), ref.direction.get(d, nan)
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
            gp2f._windowed(
                f"9 near-limit {side} position RMSE ratio",
                cand.near_limit.get(side, (0, float("nan"))),
                ref.near_limit.get(side, (0, float("nan"))),
                n_side,
                float(g["max_near_limit_ratio"]),
            )
        )
    items.append(
        gp2f._windowed(
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
            "12 planner solve p50 with fast_mlp / dyn-v0.7p's (replaces GP2g-R's 1.25 x dyn-v0.1)",
            solve.candidate_p50_ms,
            solve.reference_p50_ms,
            float(g["max_fast_solve_ratio_to_reference"]),
            f"{solve.candidate_p50_ms:.3f} ms against {solve.reference_p50_ms:.3f} ms. {solve.note}".strip(),
        )
    )
    if not zero.passed:
        return gp2g.GateResult(Decision(GATE, tuple(items), evaluated=False), INSUFFICIENT, zero)
    decision = Decision(GATE, tuple(items))
    return gp2g.GateResult(decision, PASS if decision.passed else FAIL, zero)


def render_report(out: Mapping[str, Any], metrics: Mapping[str, gp2f.Metrics], me: str) -> str:
    from .stages_v06 import render_items

    g = out["gate"]
    b, b1 = out["bootstrap_P1_vs_v07p"], out["bootstrap_P1_vs_v01"]
    lines = [
        f"# {out['model_id']} pass against dyn-v0.7p: {g['outcome']}",
        "",
        f"Freeze commit `{out['freeze_sha']}`; rule SHA-256 {out['rule_sha256']}.",
        "",
        "## H1: GP2g-R held-out parts of 2026-11-22 (decides)",
        "",
        (
            f"Windows: {out['windows']} (identical under T2 and T2s); per direction "
            f"{out['direction_windows']}; strata {out['strata']}."
        ),
        "",
        f"Shipping: {out['shipping']['config']} seed {out['shipping']['seed']} (hidden {out['shipping']['hidden']}).",
        "",
        *render_items(g["enough_data"]),
        "",
        *render_items(g["decision"]),
        "",
        "Metrics at 1.0 s (macro over strata):",
        "",
        "| model | P1 (m) | A1 (m) | C1 (m) | Y1 (rad) | S1 (m/s) | P1 T2s (m) | Yp (rad) | ccw P1 | ccw Y1 | cw P1 | cw Y1 |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for n, mm in metrics.items():
        a = mm.macro
        dd = {
            d: mm.direction.get(d, {"P1": float("nan"), "Y1": float("nan")}) for d in ("ccw", "cw")
        }
        lines.append(
            f"| {n} | {a['P1']:.4f} | {a['A1']:.4f} | {a['C1']:.4f} | {a['Y1']:.4f} | {a['S1']:.4f} | "
            f"{mm.p1_alt:.4f} | {mm.yp:.4f} | {dd['ccw']['P1']:.4f} | {dd['ccw']['Y1']:.4f} | "
            f"{dd['cw']['P1']:.4f} | {dd['cw']['Y1']:.4f} |"
        )
    lines += [
        "",
        f"P1 ratio to dyn-v0.7p, bootstrap: {b['ratio']:.4f} ({b['lo']:.4f} to {b['hi']:.4f}).",
        f"For information, P1 ratio to dyn-v0.1: {b1['ratio']:.4f} ({b1['lo']:.4f} to {b1['hi']:.4f}).",
        (
            f"Saturation: {out['saturation']['share']:.4%} of {out['saturation']['steps']} steps; "
            f"largest |output| {out['saturation']['max_abs']}."
        ),
        "",
        "## H2: dyn-v0.7p's 2026-11-15 development parts (reported only, decides nothing)",
        "",
        f"Windows: {out['h2_windows']}. Ratios to dyn-v0.7p.",
        "",
        "| group | model | windows | P1 m | ratio | Y1 rad | ratio | S1 m/s | ratio |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for r in out["h2_rows"]:
        lines.append(
            f"| {r['group']} | {r['model']} | {r['windows']} | {r['P1']} | {r['P1_ratio']} | "
            f"{r['Y1']} | {r['Y1_ratio']} | {r['S1']} | {r['S1_ratio']} |"
        )
    hb = out["h2_bootstrap_P1_vs_v07p"]
    lines += [
        "",
        f"H2 P1 ratio to dyn-v0.7p, bootstrap: {hb['ratio']:.4f} ({hb['lo']:.4f} to {hb['hi']:.4f}).",
        "",
    ]
    return "\n".join(lines) + "\n"
