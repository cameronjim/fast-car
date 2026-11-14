"""The pointer README of a v0.3 candidate (brief 31): the verdict first, every item of the gate with
its value, threshold and result, the tables under T2 and, labelled, under T1, per stratum and per
seed, the bootstrap interval, Yp, and the statement that the pass is a candidate only."""

from __future__ import annotations

import math
from typing import Any

from .evaluate import HORIZONS_S
from .pipeline_v02 import _boot_line, _decision_table
from .pipeline_v03 import BASE_NAME, CANDIDATE_STATEMENT, DO_NOT_DEPLOY


def _fmt(x: float) -> str:
    return "n/a" if math.isnan(x) else f"{x:.4f}"


def macro_table(summary: dict[str, Any]) -> list[str]:
    rows = ["| model | 0.5 s | 1.0 s | 1.5 s | Yp (rad) |", "|---|---|---|---|---|"]
    for name, m in summary["macro"].items():
        cells = " | ".join(
            f"{m[str(h)]['P']:.3f} / {m[str(h)]['Y']:.3f} / {m[str(h)]['S']:.3f}"
            for h in HORIZONS_S
        )
        rows.append(f"| {name} | {cells} | {_fmt(summary['yp'].get(name, float('nan')))} |")
    return rows


def stratum_table(summary: dict[str, Any]) -> list[str]:
    table = summary["per_stratum_position_rmse_1s_m"]
    models = [k for k in next(iter(table.values())) if k != "windows"]
    rows = [
        "| stratum | windows | " + " | ".join(models) + " |",
        "|---|---|" + "---|" * len(models),
    ]
    for name, row in table.items():
        rows.append(
            f"| {name} | {row['windows']} | " + " | ".join(f"{row[m]:.3f}" for m in models) + " |"
        )
    return rows


def _truth_block(summary: dict[str, Any], heading: str, per_seed: bool) -> list[str]:
    merges = (
        "; ".join(
            f"{m['direction']} {m['from']} ({m['windows']} windows) into {m['into']}"
            for m in summary["merges"]
        )
        or "none"
    )
    yp = [
        f"- {n}: "
        + ", ".join(f"{seg} {_fmt(v['yp'])} ({int(v['intervals'])})" for seg, v in per.items())
        for n, per in summary["yp_segments"].items()
    ]
    lines = [
        f"## {heading}",
        "",
        (
            f"Held-out windows {summary['windows']}; strata after merging: "
            f"{', '.join(summary['strata']['names'])}. Merges: {merges}."
        ),
        "",
        "Macro RMSE at each horizon, position (m) / yaw (rad) / wheel speed (m/s), and Yp:",
        "",
        *macro_table(summary),
        "",
        "Per stratum position RMSE at 1.0 s (m)" + (", per seed" if per_seed else "") + ":",
        "",
        *stratum_table(summary),
        "",
        "Yp per segment (rad, usable pose intervals):",
        "",
        *yp,
        "",
    ]
    return lines


def _verdict(gate: str, passed: bool) -> list[str]:
    if passed:
        return [
            f"Verdict: {gate} PASSED on the held-out segments under T2. {CANDIDATE_STATEMENT}.",
            "",
        ]
    return [
        (
            f"{DO_NOT_DEPLOY}: {gate} FAILED on the held-out segments under T2. The deployed "
            "model does not change; this table is the result."
        ),
        "",
    ]


def render_y_report(model_id: str, metrics: dict[str, Any]) -> str:
    ev = metrics["heldout"]
    refit = metrics["refit"]
    dec = ev["decision_GP2c_Y"]
    lines = [
        *_verdict("GP2c-Y (yaw refit)", dec["passed"]),
        f"# {model_id}",
        "",
        (
            "Candidate Y: refit of the four yaw parameters of dyn-v0.1 (`steer_gain_left`, "
            "`steer_gain_right`, `steer_offset_rad`, `understeer_s2_per_m2`), every other parameter "
            "held at its dyn-v0.1 value (read from the dyn-v0.1 manifest). Train role of the split "
            "plus the legacy train part with a T2 truth (the two October bags have no /odom/wheel "
            "and so no T2: they are not in the fit). Truth T2 (wheel seeded ICP). Pre-registration "
            "docs/notes/preregistration-pnp-v0.3.md section 2.3, constants "
            f"`evaluation/prereg/pnp-v0.3.yaml`, split SHA-256 `{metrics['split_sha256']}`."
        ),
        "",
        f"vehicle_params {metrics['vehicle_params']}.",
        "",
        "## Refit parameters",
        "",
        "| parameter | dyn-v0.1 | Y | bounds | on a bound |",
        "|---|---|---|---|---|",
        *[
            f"| {n} | {refit['base'][n]:.4f} | {refit['nominal'][n]:.4f} | "
            f"[{refit['bounds'][n][0]:g}, {refit['bounds'][n][1]:g}] | "
            f"{'YES' if n in refit['on_bound'] else 'no'} |"
            for n in refit["free"]
        ],
        "",
        "Unidentified (on a bound or a flat profile): "
        + ("; ".join(f"{k}: {v}" for k, v in refit["unidentified"].items()) or "none")
        + ".",
        "",
        (
            "Profiles (validation yaw RMSE per 0.1 s interval against each parameter, the other "
            "three refit) are in `profiles.png`. Validation yaw RMSE: "
            f"dyn-v0.1 {refit['yaw_rmse_val']['base']:.4f}, Y "
            f"{refit['yaw_rmse_val']['refit']:.4f} rad. "
            f"Intervals: train {refit['counts']['yaw_intervals_train']}, "
            f"validation {refit['counts']['yaw_intervals_val']}."
        ),
        "",
        "## GP2c-Y decision (held-out segments, truth T2)",
        "",
        _decision_table(dec),
        "",
        _boot_line("Y over dyn-v0.1", ev["bootstrap_p1_ratio_Y_over_v01"]),
        "",
        *_truth_block(ev["T2"], "Truth T2 (primary, decides)", False),
        *_truth_block(ev["T1"], "Truth T1 (labelled: the v0.2 truth, decides nothing)", False),
        CANDIDATE_STATEMENT if dec["passed"] else f"{DO_NOT_DEPLOY}: the gate failed.",
        "",
    ]
    assert BASE_NAME in ev["T2"]["macro"]
    return "\n".join(lines)


def render_r_report(model_id: str, metrics: dict[str, Any]) -> str:
    ev = metrics["heldout"]
    dec = ev["decision_GP2c_R"]
    sel = metrics["selection"]
    lines = [
        *_verdict("GP2c-R (yaw-only residual)", dec["passed"]),
        f"# {model_id}",
        "",
        (
            f"Candidate R: the MLP residual of the bundle contract with `output_scale: [0.0, s_yaw]`, "
            f"trained on top of the nominal `{ev['nominal']}`, truth T2, train segments only, "
            f"config and shipping seed chosen on validation P1 and frozen before held-out was read. "
            f"Shipping config `{ev['shipping_config']}`, seed {ev['shipping_seed']}. "
            f"Split SHA-256 `{metrics['split_sha256']}`."
        ),
        "",
        "## Addendum",
        "",
        *[f"- {a}" for a in metrics["addenda"]],
        "",
        "## GP2c-R decision (held-out segments, truth T2)",
        "",
        _decision_table(dec),
        "",
        _boot_line("shipping seed over the nominal", ev["bootstrap_p1_ratio_R_over_nominal"]),
        "",
        "Validation P1 per candidate (metres, macro, internal validation part of train):",
        "",
        "| config | mean | min | seed 0 | seed 1 | seed 2 |",
        "|---|---|---|---|---|---|",
        *[
            f"| {k} | {v['mean']:.4f} | {v['min']:.4f} | {v.get('seed0', float('nan')):.4f} | "
            f"{v.get('seed1', float('nan')):.4f} | {v.get('seed2', float('nan')):.4f} |"
            for k, v in sel.items()
            if not k.startswith("_")
        ],
        "",
        *_truth_block(ev["T2"], "Truth T2 (primary, decides), per seed", True),
        *_truth_block(ev["T1"], "Truth T1 (labelled: the v0.2 truth, decides nothing)", True),
        CANDIDATE_STATEMENT if dec["passed"] else f"{DO_NOT_DEPLOY}: the gate failed.",
        "",
    ]
    return "\n".join(lines)
