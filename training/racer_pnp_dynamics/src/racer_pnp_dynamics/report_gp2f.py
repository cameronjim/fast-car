"""Text of the v0.5 fit summary, the held-out pass report and the pointer READMEs (brief 47).

Every item of a gate is shown with its value, comparison, threshold and result; an item that cannot
be applied is printed as such with its count (a failed gate is reported in full, not as a bare no).
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

from .export_v21 import readme_first_line
from .gp2f import GATE_NOT_APPLIED, NA
from .report_v05 import render_table_rows, render_turning_diagnostic, split_half_summary
from .table_fit_v05 import Scores, T5Fit

CONFIRMATION = (
    "candidate, not confirmed: needs contract 2.1 in the planner and the fresh confirmation GP2f-C"
)


def _num(x: object) -> str:
    return f"{x:.6g}" if isinstance(x, (int, float)) else str(x)


def render_items(decision: Mapping[str, Any]) -> str:
    if decision.get("evaluated", True) is False:
        return f"### {decision['gate']}: not evaluated\n"
    lines = [
        f"### {decision['gate']}: {'PASS' if decision['passed'] else 'FAIL'}",
        "",
        "| item | value | threshold | result | note |",
        "|---|---|---|---|---|",
    ]
    for i in decision["items"]:
        if i["comparison"] == NA:
            result, threshold = GATE_NOT_APPLIED.upper(), f"at least {_num(i['threshold'])}"
        else:
            result = "PASS" if i["passed"] else "FAIL"
            threshold = f"{i['comparison']} {_num(i['threshold'])}"
        lines.append(
            f"| {i['name']} | {_num(i['value'])} | {threshold} | {result} | {i.get('note', '')} |"
        )
    return "\n".join(lines) + "\n"


def render_validity(name: str, v: Mapping[str, Any]) -> str:
    lines = [
        f"{name}: v_max {v['v_max_mps']:.3g} m/s (pipeline rule {v['rule_mps']:.3g}); {v['note']}.",
        "",
    ]
    for i in v["items"]:
        lines.append(
            f"- {i['name']}: {_num(i['value'])} {i['comparison']} {_num(i['threshold'])} "
            f"{'PASS' if i['passed'] else 'FAIL'} {i.get('note', '')}".rstrip()
        )
    return "\n".join(lines) + "\n"


def render_comparator(rows: Sequence[Mapping[str, object]]) -> str:
    lines = [
        "| model | P1 (m) | A1 (m) | C1 (m) | Y1 (rad) | S1 (m/s) | P1 under T2s (m) | Pp (m) | Yp (rad) |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        lines.append(
            f"| {r['model']} | {r['P1']:.4f} | {r['A1']:.4f} | {r['C1']:.4f} | {r['Y1']:.4f} | "
            f"{r['S1']:.4f} | {r['P1_T2s']:.4f} | {r['Pp']:.4f} | {r['Yp']:.4f} |"
        )
    return "\n".join(lines) + "\n"


def render_metric_tables(metrics: Mapping[str, Any], names: Sequence[str]) -> str:
    """Per direction P1, Y1 and yaw bias; near-limit windows per side; transition windows; fast
    windows per direction: for every model, with the window counts."""
    dirs = sorted({d for m in metrics.values() for d in m.direction})
    out = ["### Per direction (macro over the direction's strata) and yaw rate bias", ""]
    out.append(
        "| model | " + " | ".join(f"P1 {d} | Y1 {d} | bias {d} (rad/s)" for d in dirs) + " |"
    )
    out.append("|---|" + "---|---|---|" * len(dirs))
    for n in names:
        m = metrics[n]
        cells = " | ".join(
            f"{m.direction[d]['P1']:.4f} | {m.direction[d]['Y1']:.4f} | {m.yaw_bias.get(d, float('nan')):+.4f}"
            for d in dirs
        )
        out.append(f"| {n} | {cells} |")
    ref = metrics[names[0]]
    out += [
        "",
        "### Near-limit windows (mean |steering share| at least 0.85), position RMSE at 1.0 s (m)",
        "",
    ]
    sides = list(ref.near_limit)
    out.append("| model | " + " | ".join(f"{s} (n={ref.near_limit[s][0]})" for s in sides) + " |")
    out.append("|---|" + "---|" * len(sides))
    for n in names:
        out.append(
            f"| {n} | " + " | ".join(f"{metrics[n].near_limit[s][1]:.4f}" for s in sides) + " |"
        )
    out += ["", f"### Transition windows (n={ref.transition[0]}), position RMSE at 1.0 s (m)", ""]
    out.append("| model | RMSE |")
    out.append("|---|---|")
    for n in names:
        out.append(f"| {n} | {metrics[n].transition[1]:.4f} |")
    speeds = sorted(ref.fast)
    out += ["", "### Fast windows per direction, position RMSE at 1.0 s (m)", ""]
    head = [f"{d} >= {v:g} (n={ref.fast[v][d][0]})" for v in speeds for d in ("ccw", "cw")]
    out.append("| model | " + " | ".join(head) + " |")
    out.append("|---|" + "---|" * len(head))
    for n in names:
        out.append(
            f"| {n} | "
            + " | ".join(f"{metrics[n].fast[v][d][1]:.4f}" for v in speeds for d in ("ccw", "cw"))
            + " |"
        )
    return "\n".join(out) + "\n"


def render_strata(
    strata_counts: Mapping[str, int], stratum_pos: Mapping[str, Mapping[str, float]]
) -> str:
    names = list(stratum_pos)
    lines = ["| stratum | windows | " + " | ".join(names) + " |", "|---|---|" + "---|" * len(names)]
    for s, n in strata_counts.items():
        lines.append(
            f"| {s} | {n} | "
            + " | ".join(f"{stratum_pos[m].get(s, float('nan')):.4f}" for m in names)
            + " |"
        )
    return "\n".join(lines) + "\n"


def render_pass_report(
    result: Mapping[str, Any],
    comparator: str,
    metric_tables: str,
    strata_table: str,
    freeze_sha: str,
    notes: Sequence[str],
    reported_only: str = "",
) -> str:
    boot = result["bootstrap_T5_over_v01"]
    boot_l = result["bootstrap_TL5_over_T5"]
    lines = [
        "# v0.5 dynamics: the held-out pass (GP2f-S, GP2f-T, GP2f-L, GP2f-V, one pass)",
        "",
        f"Freeze commit: `{freeze_sha}`.",
        "",
        *[f"- {n}" for n in notes],
        "",
        "## Comparator, identical windows, T2 unless the column says T2s",
        "",
        comparator,
        "## Strata, position RMSE at 1.0 s (m)",
        "",
        strata_table,
        "## Everything printed on the same windows",
        "",
        metric_tables,
        (
            f"Bootstrap of the P1 ratio (blocks of 5 s, 2000 resamples, seed 0): T5 over dyn-v0.1 "
            f"{boot['ratio']:.4f} [{boot['lo']:.4f}, {boot['hi']:.4f}]; TL5 over T5 "
            f"{boot_l['ratio']:.4f} [{boot_l['lo']:.4f}, {boot_l['hi']:.4f}]."
        ),
        "",
        render_items(result["support"]),
        render_items(result["table"]),
        render_items(result["learned"]),
        f"GP2f outcome: **{result['outcome']}**.",
        "",
        "## Validity above 1.3 m/s (GP2f-V)",
        "",
        f"Top supported speed: {result['top_supported_speed']}.",
        "",
        render_validity("T5", result["validity_T5"]),
        render_validity("TL5", result["validity_TL5"]),
    ]
    if reported_only:
        lines += [
            "## Reported only: the v0.2 split held-out segments (read before, decide nothing)",
            "",
            reported_only,
        ]
    return "\n".join(lines) + "\n"


def render_fit_summary(
    t5: T5Fit,
    t5_scores: Scores,
    v01_scores: Scores,
    tl5: Mapping[str, Any] | None,
    notes: Sequence[str],
    limit_alpha: float,
    limit_gamma: float,
) -> str:
    """The frozen fits in one markdown file (selection and train numbers only, no 2026-11-15
    held-out number): T5's tables with support, ties, bounds and split half, the selection
    scores, TL5's configuration and seed selection."""
    c = t5.constants
    lines = [
        "# v0.5 fits, frozen before the held-out pass",
        "",
        *[f"- {n}" for n in notes],
        "",
        "## T5",
        "",
        (
            f"Smoothness weight: alpha {t5.lam_alpha:g} (selection A1 by weight "
            f"{ {k: round(v, 5) for k, v in t5.selection_a1.items()} }), gamma {t5.lam_gamma:g} "
            f"(selection Y1 by weight { {k: round(v, 5) for k, v in t5.selection_y1.items()} })."
        ),
        (
            f"Supported nodes {int(t5.supported.sum())} of {t5.supported.size}; on a bound "
            f"(alpha): {t5.on_bound_alpha or 'none'}; (gamma): {t5.on_bound_gamma or 'none'}."
        ),
        *split_half_summary(t5, limit_alpha, limit_gamma),
        "",
        "Selection set (the 2026-11-14 held-out segments), macro RMSE at 1.0 s:",
        "",
        "| model | P1 (m) | A1 (m) | C1 (m) | Y1 (rad) | S1 (m/s) |",
        "|---|---|---|---|---|---|",
        f"| dyn-v0.1 | {v01_scores.p1:.4f} | {v01_scores.a1:.4f} | {v01_scores.c1:.4f} | {v01_scores.y1:.4f} | {v01_scores.s1:.4f} |",
        f"| T5 | {t5_scores.p1:.4f} | {t5_scores.a1:.4f} | {t5_scores.c1:.4f} | {t5_scores.y1:.4f} | {t5_scores.s1:.4f} |",
        "",
        render_table_rows(t5),
        "### 1 + gamma next to alpha at the supported turning nodes",
        "",
        render_turning_diagnostic(t5),
        f"Alpha range [{c.alpha_min:g}, {c.alpha_max:g}], gamma range [{c.gamma_min:g}, {c.gamma_max:g}].",
    ]
    if tl5 is not None:
        ship = tl5["shipping"]
        lines += [
            "",
            "## TL5",
            "",
            f"Shipping {ship.config.name}, seed {ship.seed}, device {ship.device}.",
            "",
            (
                "Chosen by P1 on the selection set (mean over the seeds of a configuration, then "
                "its best seed):"
            ),
            "",
            "| config | mean selection P1 | min | per seed |",
            "|---|---|---|---|",
        ]
        for name, row in tl5["selection"].items():
            seeds = ", ".join(f"{k} {v:.4f}" for k, v in row.items() if k.startswith("seed"))
            lines.append(f"| {name} | {row['mean']:.4f} | {row['min']:.4f} | {seeds} |")
    return "\n".join(lines) + "\n"


def readme_v05(
    model_id: str,
    *,
    summary: str,
    sections: Sequence[str],
    data_lines: Sequence[str],
    device_note: str = "",
) -> str:
    """One pointer README: the first line (do-not-deploy, contract 2.1: no planner on main can load
    this), a title, what the candidate is, every section given, the data."""
    out = [
        readme_first_line(),
        "",
        f"# {model_id}",
        "",
        summary,
        "",
        (
            f"Status: {CONFIRMATION}. Schema 2.1.0 is the owner's decision; until then the bundle "
            "is do-not-deploy whatever the gates say."
        ),
        "",
        *sections,
        "## Data",
        "",
        *[f"- {d}" for d in data_lines],
    ]
    if device_note:
        out += ["", f"Training device: {device_note}."]
    return "\n".join(out) + "\n"
