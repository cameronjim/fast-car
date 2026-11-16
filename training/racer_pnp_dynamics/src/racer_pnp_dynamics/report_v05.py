"""Text of the v0.5 results: the table report of M-B5 and the fit summary (brief 47).

Everything is printed from a `T5Fit` and numbers computed on the selection set or on train; no
2026-11-15 held-out number can pass through here (the functions take what the fit stages hold).
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence

import numpy as np

from .table_fit_v05 import Scores, T5Fit


def _f(x: float | None, nd: int = 4) -> str:
    if x is None or (isinstance(x, float) and not np.isfinite(x)):
        return ""
    return f"{x:.{nd}f}"


def _node(c, n: int) -> str:
    n_v = len(c.speed_nodes_mps)
    return f"s{c.steering_nodes[n // n_v]:+g}/v{c.speed_nodes_mps[n % n_v]:g}"


def render_table_rows(fit: T5Fit) -> str:
    """Both tables per node: value, windows, support, tied to, on a bound, split half difference."""
    c = fit.constants
    head = (
        "| steering share | speed (m/s) | windows | supported | alpha | gamma | 1 + gamma | "
        "tied to | alpha on a bound | gamma on a bound | alpha split half | gamma split half |"
    )
    lines = [head, "|" + "---|" * 12]
    for r in fit.rows():
        tied = "" if r["tied_to"] is None else _node(c, int(r["tied_to"]))
        gamma_note = " (fixed)" if r["gamma_fixed"] else ""
        lines.append(
            f"| {r['steering_share']:+g} | {r['speed_mps']:g} | {r['windows']} | "
            f"{'yes' if r['supported'] else 'no'} | {_f(r['alpha'])} | "
            f"{_f(r['gamma'])}{gamma_note} | {_f(1.0 + r['gamma'])} | {tied} | "
            f"{'ON BOUND' if r['alpha_on_bound'] else ''} | "
            f"{'ON BOUND' if r['gamma_on_bound'] else ''} | "
            f"{_f(r['alpha_split_half'])} | {_f(r['gamma_split_half'])} |"
        )
    return "\n".join(lines) + "\n"


def render_turning_diagnostic(fit: T5Fit) -> str:
    """`1 + gamma` next to `alpha` at every supported turning node (steering share not 0): if the
    yaw rate follows ground speed rather than wheel speed, the two numbers are close."""
    lines = [
        "| steering share | speed (m/s) | alpha | 1 + gamma | (1 + gamma) - alpha |",
        "|---|---|---|---|---|",
    ]
    for r in fit.rows():
        if not r["supported"] or r["gamma_fixed"]:
            continue
        a, g = r["alpha"], 1.0 + r["gamma"]
        lines.append(
            f"| {r['steering_share']:+g} | {r['speed_mps']:g} | {_f(a)} | {_f(g)} | {_f(g - a)} |"
        )
    return "\n".join(lines) + "\n"


def split_half_summary(fit: T5Fit, limit_alpha: float, limit_gamma: float) -> list[str]:
    sh = fit.split_half
    if sh is None:
        return ["split half: not computed"]
    out = []
    for key, limit, name in (
        ("alpha_diff", limit_alpha, "alpha"),
        ("gamma_diff", limit_gamma, "gamma"),
    ):
        d = sh[key]
        if np.isnan(d).all():
            out.append(f"split half {name}: no supported node")
            continue
        worst = np.unravel_index(np.nanargmax(d), d.shape)
        c = fit.constants
        out.append(
            f"split half {name}: largest difference {np.nanmax(d):.4f} at "
            f"{_node(c, int(worst[0] * d.shape[1] + worst[1]))} against the limit {limit:g}; "
            f"{int(np.nansum(d > limit))} supported node(s) above it"
        )
    return out


def render_mb5(
    fit: T5Fit,
    m_b5: Scores,
    v01: Scores,
    *,
    limit_alpha: float,
    limit_gamma: float,
    notes: Sequence[str],
    by_direction: Mapping[str, int] | None = None,
) -> str:
    c = fit.constants
    stop = bool(fit.on_bound_alpha or fit.on_bound_gamma)
    lines = [
        "# M-B5: the T5 method on the data that exists before the 2026-11-15 session",
        "",
        (
            "Pre-registration v0.5 section 4. A measurement with one stop rule: a supported node on a "
            "bound means the reading of section 1 is wrong. No 2026-11-15 data was read."
        ),
        "",
        *[f"- {n}" for n in notes],
        "",
        "## Result against the stop rule",
        "",
        (
            "**STOP: a supported node is on a bound** "
            f"(alpha: {fit.on_bound_alpha or 'none'}; gamma: {fit.on_bound_gamma or 'none'}). "
            "Nothing more is fitted."
            if stop
            else "No supported node is on a bound (alpha: none; gamma: none). The work goes on."
        ),
        "",
        (
            f"Smoothness weight: alpha {fit.lam_alpha:g} (selection A1 by weight "
            f"{ {k: round(v, 5) for k, v in fit.selection_a1.items()} }), gamma {fit.lam_gamma:g} "
            f"(selection Y1 by weight { {k: round(v, 5) for k, v in fit.selection_y1.items()} })."
        ),
        (
            f"Supported nodes: {int(fit.supported.sum())} of {fit.supported.size}; alpha range "
            f"[{c.alpha_min:g}, {c.alpha_max:g}], gamma range [{c.gamma_min:g}, {c.gamma_max:g}], a "
            f"node is on a bound within {c.bound_margin:g}."
        ),
        *split_half_summary(fit, limit_alpha, limit_gamma),
        "",
        "## Selection set (the 2026-11-14 held-out segments), macro RMSE at 1.0 s",
        "",
        "| model | P1 (m) | A1 (m) | C1 (m) | Y1 (rad) | S1 (m/s) |",
        "|---|---|---|---|---|---|",
        f"| dyn-v0.1 | {v01.p1:.4f} | {v01.a1:.4f} | {v01.c1:.4f} | {v01.y1:.4f} | {v01.s1:.4f} |",
        (
            f"| M-B5 model (T5) | {m_b5.p1:.4f} | {m_b5.a1:.4f} | {m_b5.c1:.4f} | "
            f"{m_b5.y1:.4f} | {m_b5.s1:.4f} |"
        ),
        (
            f"| ratio | {m_b5.p1 / v01.p1:.4f} | {m_b5.a1 / v01.a1:.4f} | {m_b5.c1 / v01.c1:.4f} | "
            f"{m_b5.y1 / v01.y1:.4f} | {m_b5.s1 / v01.s1:.4f} |"
        ),
        "",
        "## Both tables, per node",
        "",
        (
            "Node = steering share of the side limit (negative right) by measured wheel speed. A node "
            "that is not supported copies the node in `tied to` and is no parameter; the gamma centre "
            "column is fixed at 0."
        ),
        "",
        render_table_rows(fit),
        "## 1 + gamma next to alpha at the supported turning nodes",
        "",
        "If the yaw rate follows ground speed rather than wheel speed, the two are close.",
        "",
        render_turning_diagnostic(fit),
    ]
    if by_direction:
        lines += [
            "## Windows",
            "",
            *[f"- {k}: {v}" for k, v in by_direction.items()],
            "",
        ]
    return "\n".join(lines) + "\n"


def render_mb6(summary: Mapping, notes: Sequence[str], hashes: Mapping[str, str]) -> str:
    """M-B6: the frozen v0.4 models on the whole PnP runs of the session. Decides nothing."""
    names = list(summary["macro"])
    dirs = list(summary["directions"])
    lines = [
        "# M-B6: the frozen v0.4 models on the PnP runs of the 2026-11-15 session, whole",
        "",
        (
            "Pre-registration v0.5 section 4. Printed, deciding nothing. The models were frozen "
            "before this session existed (freeze commit "
            "416e68501c424caeb1f6dc80b6a0ee9edc7a288c)."
        ),
        "",
        *[f"- {n}" for n in notes],
        "",
        "Pickles scored (SHA-256): " + ", ".join(f"{k} {v[:16]}" for k, v in hashes.items()) + ".",
        "",
        f"Windows: {summary['windows']}; by direction (measured heading): "
        + ", ".join(f"{d} {n}" for d, n in summary["directions"].items())
        + ".",
        "",
        "## Macro RMSE at 1.0 s, identical windows, T2",
        "",
        "| model | P1 (m) | A1 (m) | C1 (m) | Y1 (rad) | S1 (m/s) | P1 / dyn-v0.1 | Y1 / dyn-v0.1 |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for n in names:
        m, r = summary["macro"][n], summary["ratio"][n]
        lines.append(
            f"| {n} | {m['P1']:.4f} | {m['A1']:.4f} | {m['C1']:.4f} | {m['Y1']:.4f} | "
            f"{m['S1']:.4f} | {r['P1']:.4f} | {r['Y1']:.4f} |"
        )
    head = "| model | " + " | ".join(f"P1 {d} (m) | Y1 {d} (rad)" for d in dirs) + " |"
    lines += ["", "## Per direction (macro over that direction's strata)", "", head]
    lines.append("|---|" + "---|---|" * len(dirs))
    for n in names:
        cells = " | ".join(
            f"{summary['per_direction'][n][d]['P1']:.4f} | "
            f"{summary['per_direction'][n][d]['Y1']:.4f}"
            for d in dirs
        )
        lines.append(f"| {n} | {cells} |")
    lines += ["", "## Per run, pooled position RMSE at 1.0 s (m)", ""]
    lines.append("| run | windows | " + " | ".join(names) + " |")
    lines.append("|---|---|" + "---|" * len(names))
    for run, row in summary["per_run"].items():
        lines.append(
            f"| {run} | {row['windows']} | " + " | ".join(f"{row[n]:.4f}" for n in names) + " |"
        )
    lines += ["", "## Strata (windows)", ""]
    lines += [f"- {k}: {v}" for k, v in summary["strata"].items()]
    if summary["merges"]:
        lines += ["", "Merged strata: " + "; ".join(str(m) for m in summary["merges"]) + "."]
    return "\n".join(lines) + "\n"
