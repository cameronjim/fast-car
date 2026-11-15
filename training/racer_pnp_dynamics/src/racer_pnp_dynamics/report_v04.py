"""Text of the v0.4 results: item tables, the READMEs of the pointer directories, the pass report.

Every item is shown with its value, comparison, threshold and result (a failed gate is reported in
full). The first line of a README of a failed candidate starts with `do-not-deploy`; every contract
2 README says in that same first line that no planner on main can load the bundle.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

from .export_v2 import FIRST_README_LINE

DO_NOT_DEPLOY = "do-not-deploy"
CONFIRMATION = (
    "candidate, not confirmed: needs the fresh confirmation of pre-registration v0.4 section 2.5"
)


def first_line(contract2: bool, passed: bool, deployment_note: str = "") -> str:
    """The first README line: `do-not-deploy` for a failed candidate, the contract 2 warning for
    every contract 2 candidate (both, in one line, when both apply)."""
    parts = []
    if not passed:
        parts.append(DO_NOT_DEPLOY)
    if contract2:
        parts.append(FIRST_README_LINE)
    elif passed:
        parts.append(deployment_note or CONFIRMATION)
    return "; ".join(parts) if parts else CONFIRMATION


def _num(x: object) -> str:
    return f"{x:.6g}" if isinstance(x, (int, float)) else str(x)


def render_items(decision: Mapping[str, Any]) -> str:
    if decision.get("evaluated", True) is False:
        return f"{decision['gate']}: not evaluated.\n"
    lines = [
        f"### {decision['gate']}: {'PASS' if decision['passed'] else 'FAIL'}",
        "",
        "| item | value | threshold | result | note |",
        "|---|---|---|---|---|",
    ]
    for i in decision["items"]:
        lines.append(
            f"| {i['name']} | {_num(i['value'])} | {i['comparison']} {_num(i['threshold'])} | "
            f"{'PASS' if i['passed'] else 'FAIL'} | {i.get('note', '')} |"
        )
    return "\n".join(lines) + "\n"


def render_validity(v: Mapping[str, Any]) -> str:
    lines = [
        f"Validity: v_max {v['v_max_mps']:.3g} m/s (pipeline rule {v['rule_mps']:.3g}); {v['note']}.",
        "",
    ]
    for i in v["items"]:
        lines.append(
            f"- {i['name']}: {_num(i['value'])} {i['comparison']} {_num(i['threshold'])} "
            f"{'PASS' if i['passed'] else 'FAIL'} {i.get('note', '')}".rstrip()
        )
    return "\n".join(lines) + "\n"


def readme(
    model_id: str,
    *,
    contract2: bool,
    passed: bool,
    summary: str,
    sections: Sequence[str],
    data_lines: Sequence[str],
    device_note: str = "",
) -> str:
    """One pointer README: the first line, a title, what the candidate is, every section given."""
    out = [
        first_line(contract2, passed),
        "",
        f"# {model_id}",
        "",
        summary,
        "",
        f"Status: {CONFIRMATION}.",
        "",
        *sections,
        "## Data",
        "",
        *[f"- {d}" for d in data_lines],
    ]
    if device_note:
        out += ["", f"Training device: {device_note}."]
    return "\n".join(out) + "\n"


def pass_report(
    result: Mapping[str, Any],
    comparator: str,
    freeze_sha: str,
    notes: Sequence[str],
    read_before: str = "",
) -> str:
    """The report of the one held-out pass: the decisions in order, all items, the comparator."""
    lines = [
        "# v0.4 dynamics: the held-out pass (GP2d and GP2e, one pass)",
        "",
        f"Freeze commit: `{freeze_sha}`.",
        "",
        *[f"- {n}" for n in notes],
        "",
        "## Comparator, identical windows, T2 unless the column says T2s",
        "",
        comparator,
        f"Shipping nominal (GP2d): {result['shipping_nominal']}.",
        "",
        render_items(result["GP2d_N"]),
        render_items(result["GP2d_R"]),
        f"Named outcome (a label, deploys nothing): **{result['heading_outcome']['label']}**.",
        "",
        render_items(
            {
                "gate": "Heading outcome items",
                "passed": result["heading_outcome"]["label"] == "improves heading, not position",
                "items": result["heading_outcome"]["items"],
            }
        ),
        render_validity(result["validity"]),
        render_items(result["GP2e_T"]),
        render_items(result["GP2e_L"]),
        f"GP2e outcome: **{result['gp2e_outcome']}**.",
        "",
        "Validity of the contract 2 bundles:",
        "",
        "T4: " + render_validity(result["validity_T4"]),
        "TL4: " + render_validity(result["validity_TL4"]),
    ]
    if read_before:
        lines += ["## The v0.2 held-out segments (read before, decide nothing)", "", read_before]
    return "\n".join(lines) + "\n"
