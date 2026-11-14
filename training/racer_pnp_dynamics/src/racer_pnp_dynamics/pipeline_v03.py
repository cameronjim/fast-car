"""dyn-v0.3, the yaw channel (brief 31, pre-registration v0.3 section 2).

This module holds, in the order they were written:

1. scoring of named models on the held-out segments under one truth (T1 or T2) with the
   pre-registered windows, strata and macro averaging of v0.2 section 3, plus Yp (`pose_check`);
2. the M-B1 table: dyn-v0.1 and the v0.2 residual `dyn-v0.1r` scored under T2 and T1 before any
   v0.3 model is fitted.

Everything the v0.2 pipeline already does (windows, strata, bootstrap) is reused as it is.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

from . import pose_check
from .dataset import Segment
from .evaluate import HORIZONS_S, eval_set, window_errors
from .model import Nominal, Residual
from .pipeline_v02 import macro_triplet
from .strata import assign_strata, block_bootstrap_ratio, per_stratum_table, stratum_rmse
from .vehicle import Vehicle

TRUTH_T2 = "T2"
TRUTH_T1 = "T1"


@dataclass(frozen=True)
class Scored:
    """Everything the tables and the decisions read for one truth."""

    truth: str
    windows: int
    strata_names: tuple[str, ...]
    strata_windows: dict[str, int]
    merges: tuple[dict, ...]
    macro: dict[str, dict[float, dict[str, float]]]  # model -> horizon -> P / Y / S
    stratum_pos_1s: dict[str, dict[str, float]]  # stratum -> windows and model -> RMSE
    errs: dict[str, dict[float, dict[str, np.ndarray]]]
    strata: Any
    block: np.ndarray
    yp: dict[str, float]
    yp_segments: dict[str, dict[str, dict[str, float]]]


def score_models(
    heldout: Sequence[Segment],
    fvehicle: Vehicle,
    models: Mapping[str, tuple[Nominal, Residual | None]],
    truth: str,
    pose_segments: Sequence[pose_check.PoseSegment] = (),
) -> Scored:
    """Score `models` on the windows of the held-out segments (whose `motion` is the truth
    `truth`). The same windows serve every model."""
    es = eval_set([s for s in heldout if s.run.has_odom])
    if len(es) == 0:
        raise ValueError("no held-out windows")
    strata = assign_strata(es.direction, es.start_speed)
    errs = {n: window_errors(es, nom, fvehicle, res) for n, (nom, res) in models.items()}
    macro = {n: macro_triplet(es, strata, e) for n, e in errs.items()}
    yp_seg = {
        n: pose_check.yp_by_segment(pose_check.YawModel(nom, res, fvehicle), pose_segments)
        for n, (nom, res) in models.items()
    }
    yp = {
        n: pose_check.yp_macro(pose_check.YawModel(nom, res, fvehicle), pose_segments)
        for n, (nom, res) in models.items()
    }
    return Scored(
        truth=truth,
        windows=len(es),
        strata_names=tuple(strata.names),
        strata_windows=dict(strata.counts),
        merges=tuple(strata.merges),
        macro=macro,
        stratum_pos_1s=per_stratum_table({n: e[1.0]["pos"] for n, e in errs.items()}, strata),
        errs=errs,
        strata=strata,
        block=es.block,
        yp=yp,
        yp_segments=yp_seg,
    )


def bootstrap_p1_ratio(scored: Scored, a: str, b: str) -> dict[str, float]:
    """Paired block bootstrap (5 s blocks, 2000 resamples, seed 0) of P1(a) / P1(b)."""
    return block_bootstrap_ratio(
        scored.errs[a][1.0]["pos"], scored.errs[b][1.0]["pos"], scored.strata, scored.block
    )


def stratum_pos(scored: Scored, name: str) -> dict[str, float]:
    return stratum_rmse(scored.errs[name][1.0]["pos"], scored.strata)


def at1(scored: Scored, name: str) -> dict[str, float]:
    m = scored.macro[name][1.0]
    return {"P1": m["P"], "Y1": m["Y"], "S1": m["S"]}


# ---- M-B1 ----------------------------------------------------------------------------------


def _macro_rows(scored: Scored) -> list[str]:
    rows = ["| model | 0.5 s | 1.0 s | 1.5 s | Yp (rad) |", "|---|---|---|---|---|"]
    for name, m in scored.macro.items():
        cells = " | ".join(
            f"{m[h]['P']:.3f} / {m[h]['Y']:.3f} / {m[h]['S']:.3f}" for h in HORIZONS_S
        )
        rows.append(f"| {name} | {cells} | {scored.yp.get(name, float('nan')):.4f} |")
    return rows


def _stratum_rows(scored: Scored) -> list[str]:
    models = [k for k in next(iter(scored.stratum_pos_1s.values())) if k != "windows"]
    rows = [
        "| stratum | windows | " + " | ".join(models) + " |",
        "|---|---|" + "---|" * len(models),
    ]
    for name, row in scored.stratum_pos_1s.items():
        rows.append(
            f"| {name} | {row['windows']} | " + " | ".join(f"{row[m]:.3f}" for m in models) + " |"
        )
    return rows


def render_mb1(t2: Scored, t1: Scored, notes: Sequence[str], ratio_names: tuple[str, str]) -> str:
    """The M-B1 markdown: dyn-v0.1 and dyn-v0.1r under T2 (primary) and under T1 (labelled)."""
    a, b = ratio_names
    lines = [
        "# M-B1: dyn-v0.1 and the v0.2 residual dyn-v0.1r, rescored (before any v0.3 fit)",
        "",
        (
            "Pre-registration v0.3 section 2.2. Held-out segments of the v0.2 split, windows, strata, "
            "merging, macro averaging and horizons exactly as v0.2 section 3. Truth T2 (wheel seeded "
            "ICP) is primary; T1 (identity seeded ICP, the v0.2 truth) is printed labelled and "
            "decides nothing. No v0.3 model existed when this table was produced."
        ),
        "",
        *[f"- {n}" for n in notes],
        "",
        (
            "Cells: macro position RMSE (m) / yaw RMSE (rad) / wheel speed RMSE (m/s). Yp: RMS "
            "of (model yaw change minus /pose yaw change) over consecutive pose pairs, mean of "
            "G1b and T2b."
        ),
        "",
    ]
    for sc, label in ((t2, "T2 (primary)"), (t1, "T1 (labelled, v0.2 truth, decides nothing)")):
        r = at1(sc, a), at1(sc, b)
        lines += [
            f"## Truth {label}",
            "",
            f"Held-out windows: {sc.windows}. Strata after merging: {', '.join(sc.strata_names)}.",
            "",
            *_macro_rows(sc),
            "",
            (
                f"Ratios {b} / {a}: P1 {r[1]['P1'] / r[0]['P1']:.4f}, Y1 "
                f"{r[1]['Y1'] / r[0]['Y1']:.4f}, S1 {r[1]['S1'] / r[0]['S1']:.4f}."
            ),
            "",
            "Per stratum position RMSE at 1.0 s (m):",
            "",
            *_stratum_rows(sc),
            "",
            "Strata merges: "
            + (
                "; ".join(
                    f"{m['direction']} {m['from']} ({m['windows']} windows) into {m['into']}"
                    for m in sc.merges
                )
                or "none"
            )
            + ".",
            "",
            "Yp per segment (rad, usable pose intervals):",
            "",
            *[
                f"- {n}: "
                + ", ".join(
                    f"{seg} {v['yp']:.4f} ({int(v['intervals'])})" for seg, v in per.items()
                )
                for n, per in sc.yp_segments.items()
            ],
            "",
        ]
    return "\n".join(lines)
