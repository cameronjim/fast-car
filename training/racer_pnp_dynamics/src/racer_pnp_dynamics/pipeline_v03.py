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

from . import fit, pose_check
from .dataset import Segment, by_split
from .evaluate import HORIZONS_S, eval_set, window_errors
from .identify import assess
from .model import Nominal, Residual
from .pipeline_v02 import macro_triplet
from .strata import assign_strata, block_bootstrap_ratio, per_stratum_table, stratum_rmse
from .vehicle import Vehicle
from .windows import yaw_data

TRUTH_T2 = "T2"
TRUTH_T1 = "T1"

# evaluation/prereg/pnp-v0.3.yaml dynamics.yaw_refit_free_parameters. Every other field of the
# nominal stays at its dyn-v0.1 value (the base nominal is read from the bundle manifest).
YAW_REFIT_FREE = (
    "steer_gain_left",
    "steer_gain_right",
    "steer_offset_rad",
    "understeer_s2_per_m2",
)
PROFILE_POINTS = 21


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


# ---- candidate Y: the yaw refit -------------------------------------------------------------


@dataclass
class YawRefit:
    """The fitted nominal of candidate Y and what the report says about each free parameter."""

    base: Nominal
    nominal: Nominal
    free: tuple[str, ...]
    bounds: dict[str, tuple[float, float]]
    on_bound: list[str]
    unidentified: dict[str, str]
    flags: dict[str, str]
    profiles: dict[str, list[dict[str, float]]]
    yaw_rmse_val: dict[str, float]  # per 0.1 s interval on the validation part: base, refit
    counts: dict[str, int]


def frozen_unchanged(base: Nominal, fitted: Nominal, free: Sequence[str]) -> list[str]:
    """Names of the parameters that are NOT free and differ from the base (must be empty)."""
    a, b = base.to_dict(), fitted.to_dict()
    return [k for k in a if k not in free and a[k] != b[k]]


def yaw_refit_fit(
    segments: Sequence[Segment],
    fvehicle: Vehicle,
    base: Nominal,
    free: Sequence[str] = YAW_REFIT_FREE,
) -> YawRefit:
    """Refit `free` (the four yaw parameters) on the train role, truth = the motion the segments
    carry (T2), every other parameter held at `base` (dyn-v0.1). The steering time constant is the
    one fit_yaw parameter that is not free here, so it is pinned. Profiles (validation yaw error
    against each parameter, the other three refit) are on the internal validation role."""
    train = [s for s in by_split(segments, "train") if s.run.has_odom and s.motion is not None]
    val = [s for s in by_split(segments, "val") if s.run.has_odom and s.motion is not None]
    pinned = {n: getattr(base, n) for n in fit.YAW_PARAMS if n not in free}
    fitted, flags = fit.fit_yaw(train, fvehicle, base, None, tied=False, fixed=pinned)
    changed = frozen_unchanged(base, fitted, free)
    if changed:  # pragma: no cover - fit_yaw replaces only the free names
        raise RuntimeError(f"parameters that must stay frozen changed: {changed}")
    delay = base.command_delay_s
    val_data = yaw_data(val, delay)
    bounds = {n: fit.BOUNDS[n] for n in free}
    profiles: dict[str, list[dict[str, float]]] = {}
    unidentified: dict[str, str] = {}
    for name in free:
        lo, hi = bounds[name]
        grid = np.linspace(lo, hi, PROFILE_POINTS)
        errs = []
        for x in grid:
            p, _ = fit.fit_yaw(
                train, fvehicle, base, None, tied=False, fixed={**pinned, name: float(x)}
            )
            errs.append(fit.yaw_rmse(val_data, p, fvehicle))
        best = float(np.nanmin(errs))
        profiles[name] = [
            {"value": float(x), "rmse": float(e), "over_min": float(e / best - 1.0)}
            for x, e in zip(grid, errs, strict=True)
        ]
        verdict = assess(getattr(fitted, name), bounds[name], list(grid), errs, False, True)
        if not verdict.identified:
            unidentified[name] = verdict.text()
    for name, why in flags.items():
        if name in free and why.startswith(("on a bound", "not identified")):
            unidentified[name] = "; ".join(x for x in (unidentified.get(name), why) if x)
    on_bound = [n for n in free if flags.get(n, "").startswith("on a bound")]
    return YawRefit(
        base=base,
        nominal=fitted,
        free=tuple(free),
        bounds=bounds,
        on_bound=on_bound,
        unidentified=unidentified,
        flags=dict(flags),
        profiles=profiles,
        yaw_rmse_val={
            "base": fit.yaw_rmse(val_data, base, fvehicle),
            "refit": fit.yaw_rmse(val_data, fitted, fvehicle),
        },
        counts={
            "yaw_intervals_train": len(yaw_data(train, delay)),
            "yaw_intervals_val": len(val_data),
            "train_segments": len(train),
            "val_segments": len(val),
        },
    )


CANDIDATE_STATEMENT = (
    "candidate, not confirmed: needs the fresh confirmation of pre-registration 2.4"
)
DO_NOT_DEPLOY = "do-not-deploy"
BASE_NAME = "dyn-v0.1"


def scored_summary(sc: Scored) -> dict[str, Any]:
    """The JSON-able part of a Scored: windows, strata, macro at each horizon, per stratum position
    RMSE at 1.0 s, Yp."""
    return {
        "truth": sc.truth,
        "windows": sc.windows,
        "strata": {"names": list(sc.strata_names), "windows": sc.strata_windows},
        "merges": list(sc.merges),
        "macro": {n: {str(h): v for h, v in m.items()} for n, m in sc.macro.items()},
        "per_stratum_position_rmse_1s_m": sc.stratum_pos_1s,
        "yp": sc.yp,
        "yp_segments": sc.yp_segments,
    }


def evaluate_y(
    refit: YawRefit,
    held_t2: Sequence[Segment],
    held_t1: Sequence[Segment],
    fvehicle: Vehicle,
    poses_t2: Sequence[pose_check.PoseSegment],
    poses_t1: Sequence[pose_check.PoseSegment],
) -> dict[str, Any]:
    """Stage 2 of candidate Y: held-out scored ONCE under T2 (the decision) and under T1
    (labelled, decides nothing). Returns the metrics with the GP2c-Y decision."""
    from . import gp2

    models = {BASE_NAME: (refit.base, None), "Y": (refit.nominal, None)}
    sc2 = score_models(held_t2, fvehicle, models, TRUTH_T2, poses_t2)
    sc1 = score_models(held_t1, fvehicle, models, TRUTH_T1, poses_t1)
    boot = bootstrap_p1_ratio(sc2, "Y", BASE_NAME)
    decision = gp2.decide_yaw_refit(
        at1(sc2, BASE_NAME),
        at1(sc2, "Y"),
        boot["hi"],
        stratum_pos(sc2, BASE_NAME),
        stratum_pos(sc2, "Y"),
        sc2.yp[BASE_NAME],
        sc2.yp["Y"],
        refit.on_bound,
    )
    return {
        "T2": scored_summary(sc2),
        "T1": scored_summary(sc1),
        "bootstrap_p1_ratio_Y_over_v01": boot,
        "decision_GP2c_Y": decision.to_dict(),
        "passes": decision.passed,
    }


def evaluate_r(
    nominal_name: str,
    nominal: Nominal,
    candidates: Sequence[Any],
    shipping: Any,
    held_t2: Sequence[Segment],
    held_t1: Sequence[Segment],
    fvehicle: Vehicle,
    exported_vehicle: Vehicle,
    v_max_mps: float,
    poses_t2: Sequence[pose_check.PoseSegment],
    poses_t1: Sequence[pose_check.PoseSegment],
) -> dict[str, Any]:
    """Stage 2 of candidate R: the three seeds of the shipping config on top of `nominal`, scored
    ONCE under T2 (decision) and T1 (labelled)."""
    from . import gp2

    by_seed = {c.seed: c for c in candidates if c.config.name == shipping.config.name}
    models: dict[str, tuple[Nominal, Residual | None]] = {nominal_name: (nominal, None)}
    for seed, c in sorted(by_seed.items()):
        models[f"R seed {seed}"] = (nominal, c.residual)
    ship = f"R seed {shipping.seed}"
    sc2 = score_models(held_t2, fvehicle, models, TRUTH_T2, poses_t2)
    sc1 = score_models(held_t1, fvehicle, models, TRUTH_T1, poses_t1)
    boot = bootstrap_p1_ratio(sc2, ship, nominal_name)
    sweep = gp2.stability_sweep(nominal, shipping.residual, exported_vehicle, v_max_mps)
    decision = gp2.decide_yaw_residual(
        at1(sc2, nominal_name),
        at1(sc2, ship),
        boot["hi"],
        {seed: at1(sc2, f"R seed {seed}")["P1"] for seed in sorted(by_seed)},
        stratum_pos(sc2, nominal_name),
        stratum_pos(sc2, ship),
        sweep,
        sc2.yp[nominal_name],
        sc2.yp[ship],
    )
    return {
        "nominal": nominal_name,
        "T2": scored_summary(sc2),
        "T1": scored_summary(sc1),
        "bootstrap_p1_ratio_R_over_nominal": boot,
        "sweep": sweep.to_dict(),
        "shipping_seed": shipping.seed,
        "shipping_config": shipping.config.name,
        "decision_GP2c_R": decision.to_dict(),
        "passes": decision.passed,
    }


def plot_yaw_profiles(path, refit: YawRefit, title: str) -> None:  # pragma: no cover - plotting
    """One panel per free parameter: validation yaw RMSE against the parameter, the fitted value
    and the base value marked, the bounds as the axis ends."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, len(refit.free), figsize=(4.0 * len(refit.free), 3.4))
    for ax, name in zip(axes, refit.free, strict=True):
        rows = refit.profiles[name]
        ax.plot([r["value"] for r in rows], [r["rmse"] for r in rows], marker="o", ms=3)
        ax.axvline(getattr(refit.nominal, name), color="C1", label="refit")
        ax.axvline(getattr(refit.base, name), color="C2", ls="--", label="dyn-v0.1")
        ax.set_xlabel(name)
        ax.set_ylabel("validation yaw RMSE per 0.1 s (rad)")
        if name in refit.on_bound:
            ax.set_title("on a bound", color="C3")
    axes[0].legend()
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
