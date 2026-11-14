"""dyn-v0.2: refit, residual selection, pre-registered evaluation, export.

Two stages, so that nothing fitted can depend on a held-out number:

1. `fit_stage` uses only the roles `train` and `val` (and the legacy train part of the older
   bags): the nominal fit (the command delay is chosen on `val`, not on held-out as v0.1 did,
   which makes the comparison slightly generous to v0.1), the dead zone, the validity speed, and
   the residual candidates with their seeds, selected on validation P1. It returns a `Frozen`.
2. `evaluate_and_export` scores the frozen models on the `heldout` segments exactly as the
   pre-registration says (strata, macro P1 / Y1 / S1, paired block bootstrap, the GP2 decisions),
   writes the bundle, the pointer and the report.

Pre-registration: docs/notes/preregistration-pnp-v0.2.md section 3.
"""

from __future__ import annotations

import json
import logging
import shutil
from collections.abc import Sequence
from dataclasses import dataclass, field, replace
from itertools import pairwise
from pathlib import Path
from typing import Any

import numpy as np

from . import deadzone, export, gp2
from .bags import Run
from .dataset import Segment, by_split
from .evaluate import (
    HORIZONS_S,
    EvalSet,
    compare_variants,
    eval_set,
    window_errors,
)
from .fit import FitResult, fit_nominal, launches_in_segments
from .identify import steering_lag_s
from .model import Nominal, Residual
from .pipeline import _json, fit_vehicle_of, planner_min_radius
from .pnp_eval import errors_on
from .residual import SEEDS, build_samples
from .residual_train import (
    DEFAULT_CANDIDATES,
    Candidate,
    ResidualConfig,
    disabled_residual,
    fit_candidate,
    multi_step_data,
    select_candidate,
    validation_p1,
)
from .split import Split
from .strata import (
    Strata,
    assign_strata,
    block_bootstrap_ratio,
    macro_rmse,
    per_stratum_table,
    stratum_rmse,
    window_counts,
)
from .vehicle import Vehicle
from .windows import speed_rollout, speed_windows

LOG = logging.getLogger(__name__)

V_MAX_ROUND_MPS = 0.05
# Brief 23: if the 99th percentile lands above 1.5 m/s the validity is capped there (raising the
# ceiling above 1.5 is an owner decision).
V_MAX_CAP_MPS = 1.5
TRAIN_STRIDE_S = 0.1  # residual training windows every scan
TRAIN_MIN_START_MPS = 0.1
HIST_BIN_MPS = 0.25
HIST_MAX_MPS = 2.5
REFIT_NOT_BETTER = "not better than dyn-v0.1 on held-out, do not deploy"


@dataclass
class Frozen:
    """Everything fixed before the held-out segments are scored."""

    fit: FitResult
    nominal: Nominal
    v_max_mps: float
    validity: dict[str, Any]
    dead_zone: dict[str, Any]
    lag_check: dict[str, Any]
    candidates: list[Candidate]
    selection: dict[str, dict[str, float]]
    shipping: Candidate
    nominal_val: dict[str, float]
    samples: dict[str, int]
    config_names: list[str] = field(default_factory=list)


def v_max_of(segments: Sequence[Segment], vehicle: Vehicle) -> dict[str, Any]:
    """Validity speed: the 99th percentile of the true wheel speed over the (train) segments,
    rounded down to V_MAX_ROUND_MPS, never above the full scale speed and capped at
    V_MAX_CAP_MPS. Also the ICP ground speed percentile and the histograms by direction."""
    wheel: dict[str, list[np.ndarray]] = {}
    ground: list[np.ndarray] = []
    for seg in segments:
        if not seg.run.has_odom:
            continue
        v = seg.run.wheel_speed_mps[seg.i0 : seg.i1]
        wheel.setdefault(seg.direction or "none", []).append(v[np.isfinite(v)])
        m = seg.motion
        if m is not None:
            sel = m.ok & (m.t_s >= seg.t0_s) & (m.t_s <= seg.t1_s - 0.1)
            ground.append(np.hypot(m.dx_m[sel], m.dy_m[sel]) / m.dt_s[sel])
    all_v = np.concatenate([x for xs in wheel.values() for x in xs])
    p99 = float(np.percentile(all_v, 99))
    rounded = float(np.floor(p99 / V_MAX_ROUND_MPS + 1e-9) * V_MAX_ROUND_MPS)
    full = vehicle.throttle_full_scale_mps
    capped = rounded > V_MAX_CAP_MPS
    v_max = round(min(rounded, V_MAX_CAP_MPS, full), 2)
    edges = np.arange(0.0, HIST_MAX_MPS + 1e-9, HIST_BIN_MPS)
    hist = {
        d: {
            f"{lo:.2f}-{hi:.2f}": round(float(np.sum((v >= lo) & (v < hi))) / 50.0, 1)
            for lo, hi in pairwise(edges)
        }
        for d, xs in wheel.items()
        for v in [np.concatenate(xs)]
    }
    return {
        "v_max_mps": v_max,
        "p99_wheel_speed_train_mps": p99,
        "p99_rounded_down_mps": rounded,
        "capped_at_mps": V_MAX_CAP_MPS if capped else None,
        "p99_icp_ground_speed_train_mps": (
            float(np.percentile(np.concatenate(ground), 99)) if ground else None
        ),
        "speed_histogram_seconds_by_direction": hist,
        "histogram_bin_mps": HIST_BIN_MPS,
    }


def _slice_run(seg: Segment) -> Run:
    r = seg.run
    s = slice(seg.i0, seg.i1)
    return Run(
        r.name, r.t_s[s], r.speed_cmd_mps[s], r.steering_cmd_rad[s], r.wheel_speed_mps[s], []
    )


def dead_zone_report(
    segments: Sequence[Segment], result: FitResult, vehicle: Vehicle
) -> dict[str, Any]:
    """The two direct dead zone estimates over the given (train and val) segments only."""
    odom = [s for s in segments if s.run.has_odom]
    delay = result.nominal.command_delay_s
    launches = launches_in_segments(odom, delay)
    bins = deadzone.steady_bins([_slice_run(s) for s in odom])
    brackets = deadzone.steady_brackets(bins)
    fitted = result.nominal.speed_deadband_mps
    lo, hi = brackets["largest_still_bin_hi_mps"], brackets["smallest_moving_bin_lo_mps"]
    inside = (lo is None or fitted >= lo) and (hi is None or fitted <= hi)
    windows = speed_windows(odom, delay)
    model_end = speed_rollout(windows, result.nominal, vehicle)[:, -1] if len(windows) else None
    t0 = {id(s.run): float(s.run.t_s[0]) for s in odom}
    runs_by_name = {s.run.name: s.run for s in odom}
    return {
        "fitted_mps": fitted,
        "launches_from_rest": deadzone.launch_summary(launches),
        "launch_list": [
            {
                "bag": x.run_name,
                "t_s": round(x.t_s - t0[id(runs_by_name[x.run_name])], 1),
                "cmd_launch_mps": x.cmd_launch_mps,
                "cmd_rest_max_mps": x.cmd_rest_max_mps,
            }
            for x in launches
        ],
        "steady_state": brackets,
        "steady_state_bins": bins,
        "fitted_inside_steady_bracket": bool(inside),
        "rolling_below_the_dead_zone": deadzone.rolling_below_the_dead_zone(
            windows, fitted, model_end
        ),
        "choice": result.deadband_choice,
    }


def lag_report(segments: Sequence[Segment], nominal: Nominal) -> dict[str, Any]:
    """Independent estimate of the steering lag: peak of the cross-correlation of the steering
    command with ICP path curvature at speed above 0.4 m/s (train and val segments only). Not
    used by the fit; reported next to the fitted delay plus steer_tau, never tuned to."""
    lag = steering_lag_s(segments)
    out: dict[str, Any] = {"available": lag is not None}
    if lag is not None:
        fitted = nominal.command_delay_s + nominal.steer_tau_s
        out.update(
            peak_lag_s=lag.lag_s,
            peak_correlation=lag.peak_correlation,
            intervals=lag.n_intervals,
            fitted_delay_plus_steer_tau_s=fitted,
            difference_s=lag.lag_s - fitted,
            lags_s=list(lag.lags_s),
            correlation=list(lag.correlation),
        )
    return out


def candidate_residuals(
    configs: Sequence[ResidualConfig],
    seeds: Sequence[int],
    nominal: Nominal,
    fvehicle: Vehicle,
    train: Sequence[Segment],
    val: Sequence[Segment],
    one_step_trainer_fn=None,
    log: bool = True,
) -> tuple[list[Candidate], dict[str, int]]:
    """Every (config, seed) trained on train segments only and scored on validation windows."""
    train_odom = [s for s in train if s.run.has_odom]
    val_odom = [s for s in val if s.run.has_odom]
    s_train = build_samples(train_odom, nominal, fvehicle)
    s_val = build_samples(val_odom, nominal, fvehicle)
    val_set = eval_set(val_odom)
    needs_ms = any(c.mode != "one_step" for c in configs)
    train_ms = val_ms = None
    if needs_ms:
        train_ms = multi_step_data(
            eval_set(train_odom, TRAIN_STRIDE_S, TRAIN_MIN_START_MPS), nominal, fvehicle
        )
        val_ms = multi_step_data(
            eval_set(val_odom, TRAIN_STRIDE_S, TRAIN_MIN_START_MPS), nominal, fvehicle
        )
    out = []
    for cfg in configs:
        for seed in seeds:
            cand = fit_candidate(
                cfg, seed, nominal, fvehicle, s_train, s_val, val_set, train_ms, val_ms,
                one_step_trainer_fn,
            )  # fmt: skip
            if log:
                LOG.info(
                    "candidate %s seed %d: validation P1 %.4f m", cfg.name, seed, cand.val_p1_m
                )
            out.append(cand)
    counts = {
        "one_step_samples_train": len(s_train),
        "one_step_samples_val": len(s_val),
        "windows_train": 0 if train_ms is None else len(train_ms),
        "windows_val_loss": 0 if val_ms is None else len(val_ms),
        "windows_val_p1": len(val_set),
    }
    return out, counts


def macro_triplet(es: EvalSet, strata: Strata, errs: dict[float, dict[str, np.ndarray]]) -> dict:
    """Macro P / Y / S at each horizon: {h: {"P": m, "Y": rad, "S": m/s}}."""
    return {
        h: {
            "P": macro_rmse(e["pos"], strata),
            "Y": macro_rmse(e["yaw"], strata),
            "S": macro_rmse(e["speed"], strata),
        }
        for h, e in errs.items()
    }


def fit_stage(
    segments: Sequence[Segment],
    vehicle: Vehicle,
    *,
    delays: Sequence[float] | None = None,
    tau_grid: Sequence[float] | None = None,
    configs: Sequence[ResidualConfig] = DEFAULT_CANDIDATES,
    seeds: Sequence[int] = SEEDS,
    one_step_trainer_fn=None,
) -> Frozen:
    """Nominal fit and residual selection on train and val data only."""
    fvehicle = fit_vehicle_of(vehicle)
    kwargs: dict[str, Any] = {} if delays is None else {"delays": delays}
    if tau_grid is not None:
        kwargs["tau_grid"] = tau_grid
    result = fit_nominal(
        segments, fvehicle, fit_split="train", score_split="val",
        launch_splits=("train", "val"), **kwargs,
    )  # fmt: skip
    nominal = result.nominal
    train, val = by_split(segments, "train"), by_split(segments, "val")
    validity = v_max_of([*train, *val], vehicle)
    candidates, counts = candidate_residuals(
        configs, seeds, nominal, fvehicle, train, val, one_step_trainer_fn
    )
    _, shipping, table = select_candidate(candidates)
    val_set = eval_set([s for s in val if s.run.has_odom])
    nominal_p1 = validation_p1(nominal, None, fvehicle, val_set)
    return Frozen(
        fit=result,
        nominal=nominal,
        v_max_mps=validity["v_max_mps"],
        validity=validity,
        dead_zone=dead_zone_report([*train, *val], result, fvehicle),
        lag_check=lag_report([*train, *val], nominal),
        candidates=candidates,
        selection={**table, "_nominal_validation_P1": {"value": nominal_p1}},
        shipping=shipping,
        nominal_val={"P1": nominal_p1},
        samples=counts,
        config_names=[c.name for c in configs],
    )


def check_segments_against_split(segments: Sequence[Segment], split: Split) -> int:
    """Every segment of a segmented bag lies inside one split segment whose role matches (train
    and val: train, heldout: heldout). Raises ValueError otherwise. Returns the number checked."""
    expect = {"train": "train", "val": "train", "heldout": "heldout"}
    checked = 0
    for s in segments:
        entry = split.entry(s.run.name)
        if entry is None:
            raise ValueError(f"{s.run.name} is not in the split file")
        if entry.whole_bag_role is not None:
            continue
        t_ref = float(s.run.scans[0].log_time_s)
        role = split.role_of(s.run.name, s.t0_s - t_ref, s.t1_s - t_ref)
        if role != expect[s.split]:
            raise ValueError(
                f"{s.run.name} [{s.t0_s - t_ref:.1f}, {s.t1_s - t_ref:.1f}] is {s.split} but "
                f"the split file says {role}"
            )
        checked += 1
    return checked


def data_table(segments: Sequence[Segment]) -> dict[str, Any]:
    """Seconds of data per role per bag, and per role by direction and start-speed bin (the
    seconds of grid samples whose wheel speed lies in the bin)."""
    from .strata import SPEED_BIN_EDGES_MPS, bin_label

    by_bag: dict[str, dict[str, float]] = {}
    cells: dict[str, dict[str, dict[str, float]]] = {}
    edges = (0.0, *SPEED_BIN_EDGES_MPS, np.inf)
    labels = ["<0.25"] + [bin_label(b, b) for b in range(len(SPEED_BIN_EDGES_MPS))]
    for s in segments:
        row = by_bag.setdefault(s.run.name, {})
        row[s.split] = round(row.get(s.split, 0.0) + s.duration_s, 1)
        v = s.run.wheel_speed_mps[s.i0 : s.i1]
        v = v[np.isfinite(v)]
        cell = cells.setdefault(s.split, {}).setdefault(s.direction or "none", {})
        for lab, lo, hi in zip(labels, edges[:-1], edges[1:], strict=True):
            cell[lab] = round(cell.get(lab, 0.0) + float(np.sum((v >= lo) & (v < hi))) / 50.0, 1)
    return {"seconds_by_bag": by_bag, "seconds_by_role_direction_speed_bin": cells}


def named_spans(segments: Sequence[Segment]) -> dict[str, list[Segment]]:
    """Segments grouped by the split segment name they came from, each group's pieces merged
    into one segment spanning them (train and val pieces of one run join up again)."""
    groups: dict[tuple[str, str], list[Segment]] = {}
    for s in segments:
        if s.name:
            groups.setdefault((s.run.name, s.name), []).append(s)
    out: dict[str, list[Segment]] = {}
    for (_, name), pieces in groups.items():
        first = pieces[0]
        merged = replace(
            first,
            i0=min(p.i0 for p in pieces),
            i1=max(p.i1 for p in pieces),
            split="heldout" if first.split == "heldout" else "train",
        )
        out.setdefault(name, []).append(merged)
    return out


def run_table(
    segments: Sequence[Segment],
    bag: str,
    variants: dict[str, tuple[Nominal, Residual | None]],
    vehicle: Vehicle,
) -> dict[str, Any]:
    """Per run (R0 to R8: the split's pnp segments of `bag`, R7a and R7b joined as R7) open loop
    error at 0.5, 1.0 and 1.5 s, standing starts included, for each variant. Rows say which role
    the run has: `train` runs are in-sample for the refit."""
    spans = {
        n: v
        for n, v in named_spans([s for s in segments if s.run.name == bag]).items()
        if n.startswith("R")
    }
    runs: dict[str, list[Segment]] = {}
    roles: dict[str, str] = {}
    for name, segs in spans.items():
        key = name.rstrip("ab")
        runs.setdefault(key, []).extend(segs)
        roles[key] = "+".join(sorted({*roles.get(key, "").split("+"), segs[0].split} - {""}))
    out: dict[str, Any] = {}
    for key in sorted(runs, key=lambda k: int(k[1:])):
        out[key] = {
            "role": roles[key],
            "variants": {
                name: errors_on(runs[key], nom, res, vehicle)
                for name, (nom, res) in variants.items()
            },
        }
    return out


def evaluate_heldout(
    frozen: Frozen,
    heldout: Sequence[Segment],
    fvehicle: Vehicle,
    exported_vehicle: Vehicle,
    v01: tuple[Nominal, Residual],
    v01_id: str,
) -> dict[str, Any]:
    """The pre-registered evaluation of section 3 on the held-out segments."""
    es = eval_set([s for s in heldout if s.run.has_odom])
    if len(es) == 0:
        raise ValueError("no held-out windows")
    strata = assign_strata(es.direction, es.start_speed)
    nominal = frozen.nominal
    by_seed = {c.seed: c for c in frozen.candidates if c.config.name == frozen.shipping.config.name}
    models: dict[str, tuple[Nominal, Residual | None]] = {
        v01_id: (v01[0], None),
        "dyn-v0.2 nominal": (nominal, None),
        **{
            f"dyn-v0.2 + residual seed {seed}": (nominal, c.residual)
            for seed, c in sorted(by_seed.items())
        },
    }
    errs = {name: window_errors(es, nom, fvehicle, res) for name, (nom, res) in models.items()}
    macro = {name: macro_triplet(es, strata, e) for name, e in errs.items()}
    ship_name = f"dyn-v0.2 + residual seed {frozen.shipping.seed}"
    nom_name = "dyn-v0.2 nominal"

    def at1(name: str) -> dict[str, float]:
        m = macro[name][1.0]
        return {"P1": m["P"], "Y1": m["Y"], "S1": m["S"]}

    refit = gp2.decide_refit(at1(nom_name), at1(v01_id))
    boot_refit = block_bootstrap_ratio(
        errs[nom_name][1.0]["pos"], errs[v01_id][1.0]["pos"], strata, es.block
    )
    boot_res = block_bootstrap_ratio(
        errs[ship_name][1.0]["pos"], errs[nom_name][1.0]["pos"], strata, es.block
    )
    res = frozen.shipping.residual
    sweep = gp2.stability_sweep(nominal, res, exported_vehicle, frozen.v_max_mps)
    seed_p1 = {seed: at1(f"dyn-v0.2 + residual seed {seed}")["P1"] for seed in sorted(by_seed)}
    strat_nom = stratum_rmse(errs[nom_name][1.0]["pos"], strata)
    strat_res = stratum_rmse(errs[ship_name][1.0]["pos"], strata)
    residual_decision = gp2.decide_residual(
        at1(nom_name), at1(ship_name), boot_res["hi"], seed_p1, strat_nom, strat_res, sweep
    )
    pos_table = per_stratum_table({name: e[1.0]["pos"] for name, e in errs.items()}, strata)
    return {
        "windows": len(es),
        "window_rule": (
            "every 0.25 s inside a held-out segment (snapped to the nearest scan), start wheel "
            "speed at least 0.25 m/s, 1.5 s of data after the start for every horizon"
        ),
        "horizons_s": list(HORIZONS_S),
        "strata": {
            "names": list(strata.names),
            "windows": strata.counts,
            "windows_by_original_bin": window_counts(es.direction, es.start_speed),
            "merges": list(strata.merges),
            "unmerged_small": list(strata.unmerged_small),
        },
        "macro": macro,
        "per_stratum_position_rmse_1s_m": pos_table,
        "per_stratum_yaw_rmse_1s_rad": per_stratum_table(
            {n: e[1.0]["yaw"] for n, e in errs.items()}, strata
        ),
        "per_stratum_speed_rmse_1s_mps": per_stratum_table(
            {n: e[1.0]["speed"] for n, e in errs.items()}, strata
        ),
        "bootstrap_p1_ratio_v02_nominal_over_v01": boot_refit,
        "bootstrap_p1_ratio_residual_over_nominal": boot_res,
        "decision_3_1_refit": refit.to_dict(),
        "decision_3_2_residual": residual_decision.to_dict(),
        "refit_ships": refit.passed,
        "residual_ships": residual_decision.passed,
        "sweep": sweep.to_dict(),
        "shipping_seed": frozen.shipping.seed,
        "shipping_config": frozen.shipping.config.name,
        "exported_limits": _exported_limits(es, models, exported_vehicle),
    }


def _exported_limits(
    es: EvalSet, models: dict[str, tuple[Nominal, Residual | None]], vehicle: Vehicle
) -> dict[str, Any]:
    """Context: macro P1 / Y1 / S1 with the vehicle_params steering limits instead of the
    physical stops the fit and the decisions use."""
    strata = assign_strata(es.direction, es.start_speed)
    out = {}
    for name, (nom, res) in models.items():
        e = window_errors(es, nom, vehicle, res, (1.0,))
        m = macro_triplet(es, strata, e)[1.0]
        out[name] = {"P1": m["P"], "Y1": m["Y"], "S1": m["S"]}
    return out


def legacy_report(
    legacy: Sequence[Segment],
    fvehicle: Vehicle,
    variants: dict[str, tuple[Nominal, Residual | None]],
) -> dict[str, Any]:
    """The dyn-v0.1 style pooled report on the legacy held-out part of the older bags (their last
    30 percent). Decides nothing; kept so v0.1's numbers stay comparable."""
    odom = [s for s in legacy if s.run.has_odom]
    return {
        "segments": len(odom),
        "errors": compare_variants(odom, fvehicle, variants) if odom else {},
    }


def render_report(metrics: dict[str, Any], model_id: str) -> str:
    """The README of the pointer directory."""
    ev = metrics["heldout"]
    refit_ok = ev["refit_ships"]
    res_ok = ev["residual_ships"]
    lines: list[str] = []
    if not refit_ok:
        lines += [REFIT_NOT_BETTER, ""]
    lines += [
        f"# {model_id}",
        "",
        (
            "Refit of the identified bicycle model on the 2026-11-13 bags with the pre-registered "
            "split (docs/notes/preregistration-pnp-v0.2.md section 3, split file SHA-256 "
            f"`{metrics['split_sha256']}`). Weights are not committed; the bundle lives under "
            "car-data/pnp/models/."
        ),
        "",
        f"- 3.1 refit ships: **{refit_ok}**. 3.2 residual ships enabled: **{res_ok}**.",
        (
            f"- vehicle_params {metrics['vehicle_params']}; validity v up to "
            f"{metrics['validity']['v_max_mps']} m/s."
        ),
        "",
        "## Fitted nominal parameters",
        "",
        "| parameter | dyn-v0.2 | dyn-v0.1 |",
        "|---|---|---|",
        *[
            f"| {k} | {v:.4f} | {metrics['v01_nominal'][k]:.4f} |"
            for k, v in metrics["nominal"].items()
        ],
        "",
        "Unidentified (on a bound or a flat profile):",
        "",
        *(
            [f"- {k}: {v}" for k, v in metrics["unidentified"].items()]
            if metrics["unidentified"]
            else ["- none"]
        ),
        "",
        "## 3.1 Refit decision (held-out segments)",
        "",
        _decision_table(ev["decision_3_1_refit"]),
        "",
        (
            "Macro RMSE (mean over strata of the per stratum RMSE) at each horizon, position (m) "
            "/ yaw (rad) / wheel speed (m/s):"
        ),
        "",
        "| model | 0.5 s | 1.0 s | 1.5 s |",
        "|---|---|---|---|",
        *[
            "| "
            + name
            + " | "
            + " | ".join(
                f"{m[h]['P']:.3f} / {m[h]['Y']:.3f} / {m[h]['S']:.3f}"
                for h in ("0.5", "1.0", "1.5")
            )
            + " |"
            for name, m in ev["macro"].items()
        ],
        "",
        _boot_line("dyn-v0.2 nominal over dyn-v0.1", ev["bootstrap_p1_ratio_v02_nominal_over_v01"]),
        "",
        "## 3.2 GP2a residual decision (held-out segments)",
        "",
        (
            f"Shipping config `{ev['shipping_config']}`, seed {ev['shipping_seed']} (lowest "
            "validation P1, fixed before held-out was read)."
        ),
        "",
        _decision_table(ev["decision_3_2_residual"]),
        "",
        _boot_line(
            "shipping residual over dyn-v0.2 nominal",
            ev["bootstrap_p1_ratio_residual_over_nominal"],
        ),
        "",
        "Validation P1 per candidate (metres, macro, on the internal validation part):",
        "",
        "| config | mean | min | seed 0 | seed 1 | seed 2 |",
        "|---|---|---|---|---|---|",
        *[
            f"| {k} | {v['mean']:.4f} | {v['min']:.4f} | {v.get('seed0', float('nan')):.4f} | "
            f"{v.get('seed1', float('nan')):.4f} | {v.get('seed2', float('nan')):.4f} |"
            for k, v in metrics["selection"].items()
            if not k.startswith("_")
        ],
        "",
        "## Strata (direction by start speed, after merging)",
        "",
        "Per stratum position RMSE at 1.0 s (m):",
        "",
        _stratum_table(ev["per_stratum_position_rmse_1s_m"]),
        "",
        *(
            [
                "Merges: "
                + "; ".join(
                    f"{m['direction']} {m['from']} ({m['windows']} windows) into {m['into']}"
                    for m in ev["strata"]["merges"]
                ),
                "",
            ]
            if ev["strata"]["merges"]
            else ["Merges: none.", ""]
        ),
        (
            "Windows by original bin: "
            + ", ".join(f"{k} {v}" for k, v in ev["strata"]["windows_by_original_bin"].items())
        ),
        "",
        "## Command delay",
        "",
        metrics["delay_note"],
        "",
        "## Data",
        "",
        "```json",
        json.dumps(metrics["data"]["seconds_by_bag"], indent=1),
        "```",
        "",
    ]
    return "\n".join(lines)


def _decision_table(d: dict[str, Any]) -> str:
    rows = ["| item | value | threshold | result |", "|---|---|---|---|"]
    for i in d["items"]:
        rows.append(
            f"| {i['name']} | {i['value']:.4f} | {i['comparison']} {i['threshold']:g} | "
            f"{'pass' if i['passed'] else 'FAIL'} |"
        )
    rows.append("")
    rows.append(f"Gate {d['gate']}: {'PASS' if d['passed'] else 'FAIL'}.")
    return "\n".join(rows)


def _boot_line(what: str, b: dict[str, float]) -> str:
    return (
        f"Paired block bootstrap (5.0 s blocks, {b['resamples']} resamples, seed {b['seed']}), "
        f"P1 ratio {what}: {b['ratio']:.4f}, 95 percent interval [{b['lo']:.4f}, {b['hi']:.4f}]."
    )


def _stratum_table(rows: dict[str, dict[str, float]]) -> str:
    models = [k for k in next(iter(rows.values())) if k != "windows"]
    out = ["| stratum | windows | " + " | ".join(models) + " |", "|---|---|" + "---|" * len(models)]
    for name, row in rows.items():
        out.append(
            f"| {name} | {row['windows']} | " + " | ".join(f"{row[m]:.3f}" for m in models) + " |"
        )
    return "\n".join(out)


def write_outputs(
    frozen: Frozen,
    metrics: dict[str, Any],
    vehicle: Vehicle,
    *,
    model_id: str,
    created_utc: str,
    git_sha: str,
    bundle_dir: Path,
    pointer_dir: Path | None,
    data_lines: Sequence[str],
    profile_png: bool,
) -> dict[str, Any]:
    """The bundle (residual enabled only if GP2a passed), the pointer directory, the README."""
    enabled = bool(metrics["heldout"]["residual_ships"])
    residual = frozen.shipping.residual if enabled else disabled_residual()
    manifest = export.write_bundle(
        bundle_dir,
        model_id=model_id,
        created_utc=created_utc,
        git_sha=git_sha,
        training_config_sha256=export.config_sha256(
            {
                "model_id": model_id,
                "nominal": frozen.nominal.to_dict(),
                "seeds": [c.seed for c in frozen.candidates],
                "config": frozen.shipping.config.name,
                "split_sha256": metrics["split_sha256"],
            }
        ),
        vehicle=vehicle,
        nominal=frozen.nominal,
        residual=residual,
        v_max_mps=frozen.v_max_mps,
        data=list(data_lines),
        metrics=metrics,
        golden_residual_rollouts=enabled,
    )
    readme = render_report(metrics, model_id)
    if not metrics["heldout"]["refit_ships"]:
        assert readme.splitlines()[0] == REFIT_NOT_BETTER
    (bundle_dir / "README.md").write_text(readme, encoding="utf-8")
    if profile_png:
        from .plots import plot_from_metrics

        plot_from_metrics(
            bundle_dir / "profiles.png", metrics, frozen.nominal.to_dict(),
            f"{model_id}: identification profiles",
        )  # fmt: skip
    if pointer_dir is not None:
        pointer_dir.mkdir(parents=True, exist_ok=True)
        if profile_png:
            shutil.copy(bundle_dir / "profiles.png", pointer_dir / "profiles.png")
        shutil.copy(bundle_dir / export.MANIFEST_FILE, pointer_dir / export.MANIFEST_FILE)
        (pointer_dir / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
        (pointer_dir / "README.md").write_text(readme, encoding="utf-8")
    return manifest


def build_metrics(
    frozen: Frozen,
    heldout_report: dict[str, Any],
    *,
    split: Split,
    segments: Sequence[Segment],
    vehicle: Vehicle,
    fvehicle: Vehicle,
    v01_nominal: Nominal,
    v01_id: str,
    legacy: dict[str, Any],
    runs_table: dict[str, Any] | None,
) -> dict[str, Any]:
    result = frozen.fit
    delay_curve = {str(k): v for k, v in result.delay_curve.items()}
    cross = frozen.lag_check.get("peak_lag_s")
    delay_note = (
        "The command delay is chosen on the internal validation part (role val), NOT on held-out "
        "as dyn-v0.1 did (it had no validation split), so the comparison with dyn-v0.1 is slightly "
        "generous to dyn-v0.1. "
        f"Chosen {frozen.nominal.command_delay_s:g} s ({result.delay_selection}). Independent "
        "cross-correlation estimate of steering lag (steering command against ICP path "
        "curvature, speed above 0.4 m/s)"
        + (
            f": {cross:.3f} s against fitted delay plus steer_tau "
            f"{frozen.lag_check['fitted_delay_plus_steer_tau_s']:.3f} s. Not tuned to."
            if cross is not None
            else ": not available."
        )
    )
    metrics: dict[str, Any] = {
        "split_id": split.split_id,
        "split_sha256": split.sha256,
        "vehicle_params": f"{vehicle.schema_version} / {vehicle.sysid_session_id}",
        "nominal": frozen.nominal.to_dict(),
        "v01_id": v01_id,
        "v01_nominal": v01_nominal.to_dict(),
        "unidentified": result.unidentified,
        "profiles": result.profiles,
        "profile_notes": result.profile_notes,
        "dead_zone": frozen.dead_zone,
        "steering_lag_check": frozen.lag_check,
        "validity": frozen.validity,
        "delay_note": delay_note,
        "fit": {
            "flags": result.flags,
            "delay_selection": result.delay_selection,
            "delay_curve_speed_val_rmse": delay_curve,
            "yaw_training": result.yaw_training,
            "sides_tied": result.sides_tied,
            "yaw_cv_rmse_rad": result.yaw_cv_rmse_rad,
            "accel_rule": result.accel_rule,
            "counts": result.counts,
            "fitted_on": "train role (the first 85 percent of each train segment)",
            "scored_on": "val role (the last 15 percent of each train segment)",
        },
        "selection": frozen.selection,
        "residual_samples": frozen.samples,
        "frozen_before_heldout": {
            "nominal": frozen.nominal.to_dict(),
            "v_max_mps": frozen.v_max_mps,
            "shipping_config": frozen.shipping.config.name,
            "shipping_seed": frozen.shipping.seed,
        },
        "heldout": heldout_report,
        "legacy_heldout": legacy,
        "planner_min_radius_m": planner_min_radius(frozen.nominal, vehicle),
        "data": data_table(segments),
        **({"runs_r0_r8": runs_table} if runs_table is not None else {}),
    }
    return _json(metrics)
