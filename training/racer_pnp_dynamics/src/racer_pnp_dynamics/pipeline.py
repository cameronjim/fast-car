"""Fit, evaluate, decide on the residual, export: the whole offline run on already loaded
segments. `main` (real bags) is thin glue in cli.py; everything here is testable on synthetic
segments with a stand-in trainer."""

from __future__ import annotations

import json
import shutil
from collections import defaultdict
from collections.abc import Sequence
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np

from . import export
from .dataset import HELD_OUT_BAG, Segment, by_split, seconds
from .evaluate import (
    HORIZONS_S,
    compare_variants,
    eval_data,
    ground_speed_ratio,
    zero_yaw_rmse,
)
from .fit import FitResult, fit_nominal
from .model import PLACEHOLDER_NOMINAL, Nominal, Residual
from .residual import (
    SEEDS,
    Trainer,
    build_samples,
    decide,
    relative_improvement,
    shipped,
    train_residual,
)
from .vehicle import Vehicle

V_MAX_ROUND_MPS = 0.05
V_MAX_FACTOR = 1.5


def fit_vehicle_of(vehicle: Vehicle) -> Vehicle:
    """The vehicle used while FITTING: steering limits at the physical stops. Converted steering
    in the old bags reaches 0.504 rad to the left, beyond today's planning limit; the car really
    went there, so the fit clamps at the stops (the exported model uses vehicle_params limits)."""
    return replace(
        vehicle,
        steer_min_rad=vehicle.physical_lock_right_rad,
        steer_max_rad=vehicle.physical_lock_left_rad,
    )


def v_max_of(train: Sequence[Segment], vehicle: Vehicle) -> float:
    """1.5 times the 99th percentile of the true wheel speed in the real training data, rounded
    down to 0.05 and never above the full scale speed."""
    speeds = []
    for seg in train:
        if seg.run.has_odom:
            v = seg.run.wheel_speed_mps[seg.i0 : seg.i1]
            speeds.append(v[np.isfinite(v)])
    p99 = float(np.percentile(np.concatenate(speeds), 99))
    raw = V_MAX_FACTOR * p99
    rounded = np.floor(raw / V_MAX_ROUND_MPS + 1e-9) * V_MAX_ROUND_MPS
    return float(min(rounded, vehicle.throttle_full_scale_mps))


def _json(obj: Any) -> Any:
    if isinstance(obj, dict):
        return {str(k): _json(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json(v) for v in obj]
    if isinstance(obj, (bool, np.bool_)):
        return bool(obj)
    if isinstance(obj, (np.floating, float)):
        return None if not np.isfinite(obj) else float(obj)
    if isinstance(obj, (np.integer, int)):
        return int(obj)
    return obj


def seconds_by_bag(segments: Sequence[Segment]) -> dict[str, dict[str, float]]:
    out: dict[str, dict[str, float]] = defaultdict(
        lambda: {"train": 0.0, "val": 0.0, "heldout": 0.0}
    )
    for s in segments:
        out[s.run.name][s.split] += s.duration_s
    return {k: {a: round(b, 1) for a, b in v.items()} for k, v in out.items()}


def run_fit_and_export(
    segments: Sequence[Segment],
    vehicle: Vehicle,
    *,
    model_id: str,
    created_utc: str,
    git_sha: str,
    bundle_dir: Path,
    pointer_dir: Path | None,
    trainer: Trainer | None = None,
    data_lines: Sequence[str] = (),
    notes: dict[str, Any] | None = None,
    delays: Sequence[float] | None = None,
) -> dict[str, Any]:
    """Everything after the segments exist. Returns the metrics dict that was exported."""
    fvehicle = fit_vehicle_of(vehicle)
    kwargs = {} if delays is None else {"delays": delays}
    result: FitResult = fit_nominal(segments, fvehicle, **kwargs)
    nominal = result.nominal
    train, val = by_split(segments, "train"), by_split(segments, "val")
    heldout = by_split(segments, "heldout", odom_only=True)

    # Residual candidates, one per seed, scored on the held-out windows.
    s_train, s_val = build_samples(train, nominal, fvehicle), build_samples(val, nominal, fvehicle)
    candidates = [train_residual(s_train, s_val, seed, trainer) for seed in SEEDS]
    variants: dict[str, tuple[Nominal, Residual | None]] = {
        "placeholder_nominal": (PLACEHOLDER_NOMINAL, None),
        "fitted_nominal": (nominal, None),
        **{
            f"fitted_plus_residual_seed{seed}": (nominal, c)
            for seed, c in zip(SEEDS, candidates, strict=True)
        },
    }
    errors = compare_variants(heldout, fvehicle, variants)
    improvements = [
        (
            relative_improvement(
                errors["fitted_nominal"][1.0]["speed_rmse_mps"],
                errors[f"fitted_plus_residual_seed{seed}"][1.0]["speed_rmse_mps"],
            ),
            relative_improvement(
                errors["fitted_nominal"][1.0]["yaw_rmse_rad"],
                errors[f"fitted_plus_residual_seed{seed}"][1.0]["yaw_rmse_rad"],
            ),
        )
        for seed in SEEDS
    ]
    enabled, chosen, medians = decide(improvements)
    residual = shipped(candidates[chosen], enabled)

    per_bag: dict[str, Any] = {}
    for name in sorted({s.run.name for s in heldout}):
        subset = [s for s in heldout if s.run.name == name]
        per_bag[name] = compare_variants(
            subset,
            fvehicle,
            {k: v for k, v in variants.items() if not k.endswith(("seed0", "seed1", "seed2"))},
        )
    window_data = eval_data(heldout, nominal.command_delay_s)
    exported_limits = compare_variants(heldout, vehicle, {"fitted_nominal": (nominal, None)})[
        "fitted_nominal"
    ]
    v_max = v_max_of(train, vehicle)

    metrics: dict[str, Any] = {
        "heldout_windows": len(window_data),
        "heldout_horizons_s": list(HORIZONS_S),
        "heldout_errors": errors,
        "heldout_errors_per_bag": per_bag,
        "heldout_fitted_nominal_with_exported_limits": exported_limits,
        "heldout_zero_yaw_baseline_rad": zero_yaw_rmse(window_data),
        "residual_improvement_1s_per_seed": {
            str(seed): {"speed": a, "yaw": b}
            for seed, (a, b) in zip(SEEDS, improvements, strict=True)
        },
        "residual_median_improvement_1s": {"speed": medians[0], "yaw": medians[1]},
        "residual_enabled": enabled,
        "residual_shipped_seed": SEEDS[chosen],
        "residual_samples": {"train": len(s_train), "val": len(s_val)},
        "fit": {
            "flags": result.flags,
            "speed_val_rmse_mps": result.speed_val_rmse_mps,
            "yaw_val_rmse_rad": result.yaw_val_rmse_rad,
            "delay_curve_speed_val_rmse": {str(k): v for k, v in result.delay_curve.items()},
            "counts": result.counts,
        },
        "ground_speed_over_wheel_speed_by_steering": ground_speed_ratio(
            [*train, *val, *heldout], nominal.command_delay_s
        ),
        "seconds_by_bag": seconds_by_bag(segments),
        "driving_seconds": {
            "train": seconds(train),
            "val": seconds(val),
            "heldout": seconds(by_split(segments, "heldout")),
        },
        "held_out_bag": HELD_OUT_BAG,
        **(notes or {}),
    }
    metrics = _json(metrics)
    manifest = export.write_bundle(
        bundle_dir,
        model_id=model_id,
        created_utc=created_utc,
        git_sha=git_sha,
        training_config_sha256=export.config_sha256(
            {"model_id": model_id, "nominal": nominal.to_dict(), "seeds": list(SEEDS)}
        ),
        vehicle=vehicle,
        nominal=nominal,
        residual=residual,
        v_max_mps=v_max,
        data=list(data_lines),
        metrics=metrics,
    )
    if pointer_dir is not None:
        pointer_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy(bundle_dir / export.MANIFEST_FILE, pointer_dir / export.MANIFEST_FILE)
        (pointer_dir / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
        (pointer_dir / "README.md").write_text(render_readme(manifest, metrics), encoding="utf-8")
    return metrics


def _row(label: str, errs: dict[Any, dict[str, float]]) -> str:
    cells = []
    for h in (0.5, 1.0, 1.5):
        e = errs.get(h) or errs.get(str(h))
        cells.append(f"{e['speed_rmse_mps']:.3f} / {e['yaw_rmse_rad']:.3f} / {e['pos_rmse_m']:.3f}")
    return f"| {label} | " + " | ".join(cells) + " |"


def render_readme(manifest: dict[str, Any], metrics: dict[str, Any]) -> str:
    io = manifest["io"]
    nom = io["nominal"]
    errors = metrics["heldout_errors"]
    enabled = metrics["residual_enabled"]
    seed = metrics["residual_shipped_seed"]
    lines = [
        f"# {manifest['model_id']}",
        "",
        (
            "Fitted dynamics bundle for the Plug-and-Play planner. Weights are not committed; the "
            "bundle lives under car-data/pnp/models/."
        ),
        "",
        (
            f"vehicle_params {manifest['vehicle_params']['schema_version']}, "
            f"sysid session {manifest['vehicle_params']['sysid_session_id']}. "
            f"Validity: v up to {io['validity']['v_max_mps']} m/s."
        ),
        "",
        "## Fitted nominal parameters",
        "",
        "| parameter | value |",
        "|---|---|",
        *[f"| {k} | {v:.4f} |" for k, v in nom.items()],
        "",
        (
            f"Validation error of the fit: wheel speed {metrics['fit']['speed_val_rmse_mps']:.4f} m/s "
            f"(RMSE over 1 s windows), yaw change {metrics['fit']['yaw_val_rmse_rad']:.4f} rad per 0.1 s."
        ),
        "",
        "Flags: " + (json.dumps(metrics["fit"]["flags"]) if metrics["fit"]["flags"] else "none"),
        "",
        "## Held-out error, open loop",
        "",
        (
            f"{metrics['heldout_windows']} windows. Cells are wheel speed RMSE (m/s) / yaw RMSE (rad) "
            "/ position RMSE (m) at 0.5 s, 1.0 s, 1.5 s."
        ),
        "",
        "| variant | 0.5 s | 1.0 s | 1.5 s |",
        "|---|---|---|---|",
        _row("placeholder defaults", errors["placeholder_nominal"]),
        _row("fitted nominal", errors["fitted_nominal"]),
        *[
            _row(f"fitted + residual (seed {s})", errors[f"fitted_plus_residual_seed{s}"])
            for s in SEEDS
        ],
        "",
        "## Residual verdict",
        "",
        (
            f"Median relative improvement at 1.0 s: speed {metrics['residual_median_improvement_1s']['speed']:+.1%}, "
            f"yaw {metrics['residual_median_improvement_1s']['yaw']:+.1%} (rule: both at least 5 percent). "
            f"Residual {'ENABLED (seed ' + str(seed) + ')' if enabled else 'NOT enabled: zeros in residual_mlp.f32'}."
        ),
        "",
        "## Ground speed against wheel speed",
        "",
        "| steering command (rad) | n | median ICP ground speed / wheel speed | p25 | p75 |",
        "|---|---|---|---|---|",
        *[
            f"| {r['steer_lo_rad']:.2f} to {r['steer_hi_rad']:.2f} | {r['n']} | "
            f"{r['median_ratio']:.3f} | {r['p25']:.3f} | {r['p75']:.3f} |"
            for r in metrics["ground_speed_over_wheel_speed_by_steering"]
        ],
        "",
        "## Data",
        "",
        *[f"- {line}" for line in manifest["data"]],
        "",
        "Seconds of driving per bag (train / val / held out):",
        "",
        *[
            f"- {name}: {v['train']} / {v['val']} / {v['heldout']}"
            for name, v in metrics["seconds_by_bag"].items()
        ],
        "",
    ]
    return "\n".join(lines)
