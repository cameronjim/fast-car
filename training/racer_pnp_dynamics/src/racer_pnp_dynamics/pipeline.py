"""Fit, evaluate, decide on the residual, export: the whole offline run on already loaded
segments. `main` (real bags) is thin glue in cli.py; everything here is testable on synthetic
segments with a stand-in trainer."""

from __future__ import annotations

import json
import shutil
from collections import defaultdict
from collections.abc import Sequence
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import numpy as np

from . import deadzone, export
from .bags import Run
from .dataset import TRAIN_FRACTION, Segment, by_split, seconds
from .evaluate import (
    HORIZONS_S,
    compare_variants,
    eval_data,
    ground_speed_ratio,
    zero_yaw_rmse,
)
from .fit import FitResult, fit_nominal, yaw_rmse
from .icp_motion import MotionSeries
from .identify import steering_lag_s
from .model import PLACEHOLDER_NOMINAL, Nominal, Residual, steering_target
from .pnp_eval import PNP_RUNS_S, LegacyBundle, compare_on_runs
from .residual import (
    SEEDS,
    Trainer,
    build_samples,
    decide,
    early_stopping_split,
    relative_improvement,
    shipped,
    train_residual,
)
from .vehicle import Vehicle
from .windows import speed_rollout, speed_windows, yaw_data

V_MAX_ROUND_MPS = 0.05
V_MAX_FACTOR = 1.0  # brief 14: validity is the 99th percentile itself (v0 used 1.5 times it)
STEERING_LAG_DISAGREEMENT_S = 0.05
PROFILE_PNG = "profiles.png"


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
    """The 99th percentile of the true wheel speed in the real training data (times V_MAX_FACTOR),
    rounded down to 0.05 and never above the full scale speed."""
    speeds = []
    for seg in train:
        if seg.run.has_odom:
            v = seg.run.wheel_speed_mps[seg.i0 : seg.i1]
            speeds.append(v[np.isfinite(v)])
    p99 = float(np.percentile(np.concatenate(speeds), 99))
    raw = V_MAX_FACTOR * p99
    rounded = np.floor(raw / V_MAX_ROUND_MPS + 1e-9) * V_MAX_ROUND_MPS
    return round(float(min(rounded, vehicle.throttle_full_scale_mps)), 2)


def planner_min_radius(nominal: Nominal, vehicle: Vehicle) -> dict[str, float]:
    """Tightest path radius (m) the planner's model can plan at the command limits, from the
    exported vehicle, and the kinematic radius at the steering limit for reference."""
    left = steering_target(np.array(vehicle.steer_max_rad), nominal, vehicle)
    right = steering_target(np.array(vehicle.steer_min_rad), nominal, vehicle)
    return {
        "left": float(vehicle.wheelbase_m / np.tan(abs(left))),
        "right": float(vehicle.wheelbase_m / np.tan(abs(right))),
        "kinematic_at_limit": float(vehicle.wheelbase_m / np.tan(vehicle.steer_max_rad)),
    }


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


@dataclass(frozen=True)
class PnpRuns:
    """The planner runs to score the new bundle against an old one on (see pnp_eval)."""

    run: Run
    motion: MotionSeries | None
    t_ref_s: float  # stamp of "bag time 0" (the first /scan)
    old: LegacyBundle
    spans_s: Sequence[tuple[float, float]] = PNP_RUNS_S


def dead_zone_report(
    segments: Sequence[Segment], result: FitResult, vehicle: Vehicle
) -> dict[str, Any]:
    """Both direct estimates of the dead zone, the fitted value against them, and whether the
    car keeps rolling below it."""
    runs = list({id(s.run): s.run for s in segments if s.run.has_odom}.values())
    delay = result.nominal.command_delay_s
    launches = [x for r in runs for x in deadzone.find_launches(r, delay)]
    bins = deadzone.steady_bins(runs)
    brackets = deadzone.steady_brackets(bins)
    fitted = result.nominal.speed_deadband_mps
    lo, hi = brackets["largest_still_bin_hi_mps"], brackets["smallest_moving_bin_lo_mps"]
    inside = (lo is None or fitted >= lo) and (hi is None or fitted <= hi)
    odom = [s for s in segments if s.run.has_odom]
    windows = speed_windows(odom, delay)
    model_end = speed_rollout(windows, result.nominal, vehicle)[:, -1] if len(windows) else None
    return {
        "fitted_mps": fitted,
        "launches_from_rest": deadzone.launch_summary(launches),
        "launch_list": [
            {
                "bag": x.run_name,
                "t_s": round(
                    x.t_s - float(next(r for r in runs if r.name == x.run_name).t_s[0]), 1
                ),
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
    pnp: PnpRuns | None = None,
    profile_png: bool = False,
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
    s_fit, s_stop, stop_how = early_stopping_split(s_train, s_val)
    candidates = [train_residual(s_fit, s_stop, seed, trainer) for seed in SEEDS]
    variants: dict[str, tuple[Nominal, Residual | None]] = {
        "placeholder_nominal": (PLACEHOLDER_NOMINAL, None),
        "fitted_nominal": (nominal, None),
        **(
            {"fitted_nominal_other_side_tying": (result.alternative, None)}
            if result.alternative is not None
            else {}
        ),
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
    held_yaw = yaw_data(heldout, nominal.command_delay_s)
    exported_limits = compare_variants(heldout, vehicle, {"fitted_nominal": (nominal, None)})[
        "fitted_nominal"
    ]
    v_max = v_max_of(train, vehicle)
    lag = steering_lag_s(segments)
    fitted_lag = nominal.command_delay_s + nominal.steer_tau_s
    lag_check: dict[str, Any] = {"available": lag is not None}
    if lag is not None:
        lag_check.update(
            peak_lag_s=lag.lag_s,
            peak_correlation=lag.peak_correlation,
            intervals=lag.n_intervals,
            fitted_delay_plus_steer_tau_s=fitted_lag,
            difference_s=lag.lag_s - fitted_lag,
            disagrees=abs(lag.lag_s - fitted_lag) > STEERING_LAG_DISAGREEMENT_S,
            threshold_s=STEERING_LAG_DISAGREEMENT_S,
            lags_s=list(lag.lags_s),
            correlation=list(lag.correlation),
        )
    speeds_all = np.concatenate(
        [
            s.run.wheel_speed_mps[s.i0 : s.i1][np.isfinite(s.run.wheel_speed_mps[s.i0 : s.i1])]
            for s in segments
            if s.run.has_odom
        ]
    )
    pnp_report: dict[str, Any] | None = None
    if pnp is not None:
        pnp_report = compare_on_runs(
            pnp.run,
            pnp.motion,
            pnp.t_ref_s,
            {
                f"old_{pnp.old.model_id}": (pnp.old.nominal, pnp.old.residual),
                "new_nominal": (nominal, None),
                "new_shipped": (nominal, residual),
            },
            vehicle,
            pnp.spans_s,
        )

    metrics: dict[str, Any] = {
        "dead_zone": dead_zone_report(segments, result, fvehicle),
        "unidentified": result.unidentified,
        "profiles": result.profiles,
        "profile_notes": result.profile_notes,
        "steering_lag_check": lag_check,
        "validity": {
            "v_max_mps": v_max,
            "factor": V_MAX_FACTOR,
            "p99_wheel_speed_train_mps": float(
                np.percentile(
                    np.concatenate(
                        [
                            s.run.wheel_speed_mps[s.i0 : s.i1][
                                np.isfinite(s.run.wheel_speed_mps[s.i0 : s.i1])
                            ]
                            for s in train
                            if s.run.has_odom
                        ]
                    ),
                    99,
                )
            ),
            "p99_wheel_speed_all_moving_data_mps": float(np.percentile(speeds_all, 99)),
        },
        **({"pnp_runs": pnp_report} if pnp_report is not None else {}),
        "heldout_windows": len(window_data),
        "heldout_horizons_s": list(HORIZONS_S),
        "heldout_errors": errors,
        "heldout_errors_per_bag": per_bag,
        "heldout_fitted_nominal_with_exported_limits": exported_limits,
        "heldout_zero_yaw_baseline_rad": zero_yaw_rmse(window_data),
        "heldout_yaw_rmse_per_interval_rad": {
            "ground_driven": yaw_rmse(held_yaw, nominal, fvehicle, speed="ground"),
            "wheel_driven": yaw_rmse(held_yaw, nominal, fvehicle, speed="wheel"),
        },
        "planner_min_radius_m": planner_min_radius(nominal, vehicle),
        "residual_improvement_1s_per_seed": {
            str(seed): {"speed": a, "yaw": b}
            for seed, (a, b) in zip(SEEDS, improvements, strict=True)
        },
        "residual_median_improvement_1s": {"speed": medians[0], "yaw": medians[1]},
        "residual_enabled": enabled,
        "residual_shipped_seed": SEEDS[chosen],
        "residual_samples": {
            "train": len(s_fit),
            "early_stopping": len(s_stop),
            "early_stopping_from": stop_how,
        },
        "fit": {
            "flags": result.flags,
            "speed_heldout_rmse_mps": result.speed_heldout_rmse_mps,
            "yaw_heldout_rmse_rad": result.yaw_heldout_rmse_rad,
            "delay_selection": result.delay_selection,
            "yaw_target": "path_curvature_ground_speed",
            "accel_rule": result.accel_rule,
            "yaw_training": result.yaw_training,
            "sides_tied": result.sides_tied,
            "yaw_cv_rmse_rad": result.yaw_cv_rmse_rad,
            "delay_curve_speed_heldout_rmse": {str(k): v for k, v in result.delay_curve.items()},
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
        "split_rule": f"first {TRAIN_FRACTION:.0%} of each bag's moving data train, rest held out",
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
    if profile_png:
        from .plots import plot_from_metrics

        png = bundle_dir / PROFILE_PNG
        plot_from_metrics(png, metrics, nominal.to_dict(), f"{model_id}: identification profiles")
    if pointer_dir is not None:
        pointer_dir.mkdir(parents=True, exist_ok=True)
        if profile_png:
            shutil.copy(bundle_dir / PROFILE_PNG, pointer_dir / PROFILE_PNG)
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


FINDINGS = [
    (
        "- Split (brief 14): within every bag the first 70 percent of its moving data trains and "
        "the last 30 percent is held out; no bag is held out whole and there is no validation "
        "split. The delay is therefore scored on the held-out part, which makes the held-out "
        "numbers below in-sample for the delay and the dead zone profile. The bags are nested "
        "copies of three recordings (only the longest of each group is used). The two 2026-11-12 "
        "bags are in current units and are not converted; every older bag is converted once."
    ),
    (
        "- Windows where every command was at least 0.5 m/s and the wheels never turned (the car "
        "held, blocked, or the layers downstream of /drive not passing the command) are dropped "
        "from the speed fit and the evaluation: a 1.75 m/s gap follow command with the car "
        "standing was 90 percent of the squared speed error."
    ),
    (
        "- The dead zone shipped is the median command of the launches from rest (the "
        "launch side value), not the argmin of a speed error curve: the held-out windows hold "
        "almost no command inside the dead zone band, so held-out error cannot identify it (its "
        "profile table moves with the refit, not with the dead zone), and the error over "
        "windows that start at rest prefers 0.4 m/s because of windows where the car was held "
        "still under 0.36 to 0.6 m/s. A launch whose command jumped from nothing gives only an "
        "upper bound on the threshold, so the estimate leans high; launches with the car held "
        "still under an already high command are left out. The launch table lists every launch. "
        "The stiction varies between about 0.19 and 0.29 m/s within one bag; the planner adds "
        "its own margin."
    ),
    (
        "- A parameter on a bound or with a flat profile is listed under unidentified and is "
        "shipped as fitted, except the delay: an unidentified delay ships the grid minimum "
        "(0.00 s). The speed time constant, delay and the accel limit are an effective "
        "description of the drivetrain, not measured actuator constants."
    ),
    (
        "- The acceleration limit is kept at its start value unless the fitted value fits the "
        "rolling windows (start speed and every command at least 0.15 m/s) better. The yaw step "
        "is driven by ICP ground speed (steering gain = path curvature per command); ground "
        "speed falls to about 0.8 of wheel speed at full lock (table below), so in a full-lock "
        "turn the model over-predicts distance and yaw rate per second, the conservative side "
        "against walls."
    ),
    (
        "- Windows with a reverse command or a wheel speed below -0.03 m/s are skipped (the model "
        "has v >= 0). Converted steering reaches 0.504 rad to the left, beyond the planning "
        "limit: the fit and the held-out numbers clamp at the physical stops, the exported "
        "bundle uses the vehicle_params limits."
    ),
    (
        "- Gains are tied (one gain, offset 0) when a side has fewer than 300 training turning "
        "intervals. The residual ships enabled only if the median of its relative improvement "
        "over the fitted nominal at 1.0 s is at least 5 percent on both speed and yaw."
    ),
]


def _num(x: float | None, digits: int = 4) -> str:
    return "n/a" if x is None else f"{x:.{digits}f}"


def _fit_summary(fit: dict[str, Any], delay_s: float) -> str:
    curve = fit["delay_curve_speed_heldout_rmse"]
    at = curve.get(str(delay_s)) or curve.get(f"{delay_s:.2f}")
    lines = [
        (
            f"Command delay chosen by: {fit['delay_selection']} "
            f"(speed RMSE at the chosen delay {_num(at)} m/s, 1 s windows)."
        ),
        "",
        (
            f"Held-out error: wheel speed {_num(fit['speed_heldout_rmse_mps'])} m/s, yaw "
            f"change {_num(fit['yaw_heldout_rmse_rad'])} rad per 0.1 s (n/a means the split holds no "
            "data of that kind)."
        ),
        "",
        f"Steering gains tied (one gain, offset 0): {fit['sides_tied']}.",
        "",
        (
            f"Yaw channel trained on: {fit['yaw_training']}; blocked CV yaw RMSE per option "
            f"{json.dumps(fit['yaw_cv_rmse_rad'])}."
        ),
        "",
        f"Training counts: {json.dumps(fit['counts'])}.",
    ]
    return "\n".join(lines)


def _dead_zone_section(metrics: dict[str, Any]) -> list[str]:
    dz = metrics["dead_zone"]
    launches = dz["launches_from_rest"]["all"]
    resolved = dz["launches_from_rest"]["resolved"]
    steady = dz["steady_state"]
    roll = dz["rolling_below_the_dead_zone"]
    choice = dz["choice"]
    lines = [
        "## Dead zone",
        "",
        (
            f"Shipped `speed_deadband_mps`: {dz['fitted_mps']:.2f} m/s "
            f"({choice.get('rule', '')}; {choice.get('launches_used', 0)} used of "
            f"{choice.get('launches', 0)} launches)."
        ),
        "",
        (
            "- (a) Launches from rest, command at which the wheel speed first exceeds "
            f"{dz['launches_from_rest']['launch_speed_mps']} m/s: n {launches.get('n', 0)}, median "
            f"{_num(launches.get('median_mps'), 3)}, 10th to 90th percentile "
            f"{_num(launches.get('p10_mps'), 3)} to {_num(launches.get('p90_mps'), 3)} m/s. Launches "
            f"whose command interval is resolved: n {resolved.get('n', 0)}, median "
            f"{_num(resolved.get('median_mps'), 3)} m/s."
        ),
        (
            f"- (b) Steady state: the largest command bin with the wheels still ends at "
            f"{_num(steady['largest_still_bin_hi_mps'], 2)} m/s, the smallest bin with a wheel speed "
            f"above {steady['moving_wheel_mps']} m/s starts at "
            f"{_num(steady['smallest_moving_bin_lo_mps'], 2)} m/s (bins with at least "
            f"{steady['min_bin_samples']} samples; teleop and gap follow hold no steady command in "
            "between). Shipped value inside the bracket: "
            f"{dz['fitted_inside_steady_bracket']}."
        ),
        (
            "- Held-out speed RMSE over all windows is smallest at a dead zone of "
            f"{_num(choice.get('heldout_all_windows_argmin_mps'), 2)} m/s "
            f"({_num(choice.get('heldout_all_windows_rmse_min'), 4)} m/s) against "
            f"{_num(choice.get('heldout_all_windows_rmse_at_fitted'), 4)} m/s at the shipped value."
        ),
        (
            "- Rolling below the dead zone (windows that start at 0.15 m/s or more and are commanded "
            f"between 0.02 m/s and the dead zone throughout): {roll.get('windows', 0)} windows, median "
            f"start {_num(roll.get('median_start_speed_mps'), 3)} m/s, median end "
            f"{_num(roll.get('median_end_speed_mps'), 3)} m/s, share still turning at the end "
            f"{_num(roll.get('share_still_turning_at_end'), 2)}, model median end "
            f"{_num(roll.get('model_median_end_speed_mps'), 3)} m/s."
        ),
        "",
        "| bag | bag time (s) | command at launch (m/s) | highest command while still (m/s) |",
        "|---|---|---|---|",
        *[
            f"| {x['bag'][:19]}{'..' if len(x['bag']) > 19 else ''} {x['bag'][-12:]} | "
            f"{x['t_s']:.0f} | {x['cmd_launch_mps']:.3f} | {x['cmd_rest_max_mps']:.3f} |"
            for x in dz["launch_list"]
        ],
        "",
    ]
    return lines


def _identification_section(metrics: dict[str, Any]) -> list[str]:
    lines = ["## Identification", ""]
    if metrics["unidentified"]:
        lines += ["Not identified by the data (shipped as fitted, delay at the grid minimum):", ""]
        lines += [f"- {k}: {v}" for k, v in metrics["unidentified"].items()]
    else:
        lines += ["Every profiled parameter is identified."]
    lag = metrics["steering_lag_check"]
    if lag.get("available"):
        lines += [
            "",
            (
                "Steering lag cross-check (peak of the cross-correlation of the steering command with "
                "ICP path curvature, speed over 0.4 m/s, independent of the fit): "
                f"{lag['peak_lag_s']:.3f} s (correlation {lag['peak_correlation']:.2f}, "
                f"{lag['intervals']} intervals) against fitted delay plus steer_tau "
                f"{lag['fitted_delay_plus_steer_tau_s']:.3f} s; difference {lag['difference_s']:+.3f} s"
                f"{', over the 0.05 s threshold' if lag['disagrees'] else ''}."
            ),
        ]
    lines += [
        "",
        "Profiles (held-out RMSE against the parameter, the others refit): profiles.png.",
        "",
    ]
    for name in ("command_delay_s", "speed_tau_s", "steer_tau_s", "speed_deadband_mps"):
        rows = metrics["profiles"].get(name, [])
        if not rows:
            continue
        lines += [
            f"{name} ({metrics['profile_notes'].get(name, '')})",
            "",
            "| value | RMSE | over the minimum |",
            "|---|---|---|",
            *[f"| {r['value']:.3f} | {r['rmse']:.4f} | {r['over_min']:+.1%} |" for r in rows],
            "",
        ]
    return lines


def _pnp_section(metrics: dict[str, Any]) -> list[str]:
    runs = metrics.get("pnp_runs")
    if not runs:
        return []
    lines = [
        "## Prediction error on the planner runs (bag 2026-11-12T03-04-56)",
        "",
        (
            "Open loop from the recorded /drive commands, speed against the wheel, pose against "
            "chained ICP; windows that start standing included. Cells: wheel speed RMSE (m/s) / yaw "
            "RMSE (rad) / position RMSE (m)."
        ),
        "",
        "| variant | windows | 0.5 s | 1.0 s | 1.5 s |",
        "|---|---|---|---|---|",
    ]
    for name, rows in runs.items():
        pooled = rows["pooled"]
        cells = []
        for h in ("0.5", "1.0", "1.5"):
            e = pooled["errors"][h]
            cells.append(
                f"{e['speed_rmse_mps']:.3f} / {e['yaw_rmse_rad']:.3f} / {e['pos_rmse_m']:.3f}"
            )
        lines.append(f"| {name} | {pooled['windows']} | " + " | ".join(cells) + " |")
    lines.append("")
    return lines


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
        _fit_summary(metrics["fit"], nom["command_delay_s"]),
        "",
        "Flags (parameters on a bound, or that the data cannot move):",
        "",
        *(
            [f"- {k}: {v}" for k, v in metrics["fit"]["flags"].items()]
            if metrics["fit"]["flags"]
            else ["- none"]
        ),
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
        _row("fitted nominal (shipped)", errors["fitted_nominal"]),
        _row(
            "fitted nominal, other side tying (not shipped)",
            errors.get("fitted_nominal_other_side_tying", errors["fitted_nominal"]),
        ),
        *[
            _row(f"fitted + residual (seed {s})", errors[f"fitted_plus_residual_seed{s}"])
            for s in SEEDS
        ],
        "",
        "## Planner geometry",
        "",
        (
            f"Minimum path radius at the command limits (exported vehicle): left "
            f"{metrics['planner_min_radius_m']['left']:.3f} m, right "
            f"{metrics['planner_min_radius_m']['right']:.3f} m; kinematic at the steering limit "
            f"{metrics['planner_min_radius_m']['kinematic_at_limit']:.3f} m."
        ),
        "",
        (
            "Held-out yaw RMSE per 0.1 s interval, shipped nominal: driven by ICP ground speed "
            f"{metrics['heldout_yaw_rmse_per_interval_rad']['ground_driven']:.4f} rad, driven by "
            f"wheel speed {metrics['heldout_yaw_rmse_per_interval_rad']['wheel_driven']:.4f} rad."
        ),
        "",
        (
            "Acceleration limit rule: "
            f"{json.dumps(metrics['fit']['accel_rule'])} (speed RMSE on the rolling windows with "
            "the fitted value and with the start value; the start value is kept when it is not "
            "worse)."
        ),
        "",
        *_dead_zone_section(metrics),
        *_identification_section(metrics),
        *_pnp_section(metrics),
        "## Residual verdict",
        "",
        (
            f"Median relative improvement at 1.0 s: speed {metrics['residual_median_improvement_1s']['speed']:+.1%}, "
            f"yaw {metrics['residual_median_improvement_1s']['yaw']:+.1%} (rule: both at least 5 percent). "
            f"Residual {'ENABLED (seed ' + str(seed) + ')' if enabled else 'NOT enabled: zeros in residual_mlp.f32'}."
        ),
        "",
        "## Decisions and findings",
        "",
        *FINDINGS,
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
