"""Prediction error of a dynamics bundle on stretches of a bag where the planner drove, and a
reader for bundles written before schema 1.1.0.

The four PnP runs of bag 2026-11-12T03-04-56 (bag time = seconds after the first /scan) are where
the v0 bundle predicted motion that did not happen (the planner commanded 0.05 to 0.09 m/s and
the wheels did not turn). Open-loop rollouts of 0.5, 1.0 and 1.5 s from the recorded /drive
commands are compared with the wheel speed and the chained ICP pose, windows that start standing
included (evaluate.eval_data with `min_speed_mps` below zero).

`read_legacy_bundle` is for analysis only: a bundle without `speed_deadband_mps` is refused by
export.read_bundle and by the C++ loader; here it is read as a dead zone of exactly 0 (the model
it was fitted as), only to put an old bundle's error next to a new one's.
"""

from __future__ import annotations

import hashlib
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from .bags import Run
from .dataset import Segment
from .evaluate import HORIZONS_S, eval_data, horizon_errors
from .icp_motion import MotionSeries
from .model import Nominal, Residual, unpack_weights
from .vehicle import Vehicle

# Bag time (s after the first /scan) of the four planner runs, from the status messages.
PNP_RUNS_S = ((473.0, 492.0), (1087.0, 1123.0), (1131.0, 1167.0), (1174.0, 1189.0))


@dataclass(frozen=True)
class LegacyBundle:
    model_id: str
    nominal: Nominal
    residual: Residual
    v_max_mps: float


def read_legacy_bundle(bundle_dir: Path) -> LegacyBundle:
    """A schema 1.0.x dynamics bundle as the model it was fitted as (dead zone 0)."""
    manifest = yaml.safe_load((Path(bundle_dir) / "manifest.yaml").read_text(encoding="utf-8"))
    for item in manifest["payload"]:
        blob = (Path(bundle_dir) / item["file"]).read_bytes()
        if hashlib.sha256(blob).hexdigest() != item["sha256"]:
            raise ValueError(f"checksum mismatch for {item['file']}")
    io = manifest["io"]
    nominal = {"speed_deadband_mps": 0.0, **io["nominal"]}
    res = io["residual"]
    flat = np.frombuffer((Path(bundle_dir) / res["file"]).read_bytes(), dtype="<f4")
    residual = Residual(
        enabled=bool(res["enabled"]),
        weights=unpack_weights(flat, tuple(tuple(layer) for layer in res["layers"])),
        input_mean=np.asarray(res["input_mean"], dtype=float),
        input_std=np.asarray(res["input_std"], dtype=float),
        output_scale=np.asarray(res["output_scale"], dtype=float),
    )
    return LegacyBundle(
        manifest["model_id"],
        Nominal.from_dict(nominal),
        residual,
        float(io["validity"]["v_max_mps"]),
    )


def segments_between(
    run: Run,
    motion: MotionSeries | None,
    spans_s: Sequence[tuple[float, float]],
    t_ref_s: float,
) -> list[Segment]:
    """One segment per span, on the run's grid; spans are in seconds after `t_ref_s`."""
    out = []
    for a, b in spans_s:
        i0 = int(np.searchsorted(run.t_s, t_ref_s + a))
        i1 = int(np.searchsorted(run.t_s, t_ref_s + b))
        if i1 - i0 > 2:
            out.append(Segment(run, motion, i0, i1, "heldout"))
    return out


def errors_on(
    segments: Sequence[Segment],
    nominal: Nominal,
    residual: Residual | None,
    vehicle: Vehicle,
    horizons_s: Sequence[float] = HORIZONS_S,
) -> dict[str, object]:
    """Windows and the speed / yaw / position RMSE at each horizon, standing windows included."""
    data = eval_data(segments, nominal.command_delay_s, min_speed_mps=-1.0)
    errs = horizon_errors(data, nominal, vehicle, residual, horizons_s)
    return {"windows": len(data), "errors": errs}


def compare_on_runs(
    run: Run,
    motion: MotionSeries | None,
    t_ref_s: float,
    variants: dict[str, tuple[Nominal, Residual | None]],
    vehicle: Vehicle,
    spans_s: Sequence[tuple[float, float]] = PNP_RUNS_S,
) -> dict[str, dict[str, object]]:
    """{variant: {"pooled": ..., "run0": ..., ...}} on the planner runs of one bag."""
    out: dict[str, dict[str, object]] = {}
    for name, (nominal, residual) in variants.items():
        rows: dict[str, object] = {
            "pooled": errors_on(
                segments_between(run, motion, spans_s, t_ref_s), nominal, residual, vehicle
            )
        }
        for k, span in enumerate(spans_s):
            rows[f"run{k}"] = errors_on(
                segments_between(run, motion, [span], t_ref_s), nominal, residual, vehicle
            )
        out[name] = rows
    return out
