"""Simulation evaluation: centreline and width error by station on three sets.

    uv run python -m racer_pnp_perception.eval_sim --bundle <dir> [--n 4000] [--out metrics.json]

Sets: a fresh in-distribution seed, the three held-out family tracks (never trained on), and clean
scans (no noise, dropout, gaps, room or motion). For each set, the net's raw [16, 5] output (all
stations, as the net's own acceptance is measured), the net restricted to its valid stations, and
the geometric backend on its valid stations, each as the median centreline error in metres over
stations 0 to 7 and 8 to 15 and the median half width error.

Acceptance on the held-out family tracks (raw net): median centreline error at most 0.05 m over
stations 0 to 7 and at most 0.10 m over stations 8 to 15, median half width error at most 0.07 m.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
from racer_track_perception import core
from racer_track_perception.bundle import load_bundle
from racer_track_perception.runtime import CircConv1dV0
from racer_track_perception.vehicle import LiveVehicle, load_live_vehicle

from racer_pnp_perception import dataset
from racer_pnp_perception.raycast import ScanGeometry
from racer_pnp_perception.tracks import HELD_OUT_ISLANDS

log = logging.getLogger(__name__)

ACCEPT_NEAR_M = 0.05
ACCEPT_FAR_M = 0.10
ACCEPT_WIDTH_M = 0.07
NEAR = slice(0, 8)
FAR = slice(8, 16)


def error_summary(
    pos_err: NDArray[np.float64],
    width_err: NDArray[np.float64],
    valid: NDArray[np.bool_],
) -> dict[str, Any]:
    """Medians over the stations flagged valid. pos_err [n, 16], width_err [n, 16, 2]."""

    def med(values: NDArray[np.float64], mask: NDArray[np.bool_]) -> float:
        picked = values[mask]
        return float(np.median(picked)) if picked.size else math.nan

    return {
        "centreline_err_median_stations_0_7_m": med(pos_err[:, NEAR], valid[:, NEAR]),
        "centreline_err_median_stations_8_15_m": med(pos_err[:, FAR], valid[:, FAR]),
        "centreline_err_p90_stations_0_7_m": float(
            np.percentile(pos_err[:, NEAR][valid[:, NEAR]], 90)
        )
        if valid[:, NEAR].any()
        else math.nan,
        "centreline_err_p90_stations_8_15_m": float(
            np.percentile(pos_err[:, FAR][valid[:, FAR]], 90)
        )
        if valid[:, FAR].any()
        else math.nan,
        "half_width_err_median_m": med(width_err.mean(axis=-1), valid),
        "valid_share": float(valid.mean()),
    }


def evaluate_set(
    data: dict[str, NDArray[Any]],
    live: LiveVehicle,
    net: CircConv1dV0 | None,
    geometry: ScanGeometry,
) -> dict[str, Any]:
    tensors, labels, ranges = data["tensor"], data["label"], data["ranges"]
    n = len(tensors)
    out: dict[str, Any] = {"samples": n}
    geo = core.TrackPerceiver(core.BACKEND_GEOMETRIC, live)
    g_pos = np.zeros((n, 16))
    g_wid = np.zeros((n, 16, 2))
    g_valid = np.zeros((n, 16), dtype=bool)
    for i in range(n):
        t = geo.process(
            ranges[i],
            geometry.angle_min_rad,
            geometry.angle_increment_rad,
            geometry.range_min_m,
            geometry.range_max_m,
        )
        g_pos[i] = np.hypot(t.x_m - labels[i, :, 0], t.y_m - labels[i, :, 1])
        g_wid[i, :, 0] = np.abs(t.half_width_left_m - labels[i, :, 2])
        g_wid[i, :, 1] = np.abs(t.half_width_right_m - labels[i, :, 3])
        g_valid[i, : t.valid_count] = True
    out["geometric_valid"] = error_summary(g_pos, g_wid, g_valid)
    if net is not None:
        raw = np.stack([net.forward(tensors[i : i + 1]) for i in range(n)])
        pos = np.hypot(raw[..., 0] - labels[..., 0], raw[..., 1] - labels[..., 1])
        wid = np.abs(raw[..., 2:4] - labels[..., 2:4])
        out["net_raw"] = error_summary(pos, wid, np.ones((n, 16), dtype=bool))
        sigma = np.exp(np.clip(raw[..., 4], *core.LOG_SIGMA_RANGE))
        ok = np.cumprod(sigma < 0.25, axis=1).astype(bool)  # the leading run, as valid_count
        out["net_valid"] = error_summary(pos, wid, ok)
        out["net_sigma_median_m"] = float(np.median(sigma))
    return out


def acceptance(heldout: dict[str, Any]) -> dict[str, Any]:
    """The acceptance check of each held-out track on the raw net output."""
    rows = {}
    for name, res in heldout.items():
        net = res["net_raw"]
        near = net["centreline_err_median_stations_0_7_m"]
        far = net["centreline_err_median_stations_8_15_m"]
        wid = net["half_width_err_median_m"]
        rows[name] = {
            "centreline_0_7_ok": bool(near <= ACCEPT_NEAR_M),
            "centreline_8_15_ok": bool(far <= ACCEPT_FAR_M),
            "half_width_ok": bool(wid <= ACCEPT_WIDTH_M),
        }
    rows["all_ok"] = all(all(r.values()) for r in rows.values())
    return rows


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, default=None)
    parser.add_argument("--n", type=int, default=4000)
    parser.add_argument("--seed", type=int, default=777)
    parser.add_argument("--out", type=Path, default=Path("eval_sim_metrics.json"))
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    live = load_live_vehicle()
    net = None
    if args.bundle is not None:
        bundle = load_bundle(args.bundle, "track_perception", live)
        net = CircConv1dV0.from_npz(bundle.payload_path("weights.npz"))
    geometry = ScanGeometry()
    results: dict[str, Any] = {}
    log.info("in-distribution, fresh seed")
    results["in_distribution"] = evaluate_set(
        dataset.generate(args.n, args.seed, keep_ranges=True), live, net, geometry
    )
    log.info("clean scans")
    results["clean"] = evaluate_set(
        dataset.generate(args.n, args.seed + 1, clean=True, keep_ranges=True), live, net, geometry
    )
    held: dict[str, Any] = {}
    for k, name in enumerate(HELD_OUT_ISLANDS):
        log.info("held out: %s", name)
        held[name] = evaluate_set(
            dataset.generate(args.n, args.seed + 10 + k, held_out=name, keep_ranges=True),
            live,
            net,
            geometry,
        )
    results["held_out"] = held
    if net is not None:
        results["held_out_acceptance"] = acceptance(held)
    args.out.write_text(json.dumps(results, indent=2, allow_nan=True))
    print(json.dumps(results, indent=2, allow_nan=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
