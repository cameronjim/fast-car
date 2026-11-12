"""Single-scan tracks against the map-free self-labels: how much there is to gain from seeing past
the corner.

    uv run --group bags python -m racer_pnp_perception.eval_selflabels \\
        --labels /Users/cameronjim/code/car-data/pnp/selflabels --bags <bag dir> [<bag dir> ...] \\
        [--bundle <bundle dir>] [--out eval_selflabels.json]

The labels are written by `racer_pnp_perception.selflabel` (geometry of a few metres of driving,
seen by the same sensor). For every labelled scan, each backend (the geometric tracer v1 on the
single scan, and the net of --bundle) is run through `TrackPerceiver.process`, the code the node
runs, and scored against the label:

* centreline error per station: the distance between the predicted and the labelled centreline
  point of station k, over the stations valid in both, as the mean and 90th percentile per station
  and over all stations;
* valid range: the mean and 10th percentile of the predictor's `valid_count` (the label has 12
  to 16);
* path containment: the share of the car's own driven path over the next 2.0 m (the label's
  gate, `selflabel.path_containment`) that lies inside the predicted corridor shrunk by the half
  body width, as the mean over scans (a scan with fewer than two valid stations counts 0) and the
  share of scans at or above 0.95. The label's own containment is reported for reference.

Only labelled scans are scored, and a label needs the car to have driven through the corridor, so
the numbers describe the part of the driving where motion matching worked and the car stayed
inside its own corridor.
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
from racer_track_perception import track as tr
from racer_track_perception.core import TrackPerceiver
from racer_track_perception.vehicle import LiveVehicle, load_live_vehicle

from racer_pnp_perception import selflabel as sl
from racer_pnp_perception.eval_bags import build_perceivers

log = logging.getLogger(__name__)

CONTAINED_SHARE = sl.MIN_PATH_CONTAINMENT


def score_backend(
    perceiver: TrackPerceiver,
    scans: sl.ScanSet,
    index: NDArray[np.int64],
    label: NDArray[np.float32],
    label_valid: NDArray[np.int64],
    path_xy: NDArray[np.float64],
    body_half_width_m: float,
) -> dict[str, NDArray[np.float64]]:
    """Per labelled scan: centreline error per station ([m, 16], NaN where either side is not
    valid), the predictor's valid_count and its path containment (NaN without two valid
    stations)."""
    m = len(index)
    err = np.full((m, tr.K_STATIONS), np.nan)
    valid = np.zeros(m)
    contain = np.full(m, np.nan)
    for k in range(m):
        i = int(index[k])
        t = perceiver.process(
            scans.ranges[i],
            scans.angle_min,
            scans.angle_increment,
            scans.range_min,
            scans.range_max,
        )
        n = min(t.valid_count, int(label_valid[k]))
        err[k, :n] = np.hypot(t.x_m[:n] - label[k, :n, 0], t.y_m[:n] - label[k, :n, 1])
        valid[k] = t.valid_count
        contain[k] = sl.path_containment(t, path_xy[k], body_half_width_m)
    return {"error": err, "valid": valid, "containment": contain}


def summarise(
    error: NDArray[np.float64],
    valid: NDArray[np.float64],
    containment: NDArray[np.float64],
) -> dict[str, Any]:
    """The tables for one backend over a set of labelled scans."""

    def stat(values: NDArray[np.float64], q: float) -> float:
        v = values[np.isfinite(values)]
        return float(np.percentile(v, q)) if len(v) else math.nan

    flat = error[np.isfinite(error)]
    per_station = {
        str(k): {
            "n": int(np.isfinite(error[:, k]).sum()),
            "mean_m": float(np.nanmean(error[:, k]))
            if np.isfinite(error[:, k]).any()
            else math.nan,
            "p90_m": stat(error[:, k], 90),
        }
        for k in range(error.shape[1])
    }
    scored = np.where(np.isfinite(containment), containment, 0.0)
    return {
        "scans": len(valid),
        "centreline_error_mean_m": float(flat.mean()) if len(flat) else math.nan,
        "centreline_error_p90_m": stat(flat, 90),
        "centreline_error_per_station": per_station,
        "valid_count_mean": float(valid.mean()) if len(valid) else math.nan,
        "valid_count_p10": stat(valid, 10),
        "path_containment_mean": float(scored.mean()) if len(scored) else math.nan,
        "path_containment_share_at_least_0_95": float((scored >= CONTAINED_SHARE).mean())
        if len(scored)
        else math.nan,
    }


def evaluate_bag(
    bag_dir: Path,
    labels_dir: Path,
    perceivers: dict[str, TrackPerceiver],
    live: LiveVehicle,
) -> dict[str, Any]:
    scans, _t, _v = sl.load_scanset(bag_dir, labels_dir / "cache")
    with np.load(labels_dir / f"{bag_dir.name}.npz") as z:
        index = z["index"]
        label = z["label"]
        label_valid = z["valid_count"]
        path_xy = z["path_xy"]
        label_containment = z["path_containment"]
    out: dict[str, Any] = {
        "labels": len(index),
        "label_valid_count_mean": float(label_valid.mean()) if len(index) else math.nan,
        "label_path_containment_mean": float(label_containment.mean()) if len(index) else math.nan,
        "backends": {},
    }
    if len(index) == 0:
        return out
    for name, perceiver in perceivers.items():
        s = score_backend(
            perceiver, scans, index, label, label_valid, path_xy, live.body.half_width_m
        )
        out["backends"][name] = summarise(s["error"], s["valid"], s["containment"])
    return out


def format_table(result: dict[str, Any]) -> str:
    """The three tables as plain text."""
    lines = []
    for bag, r in result["bags"].items():
        lines.append(f"{bag}: {r['labels']} labels")
        for name, b in r["backends"].items():
            lines.append(
                f"  {name:9s} error mean {b['centreline_error_mean_m']:.3f} m "
                f"p90 {b['centreline_error_p90_m']:.3f} m | valid mean {b['valid_count_mean']:.1f} "
                f"p10 {b['valid_count_p10']:.0f} | containment mean {b['path_containment_mean']:.3f} "
                f"share>=0.95 {b['path_containment_share_at_least_0_95']:.3f}"
            )
            per = b["centreline_error_per_station"]
            lines.append(
                "            per station mean (m): "
                + " ".join(f"{per[str(k)]['mean_m']:.2f}" for k in range(tr.K_STATIONS))
            )
            lines.append(
                "            per station p90  (m): "
                + " ".join(f"{per[str(k)]['p90_m']:.2f}" for k in range(tr.K_STATIONS))
            )
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--labels", type=Path, default=sl.DEFAULT_OUT)
    parser.add_argument("--bags", nargs="+", type=Path, required=True)
    parser.add_argument("--bundle", type=Path, default=None)
    parser.add_argument("--out", type=Path, default=Path("eval_selflabels.json"))
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    live = load_live_vehicle()
    perceivers = build_perceivers(live, args.bundle)
    result: dict[str, Any] = {"backends": list(perceivers), "bags": {}}
    for bag in args.bags:
        result["bags"][bag.name] = evaluate_bag(bag, args.labels, perceivers, live)
    args.out.write_text(json.dumps(result, indent=2, allow_nan=True))
    print(format_table(result))
    return 0


if __name__ == "__main__":
    sys.exit(main())
