"""Score trained runs and the geometric tracer on the INTERNAL validation labels, the numbers that
model selection may use besides the sim validation.

    uv run --group train --group bags python -m racer_pnp_perception.internal_val \\
        --labels /Users/cameronjim/code/car-data/pnp/selflabels-v0.2 \\
        --split /Users/cameronjim/code/car/evaluation/prereg/pnp-v0.2-split.yaml \\
        --run <run dir> [<run dir> ...] --out internal_val.json

The internal validation part is the last 15 percent in time of each `train` segment (real.py;
labels whose window crosses the cut are in neither part). Only `train` role labels are read. For
every such label the net of each run and single-scan geometric-v1 go through
`TrackPerceiver.process` on the scan itself (scans from the label cache), and the GP1 numbers
(gp1.py: E_c, E_far, V, C95, and the label-free intrusion, residual median and clearance deficit
p90) are computed on that set. No held-out segment is opened.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path
from typing import Any

from racer_track_perception.core import BACKEND_GEOMETRIC, BACKEND_NET, TrackPerceiver
from racer_track_perception.runtime import CircConv1dV0
from racer_track_perception.vehicle import load_live_vehicle

from racer_pnp_perception import gp1, real
from racer_pnp_perception import selflabel as sl
from racer_pnp_perception.split import DEFAULT_SPLIT_PATH, load_split

log = logging.getLogger(__name__)


def net_from_run(run: Path) -> CircConv1dV0:
    import torch

    from racer_pnp_perception import model

    net = model.build_model()
    net.load_state_dict(torch.load(run / "model.pt", map_location="cpu"))
    net.eval()
    return CircConv1dV0(model.to_npz_arrays(net))


def validation_rows(
    val: real.RealSet, labels_dir: Path, perceivers: dict[str, TrackPerceiver], live: Any
) -> list[gp1.ScanRow]:
    """One scored ScanRow per internal validation label."""
    rows: list[gp1.ScanRow] = []
    scans_by_bag: dict[str, sl.ScanSet] = {}
    for k in range(len(val)):
        bag = str(val.bag[k])
        if bag not in scans_by_bag:
            scans_by_bag[bag] = sl.load_scanset(Path(bag), labels_dir / "cache")[0]
        scans = scans_by_bag[bag]
        row = gp1.ScanRow(
            bag=bag,
            index=int(val.scan_index[k]),
            set=gp1.SET_RUN,
            segment=str(val.segment[k]),
            segment_from_s=0.0,
            t_s=float(val.window_first_s[k]),
            speed_mps=0.0,
            direction="unknown",
            path=val.path_xy[k],
            label=val.label[k],
            label_valid=int(val.valid_count[k]),
        )
        gp1.score_row(row, scans, row.index, perceivers, live)
        rows.append(row)
    return rows


def summarise_rows(rows: list[gp1.ScanRow], backends: list[str]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for b in backends:
        lab = gp1.labelled_numbers(rows, b)
        free = gp1.label_free_numbers(rows, b)
        out[b] = {**vars(lab), **{f"free_{k}": v for k, v in vars(free).items()}}
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--labels", type=Path, required=True)
    parser.add_argument("--split", type=Path, default=DEFAULT_SPLIT_PATH)
    parser.add_argument("--run", type=Path, nargs="+", required=True)
    parser.add_argument("--out", type=Path, default=Path("internal_val.json"))
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    split = load_split(args.split)
    live = load_live_vehicle()
    # Only the label files of bags that have a train part; a file that also holds held-out
    # labels is filtered by role (real.load_real), exactly as in training.
    files = [
        args.labels / f"{bag}.npz"
        for bag in split.bag_names_with_role("train")
        if (args.labels / f"{bag}.npz").is_file()
    ]
    train = real.load_real(files, split)
    _tr, val, _report = real.train_val_split(train, split)
    log.info("internal validation: %d labels", len(val))
    result: dict[str, Any] = {"labels": len(val), "runs": {}}
    geo = {BACKEND_GEOMETRIC: TrackPerceiver(BACKEND_GEOMETRIC, live)}
    for run in args.run:
        perceivers = {
            **geo,
            BACKEND_NET: TrackPerceiver(
                BACKEND_NET, live, net=net_from_run(run), model_id=run.name
            ),
        }
        rows = validation_rows(val, args.labels, perceivers, live)
        result["runs"][str(run)] = summarise_rows(rows, list(perceivers))
        log.info("%s: %s", run, json.dumps(result["runs"][str(run)][BACKEND_NET]))
    args.out.write_text(json.dumps(result, indent=2, allow_nan=True))
    print(json.dumps(result, indent=2, allow_nan=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
