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
        zero = [r.valid[b] == 0 for r in rows if b in r.valid]
        out[b] = {
            **vars(lab),
            **{f"free_{k}": v for k, v in vars(free).items()},
            "v_zero_share": float(sum(zero) / len(zero)) if zero else float("nan"),
        }
    return out


# Groups of the internal validation labels reported apart (bag-name prefix). "session2" is the
# train part of the 2026-11-23 session, the analogue of H-session; "1116" the 2026-11-16 bags.
GROUPS = {"session2": "2026-11-24T04-36-54", "1116": "2026-11-16T"}


def group_of(bag: str) -> str:
    name = Path(bag).name
    for group, prefix in GROUPS.items():
        if name.startswith(prefix):
            return group
    return "old"


def summarise_groups(rows: list[gp1.ScanRow], backends: list[str]) -> dict[str, Any]:
    """`summarise_rows` over all rows and per group (GROUPS, the rest as "old")."""
    out = {"all": summarise_rows(rows, backends)}
    for group in [*GROUPS, "old"]:
        sub = [r for r in rows if group_of(r.bag) == group]
        if sub:
            out[group] = summarise_rows(sub, backends)
    return out


def parse_named(items: list[str]) -> dict[str, Path]:
    out = {}
    for item in items:
        name, _, path = item.partition("=")
        if not name or not path:
            raise SystemExit(f"{item!r}: expected NAME=DIR")
        out[name] = Path(path)
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--labels", type=Path, required=True)
    parser.add_argument("--split", type=Path, default=DEFAULT_SPLIT_PATH)
    parser.add_argument("--split-sha256", default=None, help="digest of a split other than v0.2")
    parser.add_argument("--run", type=Path, nargs="*", default=[])
    parser.add_argument(
        "--bundle",
        action="append",
        default=[],
        help="NAME=DIR, a bundle scored as the node runs it",
    )
    parser.add_argument(
        "--val-plan", default="default", choices=sorted(real.VAL_PLANS), help="real.VAL_PLANS"
    )
    parser.add_argument(
        "--refine", action="store_true", help="score --run nets with the scan refinement (as v2-v5)"
    )
    parser.add_argument("--out", type=Path, default=Path("internal_val.json"))
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    split = (
        load_split(args.split, expected_sha256=args.split_sha256)
        if args.split_sha256
        else load_split(args.split)
    )
    live = load_live_vehicle()
    # Only the label files of bags that have a train part; a file that also holds held-out
    # labels is filtered by role (real.load_real), exactly as in training.
    files = [
        args.labels / f"{bag}.npz"
        for bag in split.bag_names_with_role("train")
        if (args.labels / f"{bag}.npz").is_file()
    ]
    train = real.load_real(files, split)
    _tr, val, _report = real.train_val_split(train, split, plan=args.val_plan)
    log.info("internal validation (%s): %d labels", args.val_plan, len(val))
    perceivers: dict[str, TrackPerceiver] = {
        BACKEND_GEOMETRIC: TrackPerceiver(BACKEND_GEOMETRIC, live)
    }
    if args.bundle:
        from racer_pnp_perception import compare_v3 as cv

        bundles = cv.perceivers(live, parse_named(args.bundle))
        perceivers.update({k: v for k, v in bundles.items() if k != BACKEND_GEOMETRIC})
    for run in args.run:
        perceivers[str(run)] = TrackPerceiver(
            BACKEND_NET, live, net=net_from_run(run), model_id=run.name, refine_net=args.refine
        )
    rows = validation_rows(val, args.labels, perceivers, live)
    groups = summarise_groups(rows, list(perceivers))
    result: dict[str, Any] = {
        "labels": len(val),
        "val_plan": args.val_plan,
        "refine_runs": args.refine,
        "groups": groups,
    }
    for name in perceivers:
        log.info(
            "%s: %s",
            name,
            " ".join(
                f"{g}: n {d[name]['scans']} V {d[name]['v']:.2f} E_c {d[name]['e_c_m']:.4f} "
                f"V0 {d[name]['v_zero_share']:.3f}"
                for g, d in groups.items()
            ),
        )
    args.out.write_text(json.dumps(result, indent=2, allow_nan=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
