"""GP1b: net plus scan refinement against the geometric tracer, decided as written in
docs/notes/preregistration-pnp-v0.3.md section 3.3 (`gp1.decide_gp1b`).

    uv run --group train --group bags python -m racer_pnp_perception.gp1b \\
        --labels /Users/cameronjim/code/car-data/pnp/selflabels-v0.2 \\
        --bags-root /Users/cameronjim/code/car-data/bags \\
        --run-full <run dir> --run-f1 <run dir> --run-f2 <run dir> --out gp1b.json

Run it ONCE per frozen recipe (a rerun forced by a bug is reported next to the first).

Three models are scored: the full model (all `train` roles) on H-run, A-run and the 2026-11-12
layout bags (`heldout`, `heldout_layout` of the v0.2 split), and the fold models F1 and F2 on their
own held-out bags (all moving scans, and the labelled ones). Each model goes through
`TrackPerceiver.process` three ways on the same scans: net plus scan refinement (`net`, the
candidate), the same weights without refinement (`net_unrefined`), and single-scan `geometric-v1`.
The decision is made for the candidate; the same rule applied to the unrefined weights is printed
next to it, so the report says what the refinement did and what the retraining did.

Moving scans: wheel speed at least 0.2 m/s, and in a bag without /odom/wheel the ICP ground speed at
least 0.2 m/s (selflabel.load_scanset); the source of each bag is listed in the output.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import logging
import sys
from pathlib import Path
from typing import Any

from racer_track_perception.core import BACKEND_GEOMETRIC, BACKEND_NET, TrackPerceiver
from racer_track_perception.refine import RefineConfig
from racer_track_perception.vehicle import LiveVehicle, load_live_vehicle

from racer_pnp_perception import gp1
from racer_pnp_perception import selflabel as sl
from racer_pnp_perception.internal_val import net_from_run
from racer_pnp_perception.split import ROLE_TRAIN, Split, load_folds, load_split

log = logging.getLogger(__name__)

UNREFINED = "net_unrefined"
NO_CAP = "net_refined_no_cap"
SET_FOLD = "fold"


def perceivers_for(run: Path, live: LiveVehicle) -> dict[str, TrackPerceiver]:
    """Refined net, unrefined net (the same weights) and geometric-v1."""
    net = net_from_run(run)
    return {
        BACKEND_NET: TrackPerceiver(BACKEND_NET, live, net=net, model_id=run.name, refine_net=True),
        UNREFINED: TrackPerceiver(BACKEND_NET, live, net=net, model_id=run.name),
        NO_CAP: TrackPerceiver(
            BACKEND_NET,
            live,
            net=net,
            model_id=run.name,
            refine_net=True,
            refine_cfg=RefineConfig(clearance_cap=False),
        ),
        BACKEND_GEOMETRIC: TrackPerceiver(BACKEND_GEOMETRIC, live),
    }


def as_candidate(row: gp1.ScanRow, key: str) -> gp1.ScanRow:
    """The row with backend `key` stored under the name `net`, so the v0.2 tables and the decision
    read it unchanged."""

    def rename(d: dict[str, Any]) -> dict[str, Any]:
        out = {k: v for k, v in d.items() if k not in (BACKEND_NET, UNREFINED, NO_CAP)}
        if key in d:
            out[BACKEND_NET] = d[key]
        return out

    return dataclasses.replace(
        row,
        e_c=rename(row.e_c),
        e_far=rename(row.e_far),
        valid=rename(row.valid),
        intrusion=rename(row.intrusion),
        residual=rename(row.residual),
        deficit=rename(row.deficit),
        containment=rename(row.containment),
    )


def full_numbers(rows: list[gp1.ScanRow]) -> gp1.Numbers:
    backends = (BACKEND_NET, BACKEND_GEOMETRIC)
    run = [r for r in rows if r.set == gp1.SET_RUN]
    layout = [r for r in rows if r.set == gp1.SET_LAYOUT]
    h_run = [r for r in run if r.label is not None]
    import numpy as np

    boot = gp1.block_bootstrap_ratio(
        np.array([r.e_c[BACKEND_NET] for r in h_run]),
        np.array([r.e_c[BACKEND_GEOMETRIC] for r in h_run]),
        gp1.block_ids(h_run),
    )
    return gp1.Numbers(
        h_run={b: gp1.labelled_numbers(run, b) for b in backends},
        h_layout={b: gp1.labelled_numbers(layout, b) for b in backends},
        a_run={b: gp1.label_free_numbers(run, b) for b in backends},
        a_layout={b: gp1.label_free_numbers(layout, b) for b in backends},
        ratio_ci_hi=boot["hi"],
        cells=gp1.cell_table(rows),
    )


def fold_result(rows: list[gp1.ScanRow]) -> gp1.FoldResult:
    backends = (BACKEND_NET, BACKEND_GEOMETRIC)
    return gp1.FoldResult(
        {b: gp1.labelled_numbers(rows, b) for b in backends},
        {b: gp1.label_free_numbers(rows, b) for b in backends},
    )


def decide_variant(
    full_rows: list[gp1.ScanRow],
    fold_rows: dict[str, list[gp1.ScanRow]],
    key: str,
    by_construction: bool = False,
) -> dict[str, Any]:
    full = full_numbers([as_candidate(r, key) for r in full_rows])
    folds = {f: fold_result([as_candidate(r, key) for r in rows]) for f, rows in fold_rows.items()}
    items, outcome, failed = gp1.decide_gp1b(full, folds, by_construction)

    def dump(d: dict[str, Any]) -> dict[str, Any]:
        return {k: vars(v) for k, v in d.items()}

    return gp1._clean(
        {
            "candidate": key,
            "outcome": outcome,
            "failed_items": failed,
            "items": [vars(i) for i in items],
            "H-run": dump(full.h_run),
            "H-layout": dump(full.h_layout),
            "A-run": dump(full.a_run),
            "A-layout-2026-11-12": dump(full.a_layout),
            "bootstrap_ci_hi": full.ratio_ci_hi,
            "cells": [{**vars(c), "judged": c.judged, "ratio": c.ratio} for c in full.cells],
            "folds": {
                f: {"labelled": dump(r.labelled), "free": dump(r.free)} for f, r in folds.items()
            },
        }
    )


def heldout_bag_names(split: Split) -> list[str]:
    names = set(split.bag_names_with_role(gp1.ROLE_HELDOUT))
    names |= set(split.bag_names_with_role(gp1.ROLE_HELDOUT_LAYOUT))
    return sorted(names)


def format_report(result: dict[str, Any]) -> str:
    lines = [f"GP1b outcome: {result['refined']['outcome']}", ""]
    for label in ("refined", "refined_no_cap", "unrefined"):
        v = result[label]
        lines.append(f"[{label}] outcome: {v['outcome']}  failed items: {v['failed_items']}")
        for i in v["items"]:
            lines.append(f"  {i['number']}. {i['name']}: {'PASS' if i['passed'] else 'FAIL'}")
            if i["note"]:
                lines.append(f"     note: {i['note']}")
        for name in ("H-run", "A-run", "A-layout-2026-11-12"):
            for b, n in v[name].items():
                lines.append(f"  {name:20s} {b:9s} {json.dumps(n)}")
        for f, fr in v["folds"].items():
            for part in ("labelled", "free"):
                for b, n in fr[part].items():
                    lines.append(f"  {f} {part:8s} {b:9s} {json.dumps(n)}")
    lines.append("speed source per bag: " + json.dumps(result["speed_source"]))
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--split", type=Path, default=None)
    parser.add_argument("--labels", type=Path, required=True)
    parser.add_argument("--bags-root", type=Path, required=True)
    parser.add_argument("--run-full", type=Path, required=True)
    parser.add_argument("--run-f1", type=Path, required=True)
    parser.add_argument("--run-f2", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path("gp1b.json"))
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    split = load_split(args.split) if args.split else load_split()
    live = load_live_vehicle()
    folds = load_folds(split)
    full_p = perceivers_for(args.run_full, live)
    full_rows: list[gp1.ScanRow] = []
    for bag in heldout_bag_names(split):
        full_rows += gp1.collect_bag(args.bags_root / bag, args.labels, split, full_p, live)
    fold_rows: dict[str, list[gp1.ScanRow]] = {}
    sources: dict[str, str] = {}
    for name, run in (("F1", args.run_f1), ("F2", args.run_f2)):
        p = perceivers_for(run, live)
        rows: list[gp1.ScanRow] = []
        for bag in folds[name].heldout_bags:
            rows += gp1.collect_bag(
                args.bags_root / bag, args.labels, split, p, live, {ROLE_TRAIN: SET_FOLD}
            )
            sources[bag] = sl.speed_source(args.labels / "cache", bag)
        fold_rows[name] = rows
    result = {
        "split_sha256": split.sha256,
        "refined": decide_variant(full_rows, fold_rows, BACKEND_NET, True),
        "refined_no_cap": decide_variant(full_rows, fold_rows, NO_CAP),
        "unrefined": decide_variant(full_rows, fold_rows, UNREFINED),
        "speed_source": sources,
        "moving_rule": "wheel speed >= 0.2 m/s; ICP ground speed >= 0.2 m/s without /odom/wheel",
        "sets": {
            "full": {
                "scans": len(full_rows),
                "labelled": sum(r.label is not None for r in full_rows),
            },
            **{
                f: {"scans": len(r), "labelled": sum(x.label is not None for x in r)}
                for f, r in fold_rows.items()
            },
        },
    }
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False))
    print(format_report(result))
    return 0


if __name__ == "__main__":
    sys.exit(main())
