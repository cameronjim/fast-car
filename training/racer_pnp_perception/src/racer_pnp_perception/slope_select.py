"""Choice of `FuseConfig.max_shift_slope` by the rule of addendum 1 section 3.2 (brief 38a).

The rule, fixed before any number was seen: the largest value of the grid for which EVERY
development set meets the development margin (mean loss and mean excess at most 0.005, p99 of each
at most 0.025). Not chosen by E_c, which is only printed for information. If no value meets the
margin nothing is frozen, no fresh set is read, and the report says "GP1c not attempted: fusion
cannot keep the corridor".

    uv run --group train --group bags python -m racer_pnp_perception.slope_select \\
        --labels <selflabels-v0.2> --bags-root <bags> --run-full <run> --run-f1 <run> \\
        --run-f2 <run> --out slope_table.json

Development sets, as in `gp1c dev`: A-run and A-layout (the full model on the v0.2 `heldout` and
`heldout_layout` scans, all moving scans) and the folds F1 and F2 with their fold models. The
2026-11-14 bags are never read here (the v0.2 split does not contain them).
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
from pathlib import Path
from typing import Any

from racer_track_perception import fuse
from racer_track_perception.core import BACKEND_GEOMETRIC
from racer_track_perception.vehicle import load_live_vehicle

from racer_pnp_perception import corridor, gp1, gp1b, gp1c
from racer_pnp_perception.internal_val import net_from_run
from racer_pnp_perception.split import ROLE_TRAIN, load_folds, load_split

log = logging.getLogger(__name__)

# Addendum 1 section 3.2, yaml perception_coverage (evaluation/prereg/pnp-v0.4-addendum-1.yaml).
SLOPE_GRID = (0.05, 0.10, 0.15, 0.20, 0.30)
MARGIN_MAX_MEAN = 0.005
MARGIN_MAX_P99 = 0.025
NOT_ATTEMPTED = "GP1c not attempted: fusion cannot keep the corridor"


def meets_margin(cell: dict[str, Any]) -> bool:
    """One slope on one development set: `cell` has the keys of `corridor.summarise`."""
    loss, excess = cell.get("loss"), cell.get("excess")
    if not loss or not excess:
        return False  # nothing compared is not a pass
    return bool(
        loss["mean"] <= MARGIN_MAX_MEAN
        and excess["mean"] <= MARGIN_MAX_MEAN
        and loss["p99"] <= MARGIN_MAX_P99
        and excess["p99"] <= MARGIN_MAX_P99
    )


def choose_slope(table: dict[float, dict[str, dict[str, Any]]]) -> float | None:
    """`table[slope][set]` as above. The largest slope of the grid that meets the margin on every
    set, or None. Only the grid values are considered."""
    ok = [
        s
        for s in SLOPE_GRID
        if s in table and table[s] and all(meets_margin(c) for c in table[s].values())
    ]
    return max(ok) if ok else None


def mean_e_c(rows: list[gp1.ScanRow]) -> float:
    return gp1.nanmean([r.e_c[gp1c.FUSED] for r in rows if r.label is not None])


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--split", type=Path, default=None)
    parser.add_argument("--labels", type=Path, required=True)
    parser.add_argument("--bags-root", type=Path, required=True)
    parser.add_argument("--run-full", type=Path, required=True)
    parser.add_argument("--run-f1", type=Path, required=True)
    parser.add_argument("--run-f2", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path("slope_table.json"))
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    split = load_split(args.split) if args.split else load_split()
    live = load_live_vehicle()
    folds = load_folds(split)
    table: dict[float, dict[str, dict[str, Any]]] = {}
    e_c: dict[float, dict[str, float]] = {}
    nets = {
        "full": net_from_run(args.run_full),
        "F1": net_from_run(args.run_f1),
        "F2": net_from_run(args.run_f2),
    }
    for slope in SLOPE_GRID:
        cfg = fuse.FuseConfig(max_shift_slope=slope)
        sets: dict[str, list[gp1.ScanRow]] = {}
        p = gp1c.perceivers_from_net(nets["full"], live, "full", cfg)
        p = {k: v for k, v in p.items() if k in (BACKEND_GEOMETRIC, gp1c.FUSED)}
        full_rows: list[gp1.ScanRow] = []
        for bag in gp1b.heldout_bag_names(split):
            full_rows += gp1.collect_bag(
                args.bags_root / bag,
                args.labels,
                split,
                p,
                live,
                scorer=gp1c.score_row_with_coverage,
            )
        sets["A-run"] = [r for r in full_rows if r.set == gp1.SET_RUN]
        sets["A-layout-2026-11-12"] = [r for r in full_rows if r.set == gp1.SET_LAYOUT]
        for name, key in (("F1", "F1"), ("F2", "F2")):
            pf = gp1c.perceivers_from_net(nets[key], live, key, cfg)
            pf = {k: v for k, v in pf.items() if k in (BACKEND_GEOMETRIC, gp1c.FUSED)}
            rows: list[gp1.ScanRow] = []
            for bag in folds[name].heldout_bags:
                rows += gp1.collect_bag(
                    args.bags_root / bag,
                    args.labels,
                    split,
                    pf,
                    live,
                    {ROLE_TRAIN: gp1b.SET_FOLD},
                    scorer=gp1c.score_row_with_coverage,
                )
            sets[f"fold {name}"] = rows
        table[slope] = {
            n: corridor.summarise([r.coverage for r in rows if r.coverage is not None])
            for n, rows in sets.items()
        }
        e_c[slope] = {n: mean_e_c(rows) for n, rows in sets.items()}
        log.info("slope %.2f: %s", slope, json.dumps(table[slope]))
        # one slope at a time on disk, so a sleep or a kill loses minutes
        args.out.write_text(
            json.dumps(
                {"table": {str(k): v for k, v in table.items()}, "e_c_info": e_c},
                indent=2,
                allow_nan=False,
                default=lambda x: None if isinstance(x, float) and math.isnan(x) else str(x),
            )
        )
    chosen = choose_slope(table)
    result = {
        "table": {str(k): v for k, v in table.items()},
        "e_c_info": {
            str(k): {n: (None if math.isnan(x) else x) for n, x in v.items()}
            for k, v in e_c.items()
        },
        "chosen_max_shift_slope": chosen,
        "outcome": NOT_ATTEMPTED if chosen is None else f"max_shift_slope {chosen}",
        "rule": "largest grid value meeting the development margin on every development set",
    }
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False))
    print(json.dumps({k: result[k] for k in ("chosen_max_shift_slope", "outcome")}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
