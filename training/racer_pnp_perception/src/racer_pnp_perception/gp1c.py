"""GP1c: `fused-v1` (the geometric tracer's corridor with the refined net's centre) against the
geometric tracer, decided as written in docs/notes/preregistration-pnp-v0.4.md section 3.2, plus the
measurement M-A1 (section 3.3).

Two commands, both through `TrackPerceiver.process`, the code the node runs:

    # item 6, development data (run as part of tuning, printed again in the report)
    uv run --group train --group bags python -m racer_pnp_perception.gp1c dev \\
        --labels <selflabels-v0.2> --bags-root <bags> --run-full <run> --run-f1 <run> \\
        --run-f2 <run> --fuse-config <json> --out gp1c_dev.json

    # the one pass on the 2026-11-14 perception `heldout` segments
    uv run --group train --group bags python -m racer_pnp_perception.gp1c score \\
        --labels <selflabels-v0.4> --bags-root <bags> --bundle <v2f bundle dir> \\
        --frozen-sha <git sha> --dev-json gp1c_dev.json --latency-json latency.json \\
        --canary-json canary.json --out gp1c.json

`score` refuses unless the constants file has the pre-registered SHA-256, the working tree is clean
and HEAD is the frozen SHA, and it refuses to overwrite an existing output: the fresh sets are
scored once, and a rerun forced by a bug is written to a new file and reported next to the first.
`reserve` segments are never loaded (`split_v04`).

Backends of one scan, side by side: `geometric`, `fused` (the candidate), `net` (v2 refined) and
`net_unrefined` (v2 weights alone), the last two being M-A1. In `decide_gp1c` the candidate is
stored under the key `net`, as in `gp1.decide_gp1b`, so the v0.2 tables and rules read it unchanged.

Items 3 and 5 are simply scored (the "holds by construction" flag was withdrawn: the output
polygon was measured and the construction does not keep the corridor, brief 38a). Coverage is its
own gate, GP1c-K (`decide_gp1c_k`, addendum 1 section 3.2), scored in the same single pass on A14.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import statistics
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from racer_track_perception import fuse
from racer_track_perception.bundle import load_bundle
from racer_track_perception.core import BACKEND_GEOMETRIC, BACKEND_NET, TrackPerceiver
from racer_track_perception.runtime import CircConv1dV0
from racer_track_perception.vehicle import LiveVehicle, load_live_vehicle

from racer_pnp_perception import corridor, gp1, gp1b, split_v04, windowed
from racer_pnp_perception.internal_val import net_from_run
from racer_pnp_perception.split import ROLE_HELDOUT, ROLE_TRAIN, load_folds, load_split

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------------------------
# Pre-registered thresholds (docs/notes/preregistration-pnp-v0.4.md section 3.2). Items 1 to 5 use
# the constants of `gp1` (v0.2 section 2, "unchanged"); the rest are here.
# ---------------------------------------------------------------------------------------------

# "7. Latency p99 under 10 ms on the dev machine" (yaml perception.latency_p99_max_ms).
LATENCY_P99_MAX_MS = 10.0
# "8. ... median lap time at most 1.03 times the geometric backend's" (yaml
# perception.sim_canary_max_lap_time_ratio).
CANARY_MAX_LAP_RATIO = 1.03
CANARY_MAX_WALL_CONTACTS = 0
# Backend keys of a row in this module (`net` is v2 refined, as in gp1b).
FUSED = "fused"
UNREFINED = gp1b.UNREFINED


OUTCOME_PASS = (
    "pass: candidate backend in the layout family it trained on, on a fresh session; not "
    "evidence for unseen layouts (geometric stays deployed until the on-car rule of 3.4)"
)
# Addendum 1 section 3.2, yaml perception_coverage.gate (evaluation/prereg/pnp-v0.4-addendum-1.yaml).
K_MAX_MEAN = 0.01
K_MAX_P99 = 0.05
OUTCOME_K_CANDIDATE = (
    "candidate: GP1c and GP1c-K both pass (geometric stays deployed until the on-car rule of 3.4)"
)
OUTCOME_K_NOT_KEPT = "better centre, corridor not kept (GP1c passed, GP1c-K failed)"

OUTCOME_UNDECIDED = "undecided: fewer than 300 labelled scans in H14, geometric stays"
OUTCOME_NO_BETTER = "no better than geometric in the layout (item 2 or 3 failed), geometric stays"
OUTCOME_NOT_ELSEWHERE = (
    "better where it trained, not shown elsewhere (items 2 and 3 passed, another item failed), "
    "geometric stays deployed"
)

NOT_MEASURED = "not measured: an item that was not computed is not a pass"


@dataclass(frozen=True)
class CanaryCase:
    """One L5 canary case run with the geometric and with the fused backend (same case, seed)."""

    name: str
    completed: bool  # the requested laps were driven
    wall_contacts: int
    median_lap_fused_s: float
    median_lap_geometric_s: float

    @property
    def ratio(self) -> float:
        g = self.median_lap_geometric_s
        return self.median_lap_fused_s / g if g > 0 else math.nan


@dataclass(frozen=True)
class DevResult:
    """Item 6: the `decide_gp1b` result of `fused-v1` on the development sets."""

    passed: bool
    failed_items: list[int]
    outcome: str
    note: str = ""


def decide_gp1c(
    h14: dict[str, gp1.Labelled],
    a14: dict[str, gp1.LabelFree],
    ratio_ci_hi: float,
    cells: list[gp1.Cell],
    dev: DevResult | None,
    latency_p99_ms: float | None,
    canary: list[CanaryCase] | None,
) -> tuple[list[gp1.Item], str, list[int]]:
    """The eight items of pre-registration v0.4 section 3.2. `h14` and `a14` map the backend names
    `net` (the candidate, fused-v1) and `geometric` to their numbers; `cells` are the cells of H14
    with the candidate as `net`.

    Outcome: item 1 failed gives "undecided". Otherwise item 2 or 3 failed gives "no better than
    geometric in the layout", otherwise any other failed item gives "better where it trained, not
    shown elsewhere", otherwise "pass". An item that was not measured does not pass."""
    net, geo = BACKEND_NET, BACKEND_GEOMETRIC
    items: list[gp1.Item] = []
    h = h14
    items.append(
        gp1.Item(
            1,
            "H14 has at least 300 labelled scans",
            {"labelled_scans": h[net].scans},
            {"min": gp1.MIN_H_RUN_LABELS},
            h[net].scans >= gp1.MIN_H_RUN_LABELS,
        )
    )
    ratio = h[net].e_c_m / h[geo].e_c_m if h[geo].e_c_m > 0 else math.nan
    items.append(
        gp1.Item(
            2,
            "centreline accuracy on H14",
            {
                "e_c_fused_m": h[net].e_c_m,
                "e_c_geometric_m": h[geo].e_c_m,
                "ratio": ratio,
                "ratio_ci_upper": ratio_ci_hi,
            },
            {"ratio_max": gp1.ITEM2_MAX_RATIO, "ci_upper_below": gp1.ITEM2_CI_UPPER_BELOW},
            gp1._le(ratio, gp1.ITEM2_MAX_RATIO) and bool(ratio_ci_hi < gp1.ITEM2_CI_UPPER_BELOW),
        )
    )
    items.append(
        gp1.Item(
            3,
            "range on H14",
            {"v_fused": h[net].v, "v_geometric": h[geo].v},
            {"v_fused_min": h[geo].v - gp1.ITEM3_V_MARGIN},
            gp1._ge(h[net].v, h[geo].v - gp1.ITEM3_V_MARGIN),
        )
    )
    judged = [c for c in cells if c.judged]
    bad = [
        f"{c.set}|{c.direction}|{c.speed_bin}"
        for c in judged
        if not gp1._le(c.ratio, gp1.CELL_MAX_RATIO)
    ]
    items.append(
        gp1.Item(
            4,
            "per cell on H14",
            {
                "cells_judged": len(judged),
                "cells_not_judged": len(cells) - len(judged),
                "worst_ratio": max((c.ratio for c in judged), default=math.nan),
                "failing_cells": bad,
            },
            {"ratio_max": gp1.CELL_MAX_RATIO, "min_labelled_scans": gp1.CELL_MIN_LABELS},
            not bad,
            "" if judged else "no cell has 40 labelled scans: nothing judged",
        )
    )
    ok, value, thr = gp1.label_free_ok(a14[net], a14[geo])
    items.append(
        gp1.Item(
            5,
            "label-free rule on A14",
            {**value, "moving_scans": a14[net].scans},
            thr,
            ok,
        )
    )
    if dev is None:
        items.append(
            gp1.Item(
                6, "v0.3 3.3 items 1 to 5 on the development sets", {}, {}, False, NOT_MEASURED
            )
        )
    else:
        items.append(
            gp1.Item(
                6,
                "v0.3 3.3 items 1 to 5 on the development sets",
                {"failed_items": dev.failed_items, "outcome": dev.outcome},
                {"all_items_pass": True},
                dev.passed,
                ("development data, reported and not evidence; " + dev.note).strip("; "),
            )
        )
    if latency_p99_ms is None:
        items.append(gp1.Item(7, "latency p99 on the dev machine", {}, {}, False, NOT_MEASURED))
    else:
        items.append(
            gp1.Item(
                7,
                "latency p99 on the dev machine",
                {"p99_ms": latency_p99_ms},
                {"p99_max_ms": LATENCY_P99_MAX_MS},
                bool(latency_p99_ms < LATENCY_P99_MAX_MS),
            )
        )
    if not canary:
        items.append(
            gp1.Item(8, "L5 PnP canary with the fused backend", {}, {}, False, NOT_MEASURED)
        )
    else:
        cases = {
            c.name: {
                "completed": c.completed,
                "wall_contacts": c.wall_contacts,
                "lap_ratio": c.ratio,
            }
            for c in canary
        }
        good = all(
            c.completed
            and c.wall_contacts <= CANARY_MAX_WALL_CONTACTS
            and gp1._le(c.ratio, CANARY_MAX_LAP_RATIO)
            for c in canary
        )
        items.append(
            gp1.Item(
                8,
                "L5 PnP canary with the fused backend",
                {"cases": cases},
                {
                    "wall_contacts_max": CANARY_MAX_WALL_CONTACTS,
                    "lap_ratio_max": CANARY_MAX_LAP_RATIO,
                },
                good,
            )
        )
    failed = [i.number for i in items if not i.passed]
    if 1 in failed:
        outcome = OUTCOME_UNDECIDED
    elif 2 in failed or 3 in failed:
        outcome = OUTCOME_NO_BETTER
    elif failed:
        outcome = OUTCOME_NOT_ELSEWHERE
    else:
        outcome = OUTCOME_PASS
    return items, outcome, failed


def decide_gp1c_k(
    losses: list[float], excesses: list[float], worst: dict[str, Any] | None = None
) -> tuple[list[gp1.Item], bool]:
    """GP1c-K on A14: coverage loss and excess of the fused corridor polygon against the geometric
    one (`corridor.compare`, one value per moving scan, NaN entries dropped): mean at most 0.01 and
    p99 at most 0.05 for each. Returns the four items and whether all pass. An empty set fails."""
    items: list[gp1.Item] = []
    for n, (name, vals) in enumerate((("loss", losses), ("excess", excesses)), start=1):
        v = np.array([x for x in vals if math.isfinite(x)], dtype=np.float64)
        mean = float(v.mean()) if len(v) else math.nan
        p99 = float(np.percentile(v, 99)) if len(v) else math.nan
        note = "" if len(v) else "no scan compared: nothing to judge"
        items.append(
            gp1.Item(
                2 * n - 1,
                f"mean coverage {name} on A14",
                {"mean": mean, "scans": len(v)},
                {"max": K_MAX_MEAN},
                gp1._le(mean, K_MAX_MEAN),
                note,
            )
        )
        items.append(
            gp1.Item(
                2 * n,
                f"p99 coverage {name} on A14",
                {"p99": p99, "worst_scan": worst.get(name) if worst else None},
                {"max": K_MAX_P99},
                gp1._le(p99, K_MAX_P99),
                note,
            )
        )
    return items, all(i.passed for i in items)


def combined_outcome(gp1c_outcome: str, k_passed: bool) -> str:
    """Candidate only if GP1c and GP1c-K both pass."""
    if gp1c_outcome == OUTCOME_PASS:
        return OUTCOME_K_CANDIDATE if k_passed else OUTCOME_K_NOT_KEPT
    return gp1c_outcome


def dev_result_from_gp1b(items: list[gp1.Item], failed: list[int]) -> DevResult:
    """Item 6 from the items of `gp1.decide_gp1b` for `fused-v1`."""
    return DevResult(
        passed=not failed,
        failed_items=list(failed),
        outcome=gp1.OUTCOME_GP1B_PASS if not failed else gp1.OUTCOME_GP1B_NO_BETTER,
        note="; ".join(f"item {i.number}: {i.note}" for i in items if i.note),
    )


def median_lap_s(lap_times_s: list[float]) -> float:
    return float(statistics.median(lap_times_s)) if lap_times_s else math.nan


# ---------------------------------------------------------------------------------------------
# Perceivers and tables
# ---------------------------------------------------------------------------------------------


def perceivers_from_net(
    net: CircConv1dV0, live: LiveVehicle, model_id: str, cfg: fuse.FuseConfig
) -> dict[str, TrackPerceiver]:
    """Geometric, fused (the candidate), v2 refined and v2 unrefined, from one set of weights."""
    return {
        BACKEND_GEOMETRIC: TrackPerceiver(BACKEND_GEOMETRIC, live),
        FUSED: TrackPerceiver(BACKEND_NET, live, net=net, model_id=model_id, fuse_cfg=cfg),
        BACKEND_NET: TrackPerceiver(BACKEND_NET, live, net=net, model_id=model_id, refine_net=True),
        UNREFINED: TrackPerceiver(BACKEND_NET, live, net=net, model_id=model_id),
    }


def score_row_with_coverage(
    row: gp1.ScanRow,
    scans: Any,
    i: int,
    perceivers: dict[str, TrackPerceiver],
    live: LiveVehicle,
) -> None:
    """`gp1.score_row` plus the corridor polygon comparison of fused against geometric."""
    gp1.score_row(row, scans, i, perceivers, live)
    geometry = (scans.angle_min, scans.angle_increment, scans.range_min, scans.range_max)
    geo = perceivers[BACKEND_GEOMETRIC].process(scans.ranges[i], *geometry)
    fused = perceivers[FUSED].process(scans.ranges[i], *geometry)
    row.coverage = corridor.compare(geo, fused)


def candidate_rows(rows: list[gp1.ScanRow]) -> list[gp1.ScanRow]:
    return [gp1b.as_candidate(r, FUSED) for r in rows]


def numbers_h14_a14(
    rows: list[gp1.ScanRow],
) -> tuple[dict[str, gp1.Labelled], dict[str, gp1.LabelFree], float, list[gp1.Cell]]:
    """What `decide_gp1c` reads, from rows scored with all four backends (fused as the candidate)."""
    cand = candidate_rows(rows)
    backends = (BACKEND_NET, BACKEND_GEOMETRIC)
    labelled = [r for r in cand if r.label is not None]
    boot = gp1.block_bootstrap_ratio(
        np.array([r.e_c[BACKEND_NET] for r in labelled]),
        np.array([r.e_c[BACKEND_GEOMETRIC] for r in labelled]),
        gp1.block_ids(labelled),
    )
    return (
        {b: gp1.labelled_numbers(cand, b) for b in backends},
        {b: gp1.label_free_numbers(cand, b) for b in backends},
        boot["hi"],
        gp1.cell_table(cand),
    )


def side_by_side(rows: list[gp1.ScanRow]) -> dict[str, Any]:
    """M-A1 and the GP1c tables: every backend on H14 and A14, and E_c per cell with the bootstrap
    of the ratio to geometric for each of the three non-geometric backends."""
    out: dict[str, Any] = {"H14": {}, "A14": {}, "cells": {}, "ratio_to_geometric": {}}
    labelled = [r for r in rows if r.label is not None]
    for b in (BACKEND_GEOMETRIC, FUSED, BACKEND_NET, UNREFINED):
        out["H14"][b] = vars(gp1.labelled_numbers(rows, b))
        out["A14"][b] = vars(gp1.label_free_numbers(rows, b))
        groups: dict[str, list[gp1.ScanRow]] = {}
        for r in labelled:
            groups.setdefault(f"{r.direction}|{gp1.speed_bin(r.speed_mps)}", []).append(r)
        out["cells"][b] = {
            k: {"labelled_scans": len(v), "e_c_m": gp1.nanmean([r.e_c[b] for r in v])}
            for k, v in sorted(groups.items())
        }
        if b != BACKEND_GEOMETRIC and labelled:
            out["ratio_to_geometric"][b] = gp1.block_bootstrap_ratio(
                np.array([r.e_c[b] for r in labelled]),
                np.array([r.e_c[BACKEND_GEOMETRIC] for r in labelled]),
                gp1.block_ids(labelled),
            )
    return gp1._clean(out)


def gp1c_report(
    rows: list[gp1.ScanRow],
    dev: DevResult | None,
    latency_p99_ms: float | None,
    canary: list[CanaryCase] | None,
) -> dict[str, Any]:
    h14, a14, ci_hi, cells = numbers_h14_a14(rows)
    items, outcome, failed = decide_gp1c(h14, a14, ci_hi, cells, dev, latency_p99_ms, canary)
    cov = [r.coverage for r in rows if r.coverage is not None]
    worst = {}
    for key in ("loss", "excess"):
        finite = [
            {"bag": r.bag, "index": r.index, key: r.coverage[key]}
            for r in rows
            if r.coverage is not None and math.isfinite(r.coverage[key])
        ]
        worst[key] = max(finite, key=lambda w, key=key: w[key], default=None)
    k_items, k_ok = decide_gp1c_k([c["loss"] for c in cov], [c["excess"] for c in cov], worst)
    return gp1._clean(
        {
            "gate": "GP1c",
            "outcome": combined_outcome(outcome, k_ok),
            "gp1c_outcome": outcome,
            "gp1c_k": {"passed": k_ok, "items": [vars(i) for i in k_items]},
            "failed_items": failed,
            "items": [vars(i) for i in items],
            "bootstrap_ci_hi": ci_hi,
            "cells": [{**vars(c), "judged": c.judged, "ratio": c.ratio} for c in cells],
            "sets": {
                "H14_labelled": sum(r.label is not None for r in rows),
                "A14_moving": len(rows),
            },
            "M-A1": side_by_side(rows),
        }
    )


def format_report(result: dict[str, Any]) -> str:
    lines = [f"GP1c outcome: {result['outcome']}", f"failed items: {result['failed_items']}"]
    lines.append(f"GP1c-K: {'PASS' if result['gp1c_k']['passed'] else 'FAIL'}")
    for i in result["gp1c_k"]["items"]:
        lines.append(
            f"  K{i['number']}. {i['name']}: {'PASS' if i['passed'] else 'FAIL'} {i['value']}"
        )
    for i in result["items"]:
        lines.append(f"  {i['number']}. {i['name']}: {'PASS' if i['passed'] else 'FAIL'}")
        if i["note"]:
            lines.append(f"     note: {i['note']}")
    m = result["M-A1"]
    for part in ("H14", "A14"):
        for b, n in m[part].items():
            lines.append(f"  {part:4s} {b:13s} {json.dumps(n)}")
    for b, n in m["ratio_to_geometric"].items():
        lines.append(f"  E_c ratio to geometric, {b}: {json.dumps(n)}")
    return "\n".join(lines)


# ---------------------------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------------------------


def _git(*args: str) -> str:
    repo = Path(__file__).resolve().parents[4]
    return subprocess.run(
        ["git", "-C", str(repo), *args], capture_output=True, text=True, check=True
    ).stdout.strip()


def refuse_unless_frozen(frozen_sha: str) -> None:
    """The one-pass rule: HEAD is the frozen SHA and the tree is clean."""
    head = _git("rev-parse", "HEAD")
    if head != frozen_sha:
        raise SystemExit(f"refusing to score: HEAD is {head}, the frozen candidate is {frozen_sha}")
    if _git("status", "--porcelain", "--untracked-files=no"):
        raise SystemExit("refusing to score: the working tree has uncommitted changes")


def dev_main(args: argparse.Namespace) -> int:
    """Item 6: `gp1.decide_gp1b` with `fused-v1` as the candidate, on the development sets."""
    split = load_split(args.split) if args.split else load_split()
    live = load_live_vehicle()
    cfg = fuse.FuseConfig.from_dict(json.loads(Path(args.fuse_config).read_text()))
    folds = load_folds(split)

    def perceivers(run: Path) -> dict[str, TrackPerceiver]:
        p = perceivers_from_net(net_from_run(run), live, run.name, cfg)
        # gp1b reads the candidate as `net`: fused takes that name, refined and unrefined move aside
        return {
            BACKEND_GEOMETRIC: p[BACKEND_GEOMETRIC],
            BACKEND_NET: p[FUSED],
            gp1b.UNREFINED: p[UNREFINED],
            "net_refined": p[BACKEND_NET],
        }

    full_p = perceivers(args.run_full)
    full_rows: list[gp1.ScanRow] = []
    for bag in gp1b.heldout_bag_names(split):
        full_rows += gp1.collect_bag(args.bags_root / bag, args.labels, split, full_p, live)
    fold_rows: dict[str, list[gp1.ScanRow]] = {}
    for name, run in (("F1", args.run_f1), ("F2", args.run_f2)):
        p = perceivers(run)
        rows: list[gp1.ScanRow] = []
        for bag in folds[name].heldout_bags:
            rows += gp1.collect_bag(
                args.bags_root / bag, args.labels, split, p, live, {ROLE_TRAIN: gp1b.SET_FOLD}
            )
        fold_rows[name] = rows
    variant = gp1b.decide_variant(full_rows, fold_rows, BACKEND_NET, by_construction=False)
    result = {
        "fuse_config": cfg.as_dict(),
        "development": True,
        "fused-v1": variant,
        "git_sha": _git("rev-parse", "HEAD"),
    }
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False))
    print(json.dumps({k: variant[k] for k in ("outcome", "failed_items")}, indent=2))
    return 0


def load_dev(path: Path) -> DevResult:
    v = json.loads(path.read_text())["fused-v1"]
    return DevResult(
        passed=not v["failed_items"],
        failed_items=list(v["failed_items"]),
        outcome=str(v["outcome"]),
        note="; ".join(i["note"] for i in v["items"] if i.get("note")),
    )


def load_canary(path: Path) -> list[CanaryCase]:
    return [
        CanaryCase(
            name=str(c["name"]),
            completed=bool(c["completed"]),
            wall_contacts=int(c["wall_contacts"]),
            median_lap_fused_s=median_lap_s(c["lap_times_fused_s"]),
            median_lap_geometric_s=median_lap_s(c["lap_times_geometric_s"]),
        )
        for c in json.loads(path.read_text())["cases"]
    ]


def score_main(args: argparse.Namespace) -> int:
    """The one pass on the perception `heldout` segments of 2026-11-14."""
    if args.out.exists():
        raise SystemExit(f"refusing to overwrite {args.out}: the fresh sets are scored once")
    refuse_unless_frozen(args.frozen_sha)
    doc, digest = split_v04.read_yaml(args.prereg)
    split = split_v04.perception_split_from_doc(doc, digest)
    live = load_live_vehicle()
    bundle = load_bundle(args.bundle, "track_perception", live)
    cfg = bundle.fuse_config
    if cfg is None:
        raise SystemExit(f"{args.bundle} does not declare fused-v1")
    net = CircConv1dV0.from_npz(bundle.payload_path("weights.npz"))
    perceivers = perceivers_from_net(net, live, bundle.model_id, cfg)
    rows: list[gp1.ScanRow] = []
    for bag in sorted(split.bags):
        # the windowed cache only: a whole-bag read would decode reserve segments
        windowed.require_cache(args.labels / "cache", bag)
        rows += gp1.collect_bag(
            args.bags_root / bag,
            args.labels,
            split,
            perceivers,
            live,
            {ROLE_HELDOUT: gp1.SET_RUN},
            scorer=score_row_with_coverage,
        )
    for r in rows:  # defence in depth: no scored scan lies in a reserve segment
        split_v04.assert_not_reserve(doc, r.bag, r.t_s, r.t_s)
    dev = load_dev(args.dev_json) if args.dev_json else None
    p99 = None
    if args.latency_json:
        lat = json.loads(args.latency_json.read_text())
        p99 = float(lat["scan_to_track"][BACKEND_NET]["p99"])
    canary = load_canary(args.canary_json) if args.canary_json else None
    result = gp1c_report(rows, dev, p99, canary)
    result.update(
        {
            "frozen_sha": args.frozen_sha,
            "prereg_sha256": digest,
            "bundle": bundle.model_id,
            "fuse_config": cfg.as_dict(),
            "segments": sorted(
                {f"{r.bag}/{r.segment}" for r in rows},
            ),
        }
    )
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False))
    print(format_report(result))
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    dev = sub.add_parser("dev", help="item 6 on the development sets")
    dev.add_argument("--split", type=Path, default=None)
    dev.add_argument("--labels", type=Path, required=True)
    dev.add_argument("--bags-root", type=Path, required=True)
    dev.add_argument("--run-full", type=Path, required=True)
    dev.add_argument("--run-f1", type=Path, required=True)
    dev.add_argument("--run-f2", type=Path, required=True)
    dev.add_argument("--fuse-config", type=Path, required=True)
    dev.add_argument("--out", type=Path, default=Path("gp1c_dev.json"))
    score = sub.add_parser("score", help="the one pass on the 2026-11-14 held-out segments")
    score.add_argument("--prereg", type=Path, default=None)
    score.add_argument("--labels", type=Path, required=True)
    score.add_argument("--bags-root", type=Path, required=True)
    score.add_argument("--bundle", type=Path, required=True)
    score.add_argument("--frozen-sha", required=True)
    score.add_argument("--dev-json", type=Path, default=None)
    score.add_argument("--latency-json", type=Path, default=None)
    score.add_argument("--canary-json", type=Path, default=None)
    score.add_argument("--out", type=Path, default=Path("gp1c.json"))
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    return dev_main(args) if args.command == "dev" else score_main(args)


if __name__ == "__main__":
    sys.exit(main())
