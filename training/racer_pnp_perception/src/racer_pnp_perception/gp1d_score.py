"""The commands of GP1d after the freeze (decision rules: `gp1d`).

    # item 6, development data (run once with the frozen config, reported again in the note)
    uv run --group train --group bags python -m racer_pnp_perception.gp1d_score dev \\
        --labels <selflabels-v0.2> --bags-root <bags> --run-full <run> --run-f1 <run> \\
        --run-f2 <run> --fuse-config <json> --out gp1d_dev.json

    # labels of the two held-out sets (after the freeze commit only)
    uv run --group bags python -m racer_pnp_perception.gp1d_score prepare \\
        --bags-root <bags> --out14 <selflabels-14> --out22 <selflabels-22> --frozen-sha <sha>

    # the one pass on H14/A14 and H22/A22
    uv run --group bags python -m racer_pnp_perception.gp1d_score score \\
        --labels14 <selflabels-14> --labels22 <selflabels-22> --bags-root <bags> \\
        --bundle <v2g bundle dir> --frozen-sha <sha> --dev-json gp1d_dev.json \\
        --latency-json latency.json --canary-json canary.json --out gp1d.json

`prepare` and `score` refuse unless the constants file has the registered SHA-256, the working tree
is clean and HEAD is the frozen SHA. `score` refuses to overwrite an existing output: the held-out
sets are scored once, together, and a rerun forced by a bug is written to a new file and reported
next to the first. `reserve` segments and every bag after 2026-11-22 are refused (`gp1d`).

Per held-out set the four backends of one scan are side by side: `geometric`, `fused` (the
candidate, fused-v2), `net` (v2 refined) and `net_unrefined` (v2 weights alone).
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import multiprocessing
import sys
from pathlib import Path
from typing import Any

from racer_track_perception import fuse_v2
from racer_track_perception.bundle import load_bundle
from racer_track_perception.core import BACKEND_GEOMETRIC, BACKEND_NET, TrackPerceiver
from racer_track_perception.runtime import CircConv1dV0
from racer_track_perception.vehicle import load_live_vehicle

from racer_pnp_perception import gp1, gp1b, gp1c, gp1d, heldout22, split_v04, windowed
from racer_pnp_perception import selflabel as sl
from racer_pnp_perception.internal_val import net_from_run
from racer_pnp_perception.split import ROLE_HELDOUT, ROLE_TRAIN, Split, load_folds, load_split

log = logging.getLogger(__name__)

MOVING_MPS = 0.2


def set_report(label: str, rows: list[gp1.ScanRow]) -> tuple[dict[str, Any], list[int], bool]:
    """Items 1 to 5, GP1d-K and M-A1 of one held-out set from rows scored with all four backends."""
    h, a, ci_hi, cells = gp1c.numbers_h14_a14(rows)
    items, failed = gp1d.decide_gp1d(label, h, a, ci_hi, cells)
    cov = [r.coverage for r in rows if r.coverage is not None]
    worst: dict[str, Any] = {}
    for key in ("loss", "excess"):
        finite = [
            {"bag": r.bag, "index": r.index, key: r.coverage[key]}
            for r in rows
            if r.coverage is not None and math.isfinite(r.coverage[key])
        ]
        worst[key] = max(finite, key=lambda w, key=key: w[key], default=None)
    k_items, k_ok = gp1d.decide_gp1d_k(
        label, [c["loss"] for c in cov], [c["excess"] for c in cov], worst
    )
    report = {
        "items": [vars(i) for i in items],
        "failed_items": failed,
        "k_items": [vars(i) for i in k_items],
        "k_passed": k_ok,
        "bootstrap_ci_hi": ci_hi,
        "cells": [{**vars(c), "judged": c.judged, "ratio": c.ratio} for c in cells],
        "counts": {"labelled": sum(r.label is not None for r in rows), "moving": len(rows)},
        "side_by_side": gp1c.side_by_side(rows),
    }
    return report, failed, k_ok


def global_items(
    dev_json: Path | None, latency_json: Path | None, canary_json: Path | None
) -> tuple[list[gp1.Item], list[int]]:
    dev = gp1c.load_dev(dev_json) if dev_json else None
    p99 = None
    if latency_json:
        p99 = float(json.loads(latency_json.read_text())["scan_to_track"][BACKEND_NET]["p99"])
    canary = gp1c.load_canary(canary_json) if canary_json else None
    return gp1d.decide_global(
        None if dev is None else dev.passed, dev.note if dev else "", p99, canary
    )


def format_report(result: dict[str, Any]) -> str:
    lines = [f"GP1d outcome: {result['outcome']}"]
    for label, s in sorted(result["sets"].items()):
        lines.append(f"set {label}: {s['counts']}")
        for i in s["items"]:
            lines.append(f"  {i['number']}. {i['name']}: {'PASS' if i['passed'] else 'FAIL'}")
            lines.append(f"     {json.dumps(i['value'])}")
        for i in s["k_items"]:
            lines.append(
                f"  K{i['number']}. {i['name']}: {'PASS' if i['passed'] else 'FAIL'} {i['value']}"
            )
        sbs = s["side_by_side"]
        for b, n in sbs["ratio_to_geometric"].items():
            lines.append(f"  E_c ratio to geometric, {b}: {json.dumps(n)}")
    for i in result["global_items"]:
        lines.append(
            f"  {i['number']}. {i['name']}: {'PASS' if i['passed'] else 'FAIL'} {i['note']}"
        )
    lines.append(gp1d.ALWAYS_SAID)
    return "\n".join(lines)


def refuse_unless_ready(frozen_sha: str) -> dict[str, Any]:
    """The one-pass rule: the constants file is the registered one, HEAD is the frozen SHA and the
    tree is clean."""
    doc = gp1d.load_prereg()
    gp1c.refuse_unless_frozen(frozen_sha)
    return doc


def dev_main(args: argparse.Namespace) -> int:
    """Item 6: `gp1.decide_gp1b` with fused-v2 as the candidate, on the development sets."""
    gp1d.load_prereg()
    split = load_split()
    live = load_live_vehicle()
    cfg = fuse_v2.FuseV2Config.from_dict(json.loads(Path(args.fuse_config).read_text()))
    folds = load_folds(split)

    def perceivers(run: Path) -> dict[str, TrackPerceiver]:
        p = gp1c.perceivers_from_net(net_from_run(run), live, run.name, cfg)
        return {
            BACKEND_GEOMETRIC: p[BACKEND_GEOMETRIC],
            BACKEND_NET: p[gp1c.FUSED],
            gp1b.UNREFINED: p[gp1c.UNREFINED],
            "net_refined": p[BACKEND_NET],
        }

    full_p = perceivers(args.run_full)
    full_rows: list[gp1.ScanRow] = []
    for bag in gp1b.heldout_bag_names(split):
        gp1d.assert_bag_allowed(bag, "development")
        full_rows += gp1.collect_bag(args.bags_root / bag, args.labels, split, full_p, live)
    fold_rows: dict[str, list[gp1.ScanRow]] = {}
    for name, run in (("F1", args.run_f1), ("F2", args.run_f2)):
        p = perceivers(run)
        rows: list[gp1.ScanRow] = []
        for bag in folds[name].heldout_bags:
            gp1d.assert_bag_allowed(bag, "development")
            rows += gp1.collect_bag(
                args.bags_root / bag, args.labels, split, p, live, {ROLE_TRAIN: gp1b.SET_FOLD}
            )
        fold_rows[name] = rows
    variant = gp1b.decide_variant(full_rows, fold_rows, BACKEND_NET, by_construction=False)
    result = {
        "fuse_config": cfg.as_dict(),
        "development": True,
        "fused-v1": variant,  # key read by gp1c.load_dev; the candidate here is fused-v2
        "candidate": "fused-v2",
        "git_sha": gp1c._git("rev-parse", "HEAD"),
    }
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False))
    print(json.dumps({k: variant[k] for k in ("outcome", "failed_items")}, indent=2))
    return 0


def prepare_main(args: argparse.Namespace) -> int:
    """Windowed cache and self-labels of the two held-out sets, after the freeze commit."""
    doc = refuse_unless_ready(args.frozen_sha)
    del doc
    live = load_live_vehicle()
    # set 14: only the heldout windows are decoded (windowed.py), then labelled
    pre, digest = split_v04.read_yaml()
    split14 = split_v04.perception_split_from_doc(pre, digest)
    for name in sorted(split14.bags):
        gp1d.assert_bag_allowed(name, "heldout")
        windowed.build_windowed_cache(args.bags_root / name, split14, args.out14 / "cache", doc=pre)
        sl.label_bag(args.bags_root / name, args.out14, MOVING_MPS, args.workers, live, split14)
    # set 22: the five runs of the bag, whole
    bag = args.bags_root / gp1d.HELDOUT_22_BAG
    gp1d.assert_bag_allowed(bag.name, "heldout")
    t0 = first_scan_log_time_s(bag)
    split22 = heldout22.split_22(args.session_dir, t0)
    sl.label_bag(bag, args.out22, MOVING_MPS, args.workers, live, split22)
    return 0


def first_scan_log_time_s(bag_dir: Path) -> float:
    """Recorder log time of the first /scan message (one timestamp; no scan is used)."""
    sl._tools()
    from follower_replay.bag import read_bag

    data = read_bag(bag_dir, window_s=(0.0, 5.0))
    if not data.scans:
        raise RuntimeError(f"{bag_dir}: no scan in the first seconds")
    return float(data.scans[0].log_time_s)


def collect_set_14(
    labels14: Path, bags_root: Path, perceivers: dict[str, TrackPerceiver], live: Any
) -> list[gp1.ScanRow]:
    doc, digest = split_v04.read_yaml()
    split = split_v04.perception_split_from_doc(doc, digest)
    rows: list[gp1.ScanRow] = []
    for bag in sorted(split.bags):
        gp1d.assert_bag_allowed(bag, "heldout")
        windowed.require_cache(labels14 / "cache", bag)  # never a whole-bag read
        rows += gp1.collect_bag(
            bags_root / bag,
            labels14,
            split,
            perceivers,
            live,
            {ROLE_HELDOUT: gp1.SET_RUN},
            scorer=gp1c.score_row_with_coverage,
        )
    for r in rows:  # defence in depth: no scored scan lies in a reserve segment
        split_v04.assert_not_reserve(doc, r.bag, r.t_s, r.t_s)
    return rows


def collect_set_22(
    labels22: Path,
    bags_root: Path,
    session_dir: Path,
    perceivers: dict[str, TrackPerceiver],
    live: Any,
) -> list[gp1.ScanRow]:
    gp1d.assert_bag_allowed(gp1d.HELDOUT_22_BAG, "heldout")
    scans_cache = labels22 / "cache" / f"{gp1d.HELDOUT_22_BAG}.scans.npz"
    if not scans_cache.is_file():
        raise FileNotFoundError(f"{scans_cache} is missing: run `prepare` first")
    t0 = first_scan_log_time_s(bags_root / gp1d.HELDOUT_22_BAG)
    split: Split = heldout22.split_22(session_dir, t0)
    return gp1.collect_bag(
        bags_root / gp1d.HELDOUT_22_BAG,
        labels22,
        split,
        perceivers,
        live,
        {ROLE_HELDOUT: gp1.SET_RUN},
        scorer=gp1c.score_row_with_coverage,
    )


def score_main(args: argparse.Namespace) -> int:
    """The one pass on H14/A14 and H22/A22, together."""
    if args.out.exists():
        raise SystemExit(f"refusing to overwrite {args.out}: the held-out sets are scored once")
    doc = refuse_unless_ready(args.frozen_sha)
    live = load_live_vehicle()
    bundle = load_bundle(args.bundle, "track_perception", live)
    cfg = bundle.fuse_config
    if not isinstance(cfg, fuse_v2.FuseV2Config):
        raise SystemExit(f"{args.bundle} does not declare fused-v2")
    net = CircConv1dV0.from_npz(bundle.payload_path("weights.npz"))
    perceivers = gp1c.perceivers_from_net(net, live, bundle.model_id, cfg)
    rows = {
        gp1d.SET_14: collect_set_14(args.labels14, args.bags_root, perceivers, live),
        gp1d.SET_22: collect_set_22(
            args.labels22, args.bags_root, args.session_dir, perceivers, live
        ),
    }
    sets: dict[str, Any] = {}
    set_failed: dict[str, list[int]] = {}
    k_passed: dict[str, bool] = {}
    for label, r in rows.items():
        sets[label], set_failed[label], k_passed[label] = set_report(label, r)
    g_items, g_failed = global_items(args.dev_json, args.latency_json, args.canary_json)
    outcome = gp1d.combine_outcomes(set_failed, g_failed, k_passed)
    result = gp1._clean(
        {
            "gate": "GP1d",
            "outcome": outcome,
            "sets": sets,
            "global_items": [vars(i) for i in g_items],
            "global_failed": g_failed,
            "frozen_sha": args.frozen_sha,
            "prereg_sha256": gp1d.PREREG_SHA256,
            "bundle": bundle.model_id,
            "fuse_config": cfg.as_dict(),
            "always_said": gp1d.ALWAYS_SAID,
        }
    )
    del doc
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False))
    print(format_report(result))
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    dev = sub.add_parser("dev", help="item 6 on the development sets")
    dev.add_argument("--labels", type=Path, required=True)
    dev.add_argument("--bags-root", type=Path, required=True)
    dev.add_argument("--run-full", type=Path, required=True)
    dev.add_argument("--run-f1", type=Path, required=True)
    dev.add_argument("--run-f2", type=Path, required=True)
    dev.add_argument("--fuse-config", type=Path, required=True)
    dev.add_argument("--out", type=Path, default=Path("gp1d_dev.json"))
    prep = sub.add_parser("prepare", help="windowed cache and self-labels of the held-out sets")
    prep.add_argument("--bags-root", type=Path, required=True)
    prep.add_argument("--out14", type=Path, required=True)
    prep.add_argument("--out22", type=Path, required=True)
    prep.add_argument("--session-dir", type=Path, default=heldout22.SESSION_DIR)
    prep.add_argument("--frozen-sha", required=True)
    prep.add_argument(
        "--workers", type=int, default=max(1, min(4, (multiprocessing.cpu_count() or 2) - 2))
    )
    score = sub.add_parser("score", help="the one pass on the two held-out sets")
    score.add_argument("--labels14", type=Path, required=True)
    score.add_argument("--labels22", type=Path, required=True)
    score.add_argument("--bags-root", type=Path, required=True)
    score.add_argument("--session-dir", type=Path, default=heldout22.SESSION_DIR)
    score.add_argument("--bundle", type=Path, required=True)
    score.add_argument("--frozen-sha", required=True)
    score.add_argument("--dev-json", type=Path, default=None)
    score.add_argument("--latency-json", type=Path, default=None)
    score.add_argument("--canary-json", type=Path, default=None)
    score.add_argument("--out", type=Path, default=Path("gp1d.json"))
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    return {"dev": dev_main, "prepare": prepare_main, "score": score_main}[args.command](args)


if __name__ == "__main__":
    sys.exit(main())
