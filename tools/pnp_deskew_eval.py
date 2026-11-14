#!/usr/bin/env python3
"""M-DSK, the scan deskew measurement (docs/notes/preregistration-pnp-v0.3.md section 4).

    uv run --project tools python tools/pnp_deskew_eval.py step-a --out data/pnp_deskew
    uv run --project tools python tools/pnp_deskew_eval.py step-b --out data/pnp_deskew \\
        --stamp-is sweep_start --beam-order index_descending

Step A (selection, `train` role): the pairs R2 against R1 and R7a against R0 of the split file,
raw and each of the four timing hypotheses (stamp_is x beam_order), metric C on the fast corner
scans of the fast run. The hypothesis with the lowest pooled C is chosen. Run it, commit the
table, THEN run step B.

Step B (result, `heldout` role, once): R8 against R3 and R7b against R4, raw against the chosen
hypothesis, items 1 to 5 of the pre-registration. See tools/pnp_deskew/stepb.py.

Everything reads the frozen split and constants (evaluation/prereg/pnp-v0.2-split.yaml,
evaluation/prereg/pnp-v0.3.yaml); nothing here writes them.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pnp_deskew import bagio, metric, replay
from yaw_rate import matching, run, vehicle

log = logging.getLogger("pnp_deskew_eval")
REPO_ROOT = Path(__file__).resolve().parents[1]
BAGS_ROOT = Path("/Users/cameronjim/code/car-data/bags")
CACHE_DIR = Path("/Users/cameronjim/code/car-data/pnp/cache")
OUT_DIR = Path("/Users/cameronjim/code/car-data/pnp/deskew")
SPLIT = REPO_ROOT / "evaluation" / "prereg" / "pnp-v0.2-split.yaml"
PREREG = REPO_ROOT / "evaluation" / "prereg" / "pnp-v0.3.yaml"
SPLIT_SHA256 = "bc5185c912ece67a6177581f1b37adaaa9101b34ee1efe9881af7723681aef83"


def load_config() -> dict[str, Any]:
    import hashlib

    got = hashlib.sha256(SPLIT.read_bytes()).hexdigest()
    if got != SPLIT_SHA256:
        raise SystemExit(f"split file sha256 {got} is not the frozen {SPLIT_SHA256}")
    return yaml.safe_load(PREREG.read_text())["deskew"]


def recording(bag: str) -> dict[str, Any]:
    return bagio.load_recording(BAGS_ROOT / bag, CACHE_DIR / f"deskew_{bag}.npz")


def segment_index(rec: dict[str, Any], spec: run.SegmentSpec) -> np.ndarray:
    t = rec["scan_t"]
    return np.nonzero((t >= spec.t_from) & (t <= spec.t_to))[0].astype(np.int64)


def condition_names() -> list[str]:
    return ["raw"] + [f"{s}/{o}" for s, o in replay.HYPOTHESES]


def process(rec: dict[str, Any], idx: np.ndarray, condition: str) -> replay.Deskewed:
    if condition == "raw":
        return replay.raw_segment(rec, idx, bagio.scan_ranges)
    stamp_is, order = condition.split("/")
    return replay.deskew_segment(rec, idx, replay.build_relay(stamp_is, order), bagio.scan_ranges)


def pair_scores(
    rec: dict[str, Any],
    fast: run.SegmentSpec,
    slow: run.SegmentSpec,
    condition: str,
    which: str,
    fast_corner: dict[str, float],
    ignore: Any,
    veh: vehicle.Vehicle,
    workers: int,
) -> dict[str, Any]:
    """RMS of the best slow match for the fast run's scans of the kind `which` ("corner" or
    "other"), both runs processed with `condition`."""
    f_idx, s_idx = segment_index(rec, fast), segment_index(rec, slow)
    v, rate = replay.wheel_yaw_rate_at_scans(rec, f_idx)
    corner = (v >= fast_corner["min_wheel_speed_mps"]) & (
        np.abs(rate) >= fast_corner["min_abs_yaw_rate_radps"]
    )
    pick = corner if which == "corner" else ~corner
    f_idx = f_idx[pick]
    fast_d = process(rec, f_idx, condition)
    slow_d = process(rec, s_idx, condition)
    mount = matching.mount_of(veh)

    def meta(i: int) -> tuple[float, float, float, float]:
        return (
            float(rec["scan_angle_min"][i]),
            float(rec["scan_angle_increment"][i]),
            float(rec["scan_range_min"][i]),
            float(rec["scan_range_max"][i]),
        )

    fast_pts = [
        metric.points_of(fast_d.ranges[k], meta(int(i)), mount, ignore) for k, i in enumerate(f_idx)
    ]
    slow_pts = [
        metric.points_of(slow_d.ranges[k], meta(int(i)), mount, ignore) for k, i in enumerate(s_idx)
    ]
    rms, converged = metric.best_match_rms(fast_pts, slow_pts, workers)
    return {
        "fast": fast.name,
        "slow": slow.name,
        "condition": condition,
        "which": which,
        "scan_index": f_idx,
        "t_s": rec["scan_t"][f_idx],
        "rms_m": rms,
        "converged": converged,
        "passed_through": sum(1 for r in fast_d.reason if r not in ("", "raw")),
        "slow_passed_through": sum(1 for r in slow_d.reason if r not in ("", "raw")),
        "scans_fast_run": len(segment_index(rec, fast)),
    }


def step_a(args: argparse.Namespace) -> int:
    cfg = load_config()
    split = yaml.safe_load(SPLIT.read_text())
    names = tuple(
        {p["fast"] for p in cfg["selection_pairs"]} | {p["slow"] for p in cfg["selection_pairs"]}
    )
    specs = {s.name: s for s in run.split_segments(split, names)}
    veh = vehicle.load_vehicle()
    bags = {specs[n].bag for n in names}
    if len(bags) != 1:
        raise SystemExit("selection pairs span more than one bag; this tool expects one")
    rec = recording(bags.pop())
    ignore = matching.self_returns(rec, veh)
    table: dict[str, Any] = {"pairs": {}, "pooled": {}}
    pooled: dict[str, list[np.ndarray]] = {c: [] for c in condition_names()}
    for pair in cfg["selection_pairs"]:
        fast, slow = specs[pair["fast"]], specs[pair["slow"]]
        for cond in condition_names():
            log.info("step A %s vs %s, %s", fast.name, slow.name, cond)
            res = pair_scores(
                rec, fast, slow, cond, "corner", cfg["fast_corner"], ignore, veh, args.workers
            )
            pooled[cond].append(res["rms_m"])
            table["pairs"].setdefault(f"{fast.name}/{slow.name}", {})[cond] = {
                "fast_corner_scans": len(res["rms_m"]),
                "scored": int(np.isfinite(res["rms_m"]).sum()),
                "C_m": metric.median_ignoring_nan(res["rms_m"]),
                "passed_through": res["passed_through"],
                "slow_passed_through": res["slow_passed_through"],
            }
    for cond, parts in pooled.items():
        allv = np.concatenate(parts)
        table["pooled"][cond] = {
            "scored": int(np.isfinite(allv).sum()),
            "C_m": metric.median_ignoring_nan(allv),
        }
    hyp = {c: table["pooled"][c]["C_m"] for c in condition_names()[1:]}
    chosen = min(hyp, key=lambda c: hyp[c])
    table["chosen"] = chosen
    table["chosen_stamp_is"], table["chosen_beam_order"] = chosen.split("/")
    ties = sorted(c for c, x in hyp.items() if np.isclose(x, hyp[chosen], rtol=0, atol=1e-12))
    table["tied_with_chosen"] = ties
    table["raw_C_m"] = table["pooled"]["raw"]["C_m"]
    table["split_sha256"] = SPLIT_SHA256
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "step_a.json").write_text(json.dumps(table, indent=2))
    print(markdown_step_a(table))
    return 0


def markdown_step_a(t: dict[str, Any]) -> str:
    lines = [
        "| condition | "
        + " | ".join(f"{p} C (m), scored" for p in t["pairs"])
        + " | pooled C (m) |",
        "|---|" + "---|" * (len(t["pairs"]) + 1),
    ]
    for cond in t["pooled"]:
        cells = [
            f"{t['pairs'][p][cond]['C_m']:.5f}, {t['pairs'][p][cond]['scored']}/"
            f"{t['pairs'][p][cond]['fast_corner_scans']}"
            for p in t["pairs"]
        ]
        lines.append(f"| {cond} | " + " | ".join(cells) + f" | {t['pooled'][cond]['C_m']:.5f} |")
    lines.append(f"\nchosen: {t['chosen']} (tied with: {', '.join(t['tied_with_chosen'])})")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("step-a")
    a.add_argument("--out", type=Path, default=OUT_DIR)
    a.add_argument("--workers", type=int, default=12)
    b = sub.add_parser("step-b")
    b.add_argument("--out", type=Path, default=OUT_DIR)
    b.add_argument("--workers", type=int, default=12)
    b.add_argument("--stamp-is", required=True, choices=("sweep_start", "sweep_end"))
    b.add_argument("--beam-order", required=True, choices=("index_ascending", "index_descending"))
    b.add_argument("--dynamics-bundle", type=Path)
    b.add_argument("--cli", help="planner_replay_cli (default: $PLANNER_REPLAY_CLI, colcon)")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    if args.cmd == "step-a":
        return step_a(args)
    from pnp_deskew import stepb

    return stepb.run(args, sys.modules[__name__])


if __name__ == "__main__":
    sys.exit(main())
