"""GP1: the track perception net against single-scan `geometric-v1`, decided as written in
docs/notes/preregistration-pnp-v0.2.md section 2 on the held-out sets of the split.

    uv run --group bags python -m racer_pnp_perception.gp1 \\
        --split /Users/cameronjim/code/car/evaluation/prereg/pnp-v0.2-split.yaml \\
        --labels /Users/cameronjim/code/car-data/pnp/selflabels-v0.2 \\
        --bundle <bundle dir> --bags <bag dir> [<bag dir> ...] --out gp1.json

Both backends run through `TrackPerceiver.process`, the code the node runs, on the same scans.

Sets (pre-registration): H-run = labelled scans of the `heldout` segments, H-layout = labelled
scans of the `heldout_layout` bags, A-run and A-layout = ALL moving scans (wheel speed above
0.2 m/s) of the same segments, labelled or not. A scan belongs to a set by its own time; a label
by the window of its submap (selflabel --split already keeps only labels inside one segment).

Numbers: E_c, E_far, V, C95 on H; mean intrusion, boundary residual median, clearance deficit p90
and C95 on A (C95 on A uses the car's own next 2.0 m of ICP path, needed to lie inside the same
segment; a scan without such a path is not counted); the paired block bootstrap of E_c(net) /
E_c(geometric); the per-cell table; the six items and the named outcome. The thresholds below are
the pre-registered ones and are not to be tuned.

Run it ONCE per candidate bundle. Wheel speed and direction: speed is the latest /odom/wheel
sample at or before the scan; direction is the segment's `direction` in the split. The
cross-layout bags have no direction in the split, so their cells are by speed only.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
from racer_track_perception.core import BACKEND_GEOMETRIC, BACKEND_NET, TrackPerceiver
from racer_track_perception.preprocess import scan_to_points
from racer_track_perception.vehicle import LiveVehicle, load_live_vehicle

from racer_pnp_perception import selflabel as sl
from racer_pnp_perception.eval_bags import build_perceivers, clearance_deficit
from racer_pnp_perception.scoring import scan_centreline_errors
from racer_pnp_perception.split import (
    ROLE_HELDOUT,
    ROLE_HELDOUT_LAYOUT,
    SPLIT_SHA256,
    Split,
    load_split,
)

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------------------------
# Pre-registered thresholds (docs/notes/preregistration-pnp-v0.2.md, section 2)
# ---------------------------------------------------------------------------------------------

# "H-run holds at least 300 labelled scans. Fewer: GP1 is undecided, the geometric tracer stays."
MIN_H_RUN_LABELS = 300
# "E_c(net) at most 0.90 times E_c(geometric) on H-run, and the upper end of the bootstrap
# interval of the ratio is below 1.00."
ITEM2_MAX_RATIO = 0.90
ITEM2_CI_UPPER_BELOW = 1.00
# "V(net) at least V(geometric) on H-run."
ITEM3_V_MARGIN = 0.0
# "on H-layout, E_c(net) at most 1.00 times E_c(geometric) and V(net) at least V(geometric) minus
# 0.5. If H-layout holds fewer than 100 labelled scans this item is judged on the label-free
# numbers of A-layout alone (item 5) and that is said."
ITEM4_MAX_RATIO = 1.00
ITEM4_V_MARGIN = 0.5
MIN_H_LAYOUT_LABELS = 100
# "mean intrusion at most max(0.02, geometric + 0.005); boundary residual median at most 0.10 m;
# clearance deficit p90 at most 0.05 m; C95(net) at least C95(geometric) minus 0.02."
ITEM5_INTRUSION_FLOOR = 0.02
ITEM5_INTRUSION_MARGIN = 0.005
ITEM5_MAX_RESIDUAL_MEDIAN_M = 0.10
ITEM5_MAX_DEFICIT_P90_M = 0.05
ITEM5_C95_MARGIN = 0.02
# "Per cell (direction by speed bin [0.2, 0.6), [0.6, 0.85), 0.85 and up, by wheel speed) with at
# least 40 labelled scans: E_c(net) at most 1.10 times E_c(geometric)."
CELL_MIN_LABELS = 40
CELL_MAX_RATIO = 1.10
SPEED_BIN_EDGES_MPS = (0.2, 0.6, 0.85)
# Wheel speed of a moving scan, and the containment share that counts as contained (C95).
MOVING_MPS = 0.2
C95_SHARE = 0.95
# "paired block bootstrap over 2.0 s blocks of consecutive scans within a segment, 2000
# resamples, seed 0, 95 percent percentile interval"
BLOCK_S = 2.0
BOOTSTRAP_RESAMPLES = 2000
BOOTSTRAP_SEED = 0
BOOTSTRAP_ALPHA = 0.05

OUTCOME_PASS = (
    "pass: the net is the candidate backend for the next car session (shadow first, latency "
    "budget on the Orin)"
)
OUTCOME_UNDECIDED = "undecided: fewer than 300 labelled scans in H-run, geometric stays"
OUTCOME_NO_BETTER = "no better than geometric in the layout (item 2 or 3 failed), geometric stays"
OUTCOME_NOT_ELSEWHERE = (
    "better where it trained, not shown elsewhere (items 2 and 3 passed, item 4, 5 or 6 "
    "failed), geometric stays deployed for unseen layouts"
)


# ---------------------------------------------------------------------------------------------
# Pure pieces
# ---------------------------------------------------------------------------------------------


def speed_bin(speed_mps: float) -> str:
    """The pre-registered wheel speed bin of a moving scan."""
    lo, mid, hi = SPEED_BIN_EDGES_MPS
    if speed_mps < mid:
        return f"[{lo}, {mid})"
    if speed_mps < hi:
        return f"[{mid}, {hi})"
    return f"{hi}+"


def speed_at(
    scan_t: NDArray[np.float64], speed_t: NDArray[np.float64], speed: NDArray[np.float64]
) -> NDArray[np.float64]:
    """|wheel speed| of the latest sample at or before each scan stamp (0 before the first)."""
    order = np.argsort(speed_t)
    st, sv = np.asarray(speed_t)[order], np.asarray(speed)[order]
    if len(st) == 0:
        return np.zeros(len(scan_t))
    idx = np.searchsorted(st, scan_t, side="right") - 1
    return np.where(idx >= 0, np.abs(sv[np.clip(idx, 0, None)]), 0.0)


def nanmean(values: Sequence[float] | NDArray[np.float64]) -> float:
    v = np.asarray(values, dtype=np.float64)
    v = v[np.isfinite(v)]
    return float(v.mean()) if len(v) else math.nan


def block_bootstrap_ratio(
    err_net: NDArray[np.float64],
    err_geo: NDArray[np.float64],
    block: NDArray[np.int64],
    n_resamples: int = BOOTSTRAP_RESAMPLES,
    seed: int = BOOTSTRAP_SEED,
    alpha: float = BOOTSTRAP_ALPHA,
) -> dict[str, float]:
    """Paired block bootstrap of E_c(net) / E_c(geometric).

    `err_*` are the per-scan errors (NaN where a backend has no station in common with the label,
    such a scan adds nothing to that backend's mean), `block` the block id of every scan (the same
    block for a run of consecutive scans of one segment). Whole blocks are resampled with
    replacement, the same blocks for both backends; E_c of each backend over a resample is the mean
    of its defined scans, and the ratio is taken per resample. Returns the point ratio and the
    percentile interval."""
    err_net = np.asarray(err_net, dtype=np.float64)
    err_geo = np.asarray(err_geo, dtype=np.float64)
    ids, inverse = np.unique(np.asarray(block), return_inverse=True)
    n_blocks = len(ids)
    out = {"ratio": math.nan, "lo": math.nan, "hi": math.nan, "blocks": n_blocks}
    if n_blocks == 0:
        return out

    def block_sums(err: NDArray[np.float64]) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        ok = np.isfinite(err)
        s = np.bincount(inverse, weights=np.where(ok, err, 0.0), minlength=n_blocks)
        n = np.bincount(inverse, weights=ok.astype(np.float64), minlength=n_blocks)
        return s, n

    sn, nn = block_sums(err_net)
    sg, ng = block_sums(err_geo)
    with np.errstate(invalid="ignore", divide="ignore"):
        out["ratio"] = float((sn.sum() / nn.sum()) / (sg.sum() / ng.sum()))
        rng = np.random.default_rng(seed)
        draws = rng.integers(0, n_blocks, size=(n_resamples, n_blocks))
        net = sn[draws].sum(axis=1) / nn[draws].sum(axis=1)
        geo = sg[draws].sum(axis=1) / ng[draws].sum(axis=1)
        ratio = net / geo
    ratio = ratio[np.isfinite(ratio)]
    if len(ratio):
        out["lo"] = float(np.percentile(ratio, 100.0 * alpha / 2.0))
        out["hi"] = float(np.percentile(ratio, 100.0 * (1.0 - alpha / 2.0)))
    return out


@dataclass
class LabelFree:
    """The label-free numbers of one backend over a set of moving scans."""

    scans: int
    intrusion_mean: float
    residual_median_m: float
    deficit_p90_m: float
    c95: float
    c95_scans: int


@dataclass
class Labelled:
    """The labelled-scan numbers of one backend over a set."""

    scans: int
    e_c_m: float
    e_far_m: float
    v: float
    c95: float


@dataclass
class Cell:
    set: str
    direction: str
    speed_bin: str
    scans: int
    e_c_net_m: float
    e_c_geo_m: float

    @property
    def judged(self) -> bool:
        return self.scans >= CELL_MIN_LABELS

    @property
    def ratio(self) -> float:
        return self.e_c_net_m / self.e_c_geo_m if self.e_c_geo_m > 0 else math.nan


@dataclass
class Numbers:
    """Everything the decision rule reads."""

    h_run: dict[str, Labelled]  # backend -> numbers
    h_layout: dict[str, Labelled]
    a_run: dict[str, LabelFree]
    a_layout: dict[str, LabelFree]
    ratio_ci_hi: float  # upper end of the bootstrap interval of E_c(net) / E_c(geometric), H-run
    cells: list[Cell] = field(default_factory=list)


@dataclass
class Item:
    number: int
    name: str
    value: dict[str, Any]
    threshold: dict[str, Any]
    passed: bool
    note: str = ""


def _le(a: float, b: float) -> bool:
    return bool(a <= b)  # NaN fails


def _ge(a: float, b: float) -> bool:
    return bool(a >= b)


def label_free_ok(net: LabelFree, geo: LabelFree) -> tuple[bool, dict[str, Any], dict[str, Any]]:
    """The four conditions of item 5 on one A set."""
    intrusion_limit = max(ITEM5_INTRUSION_FLOOR, geo.intrusion_mean + ITEM5_INTRUSION_MARGIN)
    checks = {
        "intrusion": _le(net.intrusion_mean, intrusion_limit),
        "residual_median": _le(net.residual_median_m, ITEM5_MAX_RESIDUAL_MEDIAN_M),
        "deficit_p90": _le(net.deficit_p90_m, ITEM5_MAX_DEFICIT_P90_M),
        "c95": _ge(net.c95, geo.c95 - ITEM5_C95_MARGIN),
    }
    value = {
        "intrusion_mean": net.intrusion_mean,
        "residual_median_m": net.residual_median_m,
        "deficit_p90_m": net.deficit_p90_m,
        "c95": net.c95,
        "geometric_intrusion_mean": geo.intrusion_mean,
        "geometric_c95": geo.c95,
    }
    threshold = {
        "intrusion_max": intrusion_limit,
        "residual_median_max_m": ITEM5_MAX_RESIDUAL_MEDIAN_M,
        "deficit_p90_max_m": ITEM5_MAX_DEFICIT_P90_M,
        "c95_min": geo.c95 - ITEM5_C95_MARGIN,
        "checks": checks,
    }
    return all(checks.values()), value, threshold


def decide(n: Numbers) -> tuple[list[Item], str, list[int]]:
    """The six items, the named outcome and the failed item numbers.

    Outcome: item 1 failed gives "undecided". Otherwise item 2 or 3 failed gives "no better than
    geometric in the layout" (whatever else failed), otherwise item 4, 5 or 6 failed gives
    "better where it trained, not shown elsewhere", otherwise "pass"."""
    net, geo = BACKEND_NET, BACKEND_GEOMETRIC
    items: list[Item] = []
    h = n.h_run
    items.append(
        Item(
            1,
            "H-run has at least 300 labelled scans",
            {"labelled_scans": h[net].scans},
            {"min": MIN_H_RUN_LABELS},
            h[net].scans >= MIN_H_RUN_LABELS,
        )
    )
    ratio = h[net].e_c_m / h[geo].e_c_m if h[geo].e_c_m > 0 else math.nan
    items.append(
        Item(
            2,
            "in-layout accuracy",
            {
                "e_c_net_m": h[net].e_c_m,
                "e_c_geometric_m": h[geo].e_c_m,
                "ratio": ratio,
                "ratio_ci_upper": n.ratio_ci_hi,
            },
            {"ratio_max": ITEM2_MAX_RATIO, "ci_upper_below": ITEM2_CI_UPPER_BELOW},
            _le(ratio, ITEM2_MAX_RATIO) and bool(n.ratio_ci_hi < ITEM2_CI_UPPER_BELOW),
        )
    )
    items.append(
        Item(
            3,
            "in-layout range",
            {"v_net": h[net].v, "v_geometric": h[geo].v},
            {"v_net_min": h[geo].v - ITEM3_V_MARGIN},
            _ge(h[net].v, h[geo].v - ITEM3_V_MARGIN),
        )
    )
    lay = n.h_layout
    a_lay_ok, a_lay_value, a_lay_thr = label_free_ok(n.a_layout[net], n.a_layout[geo])
    if lay[net].scans < MIN_H_LAYOUT_LABELS:
        items.append(
            Item(
                4,
                "cross-layout",
                {"h_layout_labelled_scans": lay[net].scans, "a_layout": a_lay_value},
                {"min_labelled_scans": MIN_H_LAYOUT_LABELS, "a_layout": a_lay_thr},
                a_lay_ok,
                "H-layout has fewer than 100 labelled scans: judged on the label-free numbers of "
                "A-layout alone (item 5)",
            )
        )
    else:
        lay_ratio = lay[net].e_c_m / lay[geo].e_c_m if lay[geo].e_c_m > 0 else math.nan
        items.append(
            Item(
                4,
                "cross-layout",
                {
                    "e_c_net_m": lay[net].e_c_m,
                    "e_c_geometric_m": lay[geo].e_c_m,
                    "ratio": lay_ratio,
                    "v_net": lay[net].v,
                    "v_geometric": lay[geo].v,
                },
                {"ratio_max": ITEM4_MAX_RATIO, "v_net_min": lay[geo].v - ITEM4_V_MARGIN},
                _le(lay_ratio, ITEM4_MAX_RATIO) and _ge(lay[net].v, lay[geo].v - ITEM4_V_MARGIN),
            )
        )
    a_run_ok, a_run_value, a_run_thr = label_free_ok(n.a_run[net], n.a_run[geo])
    items.append(
        Item(
            5,
            "label-free non-inferiority on A-run and A-layout",
            {"a_run": a_run_value, "a_layout": a_lay_value},
            {"a_run": a_run_thr, "a_layout": a_lay_thr},
            a_run_ok and a_lay_ok,
        )
    )
    judged = [c for c in n.cells if c.judged]
    bad = [
        f"{c.set}|{c.direction}|{c.speed_bin}" for c in judged if not _le(c.ratio, CELL_MAX_RATIO)
    ]
    items.append(
        Item(
            6,
            "per cell",
            {
                "cells_judged": len(judged),
                "cells_not_judged": len(n.cells) - len(judged),
                "worst_ratio": max((c.ratio for c in judged), default=math.nan),
                "failing_cells": bad,
            },
            {"ratio_max": CELL_MAX_RATIO, "min_labelled_scans": CELL_MIN_LABELS},
            not bad,
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


# ---------------------------------------------------------------------------------------------
# GP1b (docs/notes/preregistration-pnp-v0.3.md section 3.3). All thresholds are the v0.2 ones
# above, used as they are.
# ---------------------------------------------------------------------------------------------

OUTCOME_GP1B_PASS = (
    "pass: net plus scan refinement is the candidate backend (shadow first, the on-car "
    "confirmation of v0.2 section 2 is the real test)"
)
OUTCOME_GP1B_NO_BETTER = (
    "no better than geometric in the layout (item 1 or 2 failed), geometric stays"
)
OUTCOME_GP1B_NOT_ELSEWHERE = (
    "better where it trained, not shown elsewhere (items 1 and 2 passed, item 3, 4 or 5 failed), "
    "geometric stays deployed"
)
# v0.2 items of H-run that GP1b item 1 repeats.
GP1B_ITEM1_V02_ITEMS = (1, 2, 3, 6)


@dataclass
class FoldResult:
    """One fold model on its own held-out bags: the labelled scans (E_c, V) and all moving scans
    (label-free numbers), by backend."""

    labelled: dict[str, Labelled]
    free: dict[str, LabelFree]


def decide_gp1b(full: Numbers, folds: dict[str, FoldResult]) -> tuple[list[Item], str, list[int]]:
    """The five items of pre-registration v0.3 section 3.3 for the backends `net` (the candidate)
    and `geometric` in the given numbers.

    1. v0.2 items 1, 2, 3 and 6 on H-run for the full model.
    2. v0.2 item 5 on A-run for the full model.
    3. Each fold with at least 100 labelled scans: E_c(net) / E_c(geometric) at most 1.00 and V(net)
       at least V(geometric) minus 0.5. A fold with fewer is not judged on this item (said in the
       note); item 4 still judges it.
    4. Each fold, all moving scans: the label-free rule of v0.2 item 5.
    5. The same label-free rule on the 2026-11-12 layout (`a_layout`) for the full model.

    Outcome: item 1 or 2 failed gives "no better than geometric in the layout"; otherwise 3, 4 or 5
    failed gives "better where it trained, not shown elsewhere"; otherwise pass."""
    net, geo = BACKEND_NET, BACKEND_GEOMETRIC
    v02_items, _outcome, _failed = decide(full)
    sub = [i for i in v02_items if i.number in GP1B_ITEM1_V02_ITEMS]
    items: list[Item] = [
        Item(
            1,
            "v0.2 items 1, 2, 3 and 6 on H-run, full model",
            {f"v0.2 item {i.number}": {"passed": i.passed, "value": i.value} for i in sub},
            {f"v0.2 item {i.number}": i.threshold for i in sub},
            all(i.passed for i in sub),
            "second read of H-run (read once by net v1 in v0.2)",
        )
    ]
    ok, value, thr = label_free_ok(full.a_run[net], full.a_run[geo])
    items.append(Item(2, "label-free rule on A-run, full model", value, thr, ok))
    per_fold: dict[str, Any] = {}
    thr_fold: dict[str, Any] = {}
    all_ok = True
    not_judged: list[str] = []
    for name, f in sorted(folds.items()):
        n_lab = f.labelled[net].scans
        if n_lab < MIN_H_LAYOUT_LABELS:
            not_judged.append(name)
            per_fold[name] = {"labelled_scans": n_lab, "judged": False}
            continue
        ratio = (
            f.labelled[net].e_c_m / f.labelled[geo].e_c_m if f.labelled[geo].e_c_m > 0 else math.nan
        )
        good = _le(ratio, ITEM4_MAX_RATIO) and _ge(
            f.labelled[net].v, f.labelled[geo].v - ITEM4_V_MARGIN
        )
        per_fold[name] = {
            "labelled_scans": n_lab,
            "judged": True,
            "e_c_net_m": f.labelled[net].e_c_m,
            "e_c_geometric_m": f.labelled[geo].e_c_m,
            "ratio": ratio,
            "v_net": f.labelled[net].v,
            "v_geometric": f.labelled[geo].v,
            "passed": good,
        }
        thr_fold[name] = {
            "ratio_max": ITEM4_MAX_RATIO,
            "v_net_min": f.labelled[geo].v - ITEM4_V_MARGIN,
        }
        all_ok = all_ok and good
    items.append(
        Item(
            3,
            "each fold with 100 or more labelled scans: accuracy and range",
            per_fold,
            {"min_labelled_scans": MIN_H_LAYOUT_LABELS, **thr_fold},
            all_ok,
            f"not judged (fewer than {MIN_H_LAYOUT_LABELS} labelled scans): {not_judged}"
            if not_judged
            else "",
        )
    )
    free_vals: dict[str, Any] = {}
    free_thr: dict[str, Any] = {}
    free_ok = True
    for name, f in sorted(folds.items()):
        good, v, t = label_free_ok(f.free[net], f.free[geo])
        free_vals[name] = {**v, "passed": good, "moving_scans": f.free[net].scans}
        free_thr[name] = t
        free_ok = free_ok and good
    items.append(
        Item(4, "each fold, all moving scans: the label-free rule", free_vals, free_thr, free_ok)
    )
    ok, value, thr = label_free_ok(full.a_layout[net], full.a_layout[geo])
    items.append(
        Item(
            5,
            "label-free rule on the 2026-11-12 layout, full model",
            value,
            thr,
            ok,
            "development data, reported and not evidence",
        )
    )
    failed = [i.number for i in items if not i.passed]
    if 1 in failed or 2 in failed:
        outcome = OUTCOME_GP1B_NO_BETTER
    elif failed:
        outcome = OUTCOME_GP1B_NOT_ELSEWHERE
    else:
        outcome = OUTCOME_GP1B_PASS
    return items, outcome, failed


# ---------------------------------------------------------------------------------------------
# Scans
# ---------------------------------------------------------------------------------------------

SET_RUN = "run"
SET_LAYOUT = "layout"
ROLE_TO_SET = {ROLE_HELDOUT: SET_RUN, ROLE_HELDOUT_LAYOUT: SET_LAYOUT}


@dataclass
class ScanRow:
    """One moving scan of a held-out segment, with both backends' tracks scored."""

    bag: str
    index: int
    set: str
    segment: str
    segment_from_s: float
    t_s: float
    speed_mps: float
    direction: str
    path: NDArray[np.float64] | None  # the car's own next 2.0 m inside the segment, else None
    label: NDArray[np.float32] | None
    label_valid: int
    e_c: dict[str, float] = field(default_factory=dict)
    e_far: dict[str, float] = field(default_factory=dict)
    valid: dict[str, int] = field(default_factory=dict)
    intrusion: dict[str, float] = field(default_factory=dict)
    residual: dict[str, float] = field(default_factory=dict)
    deficit: dict[str, NDArray[np.float64]] = field(default_factory=dict)
    containment: dict[str, float] = field(default_factory=dict)


def in_segment_path(
    chain: sl.Chain, stamps_bag: NDArray[np.float64], i: int, split: Split, bag: str
) -> NDArray[np.float64] | None:
    """The car's own next 2.0 m of path from scan i, or None when the chain breaks, the car does
    not travel that far, or the scans the path is built from are not all inside the segment of
    scan i."""
    n = len(chain.travel_m)
    j = int(np.searchsorted(chain.travel_m, chain.travel_m[i] + sl.PATH_LENGTH_M, side="left"))
    if j >= n:
        return None
    if split.locate(bag, float(stamps_bag[i]), float(stamps_bag[j])) is None:
        return None
    return sl.driven_path(chain, i, j)


def collect_bag(
    bag_dir: Path,
    labels_dir: Path,
    split: Split,
    perceivers: dict[str, TrackPerceiver],
    live: LiveVehicle,
    role_to_set: dict[str, str] | None = None,
) -> list[ScanRow]:
    """Every moving scan inside a held-out segment (or held-out-layout bag), scored. With
    `role_to_set` the roles that count and the set each goes to are given (GP1b scores a fold's
    whole `train` bag as its own set)."""
    role_to_set = role_to_set or ROLE_TO_SET
    name = bag_dir.name
    entry = split.bags.get(name)
    if entry is None:
        return []
    scans, speed_t, speed = sl.load_scanset(bag_dir, labels_dir / "cache")
    bag_t = scans.bag_time_s()
    speeds = speed_at(scans.stamp_s, speed_t, speed)
    moving = sl.moving_scans(scans, speed_t, speed, MOVING_MPS)
    motion_path = labels_dir / "cache" / f"{name}.motion.npz"
    chain = sl.chain_motion(sl.load_motion(motion_path)) if motion_path.is_file() else None
    labelled: dict[int, tuple[NDArray[np.float32], int, NDArray[np.float64]]] = {}
    label_file = labels_dir / f"{name}.npz"
    if label_file.is_file():
        with np.load(label_file) as z:
            for k, i in enumerate(z["index"]):
                role = split.role_of(
                    name, float(z["window_first_s"][k]), float(z["window_last_s"][k])
                )
                if role != str(z["role"][k]):
                    raise ValueError(f"{name}: label {k} role disagrees with the split")
                if role in role_to_set:
                    labelled[int(i)] = (z["label"][k], int(z["valid_count"][k]), z["path_xy"][k])
    rows: list[ScanRow] = []
    for i in np.flatnonzero(moving):
        t = float(bag_t[i])
        seg = split.locate(name, t, t)
        if seg is None or seg.role not in role_to_set:
            continue
        lab = labelled.get(int(i))
        if lab is not None:
            path: NDArray[np.float64] | None = lab[2]
        elif chain is not None:
            path = in_segment_path(chain, bag_t, int(i), split, name)
        else:
            path = None
        row = ScanRow(
            bag=name,
            index=int(i),
            set=role_to_set[seg.role],
            segment=seg.name,
            segment_from_s=seg.t_from_s if math.isfinite(seg.t_from_s) else 0.0,
            t_s=t,
            speed_mps=float(speeds[i]),
            direction=seg.direction or "unknown",
            path=path,
            label=lab[0] if lab else None,
            label_valid=lab[1] if lab else 0,
        )
        score_row(row, scans, int(i), perceivers, live)
        rows.append(row)
    log.info("%s: %d moving scans in held-out segments", name, len(rows))
    return rows


def score_row(
    row: ScanRow,
    scans: sl.ScanSet,
    i: int,
    perceivers: dict[str, TrackPerceiver],
    live: LiveVehicle,
) -> None:
    """Run every backend on scan `i` through `TrackPerceiver.process` and fill the row's numbers:
    valid_count, the self-checks, the clearance deficit, the path containment when the row has a
    driven path, and the centreline errors when it has a label."""
    pts = scan_to_points(
        scans.ranges[i],
        scans.angle_min,
        scans.angle_increment,
        scans.range_min,
        scans.range_max,
        live.mount,
        live.body,
    )
    for backend, perceiver in perceivers.items():
        track = perceiver.process(
            scans.ranges[i],
            scans.angle_min,
            scans.angle_increment,
            scans.range_min,
            scans.range_max,
        )
        row.valid[backend] = track.valid_count
        row.intrusion[backend] = track.intrusion_fraction
        row.residual[backend] = track.boundary_residual_m
        row.deficit[backend] = clearance_deficit(track, pts)
        if row.path is not None:
            c = sl.path_containment(track, row.path, live.body.half_width_m)
            row.containment[backend] = 0.0 if not math.isfinite(c) else c
        if row.label is not None:
            ec, ef, _ = scan_centreline_errors(track, row.label, row.label_valid)
            row.e_c[backend], row.e_far[backend] = ec, ef


# ---------------------------------------------------------------------------------------------
# Tables
# ---------------------------------------------------------------------------------------------


def labelled_numbers(rows: list[ScanRow], backend: str) -> Labelled:
    lab = [r for r in rows if r.label is not None]
    contained = [r.containment[backend] >= C95_SHARE for r in lab if backend in r.containment]
    return Labelled(
        scans=len(lab),
        e_c_m=nanmean([r.e_c[backend] for r in lab]),
        e_far_m=nanmean([r.e_far[backend] for r in lab]),
        v=float(np.mean([r.valid[backend] for r in lab])) if lab else math.nan,
        c95=float(np.mean(contained)) if contained else math.nan,
    )


def label_free_numbers(rows: list[ScanRow], backend: str) -> LabelFree:
    deficits = [r.deficit[backend] for r in rows]
    cat = np.concatenate([np.zeros(0), *deficits]) if deficits else np.zeros(0)
    resid = np.array([r.residual[backend] for r in rows], dtype=np.float64)
    resid = resid[np.isfinite(resid)]
    contained = [r.containment[backend] >= C95_SHARE for r in rows if backend in r.containment]
    return LabelFree(
        scans=len(rows),
        intrusion_mean=nanmean([r.intrusion[backend] for r in rows]),
        residual_median_m=float(np.median(resid)) if len(resid) else math.nan,
        deficit_p90_m=float(np.percentile(cat, 90)) if len(cat) else math.nan,
        c95=float(np.mean(contained)) if contained else math.nan,
        c95_scans=len(contained),
    )


def cell_table(rows: list[ScanRow]) -> list[Cell]:
    """Labelled scans by set, direction (from the split) and wheel speed bin."""
    groups: dict[tuple[str, str, str], list[ScanRow]] = {}
    for r in rows:
        if r.label is None:
            continue
        direction = r.direction if r.set == SET_RUN else "unknown"
        groups.setdefault((r.set, direction, speed_bin(r.speed_mps)), []).append(r)
    cells = []
    for (s, d, b), rs in sorted(groups.items()):
        cells.append(
            Cell(
                s,
                d,
                b,
                len(rs),
                nanmean([r.e_c[BACKEND_NET] for r in rs]),
                nanmean([r.e_c[BACKEND_GEOMETRIC] for r in rs]),
            )
        )
    return cells


def block_ids(rows: list[ScanRow]) -> NDArray[np.int64]:
    """A block id per row: 2.0 s blocks of bag time counted from the start of its segment, one
    id per (bag, segment, block)."""
    keys: dict[tuple[str, str, int], int] = {}
    out = np.zeros(len(rows), dtype=np.int64)
    for k, r in enumerate(rows):
        key = (r.bag, r.segment, math.floor((r.t_s - r.segment_from_s) / BLOCK_S))
        out[k] = keys.setdefault(key, len(keys))
    return out


def _clean(obj: Any) -> Any:
    if isinstance(obj, dict):
        return {k: _clean(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_clean(v) for v in obj]
    if isinstance(obj, (float, np.floating)):
        return None if not math.isfinite(float(obj)) else float(obj)
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, (np.bool_,)):
        return bool(obj)
    return obj


def summarise(rows: list[ScanRow], split: Split, bundle_id: str | None) -> dict[str, Any]:
    """The whole gp1.json from scored rows."""
    backends = (BACKEND_NET, BACKEND_GEOMETRIC)
    run = [r for r in rows if r.set == SET_RUN]
    layout = [r for r in rows if r.set == SET_LAYOUT]
    h_run = [r for r in run if r.label is not None]
    cells = cell_table(rows)
    boot = block_bootstrap_ratio(
        np.array([r.e_c[BACKEND_NET] for r in h_run]),
        np.array([r.e_c[BACKEND_GEOMETRIC] for r in h_run]),
        block_ids(h_run),
    )
    numbers = Numbers(
        h_run={b: labelled_numbers(run, b) for b in backends},
        h_layout={b: labelled_numbers(layout, b) for b in backends},
        a_run={b: label_free_numbers(run, b) for b in backends},
        a_layout={b: label_free_numbers(layout, b) for b in backends},
        ratio_ci_hi=boot["hi"],
        cells=cells,
    )
    items, outcome, failed = decide(numbers)

    def dump(d: dict[str, Any]) -> dict[str, Any]:
        return {k: vars(v) for k, v in d.items()}

    per_segment: dict[str, Any] = {}
    for r in rows:
        s = per_segment.setdefault(
            f"{r.bag}/{r.segment}", {"set": r.set, "moving_scans": 0, "labelled_scans": 0}
        )
        s["moving_scans"] += 1
        s["labelled_scans"] += int(r.label is not None)
    return _clean(
        {
            "split_sha256": split.sha256,
            "split_sha256_expected": SPLIT_SHA256,
            "bundle": bundle_id,
            "sets": {
                "H-run": {"labelled_scans": len(h_run)},
                "H-layout": {"labelled_scans": sum(r.label is not None for r in layout)},
                "A-run": {"moving_scans": len(run)},
                "A-layout": {"moving_scans": len(layout)},
            },
            "per_segment": per_segment,
            "H-run": dump(numbers.h_run),
            "H-layout": dump(numbers.h_layout),
            "A-run": dump(numbers.a_run),
            "A-layout": dump(numbers.a_layout),
            "e_c_scans_without_common_station": {
                "H-run": {
                    b: sum(1 for r in h_run if not math.isfinite(r.e_c[b])) for b in backends
                },
            },
            "bootstrap_e_c_ratio_net_over_geometric_h_run": {
                **boot,
                "block_s": BLOCK_S,
                "resamples": BOOTSTRAP_RESAMPLES,
                "seed": BOOTSTRAP_SEED,
            },
            "cells": [{**vars(c), "judged": c.judged, "ratio": c.ratio} for c in numbers.cells],
            "items": [vars(i) for i in items],
            "failed_items": failed,
            "outcome": outcome,
        }
    )


def format_report(result: dict[str, Any]) -> str:
    """The GP1 table as plain text (the first line is the outcome)."""
    lines = [f"GP1 outcome: {result['outcome']}", ""]
    for name in ("H-run", "H-layout"):
        lines.append(f"{name} ({result['sets'][name]['labelled_scans']} labelled scans)")
        for b, v in result[name].items():
            lines.append(
                f"  {b:9s} E_c {_f(v['e_c_m'])} m  E_far {_f(v['e_far_m'])} m  V {_f(v['v'], 2)}"
                f"  C95 {_f(v['c95'])}"
            )
    for name in ("A-run", "A-layout"):
        lines.append(f"{name} ({result['sets'][name]['moving_scans']} moving scans)")
        for b, v in result[name].items():
            lines.append(
                f"  {b:9s} intrusion {_f(v['intrusion_mean'], 4)}  residual median "
                f"{_f(v['residual_median_m'])} m  deficit p90 {_f(v['deficit_p90_m'])} m  "
                f"C95 {_f(v['c95'])} ({v['c95_scans']} scans)"
            )
    boot = result["bootstrap_e_c_ratio_net_over_geometric_h_run"]
    lines.append(
        f"bootstrap E_c ratio net/geometric (H-run): {_f(boot['ratio'])} "
        f"[{_f(boot['lo'])}, {_f(boot['hi'])}] over {boot['blocks']} blocks"
    )
    lines.append("cells (set, direction, speed bin: labelled scans, ratio)")
    for c in result["cells"]:
        lines.append(
            f"  {c['set']:6s} {c['direction']:8s} {c['speed_bin']:11s} {c['scans']:5d} "
            f"{_f(c['ratio'], 3)}{'' if c['judged'] else '  (under 40, not judged)'}"
        )
    lines.append("items")
    for i in result["items"]:
        lines.append(f"  {i['number']}. {i['name']}: {'PASS' if i['passed'] else 'FAIL'}")
        lines.append(f"     value {json.dumps(i['value'])}")
        lines.append(f"     threshold {json.dumps(i['threshold'])}")
        if i["note"]:
            lines.append(f"     note: {i['note']}")
    return "\n".join(lines)


def _f(x: float | None, digits: int = 3) -> str:
    return "n/a" if x is None else f"{x:.{digits}f}"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--split", type=Path, required=True)
    parser.add_argument("--labels", type=Path, required=True)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--bags", nargs="+", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path("gp1.json"))
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    split = load_split(args.split)
    live = load_live_vehicle()
    perceivers = build_perceivers(live, args.bundle)
    rows: list[ScanRow] = []
    for bag in args.bags:
        rows += collect_bag(bag, args.labels, split, perceivers, live)
    result = summarise(rows, split, perceivers[BACKEND_NET].model_id)
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False))
    print(format_report(result))
    return 0


if __name__ == "__main__":
    sys.exit(main())
