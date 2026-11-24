"""Development tuning of `fused-v2` (brief 55, yaml `tuning` block): free settings chosen on
development sets only, before the freeze. No held-out scan is read here (`gp1d.assert_bag_allowed`).

    uv run --group train --group bags python -m racer_pnp_perception.tune_v2 build \\
        --labels /Users/cameronjim/code/car-data/pnp/selflabels-v0.2 \\
        --bags-root /Users/cameronjim/code/car-data/bags
    uv run --group bags python -m racer_pnp_perception.tune_v2 sweep --out tune_table.json
    uv run --group bags python -m racer_pnp_perception.tune_v2 verify --table tune_table.json \\
        --out tune_result.json

`build` runs the tracer and the refined net once per development scan and stores the two tracks (and
the label, where there is one) per set under car-data/pnp/cache/fused-v2/. `sweep` and `verify` then
fuse and measure at every setting without touching a scan again, with the registered measures:
`corridor.compare` (loss and excess against the tracer) and E_c of `scoring.scan_centreline_errors`.

Development sets (yaml `data_roles.development`): sim; the v0.2 train roles with the full model
(in sample); A-run and A-layout 2026-11-12 with the full model (H-run, H-layout are their labelled
scans); folds F1 and F2 with their fold models; the five 2026-11-16 bags (moving scans, no labels).

The pick rule (yaml): among the settings that meet the development margin on EVERY set, the lowest
mean over the labelled sets of E_c(fused) / E_c(geometric); the setting must also have a ratio of
at most 0.95 on every labelled set. None: "GP1d not attempted", nothing frozen. The sweep is staged
(slope by extension, then reach by margin by blend around the best of stage 1) on an evenly spaced
subsample of each set; `verify` repeats the best candidates on the FULL sets and the rule is applied
to those numbers.
"""

from __future__ import annotations

import argparse
import itertools
import json
import logging
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any

import numpy as np
from racer_track_perception import fuse_v2, geometric, preprocess, refine
from racer_track_perception import track as tr
from racer_track_perception.core import BACKEND_NET, TrackPerceiver, net_output_to_track
from racer_track_perception.runtime import CircConv1dV0
from racer_track_perception.vehicle import load_live_vehicle

from racer_pnp_perception import corridor, gp1, gp1d, slope_select
from racer_pnp_perception.scoring import scan_centreline_errors

log = logging.getLogger(__name__)

CACHE_ROOT = Path("/Users/cameronjim/code/car-data/pnp/cache/fused-v2")
FULL_WEIGHTS = Path("/Users/cameronjim/code/car-data/pnp/models/trackperc-v2-2026-11-14/weights.npz")
RUN_F1 = Path("/Users/cameronjim/code/car-data/pnp/runs-v2/F1/seed0")
RUN_F2 = Path("/Users/cameronjim/code/car-data/pnp/runs-v2/F2/seed1")
SIM_N = 1500
SIM_SEED = 777  # eval_sim's fresh in-distribution seed
SUBSAMPLE = 1200
SLOPE_GRID = (0.05, 0.10, 0.15, 0.20, 0.30, fuse_v2.NO_SLOPE_LIMIT)
EXT_GRID = (0.0, 0.05, 0.10, 0.20)
REACH_GRID = (0.25, 0.35, 0.50)
MARGIN_GRID = (0.10, 0.15, 0.20)
BLEND_GRID = (0.5, 0.75, 1.0)
LABELLED = ("sim", "train-roles", "A-run", "A-layout-2026-11-12", "fold F1", "fold F2")
UNLABELLED = ("dev-2026-11-16",)
ALL_SETS = (*LABELLED, *UNLABELLED)


# ---------------------------------------------------------------------------------------------
# Capture
# ---------------------------------------------------------------------------------------------


def tracks_for_scan(
    p: TrackPerceiver, ranges: Any, angle_min: float, inc: float, rmin: float, rmax: float
) -> tuple[tr.LocalTrackData, tr.LocalTrackData]:
    """The tracer's track and the refined net's track of one scan: the inputs `TrackPerceiver`
    gives the fusion (same calls, same order)."""
    assert p.net is not None
    tensor, points = preprocess.scan_to_tensor_and_points(
        ranges, angle_min, inc, rmin, rmax, p.vehicle.mount, p.vehicle.body
    )
    net = refine.refine_track(
        net_output_to_track(p.net.forward(tensor), p.sigma_max_m), points, p._refine_cfg
    )
    geo = geometric.trace(points, p.vehicle.body.half_width_m, p.sigma_max_m, p._geo_cfg)
    return geo, net


def pack(t: tr.LocalTrackData) -> np.ndarray:
    return np.stack(
        [t.x_m, t.y_m, t.half_width_left_m, t.half_width_right_m, t.sigma_m]
    ).astype(np.float32)


def unpack(a: np.ndarray, n: int) -> tr.LocalTrackData:
    return tr.LocalTrackData(
        x_m=a[0].copy(),
        y_m=a[1].copy(),
        half_width_left_m=a[2].copy(),
        half_width_right_m=a[3].copy(),
        sigma_m=a[4].copy(),
        valid_count=int(n),
    )


class Collector:
    """Rows of one set: the two tracks and the label of every scan."""

    def __init__(self) -> None:
        self.geo: list[np.ndarray] = []
        self.net: list[np.ndarray] = []
        self.geo_n: list[int] = []
        self.net_n: list[int] = []
        self.label: list[np.ndarray] = []
        self.label_valid: list[int] = []
        self.speed: list[float] = []
        self.where: list[str] = []

    def add(
        self,
        geo: tr.LocalTrackData,
        net: tr.LocalTrackData,
        label: np.ndarray | None,
        label_valid: int,
        speed: float,
        where: str,
    ) -> None:
        self.geo.append(pack(geo))
        self.net.append(pack(net))
        self.geo_n.append(geo.valid_count)
        self.net_n.append(net.valid_count)
        lab = np.full((16, 2), np.nan, dtype=np.float32)
        if label is not None:
            lab[: len(label)] = label[:, :2]
        self.label.append(lab)
        self.label_valid.append(int(label_valid) if label is not None else 0)
        self.speed.append(speed)
        self.where.append(where)

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            path,
            geo=np.stack(self.geo),
            net=np.stack(self.net),
            geo_n=np.array(self.geo_n),
            net_n=np.array(self.net_n),
            label=np.stack(self.label),
            label_valid=np.array(self.label_valid),
            speed=np.array(self.speed),
            where=np.array(self.where),
        )


def capture_bag_rows(
    bag_dir: Path,
    labels_dir: Path,
    split: Any,
    role_to_set: dict[str, str],
    p: TrackPerceiver,
    live: Any,
    col: Collector,
) -> int:
    """Every moving scan of the split's roles in one bag into `col`, via `gp1.collect_bag`."""
    gp1d.assert_bag_allowed(bag_dir.name, "development")

    def scorer(row: gp1.ScanRow, scans: Any, i: int, _p: Any, _live: Any) -> None:
        geo, net = tracks_for_scan(
            p,
            scans.ranges[i],
            scans.angle_min,
            scans.angle_increment,
            scans.range_min,
            scans.range_max,
        )
        col.add(
            geo,
            net,
            row.label,
            row.label_valid,
            row.speed_mps,
            f"{row.bag}#{row.index}",
        )

    rows = gp1.collect_bag(bag_dir, labels_dir, split, {}, live, role_to_set, scorer)
    return len(rows)


def build(args: argparse.Namespace) -> int:
    from racer_pnp_perception import dataset
    from racer_pnp_perception.internal_val import net_from_run
    from racer_pnp_perception.raycast import ScanGeometry
    from racer_pnp_perception.split import ROLE_HELDOUT, ROLE_HELDOUT_LAYOUT, ROLE_TRAIN
    from racer_pnp_perception.split import load_folds, load_split, parse_split

    live = load_live_vehicle()
    split = load_split()
    folds = load_folds(split)
    full = CircConv1dV0.from_npz(FULL_WEIGHTS)
    cfg = fuse_v2.FuseV2Config()

    def perceiver(net: CircConv1dV0, mid: str) -> TrackPerceiver:
        return TrackPerceiver(BACKEND_NET, live, net=net, model_id=mid, fuse_cfg=cfg)

    p_full = perceiver(full, "full")
    CACHE_ROOT.mkdir(parents=True, exist_ok=True)

    def run_set(name: str, bags: list[str], roles: dict[str, str], p: TrackPerceiver) -> None:
        out = CACHE_ROOT / f"dev-{name.replace(' ', '_')}.npz"
        if out.is_file() and not args.force:
            log.info("%s: cached", name)
            return
        col = Collector()
        for bag in bags:
            capture_bag_rows(args.bags_root / bag, args.labels, split, roles, p, live, col)
        col.save(out)
        log.info("%s: %d scans, %d labelled", name, len(col.geo), sum(v > 0 for v in col.label_valid))

    heldout_bags = sorted(
        b for b, e in split.bags.items() if any(s.role == ROLE_HELDOUT for s in e.segments)
    )
    layout_bags = sorted(b for b, e in split.bags.items() if e.whole_bag_role == ROLE_HELDOUT_LAYOUT)
    train_bags = sorted(
        b
        for b, e in split.bags.items()
        if e.whole_bag_role == ROLE_TRAIN or any(s.role == ROLE_TRAIN for s in e.segments)
    )
    run_set("A-run", heldout_bags, {ROLE_HELDOUT: gp1.SET_RUN}, p_full)
    run_set("A-layout-2026-11-12", layout_bags, {ROLE_HELDOUT_LAYOUT: gp1.SET_LAYOUT}, p_full)
    run_set("train-roles", train_bags, {ROLE_TRAIN: "train"}, p_full)
    for name, run in (("F1", RUN_F1), ("F2", RUN_F2)):
        run_set(
            f"fold {name}",
            list(folds[name].heldout_bags),
            {ROLE_TRAIN: "fold"},
            perceiver(net_from_run(run), name),
        )
    # the five 2026-11-16 bags: moving scans, whole bags, no labels
    doc = {
        "split_id": "dev-2026-11-16",
        "bags": {
            b: {"units": "current", "whole_bag": {"perception": ROLE_TRAIN}}
            for b in gp1d.DEV_2026_11_16
        },
    }
    import yaml

    split_1116 = parse_split(yaml.safe_dump(doc))
    out = CACHE_ROOT / "dev-dev-2026-11-16.npz"
    if not out.is_file() or args.force:
        labels_1116 = CACHE_ROOT / "labels-2026-11-16"
        (labels_1116 / "cache").mkdir(parents=True, exist_ok=True)
        col = Collector()
        for bag in gp1d.DEV_2026_11_16:
            capture_bag_rows(
                args.bags_root / bag, labels_1116, split_1116, {ROLE_TRAIN: "dev"}, p_full, live, col
            )
        col.save(out)
        log.info("dev-2026-11-16: %d scans", len(col.geo))
    # sim
    out = CACHE_ROOT / "dev-sim.npz"
    if not out.is_file() or args.force:
        data = dataset.generate(SIM_N, SIM_SEED, keep_ranges=True)
        g = ScanGeometry()
        col = Collector()
        for i in range(SIM_N):
            geo, net = tracks_for_scan(
                p_full,
                data["ranges"][i],
                g.angle_min_rad,
                g.angle_increment_rad,
                g.range_min_m,
                g.range_max_m,
            )
            col.add(geo, net, data["label"][i], 16, 0.0, f"sim#{i}")
        col.save(out)
        log.info("sim: %d scans", SIM_N)
    return 0


# ---------------------------------------------------------------------------------------------
# Measure
# ---------------------------------------------------------------------------------------------

_DATA: dict[str, dict[str, np.ndarray]] = {}


def load_set(name: str) -> dict[str, np.ndarray]:
    if name not in _DATA:
        with np.load(CACHE_ROOT / f"dev-{name.replace(' ', '_')}.npz") as z:
            _DATA[name] = {k: z[k] for k in z.files}
    return _DATA[name]


def subsample_index(n: int, cap: int) -> np.ndarray:
    return np.arange(n) if n <= cap else np.linspace(0, n - 1, cap).round().astype(int)


def cfg_from_dict(d: dict[str, Any]) -> fuse_v2.FuseV2Config:
    return fuse_v2.FuseV2Config(**d)


def measure_set(name: str, cfg_d: dict[str, Any], cap: int | None) -> dict[str, Any]:
    """Coverage summary (`corridor.summarise`) and E_c of fused and geometric on one set."""
    z = load_set(name)
    cfg = cfg_from_dict(cfg_d)
    idx = subsample_index(len(z["geo_n"]), cap) if cap else np.arange(len(z["geo_n"]))
    results: list[dict] = []
    ec_f: list[float] = []
    ec_g: list[float] = []
    for i in idx:
        geo = unpack(z["geo"][i], z["geo_n"][i])
        net = unpack(z["net"][i], z["net_n"][i])
        fused = fuse_v2.fuse_track_v2(geo, net, cfg)
        results.append(corridor.compare(geo, fused))
        lv = int(z["label_valid"][i])
        if lv > 0:
            lab = z["label"][i]
            ec_f.append(scan_centreline_errors(fused, lab, lv)[0])
            ec_g.append(scan_centreline_errors(geo, lab, lv)[0])
    out: dict[str, Any] = {"coverage": corridor.summarise(results), "scans": len(idx)}
    if ec_g:
        out["labelled"] = len(ec_g)
        out["e_c_fused_m"] = gp1.nanmean(ec_f)
        out["e_c_geometric_m"] = gp1.nanmean(ec_g)
        out["ratio"] = out["e_c_fused_m"] / out["e_c_geometric_m"]
    return out


def _task(args: tuple[str, dict[str, Any], int | None]) -> tuple[str, dict[str, Any]]:
    name, cfg_d, cap = args
    return name, measure_set(name, cfg_d, cap)


def evaluate(
    cfgs: list[dict[str, Any]], cap: int | None, workers: int
) -> list[dict[str, dict[str, Any]]]:
    tasks = [(name, c, cap) for c in cfgs for name in ALL_SETS]
    if workers > 1:
        with ProcessPoolExecutor(workers) as ex:
            res = list(ex.map(_task, tasks, chunksize=1))
    else:
        res = [_task(t) for t in tasks]
    return [
        dict(res[k * len(ALL_SETS) : (k + 1) * len(ALL_SETS)]) for k in range(len(cfgs))
    ]


def meets_margin_all(per_set: dict[str, dict[str, Any]]) -> bool:
    return all(slope_select.meets_margin(v["coverage"]) for v in per_set.values())


def ratios(per_set: dict[str, dict[str, Any]]) -> dict[str, float]:
    return {n: v["ratio"] for n, v in per_set.items() if "ratio" in v}


def score(per_set: dict[str, dict[str, Any]]) -> tuple[bool, float]:
    """(meets margin on every set, mean E_c ratio over the labelled sets)."""
    r = ratios(per_set)
    return meets_margin_all(per_set), float(np.mean(list(r.values()))) if r else math.nan


def worst_margin(per_set: dict[str, dict[str, Any]]) -> float:
    """How far the worst set is over the margin (0 when none is): a tie-break for stage 1 when no
    setting meets it, so that the stage-2 search starts from the nearest one."""
    over = 0.0
    for v in per_set.values():
        c = v["coverage"]
        if not c.get("loss") or not c.get("excess"):
            return math.inf
        over = max(
            over,
            c["loss"]["mean"] - slope_select.MARGIN_MAX_MEAN,
            c["excess"]["mean"] - slope_select.MARGIN_MAX_MEAN,
            c["loss"]["p99"] - slope_select.MARGIN_MAX_P99,
            c["excess"]["p99"] - slope_select.MARGIN_MAX_P99,
        )
    return over


def base_cfg(**kw: Any) -> dict[str, Any]:
    d = fuse_v2.FuseV2Config().as_dict()
    d.update(kw)
    return d


def rank(cfgs: list[dict[str, Any]], table: list[dict[str, dict[str, Any]]]) -> list[int]:
    """Indices best first: settings that meet the margin everywhere and the benefit, by mean ratio;
    then the rest by how close to the margin they are."""
    keyed = []
    for k, per in enumerate(table):
        ok, mean_ratio = score(per)
        benefit = gp1d.meets_benefit(ratios(per))
        if ok and benefit:
            keyed.append((0, mean_ratio, k))
        elif ok:
            keyed.append((1, mean_ratio, k))
        else:
            keyed.append((2, worst_margin(per), k))
    return [k for *_, k in sorted(keyed)]


def sweep(args: argparse.Namespace) -> int:
    gp1d.load_prereg()
    log.info("stage 1: slope x extension")
    cfgs1 = [
        base_cfg(max_shift_slope=s, edge_end_extension_m=e)
        for s, e in itertools.product(SLOPE_GRID, EXT_GRID)
    ]
    t1 = evaluate(cfgs1, SUBSAMPLE, args.workers)
    r1 = rank(cfgs1, t1)
    best1 = cfgs1[r1[0]]
    log.info("stage 1 best: %s", json.dumps(best1))
    log.info("stage 2: reach x margin x blend")
    cfgs2 = [
        base_cfg(
            max_shift_slope=best1["max_shift_slope"],
            edge_end_extension_m=best1["edge_end_extension_m"],
            reach_m=r,
            edge_margin_m=m,
            blend=b,
        )
        for r, m, b in itertools.product(REACH_GRID, MARGIN_GRID, BLEND_GRID)
    ]
    t2 = evaluate(cfgs2, SUBSAMPLE, args.workers)
    cfgs = cfgs1 + cfgs2
    table = t1 + t2
    out = {
        "subsample": SUBSAMPLE,
        "settings": [{"config": c, "sets": t} for c, t in zip(cfgs, table, strict=True)],
        "rank": rank(cfgs, table),
    }
    args.out.write_text(json.dumps(out, indent=2, allow_nan=False, default=_json_default))
    top = out["rank"][:5]
    for k in top:
        ok, mean_ratio = score(table[k])
        log.info("rank %s margin=%s mean_ratio=%.3f %s", k, ok, mean_ratio, json.dumps(cfgs[k]))
    return 0


def _json_default(x: Any) -> Any:
    if isinstance(x, (np.floating, float)):
        return None if not math.isfinite(float(x)) else float(x)
    if isinstance(x, np.integer):
        return int(x)
    raise TypeError(type(x))


def verify(args: argparse.Namespace) -> int:
    """The best candidates of the sweep on the FULL sets, and the registered rule applied to
    those numbers."""
    gp1d.load_prereg()
    sweep_doc = json.loads(args.table.read_text())
    settings = sweep_doc["settings"]
    order = sweep_doc["rank"][: args.top]
    cfgs = [settings[k]["config"] for k in order]
    table = evaluate(cfgs, None, args.workers)
    rows = []
    chosen = None
    best_mean = math.inf
    for c, per in zip(cfgs, table, strict=True):
        ok, mean_ratio = score(per)
        benefit = gp1d.meets_benefit(ratios(per))
        rows.append(
            {"config": c, "sets": per, "meets_margin": ok, "meets_benefit": benefit, "mean_ratio": mean_ratio}
        )
        if ok and benefit and mean_ratio < best_mean:
            best_mean, chosen = mean_ratio, c
    result = {
        "rule": "among settings meeting the margin on every set, lowest mean E_c ratio over the "
        "labelled sets, and ratio at most 0.95 on every labelled set",
        "candidates": rows,
        "chosen": chosen,
        "outcome": gp1d.NOT_ATTEMPTED if chosen is None else "frozen: " + json.dumps(chosen),
    }
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False, default=_json_default))
    print(json.dumps({k: result[k] for k in ("chosen", "outcome")}, indent=2))
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    b = sub.add_parser("build")
    b.add_argument("--labels", type=Path, required=True)
    b.add_argument("--bags-root", type=Path, required=True)
    b.add_argument("--force", action="store_true")
    s = sub.add_parser("sweep")
    s.add_argument("--workers", type=int, default=4)
    s.add_argument("--out", type=Path, default=Path("tune_table.json"))
    v = sub.add_parser("verify")
    v.add_argument("--table", type=Path, required=True)
    v.add_argument("--top", type=int, default=8)
    v.add_argument("--workers", type=int, default=4)
    v.add_argument("--out", type=Path, default=Path("tune_result.json"))
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    return {"build": build, "sweep": sweep, "verify": verify}[args.command](args)


if __name__ == "__main__":
    sys.exit(main())
