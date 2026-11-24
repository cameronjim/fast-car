"""trackperc-v3 against trackperc-v2 on the held-out sets of
evaluation/prereg/w2-trackperc-v3-2026-11-23.yaml, and the ship rule written there.

    uv run --group bags python -m racer_pnp_perception.compare_v3 \\
        --split evaluation/prereg/w2-trackperc-v3-2026-11-23.yaml --split-sha256 <digest> \\
        --labels /Users/cameronjim/code/car-data/pnp/selflabels-v3 \\
        --v2 <v2 bundle dir> --v3 <v3 bundle dir> --bags <bag dir> [...] --out compare_v3.json

An unregistered development comparison (overnight 2026-11-23, W2, owner decision 29), not a gate.
Every backend (geometric-v1, v2 and v3, each with the refinement its bundle declares) runs through
`TrackPerceiver.process` on the same scans, scored by the GP1 code (`gp1.collect_bag`,
`gp1.score_row`, `labelled_numbers`, `label_free_numbers`): H-layout is the `heldout_layout`
role, H-session the `heldout` role. Run it once, after v3 is frozen.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import logging
import math
import sys
from pathlib import Path
from typing import Any

from racer_track_perception.bundle import load_bundle
from racer_track_perception.core import BACKEND_GEOMETRIC, BACKEND_NET, TrackPerceiver
from racer_track_perception.runtime import CircConv1dV0
from racer_track_perception.vehicle import LiveVehicle, load_live_vehicle

from racer_pnp_perception import gp1
from racer_pnp_perception.split import ROLE_HELDOUT, ROLE_HELDOUT_LAYOUT, load_split

log = logging.getLogger(__name__)

SET_LAYOUT = "H-layout"
SET_SESSION = "H-session"
ROLE_TO_SET = {ROLE_HELDOUT_LAYOUT: SET_LAYOUT, ROLE_HELDOUT: SET_SESSION}
V2, V3 = "v2", "v3"

# The ship rule of the yaml (stated before training).
LAYOUT_V_MARGIN = 0.25
LAYOUT_C95_MARGIN = 0.02
LAYOUT_INTRUSION_FLOOR = 0.02
LAYOUT_INTRUSION_MARGIN = 0.005
SESSION_EC_RATIO = 1.05
SESSION_V_MARGIN = 0.25
LATENCY_P99_RATIO = 1.2


def decide(
    labelled: dict[str, dict[str, gp1.Labelled]],
    label_free: dict[str, dict[str, gp1.LabelFree]],
    latency_p99_ms: dict[str, float],
) -> tuple[bool, list[dict[str, Any]]]:
    """(ship, items). `labelled[set][backend]`, `label_free[set][backend]`, backends v2 and v3.
    A NaN number fails its item (nothing to show)."""
    lay, ses, free = labelled[SET_LAYOUT], labelled[SET_SESSION], label_free[SET_LAYOUT]
    intrusion_cap = max(LAYOUT_INTRUSION_FLOOR, free[V2].intrusion_mean + LAYOUT_INTRUSION_MARGIN)
    checks = [
        ("H-layout E_c(v3) <= E_c(v2)", lay[V3].e_c_m, "<=", lay[V2].e_c_m),
        ("H-layout V(v3) >= V(v2) - 0.25", lay[V3].v, ">=", lay[V2].v - LAYOUT_V_MARGIN),
        ("H-layout C95(v3) >= C95(v2) - 0.02", lay[V3].c95, ">=", lay[V2].c95 - LAYOUT_C95_MARGIN),
        (
            "H-layout intrusion(v3) <= max(0.02, intrusion(v2) + 0.005)",
            free[V3].intrusion_mean,
            "<=",
            intrusion_cap,
        ),
        (
            "H-session E_c(v3) <= 1.05 E_c(v2)",
            ses[V3].e_c_m,
            "<=",
            SESSION_EC_RATIO * ses[V2].e_c_m,
        ),
        ("H-session V(v3) >= V(v2) - 0.25", ses[V3].v, ">=", ses[V2].v - SESSION_V_MARGIN),
        (
            "latency p99(v3) <= 1.2 p99(v2)",
            latency_p99_ms[V3],
            "<=",
            LATENCY_P99_RATIO * latency_p99_ms[V2],
        ),
    ]
    items = []
    for name, value, op, limit in checks:
        ok = math.isfinite(value) and math.isfinite(limit)
        ok = ok and (value <= limit if op == "<=" else value >= limit)
        items.append({"item": name, "value": value, "limit": limit, "passed": bool(ok)})
    return all(i["passed"] for i in items), items


def perceivers(live: LiveVehicle, bundles: dict[str, Path]) -> dict[str, TrackPerceiver]:
    out = {BACKEND_GEOMETRIC: TrackPerceiver(BACKEND_GEOMETRIC, live)}
    for name, path in bundles.items():
        b = load_bundle(path, "track_perception", live)
        out[name] = TrackPerceiver(
            BACKEND_NET,
            live,
            net=CircConv1dV0.from_npz(b.payload_path("weights.npz")),
            model_id=b.model_id,
            refine_net=b.refine,
            fuse_cfg=b.fuse_config,
        )
    return out


def latency_p99(perc: dict[str, TrackPerceiver], scans: Any, n: int = 1000) -> dict[str, float]:
    import time

    import numpy as np

    pick = np.linspace(0, len(scans) - 1, min(n, len(scans))).round().astype(int)
    geo = (scans.angle_min, scans.angle_increment, scans.range_min, scans.range_max)
    out = {}
    for name, p in perc.items():
        for i in pick[:20]:
            p.process(scans.ranges[i], *geo)
        ms = []
        for i in pick:
            t0 = time.perf_counter()
            p.process(scans.ranges[i], *geo)
            ms.append((time.perf_counter() - t0) * 1000.0)
        out[name] = float(np.percentile(ms, 99))
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--split", type=Path, required=True)
    parser.add_argument("--split-sha256", required=True)
    parser.add_argument("--labels", type=Path, required=True)
    parser.add_argument("--v2", type=Path, required=True)
    parser.add_argument("--v3", type=Path, required=True)
    parser.add_argument("--bags", type=Path, nargs="+", required=True)
    parser.add_argument("--out", type=Path, default=Path("compare_v3.json"))
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    from racer_pnp_perception import selflabel as sl

    live = load_live_vehicle()
    split = load_split(args.split, expected_sha256=args.split_sha256)
    perc = perceivers(live, {V2: args.v2, V3: args.v3})
    rows: list[gp1.ScanRow] = []
    for bag in args.bags:
        rows += gp1.collect_bag(bag, args.labels, split, perc, live, role_to_set=ROLE_TO_SET)
    labelled = {
        s: {b: gp1.labelled_numbers([r for r in rows if r.set == s], b) for b in perc}
        for s in (SET_LAYOUT, SET_SESSION)
    }
    label_free = {
        s: {b: gp1.label_free_numbers([r for r in rows if r.set == s], b) for b in perc}
        for s in (SET_LAYOUT, SET_SESSION)
    }
    scans, _t, _v = sl.load_scanset(args.bags[0], args.labels / "cache")
    latency = latency_p99(perc, scans)
    ship, items = decide(labelled, label_free, latency)
    result = {
        "split_sha256": split.sha256,
        "bundles": {V2: str(args.v2), V3: str(args.v3)},
        "labelled": {
            s: {b: dataclasses.asdict(v) for b, v in d.items()} for s, d in labelled.items()
        },
        "label_free": {
            s: {b: dataclasses.asdict(v) for b, v in d.items()} for s, d in label_free.items()
        },
        "latency_p99_ms_dev_mac": latency,
        "items": items,
        "ship_v3": ship,
    }
    args.out.write_text(json.dumps(gp1._clean(result), indent=2))
    print(json.dumps(gp1._clean(result), indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
