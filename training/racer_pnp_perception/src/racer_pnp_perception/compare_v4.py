"""trackperc-v4 against trackperc-v2 on the held-out sets of
evaluation/prereg/w2-trackperc-v4-2026-11-25.yaml, and the ship rule written there.

    uv run --group bags python -m racer_pnp_perception.compare_v4 \\
        --split evaluation/prereg/w2-trackperc-v4-2026-11-25.yaml --split-sha256 <digest> \\
        --labels /Users/cameronjim/code/car-data/pnp/labels/selflabels-v4-fast \\
        --v2 <v2 bundle dir> --v4 <v4 bundle dir> [--ref v3=<v3 bundle dir>] \\
        --bags <bag dir> [...] --out compare_v4.json

An unregistered development comparison (owner decisions 29 and 35), not a gate. The rule is the
v3 rule (`compare_v3.decide`, applied with v4 in v3's place) plus one item: H-session needs at
least `MIN_SESSION_LABELS` labelled scans (v3's H-session had 11). Reference backends (`--ref`)
are scored the same way and decide nothing. Run it once, after v4 is frozen; a run on another
label set (for example the v3 one) is a reference number only.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import logging
import sys
from pathlib import Path
from typing import Any

from racer_track_perception.vehicle import load_live_vehicle

from racer_pnp_perception import compare_v3 as cv
from racer_pnp_perception import gp1
from racer_pnp_perception.split import load_split

log = logging.getLogger(__name__)

V2, V4 = cv.V2, "v4"
MIN_SESSION_LABELS = 300


def decide(
    labelled: dict[str, dict[str, gp1.Labelled]],
    label_free: dict[str, dict[str, gp1.LabelFree]],
    latency_p99_ms: dict[str, float],
) -> tuple[bool, list[dict[str, Any]]]:
    """(ship, items) for v4 against v2: the v3 items with v4 in v3's place, then the H-session
    label count item. A NaN number fails its item."""

    def as_v3(d: dict[str, Any]) -> dict[str, Any]:
        return {cv.V2: d[V2], cv.V3: d[V4]}

    _, items = cv.decide(
        {s: as_v3(d) for s, d in labelled.items()},
        {s: as_v3(d) for s, d in label_free.items()},
        as_v3(latency_p99_ms),
    )
    for item in items:
        item["item"] = item["item"].replace("(v3)", "(v4)")
    count = labelled[cv.SET_SESSION][V4].scans
    items.insert(
        4,
        {
            "item": f"H-session labelled scans >= {MIN_SESSION_LABELS}",
            "value": count,
            "limit": MIN_SESSION_LABELS,
            "passed": bool(count >= MIN_SESSION_LABELS),
        },
    )
    return all(i["passed"] for i in items), items


def parse_refs(refs: list[str]) -> dict[str, Path]:
    out = {}
    for r in refs:
        name, _, path = r.partition("=")
        if not name or not path or name in (V2, V4):
            raise SystemExit(f"--ref {r!r}: expected NAME=DIR with a name other than v2 and v4")
        out[name] = Path(path)
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--split", type=Path, required=True)
    parser.add_argument("--split-sha256", required=True)
    parser.add_argument("--labels", type=Path, required=True)
    parser.add_argument("--v2", type=Path, required=True)
    parser.add_argument("--v4", type=Path, required=True)
    parser.add_argument("--ref", action="append", default=[], help="NAME=DIR, decides nothing")
    parser.add_argument("--bags", type=Path, nargs="+", required=True)
    parser.add_argument("--out", type=Path, default=Path("compare_v4.json"))
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    from racer_pnp_perception import selflabel as sl

    live = load_live_vehicle()
    split = load_split(args.split, expected_sha256=args.split_sha256)
    bundles = {V2: args.v2, V4: args.v4, **parse_refs(args.ref)}
    perc = cv.perceivers(live, bundles)
    rows: list[gp1.ScanRow] = []
    for bag in args.bags:
        rows += gp1.collect_bag(bag, args.labels, split, perc, live, role_to_set=cv.ROLE_TO_SET)
    sets = (cv.SET_LAYOUT, cv.SET_SESSION)
    labelled = {
        s: {b: gp1.labelled_numbers([r for r in rows if r.set == s], b) for b in perc} for s in sets
    }
    label_free = {
        s: {b: gp1.label_free_numbers([r for r in rows if r.set == s], b) for b in perc}
        for s in sets
    }
    scans, _t, _v = sl.load_scanset(args.bags[0], args.labels / "cache")
    latency = cv.latency_p99(perc, scans)
    ship, items = decide(labelled, label_free, latency)
    result = {
        "split_sha256": split.sha256,
        "labels": str(args.labels),
        "bundles": {k: str(v) for k, v in bundles.items()},
        "labelled": {
            s: {b: dataclasses.asdict(v) for b, v in d.items()} for s, d in labelled.items()
        },
        "label_free": {
            s: {b: dataclasses.asdict(v) for b, v in d.items()} for s, d in label_free.items()
        },
        "latency_p99_ms_dev_mac": latency,
        "items": items,
        "ship_v4": ship,
    }
    args.out.write_text(json.dumps(gp1._clean(result), indent=2))
    print(json.dumps(gp1._clean(result), indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
