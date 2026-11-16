"""The corridor polygon of a track and the comparison of the fused one with the geometric one.

The fused corridor is claimed to cover what the tracer's covers (design 16.4). That claim is
checked here on the OUTPUT arrays, not argued from the construction: the polygon of a track is the
left edge polyline followed by the right edge polyline reversed, where each edge point is the
station plus or minus the half width along the normal of the track's OWN polyline (the rule of
`selfcheck`, which a consumer applies), so a fused track whose centres moved is measured with its
own normals.

    coverage loss   share of the geometric polygon's area that the fused polygon does not cover
    excess          area of the fused polygon (over the tracer's stations only) outside the
                    geometric polygon, as a share of the geometric area

Both are measured on a grid (`GRID_M`) by even-odd point in polygon. Appended far stations are not
part of either number; `extra_stations` says how many there were.

    uv run --group bags python -m racer_pnp_perception.corridor \\
        --labels <dir with cache/> --bags-root <bags> --fuse-config <json> --bag <name> ... \\
        --out polygon_check.json

runs it on development bags (every bag of the 2026-11-14 session is refused: those are the fresh
sets).
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
from racer_track_perception import fuse, selfcheck
from racer_track_perception import track as tr

GRID_M = 0.01
FRESH_SESSION_PREFIX = "2026-11-14"

log = logging.getLogger(__name__)


def polygon(track: tr.LocalTrackData, n: int | None = None) -> NDArray[np.float64] | None:
    """Vertices of the corridor of the first `n` valid stations (all by default), None when there
    are fewer than two."""
    n = track.valid_count if n is None else min(n, track.valid_count)
    if n < 2:
        return None
    x = track.x_m[:n].astype(np.float64)
    y = track.y_m[:n].astype(np.float64)
    tan = selfcheck._tangents(x, y)
    nx, ny = -tan[:, 1], tan[:, 0]
    hl = track.half_width_left_m[:n].astype(np.float64)
    hr = track.half_width_right_m[:n].astype(np.float64)
    left = np.stack([x + hl * nx, y + hl * ny], axis=1)
    right = np.stack([x - hr * nx, y - hr * ny], axis=1)
    return np.concatenate([left, right[::-1]])


def _inside(poly: NDArray[np.float64], px: NDArray[np.float64], py: NDArray[np.float64]):
    """Even-odd point in polygon for arrays of points."""
    inside = np.zeros(px.shape, dtype=bool)
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if yi != yj:
            cond = (yi > py) != (yj > py)
            x_cross = (xj - xi) * (py - yi) / (yj - yi) + xi
            inside ^= cond & (px < x_cross)
        j = i
    return inside


def compare(geo: tr.LocalTrackData, fused: tr.LocalTrackData, grid_m: float = GRID_M) -> dict:
    """Coverage loss and excess of the fused polygon against the geometric one (see the module
    docstring). NaN where the geometric track has no polygon."""
    pg = polygon(geo)
    n = geo.valid_count
    pf = polygon(fused, n)  # the tracer's stations only
    extra = max(fused.valid_count - n, 0)
    if pg is None or pf is None:
        return {"geo_area_m2": math.nan, "loss": math.nan, "excess": math.nan, "extra": extra}
    both = np.concatenate([pg, pf])
    lo, hi = both.min(axis=0) - grid_m, both.max(axis=0) + grid_m
    gx, gy = np.meshgrid(
        np.arange(lo[0], hi[0], grid_m) + 0.37 * grid_m,
        np.arange(lo[1], hi[1], grid_m) + 0.37 * grid_m,
    )
    gx, gy = gx.ravel(), gy.ravel()
    in_g = _inside(pg, gx, gy)
    in_f = _inside(pf, gx, gy)
    cell = grid_m * grid_m
    area_g = float(in_g.sum()) * cell
    if area_g <= 0.0:
        return {"geo_area_m2": 0.0, "loss": math.nan, "excess": math.nan, "extra": extra}
    return {
        "geo_area_m2": area_g,
        "loss": float((in_g & ~in_f).sum()) * cell / area_g,
        "excess": float((in_f & ~in_g).sum()) * cell / area_g,
        "extra": extra,
    }


def summarise(results: list[dict]) -> dict[str, Any]:
    ok = [r for r in results if math.isfinite(r["loss"])]
    out: dict[str, Any] = {"scans": len(results), "compared": len(ok)}
    for key in ("loss", "excess"):
        v = np.array([r[key] for r in ok])
        out[key] = (
            {
                "mean": float(v.mean()),
                "p50": float(np.percentile(v, 50)),
                "p99": float(np.percentile(v, 99)),
                "max": float(v.max()),
                "share_over_0.01": float((v > 0.01).mean()),
            }
            if len(v)
            else None
        )
    out["extra_stations_mean"] = float(np.mean([r["extra"] for r in results])) if results else None
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--labels", type=Path, required=True, help="dir holding cache/")
    parser.add_argument("--bags-root", type=Path, required=True)
    parser.add_argument("--bag", nargs="+", required=True)
    parser.add_argument("--run", type=Path, required=True, help="net run dir (weights)")
    parser.add_argument("--fuse-config", type=Path, required=True)
    parser.add_argument("--max-scans", type=int, default=1500)
    parser.add_argument("--out", type=Path, default=Path("polygon_check.json"))
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    from racer_track_perception.core import BACKEND_GEOMETRIC, BACKEND_NET, TrackPerceiver
    from racer_track_perception.vehicle import load_live_vehicle

    from racer_pnp_perception import selflabel as sl
    from racer_pnp_perception.internal_val import net_from_run

    for name in args.bag:
        if name.startswith(FRESH_SESSION_PREFIX):
            raise SystemExit(
                f"refusing {name}: the 2026-11-14 bags are the fresh sets, not dev data"
            )
    cfg = fuse.FuseConfig.from_dict(json.loads(args.fuse_config.read_text()))
    live = load_live_vehicle()
    net = net_from_run(args.run)
    geo_p = TrackPerceiver(BACKEND_GEOMETRIC, live)
    fused_p = TrackPerceiver(BACKEND_NET, live, net=net, model_id=args.run.name, fuse_cfg=cfg)
    result: dict[str, Any] = {"fuse_config": cfg.as_dict(), "grid_m": GRID_M, "bags": {}}
    for name in args.bag:
        scans, speed_t, speed = sl.load_scanset(args.bags_root / name, args.labels / "cache")
        idx = np.flatnonzero(sl.moving_scans(scans, speed_t, speed, 0.2))
        idx = idx[np.linspace(0, len(idx) - 1, min(args.max_scans, len(idx))).round().astype(int)]
        rows = []
        for i in idx:
            a = (scans.angle_min, scans.angle_increment, scans.range_min, scans.range_max)
            geo = geo_p.process(scans.ranges[i], *a)
            fused = fused_p.process(scans.ranges[i], *a)
            rows.append(compare(geo, fused))
        result["bags"][name] = summarise(rows)
        log.info("%s: %s", name, json.dumps(result["bags"][name]))
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
