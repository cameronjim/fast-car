"""Self-labels against the map: an independent check of label geometry (no net involved).

    uv run --group bags python -m racer_pnp_perception.label_map_check --bag <bag dir> \\
        --labels NAME=<label dir> [--labels NAME=<label dir> ...] [--out <json>]

For a bag recorded with a localiser running against a saved occupancy grid (`/map`, and
`map -> odom -> base_link` on `/tf`), the labeller never saw the map or the pose. Every label
station is moved into the map with the pose at the scan's header stamp and compared with the
occupied cells:

* wall distance: per station, the nearest occupied cell on each side of the label's own tangent
  (within `WINDOW_M` along it, `FEELER_MAX_M` away), against the label's half widths
  (|hl - d_left|, |hr - d_right|), and the centring error |d_left - d_right| / 2 (a point of the
  true centreline is equidistant from both walls);
* curvature: the circumradius of every three consecutive stations against the physical minimum
  radius of a centreline, the corridor half width (a centreline rounds a sharp inner corner at a
  radius equal to the half width; anything tighter is not a centreline). Reported as the share of
  stations with radius under 0.8 x the map half width at that station;
* station spacing: the straight-line gaps between consecutive stations (contract 0.20 m).

Only scans labelled in every given label set are compared (same scans, same stations count per
scan: the smaller valid count), so the sets differ only in their geometry. The map, the poses and
the pose interpolation are the same for every set.
"""

from __future__ import annotations

import argparse
import glob
import json
import logging
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

log = logging.getLogger(__name__)

OCCUPIED = 65
WINDOW_M = 0.30
FEELER_MAX_M = 1.5
RADIUS_SHARE = 0.8


def read_map_and_tf(bag: Path) -> tuple[NDArray[np.float64], dict[str, Any]]:
    """Occupied cell centres [n, 2] of the last /map, and the tf series."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    occ = None
    m2o: list[tuple[float, float, float, float]] = []
    o2b: list[tuple[float, float, float, float]] = []
    for f in sorted(glob.glob(str(bag / "*.mcap"))):
        with open(f, "rb") as stream:
            messages = list(
                make_reader(stream, decoder_factories=[DecoderFactory()]).iter_decoded_messages(
                    topics=["/map", "/tf"]
                )
            )
        for _s, ch, _m, msg in messages:
            if ch.topic == "/map":
                info = msg.info
                grid = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
                iy, ix = np.nonzero(grid >= OCCUPIED)
                res = info.resolution
                yaw0 = _yaw(info.origin.orientation)
                lx, ly = (ix + 0.5) * res, (iy + 0.5) * res
                c, s = math.cos(yaw0), math.sin(yaw0)
                occ = np.c_[
                    info.origin.position.x + c * lx - s * ly,
                    info.origin.position.y + s * lx + c * ly,
                ]
                continue
            for t in msg.transforms:
                st = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
                row = (st, t.transform.translation.x, t.transform.translation.y)
                row = (*row, _yaw(t.transform.rotation))
                if t.header.frame_id == "map" and t.child_frame_id == "odom":
                    m2o.append(row)
                elif t.header.frame_id == "odom" and t.child_frame_id == "base_link":
                    o2b.append(row)
    if occ is None:
        raise SystemExit(f"{bag}: no /map")
    return occ, {"map_odom": np.array(sorted(m2o)), "odom_base": np.array(sorted(o2b))}


def _yaw(q: Any) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _compose(a: NDArray[np.float64], b: NDArray[np.float64]) -> NDArray[np.float64]:
    """Pose a (x, y, yaw) then b in a's frame."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]])


def pose_at(tf: dict[str, Any], t: float, max_gap_s: float = 0.1) -> NDArray[np.float64] | None:
    """base_link in map at t: odom->base_link interpolated, map->odom the latest at or before t.
    None when there is no odom sample within max_gap_s either side or no map->odom yet."""
    ob, mo = tf["odom_base"], tf["map_odom"]
    if len(ob) < 2 or len(mo) == 0:
        return None
    j = int(np.searchsorted(ob[:, 0], t))
    if j <= 0 or j >= len(ob) or ob[j, 0] - ob[j - 1, 0] > 2 * max_gap_s:
        return None
    f = (t - ob[j - 1, 0]) / max(ob[j, 0] - ob[j - 1, 0], 1e-9)
    dyaw = (ob[j, 3] - ob[j - 1, 3] + math.pi) % (2 * math.pi) - math.pi
    odom_base = np.array(
        [
            ob[j - 1, 1] + f * (ob[j, 1] - ob[j - 1, 1]),
            ob[j - 1, 2] + f * (ob[j, 2] - ob[j - 1, 2]),
            ob[j - 1, 3] + f * dyaw,
        ]
    )
    k = int(np.searchsorted(mo[:, 0], t, side="right")) - 1
    if k < 0:
        return None
    return _compose(mo[k, 1:], odom_base)


def to_base(occ: NDArray[np.float64], pose: NDArray[np.float64]) -> NDArray[np.float64]:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    d = occ - pose[:2]
    return np.c_[c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]]


def station_numbers(
    label: NDArray[np.floating], n: int, pts: NDArray[np.float64]
) -> dict[str, NDArray[np.float64]]:
    """Per station (first n): d_left, d_right (NaN when no wall cell), the label half widths, the
    circumradius of stations k-1, k, k+1 (NaN at the ends) and the gap to station k+1."""
    xy = label[:n, :2].astype(np.float64)
    tang = np.gradient(xy, axis=0) if n >= 2 else np.zeros_like(xy)
    norm = np.hypot(tang[:, 0], tang[:, 1])
    norm[norm < 1e-12] = 1.0
    tx, ty = tang[:, 0] / norm, tang[:, 1] / norm
    dl = np.full(n, np.nan)
    dr = np.full(n, np.nan)
    for k in range(n):
        rx, ry = pts[:, 0] - xy[k, 0], pts[:, 1] - xy[k, 1]
        along = rx * tx[k] + ry * ty[k]
        lat = -rx * ty[k] + ry * tx[k]
        dist = np.hypot(rx, ry)
        near = (np.abs(along) <= WINDOW_M) & (dist <= FEELER_MAX_M)
        if (near & (lat > 0)).any():
            dl[k] = dist[near & (lat > 0)].min()
        if (near & (lat < 0)).any():
            dr[k] = dist[near & (lat < 0)].min()
    rad = np.full(n, np.nan)
    for k in range(1, n - 1):
        a, b, c = xy[k - 1], xy[k], xy[k + 1]
        ab, bc, ca = np.hypot(*(b - a)), np.hypot(*(c - b)), np.hypot(*(a - c))
        cross = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
        rad[k] = ab * bc * ca / (2.0 * cross) if cross > 1e-12 else np.inf
    gap = np.full(n, np.nan)
    gap[: n - 1] = np.hypot(*np.diff(xy, axis=0).T)
    return {
        "dl": dl,
        "dr": dr,
        "hl": label[:n, 2].astype(np.float64),
        "hr": label[:n, 3].astype(np.float64),
        "radius": rad,
        "gap": gap,
    }


def summarise(per: list[dict[str, NDArray[np.float64]]]) -> dict[str, float]:
    cat = {k: np.concatenate([p[k] for p in per]) for k in per[0]}
    both = np.isfinite(cat["dl"]) & np.isfinite(cat["dr"])
    hw_map = 0.5 * (cat["dl"] + cat["dr"])
    wall_err = np.concatenate(
        [
            np.abs(cat["hl"] - cat["dl"])[np.isfinite(cat["dl"])],
            np.abs(cat["hr"] - cat["dr"])[np.isfinite(cat["dr"])],
        ]
    )
    centring = 0.5 * np.abs(cat["dl"] - cat["dr"])[both]
    radius = np.where(np.isinf(cat["radius"]), 1e6, cat["radius"])  # straight: no curvature
    rad_ok = np.isfinite(radius) & both
    tight = radius[rad_ok] < RADIUS_SHARE * hw_map[rad_ok]
    gaps = cat["gap"][np.isfinite(cat["gap"])]
    return {
        "stations": len(cat["dl"]),
        "stations_both_walls": int(both.sum()),
        "map_half_width_median_m": float(np.median(hw_map[both])),
        "wall_distance_error_mean_m": float(wall_err.mean()),
        "wall_distance_error_p90_m": float(np.percentile(wall_err, 90)),
        "centring_error_mean_m": float(centring.mean()),
        "centring_error_p90_m": float(np.percentile(centring, 90)),
        "radius_p1_m": float(np.percentile(radius[rad_ok], 1)),
        "radius_p5_m": float(np.percentile(radius[rad_ok], 5)),
        "share_radius_under_0p8_half_width": float(tight.mean()),
        "gap_mean_m": float(gaps.mean()),
        "gap_p5_m": float(np.percentile(gaps, 5)),
        "gap_min_m": float(gaps.min()),
    }


def check(bag: Path, label_dirs: dict[str, Path], role: str | None = None) -> dict[str, Any]:
    occ, tf = read_map_and_tf(bag)
    sets = {k: np.load(d / f"{bag.name}.npz") for k, d in label_dirs.items()}
    common = None
    for z in sets.values():
        keep = z["role"] == role if role else np.ones(len(z["index"]), bool)
        idx = set(z["index"][keep].tolist())
        common = idx if common is None else common & idx
    assert common is not None
    by = {k: {int(i): j for j, i in enumerate(z["index"])} for k, z in sets.items()}
    per: dict[str, list[dict[str, NDArray[np.float64]]]] = {k: [] for k in sets}
    used = 0
    for i in sorted(common):
        any_z = sets[next(iter(sets))]
        t = float(any_z["stamp_s"][by[next(iter(sets))][i]])
        pose = pose_at(tf, t)
        if pose is None:
            continue
        pts = to_base(occ, pose)
        n = min(int(sets[k]["valid_count"][by[k][i]]) for k in sets)
        for k, z in sets.items():
            per[k].append(station_numbers(z["label"][by[k][i]], n, pts))
        used += 1
    return {
        "bag": bag.name,
        "role": role,
        "scans_common": len(common),
        "scans_with_pose": used,
        "sets": {k: summarise(v) for k, v in per.items()} if used else {},
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--labels", action="append", required=True, help="NAME=DIR")
    parser.add_argument("--role", default=None, help="only labels of this split role")
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    dirs = {}
    for item in args.labels:
        name, _, path = item.partition("=")
        dirs[name] = Path(path)
    result = check(args.bag, dirs, args.role)
    text = json.dumps(result, indent=2)
    if args.out:
        args.out.write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
