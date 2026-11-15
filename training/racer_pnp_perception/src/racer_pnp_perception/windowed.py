"""Time-windowed read of the 2026-11-14 bags: only the perception `heldout` segments are decoded.

    uv run --group bags python -m racer_pnp_perception.windowed \\
        --bags <bag dir> [<bag dir> ...] --out <selflabels-v0.4 dir> [--prereg <yaml>]

`selflabel.load_scanset` decodes a whole bag. For the 2026-11-14 bags that would decode the
perception `reserve` segments, which nobody reads in v0.4 (pre-registration 3.1), so here each bag
is read through `follower_replay.bag.read_bag(window_s=...)`, which seeks by the chunk index and
decodes only the messages inside the given windows. The windows are exactly the heldout segments
of `split_v04.perception_split`; a reserve segment is never inside one, and the code asserts it.
The result is written as the scan cache `<out>/cache/<bag>.scans.npz` that `load_scanset` reads
instead of the bag, so `selflabel` and `gp1.collect_bag` run unchanged from it. A later read of the
bag itself is prevented by `require_cache`, which `gp1c score` calls.

The cache holds the scans of the windows only, each with its log time on the split's axis
(seconds after the first /scan message of the whole bag, `first_scan_log_time_s` of the yaml), and
the /odom/wheel samples of the same windows. A bag without /odom/wheel in the windows is refused:
the ICP ground speed would need every interval of the bag.
"""

from __future__ import annotations

import argparse
import logging
import sys
from collections.abc import Callable
from pathlib import Path
from typing import Any

import numpy as np

from racer_pnp_perception import selflabel as sl
from racer_pnp_perception import split_v04
from racer_pnp_perception.split import ROLE_HELDOUT, Split

log = logging.getLogger(__name__)


class WindowedReadError(RuntimeError):
    """The bag cannot be read by windows (no origin, several files, no odometry)."""


def heldout_windows(split: Split, bag: str) -> list[tuple[str, float, float]]:
    """(segment name, from_s, to_s) of the heldout segments of `bag` on the split's time axis."""
    entry = split.bags.get(bag)
    if entry is None or entry.first_scan_log_time_s is None:
        raise WindowedReadError(
            f"{bag}: not a heldout bag of the split, or no first_scan_log_time_s"
        )
    out = [(s.name, s.t_from_s, s.t_to_s) for s in entry.segments if s.role == ROLE_HELDOUT]
    if not out:
        raise WindowedReadError(f"{bag}: no heldout segment")
    return out


def read_windows_s(
    windows: list[tuple[str, float, float]], first_scan_log_s: float, origin_log_s: float
) -> list[tuple[float, float]]:
    """The windows as (start, end) seconds after the file's first message, the axis of
    `read_bag(window_s=...)`: split time plus the first scan's log time minus the file origin."""
    shift = first_scan_log_s - origin_log_s
    return [(lo + shift, hi + shift) for _n, lo, hi in windows]


def bag_origin_log_s(bag_dir: Path) -> float:
    """Log time (s) of the first message of the bag's single mcap file, from its index."""
    from mcap.reader import make_reader

    _tools()
    from follower_replay.bag import bag_files

    files = list(bag_files(bag_dir))
    if len(files) != 1:
        raise WindowedReadError(f"{bag_dir}: windowed read needs exactly one mcap file")
    with files[0].open("rb") as stream:
        summary = make_reader(stream).get_summary()
    if summary is None or summary.statistics is None:
        raise WindowedReadError(f"{files[0]}: no index, cannot seek by window")
    return float(summary.statistics.message_start_time) * 1e-9


def _tools() -> None:
    sl._tools()


def build_windowed_cache(
    bag_dir: Path,
    split: Split,
    cache_dir: Path,
    read_bag: Callable[..., Any] | None = None,
    origin_log_s: float | None = None,
    doc: dict[str, Any] | None = None,
) -> Path:
    """Decode only the heldout windows of `bag_dir` and write `<cache_dir>/<bag>.scans.npz`."""
    name = bag_dir.name
    windows = heldout_windows(split, name)
    if doc is not None:
        assert_windows_clear(doc, split, name)
    first_scan = split.bags[name].first_scan_log_time_s
    assert first_scan is not None
    if read_bag is None:
        _tools()
        from follower_replay.bag import read_bag as _read

        read_bag = _read
    origin = bag_origin_log_s(bag_dir) if origin_log_s is None else origin_log_s
    stamps: list[float] = []
    log_s: list[float] = []
    ranges: list[Any] = []
    odom_t: list[float] = []
    odom_v: list[float] = []
    geometry: tuple[float, float, float, float] | None = None
    for (seg, lo, hi), win in zip(
        windows, read_windows_s(windows, first_scan, origin), strict=True
    ):
        data = read_bag(bag_dir, window_s=win)
        for s in data.scans:
            t = float(s.log_time_s) - first_scan
            if not lo <= t <= hi:
                continue  # the chunk edge of the seek: outside the segment
            g = (
                float(s.angle_min),
                float(s.angle_increment),
                float(s.range_min),
                float(s.range_max),
            )
            geometry = geometry or g
            stamps.append(float(s.stamp_s))
            log_s.append(t)
            ranges.append(np.asarray(s.ranges, dtype=np.float32))
        for o in data.odom:
            odom_t.append(float(o.stamp_s))
            odom_v.append(float(o.speed_mps))
        log.info("%s/%s: %d scans decoded", name, seg, len(stamps))
    if not stamps or geometry is None:
        raise WindowedReadError(f"{name}: no scan inside the heldout windows")
    if not odom_t:
        raise WindowedReadError(f"{name}: no /odom/wheel in the heldout windows")
    log_arr = np.array(log_s)
    if doc is not None:
        for bag_name, lo, hi in split_v04.reserve_intervals(doc):
            if bag_name == name and np.any((log_arr >= lo) & (log_arr <= hi)):
                raise split_v04.ReserveSegmentRefused(f"{name}: a decoded scan lies in reserve")
    cache_dir.mkdir(parents=True, exist_ok=True)
    path = cache_dir / f"{name}.scans.npz"
    np.savez(
        path,
        stamp_s=np.array(stamps),
        ranges=np.stack(ranges),
        angle_min=geometry[0],
        angle_increment=geometry[1],
        range_min=geometry[2],
        range_max=geometry[3],
        speed_t=np.array(odom_t),
        speed_mps=np.array(odom_v),
        speed_source=sl.SPEED_SOURCE_ODOM,
        log_s=log_arr,
        log_t0_s=first_scan,
    )
    return path


def assert_windows_clear(doc: dict[str, Any], split: Split, bag: str) -> None:
    """No window to be decoded touches a reserve segment of the yaml."""
    for _n, lo, hi in heldout_windows(split, bag):
        split_v04.assert_not_reserve(doc, bag, lo, hi)


def require_cache(cache_dir: Path, bag: str) -> Path:
    """The windowed cache of `bag`, or an error: the whole bag must not be read instead."""
    path = cache_dir / f"{bag}.scans.npz"
    if not path.is_file():
        raise WindowedReadError(
            f"{path} is missing: build it with `python -m racer_pnp_perception.windowed` "
            "(a whole-bag read would decode reserve segments)"
        )
    return path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bags", nargs="+", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--prereg", type=Path, default=None)
    parser.add_argument("--moving-mps", type=float, default=0.2)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--cache-only", action="store_true")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    doc, digest = split_v04.read_yaml(args.prereg)
    split = split_v04.perception_split_from_doc(doc, digest)
    from racer_track_perception.vehicle import load_live_vehicle

    live = load_live_vehicle()
    for bag in args.bags:
        if bag.name not in split.bags:
            log.info("%s: no heldout segment, not opened", bag.name)
            continue
        build_windowed_cache(bag, split, args.out / "cache", doc=doc)
        if not args.cache_only:
            sl.label_bag(bag, args.out, args.moving_mps, args.workers, live, split)
    return 0


if __name__ == "__main__":
    sys.exit(main())
