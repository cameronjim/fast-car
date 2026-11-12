"""Regenerate tools/tests/fixtures/pnp_replay_2026-11-12.npz from the 2026-11-12 bag.

    cd tools && uv run python -m pnp_replay.cut_fixture <bag dir | recording .npz> \\
        [--out tests/fixtures/pnp_replay_2026-11-12.npz]
 Four cuts, each with the scans, wheel
odometry, recorded /pnp/local_track and recorded /pnp/planner/status of its window, plus
PREROLL_S before it so a replay has a scan, a track and odometry before its first tick:

    R0  3.0 to 7.5 s after the run's first status message   the NO_TRACK onset
    R1  6.0 to 12.0 s                                        the corner approach, the stall
    R3  4.0 to 6.0 s                                         nose-in, no free forward arc
    L   bag time 1760 to 1770 s of the longest fast stretch  another driver's laps

Keys are "<cut>__<recording key>", plus "<cut>__window" = [t_from, t_to] in bag seconds. The file
stays under 450 kB: scans are uint16 quarter millimetres (lossless, see bag.py) and status
fields are float32. `load_cut` reads one cut back as an ordinary recording.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import numpy as np

from pnp_replay import bag, planner_stream, segments

log = logging.getLogger(__name__)

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUT = REPO_ROOT / "tools" / "tests" / "fixtures" / "pnp_replay_2026-11-12.npz"
MAX_BYTES = 450_000
KEEP_GROUPS = ("scan", "odom", "trk", "st")
# name -> (segment, start offset from the segment's first status, end offset); L is absolute.
PNP_CUTS = {"R0": (3.0, 7.5), "R1": (6.0, 12.0), "R3": (4.0, 6.0)}
FAST_CUT = ("L", 1760.0, 1770.0)


def windows(found: list[segments.Segment]) -> dict[str, tuple[float, float]]:
    """Cut name -> (t_from, t_to) in bag seconds."""
    by_name = {s.name: s for s in found}
    out: dict[str, tuple[float, float]] = {}
    for name, (a, b) in PNP_CUTS.items():
        out[name] = (by_name[name].t_from + a, by_name[name].t_from + b)
    name, a, b = FAST_CUT
    seg = by_name[name]
    if not (seg.t_from <= a and b <= seg.t_to):
        raise ValueError(
            f"segment {name} ({seg.t_from:.0f}-{seg.t_to:.0f} s) does not hold {a}-{b}"
        )
    out[name] = (a, b)
    return out


def build(rec: bag.Recording) -> dict[str, np.ndarray]:
    found = segments.find_segments(rec["st_t"], rec["odom_t"], rec["odom_v"])
    out: dict[str, np.ndarray] = {}
    for name, (t_from, t_to) in windows(found).items():
        part = bag.cut(rec, t_from - planner_stream.PREROLL_S, t_to)
        for key, value in part.items():
            group = bag.group_of(key)
            if group is None or group not in KEEP_GROUPS:
                continue
            if group == "st" and key != "st_t":
                value = value.astype(np.float32)
            out[f"{name}__{key}"] = value
        out[f"{name}__window"] = np.array([t_from, t_to])
        out[f"{name}__t0_log_s"] = rec["t0_log_s"]
    out["cache_version"] = np.array(bag.CACHE_VERSION)
    return out


def load_cut(path: Path, name: str) -> tuple[bag.Recording, float, float]:
    """One cut of the fixture as a recording, and its window (t_from, t_to) in bag seconds."""
    with np.load(path, allow_pickle=False) as data:
        prefix = f"{name}__"
        rec = {k[len(prefix) :]: data[k] for k in data.files if k.startswith(prefix)}
        window = rec.pop("window")
    rec["cache_version"] = np.array(bag.CACHE_VERSION)
    return rec, float(window[0]), float(window[1])


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("bag", type=Path)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--cache", type=Path, help="recording cache .npz for the bag")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    rec = bag.load_recording(args.bag, args.cache)
    fixture = build(rec)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.out, **fixture)
    size = args.out.stat().st_size
    log.info("wrote %s, %d bytes", args.out, size)
    if size > MAX_BYTES:
        log.error("over the %d byte limit for a fixture", MAX_BYTES)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
