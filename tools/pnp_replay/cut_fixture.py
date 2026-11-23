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

A second fixture, tests/fixtures/pnp_fault_2026-11-13.npz (`--fault-fixture`), holds three cuts of
bag 2026-11-13T22-20-18_car_teleop for the fault injection tests, at absolute bag seconds:

    R2  625.5 to 633.5 s    a standing start to 1.0 m/s (the first 8 s of run R2)
    R7  2975.0 to 2981.0 s  the middle of run R7: cw at 1.0 m/s
    R4  2822.0 to 2825.0 s  3 s of run R4 (cw at 0.5 m/s): a pair of scans whose swap gives a
                            scan age of -0.5 ms (finding F2 of tools/tests/test_pnp_replay_faults.py)

A third fixture, tests/fixtures/pnp_bridge_2026-11-22.npz (`--bridge-fixture`, with `--lenient`:
that bag was not finalised), holds four cuts of bag 2026-11-23T03-50-32_car_teleop for the one
frame track bridge (fix checklist 2026-11-22, item S2), at absolute bag seconds (epoch seconds
minus 1795400000 in the checklist: add 5836.43):

    B6266  427.85 to 430.95 s    run 1 (ccw, 1.0 m/s): the outlier track at 430.27, blip at 430.35
    B7079  1075.54 to 1078.64 s  run 4 (cw check): the outlier track at 1077.99, blip at 1078.04
    B7179  1340.15 to 1343.25 s  run 5 (D v0 cw, cap 0.9): the outlier track at 1342.57, blip at
                                 1342.65 (lap 14)
    CLEAN  1250.0 to 1253.1 s    run 5: no inconsistent track frame, the bridge must change nothing

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


FAULT_OUT = REPO_ROOT / "tools" / "tests" / "fixtures" / "pnp_fault_2026-11-13.npz"
FAULT_MAX_BYTES = 450_000
FAULT_CUTS = {"R2": (625.5, 633.5), "R7": (2975.0, 2981.0), "R4": (2822.0, 2825.0)}

BRIDGE_OUT = REPO_ROOT / "tools" / "tests" / "fixtures" / "pnp_bridge_2026-11-22.npz"
BRIDGE_MAX_BYTES = 450_000
BRIDGE_CUTS = {
    "B6266": (427.85, 430.95),
    "B7079": (1075.54, 1078.64),
    "B7179": (1340.15, 1343.25),
    "CLEAN": (1250.0, 1253.1),
}


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


def build_fault(rec: bag.Recording) -> dict[str, np.ndarray]:
    """The fault fixture: FAULT_CUTS of a recording of bag 2026-11-13T22-20-18_car_teleop, at
    absolute bag seconds (the segments of its split file are R2 626.1 and R7 2967.3 onward)."""
    return build_cuts(rec, FAULT_CUTS)


def build_cuts(rec: bag.Recording, cuts: dict[str, tuple[float, float]]) -> dict[str, np.ndarray]:
    """Cuts at absolute bag seconds, each with PREROLL_S before it."""
    out: dict[str, np.ndarray] = {}
    for name, (t_from, t_to) in cuts.items():
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
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument(
        "--fault-fixture", action="store_true", help="cut the 2026-11-13 fault fixture instead"
    )
    ap.add_argument(
        "--bridge-fixture", action="store_true", help="cut the 2026-11-22 track bridge fixture"
    )
    ap.add_argument("--lenient", action="store_true", help="read a bag that was not finalised")
    ap.add_argument("--cache", type=Path, help="recording cache .npz for the bag")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    rec = bag.load_recording(args.bag, args.cache, lenient=args.lenient)
    if args.bridge_fixture:
        fixture, out, limit = build_cuts(rec, BRIDGE_CUTS), BRIDGE_OUT, BRIDGE_MAX_BYTES
    elif args.fault_fixture:
        fixture, out, limit = build_fault(rec), FAULT_OUT, FAULT_MAX_BYTES
    else:
        fixture, out, limit = build(rec), DEFAULT_OUT, MAX_BYTES
    out = args.out or out
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, **fixture)
    size = out.stat().st_size
    log.info("wrote %s, %d bytes", out, size)
    if size > limit:
        log.error("over the %d byte limit for a fixture", limit)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
