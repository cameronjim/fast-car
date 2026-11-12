"""Cut the committed real-scan test fixture out of a bag.

    uv run --group bags python -m racer_pnp_perception.cut_fixture \\
        --bag /Users/cameronjim/code/car-data/bags/2026-11-11T03-54-11_car_teleop \\
        --out ros_ws/src/racer_track_perception/tests/fixtures/real_scans_2026-11-11.npz

60 moving scans evenly spaced over the bag, float32 ranges (about 173 kB) with the scan geometry.
The golden output that goes with it is written by
ros_ws/src/racer_track_perception/tests/regen_golden.py.

With --run START N the fixture of the self-label tests instead: N consecutive scans from scan
START of the bag (the index into the scans of the bag, as selflabel counts them), as uint16
millimetres.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import numpy as np

from racer_pnp_perception.eval_bags import load_bag_scans

N_SCANS = 60


def cut(bag: Path, out: Path, n: int = N_SCANS) -> int:
    scans, mask, _info = load_bag_scans(bag)
    moving = [s for s, m in zip(scans, mask, strict=True) if m]
    pick = np.linspace(0, len(moving) - 1, n).round().astype(int)
    chosen = [moving[i] for i in pick]
    first = chosen[0]
    np.savez(
        out,
        ranges=np.array([s.ranges for s in chosen], dtype=np.float32),
        stamp_s=np.array([s.stamp_s for s in chosen], dtype=np.float64),
        angle_min=np.float64(first.angle_min),
        angle_increment=np.float64(first.angle_increment),
        range_min=np.float64(first.range_min),
        range_max=np.float64(first.range_max),
    )
    return len(chosen)


def cut_run(scans: Any, start: int, n: int, out: Path) -> int:
    """`n` consecutive scans from scan `start` of a `selflabel.ScanSet`, as uint16 millimetres
    (0 for no return) with the scan geometry and stamps, for the self-label tests."""
    sel = slice(start, start + n)
    r = np.asarray(scans.ranges[sel], dtype=np.float64)
    mm = np.where(np.isfinite(r), np.round(r * 1000.0), 0.0)
    np.savez_compressed(
        out,
        ranges_mm=np.clip(mm, 0, 65535).astype(np.uint16),
        stamp_s=np.asarray(scans.stamp_s[sel], dtype=np.float64),
        angle_min=np.float64(scans.angle_min),
        angle_increment=np.float64(scans.angle_increment),
        range_min=np.float64(scans.range_min),
        range_max=np.float64(scans.range_max),
    )
    return n


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--run",
        nargs=2,
        type=int,
        metavar=("START", "N"),
        help="instead: N consecutive scans from scan START (uint16 mm, for the self-label tests)",
    )
    parser.add_argument("--cache", type=Path, default=None, help="selflabel scan cache directory")
    args = parser.parse_args(argv)
    if args.run:
        from racer_pnp_perception.selflabel import load_scanset

        scans, _t, _v = load_scanset(args.bag, args.cache)
        n = cut_run(scans, args.run[0], args.run[1], args.out)
    else:
        n = cut(args.bag, args.out)
    print(f"{args.out}: {n} scans, {args.out.stat().st_size} bytes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
