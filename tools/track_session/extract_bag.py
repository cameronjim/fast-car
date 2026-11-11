#!/usr/bin/env python3
"""Step 0 of a track session when the mapping run is a recorded bag: pull the topics the
offline mapper needs out of an mcap bag into one npz, with no ROS install.

    uv run --project tools python tools/track_session/extract_bag.py <bag dir or .mcap> <out.npz>

Reads /scan, /drive, /drive_raw, /odom/wheel and /safety/events with tools/follower_replay/bag.py,
which copes with the truncated, unindexed mcap files a killed recorder leaves behind. The npz is
what `bag_feeder.py` replays into the real mapping stack (see docs/notes/first-boot-runbook.md,
"New track session").
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from follower_replay import bag as bag_reader


def extract(bag: Path, out: Path) -> dict[str, int]:
    data = bag_reader.read_bag(bag)
    if not data.scans:
        raise SystemExit(f"{bag}: no /scan messages")
    first = data.scans[0]
    np.savez_compressed(
        out,
        scan_t=np.array([s.stamp_s for s in data.scans]),
        scan_ranges=np.array([s.ranges for s in data.scans], dtype=np.float32),
        scan_meta=np.array(
            [first.angle_min, first.angle_increment, first.range_min, first.range_max]
        ),
        drive_t=np.array([r.stamp_s for r in data.drive]),
        drive_steer=np.array([r.steering_rad for r in data.drive]),
        drive_speed=np.array([r.speed_mps for r in data.drive]),
        raw_t=np.array([r.stamp_s for r in data.drive_raw]),
        raw_steer=np.array([r.steering_rad for r in data.drive_raw]),
        raw_speed=np.array([r.speed_mps for r in data.drive_raw]),
        odom_t=np.array([r.stamp_s for r in data.odom]),
        odom_v=np.array([r.speed_mps for r in data.odom]),
        ev_t=np.array([e.stamp_s for e in data.events]),
        ev_sev=np.array([e.severity for e in data.events]),
        ev_phase=np.array([e.phase for e in data.events]),
    )
    return {"scans": len(data.scans), "odom": len(data.odom), "drive": len(data.drive)}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("out", type=Path)
    args = parser.parse_args(argv)
    counts = extract(args.bag, args.out)
    print(f"{args.out}: {counts}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
