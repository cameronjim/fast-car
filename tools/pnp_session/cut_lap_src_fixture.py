"""Cut the L4 fixtures of `racer_laps.external_replay` out of the two DEVELOPMENT bags (brief 51).

    cd tools && uv run python -m pnp_session.cut_lap_src_fixture \\
        --bag /path/2026-11-16T22-38-47_car_teleop --events 6 \\
        --out ../ros_ws/src/racer_laps/tests/fixtures_camera/car_2026_11_16_22_38_47_laps_1_6.npz
    cd tools && uv run python -m pnp_session.cut_lap_src_fixture \\
        --bag /path/2026-11-16T02-18-54_car_teleop --events 7 \\
        --out ../ros_ws/src/racer_laps/tests/fixtures_camera/car_2026_11_16_02_18_54_events_1_7.npz

Each fixture is a `ReplayStreams` npz: the wheel odometry (arrival and header stamp), `/drive`,
the planner status, the safety events, the camera's `/lap/event` and `/lap/status`, each with its
log time, cut from one second before the planner's first DRIVING report to one second after the
arrival of the N-th camera event, on the bag axis (seconds after the log time of the first
`/scan`). Both bags are development data. The tool refuses an evaluation bag of D9-B0.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from pnp_session import lap_src

MARGIN_S = 1.0


def cut(bag: Path, events: int, out: Path) -> dict[str, int]:
    th = lap_src.load_prereg()
    if bag.name in lap_src.evaluation_bags(th):
        raise lap_src.FreezeRequired(f"{bag.name} is an evaluation bag: not cut into a fixture")
    data = lap_src.read_bag(bag)
    s = data.streams
    driving = s.planner_arrival[s.planner_state == 1]
    if not len(driving):
        raise ValueError(f"{bag}: no DRIVING planner status")
    if len(s.event_arrival) < events:
        raise ValueError(f"{bag}: {len(s.event_arrival)} camera events, {events} wanted")
    t0 = float(driving[0]) - MARGIN_S
    t1 = float(s.event_arrival[events - 1]) + MARGIN_S
    w = s.window(t0, t1)
    out.parent.mkdir(parents=True, exist_ok=True)
    w.save(out)
    return {
        "odom": len(w.odom_arrival),
        "drive": len(w.drive_arrival),
        "planner": len(w.planner_arrival),
        "safety": len(w.safety_arrival),
        "events": len(w.event_arrival),
        "status": len(w.status_arrival),
        "bytes": int(np.int64(out.stat().st_size)),
    }


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--bag", required=True, type=Path)
    p.add_argument("--events", required=True, type=int, help="cut after this many camera events")
    p.add_argument("--out", required=True, type=Path)
    args = p.parse_args(argv)
    print(cut(args.bag, args.events, args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
