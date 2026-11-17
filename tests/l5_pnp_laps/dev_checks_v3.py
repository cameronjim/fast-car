"""Development checks A3, A4 and A6 of brief 50 (lap gate v3): offline replays, no ROS.

    python3 tests/l5_pnp_laps/dev_checks_v3.py a3 --dumps DIR
    python3 tests/l5_pnp_laps/dev_checks_v3.py a4 --bag BAG --bag BAG
    python3 tests/l5_pnp_laps/dev_checks_v3.py a6 --dumps DIR --bag BAG --bag BAG

`a3`: the 13 development dumps of `v0.5-lap3-dev` (the lap node's inputs of B4 runs, DEVELOPMENT
data) through gate v3 and, next to it, through gate v2.1 (the v3 additions off): every lap time
within 0.02 s of v2.1's, every line source `start_scan`, the start alias share at most 0.05 in
every lap. `a4`: the two 2026-11-16 car bags, full replay (`racer_laps.replay`): 20 laps each,
every lap within 0.10 s of v2.1's, none missed. `a6`: the cost of `on_scan` over all scans after
the start scan of those replays, v3 and v2.1 measured the same way: mean at most 12 ms, p99 at most
25 ms. Needs numpy; `a4` and `a6` with bags need the racer_laps `replay` extra (mcap).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
sys.path.insert(0, str(REPO_ROOT / "ros_ws" / "src" / "racer_laps" / "src"))

#: A3, A4, A6 as written in the brief before any code.
A3_MAX_LAP_DIFF_S = 0.02
A3_MAX_ALIAS_SHARE = 0.05
A4_LAPS = 20
A4_MAX_LAP_DIFF_S = 0.10
A6_MEAN_MS = 12.0
A6_P99_MS = 25.0


@dataclass
class LapRow:
    lap_index: int
    t_cross_s: float
    lap_time_s: float
    length_m: float
    line_source: str
    missed: bool
    alias_share: float = 0.0


@dataclass
class Replayed:
    laps: list[LapRow] = field(default_factory=list)
    scan_ms: list[float] = field(default_factory=list)
    status: dict[str, Any] = field(default_factory=dict)


def percentile(values: list[float], q: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * len(ordered)))]


def replay_dump(dump_dir: Path, name: str, v21: bool) -> Replayed:
    """One dump (`DIR/dumps/NAME.npz` with `DIR/NAME.json` beside it) through the engine."""
    import numpy as np
    from racer_laps.engine import LapConfig, LapMemory
    from racer_laps.gate import GateConfig
    from racer_laps.lap_record import STATE_DRIVING
    from racer_laps.scan_match import scan_to_points
    from racer_laps.vehicle import load_live_vehicle

    result = json.loads((dump_dir / f"{name}.json").read_text())
    z = np.load(dump_dir / "dumps" / f"{name}.npz")
    params = result["lap_params"]
    left = float(params["yaw_gain_override"])
    right = float(params["yaw_gain_override_right"])
    vehicle = load_live_vehicle()
    extra: dict[str, Any] = {"ring_enabled": False, "unique_shift_m": 0.0} if v21 else {}
    mem = LapMemory(
        LapConfig(
            lap_source="start_scan",
            gate=GateConfig(
                wheelbase_m=vehicle.wheelbase_m,
                yaw_gain_table=((0.0, left),),
                yaw_gain_right_scale=right / left,
                lidar_x_m=vehicle.mount.x_m,
                lidar_y_m=vehicle.mount.y_m,
                **extra,
            ),
            v_cap_start_mps=float(params["v_cap_start_mps"]),
            v_cap_ceiling_mps=float(params["v_cap_ceiling_mps"]),
        )
    )
    assert mem.gate is not None
    first = result["system_lap_events"][0]
    start = first["t_cross_s"] - first["lap_time_s"]
    scan_t, scan_r, odom, drive = z["scan_t"], z["scan_r"], z["odom"], z["drive"]
    angle_min = -(2.0 * math.pi * 719.0 / 720.0) / 2.0
    angle_inc = 2.0 * math.pi / 720.0
    events = [(t, 2, i) for i, t in enumerate(scan_t)]
    events += [(r[0], 0, i) for i, r in enumerate(odom)]
    events += [(r[0], 1, i) for i, r in enumerate(drive)]
    events.sort()
    out = Replayed()
    started = False
    for _t, kind, i in events:
        if kind == 0:
            mem.on_wheel_odom(float(odom[i][0]), float(odom[i][1]))
        elif kind == 1:
            mem.on_drive(float(drive[i][1]))
        else:
            if not started and scan_t[i] >= start - 1e-3:
                mem.on_planner_status(STATE_DRIVING, 0.3)
                started = True
            pts = scan_to_points(
                np.asarray(scan_r[i], dtype=np.float64),
                angle_min,
                angle_inc,
                0.05,
                16.0,
                vehicle.mount,
                vehicle.body,
            )
            was_started = mem.gate.started
            t0 = time.perf_counter()
            closed = mem.on_scan(float(scan_t[i]), pts)
            if was_started:
                out.scan_ms.append(1000.0 * (time.perf_counter() - t0))
            if closed is not None:
                out.laps.append(
                    LapRow(
                        closed.lap_index,
                        closed.t_cross_s,
                        closed.lap_time_s,
                        closed.length_m,
                        closed.line_source,
                        closed.missed_gate,
                        mem.gate.alias_share_last_lap,
                    )
                )
    out.status = mem.status()
    return out


def replay_bag(bag: Path, v21: bool) -> Replayed:
    from racer_laps.replay import (
        default_start_scan_config,
        read_messages,
        run_replay_start_scan,
    )
    from racer_laps.vehicle import load_live_vehicle

    vehicle = load_live_vehicle()
    messages = list(read_messages(bag))
    res = run_replay_start_scan(messages, vehicle, default_start_scan_config(vehicle, v21=v21))
    out = Replayed(scan_ms=[1000.0 * t for t in res.scan_times_s], status=res.status)
    for lap, share in zip(res.laps, res.alias_share_by_lap, strict=True):
        out.laps.append(
            LapRow(
                lap.lap_index,
                lap.t_cross_s,
                lap.lap_time_s,
                lap.length_m,
                lap.line_source,
                lap.missed_gate,
                share,
            )
        )
    return out


# -- the checks (pure functions, tested) -------------------------------------------------------------


def lap_diffs(a: Replayed, b: Replayed) -> list[float]:
    """Lap time of `a` minus lap time of `b`, lap by lap (inf where one lacks the lap)."""
    n = max(len(a.laps), len(b.laps))
    out = []
    for k in range(n):
        if k < len(a.laps) and k < len(b.laps):
            out.append(a.laps[k].lap_time_s - b.laps[k].lap_time_s)
        else:
            out.append(math.inf)
    return out


def a3_verdict(v3: Replayed, v21: Replayed) -> tuple[bool, dict[str, Any]]:
    diffs = lap_diffs(v3, v21)
    worst = max((abs(d) for d in diffs), default=0.0)
    sources = sorted({lap.line_source for lap in v3.laps})
    share = max((lap.alias_share for lap in v3.laps), default=0.0)
    ok = (
        bool(v3.laps)
        and worst <= A3_MAX_LAP_DIFF_S
        and sources == ["start_scan"]
        and share <= A3_MAX_ALIAS_SHARE
    )
    return ok, {
        "laps": len(v3.laps),
        "laps_v21": len(v21.laps),
        "max_abs_diff_s": worst,
        "sources": sources,
        "max_alias_share": share,
    }


def a4_verdict(v3: Replayed, v21: Replayed) -> tuple[bool, dict[str, Any]]:
    diffs = lap_diffs(v3, v21)
    worst = max((abs(d) for d in diffs), default=0.0)
    missed = sum(1 for lap in v3.laps if lap.missed)
    ok = len(v3.laps) == A4_LAPS and len(v21.laps) == A4_LAPS
    ok = ok and worst <= A4_MAX_LAP_DIFF_S and missed == 0
    return ok, {
        "laps": len(v3.laps),
        "laps_v21": len(v21.laps),
        "max_abs_diff_s": worst,
        "missed": missed,
        "sources": sorted({lap.line_source for lap in v3.laps}),
        "max_alias_share": max((lap.alias_share for lap in v3.laps), default=0.0),
    }


def a6_verdict(runs: list[Replayed]) -> tuple[bool, dict[str, float]]:
    """Cost over all scans after the start scan of every replay together."""
    every = [ms for r in runs for ms in r.scan_ms]
    mean = sum(every) / len(every) if every else math.nan
    p99 = percentile(every, 0.99)
    return bool(every) and mean <= A6_MEAN_MS and p99 <= A6_P99_MS, {
        "scans": float(len(every)),
        "mean_ms": mean,
        "p99_ms": p99,
    }


def _fmt(x: Any) -> str:
    return f"{x:.3f}" if isinstance(x, float) else str(x)


def _table(rows: list[tuple[str, bool, dict[str, Any]]]) -> list[str]:
    if not rows:
        return []
    keys = list(rows[0][2])
    lines = ["| run | " + " | ".join(keys) + " | verdict |", "|---|" + "---|" * (len(keys) + 1)]
    for name, ok, d in rows:
        lines.append(
            f"| {name} | "
            + " | ".join(_fmt(d[k]) for k in keys)
            + f" | {'PASS' if ok else 'FAIL'} |"
        )
    return lines


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("check", choices=["a3", "a4", "a6"])
    parser.add_argument("--dumps", type=Path, help="DIR with NAME.json and dumps/NAME.npz")
    parser.add_argument("--bag", type=Path, action="append", default=[])
    args = parser.parse_args(argv)
    lines: list[str] = []
    ok_all = True
    timed: list[Replayed] = []
    timed_v21: list[Replayed] = []
    if args.check in ("a3", "a6") and args.dumps is not None:
        names = sorted(p.stem for p in (args.dumps / "dumps").glob("*.npz"))
        rows = []
        for name in names:
            v3 = replay_dump(args.dumps, name, v21=False)
            v21 = replay_dump(args.dumps, name, v21=True)
            timed.append(v3)
            timed_v21.append(v21)
            ok, d = a3_verdict(v3, v21)
            rows.append((name, ok, d))
        if args.check == "a3":
            lines += ["A3: dumps through gate v3 against gate v2.1", ""] + _table(rows)
            ok_all = bool(rows) and all(ok for _n, ok, _d in rows)
    if args.check in ("a4", "a6"):
        rows = []
        for bag in args.bag:
            v3 = replay_bag(bag, v21=False)
            v21 = replay_bag(bag, v21=True)
            timed.append(v3)
            timed_v21.append(v21)
            ok, d = a4_verdict(v3, v21)
            rows.append((bag.name, ok, d))
        if args.check == "a4":
            lines += ["A4: car bags through gate v3 against gate v2.1", ""] + _table(rows)
            ok_all = bool(rows) and all(ok for _n, ok, _d in rows)
    if args.check == "a6":
        ok3, d3 = a6_verdict(timed)
        _ok21, d21 = a6_verdict(timed_v21)
        lines += [
            "A6: cost of on_scan over all scans after the start scan",
            "",
            f"gate v3:   {d3}",
            f"gate v2.1: {d21}",
            f"limits: mean {A6_MEAN_MS} ms, p99 {A6_P99_MS} ms",
        ]
        ok_all = ok3
    sys.stdout.write(
        "\n".join(lines) + f"\n\n{args.check.upper()}: {'PASS' if ok_all else 'FAIL'}\n"
    )
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
