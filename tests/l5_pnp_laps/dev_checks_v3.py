"""Development checks A3, A4 and A6 of brief 50 (lap gate v3): offline replays, no ROS.

    python3 tests/l5_pnp_laps/dev_checks_v3.py a3 --dumps DIR
    python3 tests/l5_pnp_laps/dev_checks_v3.py a4 --bag BAG --bag BAG
    python3 tests/l5_pnp_laps/dev_checks_v3.py a6 --dumps DIR --bag BAG --bag BAG
    python3 tests/l5_pnp_laps/dev_checks_v3.py m --placed DIR --bag BAG --ring DIR [--ring DIR ...]

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
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
sys.path.insert(
    0,
    os.environ.get("RACER_LAPS_SRC", str(REPO_ROOT / "ros_ws" / "src" / "racer_laps" / "src")),
)

#: A3, A4, A6 as written in the brief before any code.
A3_MAX_LAP_DIFF_S = 0.02
A3_MAX_ALIAS_SHARE = 0.05
A4_LAPS = 20
A4_MAX_LAP_DIFF_S = 0.10
A6_MEAN_MS = 12.0
A6_P99_MS = 25.0
GAP_PLACED_MIN = 0.045  # reported, no rule: a placed track below this, or an RD gap above 0.030
GAP_RD_MAX = 0.030
#: M1, M2, M3 of brief 50d as written before any code.
M1P_MAX_SHARE_A3 = 0.5  # M1' (brief 50e): placed tracks, the share once 3 are ambiguous
M4_MIN_SHARE = 0.9
M4_MAX_LAP_DIFF_S = 0.05
M2_MIN_AMBIGUOUS_SHARE = 0.75
M3_SHIFT = 0.003
M3_MAX_LAP_DIFF_S = 0.05


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


def _dump_path(dump_dir: Path, name: str, result: dict[str, Any]) -> Path:
    """`DIR/dumps/NAME.npz`, or the dump of the accepted attempt (`NAME.attempt<k>.npz`)."""
    plain = dump_dir / "dumps" / f"{name}.npz"
    if plain.exists():
        return plain
    k = int((result.get("battery_b8") or {}).get("accepted_attempt", 1))
    return dump_dir / "dumps" / f"{name}.attempt{k}.npz"


def replay_dump(
    dump_dir: Path, name: str, v21: bool, gate_extra: dict[str, Any] | None = None
) -> Replayed:
    """One dump (`DIR/dumps/NAME.npz` with `DIR/NAME.json` beside it) through the engine."""
    import numpy as np
    from racer_laps.engine import LapConfig, LapMemory
    from racer_laps.gate import GateConfig
    from racer_laps.lap_record import STATE_DRIVING
    from racer_laps.scan_match import scan_to_points
    from racer_laps.vehicle import load_live_vehicle

    result = json.loads((dump_dir / f"{name}.json").read_text())
    z = np.load(_dump_path(dump_dir, name, result))
    params = result["lap_params"]
    left = float(params["yaw_gain_override"])
    right = float(params["yaw_gain_override_right"])
    vehicle = load_live_vehicle()
    extra: dict[str, Any] = {"ring_enabled": False, "unique_shift_m": 0.0} if v21 else {}
    extra.update(gate_extra or {})
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


def replay_bag(bag: Path, v21: bool, gate_extra: dict[str, Any] | None = None) -> Replayed:
    from racer_laps.replay import (
        default_start_scan_config,
        read_messages,
        run_replay_start_scan,
    )
    from racer_laps.vehicle import load_live_vehicle

    vehicle = load_live_vehicle()
    messages = list(read_messages(bag))
    res = run_replay_start_scan(
        messages, vehicle, default_start_scan_config(vehicle, v21=v21, gate_extra=gate_extra)
    )
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


def gap_numbers(r: Replayed) -> dict[str, Any]:
    """Reported with no rule (brief 50c): the smallest and median violation gap of the start-scan
    matches called unique and the largest of those called ambiguous."""
    st = r.status
    return {
        "gap_unique_min": st.get("unique_gap_min", math.nan),
        "gap_unique_median": st.get("unique_gap_median", math.nan),
        "gap_ambiguous_max": st.get("ambiguous_gap_max", math.nan),
    }


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
        **gap_numbers(v3),
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
        **gap_numbers(v3),
    }


def c2_verdict(r: Replayed, truth_laps_s: list[float]) -> tuple[bool, dict[str, Any]]:
    """C2 of brief 50c on one recorded RD dump: every lap by `winding`, none outside the aliased
    tolerance (the larger of 0.3 s and 4 percent of the truth lap), none missed."""
    errors = [lap.lap_time_s - t for lap, t in zip(r.laps, truth_laps_s, strict=False)]
    outside = [
        k + 1
        for k, (e, t) in enumerate(zip(errors, truth_laps_s, strict=False))
        if abs(e) > max(0.3, 0.04 * t)
    ]
    sources = sorted({lap.line_source for lap in r.laps})
    missed = sum(1 for lap in r.laps if lap.missed)
    ok = (
        bool(r.laps)
        and len(r.laps) >= len(truth_laps_s) - 1
        and sources == ["winding"]
        and not outside
        and missed == 0
    )
    return ok, {
        "laps": len(r.laps),
        "sources": sources,
        "outside": outside,
        "missed": missed,
        "largest_error_s": max((abs(e) for e in errors), default=math.nan),
        **gap_numbers(r),
    }


def _gaps(r: Replayed) -> tuple[list[float], list[float], list[float]]:
    st = r.status
    return (
        list(st.get("gaps_unique", [])),
        list(st.get("gaps_unknown", [])),
        list(st.get("gaps_ambiguous", [])),
    )


def _range(values: list[float]) -> str:
    return f"{len(values)} ({min(values):.3f} to {max(values):.3f})" if values else "0"


def gap_classes(r: Replayed) -> dict[str, Any]:
    """Count and range of the start-scan gaps in each of the three classes (50d, no rule)."""
    gu, gk, ga = _gaps(r)
    return {"unique": _range(gu), "unknown": _range(gk), "ambiguous": _range(ga)}


def m1p_verdict(r: Replayed) -> tuple[bool, dict[str, Any]]:
    """M1' (brief 50e), a placed track: never aliased at any scan, at every scan with at least 3
    ambiguous matches the ambiguous share at most `M1P_MAX_SHARE_A3`, every lap `start_scan`, and
    matches were counted at all."""
    st = r.status
    entries = int(st.get("aliased_entries", 0))
    share = float(st.get("share_max_a3", 0.0))
    sources = sorted({lap.line_source for lap in r.laps})
    counted = int(st.get("life_unique", 0)) + int(st.get("life_ambiguous", 0))
    ok = entries == 0 and share <= M1P_MAX_SHARE_A3 and sources == ["start_scan"] and counted > 0
    return ok, {
        "entries": entries,
        "u": int(st.get("life_unique", 0)),
        "a": int(st.get("life_ambiguous", 0)),
        "share_max_when_a>=3": share,
        "sources": sources,
        **gap_classes(r),
    }


def m2_verdict(r: Replayed) -> tuple[bool, dict[str, Any]]:
    """A ring: latched aliased, every lap by `winding` (so latched before the first closure),
    at least three quarters of the counted matches ambiguous."""
    st = r.status
    nu, na = int(st.get("life_unique", 0)), int(st.get("life_ambiguous", 0))
    share = na / (nu + na) if nu + na else 0.0
    sources = sorted({lap.line_source for lap in r.laps})
    ok = (
        st.get("start_verdict") == "aliased"
        and st.get("latched_at_scan") is not None
        and bool(r.laps)
        and sources == ["winding"]
        and share >= M2_MIN_AMBIGUOUS_SHARE
    )
    return ok, {
        "verdict": st.get("start_verdict"),
        "latched_at_scan": st.get("latched_at_scan"),
        "sources": sources,
        "ambiguous_share": share,
        **gap_classes(r),
    }


def m4_verdict(r: Replayed, round4_laps: list[float] | None) -> tuple[bool, dict[str, Any]]:
    """M4 (brief 50e), a ring: M2 plus never released, the ambiguous share over the run at least
    `M4_MIN_SHARE`, every lap time within `M4_MAX_LAP_DIFF_S` of round 4's."""
    st = r.status
    ok2, d2 = m2_verdict(r)
    nu, na = int(st.get("life_unique", 0)), int(st.get("life_ambiguous", 0))
    share = na / (nu + na) if nu + na else 0.0
    releases = int(st.get("aliased_releases", 0))
    worst = math.nan
    laps_ok = False
    if round4_laps is not None and len(round4_laps) == len(r.laps):
        diffs = [abs(lap.lap_time_s - t) for lap, t in zip(r.laps, round4_laps, strict=True)]
        worst = max(diffs, default=0.0)
        laps_ok = worst <= M4_MAX_LAP_DIFF_S
    never_proven = st.get("proven_first_scan") is None
    ok = ok2 and releases == 0 and share >= M4_MIN_SHARE and laps_ok and never_proven
    return ok, {
        "entered_at_scan": st.get("aliased_entered_at_scan"),
        "releases": releases,
        "never_proven": never_proven,
        "share_over_run": share,
        "share_min_since_entry": st.get("share_min_since_entry", math.nan),
        "worst_lap_vs_round4_s": worst,
        "sources": d2["sources"],
    }


def m3_verdict(base: Replayed, shifted: list[Replayed]) -> tuple[bool, dict[str, Any]]:
    """Every shifted replay: the same verdict, the same line source of every lap, lap times
    within `M3_MAX_LAP_DIFF_S` of the defaults."""
    worst = 0.0
    same_verdict = same_sources = True
    for r in shifted:
        same_verdict &= r.status.get("start_verdict") == base.status.get("start_verdict")
        same_sources &= [lap.line_source for lap in r.laps] == [
            lap.line_source for lap in base.laps
        ]
        worst = max([worst, *(abs(d) for d in lap_diffs(r, base))])
    ok = same_verdict and same_sources and worst <= M3_MAX_LAP_DIFF_S
    return ok, {
        "same_verdict": same_verdict,
        "same_sources": same_sources,
        "worst_lap_diff_s": worst,
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


def _default_gaps() -> tuple[float, float]:
    from racer_laps.gate import GateConfig

    cfg = GateConfig(wheelbase_m=0.3)
    return cfg.unique_gap, cfg.alias_gap


def _shift_extra(delta: float) -> dict[str, Any]:
    unique, alias = _default_gaps()
    return {"unique_gap": unique + delta, "alias_gap": alias + delta}


def _replay_any(job: tuple[str, str, str, float, bool]) -> tuple[str, Replayed]:
    """(kind, path, name, shift, round3) -> replay; picklable for the worker pool."""
    kind, path, name, shift, round3 = job
    extra = _shift_extra(shift)
    if round3:
        extra["alias_gap"] = extra["unique_gap"]
    if kind == "bag":
        return name, replay_bag(Path(path), v21=False, gate_extra=extra)
    return name, replay_dump(Path(path), name, v21=False, gate_extra=extra)


def run_m(args: argparse.Namespace) -> tuple[list[str], bool]:
    """M1, M2, M3 on development data (brief 50d), with the later-than-round-3 latch report."""
    import multiprocessing as mp

    placed: list[tuple[str, str, str]] = []  # kind, path, name
    for d in args.placed:
        placed += [("dump", str(d), p.stem) for p in sorted((d / "dumps").glob("*.npz"))]
    placed += [("bag", str(b), b.name) for b in args.bag]
    rings: list[tuple[str, str, str]] = []
    for d in args.ring:
        for p in sorted(d.glob("*.json")):
            if any(f"_{case}_" in p.stem for case in ("RD", "ND", "P4")):
                rings.append(("dump", str(d), p.stem))
    everything = placed + [r for r in rings if r not in placed]
    shifts = [(0.0, False), (-M3_SHIFT, False), (M3_SHIFT, False)]
    jobs = [(k, p, n, sh, r3) for k, p, n in everything for sh, r3 in shifts]
    with mp.Pool(args.workers) as pool:
        done = pool.map(_replay_any, jobs, chunksize=1)
    by_key = {(j[1], j[2], j[3], j[4]): d[1] for j, d in zip(jobs, done, strict=True)}
    lines: list[str] = []
    ok_all = True

    def get(kind_path: tuple[str, str, str], shift: float, r3: bool) -> Replayed:
        return by_key[(kind_path[1], kind_path[2], shift, r3)]

    round4 = json.loads(args.round4_laps.read_text()) if args.round4_laps else {}
    out_laps = {
        item[2]: {
            "laps": [lap.lap_time_s for lap in get(item, 0.0, False).laps],
            "entered": get(item, 0.0, False).status.get(
                "aliased_entered_at_scan", get(item, 0.0, False).status.get("latched_at_scan")
            ),
        }
        for item in everything
    }
    if args.laps_out:
        args.laps_out.write_text(json.dumps(out_laps))
    m1_rows = []
    for item in placed:
        ok, d = m1p_verdict(get(item, 0.0, False))
        m1_rows.append((item[2], ok, d))
    m2_rows = []
    for item in rings:
        base = get(item, 0.0, False)
        ok2, _d2 = m2_verdict(base)
        ref = round4.get(item[2], {})
        ok, d = m4_verdict(base, ref.get("laps"))
        ok = ok and ok2
        a, b = base.status.get("aliased_entered_at_scan"), ref.get("entered")
        d["scans_later_than_round4"] = (a - b) if a is not None and b is not None else "-"
        m2_rows.append((item[2], ok, d))
    m3_rows = []
    for item in everything:
        base = get(item, 0.0, False)
        ok, d = m3_verdict(base, [get(item, -M3_SHIFT, False), get(item, M3_SHIFT, False)])
        m3_rows.append((item[2], ok, d))
    lines += ["M1p: placed tracks, never aliased", ""] + _table(m1_rows) + [""]
    lines += (
        ["M2/M4: rings aliased before the first closure, never released", ""]
        + _table(m2_rows)
        + [""]
    )
    lines += ["M3: both gaps moved by -0.003 and +0.003", ""] + _table(m3_rows)
    for name, rows in (("M1p", m1_rows), ("M2/M4", m2_rows), ("M3", m3_rows)):
        passed = bool(rows) and all(ok for _n, ok, _d in rows)
        ok_all &= passed
        lines += ["", f"{name}: {'PASS' if passed else 'FAIL'}"]
    return lines, ok_all


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("check", choices=["a3", "a4", "a6", "c2", "m"])
    parser.add_argument("--placed", type=Path, action="append", default=[])
    parser.add_argument("--ring", type=Path, action="append", default=[])
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--round4-laps", type=Path, help="JSON NAME -> lap times of round 4")
    parser.add_argument("--laps-out", type=Path, help="write NAME -> lap times of this run")
    parser.add_argument("--dumps", type=Path, help="DIR with NAME.json and dumps/NAME.npz")
    parser.add_argument("--bag", type=Path, action="append", default=[])
    args = parser.parse_args(argv)
    if args.check == "m":
        m_lines, m_ok = run_m(args)
        sys.stdout.write("\n".join(m_lines) + "\n")
        return 0 if m_ok else 1
    lines: list[str] = []
    ok_all = True
    timed: list[Replayed] = []
    timed_v21: list[Replayed] = []
    if args.check == "c2":
        rows = []
        for name in sorted(p.stem for p in args.dumps.glob("*_RD_*.json")):
            doc = json.loads((args.dumps / f"{name}.json").read_text())
            truth = [float(t) for t in doc["gt_lap_times_registered_s"][:12]]
            ok, d = c2_verdict(replay_dump(args.dumps, name, v21=False), truth)
            rows.append((name, ok, d))
        lines += ["C2: the recorded RD dumps through the built gate", ""] + _table(rows)
        ok_all = len(rows) == 12 and all(ok for _n, ok, _d in rows)
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
