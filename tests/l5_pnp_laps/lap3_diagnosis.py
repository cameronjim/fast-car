"""Brief 48 item 5: why the crossing that ends lap 3 was placed about 0.3 m early in three B4
condition c runs (`c_N1_ccw_21`, `c_N1_ccw_22`, `c_N2_cw_22`; `c_N1_ccw_23` is the good control).
Development runs, labelled so: B4's cases and seeds are development data from brief 48 on.

    python3 tests/l5_pnp_laps/lap3_diagnosis.py run --out DIR [--workers 2] [--runs a,b,...]
    python3 tests/l5_pnp_laps/lap3_diagnosis.py table --out DIR [--runs ...]

`run` needs the ROS workspace sourced (the ros-dev container, see battery_b4.py): each run is the
B4 command of its condition c on the fixed harness with `--stop-after-laps 5` and the lap node's
input dump (RUNNER_LAP_DUMP_DIR). `table` needs no ROS: it replays every dump through the lap
memory offline (the same engine and gate as the node) and prints, for every candidate of laps 2 to
4, the along offset `x_m`, the candidate's line estimate minus the truth line (the wheel total at
the ground-truth crossing), the inlier share, the RMS, the free-space violation, which ICP seed won
and whether the previous match was still fresh, then per lap where the lap closed and where the
crossing was placed against the truth. Hypotheses of the brief: (h1) the `past` rule closes on
far candidates before the near ones arrive, (h2) a slid match carried lap long by the seed chain,
(h3) the first lap with a trusted length takes a different path.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import subprocess
import sys
import threading
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
# `table --src DIR` replays through the racer_laps in DIR (an older gate, to see what it did)
_src = [a for a in sys.argv if a.startswith("--src=")]
sys.path.insert(
    0, _src[0][6:] if _src else str(HERE.parents[1] / "ros_ws" / "src" / "racer_laps" / "src")
)

RUNS = {
    "c_N1_ccw_21": ("N1", "ccw", 21),
    "c_N1_ccw_22": ("N1", "ccw", 22),
    "c_N2_cw_22": ("N2", "cw", 22),
    "c_N1_ccw_23": ("N1", "ccw", 23),  # the good control
}
LAPS = 5
FIRST_LAP, LAST_LAP = 2, 4


def run_one(name: str, out: Path, domain: int) -> None:
    import battery_b4 as bb
    import lap_battery_b4 as b4

    const = b4.load_constants()
    case, direction, seed = RUNS[name.split("__")[0]]  # `<run>__r2` repeats a run
    work = out / "work" / name
    work.mkdir(parents=True, exist_ok=True)
    result_path = out / f"{name}.json"
    command = bb.command_for(
        const,
        "c",
        case,
        direction,
        seed,
        domain,
        work,
        result_path,
        bb.DEFAULT_BUNDLE,
        bb.DEFAULT_DATA_ROOT,
        laps=LAPS,
    )
    env = dict(os.environ)
    env["RUNNER_LAP_DUMP_DIR"] = str(out / "dumps")
    (out / "dumps").mkdir(exist_ok=True)
    with (work / "runner.log").open("w") as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, env=env, check=False)
    got = json.loads(result_path.read_text()) if result_path.exists() else {}
    system = [e["lap_time_s"] for e in got.get("system_lap_events", [])]
    truth = got.get("gt_lap_times_registered_s", [])
    diffs = [round(a - b, 2) for a, b in zip(system, truth, strict=False)]
    print(
        f"{name}: {len(system)} system laps, truth_valid {got.get('truth_valid')}, "
        f"system - truth per lap {diffs}, error {got.get('error', '')[:80]!r}",
        flush=True,
    )


def run_cmd(args: argparse.Namespace) -> None:
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    names = args.runs.split(",") if args.runs else list(RUNS)
    todo = [n for n in names if not (out / f"{n}.json").exists()]  # names may carry a `__r2`
    domains = list(range(args.domain_base, args.domain_base + args.workers))
    lock = threading.Lock()

    def worker(domain: int) -> None:
        while True:
            with lock:
                if not todo:
                    return
                name = todo.pop(0)
            run_one(name, out, domain)

    threads = [threading.Thread(target=worker, args=(d,)) for d in domains]
    for t in threads:
        t.start()
    for t in threads:
        t.join()


# -- the offline table -----------------------------------------------------------------------------


def total_at(history: list[tuple[float, float]], t: float) -> float:
    """Wheel total at time t (linear in the recorded history)."""
    import bisect

    times = [h[0] for h in history]
    i = bisect.bisect_left(times, t)
    if i <= 0:
        return history[0][1]
    if i >= len(history):
        return history[-1][1]
    (t0, s0), (t1, s1) = history[i - 1], history[i]
    return s0 + (s1 - s0) * (t - t0) / (t1 - t0) if t1 > t0 else s0


def replay_dump(name: str, out: Path, odom_lag_s: float = 0.0) -> tuple[list[dict], list[dict]]:
    """(candidate rows, lap rows) of one run: the dump through the engine, with the gate's
    candidate and match calls observed."""
    import numpy as np
    import racer_laps.gate as gate_module
    from racer_laps.engine import LapConfig, LapMemory
    from racer_laps.gate import GateConfig
    from racer_laps.lap_record import STATE_DRIVING
    from racer_laps.scan_match import scan_to_points
    from racer_laps.vehicle import load_live_vehicle

    result = json.loads((out / f"{name}.json").read_text())
    dump_dir = out / "dumps"
    if not (dump_dir / f"{name}.npz").exists():
        raise FileNotFoundError(f"no dump for {name} in {dump_dir}")
    z = np.load(dump_dir / f"{name}.npz")
    lap_params = result["lap_params"]
    left = float(lap_params["yaw_gain_override"])
    right = float(lap_params["yaw_gain_override_right"])
    vehicle = load_live_vehicle()
    gate_cfg = GateConfig(
        wheelbase_m=vehicle.wheelbase_m,
        yaw_gain_table=((0.0, left),),
        yaw_gain_right_scale=right / left,
        lidar_x_m=vehicle.mount.x_m,
        lidar_y_m=vehicle.mount.y_m,
    )
    mem = LapMemory(
        LapConfig(
            lap_source="start_scan",
            gate=gate_cfg,
            v_cap_start_mps=float(lap_params["v_cap_start_mps"]),
            v_cap_ceiling_mps=float(lap_params["v_cap_ceiling_mps"]),
        )
    )
    assert mem.gate is not None
    gate = mem.gate
    first = result["system_lap_events"][0]
    start = first["t_cross_s"] - first["lap_time_s"]
    truth = [float(c) for c in result["gt_lap_crossings_registered_s"]]
    scan_t, scan_r, odom, drive = z["scan_t"], z["scan_r"], z["odom"], z["drive"]
    meta = {
        "angle_min": -(2.0 * math.pi * 719.0 / 720.0) / 2.0,
        "angle_inc": 2.0 * math.pi / 720.0,
    }
    history: list[tuple[float, float]] = []
    cand_rows: list[dict] = []
    calls: list[tuple[tuple[float, float, float], object]] = []
    real_match = gate_module.match_scans

    def tagged(source, target, seed=(0.0, 0.0, 0.0), *a, **kw):  # type: ignore[no-untyped-def]
        res = real_match(source, target, seed, *a, **kw)
        calls.append((seed, res))
        return res

    gate_module.match_scans = tagged
    real_consider = gate._consider
    state = {"fresh": False, "now_t": 0.0}

    def consider(t_eff, res, source, now):  # type: ignore[no-untyped-def]
        n0 = len(gate._cands)
        real_consider(t_eff, res, source, now)
        if len(gate._cands) == n0:
            return
        won = next(
            (
                k
                for k, (_s, r) in enumerate(calls)
                if r is not None and (r.x_m, r.y_m, r.yaw_rad) == (res.x_m, res.y_m, res.yaw_rad)
            ),
            -1,
        )
        lap = gate.laps_completed + 1
        line = gate._cands[-1].line_m
        true_line = total_at(history, truth[lap - 1]) if lap - 1 < len(truth) else math.nan
        cand_rows.append(
            {
                "lap": lap,
                "t": t_eff - start,
                "x_m": res.x_m,
                "line_minus_truth_m": line - true_line,
                "inlier": res.inlier_share,
                "rms": res.rms_m,
                "violation": res.freespace_violation,
                "seed": f"{won + 1}/{len(calls)}{'+refine' if won >= 2 and len(calls) > 3 else ''}"
                f"{' fresh' if state['fresh'] else ' stale'}",
                "station": gate.station_m,
            }
        )

    gate._consider = consider  # type: ignore[method-assign]
    events = [(t, 2, i) for i, t in enumerate(scan_t)]
    # `odom_lag_s`: the wheel odometry reaches the node this much later than it was stamped, as it
    # does when a busy host makes the node serve a scan before the odometry that is queued behind it
    events += [(r[0] + odom_lag_s, 0, i) for i, r in enumerate(odom)]
    events += [(r[0], 1, i) for i, r in enumerate(drive)]
    events.sort()
    started = False
    lap_rows: list[dict] = []
    for _t, kind, i in events:
        if kind == 0:
            mem.on_wheel_odom(float(odom[i][0]), float(odom[i][1]))
            history.append((float(odom[i][0]), gate.total_m))
        elif kind == 1:
            mem.on_drive(float(drive[i][1]))
        else:
            if not started and scan_t[i] >= start - 1e-3:
                mem.on_planner_status(STATE_DRIVING, 0.3)
                started = True
            ranges = np.asarray(scan_r[i], dtype=np.float64)
            pts = scan_to_points(
                ranges,
                meta["angle_min"],
                meta["angle_inc"],
                0.05,
                16.0,
                vehicle.mount,
                vehicle.body,
            )
            calls.clear()
            state["fresh"] = gate.started and gate._last_good_age(gate._at(float(scan_t[i]))) <= 1.0
            closed = mem.on_scan(float(scan_t[i]), pts)
            if closed is not None:
                k = closed.lap_index
                line_true = total_at(history, truth[k - 1]) if k - 1 < len(truth) else math.nan
                crossing_total = total_at(history, closed.t_cross_s)
                lap_rows.append(
                    {
                        "lap": k,
                        "length_m": closed.length_m,
                        "crossing_minus_truth_m": crossing_total - line_true,
                        "crossing_minus_truth_s": closed.t_cross_s - truth[k - 1]
                        if k - 1 < len(truth)
                        else math.nan,
                        "system_minus_truth_lap_s": (
                            closed.lap_time_s - result["gt_lap_times_registered_s"][k - 1]
                            if k - 1 < len(result["gt_lap_times_registered_s"])
                            else math.nan
                        ),
                        "decided_at_station_m": closed.length_m
                        + (gate.total_m - total_at(history, closed.t_cross_s)),
                        "n_candidates": closed.n_candidates,
                        "spread_m": closed.spread_m,
                        "missed": closed.missed_gate,
                    }
                )
    gate_module.match_scans = real_match
    return cand_rows, lap_rows


def table_cmd(args: argparse.Namespace) -> None:
    out = Path(args.out)
    names = args.runs.split(",") if args.runs else list(RUNS)
    lines = [
        "# Lap 3 diagnosis (brief 48 item 5): development runs of B4 condition c, fixed harness",
        "",
        (
            "Candidates are STRONG matches inside the consensus window that the gate kept. Line "
            "estimate minus truth line is in metres of wheel distance (positive: the estimate is "
            "late in the lap)."
        ),
    ]
    for name in names:
        try:
            cands, laps = replay_dump(name, out, args.odom_lag_s)
        except FileNotFoundError as error:
            lines += ["", f"## {name}", f"not available: {error}"]
            continue
        lines += ["", f"## {name}", ""]
        lines += [
            "| lap | t s | x_m | line - truth m | inlier | rms | violation | seed | station m |",
            "|---|---|---|---|---|---|---|---|---|",
        ]
        for r in cands:
            if FIRST_LAP <= r["lap"] <= LAST_LAP:
                lines.append(
                    f"| {r['lap']} | {r['t']:.2f} | {r['x_m']:+.2f} | {r['line_minus_truth_m']:+.3f} "
                    f"| {r['inlier']:.2f} | {r['rms']:.3f} | {r['violation']:.3f} | {r['seed']} "
                    f"| {r['station']:.2f} |"
                )
        lines += [
            "",
            (
                "| lap | closed at station m | length m | crossing - truth m | crossing - truth s "
                "| system - truth lap s | candidates | spread m | missed |"
            ),
            "|---|---|---|---|---|---|---|---|---|",
        ]
        for r in laps:
            if r["lap"] <= LAST_LAP + 1:
                lines.append(
                    f"| {r['lap']} | {r['decided_at_station_m']:.2f} | {r['length_m']:.2f} "
                    f"| {r['crossing_minus_truth_m']:+.3f} | {r['crossing_minus_truth_s']:+.3f} "
                    f"| {r['system_minus_truth_lap_s']:+.3f} | {r['n_candidates']} "
                    f"| {r['spread_m']:.2f} | {r['missed']} |"
                )
        by_lap: dict[int, list[float]] = {}
        for r in cands:
            by_lap.setdefault(r["lap"], []).append(r["line_minus_truth_m"])
        lines += [""]
        for k in sorted(by_lap):
            if FIRST_LAP <= k <= LAST_LAP:
                v = by_lap[k]
                lines.append(
                    f"lap {k}: {len(v)} candidates, line - truth median {statistics.median(v):+.3f} m, "
                    f"min {min(v):+.3f}, max {max(v):+.3f}"
                )
    text = "\n".join(lines)
    print(text)
    (out / args.out_name).write_text(text + "\n")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("run", "table"):
        p = sub.add_parser(name)
        p.add_argument("--out", required=True)
        p.add_argument("--runs", default="", help="comma separated names (default: all four)")
        if name == "table":
            p.add_argument("--src", default="", help="use the racer_laps source in this directory")
            p.add_argument("--out-name", default="lap3_table.md")
            p.add_argument(
                "--odom-lag-s",
                type=float,
                default=0.0,
                help="deliver the wheel odometry this long after its stamp (a hypothesis test)",
            )
        if name == "run":
            p.add_argument("--workers", type=int, default=2)
            p.add_argument("--domain-base", type=int, default=140)
    args = parser.parse_args(argv)
    {"run": run_cmd, "table": table_cmd}[args.command](args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
