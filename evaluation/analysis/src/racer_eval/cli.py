"""Command line.

    python -m racer_eval session --bags <dir> [<dir> ...] --conditions <yaml> --out <dir>
    python -m racer_eval baseline --bag <2026-11-13T22-20-18 dir>

`session` writes laps.csv, interventions.csv, curves.json, decisions.json and summary.md.
`baseline` prints the section 4.2 counts per run and in total next to the hand count the
pre-registration quotes. Neither takes a threshold: the decisions use the pre-registered constants.
"""

from __future__ import annotations

import argparse
import csv
import json
import logging
from collections.abc import Callable, Sequence
from dataclasses import asdict
from pathlib import Path
from typing import Any

from racer_eval.curves import LapCurve
from racer_eval.interventions import (
    RunInterventions,
    count_interventions,
    starts_with_event,
    total,
)
from racer_eval.recording import BagData
from racer_eval.runs import find_runs
from racer_eval.session import SessionResult, analyse_session, load_session

#: The hand count quoted in pre-registration section 4.2 for R0 to R8 of 2026-11-13T22-20-18.
HAND_COUNT = {"watchdog": 8, "rate_limit": 13, "starts_with_event": 7, "starts": 9, "obstacle": 0}

log = logging.getLogger("racer_eval")


def _read(bag: Path) -> BagData:
    from racer_eval.bag_io import read_bag  # needs the `bags` dependency group

    return read_bag(bag)


def baseline_table(data: BagData) -> tuple[list[RunInterventions], RunInterventions]:
    """Section 4.2 counts of every PnP run (status stretch) of a bag, and the total."""
    runs = find_runs(data.draw_t, data.draw_speed, data.st_t)
    rows = [count_interventions(r, data.ev_t, data.ev_phase, data.ev_source) for r in runs]
    return rows, total(rows)


def format_baseline(rows: Sequence[RunInterventions], tot: RunInterventions) -> str:
    head = (
        f"{'run':<6}{'dur_s':>8}{'watchdog':>10}{'rate_limit':>12}{'ttc+obst':>10}{'start<2s':>10}"
    )
    lines = [head]
    for r in rows:
        lines.append(
            f"{r.run_id:<6}{r.duration_s:>8.1f}{r.count('watchdog'):>10}"
            f"{r.count('rate_limit'):>12}{r.obstacle:>10}{'yes' if r.start_event else 'no':>10}"
        )
    with_event, starts = starts_with_event(rows)
    lines.append(
        f"{'total':<6}{tot.duration_s:>8.1f}{tot.count('watchdog'):>10}"
        f"{tot.count('rate_limit'):>12}{tot.obstacle:>10}{f'{with_event}/{starts}':>10}"
    )
    minutes = tot.duration_s / 60.0
    if minutes > 0:
        lines.append(
            f"per minute: watchdog {tot.count('watchdog') / minutes:.2f}, "
            f"rate_limit {tot.count('rate_limit') / minutes:.2f}"
        )
    h = HAND_COUNT
    lines.append(
        f"pre-registration hand count: watchdog {h['watchdog']}, rate_limit {h['rate_limit']}, "
        f"{h['starts_with_event']} of {h['starts']} starts, ttc+obstacle {h['obstacle']}"
    )
    return "\n".join(lines)


def cmd_baseline(args: argparse.Namespace, read: Callable[[Path], BagData] | None = None) -> int:
    rows, tot = baseline_table((read or _read)(Path(args.bag)))
    print(format_baseline(rows, tot))
    return 0


def _write_csv(path: Path, header: list[str], rows: list[list[Any]]) -> None:
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def _curve_dict(c: LapCurve) -> dict[str, Any]:
    return asdict(c)


def write_outputs(result: SessionResult, out: Path) -> None:
    out.mkdir(parents=True, exist_ok=True)
    lap_header = [
        "condition", "run_id", "lap", "camera_time_s", "pnp_time_s", "diff_s",
        "wheel_distance_m", "camera_distance_m", "flag_differs", "flag_missing_camera",
        "flag_missing_pnp",
    ]  # fmt: skip
    _write_csv(
        out / "laps.csv",
        lap_header,
        [[sr.condition, *(row.to_dict()[k] for k in lap_header[1:])] for sr, row in result.laps],
    )
    iv_header = [
        "condition", "run_id", "bag", "duration_s", "watchdog", "rate_limit", "ttc",
        "ttc_reverse", "other", "watchdog_per_min", "rate_limit_per_min", "start_event",
    ]  # fmt: skip
    iv_rows = []
    for sr, iv in result.interventions:
        known = iv.count("watchdog", "rate_limit", "ttc", "ttc_reverse")
        pm = iv.per_minute
        iv_rows.append(
            [
                sr.condition, sr.run_id, sr.bag, f"{iv.duration_s:.3f}", iv.count("watchdog"),
                iv.count("rate_limit"), iv.count("ttc"), iv.count("ttc_reverse"),
                sum(iv.counts.values()) - known, pm.get("watchdog", ""),
                pm.get("rate_limit", ""), iv.start_event,
            ]
        )  # fmt: skip
    _write_csv(out / "interventions.csv", iv_header, iv_rows)
    (out / "curves.json").write_text(
        json.dumps({c: _curve_dict(v) for c, v in result.curves.items()}, indent=2) + "\n"
    )
    (out / "decisions.json").write_text(
        json.dumps({k: v.to_dict() for k, v in result.decisions.items()}, indent=2) + "\n"
    )
    (out / "summary.md").write_text(summary(result))


def summary(result: SessionResult) -> str:
    lines = ["# Session summary", ""]
    for cond, curve in result.curves.items():
        lines.append(
            f"- {cond}: {curve.n_completed} of {curve.n_runs} runs completed "
            f"(Wilson {curve.completion.lo:.2f} to {curve.completion.hi:.2f})"
        )
        if curve.points:
            first, last = curve.points[0], curve.points[-1]
            lines.append(
                f"  lap {first.lap} mean {first.mean:.2f} s (n {first.n}), "
                f"lap {last.lap} mean {last.mean:.2f} s (n {last.n})"
            )
    flagged = [(sr.run_id, row.lap) for sr, row in result.laps if row.flag_differs]
    lines += ["", f"laps where camera and pnp times differ by more than 0.3 s: {flagged or 'none'}"]
    for name, res in result.decisions.items():
        lines += ["", f"## {name}: {res.outcome}", ""]
        lines += [
            f"- {'PASS' if i.passed else 'FAIL'} {i.item_id} {i.description}: "
            f"{i.measured} {i.comparator} {i.threshold}"
            for i in res.items
        ]
    return "\n".join(lines) + "\n"


def cmd_session(args: argparse.Namespace, read: Callable[[Path], BagData] | None = None) -> int:
    doc = load_session(Path(args.conditions))
    bags: dict[str, BagData] = {}
    for d in args.bags:
        path = Path(d)
        bags[path.name] = (read or _read)(path)
    result = analyse_session(doc, bags)
    write_outputs(result, Path(args.out))
    print(summary(result))
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="racer_eval")
    sub = p.add_subparsers(dest="command", required=True)
    s = sub.add_parser("session", help="tables and decisions of one car session")
    s.add_argument("--bags", nargs="+", required=True, help="bag directories")
    s.add_argument("--conditions", required=True, help="session YAML (runs, conditions)")
    s.add_argument("--out", required=True, help="output directory")
    s.set_defaults(func=cmd_session)
    b = sub.add_parser("baseline", help="section 4.2 baseline counts of a bag")
    b.add_argument("--bag", required=True, help="the 2026-11-13T22-20-18 bag directory")
    b.set_defaults(func=cmd_baseline)
    return p


def main(argv: Sequence[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    args = build_parser().parse_args(argv)
    return int(args.func(args))
