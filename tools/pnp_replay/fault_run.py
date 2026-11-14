"""Run fault-injected replays through both consumers, and the full-bag command.

    cd tools && uv run python -m pnp_replay.fault_run <bag dir | .mcap | recording .npz> \\
        --dynamics-bundle ../config/pnp/fixtures/dyn-car-2026-11-12 \\
        [--segment R0 R1 ...] [--seed 20261113] [--windows 3] [--out data/pnp_replay/faults.json]

For every PnP run (segment) of the bag, every case of `CASES` at `--windows` seeded random
windows inside the run: the perception consumer (`TrackPerceiver.process` over the mutated scans)
and the planner consumer (`planner_replay_cli` on the mutated stream), each checked with
pnp_replay.fault_checks. The JSON has one row per (run, case, window) with the rule violations
found, and the table on stdout counts the violations per case. Open loop: the scans do not react to
the commands (docs/notes/pnp-replay.md). Exit status 1 when any rule is violated.

The pytest suite (tools/tests/test_pnp_replay_faults.py) runs every case once on the committed
fixture; this is the same code at scale on a whole bag.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from pnp_replay import bag, fault_checks, faults, perception, planner_stream, segments, vehicle

log = logging.getLogger(__name__)

# Deterministic: the sample budget halves a slow solve, and a loaded machine has slow solves.
# Open loop: the recorded wheel does not follow the replayed commands, so the wheel stall detector
# is set out of reach (docs/notes/pnp-replay.md).
REPLAY_PARAMS: dict[str, float | bool] = {"solve_halve_over_ms": 200.0, "stall_hold_s": 600.0}
SETTLE_BEFORE_S = 1.0  # a fault starts at least this long after the replay's first tick
RECOVER_AFTER_S = 1.5  # and ends at least this long before its last


@dataclass(frozen=True)
class Case:
    """One fault on one stream. `length` is seconds, or ("scans", n) for a window holding exactly
    n scans. `cut` and `start_s` (seconds after the cut's first tick) place it on the fixture."""

    id: str
    fault: str
    target: str  # "scan", "odom" or "track"
    length: float | tuple[str, int]
    params: dict[str, Any] = field(default_factory=dict)
    cut: str = "R7"
    start_s: float = 2.0

    def window(self, rec: bag.Recording, t_start: float) -> tuple[float, float]:
        if isinstance(self.length, tuple):
            return faults.window_of_scans(rec, t_start, int(self.length[1]))
        if self.target == "odom":
            return faults.window_of_odom(rec, t_start, float(self.length))
        return t_start, t_start + float(self.length)

    @property
    def max_length_s(self) -> float:
        return 0.6 if isinstance(self.length, tuple) else float(self.length)


def _c(id_: str, fault: str, target: str, length: Any, cut: str = "R7", start_s: float = 2.0,
       **params: Any) -> Case:  # fmt: skip
    return Case(id_, fault, target, length, dict(params), cut, start_s)


CASES: tuple[Case, ...] = (
    # scan
    _c("scan-nan_ranges-10-nan", "nan_ranges", "scan", 1.0, fraction=0.1, fill="nan"),
    _c("scan-nan_ranges-50-nan", "nan_ranges", "scan", 1.0, fraction=0.5, fill="nan"),
    _c("scan-nan_ranges-100-nan", "nan_ranges", "scan", 1.0, fraction=1.0, fill="nan"),
    _c("scan-nan_ranges-100-inf", "nan_ranges", "scan", 1.0, fraction=1.0, fill="inf"),
    _c("scan-nan_ranges-100-zero", "nan_ranges", "scan", 1.0, fraction=1.0, fill="zero"),
    _c("scan-drop-1scan", "drop", "scan", ("scans", 1)),
    _c("scan-drop-3scans", "drop", "scan", ("scans", 3)),
    _c("scan-drop-1.0s", "drop", "scan", 1.0),
    _c("scan-drop-1.0s-start", "drop", "scan", 1.0, cut="R2", start_s=0.5),
    _c("scan-stamp_jump_forward-0.3s", "stamp_jump_forward", "scan", 0.3),
    _c("scan-stamp_jump_forward-1.0s", "stamp_jump_forward", "scan", 1.0),
    _c("scan-stamp_jump_backward-0.3s", "stamp_jump_backward", "scan", 0.3),
    _c("scan-stamp_jump_backward-1.0s", "stamp_jump_backward", "scan", 1.0),
    _c("scan-out_of_order-1.0s", "out_of_order", "scan", 1.0),
    _c("scan-frozen-0.3s", "frozen", "scan", 0.3),
    _c("scan-frozen-1.0s", "frozen", "scan", 1.0),
    _c("scan-frozen_fresh_stamp-1.0s", "frozen_fresh_stamp", "scan", 1.0),
    # wheel odometry
    _c("odom-nan_ranges-0.06s", "nan_ranges", "odom", 0.06),
    _c("odom-nan_ranges-0.15s", "nan_ranges", "odom", 0.15),
    _c("odom-nan_ranges-1.0s", "nan_ranges", "odom", 1.0),
    _c("odom-drop-0.06s", "drop", "odom", 0.06),
    _c("odom-drop-0.15s", "drop", "odom", 0.15),
    _c("odom-drop-1.0s", "drop", "odom", 1.0),
    _c("odom-drop-1.0s-start", "drop", "odom", 1.0, cut="R2", start_s=0.5),
    _c("odom-stamp_jump_forward-1.0s", "stamp_jump_forward", "odom", 1.0),
    _c("odom-stamp_jump_backward-1.0s", "stamp_jump_backward", "odom", 1.0),
    _c("odom-out_of_order-1.0s", "out_of_order", "odom", 1.0),
    _c("odom-frozen-1.0s", "frozen", "odom", 1.0),
    _c("odom-frozen_fresh_stamp-1.0s", "frozen_fresh_stamp", "odom", 1.0),
    # the recorded local track
    _c("track-track_garbage-nan", "track_garbage", "track", 1.0, kind="nan"),
    _c("track-track_garbage-valid_out_of_range", "track_garbage", "track", 1.0,
       kind="valid_out_of_range"),
)  # fmt: skip
SCAN_CASES = tuple(c for c in CASES if c.target == "scan")
CASES_BY_ID = {c.id: c for c in CASES}


def inject(rec: bag.Recording, case: Case, t0: float, t1: float, seed: int) -> bag.Recording:
    """The recording with the case's fault in [t0, t1] (arrival times)."""
    if case.target == "track":
        return faults.track_garbage(rec, t0, t1, seed, **case.params)
    return faults.apply(rec, case.fault, t0, t1, seed, targets=(case.target,), **case.params)


def fault_record(rec: bag.Recording, case: Case, t0: float, t1: float) -> fault_checks.Fault:
    """The rule-engine's record of the injection, from the UNMUTATED recording."""
    extra = {}
    if case.fault == "nan_ranges" and case.target == "scan":
        extra = {"fraction": float(case.params["fraction"]), "fill": str(case.params["fill"])}
    return fault_checks.fault_for(
        rec["scan_t"], rec["odom_t"], rec["trk_t"], case.fault, case.target, t0, t1, **extra
    )


def perceive(
    perceiver: perception.Perceiver, rec: bag.Recording, t_from: float, t_to: float
) -> tuple[list[Any], list[bool], int]:
    """`TrackPerceiver.process` over the scans of the (mutated) recording that arrive in the
    window, in arrival order, as the node would: the tracks, which scans had no finite beam in
    range, and how many scans made the perceiver log an exception it then swallowed."""
    caught = _ExceptionCounter()
    plog = logging.getLogger("racer_track_perception.core")
    plog.addHandler(caught)
    try:
        tracks, empty = [], []
        for i in planner_stream.scan_indices(rec, t_from, t_to):
            r = bag.scan_ranges(rec, int(i))
            lo, hi = float(rec["scan_range_min"][i]), float(rec["scan_range_max"][i])
            with np.errstate(invalid="ignore"):
                usable = np.isfinite(r) & (r >= lo) & (r <= hi)
            empty.append(not bool(usable.any()))
            tracks.append(
                perceiver.impl.process(
                    r,
                    float(rec["scan_angle_min"][i]),
                    float(rec["scan_angle_increment"][i]),
                    lo,
                    hi,
                )
            )
    finally:
        plog.removeHandler(caught)
    return tracks, empty, caught.count


class _ExceptionCounter(logging.Handler):
    def __init__(self) -> None:
        super().__init__(level=logging.ERROR)
        self.count = 0

    def emit(self, record: logging.LogRecord) -> None:
        self.count += 1


@dataclass(frozen=True)
class PlannerSetup:
    """What a planner run needs besides the stream."""

    cli: planner_stream.Cli
    bundle: Path
    v_max_mps: float
    limits: fault_checks.PlannerLimits
    params: dict[str, float | bool]


def planner_setup(
    cli: planner_stream.Cli,
    bundle: Path,
    v_max_mps: float,
    params: dict[str, float | bool] | None = None,
) -> PlannerSetup:
    p = {**REPLAY_PARAMS, **(params or {})}
    describe = planner_stream.describe_cli(cli, bundle, v_max_mps, p)
    floor = fault_checks.speed_floor_from_bundle(bundle, describe)
    lim = fault_checks.PlannerLimits.from_config(describe, floor, vehicle.load_vehicle_limits())
    return PlannerSetup(cli, bundle, v_max_mps, lim, p)


def run_planner(
    setup: PlannerSetup,
    rec: bag.Recording,
    t_from: float,
    t_to: float,
    work: Path,
    name: str,
    tracks_mode: str = "recorded",
) -> dict[str, Any]:
    """The planner CSV of `rec` over [t_from, t_to] (ticks every 0.02 s)."""
    lines = planner_stream.stream_lines(rec, t_from, t_to, tracks_mode=tracks_mode)
    stream = work / f"{name}.stream"
    planner_stream.write_stream(lines, stream)
    rows = planner_stream.run_cli(
        setup.cli, setup.bundle, setup.v_max_mps, setup.params, stream, work / f"{name}.csv"
    )
    stream.unlink(missing_ok=True)
    return rows


def window_start(rec: bag.Recording, a: float, case: Case) -> float:
    return a + case.start_s


def random_starts(rng: np.random.Generator, a: float, b: float, case: Case, n: int) -> list[float]:
    """n window starts, uniform over the part of [a, b] that leaves SETTLE_BEFORE_S before and
    the window plus RECOVER_AFTER_S after."""
    lo = a + SETTLE_BEFORE_S
    hi = b - RECOVER_AFTER_S - case.max_length_s
    if hi <= lo:
        return []
    return sorted(float(x) for x in rng.uniform(lo, hi, size=n))


def _json_safe(x: Any) -> Any:
    if isinstance(x, dict):
        return {k: _json_safe(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_json_safe(v) for v in x]
    if isinstance(x, (np.floating, float)):
        return None if not np.isfinite(x) else float(x)
    if isinstance(x, (np.integer,)):
        return int(x)
    return x


def run_bag(args: argparse.Namespace) -> int:
    out = args.out or (Path("data") / "pnp_replay" / f"{args.bag.name}.faults.json")
    cache = args.cache or out.parent / f"{args.bag.name}.recording.npz"
    rec_all = bag.load_recording(args.bag, cache)
    found = segments.find_segments(rec_all["st_t"], rec_all["odom_t"], rec_all["odom_v"])
    chosen = segments.select(found, args.segment)
    chosen = [s for s in chosen if s.kind == "pnp"]
    cli = planner_stream.find_cli(args.cli)
    perceiver = perception.build_perceiver("geometric")
    rng = np.random.default_rng(args.seed)
    rows_out: list[dict[str, Any]] = []
    out.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="pnp_faults_", dir=out.parent) as tmp:
        work = Path(tmp)
        for seg in chosen:
            v_max = args.v_max_segment.get(seg.name, args.v_max)
            setup = planner_setup(cli, args.dynamics_bundle, v_max)
            a, b = seg.t_from, seg.t_to
            rec = bag.cut(rec_all, a - planner_stream.PREROLL_S, b)
            base = run_planner(setup, rec, a, b, work, f"{seg.name}.base")
            base_bad = fault_checks.always_violations(base, setup.limits)
            if base_bad:
                log.warning("%s: the UNFAULTED replay breaks the always rules: %s", seg.name,
                            base_bad[0])  # fmt: skip
            for case in CASES:
                if case.id.endswith("-start") or (
                    case.target == "track" and not (rec["trk_t"] >= a).any()
                ):
                    continue  # "-start" cases are fixed windows of the fixture's R2
                for k, t_start in enumerate(random_starts(rng, a, b, case, args.windows)):
                    t0, t1 = case.window(rec, t_start)
                    seed = int(rng.integers(0, 2**31 - 1))
                    mutated = inject(rec, case, t0, t1, seed)
                    record = fault_record(rec, case, t0, t1)
                    row: dict[str, Any] = {
                        "segment": seg.name, "case": case.id, "window": k, "t0": t0, "t1": t1,
                        "seed": seed,
                    }  # fmt: skip
                    if case.target == "scan":
                        tracks, empty, swallowed = perceive(
                            perceiver, mutated, a - planner_stream.PREROLL_S, b
                        )
                        row["perception_violations"] = fault_checks.perception_violations(
                            tracks, empty
                        )
                        row["perception_swallowed_exceptions"] = swallowed
                    planned = run_planner(setup, mutated, a, b, work, f"{seg.name}.{case.id}")
                    row["planner_violations"] = fault_checks.planner_violations(
                        planned, record, setup.limits, base, baseline_speed_slack=True
                    )
                    row["planner_states"] = {
                        str(s): int(n)
                        for s, n in zip(
                            *np.unique(planned["state"], return_counts=True), strict=True
                        )
                    }
                    rows_out.append(row)
                    log.info("%s %s w%d: %s", seg.name, case.id, k,
                             "ok" if not (row["planner_violations"] or
                                          row.get("perception_violations")) else "VIOLATION")  # fmt: skip
    result = {
        "tool": "pnp_replay.fault_run",
        "bag": args.bag.name,
        "seed": args.seed,
        "windows_per_case": args.windows,
        "dynamics_bundle": args.dynamics_bundle.name,
        "rows": rows_out,
    }
    out.write_text(json.dumps(_json_safe(result), indent=1) + "\n", encoding="utf-8")
    bad = [r for r in rows_out if r["planner_violations"] or r.get("perception_violations")]
    print(f"{len(rows_out)} injections, {len(bad)} with a violation; wrote {out}")
    tally: dict[str, list[int]] = {}
    for r in rows_out:
        t = tally.setdefault(r["case"], [0, 0, 0])
        t[0] += 1
        t[1] += bool(r["planner_violations"])
        t[2] += bool(r.get("perception_violations"))
    print(f"{'case':44s} {'runs':>5s} {'planner':>8s} {'percept.':>8s}")
    for case_id, (n, p, q) in tally.items():
        print(f"{case_id:44s} {n:5d} {p:8d} {q:8d}")
    for r in bad[:20]:
        print(f"  {r['segment']} {r['case']} w{r['window']} t={r['t0']:.1f}-{r['t1']:.1f}: "
              f"{(r['planner_violations'] + r.get('perception_violations', []))[0]}")  # fmt: skip
    return 1 if bad else 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("bag", type=Path)
    ap.add_argument("--dynamics-bundle", type=Path, required=True)
    ap.add_argument("--segment", nargs="+", metavar="NAME", help="PnP runs (default: all)")
    ap.add_argument("--v-max", type=float, default=0.5, help="planner cap (m/s)")
    ap.add_argument("--v-max-segment", nargs="*", default=[], metavar="SEG=M_S")
    ap.add_argument("--seed", type=int, default=20261113)
    ap.add_argument("--windows", type=int, default=3, help="seeded windows per run and case")
    ap.add_argument("--cli")
    ap.add_argument("--out", type=Path)
    ap.add_argument("--cache", type=Path)
    args = ap.parse_args(argv)
    args.v_max_segment = {
        k: float(v) for k, _, v in (item.partition("=") for item in args.v_max_segment)
    }
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    try:
        return run_bag(args)
    except (ValueError, planner_stream.ReplayError, FileNotFoundError) as error:
        print(f"fault_run: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
