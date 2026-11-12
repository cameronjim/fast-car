#!/usr/bin/env python3
"""Auto-tune harness for gap_follow's race profile.

Needs the ROS workspace sourced (tools/race_tune_docker.sh runs this inside the ros-dev image,
with no network). The selection rule is in tools/race_tuning/selection.py and was fixed before
the search was run; docs/notes/gap-follow-race-2026-11-12.md quotes it.

    # sample and refine parameter sets on the TUNING tracks (both directions), rank by the rule
    tools/race_tune_docker.sh search --out /data/runs/race/search --samples 24 --refine 2 --jobs 5

    # the HELD-OUT tracks: the baseline and any number of named variants
    tools/race_tune_docker.sh evaluate --out /data/runs/race/heldout --laps 3 --jobs 5 \\
        --variant 'race={"profile": "race-2026-11-12", "params": {}}'

    # the real bags: would the race profile have commanded anything crazy?
    tools/race_tune_docker.sh replay --bags /data/bags --profile race-2026-11-12

    # scan-to-command latency and CPU per scan on a real bag's scans (timer vs compute_on_scan)
    tools/race_tune_docker.sh latency --bag /data/bags/<bag> --seconds 40

Every run is one JSON file under --out, and a run whose file exists is skipped, so a killed sweep
resumes. The baseline is always the recommended profile before this work: lane-2026-10-25 with
lane_lookahead_m 0.8, outside_entry_offset_m 0.25, max_speed_mps 0.64.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests" / "l5_reactive_bench"))

from race_tuning import report, runs, selection, selection_gate, space, tracks

BASELINE_NAME = "baseline"


def _progress(spec: runs.RunSpec, result: dict[str, Any]) -> None:
    status = (
        "ERR"
        if "error" in result
        else f"{result.get('laps')} laps {result.get('lap_time_s')}s "
        f"{result.get('obstacle_brakes')}b {result.get('wall_contacts')}c"
    )
    print(f"  {spec.variant} {spec.run_id}: {status}", flush=True)


def _records(out: Path, specs: list[runs.RunSpec]) -> list[selection.RunRecord]:
    return runs.load_records(out, specs)


def search(args: argparse.Namespace) -> int:
    out = Path(args.out)
    tuning = tracks.tuning_tracks()
    baseline_specs = runs.make_specs(BASELINE_NAME, space.BASELINE, tuning, args.laps)
    candidates: dict[str, dict[str, Any]] = {
        space.candidate_id(p): space.with_base(p)
        for p in space.sample_candidates(args.samples, args.seed)
    }
    print(f"{len(tuning)} tuning tracks x 2 directions, {len(candidates)} candidates", flush=True)
    all_specs = list(baseline_specs)
    for cid, params in candidates.items():
        all_specs += runs.make_specs(cid, params, tuning, args.laps)
    runs.execute(all_specs, out, args.jobs, args.domain_base, _progress)
    for round_number in range(args.refine):
        ranking = _ranking(out, tuning, candidates, args.laps)
        seeds = [params for _, params, _ in ranking[:2]]
        fresh: dict[str, dict[str, Any]] = {}
        for params in seeds:
            plain = {k: v for k, v in params.items() if k in space.SPACE}
            for neighbour in space.neighbours(plain):
                cid = space.candidate_id(neighbour)
                if cid not in candidates and cid not in fresh:
                    fresh[cid] = space.with_base(neighbour)
        if not fresh:
            break
        print(f"refine round {round_number + 1}: {len(fresh)} neighbours", flush=True)
        specs = []
        for cid, params in fresh.items():
            specs += runs.make_specs(cid, params, tuning, args.laps)
        runs.execute(specs, out, args.jobs, args.domain_base, _progress)
        candidates.update(fresh)
    return _summary(out, tuning, candidates, args.laps)


def _baseline_aggregate(out: Path, tuning, laps: int):
    base_records = _records(out, runs.make_specs(BASELINE_NAME, space.BASELINE, tuning, laps))
    return base_records, selection.aggregate(base_records)


def _ranking(out: Path, tuning, candidates, laps: int):
    base_records, base_agg = _baseline_aggregate(out, tuning, laps)
    scored = {}
    for cid, params in candidates.items():
        recs = _records(out, runs.make_specs(cid, params, tuning, laps))
        if recs:
            scored[cid] = (params, selection.aggregate(recs, base_records))
    return selection.rank(scored, base_agg)


def _summary(out: Path, tuning, candidates, laps: int) -> int:
    base_records, base_agg = _baseline_aggregate(out, tuning, laps)
    rows: dict[str, Any] = {}
    scored = {}
    for cid, params in candidates.items():
        recs = _records(out, runs.make_specs(cid, params, tuning, laps))
        if not recs:
            continue
        agg = selection.aggregate(recs, base_records)
        ok, why = selection.admissible(agg, base_agg)
        scored[cid] = (params, agg)
        rows[cid] = {"params": params, "aggregate": agg.__dict__, "admissible": ok, "why": why}
    ranking = selection.rank(scored, base_agg)
    summary = {
        "baseline": base_agg.__dict__,
        "tuning_tracks": [t.name for t in tuning],
        "laps": laps,
        "candidates": rows,
        "ranking": [cid for cid, _, _ in ranking],
        "winner": ranking[0][0] if ranking else None,
        "winner_params": ranking[0][1] if ranking else None,
    }
    (out / "search_summary.json").write_text(json.dumps(summary, indent=2, default=str) + "\n")
    print(f"baseline: {base_agg}")
    for place, (cid, _params, agg) in enumerate(ranking[:10], start=1):
        print(f"{place:2d}. {cid}  ratio {agg.mean_ratio:.3f}  brakes/lap {agg.brakes_per_lap:.2f}")
    if not ranking:
        print("NO ADMISSIBLE CANDIDATE (see search_summary.json for why)")
    return 0


def select_cmd(args: argparse.Namespace) -> int:
    data = json.loads((Path(args.out) / "search_summary.json").read_text())
    print(json.dumps({"winner": data["winner"], "params": data["winner_params"]}, indent=2))
    return 0


def evaluate(args: argparse.Namespace) -> int:
    out = Path(args.out)
    held_out = tracks.family_tracks(args.family, args.tuning_tracks, args.tracks)
    variants: dict[str, dict[str, Any]] = {
        BASELINE_NAME: {"params": space.BASELINE, "profile": runs.DEFAULT_PROFILE}
    }
    for text in args.variant:
        name, _, spec = text.partition("=")
        variants[name] = json.loads(spec)
    for name in list(variants):
        for repeat in range(2, args.repeats + 1):
            variants[f"{name}_r{repeat}"] = variants[name]
    specs: dict[str, list[runs.RunSpec]] = {
        name: runs.make_specs(
            name,
            spec.get("params", {}),
            held_out,
            args.laps,
            spec.get("profile", runs.DEFAULT_PROFILE),
        )
        for name, spec in variants.items()
    }
    runs.execute(
        [s for ss in specs.values() for s in ss], out, args.jobs, args.domain_base, _progress
    )
    records = {name: _records(out, ss) for name, ss in specs.items()}
    first_repeat = {n: r for n, r in records.items() if not re.search(r"_r\d+$", n)}
    if args.repeats > 1:
        records = report.pool(records)
    text = (
        report.comparison_table(records, BASELINE_NAME)
        + "\n\n(per track: the first repeat)\n\n"
        + report.per_track_table(first_repeat)
    )
    (out / "report.md").write_text(text + "\n")
    print(text)
    base_agg = selection.aggregate(records[BASELINE_NAME])
    scored: dict[str, tuple[dict[str, Any], Any]] = {}
    for name, recs in records.items():
        if name == BASELINE_NAME:
            continue
        agg = selection.aggregate(recs, records[BASELINE_NAME])
        if args.rule == "gate":
            ok, why = selection_gate.admissible(agg)
            scored[name] = (variants[name].get("params", {}), agg)
        else:
            ok, why = selection.admissible(agg, base_agg)
        print(f"{name}: {'ADMISSIBLE' if ok else 'NOT admissible: ' + '; '.join(why)}")
    if args.rule == "gate":
        ranking = selection_gate.rank(scored)
        print("ranking under tools/race_tuning/selection_gate.py:")
        for place, (name, _p, agg) in enumerate(ranking, start=1):
            print(
                f"{place:2d}. {name}  ratio {agg.mean_ratio:.3f}  brakes/lap "
                f"{agg.brakes_per_lap:.2f}  escapes/lap {agg.escapes_per_lap:.2f}"
            )
        if not ranking:
            print("NO ADMISSIBLE CANDIDATE")
    return 0


def replay_cmd(args: argparse.Namespace) -> int:
    from race_tuning import replay_check

    return replay_check.main(args)


def main(argv: list[str] | None = None) -> int:
    raw = sys.argv[1:] if argv is None else argv
    if raw and raw[0] == "latency":
        # Scan-to-command latency and per-scan CPU on real scans (gap_follow_latency_bench.py).
        import gap_follow_latency_bench

        return gap_follow_latency_bench.main(raw[1:])
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = parser.add_subparsers(dest="cmd", required=True)
    for name in ("search", "evaluate"):
        p = sub.add_parser(name)
        p.add_argument("--out", type=Path, required=True)
        p.add_argument("--jobs", type=int, default=5)
        p.add_argument("--domain-base", type=int, default=200)
        p.add_argument("--laps", type=int, default=2 if name == "search" else 3)
        if name == "search":
            p.add_argument("--samples", type=int, default=24)
            p.add_argument("--refine", type=int, default=2)
            p.add_argument("--seed", type=int, default=1)
        else:
            p.add_argument("--tracks", type=int, default=None, help="how many held-out tracks")
            p.add_argument(
                "--repeats",
                type=int,
                default=1,
                help="run every variant (and the baseline) this many times and pool the runs: "
                "brake counts are not repeatable to the run",
            )
            p.add_argument(
                "--tuning-tracks",
                action="store_true",
                help="run the TUNING tracks instead (for the tuning-set column of a report)",
            )
            p.add_argument(
                "--family",
                choices=("narrow", "wide"),
                default="narrow",
                help="track family: narrow (0.85 to 1.2 m lanes, the original) or wide (1.3 to 1.5 m, "
                "the 2026-11-12 oval)",
            )
            p.add_argument(
                "--rule",
                choices=("baseline", "gate"),
                default="baseline",
                help="admissibility: baseline (selection.py, relative to the recommended profile) or "
                "gate (selection_gate.py, absolute brakes per lap: the follower must not lean on "
                "safety_node)",
            )
            p.add_argument(
                "--variant",
                action="append",
                default=[],
                help='name={"profile": ..., "params": {...}} (repeatable)',
            )
    p_sel = sub.add_parser("select")
    p_sel.add_argument("--out", type=Path, required=True)
    p_rep = sub.add_parser("replay")
    p_rep.add_argument("--bags", type=Path, required=True, help="a bag, or a directory of bags")
    p_rep.add_argument("--profile", default="race-2026-11-12")
    p_rep.add_argument("--baseline-profile", default="lane-2026-10-25")
    p_rep.add_argument("--cli", help="path to gap_follow_replay_cli")
    p_rep.add_argument("--out", type=Path)
    p_rep.add_argument("--param", action="append", default=[], help="override, name=value")
    args = parser.parse_args(argv)
    return {"search": search, "evaluate": evaluate, "select": select_cmd, "replay": replay_cmd}[
        args.cmd
    ](args)


if __name__ == "__main__":
    sys.exit(main())
