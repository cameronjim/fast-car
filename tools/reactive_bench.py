#!/usr/bin/env python3
"""Reactive benchmark battery for gap_follow_node: all tracks for a profile, a results table,
a note, and a parameter sweep.

Runs tests/l5_reactive_bench/reactive_bench_runner.py once per track (five synthetic tracks:
lane_square, lane_round, lane_open_side, chicane, open_room; see racer_gym_bridge.track_loader)
with the real command path (bridge_node, safety_node, gap_follow_node) and collects the JSON
results. It needs a sourced ROS workspace with racer_gym_bridge, racer_safety and racer_control
built, so run it in the ros-dev container: tools/reactive_bench_docker.sh does that on an
isolated docker network.

    # the battery for the floor profile and the node defaults, written to a dated note
    tools/reactive_bench.py --profile floor-2026-10-06,none

    # the CI subset (lane_square, lane_open_side, chicane, first lap only), exit 1 on a failure
    tools/reactive_bench.py --battery ci --check --no-note

    # a parameter sweep on top of a profile, ranked: no wall contact, then fewest obstacle
    # brakes per lap, then lap time
    tools/reactive_bench.py --profile floor-2026-10-06 --jobs 3 \\
        --sweep steering_gain=1.0,1.6,2.4 --sweep centering_gain=0.0,0.4,0.6

Runs side by side get distinct ROS_DOMAIN_IDs (--domain-base + slot). The results table is
markdown, printed and written to docs/notes/reactive-bench-<date>.md (--note to change it); the
tables in that file are generated, so put any prose in it AFTER regenerating.
"""

from __future__ import annotations

import argparse
import datetime
import itertools
import json
import math
import queue
import subprocess
import sys
import tempfile
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
RUNNER = REPO_ROOT / "tests" / "l5_reactive_bench" / "reactive_bench_runner.py"
NOTES_DIR = REPO_ROOT / "docs" / "notes"

TRACKS = ("lane_square", "lane_round", "lane_open_side", "chicane", "open_room")
# The loops, whose laps are counted (open_room has none).
LOOP_TRACKS = ("lane_square", "lane_round", "lane_open_side", "chicane")

# The CI battery: the floor profile must complete a lap of each without touching a wall.
# (track, max run time in s); the run stops at the first lap.
CI_BATTERY = (("lane_square", 120.0), ("lane_open_side", 80.0), ("chicane", 80.0))

# Integer-typed gap_follow_node parameters (a sweep value for anything else is a float).
INT_PARAMETERS = {"target_range_median_scans"}
# String-typed gap_follow_node parameters (a sweep value is taken as written).
STRING_PARAMETERS = {"target_mode"}

Result = dict


# -- Running ------------------------------------------------------------------------------------


def _invoke_runner(command: list[str], timeout_s: float) -> tuple[int, str]:
    """Runs the runner; returns (exit status, stderr tail). Isolated for tests."""
    proc = subprocess.run(command, capture_output=True, text=True, timeout=timeout_s, check=False)
    return proc.returncode, proc.stderr[-1500:]


def run_one(
    track: str,
    profile: str,
    overrides: dict,
    *,
    duration_s: float = 0.0,
    stop_after_laps: int = 0,
    stop_after_contacts: int = 0,
    track_params: dict | None = None,
    domain_id: int = 0,
    attempts: int = 2,
    invoke: Callable[[list[str], float], tuple[int, str]] = _invoke_runner,
) -> Result:
    """One benchmark run in a runner subprocess. A run whose stack failed to start is retried
    once (startup races); a run that still fails comes back as {"error": ...} with its setup."""
    last: Result = {}
    for _ in range(attempts):
        with tempfile.TemporaryDirectory(prefix="reactive_bench_cli_") as tmp:
            out = Path(tmp) / "result.json"
            command = [
                sys.executable,
                str(RUNNER),
                "--track",
                track,
                "--profile",
                profile,
                "--params-json",
                json.dumps(overrides),
                "--track-params-json",
                json.dumps(track_params or {}),
                "--duration",
                str(duration_s),
                "--stop-after-laps",
                str(stop_after_laps),
                "--stop-after-contacts",
                str(stop_after_contacts),
                "--domain-id",
                str(domain_id),
                "--out",
                str(out),
            ]
            budget = (duration_s or 150.0) * 3.0 + 180.0
            try:
                status, stderr = invoke(command, budget)
            except subprocess.TimeoutExpired:
                last = {"error": f"runner timed out after {budget:.0f} s"}
            else:
                if out.exists():
                    last = json.loads(out.read_text())
                else:
                    last = {"error": f"runner exited {status} without a result: {stderr}"}
        if "error" not in last:
            break
    last.setdefault("track", track)
    last.setdefault("profile", profile)
    last.setdefault("overrides", overrides)
    last.setdefault("track_params", track_params or {})
    return last


def run_batteries(
    profile: str,
    override_sets: list[dict],
    tracks: list[tuple[str, float]],
    *,
    duration_scale: float = 1.0,
    stop_after_laps: int = 0,
    stop_after_contacts: int = 0,
    track_params: dict | None = None,
    domain_ids: queue.Queue[int],
    jobs: int = 1,
    runner: Callable[..., Result] = run_one,
) -> list[list[Result]]:
    """One battery (every (track, duration) of ``tracks``) per parameter set in
    ``override_sets``, with ALL their runs pooled so up to ``jobs`` run side by side, each on a
    domain id taken from ``domain_ids`` (put back when it finishes). Returns one list of results
    per parameter set, in track order."""
    work = [(i, track, duration) for i in range(len(override_sets)) for track, duration in tracks]

    def one(item: tuple[int, str, float]) -> Result:
        i, track, duration = item
        domain = domain_ids.get()
        try:
            result = runner(
                track,
                profile,
                override_sets[i],
                duration_s=duration * duration_scale if duration else 0.0,
                stop_after_laps=stop_after_laps,
                stop_after_contacts=stop_after_contacts,
                track_params=track_params,
                domain_id=domain,
            )
        finally:
            domain_ids.put(domain)
        print(
            f"done {track} {override_sets[i]}: "
            + (
                f"ERROR {result['error'][:100]}"
                if "error" in result
                else f"laps {result['laps']}, "
                f"brakes {result['obstacle_brakes']}, contacts {result['wall_contacts']}"
            ),
            file=sys.stderr,
            flush=True,
        )
        return result

    with ThreadPoolExecutor(max_workers=max(1, jobs)) as pool:
        flat = list(pool.map(one, work))
    per_set = len(tracks)
    return [flat[i * per_set : (i + 1) * per_set] for i in range(len(override_sets))]


# -- Judging and ranking ------------------------------------------------------------------------


def ci_failures(results: list[Result]) -> list[str]:
    """Why the CI battery fails: an errored run, any wall contact, or a loop with no lap."""
    failures = []
    for r in results:
        name = r.get("track", "?")
        if "error" in r:
            failures.append(f"{name}: run failed: {r['error'][:300]}")
            continue
        if r["wall_contacts"] > 0:
            failures.append(f"{name}: {r['wall_contacts']} wall contact(s) {r['contact_points']}")
        if r["laps"] is not None and r["laps"] < 1:
            failures.append(
                f"{name}: no lap in {r['duration_s']:.0f} s (progress {r['progress_m']} of "
                f"{r['loop_length_m']} m)"
            )
    return failures


def aggregate(results: list[Result]) -> dict:
    """One parameter set's results (every track, repeats included), as the ranking uses them.

    Contacts are counted on every track; brakes per lap use the loop tracks only (the open room
    has no laps, and its brakes say how often the follower meets a wall, not how a lap goes);
    the lap time is the sum over the loop tracks of that track's mean lap time (so it does not
    grow with the number of repeats).
    """
    ok = [r for r in results if "error" not in r]
    loops = [r for r in ok if r.get("laps") is not None]
    laps = sum(r["laps"] for r in loops)
    loop_brakes = sum(r["obstacle_brakes"] for r in loops)
    lap_times: dict[str, list[float]] = {}
    for r in loops:
        if r["lap_time_s"] is not None:
            lap_times.setdefault(r["track"], []).append(r["lap_time_s"])
    offsets = [
        r["mean_centreline_offset_m"]
        for r in loops
        if r.get("mean_centreline_offset_m") is not None
    ]
    return {
        "errors": len(results) - len(ok),
        "wall_contacts": sum(r["wall_contacts"] for r in ok),
        "loop_wall_contacts": sum(r["wall_contacts"] for r in loops),
        "incomplete_loops": sum(1 for r in loops if r["laps"] < 1) + (len(results) - len(ok)),
        "laps": laps,
        "obstacle_brakes": sum(r["obstacle_brakes"] for r in ok),
        "loop_brakes": loop_brakes,
        "brakes_per_lap": loop_brakes / laps if laps else math.inf,
        "lap_time_sum_s": sum(sum(v) / len(v) for v in lap_times.values()),
        "escapes": sum(r["escapes"] for r in ok),
        "min_clearance_m": min((r["min_body_clearance_m"] for r in ok), default=math.nan),
        # Mean over the loop runs of each run's mean distance from the centreline (m).
        "mean_centreline_offset_m": sum(offsets) / len(offsets) if offsets else math.nan,
    }


def rank_key(agg: dict) -> tuple:
    """The sweep's order, best first: zero wall contacts first (fewest overall), a lap on every
    loop track, then the fewest obstacle brakes per lap, then the shortest laps."""
    return (
        agg["wall_contacts"],
        agg["incomplete_loops"],
        agg["brakes_per_lap"],
        agg["lap_time_sum_s"],
    )


def parse_sweep(specs: list[str]) -> dict[str, list]:
    """["steering_gain=1.0,1.6", "reverse_escape=true,false"] -> {name: [values]}."""
    grid: dict[str, list] = {}
    for spec in specs:
        name, _, values = spec.partition("=")
        if not name or not values:
            raise ValueError(f"sweep must be name=v1,v2,...; got {spec!r}")
        parsed: list = []
        for token in values.split(","):
            token = token.strip()
            if name in STRING_PARAMETERS:
                parsed.append(token)
            elif token.lower() in ("true", "false"):
                parsed.append(token.lower() == "true")
            elif name in INT_PARAMETERS:
                parsed.append(int(token))
            else:
                parsed.append(float(token))
        grid[name] = parsed
    return grid


def grid_points(grid: dict[str, list]) -> list[dict]:
    names = list(grid)
    return [dict(zip(names, values, strict=True)) for values in itertools.product(*grid.values())]


# -- Formatting ---------------------------------------------------------------------------------


def _fmt(value, digits: int = 2, none: str = "-") -> str:
    if value is None:
        return none
    if isinstance(value, float):
        if math.isinf(value):
            return "inf"
        return f"{value:.{digits}f}"
    return str(value)


def results_table(results: list[Result]) -> str:
    """The markdown results table of one parameter set, one row per track."""
    lines = [
        (
            "| Track | Laps | Lap time (s) | Brakes | Brakes/lap | Escapes | Min clearance (m) "
            "| Contacts | Centre offset, mean (m) | Centre offset per lap (m) "
            "| Mean abs steer (rad) | Reversals/min | Run (s) |"
        ),
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in results:
        if "error" in r:
            lines.append(
                f"| {r['track']} | ERROR | {r['error'][:80].replace('|', '/')} |||||||||||| "
            )
            continue
        per_lap = r.get("lap_centreline_offsets_m") or []
        lines.append(
            f"| {r['track']} | {_fmt(r['laps'])} | {_fmt(r['lap_time_s'], 1)} "
            f"| {r['obstacle_brakes']} | {_fmt(r.get('brakes_per_lap'), 2)} | {r['escapes']} "
            f"| {_fmt(r['min_body_clearance_m'], 3)} | {r['wall_contacts']} "
            f"| {_fmt(r.get('mean_centreline_offset_m'), 3)} "
            f"| {', '.join(f'{v:.3f}' for v in per_lap) or '-'} "
            f"| {_fmt(r['mean_abs_steering_rad'], 3)} "
            f"| {_fmt(r['steering_reversals_per_min'], 1)} | {_fmt(r['duration_s'], 0)} |"
        )
    return "\n".join(lines)


def sweep_table(rows: list[tuple[dict, dict]], names: list[str]) -> str:
    """The ranked sweep table: (parameters, aggregate) pairs, best first."""
    lines = [
        (
            "| # | " + " | ".join(names) + " | Contacts | Loop contacts | Loop runs without a lap "
            "| Laps | Loop brakes | Brakes/lap | Lap time, loops summed (s) | Escapes "
            "| Min clearance (m) | Centre offset (m) |"
        ),
        "|" + "---|" * (len(names) + 11),
    ]
    for i, (params, agg) in enumerate(rows, start=1):
        lines.append(
            f"| {i} | "
            + " | ".join(str(params[n]) for n in names)
            + f" | {agg['wall_contacts']} | {agg['loop_wall_contacts']} | {agg['incomplete_loops']} "
            f"| {agg['laps']} | {agg['loop_brakes']} | {_fmt(agg['brakes_per_lap'], 2)} "
            f"| {_fmt(agg['lap_time_sum_s'], 1)} | {agg['escapes']} "
            f"| {_fmt(agg['min_clearance_m'], 3)} "
            f"| {_fmt(agg.get('mean_centreline_offset_m', math.nan), 3)} |"
        )
    return "\n".join(lines)


def group_runs(
    runs: list[Result], names: list[str] | None = None
) -> tuple[list[str], list[tuple[dict, list[Result]]]]:
    """Regroups raw runs (--json-out of a sweep) by their parameter values: (the names of the
    parameters that vary, [(point, that point's results)]). ``names`` fixes which parameters
    count; by default every override that takes more than one value."""
    if names is None:
        values: dict[str, set] = {}
        for r in runs:
            for k, v in r.get("overrides", {}).items():
                values.setdefault(k, set()).add(v)
        names = sorted(k for k, v in values.items() if len(v) > 1)
    groups: dict[tuple, tuple[dict, list[Result]]] = {}
    for r in runs:
        point = {n: r.get("overrides", {}).get(n) for n in names}
        groups.setdefault(tuple(point.items()), (point, []))[1].append(r)
    return names, list(groups.values())


def sweep_section(
    profile: str, names: list[str], groups: list[tuple[dict, list[Result]]]
) -> tuple[str, str]:
    """(the ranked table, the note's section) for a sweep's (point, results) groups."""
    rows = sorted(
        ((point, aggregate(results)) for point, results in groups), key=lambda r: rank_key(r[1])
    )
    by_point = {tuple(sorted(point.items())): results for point, results in groups}
    table = sweep_table(rows, names)
    best_params, best_agg = rows[0]
    section = (
        f"### Sweep on profile `{profile}`, best first\n\n{table}\n\n"
        f"Best: {best_params} ({best_agg['wall_contacts']} contacts, "
        f"{_fmt(best_agg['brakes_per_lap'], 2)} brakes per lap), per track:\n\n"
        f"{results_table(by_point[tuple(sorted(best_params.items()))])}\n"
    )
    return table, section


def profile_table(result: Result) -> str:
    params = result.get("gap_follow_params") or {}
    return ", ".join(f"`{k}` {v}" for k, v in sorted(params.items()))


def contact_lines(results: list[Result]) -> list[str]:
    return [
        f"- {r['track']}: contact at t = {c['t_s']} s, ({c['x_m']}, {c['y_m']}) ({c['reason']})"
        for r in results
        if "error" not in r
        for c in r["contact_points"]
    ]


def _today() -> str:
    return datetime.datetime.now().astimezone().date().isoformat()


# -- Main ---------------------------------------------------------------------------------------


def _tracks_for(args: argparse.Namespace) -> list[tuple[str, float]]:
    if args.battery == "ci":
        return list(CI_BATTERY)
    chosen = args.tracks.split(",") if args.tracks else list(TRACKS)
    unknown = sorted(set(chosen) - set(TRACKS))
    if unknown:
        raise SystemExit(f"unknown track(s) {unknown}; known: {list(TRACKS)}")
    return [(t, max(0.0, args.duration)) for t in chosen]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--profile",
        default="floor-2026-10-06",
        help="gap_follow.launch.py profile; several separated by commas run one battery each",
    )
    parser.add_argument(
        "--params-json",
        default="{}",
        help="gap_follow_node parameters on top of the profile (JSON)",
    )
    parser.add_argument(
        "--track-params-json",
        default="{}",
        help="synthetic track parameters (JSON), e.g. "
        '\'{"lane_width_m": 1.0, "box_gap_m": 1.2, "corner_radius_m": 0.3}\'; also '
        "box_length_m, jog_offset_m, room_size_m",
    )
    parser.add_argument("--battery", choices=("full", "ci"), default="full")
    parser.add_argument("--tracks", help="comma separated subset of the tracks (full battery)")
    parser.add_argument(
        "--duration", type=float, default=0.0, help="seconds per track (0 = each track's default)"
    )
    parser.add_argument(
        "--duration-scale", type=float, default=1.0, help="multiply every track's duration"
    )
    parser.add_argument(
        "--sweep",
        action="append",
        default=[],
        metavar="NAME=V1,V2,...",
        help="sweep a parameter; repeat for a grid",
    )
    parser.add_argument(
        "--stop-after-contacts",
        type=int,
        default=0,
        help="end a run after this many wall contacts (sweeps use it to stop disqualified sets early)",
    )
    parser.add_argument(
        "--jobs", type=int, default=1, help="runs side by side (each on its own ROS_DOMAIN_ID)"
    )
    parser.add_argument("--domain-base", type=int, default=60, help="first ROS_DOMAIN_ID used")
    parser.add_argument(
        "--repeats",
        type=int,
        default=1,
        help="run each swept parameter set this many times and sum them (the simulation is "
        "deterministic, real-time scheduling jitter is not)",
    )
    parser.add_argument(
        "--from-json",
        type=Path,
        help="run nothing: rank the runs of an earlier --json-out of a sweep, write the tables",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="exit 1 unless every run is clean and laps (the CI criteria)",
    )
    parser.add_argument(
        "--note",
        type=Path,
        help="markdown note to write (default docs/notes/reactive-bench-<date>.md)",
    )
    parser.add_argument("--no-note", action="store_true")
    parser.add_argument("--json-out", type=Path, help="write every run's raw JSON here")
    args = parser.parse_args(argv)

    overrides = json.loads(args.params_json)
    track_params = json.loads(args.track_params_json)
    tracks = _tracks_for(args)
    ci = args.battery == "ci"
    domain_ids: queue.Queue[int] = queue.Queue()
    for i in range(max(1, args.jobs)):
        domain_ids.put(args.domain_base + i)
    stop_laps = 1 if ci else 0
    stop_contacts = 1 if ci else args.stop_after_contacts

    sections: list[str] = []
    all_runs: list[dict] = []
    failures: list[str] = []
    grid = parse_sweep(args.sweep)
    profiles = [p.strip() for p in args.profile.split(",") if p.strip()]
    if args.from_json:
        all_runs = json.loads(args.from_json.read_text())
        profiles = []
        names, groups = group_runs(all_runs)
        table, section = sweep_section(all_runs[0]["profile"], names, groups)
        print(f"\n### {all_runs[0]['profile']} sweep\n\n{table}\n")
        sections.append(section)
    for profile in profiles:
        if not grid:
            (results,) = run_batteries(
                profile,
                [overrides],
                tracks,
                duration_scale=args.duration_scale,
                stop_after_laps=stop_laps,
                stop_after_contacts=stop_contacts,
                track_params=track_params,
                domain_ids=domain_ids,
                jobs=args.jobs,
            )
            all_runs.extend(results)
            table = results_table(results)
            print(f"\n### {profile}\n\n{table}\n")
            sections.append(f"### Profile `{profile}`\n\n{table}\n")
            lines = contact_lines(results)
            if lines:
                sections.append("Wall contacts:\n\n" + "\n".join(lines) + "\n")
            if results and "gap_follow_params" in results[0]:
                sections.append(f"Parameters: {profile_table(results[0])}\n")
            failures += [f"[{profile}] {f}" for f in ci_failures(results)]
            continue
        points = grid_points(grid)
        repeated = [{**overrides, **point} for point in points for _ in range(args.repeats)]
        batteries = run_batteries(
            profile,
            repeated,
            tracks,
            duration_scale=args.duration_scale,
            stop_after_laps=stop_laps,
            stop_after_contacts=stop_contacts,
            track_params=track_params,
            domain_ids=domain_ids,
            jobs=args.jobs,
        )
        grouped = [
            [r for battery in batteries[i * args.repeats : (i + 1) * args.repeats] for r in battery]
            for i in range(len(points))
        ]
        all_runs.extend(r for results in grouped for r in results)
        table, section = sweep_section(profile, list(grid), list(zip(points, grouped, strict=True)))
        print(f"\n### {profile} sweep\n\n{table}\n")
        sections.append(section)

    if args.json_out and not args.from_json:
        args.json_out.write_text(json.dumps(all_runs, indent=1, sort_keys=True) + "\n")
    if not args.no_note:
        note = args.note or NOTES_DIR / f"reactive-bench-{_today()}.md"
        note.write_text(
            "# Reactive benchmark results (generated)\n\n"
            f"Generated by `tools/reactive_bench.py` on {_today()}; "
            "the tables are generated, edit any prose around them after regenerating.\n\n"
            + "\n".join(sections)
        )
        print(f"wrote {note}", file=sys.stderr)
    if args.check and failures:
        print("FAILED:\n" + "\n".join(f"  {f}" for f in failures), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
