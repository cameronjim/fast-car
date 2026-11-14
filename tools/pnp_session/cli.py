"""Command line of tools/pnp_session_report.py.

    uv run --project tools python tools/pnp_session_report.py timing --bag <dir> [--out report.json]
    uv run --project tools python tools/pnp_session_report.py laps   --bag <dir>   # D3-C0
    uv run --project tools python tools/pnp_session_report.py host-facts --out-dir <dir>

Every command reads a bag or the host and prints. None of them drives a car, writes a parameter or
sets a policy.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from pnp_session import hostfacts, laps, report, timing, whatif
from racer_eval.runs import Run

from pnp_session.bagread import SessionBag

EXIT_BASELINE_DIFFERS = 3


def _read(path: Path) -> SessionBag:
    from pnp_session.bagread import read_session_bag

    return read_session_bag(path)


def caps_from_split(split_yaml: Path, bag_name: str, analysis: timing.TimingAnalysis) -> list[float] | None:
    """The cap of each run from the pre-registered split file: the pnp segment of this bag that
    contains the run's first command. None when a run has no such segment."""
    doc = yaml.safe_load(Path(split_yaml).read_text())
    bags = doc.get("bags", doc)
    entry = bags.get(bag_name)
    if entry is None:
        return None
    segments = [s for s in entry.get("segments", []) if s.get("driver") == "pnp"]
    caps: list[float] = []
    for p in analysis.pieces:
        window = p.run.counting_window
        assert window is not None
        # The segment that overlaps the run most (a run's first command can precede the
        # segment's `from` by a few hundredths of a second).
        def overlap(seg: dict[str, Any], run: Run = p.run) -> float:
            return min(run.t_end, float(seg["to"])) - max(run.t_start, float(seg["from"]))

        best = max(segments, key=overlap, default=None)
        if best is None or overlap(best) <= 0.0:
            return None
        caps.append(float(best["cap_mps"]))
    return caps


def cmd_timing(args: argparse.Namespace, read: Callable[[Path], SessionBag] | None = None) -> int:
    path = Path(args.bag)
    bag = (read or _read)(path)
    caps: list[float] | None = None
    if args.caps:
        caps = [float(x) for x in args.caps.split(",")]
    analysis = timing.analyse_timing(bag, caps, args.wall_contacts)
    if args.caps_from_split and caps is None:
        derived = caps_from_split(Path(args.caps_from_split), bag.name, analysis)
        if derived is not None:
            analysis = timing.analyse_timing(bag, derived, args.wall_contacts)
    if not analysis.pieces:
        print(f"{bag.name}: no PnP run with a commanded window (needs /pnp/planner/status)", file=sys.stderr)
        return 2
    limiter: dict[str, Any]
    if args.no_limiter:
        limiter = whatif.not_run("--no-limiter given")
    else:
        try:
            cli = whatif.find_cli(args.limiter_cli)
            limiter = whatif.whatif_report(bag, analysis, cli)
        except whatif.LimiterUnavailable as exc:
            limiter = whatif.not_run(str(exc))
    is_baseline = bag.name.startswith(report.BASELINE_BAG) or args.check_baseline
    doc = report.assemble(bag.name, analysis.report, limiter, is_baseline)
    if args.out:
        Path(args.out).write_text(json.dumps(doc, indent=2, sort_keys=True, default=_json_default) + "\n")
    print(report.markdown(doc))
    if is_baseline and not doc["baseline_check"]["matches"]:
        print("BASELINE DIFFERS from the pre-registered counts: both are reported above.", file=sys.stderr)
        return EXIT_BASELINE_DIFFERS
    return 0


def _json_default(o: Any) -> Any:
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.floating):
        return float(o)
    if isinstance(o, np.bool_):
        return bool(o)
    raise TypeError(f"not serialisable: {type(o)}")


def cmd_laps(args: argparse.Namespace, read: Callable[[Path], SessionBag] | None = None) -> int:
    bag = (read or _read)(Path(args.bag))
    video = None
    name = "camera"
    if args.reference == "video":
        if not args.times:
            print("--reference video needs --times <csv>", file=sys.stderr)
            return 2
        video = laps.read_video_times(Path(args.times))
        name = "video"
    elif args.times:
        print("--times only goes with --reference video", file=sys.stderr)
        return 2
    result = laps.d3c0(laps.inputs_from_bag(bag, video), name)
    result = {"schema": "pnp-session/1", "report": "laps", "bag": bag.name, **result}
    if args.out:
        Path(args.out).write_text(json.dumps(result, indent=2, sort_keys=True, default=_json_default) + "\n")
    print(laps_markdown(result))
    return {"PASS": 0, "FAIL": 1}.get(result["verdict"], 2)


def laps_markdown(r: dict[str, Any]) -> str:
    lines = [
        f"# D3-C0 lap check: {r['bag']}",
        "",
        f"Reference: {r['reference']}. Rule ({r['section']}): {r['rule']}.",
        "",
        f"**{r['verdict']}**" + (": " + "; ".join(r["reasons"]) if r["reasons"] else ""),
        "",
        "| run | reference laps | lap memory laps | count equal | within tolerance |",
        "| --- | --- | --- | --- | --- |",
    ]
    for run in r["runs"]:
        lines.append(
            f"| {run['run_id']} | {run['n_reference']} | {run['n_pnp']} | "
            f"{'yes' if run['count_equal'] else 'NO'} | {run['laps_within']} of {run['laps_compared']} |"
        )
    share = r["share_within"]
    lines += [
        "",
        f"Laps within {r['tolerance_s']} s: {r['laps_within']} of {r['laps_compared']}"
        + ("" if share is None else f" ({share:.2f}), needed {r['laps_within_needed']}"),
        "",
    ]
    if r["laps_off_by_more_than_tolerance"]:
        lines.append("Laps off by more than the tolerance (reference minus lap memory):")
        for x in r["laps_off_by_more_than_tolerance"]:
            lines.append(f"- {x['run_id']} lap {x['lap']}: {x['diff_s']:+.3f} s")
        lines.append("")
    return "\n".join(lines) + "\n"


def cmd_host_facts(args: argparse.Namespace, runner: hostfacts.Runner | None = None) -> int:
    facts = hostfacts.collect(runner or hostfacts.default_runner)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "host_facts.txt").write_text(hostfacts.format_text(facts))
    (out_dir / "host_facts.json").write_text(json.dumps(facts, indent=2, sort_keys=True) + "\n")
    print(hostfacts.format_text(facts), end="")
    return 0


def cmd_limiter_stream(args: argparse.Namespace, read: Callable[[Path], SessionBag] | None = None) -> int:
    """Developer command: write the teacher-forced cycle stream of a bag's counting windows and the
    recorded rate_limit engagements inside them (the fixture of the tool's ctest)."""
    bag = (read or _read)(Path(args.bag))
    analysis = timing.analyse_timing(bag)
    windows = [p.run.counting_window for p in analysis.pieces]
    assert all(w is not None for w in windows)
    n = whatif.write_stream(bag, Path(args.out), windows, teacher=True)  # type: ignore[arg-type]
    print(f"{n} cycles written to {args.out}")
    if args.events_out:
        sel = (bag.ev_phase == 0) & (bag.ev_source == "rate_limit")
        events = [
            {"t": float(bag.ev_stamp[i]), "detail": str(bag.ev_detail[i])}
            for i in np.nonzero(sel)[0]
            if whatif._in_any(float(bag.ev_t[i]), [("w", w) for w in windows]) is not None  # type: ignore[misc]
        ]
        Path(args.events_out).write_text(json.dumps(events, indent=1) + "\n")
        print(f"{len(events)} recorded rate_limit engagements written to {args.events_out}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="pnp_session_report")
    sub = p.add_subparsers(dest="command", required=True)
    t = sub.add_parser("timing", help="M-T timing report of a bag (pre-registration v0.3 section 5)")
    t.add_argument("--bag", required=True, help="bag directory or .mcap file")
    t.add_argument("--out", help="write the JSON here")
    t.add_argument("--caps", help="comma list, the cap in m/s of each PnP run in order (for the 4.2 acceptance)")
    t.add_argument("--caps-from-split", help="pnp-v0.2-split.yaml: take each run's cap from its segment")
    t.add_argument("--wall-contacts", type=int, default=0, help="wall contacts seen by people or video (default 0, not observable from a bag)")
    t.add_argument("--limiter-cli", help="path of limiter_whatif_cli")
    t.add_argument("--no-limiter", action="store_true", help="skip the rate limiter what-if")
    t.add_argument("--check-baseline", action="store_true", help="compare with the 4.2 hand count whatever the bag is called")
    t.set_defaults(func=cmd_timing)
    lp = sub.add_parser("laps", help="D3-C0: lap memory events against the camera counter (or video)")
    lp.add_argument("--bag", required=True)
    lp.add_argument("--reference", choices=["camera", "video"], default="camera")
    lp.add_argument("--times", help="CSV run,lap,lap_time_s of the hand-timed laps (with --reference video)")
    lp.add_argument("--out", help="write the JSON here")
    lp.set_defaults(func=cmd_laps)
    h = sub.add_parser("host-facts", help="scheduling policy, CPU governor, load: read only")
    h.add_argument("--out-dir", required=True)
    h.set_defaults(func=cmd_host_facts)
    s = sub.add_parser("limiter-stream", help="developer: the cycle stream of a bag for the tool's ctest")
    s.add_argument("--bag", required=True)
    s.add_argument("--out", required=True)
    s.add_argument("--events-out", help="write the recorded rate_limit engagements here (JSON)")
    s.set_defaults(func=cmd_limiter_stream)
    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.func(args))
