"""Command line of tools/pnp_session_report.py.

    uv run --project tools python tools/pnp_session_report.py timing --bag <dir> [--out report.json]
    uv run --project tools python tools/pnp_session_report.py laps   --bag <dir>   # D3-C0
    uv run --project tools python tools/pnp_session_report.py lap-c0 --bag <dir> --bag <dir>
    uv run --project tools python tools/pnp_session_report.py host-facts --out-dir <dir>
    uv run --project tools --extra session python tools/pnp_session_report.py v04 --bag <dir> ...
    uv run --project tools --extra session python tools/pnp_session_report.py variant-form --form <f>

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
from racer_eval import variant_form
from racer_eval.runs import Run

from pnp_session import hostfacts, laps, report, timing, v04, whatif
from pnp_session.bagread import SessionBag
from pnp_session.constants import BASELINE_BAG

EXIT_BASELINE_DIFFERS = 3
EXIT_PREREG = 4


def _read(path: Path) -> SessionBag:
    from pnp_session.bagread import read_session_bag

    return read_session_bag(path)


def caps_from_split(
    split_yaml: Path, bag_name: str, analysis: timing.TimingAnalysis
) -> list[float] | None:
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
        print(
            f"{bag.name}: no PnP run with a commanded window (needs /pnp/planner/status)",
            file=sys.stderr,
        )
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
    is_baseline = bag.name.startswith(BASELINE_BAG) or args.check_baseline
    doc = report.assemble(bag.name, analysis.report, limiter, is_baseline)
    if getattr(args, "m_t2", False):
        # M-T2 (addendum 1 section 4): next to M-T, whose result above stays as it is.
        try:
            doc["m_t2"] = whatif.m_t2_report(bag, analysis, whatif.find_cli(args.limiter_cli))
        except whatif.LimiterUnavailable as exc:
            doc["m_t2"] = whatif.not_run(str(exc))
    if args.out:
        Path(args.out).write_text(
            json.dumps(doc, indent=2, sort_keys=True, default=_json_default) + "\n"
        )
    print(report.markdown(doc))
    if is_baseline and not doc["baseline_check"]["matches"]:
        print(
            "BASELINE DIFFERS from the pre-registered counts: both are reported above.",
            file=sys.stderr,
        )
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
        Path(args.out).write_text(
            json.dumps(result, indent=2, sort_keys=True, default=_json_default) + "\n"
        )
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


def cmd_lap_c0(args: argparse.Namespace) -> int:
    """D3-C0, the D7-C0 rule and D8-C0 for the same bags (brief 50): each bag is one run."""
    from pnp_session import lap_c0

    th = lap_c0.load_thresholds()
    video = lap_c0.read_video_laps(Path(args.video)) if args.video else None
    runs = []
    for bag_path in args.bag:
        bag = lap_c0.read_bag(Path(bag_path))
        replayed = lap_c0.replay_bag(Path(bag_path)) if args.replay else None
        runs.append(lap_c0.run_from_bag(bag, video, replayed))
    name = "video" if video is not None else "camera"
    report = lap_c0.build_report(runs, name, th, bool(args.replay), args.rule)
    if args.out:
        Path(args.out).write_text(
            json.dumps(report, indent=2, sort_keys=True, default=_json_default) + "\n"
        )
    print(lap_c0.markdown(report))
    verdict = report["d8_c0" if args.rule == "d8" else "d7_c0"]["verdict"]
    return 0 if verdict == lap_c0.PASS else 1


def cmd_lap_src(args: argparse.Namespace) -> int:
    """D9-C0 (default), D10-C0 (--d10), D9-B0 (--offline) or the lap table (--drive) of the
    camera lap source."""
    from pnp_session import lap_src

    bags = [Path(b) for b in args.bag]
    try:
        report, text, code = lap_src.run(
            bags,
            laps_counted=args.laps_counted or (),
            offline=bool(args.offline),
            assume_ready=bool(args.assume_ready),
            drive=bool(args.drive),
            d10=bool(args.d10),
        )
    except (lap_src.PreregError, lap_src.FreezeRequired, lap_src.NoCameraNode) as error:
        print(f"REFUSED: {error}", file=sys.stderr)
        return EXIT_PREREG
    if args.out:
        Path(args.out).write_text(
            json.dumps(report, indent=2, sort_keys=True, default=_json_default) + "\n"
        )
    print(text)
    return code


def cmd_session(args: argparse.Namespace) -> int:
    """The checks of a car session read by hand in 2026-11-22, per run (session_checks)."""
    from pnp_session import session_checks

    report, text = session_checks.run([Path(b) for b in args.bag])
    facts = None
    if args.host_facts:
        try:
            facts = json.loads(Path(args.host_facts).read_text())
        except (OSError, ValueError):
            facts = None  # reported as `not recorded`, never as a pass
    check = hostfacts.scheduling_check(facts, hostfacts.load_scheduling(), args.rt_sched)
    report["scheduling_check"] = check
    text = text.rstrip("\n") + "\n\n" + hostfacts.format_check(check)
    if args.out:
        Path(args.out).write_text(
            json.dumps(report, indent=2, sort_keys=True, default=_json_default) + "\n"
        )
    print(text)
    return 0


def cmd_host_facts(args: argparse.Namespace, runner: hostfacts.Runner | None = None) -> int:
    facts = hostfacts.collect(runner or hostfacts.default_runner)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    text = hostfacts.format_text(facts)
    if args.rt_sched:
        check = hostfacts.scheduling_check(facts, hostfacts.load_scheduling(), args.rt_sched)
        facts["scheduling_check"] = check
        text += hostfacts.format_check(check)
    (out_dir / "host_facts.txt").write_text(text)
    (out_dir / "host_facts.json").write_text(json.dumps(facts, indent=2, sort_keys=True) + "\n")
    print(text, end="")
    return 0


def cmd_check_prereg(args: argparse.Namespace) -> int:
    try:
        digest = v04.check_prereg()
    except v04.PreregMismatch as exc:
        print(f"REFUSED: {exc}", file=sys.stderr)
        return EXIT_PREREG
    print(f"{v04.PREREG_YAML}: SHA-256 {digest} matches the frozen file")
    return 0


def _parse_results(items: Sequence[str]) -> dict[str, str]:
    out: dict[str, str] = {}
    for item in items:
        key, sep, value = item.partition("=")
        if not sep or value not in v04.STATUSES:
            raise SystemExit(f"--result {item!r}: use key=met, key=unmet or key=unknown")
        out[key] = value
    return out


def cmd_v04(args: argparse.Namespace, read: Callable[[Path], SessionBag] | None = None) -> int:
    try:
        sha = v04.check_prereg()
    except v04.PreregMismatch as exc:
        print(f"REFUSED: {exc}", file=sys.stderr)
        return EXIT_PREREG
    bag = (read or _read)(Path(args.bag))
    reference = v04.choose_reference(args.owner_accepted_camera, args.m_cam_held)
    video = None
    if reference == "video" and args.times:
        video = laps.read_video_times(Path(args.times))
    caps = [float(x) for x in args.caps.split(",")] if args.caps else None
    limiter: dict[str, Any]
    if args.no_limiter:
        limiter = whatif.not_run("--no-limiter given")
    else:
        try:
            analysis = timing.analyse_timing(bag, caps, args.wall_contacts)
            limiter = whatif.whatif_report(bag, analysis, whatif.find_cli(args.limiter_cli))
        except whatif.LimiterUnavailable as exc:
            limiter = whatif.not_run(str(exc))
    doc = v04.v04_report(
        bag,
        caps=caps,
        wall_contacts=args.wall_contacts,
        reference=reference,
        video_times=video,
        lapref_dir=Path(args.lapref_dir) if args.lapref_dir else None,
        results=_parse_results(args.result),
        limiter=limiter,
        prereg_sha256=sha,
        code_equals_b4=v04.code_identity(args.b4_commit),
    )
    if args.out:
        Path(args.out).write_text(v04.to_json(doc))
    print(v04.markdown(doc))
    if reference == "video" and not args.times:
        print(
            "D3-C0 reference is the phone video (the camera counter is not accepted as primary "
            "with M-CAM held): pass --times <csv> of the hand-timed laps.",
            file=sys.stderr,
        )
    return {"PASS": 0, "FAIL": 1}.get(doc["d3_c0"]["verdict"], 2)


def cmd_variant_form(args: argparse.Namespace) -> int:
    form_path = Path(args.form)
    try:
        form = variant_form.load_form(form_path)
        earlier: list[tuple[str, float]] = []
        for p in args.earlier_forms:
            earlier += variant_form.positions_of(variant_form.load_form(Path(p)))
        z = v04.lidar_mount_z_m()
        checked = variant_form.check_form(form, z, earlier)
    except variant_form.FormInvalid as exc:
        print("VARIANT FORM REFUSED:", file=sys.stderr)
        for problem in exc.problems:
            print(f"- {problem}", file=sys.stderr)
        return 1
    photo = form_path.parent / checked.form.photo
    if not photo.is_file():
        print(f"VARIANT FORM REFUSED: photo {photo} is not next to the form", file=sys.stderr)
        return 1
    receipt = variant_form.write_receipt(
        form_path, checked, earlier, "vehicle_params sensors.lidar.mount_z_m via gen_params"
    )
    print(
        f"variant {checked.form.variant_id}: {len(checked.form.objects)} objects, LiDAR plane "
        f"{z:.3f} m, receipt {receipt.name}"
    )
    if args.conditions:
        order = variant_form.interleaved_order(
            checked, args.conditions.split(","), args.runs_per_condition
        )
        print(f"run order (seed {checked.form.seed}): {' '.join(order)}")
    return 0


def cmd_limiter_stream(
    args: argparse.Namespace, read: Callable[[Path], SessionBag] | None = None
) -> int:
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
    t = sub.add_parser(
        "timing", help="M-T timing report of a bag (pre-registration v0.3 section 5)"
    )
    t.add_argument("--bag", required=True, help="bag directory or .mcap file")
    t.add_argument("--out", help="write the JSON here")
    t.add_argument(
        "--caps",
        help="comma list, the cap in m/s of each PnP run in order (for the 4.2 acceptance)",
    )
    t.add_argument(
        "--caps-from-split", help="pnp-v0.2-split.yaml: take each run's cap from its segment"
    )
    t.add_argument(
        "--wall-contacts",
        type=int,
        default=0,
        help="wall contacts seen by people or video (default 0, not observable from a bag)",
    )
    t.add_argument("--limiter-cli", help="path of limiter_whatif_cli")
    t.add_argument("--no-limiter", action="store_true", help="skip the rate limiter what-if")
    t.add_argument(
        "--m-t2",
        action="store_true",
        help="also the M-T2 limiter replay with the latched command read back from /drive",
    )
    t.add_argument(
        "--check-baseline",
        action="store_true",
        help="compare with the 4.2 hand count whatever the bag is called",
    )
    t.set_defaults(func=cmd_timing)
    lp = sub.add_parser(
        "laps", help="D3-C0: lap memory events against the camera counter (or video)"
    )
    lp.add_argument("--bag", required=True)
    lp.add_argument("--reference", choices=["camera", "video"], default="camera")
    lp.add_argument(
        "--times", help="CSV run,lap,lap_time_s of the hand-timed laps (with --reference video)"
    )
    lp.add_argument("--out", help="write the JSON here")
    lp.set_defaults(func=cmd_laps)
    lc = sub.add_parser(
        "lap-c0",
        help="D3-C0, the D7-C0 rule and D8-C0 of the same bags (one bag per run), brief 50",
    )
    lc.add_argument("--bag", action="append", required=True, help="a bag, once per run")
    lc.add_argument(
        "--video",
        help="CSV run,lap,lap_time_s,t_cross_s[,distance_m]: the reference instead of the camera",
    )
    lc.add_argument(
        "--rule", choices=["d7", "d8"], default="d8", help="which block sets the exit code"
    )
    lc.add_argument(
        "--replay",
        action="store_true",
        help="recompute the lap events through the checked-out gate: a regression, not a gate",
    )
    lc.add_argument("--out", help="write the JSON here")
    lc.set_defaults(func=cmd_lap_c0)
    ls = sub.add_parser(
        "lap-src",
        help="D9-C0 (camera lap source, from recorded topics), --offline D9-B0, --drive lap table "
        "(brief 51)",
    )
    ls.add_argument("--bag", action="append", required=True, help="a bag, once per run")
    ls.add_argument(
        "--laps-counted",
        action="append",
        type=int,
        help="the owner's count at the stop, once per bag in order (else the wheel rule)",
    )
    ls.add_argument(
        "--offline",
        action="store_true",
        help="D9-B0: replay the camera's events through the lap source rule. Regression unless the "
        "bags are the frozen evaluation bags",
    )
    ls.add_argument(
        "--assume-ready",
        action="store_true",
        help="with --offline: the lap table shows the replay with ready assumed (development)",
    )
    ls.add_argument("--drive", action="store_true", help="the lap table of a D run")
    ls.add_argument(
        "--d10",
        action="store_true",
        help="D10-C0 (brief 51c): D9-C0's items plus (vi), the planner started after two ready "
        "lap node statuses, and the lapref period and autocorrelation peak (reported, no "
        "threshold). Thresholds read through the D10 yaml's hash",
    )
    ls.add_argument("--out", help="write the JSON here")
    ls.set_defaults(func=cmd_lap_src)
    se = sub.add_parser(
        "session",
        help="per run: lap table, peaks, missed crossings, blips, samples, safety legs (read only)",
    )
    se.add_argument("--bag", action="append", required=True, help="a bag, once per bag")
    se.add_argument("--out", help="write the JSON here")
    se.add_argument(
        "--host-facts",
        help="host_facts.json of the run (host-facts step), for the scheduling check",
    )
    se.add_argument(
        "--rt-sched",
        choices=["on", "off"],
        default="off",
        help="the rt_sched value the run was launched with (brief 52); default off",
    )
    se.set_defaults(func=cmd_session)
    h = sub.add_parser("host-facts", help="scheduling policy, CPU governor, load: read only")
    h.add_argument("--out-dir", required=True)
    h.add_argument(
        "--rt-sched",
        choices=["on", "off"],
        help="also compare with config/scheduling.yaml and this rt_sched value (brief 52)",
    )
    h.set_defaults(func=cmd_host_facts)
    sub.add_parser(
        "check-prereg", help="refuse unless the v0.4 constants file is the frozen one"
    ).set_defaults(func=cmd_check_prereg)
    v = sub.add_parser(
        "v04",
        help="v0.4 session report: M-T, D3-C0 with its reference, lapref next to it, steps allowed",
    )
    v.add_argument("--bag", required=True)
    v.add_argument("--caps", help="comma list, the cap in m/s of each PnP run in order")
    v.add_argument("--wall-contacts", type=int, default=0)
    v.add_argument("--owner-accepted-camera", action="store_true", help="v0.4 section 9 item 3")
    v.add_argument("--m-cam-held", action="store_true", help="M-CAM held (v0.4 section 4.1)")
    v.add_argument(
        "--times", help="CSV run,lap,lap_time_s of the hand-timed laps (video reference)"
    )
    v.add_argument("--lapref-dir", help="directory of <run_id>.json written by tools/lapref.py")
    v.add_argument(
        "--result",
        action="append",
        default=[],
        help="a known result, key=met|unmet|unknown (d3_s0, d3_s1, d4_s0, ... see the procedure)",
    )
    v.add_argument(
        "--b4-commit",
        help="the commit battery B4 ran on: checks the deployed racer_laps and racer_planner "
        "against it (addendum 1 section 5.1)",
    )
    v.add_argument("--limiter-cli", help="path of limiter_whatif_cli")
    v.add_argument("--no-limiter", action="store_true")
    v.add_argument("--out", help="write the JSON here")
    v.set_defaults(func=cmd_v04)
    f = sub.add_parser("variant-form", help="check a variant layout form and write its receipt")
    f.add_argument("--form", required=True)
    f.add_argument("--earlier-forms", nargs="*", default=[], help="forms of earlier variants")
    f.add_argument("--conditions", help="comma list: also print GP4-V's run order")
    f.add_argument("--runs-per-condition", type=int, default=5)
    f.set_defaults(func=cmd_variant_form)
    s = sub.add_parser(
        "limiter-stream", help="developer: the cycle stream of a bag for the tool's ctest"
    )
    s.add_argument("--bag", required=True)
    s.add_argument("--out", required=True)
    s.add_argument("--events-out", help="write the recorded rate_limit engagements here (JSON)")
    s.set_defaults(func=cmd_limiter_stream)
    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.func(args))
