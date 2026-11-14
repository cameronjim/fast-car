#!/usr/bin/env python3
"""Open-loop replay of a Plug-and-Play bag through the real perception and planner code.

    uv run --project tools python tools/pnp_replay.py <bag dir | .mcap | .npz> \\
        --segment R1 --backend geometric --check-fidelity \\
        --dynamics-bundle <dynamics bundle dir> --v-max 0.4 --out data/pnp_replay/r1.json

Reads /scan, /odom/wheel (and the recorded /pnp/local_track and /pnp/planner/status) of the bag,
runs the perceiver (--backend geometric|net) over the scans of each segment, feeds scans, wheel
speed and tracks in arrival order to planner_replay_cli (the ROS-free build of the real planner)
with one planner tick every 0.02 s, and writes one JSON with the perception and planner metrics
of each segment (tools/pnp_replay/metrics.py), a table on stdout, and with --compare the same
metrics next to a baseline JSON (tools/pnp_replay/baselines/).

The scans do not react to the replayed commands. The replay shows what the planner would have
decided at each recorded pose (stop, crawl, chatter), not a lap. docs/notes/pnp-replay.md.

--list-segments prints the PnP runs (R0, R1, ...) and fast laps (L) the bag holds. Exit status 1
when --check-fidelity fails, 2 for a usage problem.
"""

from __future__ import annotations

import argparse
import logging
import sys
import tempfile
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pnp_replay import bag as bagmod
from pnp_replay import metrics, perception, planner_stream, report, segments, vehicle

log = logging.getLogger("pnp_replay")
REPO_ROOT = Path(__file__).resolve().parents[1]


def _parse_value(text: str) -> float | bool:
    if text in ("true", "false"):
        return text == "true"
    return float(text)


def parse_sets(items: list[str]) -> dict[str, float | bool]:
    params: dict[str, float | bool] = {}
    for item in items:
        key, sep, value = item.partition("=")
        if not sep or not key.strip():
            raise ValueError(f"--set wants name=value, got {item!r}")
        params[key.strip()] = _parse_value(value.strip())
    return params


def parse_segment_sets(items: list[str]) -> dict[str, dict[str, float | bool]]:
    out: dict[str, dict[str, float | bool]] = {}
    for item in items:
        seg, sep, rest = item.partition(":")
        if not sep:
            raise ValueError(f"--set-segment wants SEGMENT:name=value, got {item!r}")
        out.setdefault(seg.strip(), {}).update(parse_sets([rest]))
    return out


def parse_strides(items: list[str]) -> tuple[int, dict[str, int]]:
    default, per = 1, {}
    for item in items:
        name, sep, n = item.partition("=")
        value = int(n if sep else item)
        if value < 1:
            raise ValueError("--stride wants a positive integer, or SEGMENT=N")
        if sep:
            per[name.strip()] = value
        else:
            default = value
    return default, per


def parse_windows(items: list[str]) -> list[segments.Segment]:
    out = []
    for item in items:
        name, sep, span = item.partition("=")
        a, sep2, b = span.partition(":")
        if not sep or not sep2 or not name.strip():
            raise ValueError(f"--window wants NAME=FROM:TO (bag seconds), got {item!r}")
        if float(b) <= float(a):
            raise ValueError(f"--window {name}: TO must be after FROM")
        out.append(segments.Segment(name.strip(), "window", float(a), float(b)))
    return out


def parse_v_max_segments(items: list[str]) -> dict[str, float]:
    out = {}
    for item in items:
        name, sep, v = item.partition("=")
        if not sep or not name.strip():
            raise ValueError(f"--v-max-segment wants SEGMENT=m_per_s, got {item!r}")
        out[name.strip()] = float(v)
    return out


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("bag", type=Path, help="bag directory, .mcap file, or a recording .npz")
    ap.add_argument("--segment", nargs="+", metavar="NAME", help="R0 R1 ... or L (default: all)")
    ap.add_argument("--from", dest="t_from", type=float, help="window start, bag seconds")
    ap.add_argument("--to", dest="t_to", type=float, help="window end, bag seconds")
    ap.add_argument("--list-segments", action="store_true")
    ap.add_argument("--backend", choices=("geometric", "net"), default="geometric")
    ap.add_argument("--perception-bundle", type=Path, help="track_perception bundle (net)")
    ap.add_argument("--check-fidelity", action="store_true", help="the harness self-test")
    ap.add_argument(
        "--stride",
        action="append",
        default=[],
        metavar="[SEGMENT=]N",
        help="perception metrics over every Nth moving scan (default 1); repeatable",
    )
    ap.add_argument(
        "--moving-mps",
        type=float,
        default=metrics.MOVING_SPEED_MPS,
        help="perception metrics over scans with wheel speed above this (m/s)",
    )
    ap.add_argument(
        "--pool",
        nargs="+",
        metavar="NAME",
        help="segments whose perception metrics are also pooled (default: the PnP\n"
        "runs among those given)",
    )
    ap.add_argument("--no-planner", action="store_true", help="perception only")
    ap.add_argument("--dynamics-bundle", type=Path, help="dynamics bundle (the planner's)")
    ap.add_argument("--v-max", type=float, default=0.4, help="planner speed cap (m/s)")
    ap.add_argument(
        "--v-max-segment",
        action="append",
        default=[],
        metavar="SEG=M_S",
        help="planner speed cap for one segment (runs with different caps in one file)",
    )
    ap.add_argument(
        "--window",
        action="append",
        default=[],
        metavar="NAME=FROM:TO",
        help="a named window in bag seconds, selectable with --segment (e.g. P0=16.6:113.9)",
    )
    ap.add_argument(
        "--ticks",
        choices=("uniform", "recorded"),
        default="uniform",
        help="planner ticks every 0.02 s (default), or at the arrival times of the recorded\n"
        "/pnp/planner/status of a PnP run (the car's bunching and gaps; the rate\n"
        "allowance count needs them). Segments with no status use the uniform grid.",
    )
    ap.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="name=value",
        help="planner node parameter, all segments; repeatable",
    )
    ap.add_argument(
        "--set-segment",
        action="append",
        default=[],
        metavar="SEG:name=value",
        help="planner node parameter for one segment; repeatable",
    )
    ap.add_argument(
        "--tracks",
        choices=planner_stream.TRACK_MODES,
        default="replay",
        help="recorded /pnp/local_track, replayed perception (default), or none",
    )
    ap.add_argument(
        "--track-latency-s",
        type=float,
        default=0.03,
        help="replayed track arrives this long after its scan (default 0.03)",
    )
    ap.add_argument(
        "--clip-speed-to-cap",
        action="store_true",
        help="feed min(wheel speed, v_max) as odometry for the segments without a\n"
        "recorded planner status (a fast lap L another driver drove): REPLAY ONLY,\n"
        "not what the car measured",
    )
    ap.add_argument("--dead-zone-mps", type=float, default=metrics.DEFAULT_DEAD_ZONE_MPS)
    ap.add_argument("--cli", help="planner_replay_cli (default: $PLANNER_REPLAY_CLI, colcon)")
    ap.add_argument("--out", type=Path, help="result JSON (default data/pnp_replay/<bag>.json)")
    ap.add_argument("--cache", type=Path, help="recording cache .npz (default next to --out)")
    ap.add_argument("--keep-stream", action="store_true", help="keep the stream files")
    ap.add_argument("--compare", type=Path, metavar="BASELINE.json")
    return ap


def _recorded_status(rec: bagmod.Recording, seg: segments.Segment) -> tuple[Any, Any] | None:
    if not seg.has_planner_status:
        return None
    m = (rec["st_t"] >= seg.t_from) & (rec["st_t"] <= seg.t_to)
    return rec["st_t"][m], rec["st_state"][m].astype(int)


def run_segment(
    seg: segments.Segment,
    rec: bagmod.Recording,
    args: argparse.Namespace,
    perceiver: perception.Perceiver,
    work: Path,
    params: dict[str, float | bool],
    stride: int,
    v_max: float,
) -> tuple[dict[str, Any], dict[str, Any] | None, bool]:
    """One segment: perception metrics, fidelity, planner replay. Returns the segment's result,
    the per-scan arrays for pooling and whether the fidelity check (if asked) passed."""
    result: dict[str, Any] = {
        "kind": seg.kind,
        "t_from": seg.t_from,
        "t_to": seg.t_to,
        "duration_s": seg.duration_s,
    }
    if not args.no_planner:
        result["v_max_mps"] = v_max
    log.info("%s: bag %.1f-%.1f s", seg.name, seg.t_from, seg.t_to)
    ext = planner_stream.scan_indices(rec, seg.t_from - planner_stream.PREROLL_S, seg.t_to)
    log.info("%s: perception over %d scans", seg.name, len(ext))
    tracks = perception.run_perceiver(perceiver, rec, ext)
    inside = (rec["scan_t"][tracks.scan_index] >= seg.t_from) & (
        rec["scan_t"][tracks.scan_index] <= seg.t_to
    )
    summary, arrays = perception.perception_summary(
        rec, tracks.subset(inside), moving_threshold_mps=args.moving_mps, stride=stride
    )
    result["perception"] = summary
    fidelity_ok = True
    if args.check_fidelity:
        recorded = (rec["trk_stamp"] >= seg.t_from) & (rec["trk_stamp"] <= seg.t_to)
        if not recorded.any():
            result["fidelity"] = {"skipped": "no recorded /pnp/local_track in this segment"}
            log.info("%s: no recorded tracks, fidelity skipped", seg.name)
        else:
            fid = perception.check_fidelity(rec, tracks, seg.t_from, seg.t_to)
            result["fidelity"] = fid.as_dict()
            fidelity_ok = fid.ok
            log.info(
                "%s: fidelity %s, max centreline diff %.3g m over %d tracks",
                seg.name,
                "ok" if fid.ok else "FAILED",
                fid.max_centreline_diff_m,
                fid.matched,
            )
    if args.no_planner:
        return result, arrays, fidelity_ok

    limits = vehicle.load_vehicle_limits()
    recorded_ticks = None
    if args.ticks == "recorded":
        in_seg = (rec["st_t"] >= seg.t_from) & (rec["st_t"] <= seg.t_to)
        if in_seg.any():
            recorded_ticks = rec["st_t"][in_seg]
    result["ticks"] = "recorded" if recorded_ticks is not None else "uniform"
    lines = planner_stream.stream_lines(
        rec,
        seg.t_from,
        seg.t_to,
        tracks_mode=args.tracks,
        tracks=tracks,
        track_latency_s=args.track_latency_s,
        clip_speed_mps=(v_max if args.clip_speed_to_cap and not seg.has_planner_status else None),
        tick_times=recorded_ticks,
    )
    stream_path = work / f"{seg.name}.stream.txt"
    planner_stream.write_stream(lines, stream_path)
    csv_path = work / f"{seg.name}.planner.csv"
    log.info("%s: planner replay, %d stream lines", seg.name, len(lines))
    rows = planner_stream.run_cli(
        args.cli_obj, args.dynamics_bundle, v_max, params, stream_path, csv_path
    )
    if not args.keep_stream:
        stream_path.unlink(missing_ok=True)
    recorded = _recorded_status(rec, seg)
    result["planner"] = metrics.planner_metrics(
        rows["now_s"],
        rows["state"],
        rows["cmd_speed_mps"],
        rows["cmd_steering_rad"],
        rows["collision_fraction"],
        rows["solve_time_s"],
        rows["scan_age_s"],
        rec["odom_t"],
        dead_zone_mps=args.dead_zone_mps,
        recorded_t=recorded[0] if recorded else None,
        recorded_state=recorded[1] if recorded else None,
        track_confidence=rows.get("track_confidence"),
        block_reason=rows.get("block_reason"),
        rate_limits=(
            limits.steering_max_rate_rad_per_s,
            limits.max_acceleration_mps2,
            planner_stream.TICK_S,
        ),
    )
    result["planner"]["parameters"] = {k: v for k, v in sorted(params.items())}
    if recorded:
        result["recorded_status_state_counts"] = metrics.recorded_state_counts(
            rec["st_t"], rec["st_state"], seg.t_from, seg.t_to
        )
    result["planner_csv"] = str(csv_path)
    return result, arrays, fidelity_ok


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    try:
        return _main(args)
    except (ValueError, planner_stream.ReplayError, FileNotFoundError) as error:
        print(f"pnp_replay: {error}", file=sys.stderr)
        return 2


def _main(args: argparse.Namespace) -> int:
    out = args.out or (REPO_ROOT / "data" / "pnp_replay" / f"{args.bag.name}.json")
    cache = args.cache or out.parent / f"{args.bag.name}.recording.npz"
    rec = bagmod.load_recording(args.bag, cache)
    found = segments.find_segments(rec["st_t"], rec["odom_t"], rec["odom_v"])
    found += parse_windows(args.window)
    if args.list_segments:
        for s in found:
            print(
                f"{s.name:4s} {s.kind:5s} {s.t_from:8.1f} - {s.t_to:8.1f} s  ({s.duration_s:.1f} s)"
            )
        return 0
    chosen = segments.select(found, args.segment, args.t_from, args.t_to)
    if not chosen:
        raise ValueError("the bag has no segments; use --from/--to for a window")
    if args.check_fidelity and args.backend != "geometric":
        raise ValueError(
            "--check-fidelity compares against the recorded tracks, which the geometric v0 "
            "tracer made; run it with --backend geometric"
        )
    if not args.no_planner:
        if args.dynamics_bundle is None:
            raise ValueError("--dynamics-bundle is required (or --no-planner)")
        args.cli_obj = planner_stream.find_cli(args.cli)
    params = parse_sets(args.set)
    seg_params = parse_segment_sets(args.set_segment)
    v_max_segments = parse_v_max_segments(args.v_max_segment)
    default_stride, strides = parse_strides(args.stride)
    perceiver = perception.build_perceiver(args.backend, args.perception_bundle)

    result: dict[str, Any] = {
        "tool": "tools/pnp_replay.py",
        "bag": args.bag.name,
        "backend": args.backend,
        "perception_model_id": perceiver.model_id,
        "tracks": args.tracks if not args.no_planner else "n/a",
        "track_latency_s": args.track_latency_s,
        "clip_speed_to_cap": bool(args.clip_speed_to_cap),
        "v_max_mps": args.v_max,
        "dynamics_bundle": args.dynamics_bundle.name if args.dynamics_bundle else None,
        "segments": {},
    }
    if args.clip_speed_to_cap:
        log.warning(
            "--clip-speed-to-cap: the planner is fed min(wheel speed, %.2f m/s), not what the "
            "car measured (replay only)",
            args.v_max,
        )
    if not args.no_planner:
        result["planner_config"] = planner_stream.describe_cli(
            args.cli_obj, args.dynamics_bundle, args.v_max, params
        )
    out.parent.mkdir(parents=True, exist_ok=True)
    all_ok = True
    pooled: dict[str, Any] = {}
    with tempfile.TemporaryDirectory(prefix="pnp_replay_", dir=out.parent) as tmp:
        work = Path(tmp)
        for seg in chosen:
            p = {**params, **seg_params.get(seg.name, {})}
            seg_result, arrays, ok = run_segment(
                seg,
                rec,
                args,
                perceiver,
                work,
                p,
                strides.get(seg.name, default_stride),
                v_max_segments.get(seg.name, args.v_max),
            )
            all_ok = all_ok and ok
            result["segments"][seg.name] = seg_result
            pooled[seg.name] = arrays
            csv = seg_result.get("planner_csv")
            if csv:  # keep the per-tick rows next to the JSON
                kept = out.with_suffix("").parent / f"{out.stem}.{seg.name}.planner.csv"
                Path(csv).replace(kept)
                seg_result["planner_csv"] = kept.name
    pnp = (
        [s for s in chosen if s.name in args.pool]
        if args.pool
        else [s for s in chosen if s.kind == "pnp"]
    )
    if len(pnp) > 1:
        result["pooled"] = {
            "segments": [s.name for s in pnp],
            "perception": perception.pool([pooled[s.name] for s in pnp]),
        }
    report.write_json(result, out)
    baseline = report.read_json(args.compare) if args.compare else None
    report.emit(report.format_table(result, baseline), sys.stdout)
    print(f"\nwrote {out}")
    if args.check_fidelity and not all_ok:
        print("pnp_replay: FIDELITY CHECK FAILED", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
