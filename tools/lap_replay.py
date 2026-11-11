#!/usr/bin/env python3
"""Replay a logged run through the camera lap counter, to tune its thresholds offline.

    uv run --project tools --extra vision python tools/lap_replay.py data/bags/<run> \\
        --param similarity_threshold=0.85 --param min_lap_time_s=10

    uv run --project tools --extra vision python tools/lap_replay.py --images frames/ --fps 15 \\
        --odom-csv odom.csv

Feeds the recorded camera frames (and /odom/wheel, /odom) through the SAME LapCounter and the SAME
frame decoding lap_counter_node uses (ros_ws/src/racer_perception/racer_perception/lap_counter.py),
at the same frame rate limit, on the bag's log-time clock, so what it prints is what the node
would have done with those parameters. It prints the laps it counted, the similarity trace (the
highest similarity in each second, with the threshold marked) and the near misses: stretches that
rose above the rearm threshold but never counted, which is where a threshold is too high.

A bag needs a camera topic (`/camera/<name>/image_raw/compressed`, recorded by car_teleop.launch.py
whenever a camera ran) and, for the speed and distance gates, /odom/wheel; /odom adds the heading
turns. An .mcap file or a bag directory both work, a bag whose recorder was killed too.

A folder of images (jpg/png, sorted by name) is timed at --fps; --odom-csv gives the odometry on
the same clock (columns t_s, speed_mps, x_m, optional yaw_rate_rps; first row is the header). With
no odometry the start gate and the distance gate cannot work, so they are switched off (the run
starts at the first frame, min_lap_distance_m is 0) and the output says so.

docs/notes/lap-counter-2026-11-12.md has the tuning procedure.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections.abc import Callable, Iterable, Iterator
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

# The counter lives in the ROS package (pure numpy, no ROS needed to import it).
_PERCEPTION = Path(__file__).resolve().parents[1] / "ros_ws" / "src" / "racer_perception"
if str(_PERCEPTION) not in sys.path:
    sys.path.insert(0, str(_PERCEPTION))

from racer_perception.lap_counter import (
    STATE_RUNNING,
    LapConfig,
    LapCounter,
    LapEvent,
)
from racer_perception.lap_frames import decode_gray
from racer_perception.throttle import FrameThrottle

DEFAULT_RATE_HZ = 15.0
DEFAULT_REDUCTION = 4
IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png")
CHART_WIDTH = 40


@dataclass(frozen=True)
class Msg:
    """One input message on the replay clock. kind: image | odom | yaw."""

    kind: str
    t: float
    data: Any = None  # image: bytes; odom: (speed_mps, x_m or None); yaw: rate_rps


@dataclass
class ReplayResult:
    cfg: LapConfig
    events: list[LapEvent] = field(default_factory=list)
    #: (time, similarity) for every processed frame after the reference was captured.
    trace: list[tuple[float, float]] = field(default_factory=list)
    frames_seen: int = 0
    frames_processed: int = 0
    frames_failed: int = 0
    odom_messages: int = 0
    yaw_messages: int = 0
    reference_time_s: float | None = None
    first_t: float | None = None
    last_t: float | None = None
    final_status: dict[str, Any] = field(default_factory=dict)


# ----------------------------------------------------------------------------- the replay


def replay(
    messages: Iterable[Msg],
    cfg: LapConfig,
    rate_hz: float = DEFAULT_RATE_HZ,
    reduction: int = DEFAULT_REDUCTION,
    decode: Callable[[bytes, int], Any] = decode_gray,
) -> ReplayResult:
    """Run messages (in clock order) through a LapCounter exactly as lap_counter_node would."""
    counter = LapCounter(cfg)
    throttle = FrameThrottle(rate_hz)
    out = ReplayResult(cfg=cfg)
    for m in messages:
        if out.first_t is None:
            out.first_t = m.t
        out.last_t = m.t
        if m.kind == "odom":
            speed, x = m.data
            counter.update_odom(m.t, speed, x)
            out.odom_messages += 1
        elif m.kind == "yaw":
            counter.update_yaw_rate(m.t, m.data)
            out.yaw_messages += 1
        elif m.kind == "image":
            out.frames_seen += 1
            if not throttle.accept(m.t):
                continue
            try:
                gray = decode(m.data, reduction)
            except Exception:  # noqa: BLE001 - one bad frame must not end a replay
                out.frames_failed += 1
                continue
            before = counter.state
            event = counter.update_frame(m.t, gray)
            out.frames_processed += 1
            if before != counter.state:
                out.reference_time_s = m.t
            if counter.state == STATE_RUNNING and counter.similarity is not None:
                out.trace.append((m.t, counter.similarity))
            if event is not None:
                out.events.append(event)
        else:
            raise ValueError(f"unknown message kind {m.kind!r}")
    out.final_status = counter.status()
    return out


# ----------------------------------------------------------------------------- sources


def _bag_files_and_records():
    # follower_replay.bag already handles killed recorders (no footer, torn last chunk).
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from follower_replay import bag

    return bag


def list_bag_topics(path: Path) -> dict[str, int]:
    """Topic -> message count, without decoding any payload."""
    from mcap.records import Channel, Message

    bag = _bag_files_and_records()
    data = bag.BagData()
    counts: dict[str, int] = {}
    for file in bag.bag_files(path):
        channels: dict[int, str] = {}
        with file.open("rb") as stream:
            for record in bag._records(stream, data, file.name):
                if isinstance(record, Channel):
                    channels[record.id] = record.topic
                elif isinstance(record, Message) and record.channel_id in channels:
                    topic = channels[record.channel_id]
                    counts[topic] = counts.get(topic, 0) + 1
    return counts


def pick_image_topic(counts: dict[str, int], wanted: str | None) -> str:
    if wanted:
        if wanted not in counts:
            raise SystemExit(f"topic {wanted} is not in the bag; it has: {sorted(counts)}")
        return wanted
    cameras = sorted(t for t in counts if t.startswith("/camera/") and t.endswith("/compressed"))
    if len(cameras) == 1:
        return cameras[0]
    if not cameras:
        raise SystemExit(
            "no /camera/*/compressed topic in the bag (was a camera running when it was "
            f"recorded?). Topics: {sorted(counts)}"
        )
    raise SystemExit(f"several cameras in the bag, pick one with --image-topic: {cameras}")


def iter_bag_messages(
    path: Path, image_topic: str, odom_topic: str, yaw_topic: str
) -> Iterator[Msg]:
    """Decode the three topics of an mcap bag, in file (log-time) order. No ROS install needed."""
    from mcap.records import Channel, Message, Schema
    from mcap_ros2.decoder import DecoderFactory

    bag = _bag_files_and_records()
    data = bag.BagData()
    factory = DecoderFactory()
    wanted = {image_topic, odom_topic, yaw_topic}
    for file in bag.bag_files(path):
        schemas: dict[int, Any] = {}
        channels: dict[int, Any] = {}
        decoders: dict[int, Any] = {}
        with file.open("rb") as stream:
            for record in bag._records(stream, data, file.name):
                if isinstance(record, Schema):
                    schemas[record.id] = record
                elif isinstance(record, Channel):
                    channels[record.id] = record
                elif isinstance(record, Message):
                    channel = channels.get(record.channel_id)
                    if channel is None or channel.topic not in wanted:
                        continue
                    decoder = decoders.get(record.channel_id)
                    if decoder is None:
                        decoder = factory.decoder_for(
                            channel.message_encoding, schemas.get(channel.schema_id)
                        )
                        if decoder is None:
                            continue
                        decoders[record.channel_id] = decoder
                    try:
                        msg = decoder(record.data)
                    except Exception:  # noqa: BLE001 - one bad message must not lose the bag
                        data.decode_errors += 1
                        continue
                    t = record.log_time / 1e9
                    if channel.topic == image_topic:
                        yield Msg("image", t, bytes(msg.data))
                    elif channel.topic == odom_topic:
                        yield Msg(
                            "odom",
                            t,
                            (float(msg.twist.twist.linear.x), float(msg.pose.pose.position.x)),
                        )
                    else:
                        yield Msg("yaw", t, float(msg.twist.twist.angular.z))
    if data.truncated:
        print(f"note: {data.truncation_note}", file=sys.stderr)
    if data.decode_errors:
        print(f"note: {data.decode_errors} message(s) could not be decoded", file=sys.stderr)


def read_odom_csv(path: Path) -> list[Msg]:
    """t_s,speed_mps,x_m[,yaw_rate_rps] rows -> odom (and yaw) messages."""
    msgs: list[Msg] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        need = {"t_s", "speed_mps", "x_m"}
        if reader.fieldnames is None or not need <= set(reader.fieldnames):
            raise SystemExit(f"{path}: the header must have at least {sorted(need)}")
        for row in reader:
            t = float(row["t_s"])
            x = row["x_m"].strip()
            msgs.append(Msg("odom", t, (float(row["speed_mps"]), float(x) if x else None)))
            yaw = (row.get("yaw_rate_rps") or "").strip()
            if yaw:
                msgs.append(Msg("yaw", t, float(yaw)))
    return msgs


def iter_folder_messages(folder: Path, fps: float, odom_csv: Path | None) -> Iterator[Msg]:
    """Images in name order at i / fps seconds, merged with the odometry CSV by time."""
    if fps <= 0 or not math.isfinite(fps):
        raise SystemExit("--fps must be > 0")
    files = sorted(p for p in folder.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES)
    if not files:
        raise SystemExit(f"no {'/'.join(IMAGE_SUFFIXES)} images in {folder}")
    odom = read_odom_csv(odom_csv) if odom_csv else []
    odom.sort(key=lambda m: m.t)
    i = 0
    for n, file in enumerate(files):
        t = n / fps
        while i < len(odom) and odom[i].t <= t:
            yield odom[i]
            i += 1
        yield Msg("image", t, file.read_bytes())
    yield from odom[i:]


# ----------------------------------------------------------------------------- reporting


def near_misses(
    trace: list[tuple[float, float]], events: list[LapEvent], cfg: LapConfig
) -> list[tuple[float, float]]:
    """(time, peak similarity) of every stretch that rose to the rearm threshold or above, after
    the car first left the start's view, and never produced a lap. Too-high thresholds show here."""
    out: list[tuple[float, float]] = []
    departed = False
    segment: list[tuple[float, float]] = []

    def flush() -> None:
        if not segment:
            return
        t, s = max(segment, key=lambda p: p[1])
        lo, hi = segment[0][0], segment[-1][0]
        if not any(lo <= e.crossing_time_s <= hi for e in events):
            out.append((t, s))

    for t, s in trace:
        if s < cfg.rearm_threshold:
            departed = True
            flush()
            segment = []
        elif departed:
            segment.append((t, s))
    flush()
    return out


def render_trace(result: ReplayResult, bucket_s: float = 1.0) -> list[str]:
    """One row per bucket: the highest similarity in it as a bar, with the threshold marked."""
    if not result.trace:
        return ["(no similarity samples: no reference was captured)"]
    cfg = result.cfg
    t0 = result.reference_time_s if result.reference_time_s is not None else result.trace[0][0]
    buckets: dict[int, float] = {}
    for t, s in result.trace:
        k = int((t - t0) // bucket_s)
        buckets[k] = max(buckets.get(k, -1.0), s)
    crossing = {int((e.crossing_time_s - t0) // bucket_s): e for e in result.events}
    thr_col = round(cfg.similarity_threshold * CHART_WIDTH)
    rearm_col = round(cfg.rearm_threshold * CHART_WIDTH)
    lines = [
        (
            f"similarity, highest per {bucket_s:g} s ('|' = similarity_threshold "
            f"{cfg.similarity_threshold:g}, ':' = rearm_threshold {cfg.rearm_threshold:g}); "
            "t is seconds from the reference"
        )
    ]
    for k in range(min(buckets), max(buckets) + 1):
        s = buckets.get(k)
        if s is None:
            lines.append(f"{k * bucket_s:7.1f}   --")
            continue
        n = max(0, min(CHART_WIDTH, round(s * CHART_WIDTH)))
        cells = [" "] * (CHART_WIDTH + 1)
        cells[rearm_col] = ":"
        cells[thr_col] = "|"
        for i in range(n):
            cells[i] = "#"
        mark = ""
        if k in crossing:
            mark = f"  <- lap {crossing[k].lap}"
        lines.append(f"{k * bucket_s:7.1f} {s:6.3f} {''.join(cells)}{mark}")
    return lines


def format_report(result: ReplayResult, source: str, notes: list[str], bucket_s: float) -> str:
    cfg = result.cfg
    lines = [f"source: {source}", *notes]
    span = (
        result.last_t - result.first_t
        if result.first_t is not None and result.last_t is not None
        else 0.0
    )
    lines.append(
        f"{result.frames_seen} frames seen over {span:.1f} s, {result.frames_processed} processed, "
        f"{result.frames_failed} undecodable; odometry {result.odom_messages}, "
        f"yaw rate {result.yaw_messages}"
    )
    lines.append(
        "config: " + ", ".join(f"{name}={getattr(cfg, name)}" for name in LapConfig.field_names())
    )
    if result.reference_time_s is None:
        lines.append(
            "no reference captured: the speed never reached start_speed_mps "
            f"({cfg.start_speed_mps} m/s) or every frame was blank"
        )
    else:
        lines.append(
            f"reference captured at t = {result.reference_time_s - (result.first_t or 0.0):.2f} s "
            "from the first message"
        )
    lines.append("")
    if result.events:
        lines.append("lap  lap_time_s  best_s  peak_sim  distance_m  turns   crossing_t_s")
        t0 = result.first_t or 0.0
        for e in result.events:
            turns = "   --" if e.heading_turns is None else f"{e.heading_turns:6.2f}"
            lines.append(
                f"{e.lap:3d}  {e.lap_time_s:10.2f}  {e.best_lap_s:6.2f}  {e.peak_similarity:8.3f}  "
                f"{e.distance_m:10.2f}  {turns}  {e.crossing_time_s - t0:12.2f}"
            )
    lines.append(f"laps counted: {len(result.events)}")
    misses = near_misses(result.trace, result.events, cfg)
    if misses:
        t0 = result.first_t or 0.0
        lines.append(
            "near misses (rose to rearm_threshold or above, no lap): "
            + ", ".join(f"{t - t0:.1f} s ({s:.3f})" for t, s in misses)
        )
    lines.append("")
    lines.extend(render_trace(result, bucket_s))
    return "\n".join(lines)


def write_trace_csv(path: Path, result: ReplayResult) -> None:
    laps = {e.crossing_time_s: e.lap for e in result.events}
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "similarity", "lap_crossing"])
        for t, s in result.trace:
            w.writerow([f"{t:.6f}", f"{s:.6f}", laps.get(t, "")])


# ----------------------------------------------------------------------------- command line


def parse_params(pairs: list[str], base: LapConfig) -> LapConfig:
    """--param NAME=VALUE overrides on top of the defaults, with the field's own type."""
    types = {f.name: f.type for f in LapConfig.__dataclass_fields__.values()}
    updates: dict[str, Any] = {}
    for pair in pairs:
        name, sep, raw = pair.partition("=")
        name = name.strip()
        if not sep or name not in types:
            raise SystemExit(
                f"--param wants NAME=VALUE with NAME one of {LapConfig.field_names()}, got {pair!r}"
            )
        kind = types[name]
        try:
            if kind in ("bool", bool):
                if raw.strip().lower() not in ("true", "false", "1", "0"):
                    raise ValueError("expected true or false")
                updates[name] = raw.strip().lower() in ("true", "1")
            elif kind in ("int", int):
                updates[name] = int(raw)
            else:
                updates[name] = float(raw)
        except ValueError as exc:
            raise SystemExit(f"--param {name}: {exc}") from exc
    cfg = replace(base, **updates)
    try:
        cfg.validate()
    except ValueError as exc:
        raise SystemExit(f"bad parameters: {exc}") from exc
    return cfg


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("bag", nargs="?", type=Path, help="an .mcap file or a bag directory")
    parser.add_argument("--images", type=Path, help="a folder of images instead of a bag")
    parser.add_argument("--fps", type=float, default=15.0, help="folder mode frame rate (15)")
    parser.add_argument("--odom-csv", type=Path, help="folder mode odometry: t_s,speed_mps,x_m")
    parser.add_argument("--image-topic", help="default: the only /camera/*/compressed in the bag")
    parser.add_argument("--odom-topic", default="/odom/wheel", help="speed and path (%(default)s)")
    parser.add_argument("--yaw-topic", default="/odom", help="yaw rate (%(default)s)")
    parser.add_argument(
        "--param",
        action="append",
        default=[],
        metavar="NAME=VALUE",
        help="a lap_counter_node parameter: " + ", ".join(LapConfig.field_names()),
    )
    parser.add_argument(
        "--rate-hz", type=float, default=DEFAULT_RATE_HZ, help="frame rate limit (%(default)s)"
    )
    parser.add_argument(
        "--decode-reduction",
        type=int,
        default=DEFAULT_REDUCTION,
        choices=(1, 2, 4, 8),
        help="JPEG decode scale 1/N (%(default)s)",
    )
    parser.add_argument("--bucket-s", type=float, default=1.0, help="trace row length (1 s)")
    parser.add_argument("--trace-csv", type=Path, help="write every similarity sample here")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if (args.bag is None) == (args.images is None):
        raise SystemExit("give either a bag or --images FOLDER")
    cfg = parse_params(args.param, LapConfig())
    notes: list[str] = []
    if args.images is not None:
        if not args.images.is_dir():
            raise SystemExit(f"no such folder: {args.images}")
        source = f"{args.images} (images at {args.fps:g} fps)"
        if args.odom_csv is None:
            cfg = replace(cfg, start_speed_mps=0.0, min_lap_distance_m=0.0)
            notes.append(
                "no --odom-csv: start gate and distance gate switched off "
                "(start_speed_mps=0, min_lap_distance_m=0); min_lap_time_s still applies"
            )
        messages: Iterable[Msg] = iter_folder_messages(args.images, args.fps, args.odom_csv)
    else:
        if not args.bag.exists():
            raise SystemExit(f"no such bag: {args.bag}")
        counts = list_bag_topics(args.bag)
        image_topic = pick_image_topic(counts, args.image_topic)
        source = f"{args.bag} ({image_topic})"
        if counts.get(args.odom_topic, 0) == 0:
            cfg = replace(cfg, start_speed_mps=0.0, min_lap_distance_m=0.0)
            notes.append(
                f"no {args.odom_topic} in the bag: start gate and distance gate switched off"
            )
        if counts.get(args.yaw_topic, 0) == 0:
            notes.append(f"no {args.yaw_topic} in the bag: heading turns not available")
        messages = iter_bag_messages(args.bag, image_topic, args.odom_topic, args.yaw_topic)
    result = replay(messages, cfg, args.rate_hz, args.decode_reduction)
    print(format_report(result, source, notes, args.bucket_s))
    if args.trace_csv:
        write_trace_csv(args.trace_csv, result)
        print(f"\ntrace written to {args.trace_csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
