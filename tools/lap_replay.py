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

Thumbnails instead of JPEGs. A bag with `/lap/thumb` (sensor_msgs/Image, mono8, the 64 x 48
thumbnail the counter itself compared) is replayed from it, which is exact; `--thumbs FILE.npz`
does the same from a file written by `--save-thumbs`. Without thumbnails the recorded camera
stream is decoded: approximate, because the counter saw the camera's own frames at 12 to 15 per
second and the record stream has only 5 or 10, and the report says so.

    uv run --project tools --extra vision python tools/lap_replay.py BAG --image-topic \\
        /camera/usb/record/compressed --t-from 1795406671 --t-to 1795406692 \\
        --param start_speed_mps=0 --peak-table --save-thumbs thumbs.npz

`--peak-table` prints, per lap crossing, the similarity peak against the standing reference (the
counter's own) and against a driving reference (the frame at the first crossing), and the
highest similarity away from any crossing (the alias level), so a threshold can be judged
against both (fix checklist 2026-11-22, section 3.5). `--lap-length-m` and `--first-crossing-m`
say where the crossings are; without them the length is estimated from the counted laps.
`--t-from` and `--t-to` cut the bag to a log-time window (a session bag holds several runs);
`start_speed_mps=0` makes the first frame of the window the reference, as the launch's reset
does on the car.

docs/notes/lap-counter-2026-11-12.md has the tuning procedure.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
from collections.abc import Callable, Iterable, Iterator
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

import numpy as np

# The counter lives in the ROS package (pure numpy, no ROS needed to import it).
_PERCEPTION = Path(__file__).resolve().parents[1] / "ros_ws" / "src" / "racer_perception"
if str(_PERCEPTION) not in sys.path:
    sys.path.insert(0, str(_PERCEPTION))

from racer_perception.lap_counter import (
    STATE_RUNNING,
    LapConfig,
    LapCounter,
    LapEvent,
    make_signature,
    resize_area,
)
from racer_perception.lap_frames import decode_gray
from racer_perception.throttle import FrameThrottle

#: The frozen v0.4 pre-registration (docs/notes/preregistration-pnp-v0.4.md). Checked on every
#: --split-run; a mismatch is a refusal, never a warning.
PREREG_V04_PATH = Path(__file__).resolve().parents[1] / "evaluation" / "prereg" / "pnp-v0.4.yaml"
PREREG_V04_SHA256 = "78510c1423278f80c42511e6caab095818598af6db6a284dd5479f74a03a1d63"

THUMB_TOPIC = "/lap/thumb"
DEFAULT_RATE_HZ = 15.0
DEFAULT_REDUCTION = 4
IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png")
CHART_WIDTH = 40


@dataclass(frozen=True)
class Msg:
    """One input message on the replay clock. kind: image | thumb | odom | yaw. A thumb carries the
    counter's own gray thumbnail (a 2-D uint8 array), so it is not decoded and not rate limited."""

    kind: str
    t: float
    data: Any = None  # image: bytes; thumb: 2-D uint8; odom: (speed_mps, x_m or None); yaw: rps


@dataclass(frozen=True)
class FrameSample:
    t: float
    signature: Any  # np.ndarray, or None for a blank frame
    distance_m: float
    speed_mps: float


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
    #: Every processed frame (time, signature, path length of the counter's odometry, speed), kept
    #: when `replay(keep_frames=True)`: what the peak table is made from.
    frames: list[FrameSample] = field(default_factory=list)
    #: "image" (decoded JPEGs: approximate) or "thumb" (the counter's own thumbnails: exact).
    frame_kind: str = "image"


# ----------------------------------------------------------------------------- the replay


def replay(
    messages: Iterable[Msg],
    cfg: LapConfig,
    rate_hz: float = DEFAULT_RATE_HZ,
    reduction: int = DEFAULT_REDUCTION,
    decode: Callable[[bytes, int], Any] = decode_gray,
    keep_frames: bool = False,
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
        elif m.kind in ("image", "thumb"):
            out.frames_seen += 1
            if m.kind == "thumb":
                out.frame_kind = "thumb"
                gray = m.data
            else:
                if not throttle.accept(m.t):
                    continue
                try:
                    gray = decode(m.data, reduction)
                except Exception:  # noqa: BLE001 - one bad frame must not end a replay
                    out.frames_failed += 1
                    continue
            if keep_frames:
                out.frames.append(
                    FrameSample(
                        m.t,
                        make_signature(gray, cfg),
                        counter.distance_m,
                        counter.speed_mps,
                    )
                )
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


# ----------------------------------------------------------------------------- split runs


@dataclass(frozen=True)
class SplitRun:
    """A whole lap run of the v0.4 yaml: one bag and an absolute log-time window."""

    name: str
    bag: Path
    t_from: float
    t_to: float
    role: str  # lap role: dev | test | unused


def load_split_run(name: str, yaml_path: Path | None = None) -> SplitRun:
    """Resolve a run name (L0, X3, K1, ...) of `lap_runs` in the v0.4 yaml to its bag and its
    log-time window. A run given as parts (X2a, X2b) spans from the first part's start to the
    last part's end (the guard between them included). Refuses a yaml whose SHA-256 is not the
    registered one. Replaces brief 36's loader only until that is on main."""
    import yaml

    path = yaml_path or PREREG_V04_PATH
    raw = path.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    if digest != PREREG_V04_SHA256:
        raise SystemExit(
            f"{path}: SHA-256 {digest} is not the registered {PREREG_V04_SHA256}; refusing"
        )
    doc = yaml.safe_load(raw)["split_2026_11_14"]
    roles = {n: role for role, names in doc["lap_runs"].items() for n in names}
    if name not in roles:
        raise SystemExit(f"--split-run {name!r}: not a lap run; known: {sorted(roles)}")
    parts = []
    for bag, entry in doc["bags"].items():
        for seg in entry.get("segments", []):
            seg_name = seg["name"]
            base = seg_name[:-1] if seg_name[-1] in "ab" and seg_name[:-1] in roles else seg_name
            if base == name:
                parts.append((bag, entry["first_scan_log_time_s"], seg))
    if not parts:
        raise SystemExit(f"--split-run {name!r}: no segment in the yaml")
    bags = {p[0] for p in parts}
    if len(bags) != 1:
        raise SystemExit(f"--split-run {name!r}: parts in several bags: {sorted(bags)}")
    bag, first, _ = parts[0]
    return SplitRun(
        name=name,
        bag=Path(doc["bags_root"]) / bag,
        t_from=first + min(p[2]["from"] for p in parts),
        t_to=first + max(p[2]["to"] for p in parts),
        role=roles[name],
    )


def clip_messages(messages: Iterable[Msg], t_from: float, t_to: float) -> Iterator[Msg]:
    """Only the messages inside [t_from, t_to] (log time)."""
    for m in messages:
        if t_from <= m.t <= t_to:
            yield m


def load_reference(path: Path) -> tuple[int, list[float]]:
    """(lap count, lap times) of a lapref JSON: `lap_count` and `lap_times_s`."""
    doc = json.loads(path.read_text())
    times = doc.get("lap_times_s", doc.get("lap_times"))
    count = doc.get("lap_count", None if times is None else len(times))
    if count is None:
        raise SystemExit(f"{path}: no lap_count / lap_times_s")
    return int(count), [float(t) for t in (times or [])]


def reference_report(events: list[LapEvent], ref_count: int, ref_times: list[float]) -> list[str]:
    """Counts only (lapref's crossing stamps are not the gate's, so laps are compared by number
    and time, never by stamp). matched = min(counted, reference); missed = reference laps the
    counter lacks; false = counted laps beyond the reference."""
    n = len(events)
    lines = [
        (
            f"reference: {ref_count} laps; counter: {n}; matched {min(n, ref_count)}, "
            f"missed {max(0, ref_count - n)}, false {max(0, n - ref_count)}"
        )
    ]
    if ref_times and n == ref_count:
        diffs = [e.lap_time_s - r for e, r in zip(events, ref_times, strict=True)]
        lines.append(
            "lap time minus reference, s: "
            + ", ".join(f"{d:+.2f}" for d in diffs)
            + f"  (max |d| {max(abs(d) for d in diffs):.2f})"
        )
    return lines


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


def thumb_from_image(msg: Any) -> np.ndarray | None:
    """The 2-D uint8 array of a mono8 sensor_msgs/Image, or None when it is not one."""
    if getattr(msg, "encoding", "mono8") != "mono8":
        return None
    h, w, step = int(msg.height), int(msg.width), int(getattr(msg, "step", msg.width))
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if h <= 0 or w <= 0 or step < w or raw.size < h * step:
        return None
    return raw[: h * step].reshape(h, step)[:, :w].copy()


def pick_image_topic(counts: dict[str, int], wanted: str | None) -> str:
    if wanted:
        if wanted not in counts:
            raise SystemExit(f"topic {wanted} is not in the bag; it has: {sorted(counts)}")
        return wanted
    if counts.get(THUMB_TOPIC, 0) > 0:  # the counter's own thumbnails: exact, always preferred
        return THUMB_TOPIC
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
    """Decode the three topics of an mcap bag, in file (log-time) order. No ROS install needed.
    `image_topic` may be THUMB_TOPIC (sensor_msgs/Image, mono8): those become "thumb" messages."""
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
                    if channel.topic == image_topic and image_topic == THUMB_TOPIC:
                        thumb = thumb_from_image(msg)
                        if thumb is None:
                            data.decode_errors += 1
                            continue
                        yield Msg("thumb", t, thumb)
                    elif channel.topic == image_topic:
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


def save_thumbs(path: Path, messages: Iterable[Msg], reduction: int = DEFAULT_REDUCTION) -> int:
    """Write the frames of `messages` as the counter's 64 x 48 thumbnails (area-resized, uint8)
    with the odometry, as an npz that `--thumbs` replays: t_s, thumbs (N, 48, 64), odom_t_s,
    odom_speed_mps, odom_x_m (NaN where unknown), yaw_t_s, yaw_rate_rps. Returns the frame count."""
    cfg = LapConfig()
    exact = True  # every frame was already a thumbnail of the counter's own
    ts: list[float] = []
    thumbs: list[np.ndarray] = []
    odom: list[tuple[float, float, float]] = []
    yaw: list[tuple[float, float]] = []
    for m in messages:
        if m.kind == "odom":
            speed, x = m.data
            odom.append((m.t, speed, math.nan if x is None else x))
        elif m.kind == "yaw":
            yaw.append((m.t, m.data))
        elif m.kind in ("image", "thumb"):
            try:
                gray = m.data if m.kind == "thumb" else decode_gray(m.data, reduction)
            except Exception:  # noqa: BLE001, S112 - an undecodable frame is left out
                continue
            exact = exact and m.kind == "thumb"
            ts.append(m.t)
            thumbs.append(
                np.rint(resize_area(gray, cfg.thumb_width_px, cfg.thumb_height_px))
                .clip(0, 255)
                .astype(np.uint8)
            )
    o = np.array(odom, dtype=np.float64).reshape(-1, 3)
    y = np.array(yaw, dtype=np.float64).reshape(-1, 2)
    np.savez_compressed(
        path,
        t_s=np.array(ts, dtype=np.float64),
        thumbs=np.stack(thumbs)
        if thumbs
        else np.zeros((0, cfg.thumb_height_px, cfg.thumb_width_px), np.uint8),
        odom_t_s=o[:, 0],
        odom_speed_mps=o[:, 1],
        odom_x_m=o[:, 2],
        yaw_t_s=y[:, 0],
        yaw_rate_rps=y[:, 1],
        exact=np.array(exact),
    )
    return len(ts)


def thumbs_are_exact(path: Path) -> bool:
    """Whether the npz holds the counter's own thumbnails (a bag with /lap/thumb) and not ones made
    from the recorded JPEG stream. A file without the flag is taken as approximate."""
    with np.load(path) as z:
        return bool(z["exact"]) if "exact" in z.files else False


def iter_thumbs_messages(path: Path) -> Iterator[Msg]:
    """Messages of an npz written by save_thumbs, merged by time (odometry first on a tie)."""
    with np.load(path) as z:
        t, thumbs = z["t_s"], z["thumbs"]
        ot, ov, ox = z["odom_t_s"], z["odom_speed_mps"], z["odom_x_m"]
        yt, yr = z["yaw_t_s"], z["yaw_rate_rps"]
        msgs = [
            Msg("odom", float(a), (float(b), None if math.isnan(c) else float(c)))
            for a, b, c in zip(ot, ov, ox, strict=True)
        ]
        msgs += [Msg("yaw", float(a), float(b)) for a, b in zip(yt, yr, strict=True)]
        msgs += [Msg("thumb", float(a), thumbs[i]) for i, a in enumerate(t)]
    order = {"odom": 0, "yaw": 1, "thumb": 2}
    msgs.sort(key=lambda m: (m.t, order[m.kind]))
    yield from msgs


# ----------------------------------------------------------------------------- the peak table

#: Conventions of the peak table (fix checklist 2026-11-22, section 9): a true crossing is the best
#: similarity within this many metres of the expected station; a peak is the highest value within
#: PEAK_HALF_WIDTH_M of the crossing frame; an off-mark frame is moving, past the first metre and
#: farther than OFF_MARK_M from every crossing.
CROSSING_SEARCH_M = 0.7
PEAK_HALF_WIDTH_M = 0.35
OFF_MARK_M = 0.6
MOVING_MPS = 0.3


@dataclass
class Crossing:
    lap: int
    t: float
    station_m: float
    standing_peak: float
    driving_peak: float | None  # None for lap 1: its frame is the driving reference


@dataclass
class PeakTable:
    lap_length_m: float
    first_station_m: float
    crossings: list[Crossing]
    #: reference name -> (highest off-mark similarity, its 99th percentile, off-mark frames)
    off_mark: dict[str, tuple[float, float, int]]
    threshold: float
    t_ref: float = 0.0

    def peaks(self, reference: str) -> list[float]:
        if reference == "standing":
            return [c.standing_peak for c in self.crossings]
        return [c.driving_peak for c in self.crossings if c.driving_peak is not None]


def estimate_lap_length(distances: list[float]) -> float | None:
    """The lap length from the distances of counted laps, folding a lap that spans two or more
    crossings (a missed one) back to one: each distance is divided by the nearest whole number of
    laps of the running estimate, which starts at the median of the shorter half. None for no laps."""
    ds = [d for d in distances if d > 0.0 and math.isfinite(d)]
    if not ds:
        return None
    low = sorted(ds)[: (len(ds) + 1) // 2]  # start from the short laps: a long one is a missed lap
    length = low[len(low) // 2]
    for _ in range(3):
        folded = [d / max(1, round(d / length)) for d in ds]
        length = sorted(folded)[len(folded) // 2]
    return length


def peak_table(
    result: ReplayResult, lap_length_m: float | None = None, first_station_m: float | None = None
) -> PeakTable | None:
    """True-crossing peaks and the off-mark level against the standing and the driving reference,
    from `replay(..., keep_frames=True)`. Distances are the counter's odometry from the reference
    frame. The lap length and the first crossing come from the arguments, else from the counted
    laps. None when there is no reference frame or no way to place the crossings."""
    frames = [
        f
        for f in result.frames
        if result.reference_time_s is not None and f.t >= result.reference_time_s
    ]
    if len(frames) < 3 or frames[0].signature is None:
        return None
    t = np.array([f.t for f in frames])
    d = np.array([f.distance_m for f in frames]) - frames[0].distance_m
    v = np.array([f.speed_mps for f in frames])
    ok = np.array([f.signature is not None for f in frames])
    sigs = np.stack(
        [
            f.signature if f.signature is not None else np.zeros_like(frames[0].signature)
            for f in frames
        ]
    )
    if lap_length_m is None:
        lap_length_m = estimate_lap_length([e.distance_m for e in result.events])
    if first_station_m is None and result.events:
        j = int(np.argmin(np.abs(t - result.events[0].crossing_time_s)))
        first_station_m = float(d[j])
    if lap_length_m is None or first_station_m is None:
        return None
    standing = sigs @ sigs[0]
    cross: list[int] = []
    expected = first_station_m
    while expected < d[-1] - 0.3:
        w = np.nonzero((np.abs(d - expected) < CROSSING_SEARCH_M) & ok)[0]
        if len(w) == 0:
            break
        j = int(w[np.argmax(standing[w])])
        cross.append(j)
        expected = float(d[j]) + lap_length_m
    if not cross:
        return None
    driving = sigs @ sigs[cross[0]]
    near = np.zeros(len(d), dtype=bool)
    for j in cross:
        near |= np.abs(d - d[j]) < OFF_MARK_M
    mask = (v > MOVING_MPS) & (d > 1.0) & ~near & ok

    def peak(sim: np.ndarray, j: int) -> float:
        sel = (np.abs(d - d[j]) < PEAK_HALF_WIDTH_M) & ok
        return float(sim[sel].max())

    def off(sim: np.ndarray) -> tuple[float, float, int]:
        vals = sim[mask]
        if len(vals) == 0:
            return (math.nan, math.nan, 0)
        return (float(vals.max()), float(np.percentile(vals, 99)), len(vals))

    crossings = [
        Crossing(
            lap=i + 1,
            t=float(t[j]),
            station_m=float(d[j]),
            standing_peak=peak(standing, j),
            driving_peak=None if i == 0 else peak(driving, j),
        )
        for i, j in enumerate(cross)
    ]
    return PeakTable(
        lap_length_m=lap_length_m,
        first_station_m=first_station_m,
        crossings=crossings,
        off_mark={"standing": off(standing), "driving": off(driving)},
        threshold=result.cfg.similarity_threshold,
        t_ref=float(t[0]),
    )


def render_peak_table(table: PeakTable, frame_kind: str, fps: float | None) -> list[str]:
    kind = (
        "exact: the counter's own thumbnails"
        if frame_kind == "thumb"
        else "APPROXIMATE: decoded from the recorded camera stream"
        + ("" if fps is None else f" at {fps:.1f} frames/s")
        + "; the live counter saw other frames, so peaks here are lower bounds"
    )
    lines = [
        (
            f"peak table ({kind}); crossings every {table.lap_length_m:.2f} m from "
            f"{table.first_station_m:.2f} m"
        ),
        "lap  station_m  t_s (from reference)  standing_peak  driving_peak",
    ]
    t0 = table.t_ref
    for c in table.crossings:
        drv = "  (reference)" if c.driving_peak is None else f"{c.driving_peak:14.3f}"
        flag = " <" if c.standing_peak < table.threshold else ""
        lines.append(
            f"{c.lap:3d}  {c.station_m:9.2f}  {c.t - t0:.2f}  {c.standing_peak:13.3f}{flag:2s}{drv}"
        )
    for name in ("standing", "driving"):
        peaks = table.peaks(name)
        if not peaks:
            continue
        hi, p99, n = table.off_mark[name]
        under = sum(1 for p in peaks if p < table.threshold)
        lines.append(
            f"{name:8s} reference: true peaks n {len(peaks)}, min {min(peaks):.3f}, median "
            f"{sorted(peaks)[len(peaks) // 2]:.3f}, max {max(peaks):.3f}; under the threshold "
            f"{table.threshold:g}: {under}; off-mark highest {hi:.3f}, p99 {p99:.3f} over {n} frames; "
            f"margin (min true peak minus highest off-mark) {min(peaks) - hi:+.3f}"
        )
    lines.append(
        f"('<' marks a peak under similarity_threshold {table.threshold:g}: a crossing the counter would miss)"
    )
    return lines


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
    parser.add_argument(
        "--split-run",
        metavar="NAME",
        help="a lap run of evaluation/prereg/pnp-v0.4.yaml (L0, X0, K1, ...): its bag and window "
        "(the yaml's SHA-256 is checked)",
    )
    parser.add_argument(
        "--reference", type=Path, help="a lapref JSON: print matched, missed and false counts"
    )
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
    parser.add_argument(
        "--thumbs", type=Path, help="an npz of thumbnails (--save-thumbs) instead of a bag"
    )
    parser.add_argument("--t-from", type=float, help="bag: first log time (epoch s) to replay")
    parser.add_argument("--t-to", type=float, help="bag: last log time (epoch s) to replay")
    parser.add_argument(
        "--save-thumbs", type=Path, help="write the replayed frames as thumbnails (npz)"
    )
    parser.add_argument(
        "--peak-table",
        action="store_true",
        help="true-crossing peaks against the standing and the driving reference, off-mark level",
    )
    parser.add_argument("--lap-length-m", type=float, help="peak table: the lap length (m)")
    parser.add_argument(
        "--first-crossing-m",
        type=float,
        help="peak table: path length from the reference to the first crossing (m)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    window: SplitRun | None = None
    sources = [args.bag is not None, args.images is not None, args.thumbs is not None]
    if args.split_run:
        if any(sources):
            raise SystemExit("--split-run replaces the bag argument, --images and --thumbs")
        window = load_split_run(args.split_run)
        args.bag = window.bag
    elif sum(sources) != 1:
        raise SystemExit(
            "give either a bag or --images FOLDER or --thumbs FILE, only one of them "
            "(or --split-run NAME instead)"
        )
    cfg = parse_params(args.param, LapConfig())
    notes: list[str] = []
    if args.thumbs is not None:
        if not args.thumbs.is_file():
            raise SystemExit(f"no such file: {args.thumbs}")
        source = f"{args.thumbs} (thumbnails)"
        messages: Iterable[Msg] = iter_thumbs_messages(args.thumbs)
    elif args.images is not None:
        if not args.images.is_dir():
            raise SystemExit(f"no such folder: {args.images}")
        source = f"{args.images} (images at {args.fps:g} fps)"
        if args.odom_csv is None:
            cfg = replace(cfg, start_speed_mps=0.0, min_lap_distance_m=0.0)
            notes.append(
                "no --odom-csv: start gate and distance gate switched off "
                "(start_speed_mps=0, min_lap_distance_m=0); min_lap_time_s still applies"
            )
        messages = iter_folder_messages(args.images, args.fps, args.odom_csv)
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
        if window is not None:
            notes.append(
                f"split run {window.name} (lap role {window.role}): log time "
                f"{window.t_from:.3f} to {window.t_to:.3f} of {args.bag.name}"
            )
            messages = clip_messages(messages, window.t_from, window.t_to)
    if args.t_from is not None or args.t_to is not None:
        lo = -math.inf if args.t_from is None else args.t_from
        hi = math.inf if args.t_to is None else args.t_to
        notes.append(f"window: log time {lo:.3f} to {hi:.3f}")
        messages = clip_messages(messages, lo, hi)
    if args.save_thumbs:
        messages = list(messages)
    result = replay(messages, cfg, args.rate_hz, args.decode_reduction, keep_frames=args.peak_table)
    if args.thumbs is not None and not thumbs_are_exact(args.thumbs):
        result.frame_kind = "image"
        notes.append(
            "the thumbnails were made from the recorded camera stream: approximate, not the "
            "counter's own frames"
        )
    print(format_report(result, source, notes, args.bucket_s))
    if args.peak_table:
        table = peak_table(result, args.lap_length_m, args.first_crossing_m)
        span = (result.last_t or 0.0) - (result.first_t or 0.0)
        fps = result.frames_processed / span if span > 0 else None
        print()
        if table is None:
            print(
                "peak table: no crossings could be placed (no reference frame, or no counted lap "
                "to take the lap length from): give --lap-length-m and --first-crossing-m"
            )
        else:
            print("\n".join(render_peak_table(table, result.frame_kind, fps)))
    if args.reference:
        ref_count, ref_times = load_reference(args.reference)
        print("\n" + "\n".join(reference_report(result.events, ref_count, ref_times)))
    if args.trace_csv:
        write_trace_csv(args.trace_csv, result)
        print(f"\ntrace written to {args.trace_csv}")
    if args.save_thumbs:
        n = save_thumbs(args.save_thumbs, messages, args.decode_reduction)
        print(f"\n{n} thumbnails written to {args.save_thumbs}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
