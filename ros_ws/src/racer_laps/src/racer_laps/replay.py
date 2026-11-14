"""Run the lap memory over a recorded bag and print the laps.

    python -m racer_laps.replay --bag <dir or .mcap>                         # map pose
    python -m racer_laps.replay --bag <dir> --lap-source start_scan          # start-scan gate v2
    python -m racer_laps.replay --m-r --split <split.yaml> --bags-root <dir> \\
        --v1-laps tests/fixtures/gate_v1_laps_2026-11-13.json                 # the M-R table

`--lap-source map_pose` (default) needs `/tf` (and `/tf_static` if the map to odom or odom to
base_link link is static): bags from stack runs with `mapping:=true`. `/pnp/planner/status`,
`/safety/events`, `/odom/wheel` and `/drive_raw` are used when present. Without planner status the
start line is set at the first pose and every lap is unclean (there is no plan clearance to judge),
which the output says.

`--lap-source start_scan` needs `/scan`, `/odom/wheel` and `/drive` (no `/tf`): the start scan is
the first scan of the segment with wheel speed under 0.05 m/s that is followed by motion, or the
segment's first scan if the car is already moving. `--from-s` and `--to-s` (seconds after the
first `/scan`, the axis of tools/pnp_replay/bag.py) or `--split <file> --segment T2a+T2b` cut the
segment; `--compare-pose` runs the map-pose detector over the same segment and starts it at the
same instant. Needs the `replay` extra (mcap, mcap-ros2-support, pyyaml, jsonschema), no ROS.

The decision logic is exactly the node's: `racer_laps.engine.LapMemory`, fed at 20 Hz of bag time
(map pose) or message by message (start scan).
"""

from __future__ import annotations

import argparse
import logging
import math
import sys
import time
from collections import deque
from collections.abc import Iterable, Iterator, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from racer_laps.engine import LAP_SOURCES, LapConfig, LapMemory, LapResult
from racer_laps.gate import GateConfig
from racer_laps.lap_detector import wrap_angle
from racer_laps.scan_match import scan_to_points
from racer_laps.vehicle import LapVehicle, load_live_vehicle

log = logging.getLogger(__name__)

TF_TOPICS = ("/tf", "/tf_static")
PLANNER_TOPIC = "/pnp/planner/status"
EVENTS_TOPIC = "/safety/events"
ODOM_TOPIC = "/odom/wheel"
DRIVE_RAW_TOPIC = "/drive_raw"
DRIVE_TOPIC = "/drive"
SCAN_TOPIC = "/scan"
WANTED = (
    *TF_TOPICS,
    PLANNER_TOPIC,
    EVENTS_TOPIC,
    ODOM_TOPIC,
    DRIVE_RAW_TOPIC,
    DRIVE_TOPIC,
    SCAN_TOPIC,
)

#: PlannerStatus.STATE_DRIVING
STATE_DRIVING = 1


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float

    def compose(self, other: Pose2D) -> Pose2D:
        """self * other: `other` expressed in self's parent frame."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(
            self.x + c * other.x - s * other.y,
            self.y + s * other.x + c * other.y,
            wrap_angle(self.yaw + other.yaw),
        )

    def inverse(self) -> Pose2D:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(-(c * self.x + s * self.y), -(-s * self.x + c * self.y), -self.yaw)


def quat_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class TfTree:
    """The latest planar transform of every (parent, child) edge, resolved by search."""

    def __init__(self) -> None:
        self._edges: dict[tuple[str, str], tuple[Pose2D, float, bool]] = {}

    def update(self, parent: str, child: str, pose: Pose2D, t_s: float, static: bool) -> None:
        self._edges[(parent.lstrip("/"), child.lstrip("/"))] = (pose, t_s, static)

    def lookup(self, parent: str, child: str, now_s: float) -> tuple[Pose2D, float] | None:
        """The pose of `child` in `parent` and the age in seconds of the oldest non-static edge
        on the path, or None when no path exists."""
        parent, child = parent.lstrip("/"), child.lstrip("/")
        queue: deque[tuple[str, Pose2D, float]] = deque([(parent, Pose2D(0.0, 0.0, 0.0), 0.0)])
        seen = {parent}
        while queue:
            frame, acc, age = queue.popleft()
            if frame == child:
                return acc, age
            for (p, c), (pose, t_s, static) in self._edges.items():
                edge_age = 0.0 if static else now_s - t_s
                if p == frame and c not in seen:
                    seen.add(c)
                    queue.append((c, acc.compose(pose), max(age, edge_age)))
                elif c == frame and p not in seen:
                    seen.add(p)
                    queue.append((p, acc.compose(pose.inverse()), max(age, edge_age)))
        return None


@dataclass
class ReplayResult:
    laps: list[LapResult]
    n_pose_samples: int
    n_pose_missing: int
    saw_planner_status: bool
    safe_set_laps: int
    start_t_s: float | None


def run_replay(
    messages: Iterable[tuple[float, str, Any]],
    cfg: LapConfig | None = None,
    pose_rate_hz: float = 20.0,
    pose_timeout_s: float = 0.5,
    map_frame: str = "map",
    base_frame: str = "base_link",
    start_s: float | None = None,
) -> ReplayResult:
    """Feed decoded messages (log time in seconds, topic, message) through the lap memory.

    Messages are plain objects shaped like the ROS ones (attribute access), in log time order.
    """
    mem = LapMemory(cfg or LapConfig())
    tree = TfTree()
    period = 1.0 / pose_rate_hz
    next_tick: float | None = None
    laps: list[LapResult] = []
    n_samples = n_missing = 0
    saw_planner = False
    assumed_driving = False
    start_t: float | None = None
    first_t: float | None = None

    def tick(t_s: float) -> None:
        nonlocal n_samples, n_missing, start_t
        found = tree.lookup(map_frame, base_frame, t_s)
        if found is None or found[1] > pose_timeout_s:
            mem.on_pose_missing()
            n_missing += 1
            return
        pose = found[0]
        n_samples += 1
        was_started = mem.detector.started
        result = mem.on_pose(t_s, pose.x, pose.y, pose.yaw)
        if not was_started and mem.detector.started:
            start_t = t_s
        if result is not None:
            laps.append(result)

    for t_s, topic, msg in messages:
        if first_t is None:
            first_t = t_s
            next_tick = t_s
        # Sample the pose for every tick before this message: it only knows what came earlier.
        assert next_tick is not None
        while next_tick < t_s:
            if (
                not saw_planner
                and not assumed_driving
                and (start_s is None or next_tick >= start_s)
            ):
                # no planner status in this bag: lap 1 starts at the first pose (or at start_s)
                mem.on_planner_status(STATE_DRIVING, math.nan)
                assumed_driving = True
            tick(next_tick)
            next_tick += period
        if topic in TF_TOPICS:
            static = topic == "/tf_static"
            for tr in msg.transforms:
                q = tr.transform.rotation
                tree.update(
                    tr.header.frame_id,
                    tr.child_frame_id,
                    Pose2D(
                        tr.transform.translation.x,
                        tr.transform.translation.y,
                        quat_yaw(q.x, q.y, q.z, q.w),
                    ),
                    t_s,
                    static,
                )
        elif topic == PLANNER_TOPIC:
            saw_planner = True
            mem.on_planner_status(int(msg.state), float(msg.plan_min_clearance_m))
        elif topic == EVENTS_TOPIC:
            mem.on_safety_event(
                int(msg.phase), int(msg.severity), str(getattr(msg, "source", "ttc"))
            )
        elif topic == ODOM_TOPIC:
            mem.on_wheel_speed(float(msg.twist.twist.linear.x))
        elif topic == DRIVE_RAW_TOPIC:
            mem.on_drive_raw(float(msg.drive.speed), float(msg.drive.steering_angle))
    return ReplayResult(
        laps=laps,
        n_pose_samples=n_samples,
        n_pose_missing=n_missing,
        saw_planner_status=saw_planner,
        safe_set_laps=mem.safe_set.n_laps,
        start_t_s=start_t,
    )


def format_report(result: ReplayResult) -> str:
    lines = []
    if result.n_pose_samples == 0:
        lines.append(
            "no usable map -> base_link transform in this bag "
            f"({result.n_pose_missing} sample times without one): record a run with "
            "mapping:=true so /tf carries it"
        )
    for lap in result.laps:
        lines.append(
            f"lap {lap.lap_index}: {lap.lap_time_s:.2f} s  {lap.length_m:.2f} m  "
            f"cap {lap.cap_mps:.2f}  brakes {lap.safety_brakes}  "
            f"min clearance {lap.min_clearance_m:.3f}  "
            f"{'clean' if lap.clean else 'unclean'}  next cap {lap.next_cap_mps:.2f}"
        )
    lines.append(
        f"{len(result.laps)} laps from {result.n_pose_samples} pose samples "
        f"({result.n_pose_missing} missing), safe set holds {result.safe_set_laps}"
    )
    if not result.saw_planner_status:
        lines.append(
            f"note: no {PLANNER_TOPIC} in the bag, so lap 1 started at the first pose and no lap "
            "can be judged clean (no plan clearance)"
        )
    return "\n".join(lines)


def read_messages(path: str | Path) -> Iterator[tuple[float, str, Any]]:
    """Decode the wanted topics of an mcap bag (a file or a directory of .mcap files), in file
    order. Copes with the truncated, unindexed files a killed recorder leaves behind."""
    from mcap.exceptions import EndOfFile
    from mcap.records import Channel, Message, Schema
    from mcap.stream_reader import StreamReader
    from mcap_ros2.decoder import DecoderFactory

    p = Path(path)
    if p.is_file():
        files = [p]
    elif p.is_dir():
        files = sorted(p.glob("*.mcap"))
    else:
        raise FileNotFoundError(f"no such bag: {p}")
    if not files:
        raise FileNotFoundError(f"no .mcap file in {p}")
    factory = DecoderFactory()
    for file in files:
        schemas: dict[int, Schema] = {}
        channels: dict[int, Channel] = {}
        decoders: dict[int, Any] = {}
        with file.open("rb") as stream:
            reader = StreamReader(stream, skip_magic=False, emit_chunks=False, validate_crcs=False)
            try:
                for record in reader.records:
                    if isinstance(record, Schema):
                        schemas[record.id] = record
                    elif isinstance(record, Channel):
                        channels[record.id] = record
                    elif isinstance(record, Message):
                        channel = channels.get(record.channel_id)
                        if channel is None or channel.topic not in WANTED:
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
                            yield record.log_time / 1e9, channel.topic, decoder(record.data)
                        except (ValueError, KeyError, IndexError, UnicodeDecodeError) as error:
                            log.warning("%s: undecodable message (%s)", file.name, error)
            except EndOfFile:
                log.info("%s: no footer (the recorder was killed); used what was there", file.name)
            except Exception as error:  # noqa: BLE001 - a damaged tail must not lose good records
                log.warning("%s: stopped at an unreadable record (%s)", file.name, error)


@dataclass
class ScanReplayResult:
    laps: list[LapResult]
    start_t_s: float | None  # log time of the start scan's instant
    first_scan_log_s: float
    n_scans: int
    armed_scans: int
    matched_scans: int
    match_times_s: list[float]  # wall seconds of every on_scan call made while armed
    wheel_distance_m: float
    status: dict[str, object]

    def bag_time(self, t_s: float) -> float:
        """A log or stamp time on the bag axis (seconds after the first /scan)."""
        return t_s - self.first_scan_log_s


def find_start_time(
    odom: Sequence[tuple[float, float]], standstill_mps: float = 0.05
) -> float | None:
    """Log time of the start scan's instant: the last standstill odometry sample before the first
    motion, or the first sample if the car is already moving. `odom` is (log time, speed)."""
    if not odom:
        return None
    t_start = odom[0][0]
    for t, v in odom:
        if abs(v) < standstill_mps:
            t_start = t
        else:
            break
    return t_start


def default_start_scan_config(vehicle: LapVehicle) -> LapConfig:
    return LapConfig(
        lap_source="start_scan",
        gate=GateConfig(
            wheelbase_m=vehicle.wheelbase_m,
            yaw_gain_table=vehicle.yaw_gain_table,
            lidar_x_m=vehicle.mount.x_m,
            lidar_y_m=vehicle.mount.y_m,
        ),
    )


def run_replay_start_scan(
    messages: Sequence[tuple[float, str, Any]],
    vehicle: LapVehicle,
    cfg: LapConfig | None = None,
    window_s: tuple[float, float] | None = None,
) -> ScanReplayResult:
    """The start-scan gate over decoded messages (log time, topic, message), in log time order.

    `window_s` is (from, to) in seconds after the first /scan. Odometry and scans use their header
    stamps for time (the node does too); the order of delivery is log time."""
    scans = [m for m in messages if m[1] == SCAN_TOPIC]
    if not scans:
        raise ValueError("no /scan in the bag")
    first_scan_log = scans[0][0]
    lo, hi = (-math.inf, math.inf) if window_s is None else window_s
    inside = [m for m in messages if lo <= m[0] - first_scan_log <= hi]
    odom = [(t, float(msg.twist.twist.linear.x)) for t, topic, msg in inside if topic == ODOM_TOPIC]
    t_start = find_start_time(odom)
    mem = LapMemory(cfg or default_start_scan_config(vehicle))
    laps: list[LapResult] = []
    n_scans = 0
    timing: list[float] = []
    wheel_m = 0.0
    last_odom_t: float | None = None
    started = False
    saw_planner = any(topic == PLANNER_TOPIC for _t, topic, _m in inside)
    for t_log, topic, msg in inside:
        if t_start is not None and t_log < t_start:
            continue
        if not started and not saw_planner and topic == SCAN_TOPIC:
            mem.on_planner_status(STATE_DRIVING, math.nan)  # no planner status in the bag
            started = True
        if topic == ODOM_TOPIC:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            v = float(msg.twist.twist.linear.x)
            mem.on_wheel_odom(stamp, v)
            if last_odom_t is not None and stamp > last_odom_t:
                wheel_m += max(0.0, v) * min(stamp - last_odom_t, 1.0)
            last_odom_t = stamp
        elif topic == DRIVE_TOPIC:
            mem.on_drive(float(msg.drive.steering_angle))
        elif topic == DRIVE_RAW_TOPIC:
            mem.on_drive_raw(float(msg.drive.speed), float(msg.drive.steering_angle))
        elif topic == PLANNER_TOPIC:
            mem.on_planner_status(int(msg.state), float(msg.plan_min_clearance_m))
        elif topic == EVENTS_TOPIC:
            mem.on_safety_event(int(msg.phase), int(msg.severity), str(msg.source))
        elif topic == SCAN_TOPIC:
            n_scans += 1
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            sweep = float(msg.scan_time) if msg.scan_time > 0.0 else 0.0
            pts = scan_to_points(
                msg.ranges,
                float(msg.angle_min),
                float(msg.angle_increment),
                float(msg.range_min),
                float(msg.range_max),
                vehicle.mount,
                vehicle.body,
            )
            armed_before = mem.gate is not None and mem.gate.armed
            t0 = time.perf_counter()
            result = mem.on_scan(stamp + 0.5 * sweep, pts)
            if armed_before:
                timing.append(time.perf_counter() - t0)
            if result is not None:
                laps.append(result)
    assert mem.gate is not None
    return ScanReplayResult(
        laps=laps,
        start_t_s=t_start,
        first_scan_log_s=first_scan_log,
        n_scans=n_scans,
        armed_scans=mem.gate.n_armed_scans,
        matched_scans=mem.gate.n_matched_scans,
        match_times_s=timing,
        wheel_distance_m=wheel_m,
        status=mem.status(),
    )


def segment_ranges(split_file: str | Path, bag_name: str, segment: str) -> tuple[float, float]:
    """(from, to) in bag seconds of a segment, or of consecutive segments joined with `+`
    (T2a+T2b is T2a's start to T2b's end). Reads the pre-registered split file, names only."""
    import yaml

    data = yaml.safe_load(Path(split_file).read_text(encoding="utf-8"))
    entries = {e["name"]: e for e in data["bags"][bag_name]["segments"]}
    parts = [entries[name] for name in segment.split("+")]
    return float(min(e["from"] for e in parts)), float(max(e["to"] for e in parts))


def format_scan_report(result: ScanReplayResult) -> str:
    lines = []
    for lap in result.laps:
        lines.append(
            f"lap {lap.lap_index}: crossing at bag {result.bag_time(lap.t_cross_s):.2f} s  "
            f"{lap.lap_time_s:.2f} s  {lap.length_m:.2f} m  "
            f"{'MISSED-GATE ' if lap.missed_gate else ''}"
            f"{'clean' if lap.clean else 'unclean'}"
        )
    cost = result.match_times_s
    if cost:
        ms = sorted(1e3 * c for c in cost)
        lines.append(
            f"on_scan cost while armed: mean {sum(ms) / len(ms):.1f} ms, "
            f"p99 {ms[min(len(ms) - 1, int(0.99 * len(ms)))]:.1f} ms, max {ms[-1]:.1f} ms"
        )
    share = result.matched_scans / result.armed_scans if result.armed_scans else math.nan
    lines.append(
        f"{len(result.laps)} laps, {result.n_scans} scans, armed {result.armed_scans}, "
        f"matched {result.matched_scans} ({100.0 * share:.0f} percent of armed), "
        f"wheel distance {result.wheel_distance_m:.1f} m"
    )
    return "\n".join(lines)


# -- M-R: gate v2 against the recorded laps of gate v1 and the pose detector ---------------------

#: Segment drivers M-R covers (pre-registration pnp-v0.3 section 1.5).
M_R_DRIVERS = ("pnp", "gap_follow", "gap_follow_race", "tracker")
#: Segments with a map pose, where every lap time is compared with the pose detector. T2a and T2b
#: are one run split by the held-out rule: they are joined.
M_R_POSE_SEGMENTS = ("G1a", "G1b", "T0", "T1", "T2a+T2b")
M_R_JOINED = {"T2a+T2b": "2026-11-13T23-27-23_car_teleop"}
#: pre-registration pnp-v0.3 1.5: every lap time within this of the pose detector.
M_R_LAP_TOLERANCE_S = 0.3
#: "at least two laps": a segment counts when some detector saw this many.
M_R_MIN_LAPS = 2


@dataclass
class MRRow:
    segment: str
    v1_laps: int
    v2_laps: int
    pose_laps: int | None  # None: no map pose on this segment
    v1_times: list[float]
    v2_times: list[float]
    pose_times: list[float]
    v2_minus_pose: list[float]  # per lap k, v2 lap time minus pose lap time (k < both counts)
    v1_minus_pose: list[float]
    v2_start_bag_s: float | None = None

    @property
    def qualifies(self) -> bool:
        return max(self.v1_laps, self.v2_laps, self.pose_laps or 0) >= M_R_MIN_LAPS

    @property
    def lap_count_ok(self) -> bool:
        return self.v2_laps >= self.v1_laps

    @property
    def lap_times_ok(self) -> bool | None:
        """Every v2 lap within the tolerance of the pose lap (None without a pose). A lap that
        the pose detector has and v2 lacks makes it False; laps v2 closed beyond the pose
        detector's count are listed in the table, not counted here."""
        if self.pose_laps is None:
            return None
        if self.v2_laps < self.pose_laps:
            return False
        return all(abs(d) <= M_R_LAP_TOLERANCE_S for d in self.v2_minus_pose)


def driven_segments(split: dict[str, Any]) -> list[tuple[str, str]]:
    """(bag, segment) of every PnP, gap follow and tracker segment, then the joined ones."""
    out = []
    for bag, entry in split["bags"].items():
        for seg in entry.get("segments", []):
            if seg.get("driver") in M_R_DRIVERS:
                out.append((bag, str(seg["name"])))
    out.extend((bag, name) for name, bag in M_R_JOINED.items())
    return out


def run_m_r(
    split_file: str | Path,
    bags_root: str | Path,
    v1_laps_file: str | Path,
    vehicle: LapVehicle | None = None,
    only: Sequence[str] | None = None,
    tail_s: float = 0.0,
) -> list[MRRow]:
    """Gate v2 over every driven segment of the split, against the laps gate v1 closed (the
    committed JSON of `record_gate_laps.py` at f45fd09) and, on the map-pose segments, against
    the pose detector started at the same instant. `tail_s` extends the end of every window (0 is
    the pre-registered rule: gate v2 publishes a lap up to about 0.3 m of travel after the line,
    so a lap closed in the last 0.3 m of a segment is lost to the cut; a tail only shows that)."""
    import json

    import yaml

    split = yaml.safe_load(Path(split_file).read_text(encoding="utf-8"))
    v1 = json.loads(Path(v1_laps_file).read_text(encoding="utf-8"))["segments"]
    vehicle = vehicle or load_live_vehicle()
    by_bag: dict[str, list[str]] = {}
    for bag, name in driven_segments(split):
        if only is None or name in only:
            by_bag.setdefault(bag, []).append(name)
    rows: list[MRRow] = []
    for bag, names in by_bag.items():
        messages = list(read_messages(Path(bags_root) / bag))
        for name in names:
            lo, hi = segment_ranges(split_file, bag, name)
            window = (lo, hi + tail_s)
            result = run_replay_start_scan(messages, vehicle, window_s=window)
            pose_laps: list[LapResult] | None = None
            if name in M_R_POSE_SEGMENTS:
                first = result.first_scan_log_s
                sub = [m for m in messages if window[0] <= m[0] - first <= window[1]]
                pose_laps = run_replay(sub, start_s=result.start_t_s).laps
            v2_times = [lap.lap_time_s for lap in result.laps]
            v1_times = list(v1.get(name, {}).get("lap_times_s", []))
            pose_times = [lap.lap_time_s for lap in pose_laps] if pose_laps is not None else []
            rows.append(
                MRRow(
                    segment=name,
                    v1_laps=len(v1_times),
                    v2_laps=len(v2_times),
                    pose_laps=None if pose_laps is None else len(pose_laps),
                    v1_times=v1_times,
                    v2_times=v2_times,
                    pose_times=pose_times,
                    v2_minus_pose=[a - b for a, b in zip(v2_times, pose_times, strict=False)],
                    v1_minus_pose=[a - b for a, b in zip(v1_times, pose_times, strict=False)],
                    v2_start_bag_s=None
                    if result.start_t_s is None
                    else result.start_t_s - result.first_scan_log_s,
                )
            )
        del messages
    return rows


def format_m_r(rows: Sequence[MRRow]) -> str:
    """The M-R table and the verdict of pre-registration 1.5 (markdown)."""

    def times(values: Sequence[float]) -> str:
        return " ".join(f"{v:.2f}" for v in values) or "-"

    lines = [
        (
            "| segment | v1 laps | v2 laps | pose laps | counts | v2 vs pose (s, per lap) | "
            "v1 vs pose (s, per lap) |"
        ),
        "|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        counts = "n/a (under 2 laps)" if not r.qualifies else ("ok" if r.lap_count_ok else "FAIL")
        pose = "-" if r.pose_laps is None else str(r.pose_laps)
        v2d = "-" if r.pose_laps is None else (times(r.v2_minus_pose))
        v1d = "-" if r.pose_laps is None else (times(r.v1_minus_pose))
        lines.append(
            f"| {r.segment} | {r.v1_laps} | {r.v2_laps} | {pose} | {counts} | {v2d} | {v1d} |"
        )
    qualifying = [r for r in rows if r.qualifies]
    count_fail = [r.segment for r in qualifying if not r.lap_count_ok]
    time_fail = [
        r.segment for r in rows if r.pose_laps is not None and r.qualifies and not r.lap_times_ok
    ]
    lines.append("")
    lines.append(
        f"M-R lap counts (v2 closes at least as many as v1, segments with two laps): "
        f"{'PASS' if not count_fail else 'FAIL ' + ', '.join(count_fail)}"
    )
    lines.append(
        f"M-R lap times (every v2 lap within {M_R_LAP_TOLERANCE_S} s of the pose detector, map "
        f"pose segments): {'PASS' if not time_fail else 'FAIL ' + ', '.join(time_fail)}"
    )
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--bag", type=Path, help="bag directory or .mcap file")
    parser.add_argument("--lap-source", choices=LAP_SOURCES, default="map_pose")
    parser.add_argument("--min-lap-length-m", type=float, default=LapConfig().min_lap_length_m)
    parser.add_argument(
        "--start-line-length-m", type=float, default=LapConfig().start_line_length_m
    )
    parser.add_argument("--from-s", type=float, help="segment start, bag seconds after first /scan")
    parser.add_argument("--to-s", type=float, help="segment end, bag seconds after first /scan")
    parser.add_argument("--split", type=Path, help="pre-registered split yaml")
    parser.add_argument("--segment", help="segment name from --split, A+B joins consecutive ones")
    parser.add_argument("--compare-pose", action="store_true", help="also run the map pose")
    parser.add_argument(
        "--m-r",
        action="store_true",
        help="M-R table (pre-registration pnp-v0.3 1.5): gate v2 on every driven segment of "
        "--split against the recorded laps of gate v1 (--v1-laps) and, on the map-pose segments, "
        "the pose detector. Needs --split, --bags-root and --v1-laps; --bag is not used",
    )
    parser.add_argument("--bags-root", type=Path, help="with --m-r: directory of the bags")
    parser.add_argument("--v1-laps", type=Path, help="with --m-r: gate v1's recorded laps (json)")
    parser.add_argument("--only", help="with --m-r: comma separated segment names")
    parser.add_argument(
        "--tail-s",
        type=float,
        default=0.0,
        help="with --m-r: extend every window by this many seconds (0 = the pre-registered rule)",
    )
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    if args.m_r:
        if args.split is None or args.bags_root is None or args.v1_laps is None:
            parser.error("--m-r needs --split, --bags-root and --v1-laps")
        only = args.only.split(",") if args.only else None
        rows = run_m_r(args.split, args.bags_root, args.v1_laps, only=only, tail_s=args.tail_s)
        sys.stdout.write(format_m_r(rows) + "\n")
        return 0
    window: tuple[float, float] | None = None
    if args.segment:
        if args.split is None:
            parser.error("--segment needs --split")
        bag_dir = args.bag if args.bag.is_dir() else args.bag.parent
        window = segment_ranges(args.split, bag_dir.name, args.segment)
    elif args.from_s is not None or args.to_s is not None:
        window = (
            -math.inf if args.from_s is None else args.from_s,
            math.inf if args.to_s is None else args.to_s,
        )
    if args.bag is None:
        parser.error("--bag is required")
    messages = list(read_messages(args.bag))
    if args.lap_source == "start_scan":
        scan_result = run_replay_start_scan(messages, load_live_vehicle(), window_s=window)
        sys.stdout.write(format_scan_report(scan_result) + "\n")
        if args.compare_pose:
            first = scan_result.first_scan_log_s
            lo, hi = (-math.inf, math.inf) if window is None else window
            sub = [m for m in messages if lo <= m[0] - first <= hi]
            pose = run_replay(sub, start_s=scan_result.start_t_s)
            sys.stdout.write("map pose, same segment and start:\n" + format_report(pose) + "\n")
            for k, lap in enumerate(pose.laps):
                sys.stdout.write(f"  pose lap {k + 1} at bag {lap.t_cross_s - first:.2f} s\n")
        return 0 if scan_result.laps else 1
    if window is not None:
        first = next((m[0] for m in messages if m[1] == SCAN_TOPIC), messages[0][0])
        messages = [m for m in messages if window[0] <= m[0] - first <= window[1]]
    cfg = LapConfig(
        min_lap_length_m=args.min_lap_length_m, start_line_length_m=args.start_line_length_m
    )
    pose_result = run_replay(messages, cfg)
    sys.stdout.write(format_report(pose_result) + "\n")
    return 0 if pose_result.n_pose_samples > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
