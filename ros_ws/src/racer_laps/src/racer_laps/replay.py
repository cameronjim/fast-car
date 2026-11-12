"""Run the lap detector over a recorded bag and print the laps.

    python -m racer_laps.replay --bag <dir or .mcap>

The bag needs `/tf` (and `/tf_static` if the map to odom or odom to base_link link is static):
bags from stack runs with `mapping:=true`. `/pnp/planner/status`, `/safety/events`,
`/odom/wheel` and `/drive_raw` are used when present. Without planner status the start line is
set at the first pose and every lap is unclean (there is no plan clearance to judge), which the
output says. Needs the `replay` extra (mcap, mcap-ros2-support), no ROS install.

The decision logic is exactly the node's: `racer_laps.engine.LapMemory` fed at 20 Hz of bag time.
"""

from __future__ import annotations

import argparse
import logging
import math
import sys
from collections import deque
from collections.abc import Iterable, Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from racer_laps.engine import LapConfig, LapMemory, LapResult
from racer_laps.lap_detector import wrap_angle

log = logging.getLogger(__name__)

TF_TOPICS = ("/tf", "/tf_static")
PLANNER_TOPIC = "/pnp/planner/status"
EVENTS_TOPIC = "/safety/events"
ODOM_TOPIC = "/odom/wheel"
DRIVE_RAW_TOPIC = "/drive_raw"
WANTED = (*TF_TOPICS, PLANNER_TOPIC, EVENTS_TOPIC, ODOM_TOPIC, DRIVE_RAW_TOPIC)

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
            if not saw_planner and not assumed_driving:
                # no planner status in this bag: lap 1 starts at the first pose
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
            mem.on_safety_event(int(msg.phase), int(msg.severity))
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


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--bag", required=True, type=Path, help="bag directory or .mcap file")
    parser.add_argument("--min-lap-length-m", type=float, default=LapConfig().min_lap_length_m)
    parser.add_argument(
        "--start-line-length-m", type=float, default=LapConfig().start_line_length_m
    )
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    cfg = LapConfig(
        min_lap_length_m=args.min_lap_length_m, start_line_length_m=args.start_line_length_m
    )
    result = run_replay(read_messages(args.bag), cfg)
    sys.stdout.write(format_report(result) + "\n")
    return 0 if result.n_pose_samples > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
