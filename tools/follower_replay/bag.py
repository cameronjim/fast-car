"""Read the topics the follower replay needs out of an mcap bag.

The recorder on the car (car_teleop.launch.py's rosbag2 with the mcap plugin) is often stopped
by killing it, which leaves a file with no summary section, no footer and sometimes a half
written last chunk. The seeking reader needs the footer, so this uses mcap's StreamReader, which
walks the records from the front, and keeps every message it decoded before the file ran out.
`BagData.truncated` says whether that happened.

Messages are decoded with mcap-ros2-support from the schemas stored in the file, so no ROS
install is needed. Only /scan, /drive_raw, /drive and /safety/events are decoded; everything
else is skipped without touching its payload.
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass, field
from pathlib import Path
from typing import IO, Any

SCAN_TOPIC = "/scan"
DRIVE_RAW_TOPIC = "/drive_raw"
DRIVE_TOPIC = "/drive"
EVENTS_TOPIC = "/safety/events"
WANTED_TOPICS = (SCAN_TOPIC, DRIVE_RAW_TOPIC, DRIVE_TOPIC, EVENTS_TOPIC)

# racer_msgs/SafetyEvent constants (ros_ws/src/racer_msgs/msg/SafetyEvent.msg).
SEVERITY_INFO = 0
SEVERITY_WARNING = 1
SEVERITY_BRAKE = 2
PHASE_ENGAGE = 0
PHASE_RELEASE = 1


@dataclass(frozen=True)
class ScanRecord:
    stamp_s: float
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: tuple[float, ...]


@dataclass(frozen=True)
class DriveRecord:
    stamp_s: float
    steering_rad: float
    speed_mps: float


@dataclass(frozen=True)
class EventRecord:
    stamp_s: float
    severity: int
    phase: int
    duration_s: float
    source: str
    detail: str


@dataclass
class BagData:
    scans: list[ScanRecord] = field(default_factory=list)
    drive_raw: list[DriveRecord] = field(default_factory=list)
    drive: list[DriveRecord] = field(default_factory=list)
    events: list[EventRecord] = field(default_factory=list)
    files: list[str] = field(default_factory=list)
    # True when a file ended before its footer or inside a record (the recorder was killed).
    truncated: bool = False
    truncation_note: str = ""
    decode_errors: int = 0
    # Topics present in the file(s) with message counts, whether or not they were decoded.
    topic_counts: dict[str, int] = field(default_factory=dict)


def bag_files(path: Path) -> list[Path]:
    """The .mcap files of a bag: the file itself, or the sorted *.mcap files of a directory."""
    if path.is_file():
        return [path]
    if path.is_dir():
        files = sorted(path.glob("*.mcap"))
        if not files:
            raise FileNotFoundError(
                f"no .mcap file in {path} (a sqlite3 bag has to be converted first: "
                "`ros2 bag convert`)"
            )
        return files
    raise FileNotFoundError(f"no such bag: {path}")


def _stamp_s(stamp: Any, log_time_ns: int) -> float:
    """A header stamp in seconds, or the log time when the stamp is unset (zero)."""
    sec = int(getattr(stamp, "sec", 0))
    nanosec = int(getattr(stamp, "nanosec", 0))
    if sec == 0 and nanosec == 0:
        return log_time_ns / 1e9
    return sec + nanosec / 1e9


def _scan_record(msg: Any, log_time_ns: int) -> ScanRecord:
    return ScanRecord(
        stamp_s=_stamp_s(msg.header.stamp, log_time_ns),
        angle_min=float(msg.angle_min),
        angle_increment=float(msg.angle_increment),
        range_min=float(msg.range_min),
        range_max=float(msg.range_max),
        ranges=tuple(float(r) for r in msg.ranges),
    )


def _drive_record(msg: Any, log_time_ns: int) -> DriveRecord:
    return DriveRecord(
        stamp_s=_stamp_s(msg.header.stamp, log_time_ns),
        steering_rad=float(msg.drive.steering_angle),
        speed_mps=float(msg.drive.speed),
    )


def _event_record(msg: Any, log_time_ns: int) -> EventRecord:
    return EventRecord(
        stamp_s=_stamp_s(msg.stamp, log_time_ns),
        severity=int(msg.severity),
        phase=int(msg.phase),
        duration_s=float(msg.duration_s),
        source=str(msg.source),
        detail=str(msg.detail),
    )


def _records(stream: IO[bytes], data: BagData, name: str) -> Iterator[Any]:
    """The records of one file; a truncated or corrupt tail ends the iteration, not the run."""
    from mcap.exceptions import EndOfFile
    from mcap.stream_reader import StreamReader

    reader = StreamReader(stream, skip_magic=False, emit_chunks=False, validate_crcs=False)
    try:
        yield from reader.records
    except EndOfFile:
        # The normal end of a bag whose recorder was killed: no DataEnd, summary or footer.
        data.truncated = True
        data.truncation_note = (
            f"{name}: no footer (the recorder was killed); messages in a last, unfinished chunk "
            "are not recoverable"
        )
    except Exception as error:  # noqa: BLE001 - a damaged tail must not lose the good records
        data.truncated = True
        data.truncation_note = (
            f"{name}: stopped at an unreadable record ({type(error).__name__}: {error}); "
            "messages after it, and the rest of its chunk, are missing"
        )


def read_bag(path: str | Path) -> BagData:
    """Decode the follower topics of an mcap bag (a file, or a directory of .mcap files)."""
    from mcap.records import Channel, Footer, Message, Schema
    from mcap_ros2.decoder import DecoderFactory

    data = BagData()
    factory = DecoderFactory()
    for file in bag_files(Path(path)):
        data.files.append(str(file))
        schemas: dict[int, Schema] = {}
        channels: dict[int, Channel] = {}
        decoders: dict[int, Any] = {}
        saw_footer = False
        with file.open("rb") as stream:
            for record in _records(stream, data, file.name):
                if isinstance(record, Schema):
                    schemas[record.id] = record
                elif isinstance(record, Channel):
                    channels[record.id] = record
                elif isinstance(record, Footer):
                    saw_footer = True
                elif isinstance(record, Message):
                    channel = channels.get(record.channel_id)
                    if channel is None:
                        continue
                    topic = channel.topic
                    data.topic_counts[topic] = data.topic_counts.get(topic, 0) + 1
                    if topic not in WANTED_TOPICS:
                        continue
                    decoder = decoders.get(record.channel_id)
                    if decoder is None:
                        schema = schemas.get(channel.schema_id)
                        decoder = factory.decoder_for(channel.message_encoding, schema)
                        if decoder is None:
                            data.decode_errors += 1
                            continue
                        decoders[record.channel_id] = decoder
                    try:
                        msg = decoder(record.data)
                        if topic == SCAN_TOPIC:
                            data.scans.append(_scan_record(msg, record.log_time))
                        elif topic == DRIVE_RAW_TOPIC:
                            data.drive_raw.append(_drive_record(msg, record.log_time))
                        elif topic == DRIVE_TOPIC:
                            data.drive.append(_drive_record(msg, record.log_time))
                        else:
                            data.events.append(_event_record(msg, record.log_time))
                    except Exception:  # noqa: BLE001 - one bad message must not lose the bag
                        data.decode_errors += 1
        if not saw_footer and not data.truncated:
            data.truncated = True
            data.truncation_note = f"{file.name}: no footer (the recorder was killed)"
    return data
