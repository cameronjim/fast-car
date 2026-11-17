"""A lenient MCAP reader for a bag that was not finalised (brief 51).

`bagread.read_session_bag` uses mcap's `make_reader`, which needs the footer and the summary: a
bag whose recorder was killed has neither. This reader goes through the file front to back with
`mcap.stream_reader.StreamReader` (chunks are opened, no CRC is checked), hands over the wanted
topics decoded, and at the first broken record (a truncated chunk, a bad length) it stops and says
how far it got. Nothing after a broken record is trusted.

The bag axis is the one of `bagread` and of the split files: seconds after the log time of the
first `/scan` message, computed in integer nanoseconds. `/scan` is read for that one log time
only and never decoded.

Real-data glue; the tests write small bags with mcap_ros2's writer and cut one short.
"""

from __future__ import annotations

import logging
from collections.abc import Iterator, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

SCAN = "/scan"

log = logging.getLogger(__name__)


@dataclass
class StreamReport:
    """How far a read got."""

    path: str = ""
    file_bytes: int = 0
    #: Bytes of the file consumed when the read ended (the whole file when `complete`).
    bytes_read: int = 0
    messages: int = 0
    #: The read reached the end of the data (the footer, or the end of a clean file).
    complete: bool = False
    #: Why it stopped, empty when it did not: the exception name and text.
    stopped: str = ""
    #: Log time (ns) of the last message handed over (None before the first).
    last_log_time_ns: int | None = None
    #: Messages per topic of the wanted topics.
    per_topic: dict[str, int] = field(default_factory=dict)

    def say(self) -> str:
        if self.complete:
            return f"{Path(self.path).name}: read to the end ({self.messages} messages)"
        share = 100.0 * self.bytes_read / self.file_bytes if self.file_bytes else 0.0
        return (
            f"{Path(self.path).name}: stopped at byte {self.bytes_read} of {self.file_bytes} "
            f"({share:.0f} percent) after {self.messages} messages, "
            f"last log time {self.last_log_time_ns}: {self.stopped}"
        )


def bag_files(bag: Path) -> list[Path]:
    files = [bag] if bag.is_file() else sorted(bag.glob("*.mcap"))
    if not files:
        raise FileNotFoundError(f"no .mcap file in {bag}")
    return files


def iter_stream(
    path: Path, topics: Sequence[str], report: StreamReport | None = None
) -> Iterator[tuple[int, str, Any]]:
    """(log time in ns, topic, decoded message) of the wanted topics of one .mcap file in file
    order. `/scan` is reported once, with message None, for its first message when it is wanted.
    The `report` is filled as the read goes and when it ends or breaks."""
    from mcap.records import Channel, Message, Schema
    from mcap.stream_reader import StreamReader
    from mcap_ros2.decoder import DecoderFactory

    rep = report if report is not None else StreamReport()
    rep.path = str(path)
    rep.file_bytes = path.stat().st_size
    wanted = set(topics)
    factory = DecoderFactory()
    schemas: dict[int, Schema] = {}
    channels: dict[int, Channel] = {}
    decoders: dict[int, Any] = {}
    scan_seen = False
    with path.open("rb") as stream:
        reader = StreamReader(stream, skip_magic=False, emit_chunks=False, validate_crcs=False)
        try:
            for record in reader.records:
                rep.bytes_read = int(getattr(getattr(reader, "_stream", None), "count", 0) or 0)
                if isinstance(record, Schema):
                    schemas[record.id] = record
                elif isinstance(record, Channel):
                    channels[record.id] = record
                elif isinstance(record, Message):
                    channel = channels.get(record.channel_id)
                    if channel is None:
                        continue
                    if channel.topic == SCAN:
                        if not scan_seen and SCAN in wanted:
                            scan_seen = True
                            rep.messages += 1
                            rep.last_log_time_ns = int(record.log_time)
                            yield int(record.log_time), SCAN, None
                        continue
                    if channel.topic not in wanted:
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
                        message = decoder(record.data)
                    except Exception as error:  # noqa: BLE001 - a bad message is skipped, not fatal
                        log.warning("%s: undecodable message (%s)", path.name, error)
                        continue
                    rep.messages += 1
                    rep.last_log_time_ns = int(record.log_time)
                    rep.per_topic[channel.topic] = rep.per_topic.get(channel.topic, 0) + 1
                    yield int(record.log_time), channel.topic, message
            rep.complete = True
            rep.bytes_read = rep.file_bytes
        except GeneratorExit:  # the caller stopped reading: not a broken file
            raise
        except Exception as error:  # noqa: BLE001 - EndOfFile, a bad length, a truncated chunk
            rep.complete = False
            rep.stopped = f"{type(error).__name__}: {error}"
            log.warning("%s", rep.say())


def read_topics(
    bag: Path, topics: Sequence[str]
) -> tuple[int, list[tuple[int, str, Any]], list[StreamReport]]:
    """Every wanted message of a bag (a file or a directory of .mcap files), the log time (ns) of
    the first `/scan` (the zero of the bag axis) and one report per file. Raises ValueError when
    the bag has no `/scan`."""
    t0: int | None = None
    out: list[tuple[int, str, Any]] = []
    reports: list[StreamReport] = []
    for file in bag_files(Path(bag)):
        rep = StreamReport()
        reports.append(rep)
        for t_ns, topic, msg in iter_stream(file, [SCAN, *topics], rep):
            if topic == SCAN:
                if t0 is None:
                    t0 = t_ns
                continue
            out.append((t_ns, topic, msg))
    if t0 is None:
        raise ValueError(f"{bag}: no /scan message, so the bag axis has no zero")
    return t0, out, reports
