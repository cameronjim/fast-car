"""Read a few topics of an mcap bag onto the bag axis, with nothing else decoded.

The bag axis is the one of the split files: seconds after the log time of the first `/scan`
message, computed in integer nanoseconds. `/scan` is read for that one log time only and is never
decoded. A truncated or unindexed file (the recorder was killed) is read as far as it goes.
Real-data glue; the tests write a small bag with mcap_ros2's writer.
"""

from __future__ import annotations

import logging
from collections.abc import Iterator, Sequence
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

log = logging.getLogger(__name__)

SCAN = "/scan"
DRIVE = "/drive"
DRIVE_RAW = "/drive_raw"
ODOM = "/odom/wheel"
STATUS = "/pnp/planner/status"


def bag_files(bag: Path) -> list[Path]:
    """The .mcap files of a bag: the file itself or the sorted files of a directory."""
    files = [bag] if bag.is_file() else sorted(bag.glob("*.mcap"))
    if not files:
        raise FileNotFoundError(f"no .mcap file in {bag}")
    return files


def iter_messages(bag: Path, topics: Sequence[str]) -> Iterator[tuple[int, str, Any]]:
    """(log time in ns, topic, decoded message) of the wanted topics in file order. `/scan` is
    reported with message None for its first message only (never decoded) when it is listed."""
    from mcap.exceptions import EndOfFile
    from mcap.records import Channel, Message, Schema
    from mcap.stream_reader import StreamReader
    from mcap_ros2.decoder import DecoderFactory

    wanted = set(topics)
    factory = DecoderFactory()
    scan_seen = False
    for file in bag_files(bag):
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
                        if channel is None:
                            continue
                        if channel.topic == SCAN:
                            if not scan_seen:
                                scan_seen = True
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
                            yield int(record.log_time), channel.topic, decoder(record.data)
                        except (ValueError, KeyError, IndexError, UnicodeDecodeError) as error:
                            log.warning("%s: undecodable message (%s)", file.name, error)
            except EndOfFile:
                log.info("%s: no footer (the recorder was killed); used what was there", file.name)
            except Exception as error:  # noqa: BLE001 - a damaged tail must not lose good records
                log.warning("%s: stopped at an unreadable record (%s)", file.name, error)


def read_columns(bag: Path, topics: dict[str, Any]) -> tuple[float, dict[str, NDArray[np.float64]]]:
    """Read the wanted topics into float arrays on the bag axis.

    `topics` maps a topic to a function msg -> tuple of floats. The result is the log time of the
    first /scan (seconds since the epoch) and, per topic, an array of shape (n, 1 + k): the bag
    time, then the extracted values. Raises ValueError if the bag has no /scan."""
    t0_ns: int | None = None
    rows: dict[str, list[tuple[int, tuple[float, ...]]]] = {k: [] for k in topics}
    for t_ns, topic, msg in iter_messages(bag, [SCAN, *topics]):
        if topic == SCAN:
            t0_ns = t_ns
        else:
            rows[topic].append((t_ns, tuple(float(v) for v in topics[topic](msg))))
    if t0_ns is None:
        raise ValueError(f"{bag}: no /scan message, so the bag axis has no zero")
    out: dict[str, NDArray[np.float64]] = {}
    for topic, items in rows.items():
        width = len(items[0][1]) if items else 0
        arr = np.zeros((len(items), 1 + width), dtype=np.float64)
        for i, (t_ns, values) in enumerate(items):
            arr[i, 0] = (t_ns - t0_ns) / 1e9
            arr[i, 1:] = values
        out[topic] = arr
    return t0_ns / 1e9, out
