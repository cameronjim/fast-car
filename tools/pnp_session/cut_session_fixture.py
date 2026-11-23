"""Cut the session-report fixture out of a real bag (provenance of
tools/tests/fixtures/pnp_session_2026_11_22_cut.mcap).

    cd tools && uv run --project . --extra session python -m pnp_session.cut_session_fixture \\
        --bag <2026-11-23T03-50-32_car_teleop dir> --out tests/fixtures/pnp_session_2026_11_22_cut.mcap \\
        --window 1203,1208 --window 1250,1270 --window 425,445

Raw messages are copied, not re-encoded: the first /scan of the bag (the zero of the bag axis, kept
with its original log time so every time in the cut equals the same time in the bag), then the
topics `session_checks` reads inside each window, given on the bag axis in seconds. The bag may be
one that was not finalised (read with the stream reader, as `streamread` does). zstd.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from mcap.records import Channel, Message, Schema
from mcap.stream_reader import StreamReader
from mcap.writer import CompressionType, Writer

from pnp_session import session_checks, streamread


def cut(bag: Path, windows: list[tuple[float, float]], out: Path) -> int:
    wanted = set(session_checks.TOPICS)
    schemas: dict[int, Schema] = {}
    channels: dict[int, Channel] = {}
    t0_ns: int | None = None
    n = 0
    with out.open("wb") as stream:
        writer = Writer(stream, compression=CompressionType.ZSTD)
        writer.start()
        wschema: dict[int, int] = {}
        wchannel: dict[int, int] = {}

        def put(channel: Channel, message: Message) -> None:
            nonlocal n
            schema = schemas.get(channel.schema_id)
            if schema is not None and schema.id not in wschema:
                wschema[schema.id] = writer.register_schema(
                    name=schema.name, encoding=schema.encoding, data=schema.data
                )
            if channel.id not in wchannel:
                wchannel[channel.id] = writer.register_channel(
                    topic=channel.topic,
                    message_encoding=channel.message_encoding,
                    schema_id=wschema.get(channel.schema_id, 0),
                    metadata=dict(channel.metadata),
                )
            writer.add_message(
                channel_id=wchannel[channel.id],
                log_time=message.log_time,
                data=message.data,
                publish_time=message.publish_time,
                sequence=message.sequence,
            )
            n += 1

        for file in streamread.bag_files(bag):
            with file.open("rb") as f:
                try:
                    for record in StreamReader(f, skip_magic=False, validate_crcs=False).records:
                        if isinstance(record, Schema):
                            schemas[record.id] = record
                        elif isinstance(record, Channel):
                            channels[record.id] = record
                        elif isinstance(record, Message):
                            channel = channels[record.channel_id]
                            if channel.topic == streamread.SCAN and t0_ns is None:
                                t0_ns = int(record.log_time)
                                put(channel, record)
                            elif channel.topic in wanted and t0_ns is not None:
                                t = (record.log_time - t0_ns) / 1e9
                                if any(a <= t <= b for a, b in windows):
                                    put(channel, record)
                except Exception as error:  # noqa: BLE001 - a bag that was not finalised ends so
                    print(f"{file.name}: read stopped ({type(error).__name__}: {error})")
        writer.finish()
    return n


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--bag", required=True, type=Path)
    p.add_argument("--out", required=True, type=Path)
    p.add_argument("--window", action="append", required=True, help="A,B seconds on the bag axis")
    args = p.parse_args()
    windows = [tuple(float(x) for x in w.split(",")) for w in args.window]
    n = cut(args.bag, windows, args.out)  # type: ignore[arg-type]
    print(f"{n} messages, {args.out.stat().st_size} bytes")


if __name__ == "__main__":
    main()
