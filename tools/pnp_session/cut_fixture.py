"""Cut the committed L4 fixture out of a real bag (provenance of tools/tests/fixtures/).

    uv run --project tools --extra session python -m pnp_session.cut_fixture \\
        --bag <2026-11-13T22-20-18_car_teleop dir> --run R2 --out tools/tests/fixtures/pnp_session_r2_cut.mcap

Raw messages are copied, not re-encoded: the first /scan of the bag (the zero of the time axis,
kept with its original log time so every time in the cut equals the same time in the bag), then
the topics the report needs inside the chosen run plus a margin: /drive_raw, /drive,
/pnp/planner/status, /safety/events. Compressed with zstd.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from mcap.reader import make_reader
from mcap.writer import CompressionType, Writer

from pnp_session.bagread import (
    DRIVE,
    DRIVE_RAW,
    EVENTS,
    SCAN,
    STATUS,
    bag_files,
    read_session_bag,
)
from pnp_session.timing import find_pnp_runs

TOPICS = [DRIVE_RAW, DRIVE, STATUS, EVENTS]
MARGIN_BEFORE_S = 2.0
MARGIN_AFTER_S = 3.0


def cut(bag: Path, run_id: str, out: Path) -> tuple[int, str]:
    data = read_session_bag(bag)
    runs = {r.run_id: r for r in find_pnp_runs(data)}
    run = runs[run_id]
    t0_ns = round(data.t0_log_s * 1e9)
    start_ns = t0_ns + int((run.t_start - MARGIN_BEFORE_S) * 1e9)
    end_ns = t0_ns + int((run.t_end + MARGIN_AFTER_S) * 1e9)
    n = 0
    with out.open("wb") as stream:
        writer = Writer(stream, compression=CompressionType.ZSTD)
        writer.start()
        schemas: dict[int, int] = {}
        channels: dict[int, int] = {}

        def put(schema, channel, message) -> None:  # type: ignore[no-untyped-def]
            nonlocal n
            if schema is not None and schema.id not in schemas:
                schemas[schema.id] = writer.register_schema(
                    name=schema.name, encoding=schema.encoding, data=schema.data
                )
            if channel.id not in channels:
                channels[channel.id] = writer.register_channel(
                    topic=channel.topic,
                    message_encoding=channel.message_encoding,
                    schema_id=schemas.get(channel.schema_id, 0),
                    metadata=dict(channel.metadata),
                )
            writer.add_message(
                channel_id=channels[channel.id],
                log_time=message.log_time,
                data=message.data,
                publish_time=message.publish_time,
                sequence=message.sequence,
            )
            n += 1

        for file in bag_files(bag):
            with file.open("rb") as f:
                reader = make_reader(f)
                for schema, channel, message in reader.iter_messages(topics=[SCAN]):
                    put(schema, channel, message)
                    break
                for schema, channel, message in reader.iter_messages(
                    topics=TOPICS, starting_at=start_ns, ending_at=end_ns, log_time_order=True
                ):
                    put(schema, channel, message)
        writer.finish()
    return n, f"{run_id} bag s {run.t_start:.1f} to {run.t_end:.1f}"


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--bag", required=True)
    p.add_argument("--run", required=True)
    p.add_argument("--out", required=True)
    args = p.parse_args()
    n, what = cut(Path(args.bag), args.run, Path(args.out))
    print(f"{n} messages, {what}, {Path(args.out).stat().st_size} bytes")


if __name__ == "__main__":
    main()
