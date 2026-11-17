"""M-R2 of brief 48: the lap node's logic (`racer_laps.engine.LapMemory`, every message fed from the
start of the bag, D v0 defaults) over the full car bags of 2026-11-16, next to the camera lap
counter. Development and regression data: it decides nothing.

    uv run --extra replay python tests/fixtures_car_2026_11_16/m_r2_table.py <bags dir> [bag ...]

Prints a markdown table. Run it on the code under test and on main to fill the two columns of the
brief. The camera counter's events are the bag's `/lap/event` (JSON in a std_msgs/String:
`lap`, `lap_time_s`, `crossing_time_s`).
"""

from __future__ import annotations

import glob
import json
import math
import statistics
import sys
from pathlib import Path

from racer_laps.replay import ODOM_TOPIC, read_messages, run_replay_start_scan
from racer_laps.vehicle import load_live_vehicle

try:
    from racer_laps.gate import STANDSTILL_MPS
except ImportError:  # main before brief 48 (the same script gives its column of the table)
    STANDSTILL_MPS = 0.05

BAGS = (
    "2026-11-16T22-38-47_car_teleop",
    "2026-11-16T02-18-54_car_teleop",
    "2026-11-16T22-45-39_car_teleop",
    "2026-11-16T23-04-09_car_teleop",
    "2026-11-16T02-29-27_car_teleop",
)
TOLERANCE_S = 0.3


def camera_events(bag: Path) -> list[dict]:
    from mcap.records import Channel, Message, Schema
    from mcap.stream_reader import StreamReader
    from mcap_ros2.decoder import DecoderFactory

    factory = DecoderFactory()
    out: list[dict] = []
    for file in sorted(glob.glob(str(bag / "*.mcap"))):
        schemas: dict[int, Schema] = {}
        channels: dict[int, Channel] = {}
        with open(file, "rb") as fh:
            reader = StreamReader(fh, skip_magic=False, emit_chunks=False, validate_crcs=False)
            try:
                for rec in reader.records:
                    if isinstance(rec, Schema):
                        schemas[rec.id] = rec
                    elif isinstance(rec, Channel):
                        channels[rec.id] = rec
                    elif (
                        isinstance(rec, Message) and channels[rec.channel_id].topic == "/lap/event"
                    ):
                        ch = channels[rec.channel_id]
                        decode = factory.decoder_for(ch.message_encoding, schemas[ch.schema_id])
                        out.append(json.loads(decode(rec.data).data))
            except Exception as error:  # noqa: BLE001 - a truncated tail keeps what was read
                print(f"{file}: stopped at an unreadable record ({error})", file=sys.stderr)
    return out


def row(bag: Path) -> dict[str, object]:
    messages = list(read_messages(bag))
    res = run_replay_start_scan(messages, load_live_vehicle())
    cam = camera_events(bag)
    laps = res.laps
    first = res.first_scan_log_s
    odom = [(t - first, float(m.twist.twist.linear.x)) for t, tp, m in messages if tp == ODOM_TOPIC]
    moving = [t for t, v in odom if abs(v) >= STANDSTILL_MPS]
    out: dict[str, object] = {
        "bag": bag.name,
        "laps": len(laps),
        "camera_laps": len(cam),
        "start_scan_bag_s": None if res.start_t_s is None else round(res.start_t_s - first, 2),
        "lap1_s": round(laps[0].lap_time_s, 2) if laps else None,
        # (getattr: the same script runs on main for the "on main today" column)
        "launch_dwell_s": round(getattr(laps[0], "launch_dwell_s", math.nan), 2) if laps else None,
        "missed": [lap.lap_index for lap in laps if lap.missed_gate],
        "unclean": [lap.lap_index for lap in laps if not lap.clean],
        "wheel_m": round(res.wheel_distance_m, 1),
    }
    if len(laps) > 5:
        caps = [lap.next_cap_mps for lap in laps[4:]]
        out["cap_from_lap_6_min"] = round(min(caps), 2)
        out["cap_final"] = round(laps[-1].next_cap_mps, 2)
    decisions = getattr(res, "decision_log_s", [])
    if laps and moving and decisions:
        # the first wheel sample standing after the last crossing: the car's standstill instant
        t_last = laps[-1].t_cross_s - first
        standing = [t for t, v in odom if t >= t_last and abs(v) < STANDSTILL_MPS]
        decided = decisions[-1] - first
        out["last_lap_decision_bag_s"] = round(decided, 2)
        out["standstill_after_last_crossing_bag_s"] = round(standing[0], 2) if standing else None
        out["last_lap_decision_minus_standstill_s"] = (
            round(decided - standing[0], 2) if standing else None
        )
    if cam and len(cam) != len(laps):
        out["camera_comparison"] = (
            f"not comparable: the camera counted {len(cam)}, the node {len(laps)}"
        )
    elif cam and laps:
        n = len(laps)
        dt = [laps[k].lap_time_s - float(cam[k]["lap_time_s"]) for k in range(1, n)]
        off = [laps[k].t_cross_s - float(cam[k]["crossing_time_s"]) for k in range(n)]
        med = statistics.median(off[1:]) if len(off) > 1 else math.nan
        out["laps_2_on_within_0.3_of_camera"] = (
            f"{sum(abs(d) <= TOLERANCE_S for d in dt)} of {len(dt)}"
        )
        out["max_abs_lap_time_diff_s"] = round(max((abs(d) for d in dt), default=math.nan), 3)
        out["crossing_offset_median_2_on_s"] = round(med, 3)
        out["crossing_offset_1_s"] = round(off[0], 3)
        out["lap1_offset_minus_median_s"] = round(off[0] - med, 3)
        out["camera_lap1_s"] = round(float(cam[0]["lap_time_s"]), 2)
    return out


def main(bags: str, names: list[str]) -> None:
    for name in names or BAGS:
        bag = Path(bags) / name
        if not bag.exists():
            print(f"{name}: no such bag")
            continue
        print(json.dumps(row(bag), indent=1), flush=True)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2:])
