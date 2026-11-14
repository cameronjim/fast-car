"""The topics the session report reads from an mcap bag, as plain arrays on one time axis.

Time axis: bag seconds, the recorder's log time minus the log time of the first `/scan` message
(the axis of racer_eval and of the split file), computed in integer nanoseconds so that a gap of
60 ms is 0.06 and not 0.0600000003. Header stamps are moved onto the same axis by subtracting the
same zero. Two clocks therefore appear side by side everywhere:

* `*_t`: the recorder's receive time (log time). This is "recorder receive time" in the report.
* `*_stamp`: the message's own header stamp (the sender's `now()`).

Real-data glue like racer_eval.bag_io: only the wanted channels are decoded, and `/scan` is only
read for the log time of its first message.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

SCAN = "/scan"
DRIVE_RAW = "/drive_raw"
DRIVE = "/drive"
STATUS = "/pnp/planner/status"
EVENTS = "/safety/events"
LAP_CAMERA = "/lap/event"
LAP_PNP = "/pnp/lap/event"
TOPICS = (DRIVE_RAW, DRIVE, STATUS, EVENTS, LAP_CAMERA, LAP_PNP)


def _f() -> NDArray[np.float64]:
    return np.zeros(0, dtype=np.float64)


def _i() -> NDArray[np.int64]:
    return np.zeros(0, dtype=np.int64)


def _s() -> NDArray[np.str_]:
    return np.zeros(0, dtype=str)


@dataclass
class SessionBag:
    name: str = ""
    #: Log time of the first /scan in seconds since the epoch (the zero of the axis).
    t0_log_s: float = 0.0
    #: /drive_raw: receive time, header stamp, steering (rad), speed (m/s).
    draw_t: NDArray[np.float64] = field(default_factory=_f)
    draw_stamp: NDArray[np.float64] = field(default_factory=_f)
    draw_steer: NDArray[np.float64] = field(default_factory=_f)
    draw_speed: NDArray[np.float64] = field(default_factory=_f)
    #: /drive (safety_node's output, one per gate cycle): receive time, header stamp, steering,
    #: speed. The header stamp is the gate cycle's own time.
    drv_t: NDArray[np.float64] = field(default_factory=_f)
    drv_stamp: NDArray[np.float64] = field(default_factory=_f)
    drv_steer: NDArray[np.float64] = field(default_factory=_f)
    drv_speed: NDArray[np.float64] = field(default_factory=_f)
    #: /pnp/planner/status: receive time, header stamp, state, cycle, solve time (s).
    st_t: NDArray[np.float64] = field(default_factory=_f)
    st_stamp: NDArray[np.float64] = field(default_factory=_f)
    st_state: NDArray[np.int64] = field(default_factory=_i)
    st_cycle: NDArray[np.int64] = field(default_factory=_i)
    st_solve_s: NDArray[np.float64] = field(default_factory=_f)
    #: /safety/events: receive time, message stamp (the gate cycle), severity, phase, source, detail.
    ev_t: NDArray[np.float64] = field(default_factory=_f)
    ev_stamp: NDArray[np.float64] = field(default_factory=_f)
    ev_severity: NDArray[np.int64] = field(default_factory=_i)
    ev_phase: NDArray[np.int64] = field(default_factory=_i)
    ev_source: NDArray[np.str_] = field(default_factory=_s)
    ev_detail: NDArray[np.str_] = field(default_factory=_s)
    #: /lap/event (camera counter): receive time, lap, lap time (s), path length (m).
    cam_t: NDArray[np.float64] = field(default_factory=_f)
    cam_lap: NDArray[np.int64] = field(default_factory=_i)
    cam_lap_time_s: NDArray[np.float64] = field(default_factory=_f)
    cam_distance_m: NDArray[np.float64] = field(default_factory=_f)
    #: /pnp/lap/event: receive time, lap index, lap time (s).
    pnp_t: NDArray[np.float64] = field(default_factory=_f)
    pnp_lap_index: NDArray[np.int64] = field(default_factory=_i)
    pnp_lap_time_s: NDArray[np.float64] = field(default_factory=_f)

    def save(self, path: Path) -> None:
        arrays = {f.name: getattr(self, f.name) for f in fields(self) if f.name != "name"}
        arrays["name"] = np.array(self.name)
        np.savez_compressed(path, **arrays)  # type: ignore[arg-type]

    @classmethod
    def load(cls, path: Path) -> SessionBag:
        out = cls()
        with np.load(path, allow_pickle=False) as data:
            for f in fields(cls):
                if f.name in data.files:
                    value = data[f.name]
                    setattr(out, f.name, str(value) if f.name == "name" else value)
        out.t0_log_s = float(out.t0_log_s)
        return out


def _stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def bag_files(bag: Path) -> list[Path]:
    files = [bag] if bag.is_file() else sorted(bag.glob("*.mcap"))
    if not files:
        raise FileNotFoundError(f"no .mcap file in {bag}")
    return files


def read_session_bag(bag: Path) -> SessionBag:
    """Read a bag (a .mcap file or a directory holding .mcap files)."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    t0_ns: int | None = None
    rows: dict[str, list[tuple[Any, ...]]] = {k: [] for k in TOPICS}
    for file in bag_files(bag):
        with file.open("rb") as stream:
            reader = make_reader(stream, decoder_factories=[DecoderFactory()])
            if t0_ns is None:
                for _s, _c, message in reader.iter_messages(topics=[SCAN]):
                    t0_ns = int(message.log_time)
                    break
            for _s, channel, message, msg in reader.iter_decoded_messages(topics=list(TOPICS)):
                t = int(message.log_time)
                topic = channel.topic
                if topic in (DRIVE_RAW, DRIVE):
                    rows[topic].append(
                        (
                            t,
                            _stamp_ns(msg.header.stamp),
                            float(msg.drive.steering_angle),
                            float(msg.drive.speed),
                        )
                    )
                elif topic == STATUS:
                    rows[topic].append(
                        (
                            t,
                            _stamp_ns(msg.header.stamp),
                            int(msg.state),
                            int(msg.cycle),
                            float(msg.solve_time_s),
                        )
                    )
                elif topic == EVENTS:
                    rows[topic].append(
                        (
                            t,
                            _stamp_ns(msg.stamp),
                            int(msg.severity),
                            int(msg.phase),
                            str(msg.source),
                            str(msg.detail),
                        )
                    )
                elif topic == LAP_CAMERA:
                    d = json.loads(msg.data)
                    rows[topic].append(
                        (t, int(d["lap"]), float(d["lap_time_s"]), float(d["distance_m"]))
                    )
                elif topic == LAP_PNP:
                    rows[topic].append((t, int(msg.lap_index), float(msg.lap_time_s)))
    if t0_ns is None:
        raise ValueError(f"{bag}: no /scan message, so the bag axis has no zero")
    return bag_from_rows(bag.name, t0_ns, rows)


def bag_from_rows(name: str, t0_ns: int, rows: dict[str, list[tuple[Any, ...]]]) -> SessionBag:
    """Arrays from decoded rows (times in integer nanoseconds), moved onto the bag axis."""

    def axis(topic: str, i: int) -> NDArray[np.float64]:
        ns = np.array([r[i] for r in rows[topic]], dtype=np.int64)
        return (ns - np.int64(t0_ns)).astype(np.float64) / 1e9

    def col(topic: str, i: int, dtype: Any) -> Any:
        return np.array([r[i] for r in rows[topic]], dtype=dtype)

    out = SessionBag(name=name, t0_log_s=t0_ns / 1e9)
    out.draw_t, out.draw_stamp = axis(DRIVE_RAW, 0), axis(DRIVE_RAW, 1)
    out.draw_steer, out.draw_speed = col(DRIVE_RAW, 2, float), col(DRIVE_RAW, 3, float)
    out.drv_t, out.drv_stamp = axis(DRIVE, 0), axis(DRIVE, 1)
    out.drv_steer, out.drv_speed = col(DRIVE, 2, float), col(DRIVE, 3, float)
    out.st_t, out.st_stamp = axis(STATUS, 0), axis(STATUS, 1)
    out.st_state, out.st_cycle = col(STATUS, 2, np.int64), col(STATUS, 3, np.int64)
    out.st_solve_s = col(STATUS, 4, float)
    out.ev_t, out.ev_stamp = axis(EVENTS, 0), axis(EVENTS, 1)
    out.ev_severity, out.ev_phase = col(EVENTS, 2, np.int64), col(EVENTS, 3, np.int64)
    out.ev_source, out.ev_detail = col(EVENTS, 4, str), col(EVENTS, 5, str)
    out.cam_t = axis(LAP_CAMERA, 0)
    out.cam_lap, out.cam_lap_time_s = col(LAP_CAMERA, 1, np.int64), col(LAP_CAMERA, 2, float)
    out.cam_distance_m = col(LAP_CAMERA, 3, float)
    out.pnp_t = axis(LAP_PNP, 0)
    out.pnp_lap_index, out.pnp_lap_time_s = col(LAP_PNP, 1, np.int64), col(LAP_PNP, 2, float)
    return out
