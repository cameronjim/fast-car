"""Read the topics the analysis needs from an mcap bag (needs the `bags` dependency group).

Real-data glue, excluded from coverage like the other packages' bag readers. Only the wanted
channels are decoded. `/scan` is only read for the log time of its first message (the zero of the
bag axis), without decoding it.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any

import numpy as np

from racer_eval.recording import BagData

log = logging.getLogger(__name__)

SCAN = "/scan"
DRIVE_RAW = "/drive_raw"
STATUS = "/pnp/planner/status"
EVENTS = "/safety/events"
ODOM = "/odom/wheel"
LAP_CAMERA = "/lap/event"
LAP_PNP = "/pnp/lap/event"
_DECODED = (DRIVE_RAW, STATUS, EVENTS, ODOM, LAP_CAMERA, LAP_PNP)


def read_bag(bag: Path) -> BagData:
    """Read a bag (a .mcap file or a directory holding .mcap files)."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    files = [bag] if bag.is_file() else sorted(bag.glob("*.mcap"))
    if not files:
        raise FileNotFoundError(f"no .mcap file in {bag}")
    t0: float | None = None
    rows: dict[str, list[tuple[Any, ...]]] = {k: [] for k in _DECODED}
    for file in files:
        with file.open("rb") as stream:
            reader = make_reader(stream, decoder_factories=[DecoderFactory()])
            if t0 is None:
                for _s, _c, message in reader.iter_messages(topics=[SCAN]):
                    t0 = message.log_time / 1e9
                    break
            for _s, channel, message, msg in reader.iter_decoded_messages(topics=list(_DECODED)):
                t = message.log_time / 1e9
                topic = channel.topic
                if topic == DRIVE_RAW:
                    rows[topic].append((t, float(msg.drive.speed)))
                elif topic == STATUS:
                    rows[topic].append((t, int(msg.state)))
                elif topic == EVENTS:
                    rows[topic].append((t, int(msg.severity), int(msg.phase), str(msg.source)))
                elif topic == ODOM:
                    rows[topic].append((t, float(msg.twist.twist.linear.x)))
                elif topic == LAP_CAMERA:
                    d = json.loads(msg.data)
                    rows[topic].append(
                        (t, int(d["lap"]), float(d["lap_time_s"]), float(d["distance_m"]))
                    )
                elif topic == LAP_PNP:
                    rows[topic].append((t, int(msg.lap_index), float(msg.lap_time_s)))
    if t0 is None:
        raise ValueError(f"{bag}: no /scan message, so the bag axis has no zero")
    log.info("%s: first /scan at log time %.3f s", bag.name, t0)

    def col(topic: str, i: int, dtype: Any, shift: bool = False) -> Any:
        values = np.array([r[i] for r in rows[topic]], dtype=dtype)
        return values - t0 if shift else values

    data = BagData(name=bag.name, t0_log_s=t0)
    data.draw_t, data.draw_speed = col(DRIVE_RAW, 0, np.float64, True), col(DRIVE_RAW, 1, float)
    data.st_t, data.st_state = col(STATUS, 0, np.float64, True), col(STATUS, 1, np.int64)
    data.ev_t = col(EVENTS, 0, np.float64, True)
    data.ev_severity, data.ev_phase = col(EVENTS, 1, np.int64), col(EVENTS, 2, np.int64)
    data.ev_source = col(EVENTS, 3, str)
    data.odom_t, data.odom_v = col(ODOM, 0, np.float64, True), col(ODOM, 1, float)
    data.cam_t = col(LAP_CAMERA, 0, np.float64, True)
    data.cam_lap, data.cam_lap_time_s = col(LAP_CAMERA, 1, np.int64), col(LAP_CAMERA, 2, float)
    data.cam_distance_m = col(LAP_CAMERA, 3, float)
    data.pnp_t = col(LAP_PNP, 0, np.float64, True)
    data.pnp_lap_index, data.pnp_lap_time_s = col(LAP_PNP, 1, np.int64), col(LAP_PNP, 2, float)
    return data
