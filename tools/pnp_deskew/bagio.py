"""The topics scan deskew needs, from an mcap bag, as numpy arrays (cached as npz).

The existing recordings (tools/pnp_replay/bag.py) drop the two timing fields of the LiDAR message
(`scan_time`, `time_increment`) and the header stamp of the wheel odometry, which deskew needs, so
this reads them again. Times are seconds after the first /scan's log time (recorder arrival time,
the axis of the split file). Scans are stored as uint16 quarter millimetres like bag.py.

    scan_t, scan_stamp, scan_time, scan_dt (time_increment), scan_angle_min,
    scan_angle_increment, scan_range_min, scan_range_max, scan_q [n, beams]
    odom_t, odom_stamp, odom_v            /odom/wheel arrival, header stamp, forward speed
    drive_t, drive_steer                  /drive (the gated command), arrival and steering
"""

from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
from pnp_replay.bag import dequantise_ranges, quantise_ranges

log = logging.getLogger(__name__)

CACHE_VERSION = 1
Recording = dict[str, NDArray[Any]]


def _stamp(header: Any) -> float:
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


def extract(bag: Path) -> Recording:
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    files = [bag] if bag.is_file() else sorted(bag.glob("*.mcap"))
    if not files:
        raise FileNotFoundError(f"no .mcap file in {bag}")
    scan: list[tuple[Any, ...]] = []
    odom: list[tuple[float, float, float]] = []
    drive: list[tuple[float, float]] = []
    started = time.perf_counter()
    count = 0
    for file in files:
        with file.open("rb") as stream:
            reader = make_reader(stream, decoder_factories=[DecoderFactory()])
            for _s, channel, message, msg in reader.iter_decoded_messages(
                topics=["/scan", "/odom/wheel", "/drive"]
            ):
                t = message.log_time / 1e9
                count += 1
                if count % 20000 == 0:
                    log.info("  %d messages, %.0f s", count, time.perf_counter() - started)
                if channel.topic == "/scan":
                    scan.append(
                        (
                            t,
                            _stamp(msg.header),
                            float(msg.scan_time),
                            float(msg.time_increment),
                            float(msg.angle_min),
                            float(msg.angle_increment),
                            float(msg.range_min),
                            float(msg.range_max),
                            quantise_ranges(np.asarray(msg.ranges, dtype=np.float32)),
                        )
                    )
                elif channel.topic == "/odom/wheel":
                    odom.append((t, _stamp(msg.header), float(msg.twist.twist.linear.x)))
                else:
                    drive.append((t, float(msg.drive.steering_angle)))
    if not scan:
        raise ValueError(f"{bag}: no /scan messages")
    t0 = float(scan[0][0])
    log.info("read %d messages in %.0f s", count, time.perf_counter() - started)
    col = lambda rows, i, shift=False: (
        np.array([r[i] for r in rows], dtype=np.float64) - (t0 if shift else 0.0)
    )
    return {
        "cache_version": np.array(CACHE_VERSION),
        "t0_log_s": np.array(t0),
        "scan_t": col(scan, 0, True),
        "scan_stamp": col(scan, 1, True),
        "scan_time": col(scan, 2),
        "scan_dt": col(scan, 3),
        "scan_angle_min": col(scan, 4),
        "scan_angle_increment": col(scan, 5),
        "scan_range_min": col(scan, 6),
        "scan_range_max": col(scan, 7),
        "scan_q": np.stack([r[8] for r in scan]),
        "odom_t": col(odom, 0, True),
        "odom_stamp": col(odom, 1, True),
        "odom_v": col(odom, 2),
        "drive_t": col(drive, 0, True),
        "drive_steer": col(drive, 1),
    }


def load_recording(bag: Path, cache: Path | None) -> Recording:
    """The recording from its npz cache, else read the bag (a few minutes) and write the cache."""
    if cache is not None and cache.is_file() and cache.stat().st_mtime >= bag.stat().st_mtime:
        with np.load(cache) as z:
            rec = {k: z[k] for k in z.files}
        if int(rec.get("cache_version", -1)) == CACHE_VERSION:
            return rec
        log.warning("%s: cache version differs, reading the bag again", cache)
    rec = extract(bag)
    if cache is not None:
        cache.parent.mkdir(parents=True, exist_ok=True)
        np.savez(cache, **rec)
    return rec


def scan_ranges(rec: Recording, i: int) -> NDArray[np.float32]:
    return dequantise_ranges(rec["scan_q"][i])
