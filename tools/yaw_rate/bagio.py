"""The topics the yaw rate measurement needs, from an mcap bag, as numpy arrays (cached as npz).

Keys (times in seconds after the first /scan's log time, the axis of tools/pnp_replay/bag.py and
of the split file; `*_stamp` are header stamps on the same axis):

    scan_t, scan_stamp, scan_angle_min, scan_angle_increment, scan_range_min, scan_range_max,
    scan_q (uint16, 0.25 mm quanta, see pnp_replay.bag)
    wt_t, wt_stamp, wt_vx, wt_wz        /odom/wheel_twist (the wheel model's twist)
    odom_t, odom_v                      /odom/wheel linear speed
    drive_t, drive_speed, drive_steer   /drive (the gated command)
    pose_t, pose_stamp, pose_x, pose_y, pose_yaw   /pose (map frame)
"""

from __future__ import annotations

import logging
import math
import time
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
from pnp_replay.bag import dequantise_ranges, quantise_ranges

log = logging.getLogger(__name__)

SCAN_TOPIC = "/scan"
TWIST_TOPIC = "/odom/wheel_twist"
ODOM_TOPIC = "/odom/wheel"
DRIVE_TOPIC = "/drive"
POSE_TOPIC = "/pose"
TOPICS = (SCAN_TOPIC, TWIST_TOPIC, ODOM_TOPIC, DRIVE_TOPIC, POSE_TOPIC)
CACHE_VERSION = 1

Recording = dict[str, NDArray[Any]]


def _stamp(header: Any) -> float:
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rad) of a quaternion, about +z."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def extract(bag: Path) -> Recording:
    """Read the topics of a bag (a .mcap file or a directory holding one) through its index."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    files = [bag] if bag.is_file() else sorted(bag.glob("*.mcap"))
    if not files:
        raise FileNotFoundError(f"no .mcap file in {bag}")
    cols: dict[str, list[tuple[Any, ...]]] = {
        k: [] for k in ("scan", "wt", "odom", "drive", "pose")
    }
    started = time.perf_counter()
    count = 0
    for file in files:
        with file.open("rb") as stream:
            reader = make_reader(stream, decoder_factories=[DecoderFactory()])
            for _s, channel, message, msg in reader.iter_decoded_messages(topics=list(TOPICS)):
                t = message.log_time / 1e9
                topic = channel.topic
                count += 1
                if count % 20000 == 0:
                    log.info("  %d messages, %.0f s", count, time.perf_counter() - started)
                if topic == SCAN_TOPIC:
                    cols["scan"].append(
                        (
                            t,
                            _stamp(msg.header),
                            msg.angle_min,
                            msg.angle_increment,
                            msg.range_min,
                            msg.range_max,
                            quantise_ranges(np.asarray(msg.ranges, dtype=np.float32)),
                        )
                    )
                elif topic == TWIST_TOPIC:
                    tw = msg.twist.twist
                    cols["wt"].append((t, _stamp(msg.header), tw.linear.x, tw.angular.z))
                elif topic == ODOM_TOPIC:
                    cols["odom"].append((t, float(msg.twist.twist.linear.x)))
                elif topic == DRIVE_TOPIC:
                    cols["drive"].append((t, msg.drive.speed, msg.drive.steering_angle))
                elif topic == POSE_TOPIC:
                    p = msg.pose.pose
                    q = p.orientation
                    cols["pose"].append(
                        (
                            t,
                            _stamp(msg.header),
                            p.position.x,
                            p.position.y,
                            quaternion_yaw(q.x, q.y, q.z, q.w),
                        )
                    )
    if not cols["scan"]:
        raise ValueError(f"{bag}: no /scan messages")
    log.info("read %d messages in %.0f s", count, time.perf_counter() - started)
    return assemble(cols)


def assemble(cols: dict[str, list[tuple[Any, ...]]]) -> Recording:
    t0 = float(cols["scan"][0][0])
    rec: Recording = {"t0_log_s": np.array(t0), "cache_version": np.array(CACHE_VERSION)}

    def col(group: str, i: int, shift: bool = False) -> NDArray[np.float64]:
        v = np.array([row[i] for row in cols[group]], dtype=np.float64)
        return v - t0 if shift else v

    s = cols["scan"]
    rec["scan_t"] = col("scan", 0, True)
    rec["scan_stamp"] = col("scan", 1, True)
    rec["scan_angle_min"] = col("scan", 2)
    rec["scan_angle_increment"] = col("scan", 3)
    rec["scan_range_min"] = col("scan", 4)
    rec["scan_range_max"] = col("scan", 5)
    rec["scan_q"] = np.stack([row[6] for row in s])
    rec["wt_t"] = col("wt", 0, True)
    rec["wt_stamp"] = col("wt", 1, True)
    rec["wt_vx"] = col("wt", 2)
    rec["wt_wz"] = col("wt", 3)
    rec["odom_t"] = col("odom", 0, True)
    rec["odom_v"] = col("odom", 1)
    rec["drive_t"] = col("drive", 0, True)
    rec["drive_speed"] = col("drive", 1)
    rec["drive_steer"] = col("drive", 2)
    rec["pose_t"] = col("pose", 0, True)
    rec["pose_stamp"] = col("pose", 1, True)
    rec["pose_x"] = col("pose", 2)
    rec["pose_y"] = col("pose", 3)
    rec["pose_yaw"] = col("pose", 4)
    return rec


def load_recording(bag: Path, cache: Path | None) -> Recording:
    """The bag's recording from its npz cache when it is newer than the bag, else read the bag
    (and write the cache). `bag` may itself be an .npz (a test fixture)."""
    if bag.suffix == ".npz":
        return read_recording(bag)
    if cache is not None and cache.is_file() and cache.stat().st_mtime >= bag.stat().st_mtime:
        try:
            return read_recording(cache)
        except ValueError as error:
            log.warning("%s; reading the bag again", error)
    rec = extract(bag)
    if cache is not None:
        cache.parent.mkdir(parents=True, exist_ok=True)
        np.savez(cache, **rec)
    return rec


def read_recording(path: Path) -> Recording:
    with np.load(path, allow_pickle=False) as data:
        rec = {key: data[key] for key in data.files}
    if int(rec.get("cache_version", -1)) != CACHE_VERSION:
        raise ValueError(f"{path}: cache version {rec.get('cache_version')} is not {CACHE_VERSION}")
    return rec


def scan_ranges(rec: Recording, index: int) -> NDArray[np.float32]:
    return dequantise_ranges(rec["scan_q"][index])
