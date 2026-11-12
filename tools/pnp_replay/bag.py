"""Read the Plug-and-Play topics of an mcap bag into numpy arrays, and cache them.

A recording is a flat dict of numpy arrays whose keys start with a group name (`scan_`, `odom_`,
`drive_`, `draw_`, `ev_`, `trk_`, `st_`, `plan_`); every group has a `<group>_t` array, the
ARRIVAL time (the recorder's log time) in seconds after the first /scan. Header stamps are on the
same axis (`scan_stamp`, `trk_stamp`). The planner reads messages in arrival order, so arrival is
what a replay feeds.

Scans are stored as `scan_q`, uint16 in units of 0.25 mm (0 for a beam that is not finite): the
LiDAR reports in 0.25 mm steps, so this is lossless (checked on every scan at extraction) at a
quarter of the size of float32. `scan_ranges` turns them back into the float32 the driver saw.

The bag is large (6.3 GB for the 2026-11-12 run) and mostly not these topics, so the read goes
through the mcap summary index and only the wanted channels are decoded. The first read takes a
few minutes; `load_recording` caches the extraction as an .npz and the second read is instant.
"""

from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

log = logging.getLogger(__name__)

SCAN_TOPIC = "/scan"
ODOM_TOPIC = "/odom/wheel"
DRIVE_TOPIC = "/drive"
DRIVE_RAW_TOPIC = "/drive_raw"
EVENTS_TOPIC = "/safety/events"
TRACK_TOPIC = "/pnp/local_track"
STATUS_TOPIC = "/pnp/planner/status"
PLAN_TOPIC = "/pnp/plan"
TOPICS = (
    SCAN_TOPIC,
    ODOM_TOPIC,
    DRIVE_TOPIC,
    DRIVE_RAW_TOPIC,
    EVENTS_TOPIC,
    TRACK_TOPIC,
    STATUS_TOPIC,
    PLAN_TOPIC,
)

RANGE_QUANTUM_M = 0.00025
CACHE_VERSION = 1
# The numeric fields of racer_pnp_msgs/PlannerStatus, in the order of the message.
STATUS_FIELDS = (
    "state",
    "cycle",
    "solve_time_s",
    "cmd_speed_mps",
    "cmd_steering_rad",
    "est_speed_mps",
    "est_steering_rad",
    "v_cap_mps",
    "best_cost",
    "plan_progress_m",
    "plan_min_clearance_m",
    "collision_fraction",
    "track_age_s",
    "scan_age_s",
    "samples",
)
GROUPS = ("scan", "odom", "drive", "draw", "ev", "trk", "st", "plan")

Recording = dict[str, NDArray[Any]]


def quantise_ranges(ranges: NDArray[np.floating]) -> NDArray[np.uint16]:
    """Float ranges (m) to uint16 quarter millimetres, 0 for a beam that is not finite. Raises
    ValueError when a finite range does not fit or does not come back as the same float32."""
    r = np.asarray(ranges, dtype=np.float32)
    finite = np.isfinite(r) & (r > 0.0)  # 0 m is not a return
    q = np.zeros(r.shape, dtype=np.float64)
    q[finite] = np.round(r[finite].astype(np.float64) / RANGE_QUANTUM_M)
    if q.size and q.max() > np.iinfo(np.uint16).max:
        raise ValueError(f"a range of {r[finite].max()} m does not fit uint16 quarter millimetres")
    back = dequantise_ranges(q.astype(np.uint16))
    if not np.array_equal(back[finite], r[finite]):
        raise ValueError("ranges are not on the 0.25 mm grid; the quantised cache would be lossy")
    return q.astype(np.uint16)


def dequantise_ranges(q: NDArray[np.integer]) -> NDArray[np.float32]:
    """The float32 ranges (m) of `quantise_ranges`; 0 comes back as inf (not a return)."""
    out = q.astype(np.float32) / np.float32(1.0 / RANGE_QUANTUM_M)
    out[q == 0] = np.inf
    return out


def scan_ranges(rec: Recording, index: int) -> NDArray[np.float32]:
    return dequantise_ranges(rec["scan_q"][index])


def _stamp(header: Any) -> float:
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


def extract(bag: Path, topics: tuple[str, ...] = TOPICS) -> Recording:
    """Read the topics of a bag (a .mcap file, or a directory holding one) through its summary
    index. Times are returned relative to the first /scan's log time (`t0_log_s`)."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    files = [bag] if bag.is_file() else sorted(bag.glob("*.mcap"))
    if not files:
        raise FileNotFoundError(f"no .mcap file in {bag}")
    cols: dict[str, list[Any]] = {g: [] for g in GROUPS}
    started = time.perf_counter()
    count = 0
    for file in files:
        with file.open("rb") as stream:
            reader = make_reader(stream, decoder_factories=[DecoderFactory()])
            for _schema, channel, message, msg in reader.iter_decoded_messages(topics=list(topics)):
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
                elif topic == ODOM_TOPIC:
                    cols["odom"].append((t, float(msg.twist.twist.linear.x)))
                elif topic == DRIVE_TOPIC:
                    cols["drive"].append((t, msg.drive.speed, msg.drive.steering_angle))
                elif topic == DRIVE_RAW_TOPIC:
                    cols["draw"].append((t, msg.drive.speed, msg.drive.steering_angle))
                elif topic == EVENTS_TOPIC:
                    cols["ev"].append(
                        (t, int(msg.severity), int(msg.phase), str(msg.source), str(msg.detail))
                    )
                elif topic == TRACK_TOPIC:
                    cols["trk"].append(
                        (
                            t,
                            _stamp(msg.header),
                            np.asarray(msg.x_m, dtype=np.float32),
                            np.asarray(msg.y_m, dtype=np.float32),
                            np.asarray(msg.half_width_left_m, dtype=np.float32),
                            np.asarray(msg.half_width_right_m, dtype=np.float32),
                            np.asarray(msg.sigma_m, dtype=np.float32),
                            int(msg.valid_count),
                            float(msg.boundary_residual_m),
                            float(msg.intrusion_fraction),
                        )
                    )
                elif topic == STATUS_TOPIC:
                    cols["st"].append((t, *(float(getattr(msg, f)) for f in STATUS_FIELDS)))
                elif topic == PLAN_TOPIC:
                    cols["plan"].append((t, len(msg.poses)))
    if not cols["scan"]:
        raise ValueError(f"{bag}: no /scan messages")
    t0 = float(cols["scan"][0][0])
    log.info("read %d messages in %.0f s", count, time.perf_counter() - started)
    return _assemble(cols, t0)


def _assemble(cols: dict[str, list[Any]], t0: float) -> Recording:
    rec: Recording = {"t0_log_s": np.array(t0), "cache_version": np.array(CACHE_VERSION)}

    def col(group: str, index: int, dtype: Any, shift: bool = False) -> NDArray[Any]:
        values = np.array([row[index] for row in cols[group]], dtype=dtype)
        return values - t0 if shift else values

    s = cols["scan"]
    n_beams = len(s[0][6])
    rec["scan_t"] = col("scan", 0, np.float64, True)
    rec["scan_stamp"] = col("scan", 1, np.float64, True)
    rec["scan_angle_min"] = col("scan", 2, np.float64)
    rec["scan_angle_increment"] = col("scan", 3, np.float64)
    rec["scan_range_min"] = col("scan", 4, np.float64)
    rec["scan_range_max"] = col("scan", 5, np.float64)
    rec["scan_q"] = np.stack([row[6] for row in s]) if s else np.zeros((0, n_beams), np.uint16)
    for group, names in (
        ("odom", ("v",)),
        ("drive", ("speed", "steer")),
        ("draw", ("speed", "steer")),
    ):
        rec[f"{group}_t"] = col(group, 0, np.float64, True)
        for i, name in enumerate(names, start=1):
            rec[f"{group}_{name}"] = col(group, i, np.float64)
    ev = cols["ev"]
    rec["ev_t"] = col("ev", 0, np.float64, True)
    rec["ev_severity"] = col("ev", 1, np.int64)
    rec["ev_phase"] = col("ev", 2, np.int64)
    rec["ev_source"] = np.array([row[3] for row in ev], dtype=str)
    rec["ev_detail"] = np.array([row[4] for row in ev], dtype=str)
    rec["trk_t"] = col("trk", 0, np.float64, True)
    rec["trk_stamp"] = col("trk", 1, np.float64, True)
    for i, name in enumerate(("x", "y", "hl", "hr", "sigma"), start=2):
        tr = cols["trk"]
        rec[f"trk_{name}"] = (
            np.stack([row[i] for row in tr]).astype(np.float32)
            if tr
            else np.zeros((0, 16), np.float32)
        )
    rec["trk_valid"] = col("trk", 7, np.int64)
    rec["trk_residual"] = col("trk", 8, np.float64)
    rec["trk_intrusion"] = col("trk", 9, np.float64)
    rec["st_t"] = col("st", 0, np.float64, True)
    for i, name in enumerate(STATUS_FIELDS, start=1):
        rec[f"st_{name}"] = col("st", i, np.float64)
    rec["plan_t"] = col("plan", 0, np.float64, True)
    rec["plan_n"] = col("plan", 1, np.int64)
    return rec


def save_recording(rec: Recording, path: Path, compressed: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    (np.savez_compressed if compressed else np.savez)(path, **rec)


def read_recording(path: Path) -> Recording:
    with np.load(path, allow_pickle=False) as data:
        rec = {key: data[key] for key in data.files}
    if int(rec.get("cache_version", -1)) != CACHE_VERSION:
        raise ValueError(f"{path}: cache version {rec.get('cache_version')} is not {CACHE_VERSION}")
    return rec


def load_recording(bag: Path, cache: Path | None) -> Recording:
    """The bag's recording from its .npz cache when there is one, else read the bag (and write
    the cache). `bag` may itself be an .npz recording (a test fixture)."""
    if bag.suffix == ".npz":
        return read_recording(bag)
    if cache is not None and cache.is_file() and cache.stat().st_mtime >= bag.stat().st_mtime:
        log.info("reading the cache %s", cache)
        try:
            return read_recording(cache)
        except ValueError as error:
            log.warning("%s; reading the bag again", error)
    log.info("reading %s (indexed, a few minutes the first time)", bag)
    rec = extract(bag)
    if cache is not None:
        save_recording(rec, cache)
        log.info("cached %s", cache)
    return rec


def group_of(key: str) -> str | None:
    head = key.split("_", 1)[0]
    return head if head in GROUPS else None


def cut(rec: Recording, t_from: float, t_to: float) -> Recording:
    """The rows of every group whose arrival time is in [t_from, t_to]. Times stay on the bag's
    axis (not shifted to zero)."""
    out: Recording = {}
    masks = {
        g: (rec[f"{g}_t"] >= t_from) & (rec[f"{g}_t"] <= t_to) for g in GROUPS if f"{g}_t" in rec
    }
    for key, value in rec.items():
        group = group_of(key)
        out[key] = value[masks[group]] if group in masks and value.ndim >= 1 else value
    return out
