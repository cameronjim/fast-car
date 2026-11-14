"""The decisions of scan_deskew_node, ROS-free: which twist a sweep gets, when it passes through
unchanged, and the numbers of the 1 Hz status.

Twist of a sweep. The wheel speed is `/odom/wheel` and the steering is the gated `/drive` command,
both kept as short histories on their header stamps (a zero stamp falls back to the arrival time).
The sweep's middle on the ROS clock is `stamp + T/2` when header.stamp is the first beam's time and
`stamp - T/2` when it is the last beam's (T = the sweep time of the message). The twist is the
newest speed and steering sample at or before that time. So `stamp_is` is not only the output
offset: it decides which odometry a sweep is paired with, which is what the selection on real
bags (M-DSK, step A) can tell apart. Both are in the past at the time the scan arrives, so nothing
waits for data.

    yaw rate = gain(|delta|) * v * tan(delta) / L

with the yaw gain table and the wheelbase from vehicle_params: the formula racer_laps' gate and
racer_state's wheel_twist_node use (np.interp holds the first and last gain outside the knots).

A scan passes through UNCHANGED (original stamp and timing fields), and the reason is counted, when:
no wheel speed has arrived (`no_odom`), the newest one is older than `max_twist_age_s` or the
sample the sweep needs is (`twist_stale`), the clock went backwards (`clock_jump`), no steering
has arrived (`no_steering`), the car is at standstill (`standstill`), the message has no timing or
no usable stamp (`no_timing`, `bad_stamp`), or the core raised (`exception`). Odometry and drive
samples whose stamp does not advance are ignored (a stuck driver resending its last message is not
fresh data), and counted.
"""

from __future__ import annotations

import json
import math
import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from racer_track_perception import deskew
from racer_track_perception.vehicle import BodyFootprint, LidarMount

HISTORY_LEN = 100  # samples kept per stream (2 s of 50 Hz)
#: Wheel speed below this at the sweep's middle is standstill: the scan is passed through.
#: Engineering choice (not from the pre-registration): the encoder's resolution is far finer, and
#: at 0.02 m/s a sweep moves 2 mm.
STANDSTILL_MPS = 0.02
LATENCY_WINDOW = 1000

REASONS = (
    "no_odom",
    "twist_stale",
    "clock_jump",
    "no_steering",
    "standstill",
    "no_timing",
    "bad_stamp",
    "exception",
)


def yaw_gain(table: tuple[tuple[float, float], ...], abs_delta: float) -> float:
    """Piecewise linear in |delta|, held at the first and last gain outside the knots."""
    if not table:
        return 1.0
    return float(np.interp(abs_delta, [k[0] for k in table], [k[1] for k in table]))


def wheel_yaw_rate(
    v_mps: float, steer_rad: float, wheelbase_m: float, table: tuple[tuple[float, float], ...]
) -> float:
    """gain(|delta|) v tan(delta) / L."""
    return yaw_gain(table, abs(steer_rad)) * v_mps * math.tan(steer_rad) / wheelbase_m


@dataclass
class RelayResult:
    """What to publish for one scan. `ranges` None means: republish the input unchanged."""

    ranges: NDArray[np.float32] | None
    stamp_offset_s: float
    reason: str  # "" when deskewed, else one of REASONS
    twist: deskew.Twist | None
    twist_age_s: float | None
    process_ms: float


class _History:
    """(stamp, value) samples with strictly increasing stamps."""

    def __init__(self) -> None:
        self.stamps: deque[float] = deque(maxlen=HISTORY_LEN)
        self.values: deque[float] = deque(maxlen=HISTORY_LEN)
        self.last_arrival_s: float | None = None
        self.rejected = 0

    def add(self, arrival_s: float, stamp_s: float, value: float) -> None:
        if not (math.isfinite(arrival_s) and math.isfinite(value)):
            self.rejected += 1
            return
        if not (math.isfinite(stamp_s) and stamp_s > 0.0):
            stamp_s = arrival_s
        if self.stamps and not stamp_s > self.stamps[-1]:
            self.rejected += 1
            return
        self.stamps.append(stamp_s)
        self.values.append(value)
        self.last_arrival_s = arrival_s

    def at(self, t_s: float) -> tuple[float, float] | None:
        """(stamp, value) of the newest sample at or before t_s, or None."""
        for stamp, value in zip(reversed(self.stamps), reversed(self.values), strict=True):
            if stamp <= t_s:
                return stamp, value
        return None


class DeskewRelay:
    def __init__(
        self,
        cfg: deskew.DeskewConfig,
        mount: LidarMount,
        body: BodyFootprint,
        yaw_gain_table: tuple[tuple[float, float], ...],
        standstill_mps: float = STANDSTILL_MPS,
    ) -> None:
        self.cfg = cfg
        self.mount = mount
        self.body = body
        self.wheelbase_m = body.wheelbase_m
        self.table = tuple((float(a), float(g)) for a, g in yaw_gain_table)
        self.standstill_mps = standstill_mps
        self.odom = _History()
        self.drive = _History()

    def on_odom(self, arrival_s: float, stamp_s: float, v_mps: float) -> None:
        self.odom.add(arrival_s, stamp_s, v_mps)

    def on_drive(self, arrival_s: float, stamp_s: float, steer_rad: float) -> None:
        self.drive.add(arrival_s, stamp_s, steer_rad)

    def _pass(self, reason: str, age: float | None, t0: float) -> RelayResult:
        return RelayResult(None, 0.0, reason, None, age, (time.perf_counter() - t0) * 1000.0)

    def twist_for(
        self, now_s: float, mid_s: float
    ) -> tuple[deskew.Twist | None, str, float | None]:
        """(twist, "", age) for a sweep whose middle is `mid_s` on the ROS clock, or
        (None, reason, age)."""
        if self.odom.last_arrival_s is None:
            return None, "no_odom", None
        age = now_s - self.odom.last_arrival_s
        if age < 0.0:
            return None, "clock_jump", age
        if age > self.cfg.max_twist_age_s:
            return None, "twist_stale", age
        speed = self.odom.at(mid_s)
        if speed is None or mid_s - speed[0] > self.cfg.max_twist_age_s:
            return None, "twist_stale", age
        if self.drive.last_arrival_s is None:
            return None, "no_steering", age
        steer = self.drive.at(mid_s)
        if steer is None:
            # the first steering sample is newer than the sweep's middle: the earliest one holds
            steer = (self.drive.stamps[0], self.drive.values[0])
        v, delta = speed[1], steer[1]
        return deskew.Twist(v, wheel_yaw_rate(v, delta, self.wheelbase_m, self.table)), "", age

    def process(
        self,
        now_s: float,
        stamp_s: float,
        ranges: NDArray[np.floating],
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        scan_time_s: float,
        time_increment_s: float,
    ) -> RelayResult:
        t0 = time.perf_counter()
        try:
            n = len(ranges)
            t = deskew.beam_times(n, scan_time_s, time_increment_s)
            sweep_s = float(t.max()) if n else 0.0
            if not sweep_s > 0.0:
                return self._pass("no_timing", None, t0)
            if not (math.isfinite(stamp_s) and stamp_s > 0.0):
                return self._pass("bad_stamp", None, t0)
            half = 0.5 * sweep_s
            mid_s = stamp_s + half if self.cfg.stamp_is == "sweep_start" else stamp_s - half
            twist, reason, age = self.twist_for(now_s, mid_s)
            if twist is None:
                return self._pass(reason, age, t0)
            if abs(twist.v_mps) < self.standstill_mps:
                return self._pass("standstill", age, t0)
            static = deskew.body_beams(ranges, angle_min, angle_increment, self.mount, self.body)
            out, offset = deskew.deskew_scan(
                np.asarray(ranges, dtype=np.float64),
                angle_min,
                angle_increment,
                scan_time_s,
                time_increment_s,
                twist,
                self.mount,
                self.cfg,
                range_min=range_min,
                range_max=range_max,
                static=static,
            )
            return RelayResult(
                out.astype(np.float32),
                offset,
                "",
                twist,
                age,
                (time.perf_counter() - t0) * 1000.0,
            )
        except Exception:  # noqa: BLE001 - the relay must never lose a scan
            return self._pass("exception", None, t0)


@dataclass
class DeskewStats:
    scans_in: int = 0
    scans_out: int = 0
    deskewed: int = 0
    passed: dict[str, int] = field(default_factory=lambda: {r: 0 for r in REASONS})
    last_twist_age_s: float | None = None
    last_scan_arrival_s: float | None = None
    _ms: deque[float] = field(default_factory=lambda: deque(maxlen=LATENCY_WINDOW))
    _max_ms: float = 0.0

    def scan_in(self, now_s: float) -> None:
        self.scans_in += 1
        self.last_scan_arrival_s = now_s

    def scan_out(self, result: RelayResult) -> None:
        self.scans_out += 1
        if result.twist_age_s is not None and math.isfinite(result.twist_age_s):
            self.last_twist_age_s = result.twist_age_s
        if result.reason:
            self.passed[result.reason] = self.passed.get(result.reason, 0) + 1
            return
        self.deskewed += 1
        self._ms.append(result.process_ms)
        self._max_ms = max(self._max_ms, result.process_ms)

    def as_dict(self, now_s: float, odom_rejected: int = 0, drive_rejected: int = 0) -> dict:
        ms = np.array(self._ms, dtype=np.float64)
        pct = lambda q: float(np.percentile(ms, q)) if ms.size else None
        age = None if self.last_scan_arrival_s is None else now_s - self.last_scan_arrival_s
        return {
            "state": "no_scan" if age is None or age > 1.0 else "running",
            "scans_in": self.scans_in,
            "scans_out": self.scans_out,
            "scans_deskewed": self.deskewed,
            "scans_passed_through": sum(self.passed.values()),
            "passed_through_by_reason": dict(self.passed),
            "twist_age_s": self.last_twist_age_s,
            "odom_samples_ignored": odom_rejected,
            "drive_samples_ignored": drive_rejected,
            "process_ms": {
                "p50": pct(50),
                "p99": pct(99),
                "max": float(self._max_ms) if ms.size else None,
            },
        }

    def as_json(self, now_s: float, odom_rejected: int = 0, drive_rejected: int = 0) -> str:
        return json.dumps(self.as_dict(now_s, odom_rejected, drive_rejected), allow_nan=False)
