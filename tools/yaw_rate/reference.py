"""The reference: yaw change between consecutive /pose messages, and the jump filter.

Between two /pose updates the map to base_link transform is carried by wheel odometry, so /tf is
not an independent reference at 10 Hz; only the pose messages are (pre-registration section 3.4).
A pose interval is [stamp of pose k, stamp of pose k + 1]. The /pose header stamp is the stamp of
the scan the pose was computed from, so the intervals line up with scan stamps exactly.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

MAX_INTERVAL_S = 1.5  # intervals longer than this are dropped (the pose stalled)
JUMP_TOLERANCE_M = 0.3  # pose motion vs wheel distance beyond this is a relocalisation

Array = NDArray[np.float64]


def wrap(angle: Array | float) -> Array:
    """Angles wrapped to [-pi, pi)."""
    return (np.asarray(angle, dtype=float) + np.pi) % (2.0 * np.pi) - np.pi


@dataclass(frozen=True)
class Intervals:
    """Pose intervals: stamps of both ends, the wrapped yaw change and the straight-line
    position change (chord)."""

    t0: Array
    t1: Array
    dyaw: Array
    chord: Array

    def __len__(self) -> int:
        return len(self.t0)

    @property
    def dt(self) -> Array:
        return self.t1 - self.t0

    @property
    def rate(self) -> Array:
        """Interval-mean yaw rate of the reference (rad/s)."""
        return self.dyaw / self.dt

    def select(self, mask: NDArray[np.bool_]) -> Intervals:
        return Intervals(self.t0[mask], self.t1[mask], self.dyaw[mask], self.chord[mask])


def pose_intervals(
    stamp: Array, x: Array, y: Array, yaw: Array, min_interval_s: float = 0.0
) -> Intervals:
    """Intervals between consecutive pose messages. With `min_interval_s` > 0 consecutive
    messages are merged until the interval is at least that long (the AMCL pose arrives at the
    scan rate while the car moves, and a 0.1 s yaw difference is mostly pose noise)."""
    n = len(stamp)
    if n < 2:
        empty = np.zeros(0)
        return Intervals(empty, empty, empty, empty)
    if min_interval_s <= 0.0:
        starts = np.arange(n - 1)
        ends = starts + 1
    else:
        s_list: list[int] = []
        e_list: list[int] = []
        start = 0
        for k in range(1, n):
            if stamp[k] - stamp[start] >= min_interval_s:
                s_list.append(start)
                e_list.append(k)
                start = k
        starts = np.array(s_list, dtype=int)
        ends = np.array(e_list, dtype=int)
    return Intervals(
        t0=stamp[starts],
        t1=stamp[ends],
        dyaw=wrap(yaw[ends] - yaw[starts]),
        chord=np.hypot(x[ends] - x[starts], y[ends] - y[starts]),
    )


@dataclass(frozen=True)
class FilterResult:
    keep: NDArray[np.bool_]
    n_total: int
    n_too_long: int
    n_jumped: int


def expected_chord(path_m: Array, dyaw: Array) -> Array:
    """Straight-line distance after driving `path_m` along an arc that turns the heading by
    `dyaw`: s * sin(dyaw / 2) / (dyaw / 2). A wheel odometer measures the arc, the pose message
    the chord, and in a 0.83 m radius corner they differ by more than the jump tolerance."""
    return np.asarray(path_m) * np.sinc(np.asarray(dyaw) / (2.0 * np.pi))


def filter_intervals(
    iv: Intervals,
    wheel_path_m: Array,
    max_interval_s: float = MAX_INTERVAL_S,
    jump_tolerance_m: float = JUMP_TOLERANCE_M,
) -> FilterResult:
    """Drop intervals longer than `max_interval_s`, then intervals whose pose chord differs
    from the wheel path (as a chord) by more than `jump_tolerance_m` (relocalisation)."""
    too_long = iv.dt > max_interval_s
    jumped = ~too_long & (
        np.abs(iv.chord - expected_chord(wheel_path_m, iv.dyaw)) > jump_tolerance_m
    )
    return FilterResult(
        keep=~(too_long | jumped),
        n_total=len(iv),
        n_too_long=int(too_long.sum()),
        n_jumped=int(jumped.sum()),
    )
