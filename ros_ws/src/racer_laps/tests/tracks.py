"""Synthetic pose tracks for the lap tests. Seeded, no ROS."""

from __future__ import annotations

import math
from collections.abc import Iterator

Pose = tuple[float, float, float, float]  # t, x, y, yaw


def circle_poses(
    loops: float,
    radius: float = 1.0,
    omega: float = 1.0,
    rate_hz: float = 20.0,
    t0: float = 0.0,
    phase_s: float = 0.0,
    clockwise: bool = False,
) -> Iterator[Pose]:
    """Poses on a circle of `radius` centred on (0, 0). The first sample is at angle 0 at time
    `t0`; later samples are on the grid `t0 + phase_s + i / rate_hz`. The true k-th return to
    angle 0 is at `t0 + 2 pi k / omega`; the track runs one sample period past the last one.

    Counter-clockwise: heading at (r, 0) is +y (pi/2). Clockwise: heading is -y.
    """
    sign = -1.0 if clockwise else 1.0
    total_t = loops * 2.0 * math.pi / omega
    yield (t0, radius, 0.0, sign * math.pi / 2.0)
    i = 0
    while True:
        t = t0 + phase_s + i / rate_hz
        i += 1
        if t <= t0:
            continue
        if t - t0 > total_t + 1.0 / rate_hz:
            return
        a = sign * omega * (t - t0)
        yield (t, radius * math.cos(a), radius * math.sin(a), a + sign * math.pi / 2.0)
