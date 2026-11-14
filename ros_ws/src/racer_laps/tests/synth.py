"""A synthetic stadium for the start-scan tests: walls, a car driving the centre line, a LiDAR
cast against the walls, wheel odometry and steering. Seeded, numpy only, no ROS.

The numbers here (lane width, mount, wheelbase) are the test world's own; the code under test
takes the same quantities as arguments.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Iterator
from dataclasses import dataclass

import numpy as np
from racer_laps.scan_match import Body, Mount

WHEELBASE_M = 0.33
MOUNT = Mount(0.285, 0.0, math.pi)
BODY = Body(-0.14, WHEELBASE_M + 0.11, 0.155)
N_BEAMS = 720
RANGE_MAX_M = 16.0


@dataclass(frozen=True)
class Stadium:
    straight_m: float = 2.0
    lane_m: float = 1.0
    island_r_m: float = 0.4
    arc_segments: int = 28

    @property
    def centre_r_m(self) -> float:
        return self.island_r_m + 0.5 * self.lane_m

    @property
    def length_m(self) -> float:
        return 2.0 * self.straight_m + 2.0 * math.pi * self.centre_r_m

    def walls(self) -> np.ndarray:
        """(M, 4) wall segments x0, y0, x1, y1: the island and the outer boundary."""
        segs: list[tuple[float, float, float, float]] = []
        for r in (self.island_r_m, self.island_r_m + self.lane_m):
            pts: list[tuple[float, float]] = []
            for k in range(self.arc_segments + 1):  # right end cap, from -90 to +90 degrees
                a = -math.pi / 2 + math.pi * k / self.arc_segments
                pts.append((self.straight_m + r * math.cos(a), r * math.sin(a)))
            for k in range(self.arc_segments + 1):  # left end cap, from 90 to 270 degrees
                a = math.pi / 2 + math.pi * k / self.arc_segments
                pts.append((r * math.cos(a), r * math.sin(a)))
            for i in range(len(pts)):
                x0, y0 = pts[i]
                x1, y1 = pts[(i + 1) % len(pts)]
                segs.append((x0, y0, x1, y1))
        return np.array(segs, dtype=float)

    def pose_at(self, s_m: float, clockwise: bool = False) -> tuple[float, float, float, float]:
        """(x, y, yaw, curvature) of the centre line at arc length s, counter-clockwise from the
        middle of the bottom straight (clockwise mirrors it)."""
        r = self.centre_r_m
        a = self.straight_m
        total = self.length_m
        s = (s_m + a / 2.0) % total  # start at the middle of the bottom straight
        if s < a:
            x, y, yaw, k = s, -r, 0.0, 0.0
        elif s < a + math.pi * r:
            th = (s - a) / r
            x, y, yaw, k = a + r * math.sin(th), -r * math.cos(th), th, 1.0 / r
        elif s < 2 * a + math.pi * r:
            x, y, yaw, k = a - (s - a - math.pi * r), r, math.pi, 0.0
        else:
            th = (s - 2 * a - math.pi * r) / r
            x, y, yaw, k = -r * math.sin(th), r * math.cos(th), math.pi + th, 1.0 / r
        if clockwise:
            return x, -y, -yaw, -k
        return x, y, yaw, k


def raycast(
    walls: np.ndarray, ox: float, oy: float, angles: np.ndarray, max_range: float = RANGE_MAX_M
) -> np.ndarray:
    """Range along each global bearing to the nearest wall segment (max_range + 1 if none)."""
    d = np.stack([np.cos(angles), np.sin(angles)], axis=1)  # (N, 2)
    p = walls[:, :2]
    e = walls[:, 2:] - walls[:, :2]  # (M, 2)
    # Solve o + t d = p + u e for t, u.
    denom = d[:, None, 0] * e[None, :, 1] - d[:, None, 1] * e[None, :, 0]  # (N, M)
    dx = p[None, :, 0] - ox
    dy = p[None, :, 1] - oy
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (dx * e[None, :, 1] - dy * e[None, :, 0]) / denom
        u = (dx * d[:, None, 1] - dy * d[:, None, 0]) / denom
    ok = (np.abs(denom) > 1e-12) & (t > 0.0) & (u >= 0.0) & (u <= 1.0)
    t = np.where(ok, t, np.inf)
    out = t.min(axis=1)
    return np.where(np.isfinite(out), out, max_range + 1.0)


def scan_at(
    walls: np.ndarray,
    x: float,
    y: float,
    yaw: float,
    rng: np.random.Generator,
    noise_m: float = 0.02,
    mount: Mount = MOUNT,
) -> tuple[np.ndarray, float, float]:
    """Ranges of a 720 beam, full circle scan from the car at (x, y, yaw) (base_link), and the
    (angle_min, angle_increment) of the scan."""
    angle_min = -math.pi
    inc = 2.0 * math.pi / N_BEAMS
    laser_angles = angle_min + inc * np.arange(N_BEAMS)
    ox = x + math.cos(yaw) * mount.x_m - math.sin(yaw) * mount.y_m
    oy = y + math.sin(yaw) * mount.x_m + math.cos(yaw) * mount.y_m
    ranges = raycast(walls, ox, oy, yaw + mount.yaw_rad + laser_angles)
    ranges = ranges + rng.normal(0.0, noise_m, ranges.shape)
    return ranges, angle_min, inc


@dataclass(frozen=True)
class Tick:
    t: float
    kind: str  # "odom", "drive" or "scan"
    payload: tuple[float, ...]


def drive_stadium(
    stadium: Stadium,
    laps: float,
    speed_mps: float = 1.0,
    seed: int = 1,
    clockwise: bool = False,
    odom_hz: float = 50.0,
    scan_hz: float = 10.0,
    start_s: float = 0.0,
    t0: float = 100.0,
) -> Iterator[Tick]:
    """Events (odom, drive, scan) of a car driving the centre line at a constant speed from
    arc length `start_s`, in time order. Scan payload is (x, y, yaw, scan_stamp); the caller
    casts the scan itself with `scan_at` so tests can tamper with it."""
    del seed
    total_t = laps * stadium.length_m / speed_mps
    n_odom = int(total_t * odom_hz)
    next_scan = 0.0
    for i in range(n_odom + 1):
        t = i / odom_hz
        s = start_s + speed_mps * t
        x, y, yaw, k = stadium.pose_at(s, clockwise)
        steer = math.atan(WHEELBASE_M * k)
        yield Tick(t0 + t, "drive", (steer,))
        yield Tick(t0 + t, "odom", (speed_mps,))
        if t >= next_scan - 1e-9:
            yield Tick(t0 + t, "scan", (x, y, yaw, t0 + t))
            next_scan += 1.0 / scan_hz


def feed(
    mem: object,
    stadium: Stadium,
    laps: float,
    speed_mps: float = 1.0,
    seed: int = 1,
    clockwise: bool = False,
    start_s: float = 0.0,
    on_tick: Callable[[object, Tick], None] | None = None,
) -> list[object]:
    """Drive `mem` (a LapMemory) round the stadium; returns its LapResults."""
    from racer_laps.scan_match import scan_to_points

    rng = np.random.default_rng(seed)
    walls = stadium.walls()
    out: list[object] = []
    for tick in drive_stadium(stadium, laps, speed_mps, seed, clockwise, start_s=start_s):
        if on_tick is not None:
            on_tick(mem, tick)
        if tick.kind == "drive":
            mem.on_drive(tick.payload[0])  # type: ignore[attr-defined]
        elif tick.kind == "odom":
            mem.on_wheel_odom(tick.t, tick.payload[0])  # type: ignore[attr-defined]
        else:
            x, y, yaw, stamp = tick.payload
            ranges, a0, da = scan_at(walls, x, y, yaw, rng)
            pts = scan_to_points(ranges, a0, da, 0.05, RANGE_MAX_M, MOUNT, BODY)
            r = mem.on_scan(stamp + 0.04, pts)  # type: ignore[attr-defined]
            if r is not None:
                out.append(r)
    return out
