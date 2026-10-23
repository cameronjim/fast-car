"""Pure metric logic of the reactive benchmark (tests/l5_reactive_bench), no ROS, numpy only.

The runner (reactive_bench_runner.py) records what happened on the topics; everything that
turns those records into the benchmark's numbers lives here so it is unit-tested without a
simulator (tools/tests/test_reactive_bench.py):

  * ``body_clearance_m``: gap from the car's body to the nearest wall face.
  * ``LapTracker``: laps and lap times from the car's position along a loop.
  * ``count_reversals``: zig-zag count of a steering signal with a hysteresis.
  * ``Recording`` and ``compute_metrics``: the run's record and the JSON-ready result.

Units are SI throughout (metres, seconds, radians).
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field

import numpy as np

SCHEMA_VERSION = 1

# A wall segment is a zero-width line in the track description; the simulator draws it
# WALL_THICKNESS_M thick (racer_gym_bridge.track_loader.WALL_THICKNESS_M), so the wall's face is
# half that from the line. Typed here again, with a test pinning it, so this module needs no
# bridge import.
WALL_THICKNESS_M = 0.08
# Clearances beyond this are not worth computing; the minimum is reported capped at it.
CLEARANCE_CAP_M = 1.5

# f1tenth_gym zeroes the car's speed AND heading when it detects a contact. Either the heading
# jumps by more than this in one 100 Hz step (a car turns at most about 0.012 rad per step) ...
CONTACT_YAW_JUMP_RAD = 0.3
# ... or the speed drops by more than this in one step (the gym's acceleration limit is about
# 9.5 m/s^2, 0.1 m/s per step), which catches a contact made with the heading already at zero.
CONTACT_SPEED_DROP_MPS = 0.3
# A position jump larger than this between two consecutive odometry samples is a /sim/reset (a
# car moves under 0.03 m per step at its 3 m/s limit).
TELEPORT_M = 0.5

# Steering swing that counts as a reversal of direction (rad): smaller wiggles are noise.
STEERING_REVERSAL_SWING_RAD = 0.1
# Reverse speed below which a /drive_raw request counts as "reverse" (m/s).
REVERSE_SPEED_MPS = -1e-3


@dataclass(frozen=True)
class BodyBox:
    """The car's bounding box in the frame of the simulator's pose (its centre of gravity):
    ``front_x_m`` ahead of it, ``rear_x_m`` behind it, ``half_width_m`` to each side."""

    front_x_m: float
    rear_x_m: float
    half_width_m: float

    @classmethod
    def from_vehicle_params(cls, params: dict) -> BodyBox:
        """From the parsed config/vehicle_params.yaml: the box about the rear axle (front
        bumper wheelbase + front overhang ahead of it, rear overhang behind it, the chassis
        width) re-expressed about the CG, which is cg_to_rear_axle ahead of the rear axle."""
        chassis = params["chassis"]
        lr = float(chassis["cg_to_rear_axle_m"])
        return cls(
            front_x_m=float(chassis["wheelbase_m"]) + float(chassis["front_overhang_m"]) - lr,
            rear_x_m=lr + float(chassis["rear_overhang_m"]),
            half_width_m=float(chassis["width_m"]) / 2.0,
        )


def _point_segment_distance(px, py, ax, ay, bx, by):
    """Distance from points (px, py) to segments a-b; every argument broadcasts."""
    dx, dy = bx - ax, by - ay
    len2 = dx * dx + dy * dy
    t = np.clip(((px - ax) * dx + (py - ay) * dy) / np.where(len2 > 0.0, len2, 1.0), 0.0, 1.0)
    return np.hypot(px - (ax + t * dx), py - (ay + t * dy))


def body_clearance_m(
    x: float,
    y: float,
    yaw: float,
    box: BodyBox,
    walls: np.ndarray,
    wall_thickness_m: float = WALL_THICKNESS_M,
    cap_m: float = CLEARANCE_CAP_M,
) -> float:
    """Gap (m) from the body at pose (x, y, yaw) to the nearest wall FACE.

    ``walls`` is an (N, 4) array of segments (x0, y0, x1, y1). The distance is exact between
    the body rectangle and the wall lines, less half the wall thickness; 0 or less means the
    body touches or overlaps a wall (down to -thickness / 2 for a body that reaches a wall's
    centre line). Capped at ``cap_m``.
    """
    reach = cap_m + math.hypot(max(box.front_x_m, box.rear_x_m), box.half_width_m)
    near = (
        (np.maximum(walls[:, 0], walls[:, 2]) >= x - reach)
        & (np.minimum(walls[:, 0], walls[:, 2]) <= x + reach)
        & (np.maximum(walls[:, 1], walls[:, 3]) >= y - reach)
        & (np.minimum(walls[:, 1], walls[:, 3]) <= y + reach)
    )
    if not near.any():
        return cap_m
    seg = walls[near]
    c, s = math.cos(yaw), math.sin(yaw)
    # Segment ends in the body frame (x forward from the pose, y left).
    ax, ay = c * (seg[:, 0] - x) + s * (seg[:, 1] - y), -s * (seg[:, 0] - x) + c * (seg[:, 1] - y)
    bx, by = c * (seg[:, 2] - x) + s * (seg[:, 3] - y), -s * (seg[:, 2] - x) + c * (seg[:, 3] - y)
    lo_x, hi_x, lo_y, hi_y = -box.rear_x_m, box.front_x_m, -box.half_width_m, box.half_width_m

    # Does the segment cross the rectangle? Liang-Barsky clipping against both slabs.
    t_in = np.zeros(len(seg))
    t_out = np.ones(len(seg))
    inside = np.ones(len(seg), dtype=bool)
    for start, delta, lo, hi in ((ax, bx - ax, lo_x, hi_x), (ay, by - ay, lo_y, hi_y)):
        parallel = np.abs(delta) < 1e-12
        inside &= ~parallel | ((start >= lo) & (start <= hi))
        with np.errstate(divide="ignore", invalid="ignore"):
            t_a, t_b = (lo - start) / delta, (hi - start) / delta
        t_in = np.maximum(t_in, np.where(parallel, 0.0, np.minimum(t_a, t_b)))
        t_out = np.minimum(t_out, np.where(parallel, 1.0, np.maximum(t_a, t_b)))
    crosses = inside & (t_in <= t_out)

    def point_to_rect(px, py):
        return np.hypot(
            np.maximum(np.maximum(lo_x - px, px - hi_x), 0.0),
            np.maximum(np.maximum(lo_y - py, py - hi_y), 0.0),
        )

    gap = np.minimum(point_to_rect(ax, ay), point_to_rect(bx, by))
    for corner_x, corner_y in ((lo_x, lo_y), (lo_x, hi_y), (hi_x, lo_y), (hi_x, hi_y)):
        gap = np.minimum(gap, _point_segment_distance(corner_x, corner_y, ax, ay, bx, by))
    gap = np.where(crosses, 0.0, gap)
    return float(min(gap.min() - wall_thickness_m / 2.0, cap_m))


class LapTracker:
    """Laps of a closed loop from the car's position, in the driving direction.

    Progress is the arc length of the nearest centreline point within a window of the previous
    one (a window, so a car passing close to another part of the loop is not teleported there),
    unwrapped. A lap is counted the first time progress reaches each multiple of the loop
    length: backing up (a reverse escape) loses progress without losing a lap already counted.
    """

    def __init__(self, xs: np.ndarray, ys: np.ndarray, window: int = 50) -> None:
        self._xs = np.asarray(xs, dtype=float)
        self._ys = np.asarray(ys, dtype=float)
        steps = np.hypot(
            np.diff(self._xs, append=self._xs[0]), np.diff(self._ys, append=self._ys[0])
        )
        self._ss = np.concatenate([[0.0], np.cumsum(steps)[:-1]])
        self.length_m = float(steps.sum())
        self._window = window
        self.reset()

    def reset(self) -> None:
        """Forget the run so far (the car was put back at the start)."""
        self.lap_times_s: list[float] = []
        self.progress_m = 0.0
        self.best_progress_m = 0.0
        self.lateral_m = 0.0
        self._unwrapped = None
        self._start_s = 0.0
        self._start_t = 0.0
        self._index = 0

    def _nearest(self, x: float, y: float, windowed: bool) -> tuple[int, float]:
        n = len(self._xs)
        indices = (
            (self._index + np.arange(-self._window, self._window + 1)) % n
            if windowed
            else np.arange(n)
        )
        d = np.hypot(self._xs[indices] - x, self._ys[indices] - y)
        k = int(np.argmin(d))
        return int(indices[k]), float(d[k])

    def update(self, x: float, y: float, t: float) -> None:
        if self._unwrapped is None:
            self._index, self.lateral_m = self._nearest(x, y, windowed=False)
            self._unwrapped = self._start_s = float(self._ss[self._index])
            self._start_t = t
            self._crossings: list[float] = []
            return
        self._index, self.lateral_m = self._nearest(x, y, windowed=True)
        delta = float(self._ss[self._index]) - (self._unwrapped % self.length_m)
        if delta < -self.length_m / 2.0:
            delta += self.length_m
        elif delta > self.length_m / 2.0:
            delta -= self.length_m
        self._unwrapped += delta
        self.progress_m = self._unwrapped - self._start_s
        self.best_progress_m = max(self.best_progress_m, self.progress_m)
        while len(self._crossings) < int(self.best_progress_m // self.length_m):
            self._crossings.append(t)
            previous = self._crossings[-2] if len(self._crossings) > 1 else self._start_t
            self.lap_times_s.append(t - previous)

    @property
    def laps(self) -> int:
        return len(self.lap_times_s)


def count_reversals(values: list[float] | np.ndarray, min_swing: float) -> int:
    """How many times the signal turns round: a peak or a trough is a reversal once the signal
    has moved ``min_swing`` away from it the other way (a hysteresis, so wiggles smaller than
    ``min_swing`` are not reversals). A monotonic signal has none; one that goes up, then
    down, then up has two."""
    if min_swing <= 0.0:
        raise ValueError("min_swing must be positive")
    reversals = 0
    direction = 0  # +1 rising, -1 falling, 0 not yet known
    high = low = None
    for v in values:
        if high is None:
            high = low = v
            continue
        high, low = max(high, v), min(low, v)
        if direction >= 0 and high - v >= min_swing:
            # Fell min_swing from the high: if it had been rising, that peak is a reversal.
            if direction == 1:
                reversals += 1
            direction = -1
            low = v
        elif direction <= 0 and v - low >= min_swing:
            if direction == -1:
                reversals += 1
            direction = 1
            high = v
    return reversals


@dataclass
class Recording:
    """What the runner saw on the topics, in the order it saw it. Times are ROS header stamps
    (s, the simulator's own clock domain: wall time, since every node here runs on wall time)."""

    # (t, x, y, yaw, v): ground truth pose of the car's CG and its body-frame speed.
    odom: list[tuple[float, float, float, float, float]] = field(default_factory=list)
    # (t, speed, steering_angle) of the follower's request and of the gated command.
    drive_raw: list[tuple[float, float, float]] = field(default_factory=list)
    drive: list[tuple[float, float, float]] = field(default_factory=list)
    # (t, source, phase, severity) of /safety/events.
    events: list[tuple[float, str, int, int]] = field(default_factory=list)
    # (t, x, y, reason): contacts the runner reacted to by putting the car back at the start.
    contacts: list[tuple[float, float, float, str]] = field(default_factory=list)

    def clear(self) -> None:
        for records in (self.odom, self.drive_raw, self.drive, self.events, self.contacts):
            records.clear()


def is_teleport(prev: tuple[float, float], cur: tuple[float, float]) -> bool:
    return math.hypot(cur[0] - prev[0], cur[1] - prev[1]) > TELEPORT_M


def gym_contact_signature(prev_yaw: float, yaw: float, prev_v: float, v: float) -> str | None:
    """The reason string if two consecutive odometry samples show f1tenth_gym's collision
    handler (speed and heading forced to zero), else None. See the CONTACT_* constants."""
    jump = math.atan2(math.sin(yaw - prev_yaw), math.cos(yaw - prev_yaw))
    if abs(jump) > CONTACT_YAW_JUMP_RAD:
        return "gym_yaw_jump"
    if prev_v - v > CONTACT_SPEED_DROP_MPS and abs(v) < 0.05:
        return "gym_speed_drop"
    return None


def compute_metrics(
    recording: Recording,
    *,
    box: BodyBox,
    walls: np.ndarray,
    lap_tracker: LapTracker | None,
    env_dt_s: float = 0.01,
    steering_swing_rad: float = STEERING_REVERSAL_SWING_RAD,
    clearance_stride: int = 2,
) -> dict:
    """The benchmark's result for one run (JSON-serialisable).

    ``lap_tracker`` is the tracker the runner fed during the run (None for a track that is not
    a loop). Clearance is evaluated on every ``clearance_stride``-th odometry sample (50 Hz
    with the default), plus the sample of every contact.
    """
    odom = recording.odom
    result: dict = {"schema_version": SCHEMA_VERSION}
    if len(odom) < 2:
        raise ValueError("no odometry recorded: the bridge never published")
    duration_s = odom[-1][0] - odom[0][0]
    sim_s = len(odom) * env_dt_s
    result["duration_s"] = round(duration_s, 3)
    result["sim_s"] = round(sim_s, 3)
    result["real_time_factor"] = round(sim_s / duration_s, 3) if duration_s > 0.0 else None

    # Laps.
    if lap_tracker is None:
        result["laps"] = None
        result["lap_time_s"] = None
        result["lap_times_s"] = []
        result["progress_m"] = None
    else:
        times = list(lap_tracker.lap_times_s)
        result["laps"] = len(times)
        result["lap_time_s"] = round(float(np.mean(times)), 3) if times else None
        result["lap_times_s"] = [round(t, 3) for t in times]
        result["progress_m"] = round(lap_tracker.best_progress_m, 3)
        result["loop_length_m"] = round(lap_tracker.length_m, 3)

    # Obstacle brakes (safety_node's TTC gates, forward or reverse) and reverse escapes.
    brakes = [
        e for e in recording.events if e[2] == 0 and e[3] == 2 and e[1] in ("ttc", "ttc_reverse")
    ]
    result["obstacle_brakes"] = len(brakes)
    reverse = [d[1] < REVERSE_SPEED_MPS for d in recording.drive_raw]
    result["escapes"] = sum(
        1 for a, b in zip([False, *reverse], reverse, strict=False) if b and not a
    )

    # Walls.
    result["wall_contacts"] = len(recording.contacts)
    result["contact_points"] = [
        {"t_s": round(t - odom[0][0], 3), "x_m": round(x, 3), "y_m": round(y, 3), "reason": why}
        for t, x, y, why in recording.contacts
    ]
    indices = set(range(0, len(odom), max(1, clearance_stride)))
    contact_times = {c[0] for c in recording.contacts}
    indices.update(i for i, o in enumerate(odom) if o[0] in contact_times)
    clearance = min(
        (body_clearance_m(odom[i][1], odom[i][2], odom[i][3], box, walls) for i in sorted(indices)),
        default=CLEARANCE_CAP_M,
    )
    result["min_body_clearance_m"] = round(clearance, 4)

    # Steering: the follower's request.
    steering = [d[2] for d in recording.drive_raw]
    minutes = duration_s / 60.0
    result["mean_abs_steering_rad"] = (
        round(float(np.mean(np.abs(steering))), 4) if steering else None
    )
    result["steering_reversals_per_min"] = (
        round(count_reversals(steering, steering_swing_rad) / minutes, 2)
        if steering and minutes > 0.0
        else None
    )

    # Motion.
    distance = 0.0
    for prev, cur in itertools.pairwise(odom):
        if not is_teleport(prev[1:3], cur[1:3]):
            distance += math.hypot(cur[1] - prev[1], cur[2] - prev[2])
    result["distance_m"] = round(distance, 3)
    result["mean_speed_mps"] = round(float(np.mean([abs(o[4]) for o in odom])), 3)
    laps = result["laps"]
    result["brakes_per_lap"] = round(len(brakes) / laps, 3) if laps else None
    return result
