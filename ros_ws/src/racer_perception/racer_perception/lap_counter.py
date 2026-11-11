"""Camera lap counter core: pure numpy, no ROS, no OpenCV.

The idea (the owner's): remember what the camera saw when the run started, then call it a lap
every time the camera sees that again. "Sees that again" is the cosine similarity of two small
grayscale thumbnails after removing each one's mean (that is the normalised cross-correlation
at zero shift, a dot product of two unit vectors). No markers, no map, no pose.

    ref = unit_signature(thumbnail of the first frame)
    sim = dot(unit_signature(thumbnail of this frame), ref)         # -1 .. 1

Why mean removal and unit norm: a brighter or darker room, auto exposure and a different gain
change a frame by `a * pixel + b`, and the similarity of `a * x + b` with `x` is exactly 1 for
a > 0. What is left is the structure of the scene, which is what repeats every lap.

Lap decision (every number is a LapConfig field, so a bag can be replayed to tune them):

  1. WAITING. No reference yet. The first frame after the car's speed reaches `start_speed_mps`
     (or the first frame after reset()) becomes the reference; lap 0 starts there.
  2. ARMED. After the start, and after every counted lap, the counter is disarmed until the
     similarity has dropped below `rearm_threshold`: the car has left the start line's view.
     Standing at the start, or creeping past it, can therefore never count twice.
  3. WINDOW. While armed, a frame with similarity >= `similarity_threshold` opens a window, but
     only if `min_lap_time_s` and `min_lap_distance_m` (odometry path length) have passed since
     the lap started. A window that opens before the gates pass is thrown away, whole.
  4. PEAK. The lap is recorded when the window closes (similarity back under the threshold, or
     `peak_window_max_s` elapsed), stamped at the frame with the highest similarity inside it,
     so the lap time does not depend on where in the bright part the threshold happens to sit.

Heading turns (yaw rate integrated from odometry) are reported with each lap as a cross-check
only: a lap of an oval is about one turn. They never count or veto a lap.

Time is whatever clock the caller uses, in seconds, increasing (the node uses its ROS clock, the
replay tool the bag's log time). SI units throughout: seconds, metres, radians. No vehicle
constants live here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields
from typing import Any

import numpy as np

STATE_WAITING = "waiting"
STATE_RUNNING = "running"

_TWO_PI = 2.0 * math.pi
#: An odometry sample after a gap longer than this integrates over this much, not the gap.
_MAX_ODOM_DT_S = 0.5


@dataclass(frozen=True)
class LapConfig:
    """Tunable parameters. Defaults are for the 5 m indoor loop (docs/notes/track-session-*)."""

    #: Count a lap at a similarity peak at or above this.
    similarity_threshold: float = 0.80
    #: After a lap (or at the start) the similarity must first fall below this.
    rearm_threshold: float = 0.55
    #: A lap cannot be shorter than this many seconds.
    min_lap_time_s: float = 6.0
    #: A lap cannot be shorter than this many metres of odometry path length.
    min_lap_distance_m: float = 3.0
    #: The run starts (reference captured) once the speed is at least this. <= 0: first frame.
    start_speed_mps: float = 0.3
    #: Thumbnail size the similarity is computed on.
    thumb_width_px: int = 64
    thumb_height_px: int = 48
    #: 3x3 binomial blur of the thumbnail before comparing (takes the edge off pixel noise).
    blur: bool = True
    #: A window still open after this long is closed at its peak (car parked at the line).
    peak_window_max_s: float = 2.0
    #: A reference whose pixel standard deviation (gray levels, 0..255) is below this is a blank
    #: frame (lens cap, camera starting up): refused, the next frame is tried.
    min_reference_std: float = 2.0

    def validate(self) -> None:
        for f in fields(self):
            value = getattr(self, f.name)
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError(f"{f.name} must be finite, got {value}")
        if not 0.0 < self.rearm_threshold < self.similarity_threshold <= 1.0:
            raise ValueError(
                "need 0 < rearm_threshold < similarity_threshold <= 1, got "
                f"rearm {self.rearm_threshold}, similarity {self.similarity_threshold}"
            )
        if self.min_lap_time_s < 0.0 or self.min_lap_distance_m < 0.0:
            raise ValueError("min_lap_time_s and min_lap_distance_m must be >= 0")
        if self.peak_window_max_s <= 0.0:
            raise ValueError("peak_window_max_s must be > 0")
        if self.thumb_width_px < 4 or self.thumb_height_px < 4:
            raise ValueError("the thumbnail must be at least 4 x 4 pixels")
        if self.min_reference_std < 0.0:
            raise ValueError("min_reference_std must be >= 0")

    @classmethod
    def field_names(cls) -> list[str]:
        return [f.name for f in fields(cls)]


@dataclass(frozen=True)
class LapEvent:
    lap: int
    #: Seconds between the previous crossing (or the reference capture for lap 1) and this one.
    lap_time_s: float
    best_lap_s: float
    peak_similarity: float
    #: Clock time of the frame with the peak similarity.
    crossing_time_s: float
    #: Odometry path length of this lap, metres.
    distance_m: float
    #: Signed heading turns of this lap from integrated yaw rate (1.0 = one full left turn);
    #: None when no yaw rate has been received.
    heading_turns: float | None

    def to_dict(self) -> dict[str, Any]:
        return {
            "lap": self.lap,
            "lap_time_s": self.lap_time_s,
            "best_lap_s": self.best_lap_s,
            "peak_similarity": self.peak_similarity,
            "crossing_time_s": self.crossing_time_s,
            "distance_m": self.distance_m,
            "heading_turns": self.heading_turns,
        }


# ------------------------------------------------------------------------------ image maths


def resize_area(gray: np.ndarray, width: int, height: int) -> np.ndarray:
    """Shrink a 2-D array to (height, width) by averaging the source pixels in each cell.

    Cells are the integer-edged bins `linspace(0, size, n + 1)`, so any source size works.
    A source smaller than the target in either direction is sampled nearest-neighbour there
    instead (nothing to average). float32 out.
    """
    if gray.ndim != 2:
        raise ValueError(f"expected a 2-D grayscale image, got shape {gray.shape}")
    src_h, src_w = gray.shape
    out = gray.astype(np.float32, copy=False)
    out = _shrink_axis(out, height, axis=0, size=src_h)
    out = _shrink_axis(out, width, axis=1, size=src_w)
    return out


def _shrink_axis(a: np.ndarray, n: int, axis: int, size: int) -> np.ndarray:
    if size < n:
        index = np.minimum((np.arange(n) * size) // n, size - 1)
        return np.take(a, index, axis=axis)
    if size == n:
        return a
    edges = np.linspace(0, size, n + 1).astype(np.int64)
    sums = np.add.reduceat(a, edges[:-1], axis=axis, dtype=np.float32)
    counts = np.diff(edges).astype(np.float32)
    shape = [1, 1]
    shape[axis] = n
    return sums / counts.reshape(shape)


def blur3(a: np.ndarray) -> np.ndarray:
    """Separable [1 2 1] / 4 blur with replicated edges."""
    p = np.pad(a, ((1, 1), (0, 0)), mode="edge")
    a = (p[:-2] + 2.0 * p[1:-1] + p[2:]) * 0.25
    p = np.pad(a, ((0, 0), (1, 1)), mode="edge")
    return (p[:, :-2] + 2.0 * p[:, 1:-1] + p[:, 2:]) * 0.25


def unit_signature(thumb: np.ndarray, min_std: float = 0.0) -> np.ndarray | None:
    """Zero-mean, unit-norm flat vector of a thumbnail, or None for a blank (flat) image."""
    v = thumb.astype(np.float32, copy=False).ravel()
    v = v - v.mean()
    norm = float(np.linalg.norm(v))
    std = norm / math.sqrt(v.size)
    if not math.isfinite(norm) or std <= max(min_std, 1e-6):
        return None
    return v / norm


def make_signature(gray: np.ndarray, cfg: LapConfig, min_std: float = 0.0) -> np.ndarray | None:
    """Gray image of any size -> the vector the similarity is a dot product of."""
    thumb = resize_area(gray, cfg.thumb_width_px, cfg.thumb_height_px)
    if cfg.blur:
        thumb = blur3(thumb)
    return unit_signature(thumb, min_std)


# ---------------------------------------------------------------------------------- counter


class LapCounter:
    def __init__(self, cfg: LapConfig | None = None) -> None:
        self.cfg = cfg or LapConfig()
        self.cfg.validate()
        # Odometry survives reset(): the speed and the path length keep running.
        self.distance_m = 0.0
        self.speed_mps = 0.0
        self.yaw_rad = 0.0
        self.odom_seen = False
        self.yaw_seen = False
        self._last_x: float | None = None
        self._last_odom_t: float | None = None
        self._last_yaw_t: float | None = None
        self.reset(capture_next_frame=False)

    # ------------------------------------------------------------------ lifecycle

    def reset(self, capture_next_frame: bool = True) -> None:
        """Forget the reference and every count. The next frame becomes the new reference when
        `capture_next_frame` (the car is at the start line now), else the first frame after the
        car moves does."""
        self.state = STATE_WAITING
        self._capture_now = capture_next_frame
        self._ref: np.ndarray | None = None
        self.laps = 0
        self.last_lap_s: float | None = None
        self.best_lap_s: float | None = None
        self.similarity: float | None = None
        self._armed = False
        self._suppressed = False
        self._window: dict[str, float] | None = None
        self._lap_start_t = 0.0
        self._lap_start_dist = 0.0
        self._lap_start_yaw = 0.0
        self.start_time_s: float | None = None

    # ------------------------------------------------------------------ odometry

    def update_odom(self, t: float, speed_mps: float, x_m: float | None = None) -> None:
        """Wheel odometry: speed (m/s) and, if the source has it, the along-track coordinate
        (m). Path length grows by |delta x| (so reversing counts as distance too); without x it
        integrates |speed| over the time between samples. Non-finite inputs are ignored."""
        if not math.isfinite(speed_mps) or not math.isfinite(t):
            return
        self.odom_seen = True
        self.speed_mps = speed_mps
        if x_m is not None and math.isfinite(x_m):
            if self._last_x is not None:
                self.distance_m += abs(x_m - self._last_x)
            self._last_x = x_m
        elif self._last_odom_t is not None:
            self.distance_m += abs(speed_mps) * min(max(t - self._last_odom_t, 0.0), _MAX_ODOM_DT_S)
        self._last_odom_t = t

    def update_yaw_rate(self, t: float, yaw_rate_rps: float) -> None:
        """Yaw rate (rad/s, left positive) for the heading-turn diagnostic only."""
        if not math.isfinite(yaw_rate_rps) or not math.isfinite(t):
            return
        self.yaw_seen = True
        if self._last_yaw_t is not None:
            dt = min(max(t - self._last_yaw_t, 0.0), _MAX_ODOM_DT_S)
            self.yaw_rad += yaw_rate_rps * dt
        self._last_yaw_t = t

    # ------------------------------------------------------------------ frames

    def update_frame(self, t: float, gray: np.ndarray) -> LapEvent | None:
        """One camera frame (2-D uint8/float gray image of any size). Returns the lap event if
        this frame closed a lap window, else None."""
        if not math.isfinite(t):
            return None
        sig = make_signature(
            gray, self.cfg, self.cfg.min_reference_std if self._ref is None else 0.0
        )
        return self.update_signature(t, sig)

    def update_signature(self, t: float, sig: np.ndarray | None) -> LapEvent | None:
        """Like update_frame for a signature already made with make_signature."""
        cfg = self.cfg
        if self.state == STATE_WAITING:
            if not (
                self._capture_now
                or cfg.start_speed_mps <= 0.0
                or self.speed_mps >= cfg.start_speed_mps
            ):
                return None
            if sig is None:  # a blank frame cannot be a reference
                return None
            self._ref = sig
            self.state = STATE_RUNNING
            self.start_time_s = t
            self._lap_start_t = t
            self._lap_start_dist = self.distance_m
            self._lap_start_yaw = self.yaw_rad
            self._armed = False
            self.similarity = 1.0
            return None

        assert self._ref is not None
        s = 0.0 if sig is None else float(np.dot(sig, self._ref))
        self.similarity = s

        if self._window is not None:
            w = self._window
            if s > w["peak"]:
                w.update(peak=s, t=t, dist=self.distance_m, yaw=self.yaw_rad)
            if s < cfg.similarity_threshold or t - w["open_t"] >= cfg.peak_window_max_s:
                return self._close_window()
            return None

        if s < cfg.similarity_threshold:
            self._suppressed = False
            if not self._armed and s < cfg.rearm_threshold:
                self._armed = True
            return None
        if not self._armed or self._suppressed:
            return None
        gates_ok = (t - self._lap_start_t >= cfg.min_lap_time_s) and (
            self.distance_m - self._lap_start_dist >= cfg.min_lap_distance_m
        )
        if not gates_ok:
            self._suppressed = True  # the whole bright stretch is ignored, even if a gate opens
            return None
        self._window = {
            "open_t": t,
            "peak": s,
            "t": t,
            "dist": self.distance_m,
            "yaw": self.yaw_rad,
        }
        return None

    def _close_window(self) -> LapEvent:
        w = self._window
        assert w is not None
        self._window = None
        self._armed = False
        self._suppressed = False
        self.laps += 1
        lap_time = w["t"] - self._lap_start_t
        self.last_lap_s = lap_time
        self.best_lap_s = lap_time if self.best_lap_s is None else min(self.best_lap_s, lap_time)
        event = LapEvent(
            lap=self.laps,
            lap_time_s=lap_time,
            best_lap_s=self.best_lap_s,
            peak_similarity=w["peak"],
            crossing_time_s=w["t"],
            distance_m=w["dist"] - self._lap_start_dist,
            heading_turns=(w["yaw"] - self._lap_start_yaw) / _TWO_PI if self.yaw_seen else None,
        )
        self._lap_start_t = w["t"]
        self._lap_start_dist = w["dist"]
        self._lap_start_yaw = w["yaw"]
        return event

    # ------------------------------------------------------------------ reporting

    def status(self) -> dict[str, Any]:
        """JSON-able snapshot for /lap/status."""
        return {
            "state": self.state,
            "similarity": self.similarity,
            "laps": self.laps,
            "last_lap_s": self.last_lap_s,
            "best_lap_s": self.best_lap_s,
            "armed": self._armed,
            "speed_mps": self.speed_mps,
            "distance_m": self.distance_m - self._lap_start_dist
            if self.state == STATE_RUNNING
            else 0.0,
            "heading_turns": (self.yaw_rad - self._lap_start_yaw) / _TWO_PI
            if self.yaw_seen and self.state == STATE_RUNNING
            else None,
            "odom_seen": self.odom_seen,
        }
