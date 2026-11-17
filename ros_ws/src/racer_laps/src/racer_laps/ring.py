"""The keyframe ring of lap gate v3. Numpy only, no ROS. Design: claude-docs/13-plug-and-play.md
section 20, brief 50.

A keyframe is a scan taught on lap 1 at a known station (wheel distance since the start). On later
laps the gate matches each scan to the one keyframe at the car's station; a keyframe whose matches
are unique gives a fix (the wheel total at which the car was at it), and the latest fix of a lap
places the line by wheel distance (`anchor_total_m`). A keyframe that looks like its neighbours
(aliased: counted over its life, latched) gives no fix and is retired. A fix needs a proven
keyframe (enough unique matches). This module holds the bookkeeping only; the matching is the
gate's.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from racer_laps.scan_match import FloatArray, MatchResult, polar_profile, surface_normals


@dataclass(frozen=True)
class Ref:
    """A reference scan: what a match is made against (the start scan or a keyframe)."""

    pts: FloatArray
    normals: FloatArray
    dense: FloatArray
    profile: FloatArray


def make_ref(
    points: FloatArray,
    thin_pts: FloatArray,
    dense: FloatArray,
    lidar_xy: tuple[float, float],
    compact: bool = False,
) -> Ref:
    """Reference from a scan: its thin and dense point sets (already made by the caller), the
    surface normals of the thin set and the polar profile of the raw points. `compact` stores
    float32 (keyframes: 120 of them stay under 2 MB; the start scan keeps float64)."""
    ref = Ref(thin_pts, surface_normals(thin_pts), dense, polar_profile(points, lidar_xy))
    if not compact:
        return ref
    return Ref(
        ref.pts.astype(np.float32),
        ref.normals.astype(np.float32),
        ref.dense.astype(np.float32),
        ref.profile.astype(np.float32),
    )


def is_aliased(n_unique: int, n_ambiguous: int, min_scans: int, max_share: float) -> bool:
    """A reference is aliased on an approach when at least `min_scans` of its STRONG matches are
    ambiguous and their share of all STRONG matches is above `max_share`."""
    total = n_unique + n_ambiguous
    if total <= 0 or n_ambiguous < min_scans:
        return False
    return n_ambiguous / total > max_share


@dataclass(frozen=True)
class Estimate:
    """One STRONG, unique match: an estimate of the wheel total at the reference."""

    line_m: float
    inlier_share: float
    rms_m: float


@dataclass
class Keyframe:
    index: int
    station_m: float
    heading_rad: float
    total_m: float  # wheel total when taught
    ref: Ref
    retired: bool = False  # aliased: retired for good (until a re-teach)
    #: Unique and ambiguous STRONG matches over the life of this keyframe (not per visit): its
    #: verdict belongs to the place.
    n_unique: int = 0
    n_ambiguous: int = 0
    visited: bool = False  # matched on the current lap (the tail of a lap is tracked by the
    # start scan and never visited: it must not keep a useless ring alive)

    @property
    def nbytes(self) -> int:
        r = self.ref
        return int(r.pts.nbytes + r.normals.nbytes + r.dense.nbytes + r.profile.nbytes)


@dataclass(frozen=True)
class Fix:
    keyframe_index: int
    station_m: float
    total_m: float  # wheel total at which the car was at the keyframe


@dataclass
class Visit:
    """The scans matched to one keyframe in a row."""

    keyframe: Keyframe
    estimates: list[Estimate] = field(default_factory=list)
    #: previous accepted match: (time, result, dead-reckoned pose at that scan)
    last: tuple[float, MatchResult, tuple[float, float, float]] | None = None


class Ring:
    def __init__(self, spacing_m: float, kf_max: int, window_m: float) -> None:
        if spacing_m <= 0.0 or kf_max < 2 or window_m <= 0.0:
            raise ValueError("ring: spacing and window positive, kf_max >= 2")
        self._base_spacing_m = spacing_m
        self.kf_max = kf_max
        self.window_m = window_m
        self.spacing_m = spacing_m
        self.keyframes: list[Keyframe] = []
        self.length_m = math.nan  # L_ring: lap 1's wheel length (the teaching lap's)
        self.fixed = False  # taught: a closure ended the teaching lap
        self.generation = 0
        self.laps_without_fix = 0
        self.fixes: list[Fix] = []  # of the current lap
        self.fixes_last_lap = 0
        self._next_index = 0

    # -- teaching ---------------------------------------------------------------------------

    def reset(self) -> None:
        """`set_start`: forget the ring."""
        self.spacing_m = self._base_spacing_m
        self.keyframes = []
        self.length_m = math.nan
        self.fixed = False
        self.generation = 0
        self.laps_without_fix = 0
        self.fixes = []
        self.fixes_last_lap = 0
        self._next_index = 0

    @property
    def teaching(self) -> bool:
        return not self.fixed

    def due(self, station_m: float) -> bool:
        """A keyframe is due: `spacing_m` of wheel distance since the last one."""
        if not self.keyframes:
            return True
        return station_m - self.keyframes[-1].station_m >= self.spacing_m

    def add(self, station_m: float, heading_rad: float, total_m: float, ref: Ref) -> Keyframe:
        """Add a keyframe. At `kf_max` every second one is dropped first (keyframe 0 stays) and the
        spacing doubles."""
        if len(self.keyframes) >= self.kf_max:
            self.keyframes = self.keyframes[::2]
            self.spacing_m *= 2.0
        kf = Keyframe(self._next_index, station_m, heading_rad, total_m, ref)
        self._next_index += 1
        self.keyframes.append(kf)
        return kf

    def finish_teaching(self, length_m: float) -> None:
        """The teaching lap closed after `length_m` of wheel distance: drop keyframes taught in
        the last half spacing (they look like keyframe 0) and fix the ring."""
        self.length_m = length_m
        cut = length_m - 0.5 * self.spacing_m
        self.keyframes = [k for k in self.keyframes if k.station_m <= cut or k.index == 0]
        self.fixed = True
        self.laps_without_fix = 0

    # -- repeating --------------------------------------------------------------------------

    def nearest_live(self, station_m: float) -> Keyframe | None:
        """The live keyframe whose station is nearest `station_m`, if within the window."""
        best: Keyframe | None = None
        best_d = math.inf
        for k in self.keyframes:
            if k.retired:
                continue
            d = abs(k.station_m - station_m)
            if d < best_d:
                best, best_d = k, d
        if best is None or best_d > self.window_m:
            return None
        return best

    def note_visit(self, kf: Keyframe) -> None:
        """A visit ended: the keyframe was matched on this lap."""
        kf.visited = True

    def add_fix(self, fix: Fix) -> None:
        self.fixes.append(fix)

    @property
    def anchor_total_m(self) -> float:
        """Where the line is by wheel distance, from the latest fix of this lap (NaN: no fix)."""
        if not self.fixes or not math.isfinite(self.length_m):
            return math.nan
        f = self.fixes[-1]
        return f.total_m + (self.length_m - f.station_m)

    def end_lap(self) -> None:
        """A lap closed (not the teaching lap): book its fixes and decide whether the next lap
        teaches again (live keyframes but no fix on two laps running)."""
        self.fixes_last_lap = len(self.fixes)
        self.laps_without_fix = 0 if self.fixes else self.laps_without_fix + 1
        self.fixes = []
        live_other = any(k.visited and not k.retired and k.index != 0 for k in self.keyframes)
        for k in self.keyframes:
            k.visited = False
        if self.laps_without_fix >= 2 and live_other:
            self.keyframes = []
            self.fixed = False
            self.spacing_m = self._base_spacing_m
            self.generation += 1
            self.laps_without_fix = 0

    # -- status -----------------------------------------------------------------------------

    @property
    def n_live(self) -> int:
        return sum(1 for k in self.keyframes if not k.retired)

    @property
    def n_retired(self) -> int:
        return sum(1 for k in self.keyframes if k.retired)

    @property
    def nbytes(self) -> int:
        return sum(k.nbytes for k in self.keyframes)

    def status(self) -> dict[str, object]:
        return {
            "keyframes": len(self.keyframes),
            "live": self.n_live,
            "retired": self.n_retired,
            "fixes_last_lap": self.fixes_last_lap,
            "generation": self.generation,
        }
