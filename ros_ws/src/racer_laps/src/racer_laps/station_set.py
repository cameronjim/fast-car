"""The station set (D v1): what the car has learned about its own laps, indexed by station.
Numpy only, no ROS. `safe_set.py` keeps the map-pose version.

Station `s` is the wheel distance since the last lap closure (see `gate.py`). Every clean lap is
resampled on a station grid of 0.10 m (rounded so the grid closes on the reference lap length
`L_ref`), after rescaling its stations by `L_ref / lap length`, giving the wheel speed `v(s)` and
the time to finish `ctg(s)` (the lap time at s = 0, 0 at the end). Across the stored laps:

* `v_safe(s)` is the largest of the laps' `v(s)` after a MIN filter over plus or minus
  `min_filter_half_m` (0.4 m, periodic): station drift between laps must never make the profile
  optimistic, so a speed counts only where the lap was at least that fast over the whole window.
* `Q(s)` is the smallest `ctg(s)`. It is non-increasing. Past the lap end it continues as
  `Q(L_ref + x) = Q(x) - T_best` (T_best, the best stored lap time, equals `Q(0)`), so it is
  continuous and still non-increasing, and negative after the first lap end.
* Caution zones: the `caution_before_m` (1.0 m) of station before each of an obstacle-gate brake,
  a BLOCKED or RECOVERING planner state, or a plan clearance under the clean threshold, on ANY
  lap, stored or not. In a zone the bound is `min(v_safe, speed driven there - 0.1)` (never
  below 0) with no exploration, for the next `caution_laps` (2) laps; then the zone expires.
* The bound handed to the planner is `v_bound(s) = v_safe(s) + explore_dv_mps` outside zones.
  Exploration therefore adds at most `explore_dv_mps` over the best speed already driven.

Honest limits. The set is (station, speed), not the full state: lateral position and heading are
constrained by the live scan in the planner, not by memory. The improvement proof of Learning MPC
(Rosolia and Borrelli) assumes a perfect model and a full-state safe set; neither holds here. That
is why the planner treats the bound as a soft trust region, why zones exist, and why the sim
battery decides whether D v1 goes on the car. Nothing here is a guarantee.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]

SPACING_M = 0.10
#: A lap is stored only if its samples span at least this share of the reference length.
MIN_COVERAGE = 0.8
_FORMAT_VERSION = 1


@dataclass(frozen=True)
class StationLap:
    lap_index: int
    lap_time_s: float
    length_m: float  # wheel distance of the lap before rescaling
    cap_mps: float
    v_mps: FloatArray  # wheel speed at the grid stations
    ctg_s: FloatArray  # time to finish at the grid stations (grid_n + 1 values, last is 0)


@dataclass(frozen=True)
class CautionZone:
    centre_m: float  # station of the event on the reference lap grid
    length_m: float
    speed_mps: float  # speed driven in the zone on the offending lap
    created_lap: int  # lap in progress when it was created
    hold_laps: int  # it applies to laps created_lap + 1 .. created_lap + hold_laps
    reason: str

    def active_on(self, lap_in_progress: int) -> bool:
        return self.created_lap < lap_in_progress <= self.created_lap + self.hold_laps


def _periodic_min_filter(v: FloatArray, half: int) -> FloatArray:
    if half <= 0 or v.shape[0] == 0:
        return v.copy()
    out = v.copy()
    for k in range(1, half + 1):
        out = np.minimum(out, np.minimum(np.roll(v, k), np.roll(v, -k)))
    return out


def resample_station_lap(
    station_m: FloatArray,
    t_s: FloatArray,
    speed_mps: FloatArray,
    lap_time_s: float,
    length_m: float,
    l_ref_m: float,
    n_grid: int,
) -> tuple[FloatArray, FloatArray] | None:
    """(v at the n_grid stations, ctg at n_grid + 1 stations) of one lap, or None when the lap has
    too little motion. `station_m`, `t_s` and `speed_mps` are the lap's samples: wheel distance
    and time since the lap start, and the wheel speed. The lap spans station [0, length_m] and
    time [0, lap_time_s]; the ends are anchored there."""
    if (
        station_m.shape[0] < 2
        or length_m <= 0.0
        or lap_time_s <= 0.0
        or l_ref_m <= 0.0
        or n_grid < 2
    ):
        return None
    scale = l_ref_m / length_m
    s = np.clip(station_m * scale, 0.0, l_ref_m)
    if float(s.max() - s.min()) < MIN_COVERAGE * l_ref_m:
        return None  # the samples do not cover the lap: a stall, or a lap of the wrong length
    t_rel = np.clip(t_s, 0.0, lap_time_s)
    # Anchor the ends: the lap starts at station 0, time 0 and ends at station L_ref, time T.
    s_k = np.concatenate([[0.0], s, [l_ref_m]])
    t_k = np.concatenate([[0.0], t_rel, [lap_time_s]])
    v_k = np.concatenate([[speed_mps[0]], speed_mps, [speed_mps[-1]]])
    s_k = np.maximum.accumulate(s_k)
    t_k = np.maximum.accumulate(t_k)
    keep = np.flatnonzero(np.concatenate([[True], np.diff(s_k) > 1e-9]))
    if keep.shape[0] < 2:
        return None
    s_k, t_k, v_k = s_k[keep], t_k[keep], v_k[keep]
    if not np.all(np.isfinite(v_k)):
        return None
    # A lap that starts from rest has speed 0 at its first stations, but the car crosses the line at
    # the speed it ends the lap with on every later lap: do not let the standing start become the
    # profile there (it would bound the start line at the dead zone for good).
    v_end = float(np.mean(speed_mps[-5:]))
    if v_end > 0.0 and float(speed_mps[0]) < 0.5 * v_end:
        reached = np.flatnonzero(speed_mps >= v_end)
        first = int(reached[0]) if reached.size else speed_mps.shape[0]
        speed_k = speed_mps.copy()
        speed_k[:first] = v_end
        v_k = np.concatenate([[v_end], speed_k, [speed_k[-1]]])[keep]
    h = l_ref_m / n_grid
    grid = np.arange(n_grid + 1) * h
    t_at = np.interp(grid, s_k, t_k)
    v_at = np.interp(grid[:n_grid], s_k, v_k)
    ctg = lap_time_s - t_at
    ctg[0] = lap_time_s
    ctg[-1] = 0.0
    return v_at, np.maximum.accumulate(ctg[::-1])[::-1]


class StationSet:
    def __init__(
        self,
        spacing_m: float = SPACING_M,
        min_filter_half_m: float = 0.4,
        caution_before_m: float = 1.0,
        caution_laps: int = 2,
        floor_mps: float = 0.0,
        zone_merge: bool = True,
    ) -> None:
        if spacing_m <= 0.0 or min_filter_half_m < 0.0 or caution_before_m <= 0.0:
            raise ValueError("station set spacing and zone lengths must be positive")
        if caution_laps < 0:
            raise ValueError("caution_laps must not be negative")
        self.spacing_m = float(spacing_m)
        self.min_filter_half_m = float(min_filter_half_m)
        self.caution_before_m = float(caution_before_m)
        self.caution_laps = int(caution_laps)
        #: No bound is below this: the planner cannot drive below its speed dead zone, so a bound
        #: under it is a stop, not a slow speed.
        self.floor_mps = float(floor_mps)
        #: False: `add_zone` always appends (no merging of a flickering state into one zone).
        self.zone_merge = bool(zone_merge)
        self._laps: list[StationLap] = []
        self._zones: list[CautionZone] = []
        self._l_ref = math.nan
        self._n = 0
        self._v_safe: FloatArray = np.zeros(0)
        self._q: FloatArray = np.zeros(0)  # n + 1 values
        self._t_best = math.nan

    # -- laps ------------------------------------------------------------------------------

    @property
    def n_laps(self) -> int:
        return len(self._laps)

    @property
    def laps(self) -> tuple[StationLap, ...]:
        return tuple(self._laps)

    @property
    def l_ref_m(self) -> float:
        return self._l_ref

    @property
    def best_lap_time_s(self) -> float:
        return self._t_best

    @property
    def zones(self) -> tuple[CautionZone, ...]:
        return tuple(self._zones)

    def set_reference_length(self, l_ref_m: float) -> None:
        """Fix the reference lap length (the first closure). Once laps are stored it is final."""
        if not (math.isfinite(l_ref_m) and l_ref_m > self.spacing_m):
            raise ValueError(f"reference lap length must exceed the grid spacing, got {l_ref_m}")
        if self._laps:
            return
        self._l_ref = float(l_ref_m)
        self._n = max(2, round(l_ref_m / self.spacing_m))

    def add_lap(
        self,
        lap_index: int,
        station_m: FloatArray,
        t_s: FloatArray,
        speed_mps: FloatArray,
        lap_time_s: float,
        length_m: float,
        cap_mps: float,
    ) -> bool:
        """Store a CLEAN lap (the caller decides that). False if it has too little motion."""
        if not math.isfinite(self._l_ref):
            return False
        res = resample_station_lap(
            station_m, t_s, speed_mps, lap_time_s, length_m, self._l_ref, self._n
        )
        if res is None:
            return False
        v, ctg = res
        self._laps.append(StationLap(lap_index, lap_time_s, length_m, cap_mps, v, ctg))
        self._rebuild()
        return True

    def remove_lap(self, lap_index: int) -> bool:
        """Drop a stored lap (it turned out to be a suspect closure). True if it was stored."""
        kept = [lap for lap in self._laps if lap.lap_index != lap_index]
        if len(kept) == len(self._laps):
            return False
        self._laps = kept
        if self._laps:
            self._rebuild()
        else:
            self._v_safe = np.zeros(0)
            self._q = np.zeros(0)
            self._t_best = math.nan
        return True

    def rescale(self, l_ref_m: float) -> None:
        """Change the reference lap length. Stored laps are normalised to their own length, so
        they keep their shape: resample them on the grid of the new length (periodic in speed).
        Zone centres scale with the length."""
        if not (math.isfinite(l_ref_m) and l_ref_m > self.spacing_m):
            raise ValueError(f"reference lap length must exceed the grid spacing, got {l_ref_m}")
        if not math.isfinite(self._l_ref) or not self._laps:
            self._laps = []
            self._l_ref = float(l_ref_m)
            self._n = max(2, round(l_ref_m / self.spacing_m))
            return
        ratio = l_ref_m / self._l_ref
        n_old, n_new = self._n, max(2, round(l_ref_m / self.spacing_m))
        u_old = np.arange(n_old + 1) / n_old
        u_new = np.arange(n_new + 1) / n_new
        laps = []
        for lap in self._laps:
            v_ext = np.concatenate([lap.v_mps, lap.v_mps[:1]])
            v = np.interp(u_new[:n_new], u_old, v_ext)
            ctg = np.interp(u_new, u_old, lap.ctg_s)
            laps.append(
                StationLap(lap.lap_index, lap.lap_time_s, lap.length_m, lap.cap_mps, v, ctg)
            )
        self._laps = laps
        self._zones = [
            CautionZone(
                z.centre_m * ratio, z.length_m, z.speed_mps, z.created_lap, z.hold_laps, z.reason
            )
            for z in self._zones
        ]
        self._l_ref = float(l_ref_m)
        self._n = n_new
        self._rebuild()

    def _rebuild(self) -> None:
        half = round(self.min_filter_half_m / self.spacing_m)
        v = np.stack([_periodic_min_filter(lap.v_mps, half) for lap in self._laps])
        self._v_safe = v.max(axis=0)
        self._q = np.stack([lap.ctg_s for lap in self._laps]).min(axis=0)
        self._t_best = float(min(lap.lap_time_s for lap in self._laps))
        # Q(0) = T_best by construction; make the lap end exactly Q(0) - T_best.
        self._q[-1] = self._q[0] - self._t_best

    # -- zones -----------------------------------------------------------------------------

    def add_zone(
        self, centre_m: float, speed_mps: float, created_lap: int, reason: str
    ) -> CautionZone | None:
        """A caution zone ending at `centre_m` (station of the event). Needs the reference length
        (a zone before the first closure has no grid to live on and is not stored)."""
        if not (
            math.isfinite(self._l_ref) and math.isfinite(centre_m) and math.isfinite(speed_mps)
        ):
            return None
        centre = float(centre_m) % self._l_ref
        for k, old in enumerate(self._zones if self.zone_merge else ()):
            gap = abs(old.centre_m - centre)
            gap = min(gap, self._l_ref - gap)
            if old.created_lap == int(created_lap) and gap <= 0.5 * self.caution_before_m:
                # the same spot again on the same lap (a state that flickers): one zone, the
                # lower speed
                merged = CautionZone(
                    old.centre_m,
                    old.length_m,
                    min(old.speed_mps, max(0.0, float(speed_mps))),
                    old.created_lap,
                    old.hold_laps,
                    old.reason,
                )
                self._zones[k] = merged
                return merged
        zone = CautionZone(
            centre_m=centre,
            length_m=self.caution_before_m,
            speed_mps=max(0.0, float(speed_mps)),
            created_lap=int(created_lap),
            hold_laps=self.caution_laps,
            reason=reason,
        )
        self._zones.append(zone)
        return zone

    def expire_zones(self, lap_in_progress: int) -> None:
        self._zones = [z for z in self._zones if lap_in_progress <= z.created_lap + z.hold_laps]

    # -- the bound and the cost to go --------------------------------------------------------

    def _v_safe_at(self, s_mod: FloatArray) -> FloatArray:
        h = self._l_ref / self._n
        ext = np.concatenate([self._v_safe, self._v_safe[:1]])
        return np.interp(s_mod, np.arange(self._n + 1) * h, ext)

    def v_safe(self, s_m: FloatArray | float) -> FloatArray:
        s = np.atleast_1d(np.asarray(s_m, dtype=np.float64))
        if self._n == 0 or self.n_laps == 0:
            return np.full(s.shape, math.nan)
        return self._v_safe_at(np.mod(s, self._l_ref))

    def v_bound(
        self, s_m: FloatArray | float, lap_in_progress: int, explore_dv_mps: float
    ) -> FloatArray:
        """`v_safe + explore_dv_mps` outside caution zones; inside one `min(v_safe, speed - 0.1)`
        with no exploration. NaN when no lap is stored."""
        s = np.atleast_1d(np.asarray(s_m, dtype=np.float64))
        if self._n == 0 or self.n_laps == 0:
            return np.full(s.shape, math.nan)
        s_mod = np.mod(s, self._l_ref)
        safe = self._v_safe_at(s_mod)
        zone_cap = np.full(s.shape, math.inf)
        for z in self._zones:
            if not z.active_on(lap_in_progress):
                continue
            behind = np.mod(z.centre_m - s_mod, self._l_ref)  # distance back to the event
            inside = behind <= z.length_m
            zone_cap = np.where(inside, np.minimum(zone_cap, max(0.0, z.speed_mps - 0.1)), zone_cap)
        # inside any zone: the lowest of the zones' caps and v_safe, no exploration step
        bound = np.where(np.isfinite(zone_cap), np.minimum(safe, zone_cap), safe + explore_dv_mps)
        return np.maximum(bound, self.floor_mps)

    def cost_to_go(self, s_m: FloatArray | float) -> FloatArray:
        """Q, unwrapped past the lap end. NaN when no lap is stored."""
        s = np.atleast_1d(np.asarray(s_m, dtype=np.float64))
        if self._n == 0 or self.n_laps == 0:
            return np.full(s.shape, math.nan)
        k = np.floor(s / self._l_ref)
        s_mod = s - k * self._l_ref
        h = self._l_ref / self._n
        return np.interp(s_mod, np.arange(self._n + 1) * h, self._q) - k * self._t_best

    def max_bound(self, lap_in_progress: int, explore_dv_mps: float) -> float:
        """Largest bound over the lap grid (NaN with no stored lap)."""
        if self._n == 0 or self.n_laps == 0:
            return math.nan
        grid = np.arange(self._n) * (self._l_ref / self._n)
        return float(np.max(self.v_bound(grid, lap_in_progress, explore_dv_mps)))

    def arrays(
        self,
        station_now_m: float,
        horizon_m: float,
        lap_in_progress: int,
        explore_dv_mps: float,
        spacing_m: float = SPACING_M,
    ) -> tuple[FloatArray, FloatArray, FloatArray]:
        """The guidance arrays: stations every `spacing_m` from `station_now_m` over `horizon_m`
        (horizon / spacing + 1 of them), the bound and Q there. Empty before the first stored
        lap."""
        empty = np.zeros(0)
        if self._n == 0 or self.n_laps == 0 or not math.isfinite(station_now_m):
            return empty, empty, empty
        n = round(horizon_m / spacing_m) + 1
        s = station_now_m + spacing_m * np.arange(n)
        return (
            s,
            np.maximum(self.v_bound(s, lap_in_progress, explore_dv_mps), 0.0),
            self.cost_to_go(s),
        )

    # -- persistence -----------------------------------------------------------------------

    def save(self, path: str | Path) -> None:
        """Write the stored laps and zones as one npz, atomically."""
        path = Path(path)
        data: dict[str, FloatArray] = {
            "format_version": np.array([_FORMAT_VERSION], dtype=float),
            "spacing_m": np.array([self.spacing_m]),
            "l_ref_m": np.array([self._l_ref]),
            "n_laps": np.array([len(self._laps)], dtype=float),
        }
        for i, lap in enumerate(self._laps):
            data[f"lap{i}_meta"] = np.array(
                [lap.lap_index, lap.lap_time_s, lap.length_m, lap.cap_mps], dtype=float
            )
            data[f"lap{i}_v"] = lap.v_mps
            data[f"lap{i}_ctg"] = lap.ctg_s
        data["zones"] = np.array(
            [
                [z.centre_m, z.length_m, z.speed_mps, z.created_lap, z.hold_laps]
                for z in self._zones
            ],
            dtype=float,
        ).reshape(-1, 5)
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_name(path.name + ".tmp")
        with tmp.open("wb") as f:
            np.savez(f, **data)  # type: ignore[arg-type]
        os.replace(tmp, path)

    @classmethod
    def load(cls, path: str | Path) -> StationSet:
        with np.load(Path(path)) as z:
            if int(z["format_version"][0]) != _FORMAT_VERSION:
                raise ValueError(f"{path}: unsupported station set format")
            out = cls(float(z["spacing_m"][0]))
            if math.isfinite(float(z["l_ref_m"][0])):
                out.set_reference_length(float(z["l_ref_m"][0]))
            for i in range(int(z["n_laps"][0])):
                meta = z[f"lap{i}_meta"]
                out._laps.append(
                    StationLap(
                        int(meta[0]),
                        float(meta[1]),
                        float(meta[2]),
                        float(meta[3]),
                        z[f"lap{i}_v"],
                        z[f"lap{i}_ctg"],
                    )
                )
            for row in z["zones"]:
                out._zones.append(
                    CautionZone(
                        float(row[0]),
                        float(row[1]),
                        float(row[2]),
                        int(row[3]),
                        int(row[4]),
                        "loaded",
                    )
                )
        if out._laps:
            out._rebuild()
        return out
