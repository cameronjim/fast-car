"""The safe set: completed clean laps resampled by arc length from the start line. Numpy only.

Each stored lap is resampled on stations `s = k * spacing_m` (0.10 m) with
  x, y     the pose in `map` at that station,
  v        the wheel speed driven there,
  ctg      the cost to go: seconds left to finish the lap from there (the lap time at s = 0,
           0 at the end).

Across laps, per station: `v_ref` is the fastest speed any stored lap drove there and
`cost_to_go` is the smallest cost to go. These are the two things the next version (Learning MPC,
Rosolia and Borrelli) turns into a terminal cost. Only clean laps are stored.

Arc length of a lap is its pose-sample path length scaled to the lap's recorded length, and its
time axis is scaled so the first sample is time 0 and the last is the lap time: this makes
`cost_to_go(0)` the lap time exactly and `cost_to_go` non-increasing. The scale factors are both
within a percent or two of 1 at 20 Hz sampling.

One npz per session; `load` rebuilds the aggregate from the stored laps. Poses are in the `map`
frame of that session (a slam_toolbox map can shift between sessions, so do not compare the
x and y of different sessions without checking).
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from racer_laps.lap_record import LapRecord

FloatArray = NDArray[np.float64]

SPACING_M = 0.10
_FORMAT_VERSION = 1


@dataclass(frozen=True)
class ResampledLap:
    lap_index: int
    lap_time_s: float
    length_m: float
    cap_mps: float
    x_m: FloatArray
    y_m: FloatArray
    v_mps: FloatArray
    cost_to_go_s: FloatArray

    @property
    def n_stations(self) -> int:
        return int(self.v_mps.shape[0])


def resample_lap(record: LapRecord, spacing_m: float = SPACING_M) -> ResampledLap | None:
    """Resample a lap by arc length, or None if it has too little motion to do so."""
    n = record.t_s.shape[0]
    if n < 2 or record.length_m <= 0.0 or record.lap_time_s <= 0.0:
        return None
    ds = np.hypot(np.diff(record.x_m), np.diff(record.y_m))
    total = float(ds.sum())
    if total <= 1e-9:
        return None
    s = np.concatenate([[0.0], np.cumsum(ds)]) * (record.length_m / total)
    t_span = float(record.t_s[-1] - record.t_s[0])
    if t_span <= 0.0:
        return None
    t_rel = (record.t_s - record.t_s[0]) * (record.lap_time_s / t_span)
    # np.interp needs strictly increasing x: keep the first sample of every stationary stretch.
    idx = np.flatnonzero(np.concatenate([[True], np.diff(s) > 1e-9]))
    if idx.shape[0] < 2:
        return None
    s_k = s[idx]
    cols = [record.x_m, record.y_m, record.wheel_speed_mps, t_rel]
    grid = np.arange(math.floor(record.length_m / spacing_m + 1e-9) + 1) * spacing_m
    x, y, v, t_at = (np.interp(grid, s_k, c[idx]) for c in cols)
    ctg = record.lap_time_s - t_at
    ctg[0] = record.lap_time_s
    return ResampledLap(
        lap_index=record.lap_index,
        lap_time_s=record.lap_time_s,
        length_m=record.length_m,
        cap_mps=record.cap_mps,
        x_m=x,
        y_m=y,
        v_mps=v,
        cost_to_go_s=np.maximum(ctg, 0.0),
    )


class SafeSet:
    def __init__(self, spacing_m: float = SPACING_M) -> None:
        self.spacing_m = float(spacing_m)
        self._laps: list[ResampledLap] = []
        self._v_ref: FloatArray = np.zeros(0)
        self._ctg: FloatArray = np.zeros(0)

    @property
    def n_laps(self) -> int:
        return len(self._laps)

    @property
    def laps(self) -> tuple[ResampledLap, ...]:
        return tuple(self._laps)

    @property
    def length_m(self) -> float:
        """Arc length covered by the aggregate (the longest stored lap, on the station grid)."""
        return 0.0 if self._v_ref.shape[0] == 0 else (self._v_ref.shape[0] - 1) * self.spacing_m

    @property
    def best_lap_time_s(self) -> float:
        return min((lap.lap_time_s for lap in self._laps), default=math.nan)

    def add_lap(self, record: LapRecord) -> bool:
        """Store a lap if it is clean and has enough motion. Returns whether it was stored."""
        if not record.clean:
            return False
        lap = resample_lap(record, self.spacing_m)
        if lap is None:
            return False
        self._laps.append(lap)
        self._rebuild()
        return True

    def _rebuild(self) -> None:
        n = max((lap.n_stations for lap in self._laps), default=0)
        v = np.full((len(self._laps), n), np.nan)
        c = np.full((len(self._laps), n), np.nan)
        for i, lap in enumerate(self._laps):
            v[i, : lap.n_stations] = lap.v_mps
            c[i, : lap.n_stations] = lap.cost_to_go_s
        if n == 0:
            self._v_ref = np.zeros(0)
            self._ctg = np.zeros(0)
        else:
            self._v_ref = np.nanmax(v, axis=0)
            self._ctg = np.nanmin(c, axis=0)

    def _query(self, table: FloatArray, s_m: float) -> float:
        if table.shape[0] == 0 or not (0.0 <= s_m <= self.length_m + 1e-9):
            return math.nan
        return float(np.interp(s_m, np.arange(table.shape[0]) * self.spacing_m, table))

    def v_ref(self, s_m: float) -> float:
        """Fastest speed driven at arc length s by any stored lap (NaN outside the stored range)."""
        return self._query(self._v_ref, s_m)

    def cost_to_go(self, s_m: float) -> float:
        """Smallest time to finish the lap from arc length s (NaN outside the stored range)."""
        return self._query(self._ctg, s_m)

    def horizon(self, s_m: float, horizon_m: float) -> tuple[FloatArray, FloatArray, FloatArray]:
        """Stations from `s_m` to `s_m + horizon_m` ahead (clipped to the stored range), as
        (s, v_ref, cost_to_go). Empty arrays when s is outside the stored range."""
        empty = np.zeros(0)
        if self._v_ref.shape[0] == 0 or not (0.0 <= s_m <= self.length_m):
            return empty, empty, empty
        k0 = math.ceil(s_m / self.spacing_m - 1e-9)
        k1 = min(self._v_ref.shape[0] - 1, math.floor((s_m + horizon_m) / self.spacing_m + 1e-9))
        idx = np.arange(k0, k1 + 1)
        s = (idx * self.spacing_m).astype(np.float64)
        return s, self._v_ref[idx].copy(), self._ctg[idx].copy()

    def save(self, path: str | Path) -> None:
        """Write the whole set as one npz, atomically (temp file then rename)."""
        path = Path(path)
        data: dict[str, FloatArray] = {
            "format_version": np.array([_FORMAT_VERSION], dtype=float),
            "spacing_m": np.array([self.spacing_m]),
            "n_laps": np.array([len(self._laps)], dtype=float),
        }
        for i, lap in enumerate(self._laps):
            data[f"lap{i}_meta"] = np.array(
                [lap.lap_index, lap.lap_time_s, lap.length_m, lap.cap_mps], dtype=float
            )
            data[f"lap{i}_x"] = lap.x_m
            data[f"lap{i}_y"] = lap.y_m
            data[f"lap{i}_v"] = lap.v_mps
            data[f"lap{i}_ctg"] = lap.cost_to_go_s
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_name(path.name + ".tmp")
        with tmp.open("wb") as f:
            np.savez(f, **data)  # type: ignore[arg-type]
        os.replace(tmp, path)

    @classmethod
    def load(cls, path: str | Path) -> SafeSet:
        with np.load(Path(path)) as z:
            if int(z["format_version"][0]) != _FORMAT_VERSION:
                raise ValueError(f"{path}: unsupported safe set format {z['format_version'][0]}")
            out = cls(float(z["spacing_m"][0]))
            for i in range(int(z["n_laps"][0])):
                meta = z[f"lap{i}_meta"]
                out._laps.append(
                    ResampledLap(
                        lap_index=int(meta[0]),
                        lap_time_s=float(meta[1]),
                        length_m=float(meta[2]),
                        cap_mps=float(meta[3]),
                        x_m=z[f"lap{i}_x"],
                        y_m=z[f"lap{i}_y"],
                        v_mps=z[f"lap{i}_v"],
                        cost_to_go_s=z[f"lap{i}_ctg"],
                    )
                )
        out._rebuild()
        return out
