# metric distance to the nearest wall of a track occupancy grid

from __future__ import annotations

import numpy as np
from scipy.ndimage import distance_transform_edt


class ClearanceMap:
    """distance field from any world point to the nearest occupied cell of an occupancy map."""

    def __init__(self, occupancy_map, resolution_m: float, origin_xy) -> None:
        occupancy = np.asarray(occupancy_map)
        if occupancy.ndim != 2:
            raise ValueError(f"occupancy map must be 2d, got shape {occupancy.shape}")
        if resolution_m <= 0.0:
            raise ValueError(f"resolution_m must be > 0, got {resolution_m}")
        self.resolution_m = float(resolution_m)
        self.origin_x = float(origin_xy[0])
        self.origin_y = float(origin_xy[1])
        # the gym writes 0.0 for occupied and 255.0 for free, and row 0 is the lowest world y
        self._distance_m = distance_transform_edt(occupancy > 0.0) * self.resolution_m

    @classmethod
    def from_track(cls, track) -> "ClearanceMap":
        """clearance field of a loaded gym track."""
        return cls(track.occupancy_map, track.spec.resolution, track.spec.origin[:2])

    def distance_at(self, xs, ys) -> np.ndarray:
        """clearance in metres at world points, zero for anything off the map."""
        xs = np.atleast_1d(np.asarray(xs, dtype=np.float64))
        ys = np.atleast_1d(np.asarray(ys, dtype=np.float64))
        if xs.shape != ys.shape:
            raise ValueError(f"xs and ys must match, got {xs.shape} and {ys.shape}")
        cols = np.floor((xs - self.origin_x) / self.resolution_m).astype(np.int64)
        rows = np.floor((ys - self.origin_y) / self.resolution_m).astype(np.int64)
        height, width = self._distance_m.shape
        on_map = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
        clearance = np.zeros(xs.shape, dtype=np.float64)
        clearance[on_map] = self._distance_m[rows[on_map], cols[on_map]]
        return clearance

    def lateral_bounds(self, xs, ys, directions, max_reach_m: float, min_clearance_m: float):
        """per point, the offsets along +direction bounding the band of at least min_clearance_m
        the point can slide to without crossing a wall, as (upper, lower)."""
        xs = np.asarray(xs, dtype=np.float64).reshape(-1)
        ys = np.asarray(ys, dtype=np.float64).reshape(-1)
        directions = np.asarray(directions, dtype=np.float64).reshape(-1, 2)
        if directions.shape[0] != xs.size:
            raise ValueError(f"need one direction per point, got {directions.shape[0]} and {xs.size}")
        if max_reach_m <= 0.0:
            raise ValueError(f"max_reach_m must be > 0, got {max_reach_m}")
        if min_clearance_m < 0.0:
            raise ValueError(f"min_clearance_m must be >= 0, got {min_clearance_m}")

        step_m = 0.5 * self.resolution_m
        reach = int(max_reach_m / step_m)
        offsets = (np.arange(2 * reach + 1) - reach) * step_m
        sample_x = xs[None, :] + offsets[:, None] * directions[None, :, 0]
        sample_y = ys[None, :] + offsets[:, None] * directions[None, :, 1]
        clearance = self.distance_at(sample_x.ravel(), sample_y.ravel()).reshape(sample_x.shape)
        # the extra step covers the gap between samples, so no wall can hide between two of them
        walkable = clearance >= step_m
        roomy = clearance >= min_clearance_m + step_m

        upper = np.empty(xs.size, dtype=np.float64)
        lower = np.empty(xs.size, dtype=np.float64)
        for index in range(xs.size):
            low, high = _reachable_band(walkable[:, index], roomy[:, index], reach)
            lower[index], upper[index] = low * step_m, high * step_m
        return upper, lower


def _reachable_band(walkable: np.ndarray, roomy: np.ndarray, zero: int) -> tuple[int, int]:
    """sample offsets from zero spanning the roomy run nearest zero, without crossing a wall."""
    if not walkable[zero]:
        return 0, 0
    # a wall on either side of zero ends the search: anything past it is a different corridor
    start, end = zero, zero
    while start > 0 and walkable[start - 1]:
        start -= 1
    while end + 1 < walkable.size and walkable[end + 1]:
        end += 1
    within_reach = np.flatnonzero(roomy[start : end + 1]) + start
    if within_reach.size == 0:
        return 0, 0
    low = high = int(within_reach[np.argmin(np.abs(within_reach - zero))])
    while low > start and roomy[low - 1]:
        low -= 1
    while high < end and roomy[high + 1]:
        high += 1
    return low - zero, high - zero
