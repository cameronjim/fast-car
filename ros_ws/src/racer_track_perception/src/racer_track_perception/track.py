"""The local track in the car frame: 16 stations 0.20 m apart, as the /pnp/local_track message
carries it, and the validity rules a consumer applies.

`LocalTrackData` is ROS-free. Station 0 is the centreline point nearest base_link, station k lies
k * 0.20 m further along the travel direction. `valid_count` says how many leading stations may be
used; 0 means "no track".
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

K_STATIONS = 16
STATION_SPACING_M = 0.20
HALF_WIDTH_CAP_M = 2.0
# sigma written for a station that is not valid (strictly above any sigma_max_m a node accepts).
INVALID_SIGMA_M = 1.0


def _zeros() -> NDArray[np.float32]:
    return np.zeros(K_STATIONS, dtype=np.float32)


@dataclass
class LocalTrackData:
    """Arrays of K_STATIONS float32 plus the count of usable stations and the two self-checks."""

    x_m: NDArray[np.float32] = field(default_factory=_zeros)
    y_m: NDArray[np.float32] = field(default_factory=_zeros)
    half_width_left_m: NDArray[np.float32] = field(default_factory=_zeros)
    half_width_right_m: NDArray[np.float32] = field(default_factory=_zeros)
    sigma_m: NDArray[np.float32] = field(
        default_factory=lambda: np.full(K_STATIONS, INVALID_SIGMA_M, dtype=np.float32)
    )
    valid_count: int = 0
    boundary_residual_m: float = math.nan
    intrusion_fraction: float = math.nan

    def arrays_ok(self) -> bool:
        """The consumer's rule: every array has K entries and the first valid_count are finite."""
        arrays = (
            self.x_m,
            self.y_m,
            self.half_width_left_m,
            self.half_width_right_m,
            self.sigma_m,
        )
        if any(len(a) != K_STATIONS for a in arrays):
            return False
        n = self.valid_count
        if not 0 <= n <= K_STATIONS:
            return False
        return all(bool(np.all(np.isfinite(a[:n]))) for a in arrays)


def no_track() -> LocalTrackData:
    """A message the planner reads as "no track"."""
    return LocalTrackData()


def valid_count_from_sigma(sigma_m: NDArray[np.floating], sigma_max_m: float) -> int:
    """Leading stations whose sigma is finite and under sigma_max_m."""
    sigma = np.asarray(sigma_m, dtype=np.float64)
    with np.errstate(invalid="ignore"):
        ok = np.isfinite(sigma) & (sigma < sigma_max_m)
    bad = np.nonzero(~ok)[0]
    return int(bad[0]) if bad.size else int(sigma.size)


def assemble(
    x_m: NDArray[np.floating],
    y_m: NDArray[np.floating],
    half_width_left_m: NDArray[np.floating],
    half_width_right_m: NDArray[np.floating],
    sigma_m: NDArray[np.floating],
    sigma_max_m: float,
    valid_cap: int = K_STATIONS,
) -> LocalTrackData:
    """Build a LocalTrackData from raw arrays of K_STATIONS.

    valid_count is the leading run of stations with sigma under sigma_max_m (capped by
    `valid_cap`) and with every value finite; widths are clipped to [0, 2.0] m. Entries at and
    beyond valid_count are made finite (the last valid station repeated, sigma INVALID_SIGMA_M)
    so the message never carries NaN or inf. A wrong-length input gives "no track".
    """
    arrays = [
        np.asarray(a, dtype=np.float64)
        for a in (x_m, y_m, half_width_left_m, half_width_right_m, sigma_m)
    ]
    if any(a.shape != (K_STATIONS,) for a in arrays):
        return no_track()
    x, y, hl, hr, sg = arrays
    with np.errstate(invalid="ignore"):
        finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(hl) & np.isfinite(hr)
    sg_ok = np.where(finite, sg, np.nan)
    n = min(valid_count_from_sigma(sg_ok, sigma_max_m), int(valid_cap), K_STATIONS)
    if n <= 0:
        return no_track()
    hl = np.clip(hl, 0.0, HALF_WIDTH_CAP_M)
    hr = np.clip(hr, 0.0, HALF_WIDTH_CAP_M)
    out = LocalTrackData(valid_count=n)
    for dst, src in ((out.x_m, x), (out.y_m, y), (out.half_width_left_m, hl)):
        dst[:n] = src[:n]
        dst[n:] = src[n - 1]
    out.half_width_right_m[:n] = hr[:n]
    out.half_width_right_m[n:] = hr[n - 1]
    out.sigma_m[:n] = sg[:n]
    out.sigma_m[n:] = INVALID_SIGMA_M
    return out
