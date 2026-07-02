# arc-length indexed reference line with nearest-point projection

from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree

CLOSING_TOLERANCE_M = 1e-9
# raceline files sample every few centimetres, so a three-point derivative reads mostly noise
CURVATURE_STENCIL_M = 1.0


class RacelineIndex:
    """closed reference line supporting pose projection and arc-length lookup."""

    def __init__(self, xs, ys, speeds=None):
        xs = np.asarray(xs, dtype=np.float64).reshape(-1)
        ys = np.asarray(ys, dtype=np.float64).reshape(-1)
        if xs.shape != ys.shape:
            raise ValueError(f"xs and ys must match, got {xs.shape} and {ys.shape}")
        speeds = None if speeds is None else np.asarray(speeds, dtype=np.float64).reshape(-1)
        if speeds is not None and speeds.shape != xs.shape:
            raise ValueError(f"speeds must match xs, got {speeds.shape} and {xs.shape}")
        # a duplicated closing waypoint makes a zero-length segment, which breaks projection
        if xs.size > 1 and np.hypot(xs[-1] - xs[0], ys[-1] - ys[0]) < CLOSING_TOLERANCE_M:
            xs, ys = xs[:-1], ys[:-1]
            speeds = None if speeds is None else speeds[:-1]
        if xs.size < 3:
            raise ValueError(f"reference line needs at least 3 distinct waypoints, got {xs.size}")

        self.xs = xs
        self.ys = ys
        self.speeds = speeds
        self.n = int(xs.size)
        self._xs_closed = np.append(xs, xs[0])
        self._ys_closed = np.append(ys, ys[0])
        # recomputed from the polyline because raceline files sometimes normalize s to [0, 1]
        steps = np.hypot(np.diff(self._xs_closed), np.diff(self._ys_closed))
        self._ss_closed = np.concatenate(([0.0], np.cumsum(steps)))
        self.ss = self._ss_closed[:-1]
        self.length = float(self._ss_closed[-1])
        self._speeds_closed = None if speeds is None else np.append(speeds, speeds[0])
        self._tree = cKDTree(np.column_stack((xs, ys)))
        headings, curvatures = self._differentiate(CURVATURE_STENCIL_M)
        self._headings_closed = np.append(headings, headings[0])
        self._curvatures_closed = np.append(curvatures, curvatures[0])
        self._cos_closed = np.cos(self._headings_closed)
        self._sin_closed = np.sin(self._headings_closed)

    @classmethod
    def from_track(cls, track, use_centerline: bool = False) -> "RacelineIndex":
        """reference line of a gym track, falling back to the centerline when no raceline file exists."""
        line = track.centerline if use_centerline else track.raceline
        # the gym aliases raceline to centerline when the map ships no raceline csv
        has_raceline = not use_centerline and line is not track.centerline
        speeds = line.vxs if has_raceline and line.vxs is not None else None
        return cls(line.xs, line.ys, speeds=speeds)

    @property
    def has_speed_profile(self) -> bool:
        return self._speeds_closed is not None

    def project(self, x: float, y: float) -> tuple[float, float]:
        """arc-length of the nearest point on the line, with the signed lateral offset."""
        _, nearest = self._tree.query([x, y])
        best_s, best_lateral = 0.0, float("inf")
        for segment in (int(nearest) - 1, int(nearest)):
            s, lateral = self._project_on_segment(x, y, segment % self.n)
            if abs(lateral) < abs(best_lateral):
                best_s, best_lateral = s, lateral
        return best_s, best_lateral

    def point_at(self, s: float) -> tuple[float, float]:
        """position at arc-length s, wrapping around the lap."""
        wrapped = float(s) % self.length
        return (
            float(np.interp(wrapped, self._ss_closed, self._xs_closed)),
            float(np.interp(wrapped, self._ss_closed, self._ys_closed)),
        )

    def speed_at(self, s: float) -> float:
        """reference speed at arc-length s, wrapping around the lap."""
        if self._speeds_closed is None:
            raise ValueError("this reference line carries no speed profile")
        return float(np.interp(float(s) % self.length, self._ss_closed, self._speeds_closed))

    def heading_at(self, s: float) -> float:
        """tangent heading at arc-length s, interpolated through cos and sin so wrap never jumps."""
        wrapped = float(s) % self.length
        cos = np.interp(wrapped, self._ss_closed, self._cos_closed)
        sin = np.interp(wrapped, self._ss_closed, self._sin_closed)
        return float(np.arctan2(sin, cos))

    def curvature_at(self, s):
        """signed curvature at arc-length s, positive turning left, wrapping around the lap."""
        wrapped = np.asarray(s, dtype=np.float64) % self.length
        return np.interp(wrapped, self._ss_closed, self._curvatures_closed)

    def _differentiate(self, stencil_m: float) -> tuple[np.ndarray, np.ndarray]:
        """per-waypoint heading and signed curvature from a central difference wide enough to smooth."""
        offset = max(1, int(round(stencil_m * self.n / self.length)))
        # past a third of the lap a central difference stops describing anything local
        offset = min(offset, max(1, self.n // 3))
        here = np.column_stack((self.xs, self.ys))
        ahead = np.roll(here, -offset, axis=0)
        behind = np.roll(here, offset, axis=0)
        back, forward, chord = here - behind, ahead - here, ahead - behind
        headings = np.arctan2(chord[:, 1], chord[:, 0])
        # menger curvature: twice the triangle's signed area over the product of its side lengths
        cross = back[:, 0] * forward[:, 1] - back[:, 1] * forward[:, 0]
        sides = (
            np.hypot(back[:, 0], back[:, 1])
            * np.hypot(forward[:, 0], forward[:, 1])
            * np.hypot(chord[:, 0], chord[:, 1])
        )
        curvatures = np.divide(2.0 * cross, sides, out=np.zeros_like(cross), where=sides > 1e-12)
        return headings, curvatures

    def _project_on_segment(self, x: float, y: float, index: int) -> tuple[float, float]:
        x0, y0 = self._xs_closed[index], self._ys_closed[index]
        dx, dy = self._xs_closed[index + 1] - x0, self._ys_closed[index + 1] - y0
        span_sq = dx * dx + dy * dy
        travel = np.clip(((x - x0) * dx + (y - y0) * dy) / span_sq, 0.0, 1.0)
        foot_x, foot_y = x0 + travel * dx, y0 + travel * dy
        # cross product sign puts the car on the left (+) or right (-) of the direction of travel
        side = np.sign(dx * (y - y0) - dy * (x - x0)) or 1.0
        s = self._ss_closed[index] + travel * np.sqrt(span_sq)
        return float(s), float(side * np.hypot(x - foot_x, y - foot_y))
