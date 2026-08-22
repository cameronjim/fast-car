"""Centerline extraction from a track mask ("occupancy grid" input mode).

This is a SIMPLIFIED grid convention, not a full ROS map_server/PGM map: ``grid`` is a 2D
array where ``1`` means "drivable track surface" and ``0`` means "not drivable" (off-track,
either infield or outside the track). A real venue map (ROS map_server YAML+PGM, or the
occupancy convention f1tenth_gym maps use, see ``config/vehicle_params.md``-adjacent notes
in ``sim/bridge``) is a genuinely separate, larger integration -- out of scope for this
first version and left for whoever brings in a real venue map. This module exists so the
tool has *an* occupancy-grid input path and so the "raceline stays within track bounds on
the fixture map" L1 test (claude-docs/12-testing.md) has a fixture to run against.

Extraction method: a radial sweep from a caller-supplied center point. For each of
``num_angle_samples`` angles, march outward in ``resolution/2`` steps and record the first
drivable->non-drivable transition pair (inner edge, outer edge); the centerline point for
that angle is their midpoint. This assumes the track is "star-convex" around the given
center (true for a simple closed ring/annulus, and for centers placed at a stadium
track's own center) -- a real venue map with a more complex topology would need a proper
skeleton/medial-axis method instead. That is a documented limitation of this first version,
not a hidden one.
"""

from __future__ import annotations

import math

import numpy as np


def annulus_track_mask(
    resolution_m: float, inner_radius_m: float, outer_radius_m: float, margin_m: float = 1.0
) -> tuple[np.ndarray, float, tuple[float, float]]:
    """A synthetic ring-shaped track mask, centered at world-frame origin.

    Returns ``(grid, resolution_m, origin)`` where ``origin`` is the world-frame (x, y) of
    grid cell ``[0, 0]`` (ROS map_server convention: origin at the bottom-left / minimum
    corner). Used as the fixture map for the L1 bounds test.
    """
    if inner_radius_m <= 0.0 or outer_radius_m <= inner_radius_m:
        raise ValueError(
            f"require 0 < inner_radius_m < outer_radius_m, got {inner_radius_m}, {outer_radius_m}"
        )
    extent = outer_radius_m + margin_m
    size = round(2.0 * extent / resolution_m)
    half = size // 2
    idx = np.arange(size)
    xs = (idx - half) * resolution_m
    ys = (idx - half) * resolution_m
    xx, yy = np.meshgrid(xs, ys, indexing="xy")
    r = np.hypot(xx, yy)
    grid = ((r >= inner_radius_m) & (r <= outer_radius_m)).astype(np.uint8)
    origin = (-half * resolution_m, -half * resolution_m)
    return grid, resolution_m, origin


def centerline_from_track_mask(
    grid: np.ndarray,
    resolution_m: float,
    origin: tuple[float, float],
    center_xy: tuple[float, float],
    num_angle_samples: int = 360,
    max_radius_m: float | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Radial-sweep centerline extraction; see module docstring for the method and its
    star-convexity assumption. Returns ``(x, y)`` centerline points in world frame, one per
    angle that found both an inner and an outer edge (angles that find neither -- e.g. a
    ray that misses the ring entirely -- are skipped).
    """
    ny, nx = grid.shape
    ox, oy = origin
    cx, cy = center_xy
    if max_radius_m is None:
        # Comfortably covers the grid's extent from an arbitrary interior center point.
        max_radius_m = math.hypot(nx * resolution_m, ny * resolution_m)
    step = resolution_m / 2.0

    def _occupied(wx: float, wy: float) -> bool:
        px = round((wx - ox) / resolution_m)
        py = round((wy - oy) / resolution_m)
        if 0 <= px < nx and 0 <= py < ny:
            return bool(grid[py, px])
        return False

    xs_out: list[float] = []
    ys_out: list[float] = []
    for theta in np.linspace(0.0, 2.0 * math.pi, num_angle_samples, endpoint=False):
        dxu, dyu = math.cos(theta), math.sin(theta)
        inner_r: float | None = None
        outer_r: float | None = None
        r = 0.0
        while r <= max_radius_m:
            occ = _occupied(cx + r * dxu, cy + r * dyu)
            if occ and inner_r is None:
                inner_r = r
            elif (not occ) and inner_r is not None:
                outer_r = r
                break
            r += step
        if inner_r is not None and outer_r is not None:
            mid_r = 0.5 * (inner_r + outer_r)
            xs_out.append(cx + mid_r * dxu)
            ys_out.append(cy + mid_r * dyu)

    if len(xs_out) < 3:
        raise ValueError(
            "centerline_from_track_mask found fewer than 3 valid points; check center_xy/mask"
        )
    return np.array(xs_out), np.array(ys_out)
