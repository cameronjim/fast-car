"""Map quality numbers for a slam_toolbox map that has no ground truth to compare with.

A real track has no truth map, so the failure that matters (the loop closed at the wrong place
or not at all, and the same wall is laid down twice) is measured from the saved occupancy grid
itself. Everything is numpy; frames and units are the map's (metres).

  * ``thickness_p50_m`` / ``thickness_p90_m``: width of the occupied band along its ridge (the
    cells that are local maxima of the distance to the nearest non-occupied cell). A wall seen
    once is one or two cells wide, the same wall laid down twice a few centimetres apart is a
    band several cells wide. The p90 is the number that moves first when a map smears.
  * ``interior_wall_cells`` and ``interior_fraction``: occupied cells with FREE space on both
    sides along some axis. A real wall has free floor on one side and nothing known behind it
    (the laser does not see through foam); a second copy of a wall inside the lane has free floor
    on both sides. Free-standing walls thinner than a few cells (a thin island) also count, so
    compare against a healthy map of the same track, not against zero.
  * ``wall_cells``: occupied cell count (the 2026-11-11 mapping note's number: fewer is crisper
    for the same track).
  * ``free_regions``: connected regions of free cells of at least ``MIN_REGION_M2``. One lane or
    room is one region; a ghost wall that cuts it in two is two.

``map_metrics`` returns plain floats and ints so a run can be dumped as JSON.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml
from raceline.map_track import MapGrid, _read_pgm

MIN_REGION_M2 = 0.25
# Two occupied bands closer than this many free cells are one wall (smear), not two (a duplicate).
MIN_LAYER_GAP_CELLS = 3
# A wall band wider than this many cells is skipped over when looking for what is behind it.
MAX_BAND_CELLS = 5
_AXES = ((0, 1), (1, 0), (1, 1), (1, -1))  # (d_row, d_col)


def load_map_strict(yaml_path: str | Path) -> MapGrid:
    """A map_server yaml + pgm where only a (nearly) white pixel is free and only a (nearly) black
    one occupied. slam_toolbox writes unknown as 205, which raceline.map_track.load_map's trinary
    rule (free below 0.25 occupancy, 205 reads 0.196) calls free; the metrics here need unknown
    to stay unknown, because "free on both sides of a wall" is what they look for."""
    yaml_path = Path(yaml_path)
    meta = yaml.safe_load(yaml_path.read_text())
    pixels = np.flipud(_read_pgm(yaml_path.parent / meta["image"]))
    return MapGrid(
        occupied=pixels < 100,
        free=pixels > 240,
        resolution_m=float(meta["resolution"]),
        origin_xy=(float(meta["origin"][0]), float(meta["origin"][1])),
    )


def occupied_thickness_cells(occupied: np.ndarray) -> np.ndarray:
    """Band width in cells at every ridge cell of the occupied mask (empty if none).

    Width = 2 * d - 1 where d is the distance from the ridge cell's centre to the nearest
    non-occupied cell centre, so a one cell line reads 1 and a three cell band reads 3."""
    rows, cols = np.nonzero(occupied)
    if rows.size == 0:
        return np.zeros(0)
    padded = np.pad(occupied, 1, constant_values=False)
    # Non-occupied cells touching an occupied one (4-neighbourhood) are the only candidates for
    # the nearest non-occupied cell.
    touching = ~padded & (
        np.roll(padded, 1, 0)
        | np.roll(padded, -1, 0)
        | np.roll(padded, 1, 1)
        | np.roll(padded, -1, 1)
    )
    brow, bcol = np.nonzero(touching)
    boundary = np.column_stack([brow - 1, bcol - 1]).astype(np.float32)
    cells = np.column_stack([rows, cols]).astype(np.float32)
    dist = np.empty(len(cells), dtype=np.float32)
    chunk = max(1, int(4e6 // max(len(boundary), 1)))
    for i in range(0, len(cells), chunk):
        d2 = ((cells[i : i + chunk, None, :] - boundary[None, :, :]) ** 2).sum(axis=2)
        dist[i : i + chunk] = np.sqrt(d2.min(axis=1))
    field = np.zeros(occupied.shape, dtype=np.float32)
    field[rows, cols] = dist
    fpad = np.pad(field, 1)
    neighbours = np.stack(
        [
            np.roll(np.roll(fpad, dr, 0), dc, 1)
            for dr in (-1, 0, 1)
            for dc in (-1, 0, 1)
            if (dr, dc) != (0, 0)
        ]
    )[:, 1:-1, 1:-1].max(axis=0)
    ridge = occupied & (field >= neighbours)
    return 2.0 * field[ridge].astype(float) - 1.0


def interior_wall_mask(grid: MapGrid) -> np.ndarray:
    """Occupied cells with free floor on both sides along at least one of four axes.

    From each occupied cell the walk goes outwards in one direction through the occupied band
    (at most MAX_BAND_CELLS cells) and the first non-occupied cell decides that side: free floor,
    or something else (unknown, the grid edge, a band too thick to cross)."""
    occ, free = grid.occupied, grid.free & ~grid.occupied
    h, w = occ.shape
    rows, cols = np.nonzero(occ)
    out = np.zeros_like(occ)
    for dr, dc in _AXES:
        sides = []
        for sign in (1, -1):
            r = rows + sign * dr
            c = cols + sign * dc
            free_side = np.zeros(len(rows), dtype=bool)
            walking = np.ones(len(rows), dtype=bool)
            for _ in range(MAX_BAND_CELLS + 1):
                walking &= (r >= 0) & (r < h) & (c >= 0) & (c < w)
                rr = np.clip(r, 0, h - 1)
                cc = np.clip(c, 0, w - 1)
                is_occ = occ[rr, cc]
                free_side |= walking & ~is_occ & free[rr, cc]
                walking &= is_occ
                r = r + sign * dr
                c = c + sign * dc
            sides.append(free_side)
        both = sides[0] & sides[1]
        out[rows[both], cols[both]] = True
    return out


def wall_layers(grid: MapGrid, stride_cells: int = 6, n_rays: int = 72, max_range_m: float = 8.0):
    """How many separate walls a ray from free floor crosses before it reaches unknown.

    Rays go out from free cells on a ``stride_cells`` grid. Each ray is followed (half cell
    steps) until it enters an unknown cell or leaves the map; every entry into an occupied band
    after at least ``MIN_LAYER_GAP_CELLS`` free cells since the last band is one layer. A wall
    seen once has free floor in front and nothing known behind it: one layer. The second copy of
    a doubled wall lies in free floor in front of the first: two. Returns the layer count of
    every ray; the caller wants the fraction of rays with two or more."""
    occ, free = grid.occupied, grid.free & ~grid.occupied
    h, w = occ.shape
    rows, cols = np.nonzero(free)
    keep = (rows % stride_cells == 0) & (cols % stride_cells == 0)
    rows, cols = rows[keep], cols[keep]
    if rows.size == 0:
        return np.zeros(0, dtype=int)
    angles = np.linspace(0.0, 2.0 * math.pi, n_rays, endpoint=False)
    dr = np.sin(angles)[None, :] * np.ones((len(rows), 1))
    dc = np.cos(angles)[None, :] * np.ones((len(rows), 1))
    r0 = np.repeat(rows[:, None].astype(float), n_rays, axis=1)
    c0 = np.repeat(cols[:, None].astype(float), n_rays, axis=1)
    layers = np.zeros(r0.shape, dtype=int)
    free_run = np.full(r0.shape, 1e9)  # free cells walked since the last band
    alive = np.ones(r0.shape, dtype=bool)
    steps = int(max_range_m / grid.resolution_m * 2)
    for k in range(1, steps + 1):
        r = np.rint(r0 + 0.5 * k * dr).astype(int)
        c = np.rint(c0 + 0.5 * k * dc).astype(int)
        alive &= (r >= 0) & (r < h) & (c >= 0) & (c < w)
        rr, cc = np.clip(r, 0, h - 1), np.clip(c, 0, w - 1)
        is_occ, is_free = occ[rr, cc], free[rr, cc]
        alive &= is_occ | is_free  # unknown ends the ray
        layers += alive & is_occ & (free_run >= MIN_LAYER_GAP_CELLS)
        free_run = np.where(is_occ, 0.0, free_run + 0.5 * is_free)
    return layers.ravel()


def free_region_areas_m2(grid: MapGrid) -> list[float]:
    """Areas of the 4-connected free regions, largest first."""
    free = grid.free & ~grid.occupied
    h, w = free.shape
    seen = np.zeros_like(free)
    cell_area = grid.resolution_m**2
    areas: list[float] = []
    for r0, c0 in zip(*np.nonzero(free), strict=True):
        if seen[r0, c0]:
            continue
        seen[r0, c0] = True
        frontier = [(int(r0), int(c0))]
        count = 0
        while frontier:
            nxt = []
            for r, c in frontier:
                count += 1
                for rr, cc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
                    if 0 <= rr < h and 0 <= cc < w and free[rr, cc] and not seen[rr, cc]:
                        seen[rr, cc] = True
                        nxt.append((rr, cc))
            frontier = nxt
        areas.append(count * cell_area)
    return sorted(areas, reverse=True)


def map_metrics(grid: MapGrid) -> dict:
    """The numbers above for one map."""
    width = occupied_thickness_cells(grid.occupied) * grid.resolution_m
    interior = interior_wall_mask(grid)
    layers = wall_layers(grid)
    n_occ = int(grid.occupied.sum())
    regions = [a for a in free_region_areas_m2(grid) if a >= MIN_REGION_M2]
    return {
        "wall_cells": n_occ,
        "thickness_p50_m": float(np.percentile(width, 50)) if width.size else math.nan,
        "thickness_p90_m": float(np.percentile(width, 90)) if width.size else math.nan,
        "interior_wall_cells": int(interior.sum()),
        "interior_fraction": float(interior.sum() / n_occ) if n_occ else math.nan,
        "multi_layer_fraction": float((layers >= 2).mean()) if layers.size else math.nan,
        "layers_mean": float(layers.mean()) if layers.size else math.nan,
        "free_regions": len(regions),
        "free_area_m2": float(sum(regions)),
    }
