"""A raceline from a saved occupancy map: lane centreline, lane geometry and a racing line that
respects the car's real turning limits.

Session workflow (docs/notes/first-boot-runbook.md, "New track session"): slam_toolbox saves a
map (map_server yaml + pgm); this module turns it into

  1. the CENTRELINE of the lane (`trace_centreline`): the ridge of the distance-to-wall field,
     followed from a seed point (the map origin, where the car started mapping, is on the lane)
     in small steps, re-centring across the lane each step, until it closes;
  2. the LANE GEOMETRY (`lane_report`): width along the centreline, the two ends' radii (circles
     fitted to the inner and outer wall at each U-turn), the straights;
  3. a RACING LINE (`racing_line`): offsets from the centreline along its normal that minimise
     curvature inside the free corridor, with the whole car body (not just the rear axle)
     kept `body_clearance_m` off the walls, and the curvature held under what the car's steering
     can do on each side (left and right locks differ). It runs wide into a corner and turns in
     late because that is what minimum curvature does, with no hand-made rule;
  4. the speed profile of the existing pipeline (`raceline.build_raceline_from_centerline`) and
     the usual provenanced CSV (`io.write_raceline_csv`) for tracker_node.

Everything is numpy, no scipy: the distance field is brute force (maps are a few thousand wall
cells), the optimiser a projected accelerated gradient on a quadratic. Frames are the map's.
Units SI, steering and curvature LEFT positive.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from raceline.geometry import (
    arc_length_closed,
    curvature_from_points_closed,
    heading_from_points_closed,
    resample_closed_uniform,
    smooth_closed,
)

# -- Map ------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class MapGrid:
    """A map_server map as boolean grids with ROW 0 AT MIN Y (cell (r, c) is centred at
    origin + ((c + 0.5), (r + 0.5)) * resolution)."""

    occupied: np.ndarray
    free: np.ndarray
    resolution_m: float
    origin_xy: tuple[float, float]

    @property
    def shape(self) -> tuple[int, int]:
        return self.occupied.shape

    def cell_of(self, x: float, y: float) -> tuple[int, int]:
        return (
            math.floor((y - self.origin_xy[1]) / self.resolution_m),
            math.floor((x - self.origin_xy[0]) / self.resolution_m),
        )

    def centre_of(self, row: int, col: int) -> tuple[float, float]:
        return (
            self.origin_xy[0] + (col + 0.5) * self.resolution_m,
            self.origin_xy[1] + (row + 0.5) * self.resolution_m,
        )


def _read_pgm(path: Path) -> np.ndarray:
    data = path.read_bytes()
    tokens: list[bytes] = []
    pos = 0
    while len(tokens) < 4:
        while data[pos : pos + 1].isspace():
            pos += 1
        if data[pos : pos + 1] == b"#":
            while data[pos : pos + 1] != b"\n":
                pos += 1
            continue
        start = pos
        while not data[pos : pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    if tokens[0] != b"P5":
        raise ValueError(f"{path}: binary pgm (P5) expected, got {tokens[0]!r}")
    width, height = int(tokens[1]), int(tokens[2])
    return np.frombuffer(data, dtype=np.uint8, count=width * height, offset=pos + 1).reshape(
        height, width
    )


def load_map(yaml_path: str | Path) -> MapGrid:
    """A map_server yaml + pgm by the trinary rule (occupancy = (255 - pixel) / 255)."""
    yaml_path = Path(yaml_path)
    meta = yaml.safe_load(yaml_path.read_text())
    pixels = np.flipud(_read_pgm(yaml_path.parent / meta["image"])).astype(float)
    occupancy = pixels / 255.0 if meta.get("negate", 0) else (255.0 - pixels) / 255.0
    return MapGrid(
        occupied=occupancy > float(meta.get("occupied_thresh", 0.65)),
        free=occupancy < float(meta.get("free_thresh", 0.25)),
        resolution_m=float(meta["resolution"]),
        origin_xy=(float(meta["origin"][0]), float(meta["origin"][1])),
    )


def lane_mask(grid: MapGrid, seed_xy: tuple[float, float]) -> np.ndarray:
    """The free cells connected to the seed (4-connected flood fill): the lane, without the
    free floor outside the walls. The seed moves to the nearest free cell if it is not on one."""
    free = grid.free & ~grid.occupied
    r0, c0 = grid.cell_of(*seed_xy)
    rows, cols = np.nonzero(free)
    if rows.size == 0:
        raise ValueError("the map has no free cells")
    k = int(np.argmin((rows - r0) ** 2 + (cols - c0) ** 2))
    start = (int(rows[k]), int(cols[k]))
    mask = np.zeros_like(free)
    mask[start] = True
    frontier = [start]
    h, w = free.shape
    while frontier:
        nxt = []
        for r, c in frontier:
            for rr, cc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
                if 0 <= rr < h and 0 <= cc < w and free[rr, cc] and not mask[rr, cc]:
                    mask[rr, cc] = True
                    nxt.append((rr, cc))
        frontier = nxt
    return mask


class DistanceField:
    """Distance (m) from any point to the nearest occupied cell centre, bilinear between cells,
    computed brute force for every cell of the grid (maps here are a few thousand wall cells)."""

    def __init__(self, grid: MapGrid) -> None:
        self.grid = grid
        rows, cols = np.nonzero(grid.occupied)
        if rows.size == 0:
            raise ValueError("the map has no occupied cells")
        wall = np.column_stack([cols, rows]).astype(np.float32)
        h, w = grid.shape
        qc, qr = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
        queries = np.column_stack([qc.ravel(), qr.ravel()])
        best = np.full(len(queries), np.inf, dtype=np.float32)
        chunk = max(1, int(4e6 // max(len(wall), 1)))
        for i in range(0, len(queries), chunk):
            q = queries[i : i + chunk]
            d2 = ((q[:, None, :] - wall[None, :, :]) ** 2).sum(axis=2)
            best[i : i + chunk] = d2.min(axis=1)
        self.field = (np.sqrt(best).reshape(h, w) * grid.resolution_m).astype(float)

    def at(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Bilinear distance at points; outside the grid reads as the nearest edge cell."""
        res = self.grid.resolution_m
        fx = (np.asarray(x) - self.grid.origin_xy[0]) / res - 0.5
        fy = (np.asarray(y) - self.grid.origin_xy[1]) / res - 0.5
        h, w = self.field.shape
        fx = np.clip(fx, 0.0, w - 1.001)
        fy = np.clip(fy, 0.0, h - 1.001)
        c0, r0 = np.floor(fx).astype(int), np.floor(fy).astype(int)
        tx, ty = fx - c0, fy - r0
        f = self.field
        return (
            f[r0, c0] * (1 - tx) * (1 - ty)
            + f[r0, c0 + 1] * tx * (1 - ty)
            + f[r0 + 1, c0] * (1 - tx) * ty
            + f[r0 + 1, c0 + 1] * tx * ty
        )


# -- Centreline -----------------------------------------------------------------------------------


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _seed_heading(mask: np.ndarray, grid: MapGrid, seed_xy: tuple[float, float]) -> float:
    """Direction of the lane at the seed: the principal axis of the lane cells within 1 m."""
    rows, cols = np.nonzero(mask)
    xs = grid.origin_xy[0] + (cols + 0.5) * grid.resolution_m
    ys = grid.origin_xy[1] + (rows + 0.5) * grid.resolution_m
    near = np.hypot(xs - seed_xy[0], ys - seed_xy[1]) < 1.0
    pts = np.column_stack([xs[near] - xs[near].mean(), ys[near] - ys[near].mean()])
    eigvals, eigvecs = np.linalg.eigh(pts.T @ pts)
    axis = eigvecs[:, int(np.argmax(eigvals))]
    return math.atan2(axis[1], axis[0])


def trace_centreline(
    grid: MapGrid,
    seed_xy: tuple[float, float] = (0.0, 0.0),
    step_m: float = 0.05,
    search_m: float = 0.4,
    max_length_m: float = 200.0,
) -> tuple[np.ndarray, np.ndarray]:
    """The lane's centreline as a closed counter-clockwise polyline, points `step_m` apart.

    Starting at the ridge of the distance field nearest the seed, step along the heading, move
    across the lane (up to ``search_m`` either way) to the local maximum of the distance to the
    walls, take the new heading from the step actually made, and repeat until the trace comes back
    to its start going the same way. Raises ValueError if it does not close within
    ``max_length_m`` (a map with a gap in a wall, or two lanes).
    """
    mask = lane_mask(grid, seed_xy)
    field = DistanceField(grid)

    def recentre(px: float, py: float, heading: float) -> tuple[float, float]:
        offsets = np.arange(-search_m, search_m + 1e-9, 0.01)
        nx, ny = -math.sin(heading), math.cos(heading)
        qx, qy = px + offsets * nx, py + offsets * ny
        score = field.at(qx, qy) - 1e-3 * np.abs(offsets)
        # Only cells of the lane count (a wall's far side is as far from the walls as the lane).
        rr = np.floor((qy - grid.origin_xy[1]) / grid.resolution_m).astype(int)
        cc = np.floor((qx - grid.origin_xy[0]) / grid.resolution_m).astype(int)
        inside = (rr >= 0) & (rr < mask.shape[0]) & (cc >= 0) & (cc < mask.shape[1])
        ok = np.zeros(len(offsets), dtype=bool)
        ok[inside] = mask[rr[inside], cc[inside]]
        score = np.where(ok, score, -np.inf)
        k = int(np.argmax(score))
        if not np.isfinite(score[k]):
            return px, py
        return float(qx[k]), float(qy[k])

    heading = _seed_heading(mask, grid, seed_xy)
    r0, c0 = grid.cell_of(*seed_xy)
    x, y = recentre(*grid.centre_of(r0, c0), heading)
    # Settle the start on the ridge (recentre moves along the normal only).
    for _ in range(3):
        x, y = recentre(x, y, heading)
    start = (x, y)
    points = [start]
    travelled = 0.0
    while travelled < max_length_m:
        nx_, ny_ = x + step_m * math.cos(heading), y + step_m * math.sin(heading)
        nx_, ny_ = recentre(nx_, ny_, heading)
        step_heading = math.atan2(ny_ - y, nx_ - x)
        heading = heading + 0.5 * _wrap(step_heading - heading)
        travelled += math.hypot(nx_ - x, ny_ - y)
        x, y = nx_, ny_
        if (
            travelled > 8 * step_m
            and math.hypot(x - start[0], y - start[1]) < 1.5 * step_m
            and len(points) > 20
        ):
            break
        points.append((x, y))
    else:
        raise ValueError(f"the centreline did not close within {max_length_m} m")
    xs, ys = np.array([p[0] for p in points]), np.array([p[1] for p in points])
    xs, ys = resample_closed_uniform(xs, ys, step_m)
    # The walk jitters by a cell or so; smooth over about 0.35 m (twice) before anyone takes a
    # curvature of it.
    for _ in range(2):
        xs, ys = smooth_closed(xs, ys, max(int(0.35 / step_m) | 1, 3))
    area = 0.5 * float(np.sum(xs * np.roll(ys, -1) - np.roll(xs, -1) * ys))
    if area < 0.0:  # clockwise: report counter-clockwise, callers reverse for the other way
        xs, ys = xs[::-1].copy(), ys[::-1].copy()
    return xs, ys


# -- Lane geometry --------------------------------------------------------------------------------


def wall_distances(
    grid: MapGrid, xs: np.ndarray, ys: np.ndarray, max_m: float = 3.0
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """For each point of a closed path, the distance to the first occupied cell along the left
    and right normals (m), and the two hit points (left_xy, right_xy) as (n, 2) arrays."""
    heading = heading_from_points_closed(xs, ys)
    nx, ny = -np.sin(heading), np.cos(heading)
    occupied = grid.occupied
    h, w = occupied.shape
    steps = np.arange(0.0, max_m, grid.resolution_m / 2.0)
    out = []
    for sign in (1.0, -1.0):
        qx = xs[:, None] + sign * steps[None, :] * nx[:, None]
        qy = ys[:, None] + sign * steps[None, :] * ny[:, None]
        rr = np.floor((qy - grid.origin_xy[1]) / grid.resolution_m).astype(int)
        cc = np.floor((qx - grid.origin_xy[0]) / grid.resolution_m).astype(int)
        inside = (rr >= 0) & (rr < h) & (cc >= 0) & (cc < w)
        hit = np.zeros(qx.shape, dtype=bool)
        hit[inside] = occupied[rr[inside], cc[inside]]
        hit[~inside] = True
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), len(steps) - 1)
        d = steps[first]
        out.append((d, np.column_stack([xs + sign * d * nx, ys + sign * d * ny])))
    (dl, pl), (dr, pr) = out
    return dl, dr, pl, pr


def fit_circle(points: np.ndarray) -> tuple[float, float, float]:
    """Algebraic (Kasa) circle fit: (cx, cy, radius)."""
    x, y = points[:, 0], points[:, 1]
    a = np.column_stack([2 * x, 2 * y, np.ones(len(x))])
    b = x * x + y * y
    (cx, cy, c), *_ = np.linalg.lstsq(a, b, rcond=None)
    return float(cx), float(cy), float(math.sqrt(max(c + cx * cx + cy * cy, 0.0)))


@dataclass
class LaneReport:
    length_m: float
    width_mean_m: float
    width_median_m: float
    width_p10_m: float
    width_p90_m: float
    width_min_m: float
    width_max_m: float
    surface_width_median_m: float
    ends: list[dict] = field(default_factory=list)
    island: dict = field(default_factory=dict)
    straights_m: list[float] = field(default_factory=list)
    centre_size_m: tuple[float, float] = (0.0, 0.0)
    start_xy: tuple[float, float] = (0.0, 0.0)

    def as_dict(self) -> dict:
        return {
            "length_m": round(self.length_m, 3),
            "width_mean_m": round(self.width_mean_m, 3),
            "width_median_m": round(self.width_median_m, 3),
            "width_p10_m": round(self.width_p10_m, 3),
            "width_p90_m": round(self.width_p90_m, 3),
            "width_min_m": round(self.width_min_m, 3),
            "width_max_m": round(self.width_max_m, 3),
            "surface_width_median_m": round(self.surface_width_median_m, 3),
            "ends": self.ends,
            "island": self.island,
            "straights_m": [round(s, 3) for s in self.straights_m],
            "centre_size_m": [round(v, 3) for v in self.centre_size_m],
            "start_xy": [round(v, 3) for v in self.start_xy],
        }


def _inside_polygon(px: np.ndarray, py: np.ndarray, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    """Even-odd ray casting: which points are inside the closed polygon (xs, ys)."""
    inside = np.zeros(len(px), dtype=bool)
    x0, y0 = xs, ys
    x1, y1 = np.roll(xs, -1), np.roll(ys, -1)
    for ax, ay, bx, by in zip(x0, y0, x1, y1, strict=True):
        crosses = (ay > py) != (by > py)
        with np.errstate(divide="ignore", invalid="ignore"):
            x_at = ax + (py - ay) * (bx - ax) / (by - ay)
        inside ^= crosses & (px < x_at)
    return inside


def _circular_mean(values: np.ndarray, window: int) -> np.ndarray:
    kernel = np.ones(window) / window
    half = window // 2
    padded = np.concatenate([values[-half:], values, values[: window - half - 1]])
    return np.convolve(padded, kernel, mode="valid")


def _band_centre_distance(grid: MapGrid, xs, ys, sign: float, max_m: float = 3.0) -> np.ndarray:
    """Distance along a normal to the MIDDLE of the first occupied run (the wall band), as
    opposed to its near edge: a smeared wall's best estimate of where the surface is."""
    heading = heading_from_points_closed(xs, ys)
    nx, ny = -np.sin(heading) * sign, np.cos(heading) * sign
    steps = np.arange(0.0, max_m, grid.resolution_m / 2.0)
    h, w = grid.occupied.shape
    out = np.zeros(len(xs))
    for i in range(len(xs)):
        qx, qy = xs[i] + steps * nx[i], ys[i] + steps * ny[i]
        rr = np.floor((qy - grid.origin_xy[1]) / grid.resolution_m).astype(int)
        cc = np.floor((qx - grid.origin_xy[0]) / grid.resolution_m).astype(int)
        inside = (rr >= 0) & (rr < h) & (cc >= 0) & (cc < w)
        hit = np.zeros(len(steps), dtype=bool)
        hit[inside] = grid.occupied[rr[inside], cc[inside]]
        if not hit.any():
            out[i] = max_m
            continue
        first = int(np.argmax(hit))
        last = first
        while last + 1 < len(hit) and hit[last + 1]:
            last += 1
        out[i] = 0.5 * (steps[first] + steps[last])
    return out


def lane_report(grid: MapGrid, xs: np.ndarray, ys: np.ndarray) -> LaneReport:
    """Width, ends and straights of a counter-clockwise lane.

    WIDTH is the free corridor along the centreline's normals (near edge of one wall band to the
    near edge of the other), and ``surface_width_median_m`` the same to the middle of each band.
    An END is a peak of the centreline's curvature (smoothed over 0.6 m) tighter than 1.25 m
    radius; circles are fitted to the points the left (island) and right (outer wall) normals hit
    and to the centreline within 60 degrees of heading either side of the peak. A STRAIGHT is a
    stretch with the smoothed curvature under 0.3 / m.
    """
    s, seg = arc_length_closed(xs, ys)
    length = float(s[-1] + seg[-1])
    n = len(xs)
    ds = length / n
    dl, dr, _pl, _pr = wall_distances(grid, xs, ys)
    # A ray through a gap in a wall reads the far side of the infield: only rays that hit a wall
    # within 1.6 m each side count as a width.
    plausible = (dl < 1.6) & (dr < 1.6)
    width = (dl + dr)[plausible] if plausible.sum() > 10 else dl + dr
    surface = _band_centre_distance(grid, xs, ys, 1.0) + _band_centre_distance(grid, xs, ys, -1.0)
    if plausible.sum() > 10:
        surface = surface[plausible]
    kappa = _circular_mean(curvature_from_points_closed(xs, ys), max(int(0.6 / ds), 3))
    ends: list[dict] = []
    peak_floor = 0.8
    candidates = [
        i
        for i in range(n)
        if kappa[i] >= peak_floor
        and kappa[i] >= kappa[(i - 1) % n]
        and kappa[i] >= kappa[(i + 1) % n]
    ]
    # Keep the strongest of any group of peaks within 1 m of path.
    chosen: list[int] = []
    for i in sorted(candidates, key=lambda k: -kappa[k]):
        if all(min((i - j) % n, (j - i) % n) * ds > 1.0 for j in chosen):
            chosen.append(i)
    occ_rows, occ_cols = np.nonzero(grid.occupied)
    occ_x = grid.origin_xy[0] + (occ_cols + 0.5) * grid.resolution_m
    occ_y = grid.origin_xy[1] + (occ_rows + 0.5) * grid.resolution_m
    # The island is the occupied cells inside the centreline loop, the outer wall the rest.
    inside = _inside_polygon(occ_x, occ_y, xs, ys)
    island_report: dict = {}
    if inside.sum() >= 5:
        pts = np.column_stack([occ_x[inside], occ_y[inside]])
        centred = pts - pts.mean(axis=0)
        _eigvals, eigvecs = np.linalg.eigh(centred.T @ centred)
        along = centred @ eigvecs[:, 1]
        across = centred @ eigvecs[:, 0]
        res = grid.resolution_m
        island_report = {
            "island_length_m": round(float(np.ptp(along)) + res, 3),
            "island_width_m": round(float(np.ptp(across)) + res, 3),
            "island_centre_xy": [
                round(float(pts[:, 0].mean()), 3),
                round(float(pts[:, 1].mean()), 3),
            ],
            "island_axis_deg": round(math.degrees(math.atan2(eigvecs[1, 1], eigvecs[0, 1])), 1),
        }
    outer_xy = np.column_stack([occ_x[~inside], occ_y[~inside]])
    for i in sorted(chosen):
        # The arc round the peak where the curvature is at least 60 percent of it.
        arc = [i]
        for direction in (1, -1):
            j = i
            while kappa[(j + direction) % n] >= 0.6 * kappa[i] and len(arc) < n:
                j = (j + direction) % n
                arc.append(j)
        idx = np.array(arc)
        if len(idx) < 6:
            continue
        ccx, ccy, r_centre = fit_circle(np.column_stack([xs[idx], ys[idx]]))
        # The outer wall by its near face: for every 6 degrees of the arc (seen from the circle's
        # centre), the nearest outer-wall cell farther out than the centreline; a circle through
        # those. Robust to the gaps in a map's walls that let a normal ray through.
        ang_arc = np.degrees(np.arctan2(ys[idx] - ccy, xs[idx] - ccx))
        rad_arc = np.radians(ang_arc)
        mid = float(np.degrees(np.arctan2(np.sin(rad_arc).mean(), np.cos(rad_arc).mean())))
        spread = np.abs(
            np.arctan2(np.sin(np.radians(ang_arc - mid)), np.cos(np.radians(ang_arc - mid)))
        )
        half = float(np.clip(spread.max(), math.radians(40.0), math.radians(70.0)))
        ang = np.arctan2(outer_xy[:, 1] - ccy, outer_xy[:, 0] - ccx)
        off = np.arctan2(np.sin(ang - math.radians(mid)), np.cos(ang - math.radians(mid)))
        d = np.hypot(outer_xy[:, 0] - ccx, outer_xy[:, 1] - ccy)
        near = []
        for lo in np.arange(-half, half, math.radians(6.0)):
            sel = (off >= lo) & (off < lo + math.radians(6.0)) & (d > r_centre)
            if sel.any():
                k = int(np.argmin(np.where(sel, d, np.inf)))
                near.append(outer_xy[k])
        if len(near) < 5:
            continue
        ocx, ocy, r_outer = fit_circle(np.array(near))
        ends.append(
            {
                "at_xy": [round(float(xs[i]), 3), round(float(ys[i]), 3)],
                "arc_deg": round(math.degrees(2.0 * half)),
                "centreline_radius_m": round(r_centre, 3),
                "outer_wall_radius_m": round(r_outer, 3),
                "outer_wall_circle_centre_xy": [round(ocx, 3), round(ocy, 3)],
            }
        )
    flat = np.abs(kappa) < 0.3
    straights: list[float] = []
    if flat.any() and not flat.all():
        start = int(np.argmax(~flat))
        run = 0
        for k in range(n):
            if flat[(start + k) % n]:
                run += 1
            elif run:
                straights.append(run * ds)
                run = 0
        if run:
            straights.append(run * ds)
    return LaneReport(
        length_m=length,
        width_mean_m=float(width.mean()),
        width_median_m=float(np.median(width)),
        width_p10_m=float(np.percentile(width, 10)),
        width_p90_m=float(np.percentile(width, 90)),
        width_min_m=float(width.min()),
        width_max_m=float(width.max()),
        surface_width_median_m=float(np.median(surface)),
        ends=ends,
        island=island_report,
        straights_m=straights,
        centre_size_m=(float(np.ptp(xs)), float(np.ptp(ys))),
        start_xy=(float(xs[0]), float(ys[0])),
    )


# -- Racing line ----------------------------------------------------------------------------------


@dataclass(frozen=True)
class CarGeometry:
    """The body about the rear axle (base_link): metres."""

    wheelbase_m: float
    front_overhang_m: float
    rear_overhang_m: float
    width_m: float

    def perimeter(self, per_edge: int = 6) -> np.ndarray:
        """Points on the body outline in the rear-axle frame, corners included."""
        x0, x1 = -self.rear_overhang_m, self.wheelbase_m + self.front_overhang_m
        y = self.width_m / 2.0
        xs = np.linspace(x0, x1, per_edge)
        ys = np.linspace(-y, y, max(3, per_edge // 2 + 1))
        pts = [(x, y) for x in xs] + [(x, -y) for x in xs]
        pts += [(x0, v) for v in ys] + [(x1, v) for v in ys]
        return np.array(pts)


@dataclass
class LineResult:
    x_m: np.ndarray
    y_m: np.ndarray
    offsets_m: np.ndarray
    max_left_curvature_1pm: float
    max_right_curvature_1pm: float
    min_body_clearance_m: float
    curvature_violations: int
    clearance_violations: int
    iterations: int


@dataclass(frozen=True)
class _Body:
    """The body outline against the distance field: ``required_m`` clear of every wall at each of
    the outline points ``per`` (rear-axle frame), penalised by ``weight`` x deficit squared."""

    field: DistanceField
    per: np.ndarray
    required_m: float
    weight: float


def _terms(
    centre: np.ndarray,
    normal: np.ndarray,
    a: np.ndarray,
    klim_left: float,
    klim_right: float,
    rho: float,
    body: _Body | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Per point of the closed path ``centre + a * normal``: the cost term kappa^2 ds (the bending
    energy, kappa the circle through the point and its neighbours, ds half the two chords) plus
    rho times the same for the part of kappa over the left or right limit, plus (with ``body``)
    the clearance penalty of the body outline laid along the path; and kappa."""
    p = centre + a[:, None] * normal
    d1 = p - np.roll(p, 1, axis=0)
    d2 = np.roll(p, -1, axis=0) - p
    d3 = np.roll(p, -1, axis=0) - np.roll(p, 1, axis=0)
    l1, l2, l3 = (np.hypot(d[:, 0], d[:, 1]) for d in (d1, d2, d3))
    kappa = 2.0 * (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.maximum(l1 * l2 * l3, 1e-9)
    ds = 0.5 * (l1 + l2)
    over = np.maximum(kappa - klim_left, 0.0) + np.maximum(-kappa - klim_right, 0.0)
    terms = (kappa**2 + rho * over**2) * ds
    if body is not None:
        c, sn = d3[:, 0] / np.maximum(l3, 1e-9), d3[:, 1] / np.maximum(l3, 1e-9)
        gx = p[:, 0:1] + c[:, None] * body.per[None, :, 0] - sn[:, None] * body.per[None, :, 1]
        gy = p[:, 1:2] + sn[:, None] * body.per[None, :, 0] + c[:, None] * body.per[None, :, 1]
        deficit = np.maximum(body.required_m - body.field.at(gx, gy), 0.0)
        terms = terms + body.weight * np.sum(deficit**2, axis=1) * ds
    return terms, kappa


def _gradient(centre, normal, a, klim_left, klim_right, rho, body, eps: float = 1e-6) -> np.ndarray:
    """d(sum of terms)/d a_i by finite differences, four perturbed groups of every fourth index
    at once (a point only touches the terms of itself and its two neighbours, so indices four
    apart never share one)."""
    base, _ = _terms(centre, normal, a, klim_left, klim_right, rho, body)
    n = len(a)
    grad = np.zeros(n)
    for r in range(4):
        idx = np.arange(r, n, 4)
        bumped = a.copy()
        bumped[idx] += eps
        terms, _ = _terms(centre, normal, bumped, klim_left, klim_right, rho, body)
        delta = (terms - base) / eps
        for shift in (-1, 0, 1):
            grad[idx] += delta[(idx + shift) % n]
    return grad


def _solve_offsets(
    centre: np.ndarray,
    normal: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    klim_left: float,
    klim_right: float,
    start: np.ndarray,
    body: _Body | None = None,
    rho: float = 30.0,
    iterations: int = 4000,
) -> np.ndarray:
    """Minimise the bending energy of the closed path ``centre + a * normal`` (plus the penalty
    on curvature over the limits) over ``lower <= a <= upper``: projected gradient descent on a
    smoothed gradient (a Sobolev preconditioner applied with an FFT, which is what lets the
    smooth, slow modes move) with a backtracking step."""
    n = len(start)
    freq = 2.0 - 2.0 * np.cos(2.0 * np.pi * np.arange(n) / n)
    smoother = 1.0 / (1.0 + 4.0 * freq**2)
    a = np.clip(start, lower, upper)
    cost = float(np.sum(_terms(centre, normal, a, klim_left, klim_right, rho, body)[0]))
    alpha = 1e-3
    stalled = 0
    for _ in range(iterations):
        grad = _gradient(centre, normal, a, klim_left, klim_right, rho, body)
        direction = np.real(np.fft.ifft(np.fft.fft(grad) * smoother))
        improved = False
        for _try in range(25):
            trial = np.clip(a - alpha * direction, lower, upper)
            trial_cost = float(
                np.sum(_terms(centre, normal, trial, klim_left, klim_right, rho, body)[0])
            )
            if trial_cost < cost:
                improved = True
                break
            alpha *= 0.5
        if not improved:
            break
        stalled = stalled + 1 if cost - trial_cost < 1e-10 * max(cost, 1e-9) else 0
        a, cost = trial, trial_cost
        alpha *= 1.5
        if stalled >= 20:
            break
    return a


def _curvature(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    return curvature_from_points_closed(x, y)


def _body_clearance(
    field: DistanceField, x: np.ndarray, y: np.ndarray, car: CarGeometry
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Per path point: the smallest distance from any body outline point to a wall (m), and the
    outline point's side (+1 left, -1 right) and body-frame x of that worst point."""
    heading = heading_from_points_closed(x, y)
    c, s = np.cos(heading), np.sin(heading)
    per = car.perimeter()
    gx = x[:, None] + c[:, None] * per[None, :, 0] - s[:, None] * per[None, :, 1]
    gy = y[:, None] + s[:, None] * per[None, :, 0] + c[:, None] * per[None, :, 1]
    d = field.at(gx, gy)
    worst = np.argmin(d, axis=1)
    return d[np.arange(len(x)), worst], np.sign(per[worst, 1] + 1e-12), per[worst, 0]


def _free_reach(
    field: DistanceField,
    cx: np.ndarray,
    cy: np.ndarray,
    normal: np.ndarray,
    margin_m: float,
    sign: float,
    max_m: float = 3.0,
) -> np.ndarray:
    """How far each centreline point can move along its normal (``sign`` +1 left, -1 right)
    before the distance to a wall drops under ``margin_m``. Marching on the distance field, not
    a ray to the first wall cell, so a gap in a map's wall does not let the path through."""
    steps = np.arange(0.0, max_m, field.grid.resolution_m / 2.0)
    qx = cx[:, None] + sign * steps[None, :] * normal[:, 0:1]
    qy = cy[:, None] + sign * steps[None, :] * normal[:, 1:2]
    ok = field.at(qx, qy) >= margin_m
    # A point whose own spot is already too close can not move that way at all.
    blocked = ~ok
    first_bad = np.where(blocked.any(axis=1), blocked.argmax(axis=1), len(steps))
    reach = steps[np.maximum(first_bad - 1, 0)]
    return np.where(first_bad == 0, 0.0, reach)


def racing_line(
    grid: MapGrid,
    centre_x: np.ndarray,
    centre_y: np.ndarray,
    car: CarGeometry,
    *,
    max_left_curvature_1pm: float,
    max_right_curvature_1pm: float,
    body_clearance_m: float = 0.08,
    step_m: float = 0.1,
    max_rounds: int = 5,
    body_weight: float = 2000.0,
) -> LineResult:
    """Offsets from the centreline that minimise curvature, inside the corridor, with the body
    outline at least ``body_clearance_m`` from every wall and the path's curvature under the
    limits (left positive: ``max_left_curvature_1pm`` is tan(left lock) / wheelbase and so on,
    already reduced by whatever control margin the caller wants).

    The objective is the path's bending energy (sum of kappa^2 ds), minimised by projected gradient
    descent (``_solve_offsets``) with penalties for curvature over a limit and for the body
    outline closer to a wall than allowed. When a round leaves either broken its penalty is
    tripled and the solve continues from there, up to ``max_rounds``; the best round is kept. The
    counts left over are in the result (zero for both is feasible).
    """
    field = DistanceField(grid)
    cx, cy = resample_closed_uniform(centre_x, centre_y, step_m)
    n = len(cx)
    heading = heading_from_points_closed(cx, cy)
    centre = np.column_stack([cx, cy])
    normal = np.column_stack([-np.sin(heading), np.cos(heading)])
    base = car.width_m / 2.0 + body_clearance_m
    upper = _free_reach(field, cx, cy, normal, base, +1.0)  # left
    lower = -_free_reach(field, cx, cy, normal, base, -1.0)  # right
    a = np.zeros(n)
    kmax_l, kmax_r = max_left_curvature_1pm, max_right_curvature_1pm
    outline = _Body(field, car.perimeter(per_edge=5), body_clearance_m, body_weight)
    rho = 30.0
    best: tuple[int, np.ndarray, int, int] | None = None
    rounds = 0
    for rounds in range(1, max_rounds + 1):
        a = _solve_offsets(centre, normal, lower, upper, kmax_l, kmax_r, a, outline, rho)
        px, py = cx + a * normal[:, 0], cy + a * normal[:, 1]
        kappa = _curvature(px, py)
        curvature_bad = int(((kappa > kmax_l * 1.05) | (kappa < -kmax_r * 1.05)).sum())
        clear, _, _ = _body_clearance(field, px, py, car)
        clearance_bad = int((clear < body_clearance_m - 0.01).sum())
        score = 10 * clearance_bad + curvature_bad
        if best is None or score < best[0]:
            best = (score, a.copy(), curvature_bad, clearance_bad)
        if score == 0:
            break
        # Not clean yet: push harder on whichever limit is broken, and go on from here.
        if clearance_bad:
            outline = _Body(field, outline.per, outline.required_m, outline.weight * 3.0)
        if curvature_bad:
            rho *= 3.0
    assert best is not None
    _, a, curvature_bad, clearance_bad = best
    px, py = cx + a * normal[:, 0], cy + a * normal[:, 1]
    kappa = _curvature(px, py)
    clear, _, _ = _body_clearance(field, px, py, car)
    return LineResult(
        x_m=px,
        y_m=py,
        offsets_m=a,
        max_left_curvature_1pm=float(max(kappa.max(), 0.0)),
        max_right_curvature_1pm=float(max(-kappa.min(), 0.0)),
        min_body_clearance_m=float(clear.min()),
        curvature_violations=curvature_bad,
        clearance_violations=clearance_bad,
        iterations=rounds,
    )


def curvature_limits(
    wheelbase_m: float, left_lock_rad: float, right_lock_rad: float, margin: float
) -> tuple[float, float]:
    """Curvature the car can hold, left and right (1/m): tan(lock) / wheelbase, times ``margin``
    (e.g. 0.85) to leave the controller some steering to correct with."""
    return (
        margin * math.tan(left_lock_rad) / wheelbase_m,
        margin * math.tan(right_lock_rad) / wheelbase_m,
    )
