"""Loads a tools/raceline-format CSV into the x/y/target-speed arrays f1tenth_gym's
``Track.from_refline`` needs (roadmap task S.2).

Pure stdlib + numpy, no ROS/gym imports -- same "testable as an ordinary L1 unit test"
rationale as this package's ``conversions.py``. Parses the SAME CSV format
``ros_ws/src/racer_control/include/racer_control/raceline.hpp`` parses independently in
C++: `#`-commented provenance header, a header row, then
``s_m,x_m,y_m,heading_rad,curvature_1pm,target_speed_mps`` rows (see
``tools/raceline/io.py``, the format's producer).
"""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

_EXPECTED_HEADER = ("s_m", "x_m", "y_m", "heading_rad", "curvature_1pm", "target_speed_mps")


class RacelineLoadError(ValueError):
    pass


def load_raceline_xy_speed(path: str | Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Returns ``(x_m, y_m, target_speed_mps)`` arrays, in raceline order."""
    path = Path(path)
    if not path.is_file():
        raise RacelineLoadError(f"raceline file not found: {path}")

    rows: list[list[str]] = []
    header_seen = False
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            fields = next(csv.reader([line]))
            if not header_seen:
                if tuple(fields) != _EXPECTED_HEADER:
                    raise RacelineLoadError(
                        f"{path}: expected CSV header {_EXPECTED_HEADER}, got {tuple(fields)}"
                    )
                header_seen = True
                continue
            rows.append(fields)

    if not header_seen:
        raise RacelineLoadError(f"{path}: no CSV header row found")
    if not rows:
        raise RacelineLoadError(f"{path}: no raceline data rows found")

    try:
        data = np.array(rows, dtype=float)
    except ValueError as e:
        raise RacelineLoadError(f"{path}: non-numeric field in raceline data: {e}") from e

    x_m = data[:, 1]
    y_m = data[:, 2]
    target_speed_mps = data[:, 5]
    return x_m, y_m, target_speed_mps


def build_corridor_occupancy(
    x_m: np.ndarray,
    y_m: np.ndarray,
    half_width_m: float,
    resolution_m: float,
    border_m: float = 1.0,
) -> tuple[np.ndarray, tuple[float, float, float]]:
    """A walled occupancy grid for a CLOSED centerline: free within ``half_width_m`` of it.

    GitHub issue 26 (the reactive-controller L5 canary). ``Track.from_refline`` builds an
    occupancy map that is free everywhere, so a LiDAR-driven controller sees no walls at all.
    This builds the map a real track would have: every cell whose centre is within
    ``half_width_m`` of the closed polyline (x_m, y_m) -> (x_m[0], y_m[0]) is free (255), every
    other cell is a wall (0), with ``border_m`` of wall around the corridor's bounding box.

    Returns ``(occupancy_map, origin)`` in f1tenth_gym / ROS map_server layout: row index grows
    with y and column index with x (``f1tenth_gym.envs.laser_models.xy_2_rc``), and ``origin``
    is the world (x, y, yaw) of the lower-left corner of cell (0, 0).

    Pure numpy: each segment only updates the cells inside its own bounding box grown by the
    half width, so the cost is O(segments * (half_width / resolution)^2), not O(segments *
    cells).
    """
    if half_width_m <= 0.0 or resolution_m <= 0.0:
        raise ValueError("half_width_m and resolution_m must be positive")
    x = np.asarray(x_m, dtype=float)
    y = np.asarray(y_m, dtype=float)
    if x.shape != y.shape or x.ndim != 1 or x.size < 2:
        raise ValueError("x_m and y_m must be equal-length 1-D arrays with at least 2 points")

    pad = half_width_m + border_m
    x0 = float(np.min(x)) - pad
    y0 = float(np.min(y)) - pad
    cols = int(np.ceil((float(np.max(x)) + pad - x0) / resolution_m))
    rows = int(np.ceil((float(np.max(y)) + pad - y0) / resolution_m))
    distance = np.full((rows, cols), np.inf)

    xs = np.append(x, x[0])
    ys = np.append(y, y[0])
    reach = int(np.ceil(half_width_m / resolution_m)) + 1
    for i in range(xs.size - 1):
        ax, ay, bx, by = xs[i], ys[i], xs[i + 1], ys[i + 1]
        c_lo = max(int(np.floor((min(ax, bx) - x0) / resolution_m)) - reach, 0)
        c_hi = min(int(np.floor((max(ax, bx) - x0) / resolution_m)) + reach, cols - 1)
        r_lo = max(int(np.floor((min(ay, by) - y0) / resolution_m)) - reach, 0)
        r_hi = min(int(np.floor((max(ay, by) - y0) / resolution_m)) + reach, rows - 1)
        cx = x0 + (np.arange(c_lo, c_hi + 1) + 0.5) * resolution_m
        cy = y0 + (np.arange(r_lo, r_hi + 1) + 0.5) * resolution_m
        px, py = np.meshgrid(cx, cy)
        dx, dy = bx - ax, by - ay
        seg_len2 = dx * dx + dy * dy
        if seg_len2 > 0.0:
            t = np.clip(((px - ax) * dx + (py - ay) * dy) / seg_len2, 0.0, 1.0)
        else:
            t = np.zeros_like(px)
        d = np.hypot(px - (ax + t * dx), py - (ay + t * dy))
        window = distance[r_lo : r_hi + 1, c_lo : c_hi + 1]
        np.minimum(window, d, out=window)

    occupancy = np.where(distance <= half_width_m, 255.0, 0.0).astype(np.float32)
    return occupancy, (x0, y0, 0.0)


def carve_pocket(
    occupancy: np.ndarray,
    origin: tuple[float, float, float],
    resolution_m: float,
    x_m: np.ndarray,
    y_m: np.ndarray,
    half_width_m: float,
    start_s_m: float,
    length_m: float,
    depth_m: float,
    side: str,
) -> np.ndarray:
    """A copy of a corridor map (``build_corridor_occupancy``) with a POCKET cut into one wall.

    Roadmap 2.9 (park_node's L5 scenario): a parking slot is a recess in a row of obstacles.
    Along the CLOSED centreline (x_m, y_m), from arc length ``start_s_m`` (measured from the
    first point, in point order) for ``length_m``, every cell whose centre is on ``side``
    ("right" or "left" of the direction of travel) at a perpendicular distance between 0 and
    ``half_width_m + depth_m`` from the centreline is made free (255): the wall on that side is
    pushed back by ``depth_m`` over that stretch. On a straight stretch the pocket is a
    rectangle whose edges sit on cell boundaries, within half a cell (``resolution_m`` / 2) of
    the nominal values. A cell is tested against the centreline segment it projects onto, so the
    pocket must not span a corner (the caller's job).

    Pure numpy, like ``build_corridor_occupancy``; the map must already be large enough (its
    border at least ``depth_m`` beyond the corridor).
    """
    if side not in ("right", "left"):
        raise ValueError(f"side must be right or left, got {side!r}")
    if length_m <= 0.0 or depth_m <= 0.0 or half_width_m <= 0.0 or resolution_m <= 0.0:
        raise ValueError("length_m, depth_m, half_width_m and resolution_m must be positive")
    x = np.asarray(x_m, dtype=float)
    y = np.asarray(y_m, dtype=float)
    xs = np.append(x, x[0])
    ys = np.append(y, y[0])
    seg_len = np.hypot(np.diff(xs), np.diff(ys))
    s_at = np.concatenate([[0.0], np.cumsum(seg_len)])
    end_s = start_s_m + length_m
    if start_s_m < 0.0 or end_s > s_at[-1]:
        raise ValueError(f"pocket [{start_s_m}, {end_s}] m is outside the loop [0, {s_at[-1]}] m")
    sign = -1.0 if side == "right" else 1.0
    reach = half_width_m + depth_m
    out = occupancy.copy()
    rows, cols = out.shape
    x0, y0, _ = origin
    pad = int(np.ceil(reach / resolution_m)) + 1
    for i in range(xs.size - 1):
        if seg_len[i] <= 0.0 or s_at[i + 1] < start_s_m or s_at[i] > end_s:
            continue
        ax, ay, bx, by = xs[i], ys[i], xs[i + 1], ys[i + 1]
        c_lo = max(int(np.floor((min(ax, bx) - x0) / resolution_m)) - pad, 0)
        c_hi = min(int(np.floor((max(ax, bx) - x0) / resolution_m)) + pad, cols - 1)
        r_lo = max(int(np.floor((min(ay, by) - y0) / resolution_m)) - pad, 0)
        r_hi = min(int(np.floor((max(ay, by) - y0) / resolution_m)) + pad, rows - 1)
        cx = x0 + (np.arange(c_lo, c_hi + 1) + 0.5) * resolution_m
        cy = y0 + (np.arange(r_lo, r_hi + 1) + 0.5) * resolution_m
        px, py = np.meshgrid(cx, cy)
        ux, uy = (bx - ax) / seg_len[i], (by - ay) / seg_len[i]
        along = (px - ax) * ux + (py - ay) * uy
        lateral = ux * (py - ay) - uy * (px - ax)  # left positive
        s = s_at[i] + along
        inside = (
            (along >= 0.0)
            & (along <= seg_len[i])
            & (s >= start_s_m)
            & (s <= end_s)
            & (sign * lateral >= 0.0)
            & (sign * lateral <= reach)
        )
        window = out[r_lo : r_hi + 1, c_lo : c_hi + 1]
        window[inside] = 255.0
    return out
