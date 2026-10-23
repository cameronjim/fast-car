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
import math
from collections.abc import Callable
from dataclasses import dataclass
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


# -- Synthetic benchmark tracks (reactive bench, 2026-10-23) ---------------------------------------
#
# Parametric tracks described as WALL SEGMENTS, rasterised into the occupancy grid the gym's
# LaserScan caster reads (``rasterize_walls``). Unlike ``build_corridor_occupancy`` (a solid
# block with a corridor cut out of it) the walls here are thin, so a wall can be missing (the open
# side of ``lane_open_side``) and there is real free floor behind it. Every geometry is a pure
# function of its parameters; ``build_synthetic_track`` is the single entry point.

SYNTHETIC_TRACK_NAMES = ("lane_square", "lane_round", "lane_open_side", "chicane", "open_room")

# The loop the lane tracks run on: a rectangle, centreline dimensions (m). The same 5.5 x 4.0 for
# every lane track, so the infield behind lane_open_side's boxes is 4.4 x 2.9 m of open floor.
LOOP_WIDTH_M = 5.5
LOOP_HEIGHT_M = 4.0
# Centreline radius of the corners of the lane tracks whose subject is not the corner.
EASY_CORNER_RADIUS_M = 0.8
# lane_round's default centreline corner radius (the round escape canary's).
ROUND_CORNER_RADIUS_M = 0.25
DEFAULT_LANE_WIDTH_M = 1.1
DEFAULT_BOX_LENGTH_M = 0.4
DEFAULT_BOX_GAP_M = 0.6
# How far a lane_open_side box sticks out into the open floor (across the lane).
BOX_DEPTH_M = 0.3
DEFAULT_JOG_OFFSET_M = 0.5
# Along-track length of one chicane jog: the walls move sideways over this distance.
DEFAULT_JOG_LENGTH_M = 1.0
CHICANE_PLATEAU_M = 0.8
DEFAULT_ROOM_SIZE_M = 4.0
ROOM_BOX_SIZE_M = 0.5
# Wall thickness once rasterised (m): more than one 0.05 m cell whatever the wall's offset from
# the cell grid, so a ray cannot slip between two diagonal cells.
WALL_THICKNESS_M = 0.08
_DS_M = 0.05


@dataclass(frozen=True)
class SyntheticTrack:
    """A synthetic track: the reference line the car starts on and the walls around it.

    ``centreline_x/y`` is in driving direction; for a ``loop`` it closes from the last point back
    to the first, and the car starts on the first point facing the second. ``walls`` is an
    (N, 4) array of segments (x0, y0, x1, y1), metres.
    """

    name: str
    centreline_x: np.ndarray
    centreline_y: np.ndarray
    walls: np.ndarray
    loop: bool
    lane_width_m: float

    def bounds(self) -> tuple[float, float, float, float]:
        """(x_min, y_min, x_max, y_max) of the walls."""
        xs = self.walls[:, [0, 2]]
        ys = self.walls[:, [1, 3]]
        return float(xs.min()), float(ys.min()), float(xs.max()), float(ys.max())

    def reversed(self) -> SyntheticTrack:
        """The same walls driven the other way (start on the last point, facing back)."""
        return SyntheticTrack(
            self.name,
            self.centreline_x[::-1].copy(),
            self.centreline_y[::-1].copy(),
            self.walls,
            self.loop,
            self.lane_width_m,
        )


def _line_points(
    start: tuple[float, float], end: tuple[float, float], ds_m: float = _DS_M
) -> list[tuple[float, float]]:
    """Points from ``start`` towards ``end``, about ds_m apart, ``end`` itself not included."""
    (x0, y0), (x1, y1) = start, end
    n = max(1, round(math.hypot(x1 - x0, y1 - y0) / ds_m))
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n)]


def _arc_points(
    centre: tuple[float, float], r: float, a0: float, a1: float, ds_m: float = _DS_M
) -> list[tuple[float, float]]:
    n = max(2, round(r * abs(a1 - a0) / ds_m))
    cx, cy = centre
    return [
        (cx + r * math.cos(a0 + (a1 - a0) * i / n), cy + r * math.sin(a0 + (a1 - a0) * i / n))
        for i in range(n)
    ]


def rounded_rect_points(
    x_min: float,
    y_min: float,
    x_max: float,
    y_max: float,
    radius_m: float,
    bottom_dy: Callable[[float], float] | None = None,
    ds_m: float = _DS_M,
) -> list[tuple[float, float]]:
    """A counter-clockwise rounded rectangle as points about ``ds_m`` apart (the end point is
    not repeated), starting where the bottom straight begins, (x_min + radius, y_min), and ending
    just before it. ``radius_m`` 0 gives square corners. ``bottom_dy``, when given, moves the
    points of the bottom straight sideways (in y) by ``bottom_dy(x)``; it must be 0 at both ends
    of the straight so the loop still closes."""
    r = radius_m
    if r < 0.0 or 2.0 * r > min(x_max - x_min, y_max - y_min) + 1e-9:
        raise ValueError(
            f"corner radius {r} m does not fit a {x_max - x_min} x {y_max - y_min} rect"
        )
    lines = [
        ((x_min + r, y_min), (x_max - r, y_min)),
        ((x_max, y_min + r), (x_max, y_max - r)),
        ((x_max - r, y_max), (x_min + r, y_max)),
        ((x_min, y_max - r), (x_min, y_min + r)),
    ]
    arcs = [
        ((x_max - r, y_min + r), -math.pi / 2.0, 0.0),
        ((x_max - r, y_max - r), 0.0, math.pi / 2.0),
        ((x_min + r, y_max - r), math.pi / 2.0, math.pi),
        ((x_min + r, y_min + r), math.pi, 1.5 * math.pi),
    ]
    points: list[tuple[float, float]] = []
    for i in range(4):
        line = _line_points(*lines[i], ds_m)
        if i == 0 and bottom_dy is not None:
            line = [(x, y + bottom_dy(x)) for x, y in line]
        points.extend(line)
        if r > 0.0:
            centre, a0, a1 = arcs[i]
            points.extend(_arc_points(centre, r, a0, a1, ds_m))
    return points


def _segments(points: list[tuple[float, float]], closed: bool) -> np.ndarray:
    p = np.asarray(points, dtype=float)
    q = np.roll(p, -1, axis=0)
    if not closed:
        p, q = p[:-1], q[:-1]
    return np.hstack([p, q])


def _box_segments(x0: float, y0: float, x1: float, y1: float) -> np.ndarray:
    return _segments([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], closed=True)


def _chicane_profile(
    straight_start: float, straight_end: float, offset_m: float, jog_length_m: float
) -> Callable[[float], float]:
    """Sideways displacement of the bottom straight: up by ``offset_m`` over one jog, a plateau,
    back down over a second."""
    length = straight_end - straight_start
    lead = (length - 2.0 * jog_length_m - CHICANE_PLATEAU_M) / 2.0
    if lead < 0.0 or jog_length_m <= 0.0:
        raise ValueError(f"two {jog_length_m} m jogs do not fit a {length:.2f} m straight")
    up = straight_start + lead
    down = up + jog_length_m + CHICANE_PLATEAU_M

    def dy(x: float) -> float:
        rise = min(max((x - up) / jog_length_m, 0.0), 1.0)
        fall = min(max((x - down) / jog_length_m, 0.0), 1.0)
        return offset_m * (rise - fall)

    return dy


def _boxes_along(
    x_start: float, x_end: float, wall_y: float, length_m: float, gap_m: float
) -> np.ndarray:
    """As many boxes ``length_m`` long, ``gap_m`` apart, as fit between a ``gap_m`` margin at each
    end of [x_start, x_end], centred; their lane-facing side on y = wall_y, ``BOX_DEPTH_M`` deep,
    as outlines."""
    if length_m <= 0.0 or gap_m <= 0.0:
        raise ValueError("box length and gap must be positive")
    pitch = length_m + gap_m
    count = int((x_end - x_start - gap_m) // pitch)
    if count < 1:
        raise ValueError("no box fits the straight")
    span = count * length_m + (count - 1) * gap_m
    lead = x_start + (x_end - x_start - span) / 2.0
    return np.vstack(
        [
            _box_segments(
                lead + i * pitch, wall_y, lead + i * pitch + length_m, wall_y + BOX_DEPTH_M
            )
            for i in range(count)
        ]
    )


def _open_room(size_m: float) -> SyntheticTrack:
    if size_m < 3.0:
        raise ValueError(f"room_size_m must be at least 3 m, got {size_m}")
    s = size_m
    walls = [_box_segments(0.0, 0.0, s, s)]
    # Three boxes at fixed fractions of the room, clear of the start line (y = 0.2 s).
    for fx, fy in ((0.4, 0.35), (0.7, 0.65), (0.25, 0.7)):
        walls.append(
            _box_segments(fx * s, fy * s, fx * s + ROOM_BOX_SIZE_M, fy * s + ROOM_BOX_SIZE_M)
        )
    n = max(int(0.8 * s / _DS_M), 2)
    cx = np.linspace(0.15 * s, 0.95 * s - 0.2, n)
    cy = np.full(n, 0.2 * s)
    return SyntheticTrack("open_room", cx, cy, np.vstack(walls), False, 0.0)


def build_synthetic_track(
    name: str,
    lane_width_m: float = DEFAULT_LANE_WIDTH_M,
    corner_radius_m: float | None = None,
    box_length_m: float = DEFAULT_BOX_LENGTH_M,
    box_gap_m: float = DEFAULT_BOX_GAP_M,
    jog_offset_m: float = DEFAULT_JOG_OFFSET_M,
    jog_length_m: float = DEFAULT_JOG_LENGTH_M,
    room_size_m: float = DEFAULT_ROOM_SIZE_M,
) -> SyntheticTrack:
    """One of ``SYNTHETIC_TRACK_NAMES`` as a ``SyntheticTrack`` (the reactive bench's tracks).

    The four lane tracks share a counter-clockwise rectangular loop (``LOOP_WIDTH_M`` x
    ``LOOP_HEIGHT_M`` centreline) and a lane ``lane_width_m`` wide; the car starts mid bottom
    straight facing +x. ``corner_radius_m`` is the CENTRELINE corner radius where the track has
    corners to tune (None = the track's own default).

      * ``lane_square``: four square corners (centreline radius 0). The walls are the
        centreline's offsets, so each inside corner is square and the outside wall an arc of
        radius w/2: the geometry of the L5 escape canary, where the car ends nose-in as it did
        on the floor. (Square OUTSIDE walls leave a 1.56 m diagonal pocket the car drives round
        in one sweep, which is not the failure.)
      * ``lane_round``: the corners rounded to ``corner_radius_m`` (default 0.25 m): outside
        wall radius r + w/2, inside wall radius max(r - w/2, 0).
      * ``lane_open_side``: easy corners (0.8 m) and, along the bottom straight, the LEFT (inner)
        wall replaced by boxes ``box_length_m`` long with ``box_gap_m`` gaps, and the open floor
        of the infield behind them: the 2026-10-06 floor failure, where the follower turned into
        the gaps.
      * ``chicane``: easy corners and, on the bottom straight, two jogs of ``jog_offset_m``
        (first to the left, then back), each taking ``jog_length_m`` of track.
      * ``open_room``: not a loop. A ``room_size_m`` square room with three boxes; the car
        starts at the bottom left facing +x.
    """
    if name not in SYNTHETIC_TRACK_NAMES:
        raise ValueError(f"synthetic track must be one of {SYNTHETIC_TRACK_NAMES}, got {name!r}")
    if name == "open_room":
        return _open_room(room_size_m)
    half = lane_width_m / 2.0
    w, h = LOOP_WIDTH_M, LOOP_HEIGHT_M
    if lane_width_m <= 0.0 or lane_width_m >= min(w, h):
        raise ValueError(f"lane width {lane_width_m} m does not fit the {w} x {h} m loop")
    default_radius = {
        "lane_square": 0.0,
        "lane_round": ROUND_CORNER_RADIUS_M,
    }.get(name, EASY_CORNER_RADIUS_M)
    r = default_radius if corner_radius_m is None else corner_radius_m
    bottom_dy = (
        _chicane_profile(r, w - r, jog_offset_m, jog_length_m) if name == "chicane" else None
    )

    # Centreline, and the walls it grows and shrinks to: corner radii r + w/2 outside and
    # max(r - w/2, 0) inside (offsetting a rounded centreline).
    outer_r = r + half
    inner_r = max(r - half, 0.0)
    centre = rounded_rect_points(0.0, 0.0, w, h, r, bottom_dy)
    outer = rounded_rect_points(-half, -half, w + half, h + half, outer_r, bottom_dy)
    inner = rounded_rect_points(half, half, w - half, h - half, inner_r, bottom_dy)

    # Start mid bottom straight: roll the centreline to the straight's point nearest x = w / 2.
    n_bottom = len(_line_points((r, 0.0), (w - r, 0.0)))
    start = int(np.argmin([abs(x - w / 2.0) for x, _ in centre[:n_bottom]]))
    centre = centre[start:] + centre[:start]
    cx, cy = (np.asarray(v, dtype=float) for v in zip(*centre, strict=True))

    walls = [_segments(outer, closed=True)]
    if name == "lane_open_side":
        # The inner wall minus its bottom straight: an open polyline from the straight's far end
        # round the loop to its near end (the points start with the straight and the first point
        # after it is its far end). The boxes stand where the straight was.
        n_inner = len(_line_points((half + inner_r, half), (w - half - inner_r, half)))
        walls.append(_segments(inner[n_inner:] + [inner[0]], closed=False))
        walls.append(
            _boxes_along(half + inner_r, w - half - inner_r, half, box_length_m, box_gap_m)
        )
    else:
        walls.append(_segments(inner, closed=True))
    return SyntheticTrack(name, cx, cy, np.vstack(walls), True, lane_width_m)


def rasterize_walls(
    walls: np.ndarray,
    bounds: tuple[float, float, float, float],
    resolution_m: float,
    border_m: float = 1.0,
    thickness_m: float = WALL_THICKNESS_M,
) -> tuple[np.ndarray, tuple[float, float, float]]:
    """Wall segments (N, 4) as an occupancy grid in f1tenth_gym / map_server layout (see
    ``build_corridor_occupancy``): wall cells (centre within ``thickness_m / 2`` of a segment)
    are 0, everything else 255. ``bounds`` is (x_min, y_min, x_max, y_max); ``border_m`` of
    free floor is kept around it. Returns ``(occupancy_map, origin)``."""
    if resolution_m <= 0.0 or thickness_m <= 0.0:
        raise ValueError("resolution_m and thickness_m must be positive")
    x_min, y_min, x_max, y_max = bounds
    x0, y0 = x_min - border_m, y_min - border_m
    cols = int(np.ceil((x_max + border_m - x0) / resolution_m))
    rows = int(np.ceil((y_max + border_m - y0) / resolution_m))
    occupancy = np.full((rows, cols), 255.0, dtype=np.float32)
    reach = thickness_m / 2.0 + resolution_m
    for ax, ay, bx, by in np.asarray(walls, dtype=float):
        c_lo = max(int(np.floor((min(ax, bx) - reach - x0) / resolution_m)), 0)
        c_hi = min(int(np.ceil((max(ax, bx) + reach - x0) / resolution_m)), cols - 1)
        r_lo = max(int(np.floor((min(ay, by) - reach - y0) / resolution_m)), 0)
        r_hi = min(int(np.ceil((max(ay, by) + reach - y0) / resolution_m)), rows - 1)
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
        window = occupancy[r_lo : r_hi + 1, c_lo : c_hi + 1]
        window[d <= thickness_m / 2.0] = 0.0
    return occupancy, (x0, y0, 0.0)
