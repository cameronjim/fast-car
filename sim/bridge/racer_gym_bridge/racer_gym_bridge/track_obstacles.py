"""Obstacles: solid boxes against a wall (brief 40, pre-registration v0.4 section 5).

An obstacle is a solid box against the inner or the outer wall of a loop: `at` is the share of
the centreline length from the start pose in the counter-clockwise sense, `side` says which wall,
`protrusion_m` how far the box reaches into the lane beyond the wall's lane-facing surface and
`length_m` its size along the lane. The box is drawn like the walls (segments of
``wall_thickness_m``, filled by hatching, and for a map into its occupancy grid), so the
simulator's ray casting, its collision check and the scorer's body clearance all see it. The
centreline is moved to the midline of the free lane: shifted by protrusion / 2 away from the box
over the box's length, with a cosine ramp of OBSTACLE_RAMP_M on each side. Nothing but boxes
against a wall: a free-standing object in the lane is not supported.

Pure numpy, like track_loader.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field, replace

import numpy as np

from .track_loader import SyntheticTrack, rasterize_walls

#: How far the box reaches behind the wall's lane-facing surface (m), so the curvature of the wall
#: never leaves a gap between box and wall (a wall thickness, more than the sagitta of a 0.4 m box
#: on a 0.4 m radius).
OBSTACLE_BACK_M = 0.08
#: Length over which the centreline returns to the lane's midline on each side of a box (m).
OBSTACLE_RAMP_M = 0.25
#: Pre-registration v0.4 section 5 (and brief 40): the start pose is never inside or within this
#: distance of an obstacle; a registered case that violates it is reported, not moved.
OBSTACLE_START_CLEARANCE_M = 0.5
OBSTACLE_SIDES = ("inner", "outer")
_OBSTACLE_KEYS = {"at", "side", "protrusion_m", "length_m"}
#: How far a map wall is searched for from the centreline (m), and the marching step (m).
_MAP_WALL_SEARCH_M = 2.0
_MAP_WALL_STEP_M = 0.01


class ObstacleError(ValueError):
    """An obstacle that cannot be drawn as specified. The caller reports it; nothing is moved."""


@dataclass(frozen=True)
class Obstacle:
    at: float
    side: str
    protrusion_m: float
    length_m: float
    # Where the box's centre stands (m), filled in by apply_obstacles; NaN before.
    centre_x: float = field(default=math.nan, compare=False)
    centre_y: float = field(default=math.nan, compare=False)


def parse_obstacles(spec: Sequence[Mapping] | None) -> tuple[Obstacle, ...]:
    """Obstacles from the `--track-params-json` list, validated."""
    out = []
    for item in spec or ():
        if set(item) != _OBSTACLE_KEYS:
            raise ObstacleError(f"obstacle keys must be exactly {sorted(_OBSTACLE_KEYS)}: {item}")
        ob = Obstacle(
            float(item["at"]),
            str(item["side"]),
            float(item["protrusion_m"]),
            float(item["length_m"]),
        )
        if not 0.0 <= ob.at < 1.0:
            raise ObstacleError(f"obstacle at {ob.at} is not a share in [0, 1)")
        if ob.side not in OBSTACLE_SIDES:
            raise ObstacleError(f"obstacle side must be one of {OBSTACLE_SIDES}, got {ob.side!r}")
        if ob.protrusion_m <= 0.0 or ob.length_m <= 0.0:
            raise ObstacleError("obstacle protrusion_m and length_m must be positive")
        out.append(ob)
    return tuple(out)


def loop_orientation(x: np.ndarray, y: np.ndarray) -> int:
    """+1 for a counter-clockwise closed polyline, -1 for clockwise (shoelace sign)."""
    area2 = float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))
    if area2 == 0.0:
        raise ObstacleError("the centreline encloses no area: no inner and outer side")
    return 1 if area2 > 0.0 else -1


def arc_positions(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, float]:
    """Arc length of every centreline point from the first one, and the loop length."""
    step = np.hypot(np.diff(np.r_[x, x[0]]), np.diff(np.r_[y, y[0]]))
    s = np.r_[0.0, np.cumsum(step[:-1])]
    return s, float(step.sum())


def _centre_frame(
    x: np.ndarray, y: np.ndarray, s: np.ndarray, length: float, at_s: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Centre point, unit tangent and left unit normal of the loop at arc position at_s."""
    sx, sy, ss = np.r_[x, x[0]], np.r_[y, y[0]], np.r_[s, length]

    def point(q: float) -> np.ndarray:
        q = q % length
        return np.array([np.interp(q, ss, sx), np.interp(q, ss, sy)])

    centre = point(at_s)
    tangent = point(at_s + 0.05) - point(at_s - 0.05)
    tangent = tangent / np.hypot(*tangent)
    return centre, tangent, np.array([-tangent[1], tangent[0]])


def _wrapped(d: np.ndarray, length: float) -> np.ndarray:
    return (d + length / 2.0) % length - length / 2.0


def _ramp(distance: np.ndarray, half_length: float) -> np.ndarray:
    """1 within half_length, a cosine ramp to 0 over OBSTACLE_RAMP_M beyond it, 0 after."""
    t = np.clip((distance - half_length) / OBSTACLE_RAMP_M, 0.0, 1.0)
    return 0.5 * (1.0 + np.cos(np.pi * t))


def _distance_to_footprint(
    point: np.ndarray,
    centre: np.ndarray,
    tangent: np.ndarray,
    normal: np.ndarray,
    half_length: float,
    lateral_lo: float,
    lateral_hi: float,
) -> float:
    """Distance from a point to the box's footprint (0 inside); the footprint spans along
    +-half_length and the signed lateral range [lateral_lo, lateral_hi] in the (tangent, left
    normal) frame at the box's centre."""
    a = float((point - centre) @ tangent)
    lat = float((point - centre) @ normal)
    da = max(abs(a) - half_length, 0.0)
    dl = max(lateral_lo - lat, lat - lateral_hi, 0.0)
    return math.hypot(da, dl)


def _map_wall_distance(
    track: SyntheticTrack, centre: np.ndarray, direction: np.ndarray
) -> float | None:
    """Distance from a point along a unit direction to the first occupied cell of the track's
    occupancy grid, or None when there is none within _MAP_WALL_SEARCH_M."""
    ox, oy, _ = track.occupancy_origin
    res = track.occupancy_resolution_m
    rows, cols = track.occupancy.shape
    for d in np.arange(0.0, _MAP_WALL_SEARCH_M, _MAP_WALL_STEP_M):
        p = centre + d * direction
        col = math.floor((p[0] - ox) / res)
        row = math.floor((p[1] - oy) / res)
        if not (0 <= row < rows and 0 <= col < cols):
            return None
        if track.occupancy[row, col] < 128.0:
            return float(d)
    return None


def _hatch_segments(
    centre: np.ndarray,
    tangent: np.ndarray,
    normal: np.ndarray,
    half_length: float,
    lateral_lo: float,
    lateral_hi: float,
    thickness_m: float,
) -> np.ndarray:
    """Wall segments that fill a footprint when rasterised with ``thickness_m``: the footprint
    inset by half a thickness, covered by lines along the lane at half-thickness spacing, closed
    off at both ends."""
    inset = thickness_m / 2.0
    a0, a1 = -(half_length - inset), half_length - inset
    laterals = np.arange(lateral_lo + inset, lateral_hi - inset + 1e-9, inset)
    if laterals.size == 0 or a1 < a0:
        raise ObstacleError("an obstacle smaller than a wall thickness cannot be drawn")
    segs = []
    for lat in laterals:
        p0 = centre + a0 * tangent + lat * normal
        p1 = centre + a1 * tangent + lat * normal
        segs.append([p0[0], p0[1], p1[0], p1[1]])
    for a in (a0, a1):
        p0 = centre + a * tangent + laterals[0] * normal
        p1 = centre + a * tangent + laterals[-1] * normal
        segs.append([p0[0], p0[1], p1[0], p1[1]])
    return np.asarray(segs, dtype=float)


def apply_obstacles(track: SyntheticTrack, spec: Sequence[Mapping]) -> SyntheticTrack:
    """The track with solid boxes against its walls and its centreline moved to the midline of
    the free lane (module docstring). The track is the counter-clockwise one, before
    ``reversed()``. Raises ObstacleError, and changes nothing, when a box cannot be drawn, would
    block the lane, overlaps another one (their ramps may not touch) or comes within
    OBSTACLE_START_CLEARANCE_M of the start pose."""
    obstacles = parse_obstacles(spec)
    if not obstacles:
        return track
    if not track.loop:
        raise ObstacleError("obstacles are for loops")
    if track.obstacles:
        raise ObstacleError("the track already has obstacles")
    x, y = track.centreline_x, track.centreline_y
    s, length = arc_positions(x, y)
    orient = loop_orientation(x, y)
    from_map = track.occupancy is not None
    walls = [track.walls]
    occupancy = None if track.occupancy is None else track.occupancy.copy()
    new_cells: list[np.ndarray] = []
    shift = np.zeros(len(x))
    dx = np.roll(x, -1) - np.roll(x, 1)
    dy = np.roll(y, -1) - np.roll(y, 1)
    norm = np.hypot(dx, dy)
    nx, ny = -dy / norm, dx / norm
    spans = []
    placed = []
    start_points = [np.array([x[0], y[0]]), np.array([x[-1], y[-1]])]
    for k, ob in enumerate(obstacles):
        at_s = ob.at * length
        sigma = float(orient if ob.side == "inner" else -orient)  # +1: left of travel
        centre, tangent, normal = _centre_frame(x, y, s, length, at_s)
        half = ob.length_m / 2.0
        if from_map:
            d_wall = _map_wall_distance(track, centre, sigma * normal)
            d_other = _map_wall_distance(track, centre, -sigma * normal)
            if d_wall is None or d_other is None:
                raise ObstacleError(
                    f"obstacle {k}: no wall within {_MAP_WALL_SEARCH_M} m of the centreline at "
                    f"{ob.at} of the loop, cannot draw it against the map"
                )
            surface = d_wall
            free = d_wall + d_other
        else:
            if track.lane_width_m <= 0.0:
                raise ObstacleError("a generated track needs a lane width")
            surface = track.lane_width_m / 2.0 - track.wall_thickness_m / 2.0
            free = 2.0 * surface
        if ob.protrusion_m >= free:
            raise ObstacleError(f"obstacle {k}: protrusion {ob.protrusion_m} m blocks the lane")
        lo, hi = sorted((sigma * (surface - ob.protrusion_m), sigma * (surface + OBSTACLE_BACK_M)))
        for sp in start_points:
            near = _distance_to_footprint(sp, centre, tangent, normal, half, lo, hi)
            if near < OBSTACLE_START_CLEARANCE_M:
                raise ObstacleError(
                    f"obstacle {k} (at {ob.at}) is {near:.2f} m from the start pose, "
                    f"under {OBSTACLE_START_CLEARANCE_M} m"
                )
        spans.append((at_s, half + OBSTACLE_RAMP_M))
        mid = centre + sigma * (surface - ob.protrusion_m / 2.0) * normal
        placed.append(replace(ob, centre_x=float(mid[0]), centre_y=float(mid[1])))
        shift -= sigma * (ob.protrusion_m / 2.0) * _ramp(np.abs(_wrapped(s - at_s, length)), half)
        if from_map:
            ox, oy, _ = track.occupancy_origin
            res = track.occupancy_resolution_m
            reach = half + OBSTACLE_BACK_M + surface + 1.0
            c_lo = max(int((centre[0] - reach - ox) / res), 0)
            c_hi = min(int((centre[0] + reach - ox) / res) + 1, occupancy.shape[1])
            r_lo = max(int((centre[1] - reach - oy) / res), 0)
            r_hi = min(int((centre[1] + reach - oy) / res) + 1, occupancy.shape[0])
            px, py = np.meshgrid(
                ox + (np.arange(c_lo, c_hi) + 0.5) * res, oy + (np.arange(r_lo, r_hi) + 0.5) * res
            )
            a = (px - centre[0]) * tangent[0] + (py - centre[1]) * tangent[1]
            lat = (px - centre[0]) * normal[0] + (py - centre[1]) * normal[1]
            inside = (np.abs(a) <= half) & (lat >= lo) & (lat <= hi)
            window = occupancy[r_lo:r_hi, c_lo:c_hi]
            fresh = inside & (window > 128.0)
            window[inside] = 0.0
            new_cells.append(np.column_stack([px[fresh], py[fresh]]))
        else:
            walls.append(
                _hatch_segments(centre, tangent, normal, half, lo, hi, track.wall_thickness_m)
            )
    for i in range(len(spans)):
        for j in range(i + 1, len(spans)):
            gap = abs(float(_wrapped(np.array(spans[i][0] - spans[j][0]), length)))
            if gap < spans[i][1] + spans[j][1]:
                raise ObstacleError(f"obstacles {i} and {j} are too close to each other")
    if from_map and new_cells:
        cells = np.vstack(new_cells)
        walls.append(np.column_stack([cells[:, 0], cells[:, 1], cells[:, 0], cells[:, 1]]))
    return replace(
        track,
        centreline_x=x + shift * nx,
        centreline_y=y + shift * ny,
        walls=np.vstack(walls),
        occupancy=occupancy,
        obstacles=tuple(placed),
    )


def ground_truth_grid(track: SyntheticTrack) -> tuple[np.ndarray, tuple[float, float], float]:
    """The occupancy grid the simulator ray casts and collides against for this track, as
    (grid with 255 free and 0 wall and row 0 at min y, (origin x, origin y), resolution): the
    imported map's own grid, or the walls rasterised exactly as ``bridge_node`` does."""
    if track.occupancy is not None:
        ox, oy, _ = track.occupancy_origin
        return track.occupancy, (ox, oy), track.occupancy_resolution_m
    grid, origin = rasterize_walls(track.walls, track.bounds(), track.occupancy_resolution_m)
    return grid, (origin[0], origin[1]), track.occupancy_resolution_m
