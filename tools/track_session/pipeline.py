"""The track-session pipeline as functions: map -> lane report -> raceline CSV.

`track_session.py` (the CLI) and the sim bench (tests/l5_reactive_bench) both call these, so what
the bench drives on is made the way the car's session makes it.

Physical constants, steering limits included, come from the generated vehicle_params binding
(`raceline.params_loader`).
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

_TOOLS = Path(__file__).resolve().parents[1]
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

from raceline import map_track as mt
from raceline.io import write_raceline_csv
from raceline.params_loader import load_vehicle_params
from raceline.raceline import build_raceline_from_centerline


def steering_limits() -> tuple[float, float]:
    """The planning limits on the true road-wheel angle, (left, right), both positive, from
    vehicle_params (schema 0.14.0: the symmetric, right-hand 0.449 rad; the left stop's 0.504 rad is
    in the pulse table, not in what planners may ask for)."""
    steering = load_vehicle_params().steering
    return float(steering.max_angle_rad), float(-steering.min_angle_rad)


def car_geometry(front_overhang_m: float | None = None, rear_overhang_m: float | None = None):
    """The body about the rear axle from vehicle_params (overhangs overridable for what-ifs)."""
    chassis = load_vehicle_params().chassis
    return mt.CarGeometry(
        wheelbase_m=chassis.wheelbase_m,
        front_overhang_m=chassis.front_overhang_m if front_overhang_m is None else front_overhang_m,
        rear_overhang_m=chassis.rear_overhang_m if rear_overhang_m is None else rear_overhang_m,
        width_m=chassis.width_m,
    )


def write_surface_map(
    occupancy: np.ndarray, origin_xy: tuple[float, float], resolution_m: float, prefix: Path
) -> Path:
    """A gym-layout occupancy grid (255 free, 0 wall, row 0 at min y) as a map_server pgm + yaml
    the way slam_toolbox draws a map: free cells, the wall cells that touch free floor, and
    unknown (205) everywhere else. AMCL's likelihood field needs thin walls (see
    docs/notes/localization-2026-11-03.md, finding 1). Returns the yaml path."""
    prefix = Path(prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    free = np.asarray(occupancy) > 127
    touches = np.zeros_like(free)
    touches[1:, :] |= free[:-1, :]
    touches[:-1, :] |= free[1:, :]
    touches[:, 1:] |= free[:, :-1]
    touches[:, :-1] |= free[:, 1:]
    pixels = np.where(free, 254, np.where(touches, 0, 205)).astype(np.uint8)
    image = np.flipud(pixels)
    pgm = prefix.with_suffix(".pgm")
    with pgm.open("wb") as f:
        f.write(f"P5\n{image.shape[1]} {image.shape[0]}\n255\n".encode())
        f.write(image.tobytes())
    yaml_path = prefix.with_suffix(".yaml")
    # Flow-style origin, as nav2's map_saver writes it (track_loader reads the file by hand).
    yaml_path.write_text(
        f"image: {pgm.name}\nmode: trinary\nresolution: {float(resolution_m)}\n"
        f"origin: [{float(origin_xy[0])}, {float(origin_xy[1])}, 0.0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.25\n"
    )
    return yaml_path


def write_centreline_csv(
    path: Path, xs: np.ndarray, ys: np.ndarray, speed_mps: float = 0.5
) -> None:
    """A raceline-format CSV of a plain centreline (the sim's map track reads x and y)."""
    result = build_raceline_from_centerline(
        xs,
        ys,
        a_lat_max_mps2=2.0,
        a_max_mps2=2.0,
        v_max_mps=speed_mps,
        resample_ds_m=0.05,
        smooth_window=1,
    )
    write_raceline_csv(
        path,
        result,
        track_id="centreline",
        vehicle_params_schema_version="n/a",
        vehicle_params_sysid_session_id="n/a",
        generation_params={"source": "map_track.trace_centreline"},
    )


@dataclass
class SessionLine:
    """What `raceline_from_map` produced."""

    csv: Path
    report: dict
    line: mt.LineResult
    centre_xy: tuple[np.ndarray, np.ndarray]


def raceline_from_map(
    map_yaml: Path,
    out_csv: Path,
    *,
    direction: str = "ccw",
    seed_xy: tuple[float, float] = (0.0, 0.0),
    margin: float = 0.85,
    body_clearance_m: float = 0.08,
    v_max_mps: float = 0.9,
    a_lat_mps2: float = 2.0,
    a_long_mps2: float = 1.0,
    clearance_candidates: tuple[float, ...] | None = None,
    front_overhang_m: float | None = None,
    rear_overhang_m: float | None = None,
    centre_only: bool = False,
    centre_csv: Path | None = None,
) -> SessionLine:
    """Map -> centreline -> racing line -> speed profile -> CSV.

    Speeds are TRUE speeds (``v_max_mps``, ``a_lat_mps2``, ``a_long_mps2``): since vehicle_params
    0.14.0 a speed command is a true speed.

    ``margin`` scales the curvature the steering can hold on each side (tan(lock) / wheelbase)
    so the controller keeps some steering to correct with; ``body_clearance_m`` is how far the
    whole body outline stays off the walls. ``centre_only`` writes the centreline itself with the
    speed profile (no offset): the plain-centreline baseline."""
    left_lock, right_lock = steering_limits()
    car = car_geometry(front_overhang_m, rear_overhang_m)
    grid = mt.load_map(map_yaml)
    cx, cy = mt.trace_centreline(grid, seed_xy)
    lane = mt.lane_report(grid, cx, cy)
    if centre_csv is not None:
        write_centreline_csv(centre_csv, cx, cy)
    if direction == "cw":
        cx, cy = cx[::-1].copy(), cy[::-1].copy()
    elif direction != "ccw":
        raise ValueError("direction must be ccw or cw")
    kl, kr = mt.curvature_limits(car.wheelbase_m, left_lock, right_lock, margin)
    if centre_only:
        line = mt.LineResult(cx, cy, np.zeros(len(cx)), 0.0, 0.0, 0.0, 0, 0, 0)
        px, py = cx, cy
    else:
        # Either the one clearance asked for, or the widest of the candidates the track allows.
        tries = list(clearance_candidates) if clearance_candidates else [body_clearance_m]
        for clearance in tries:
            line = mt.racing_line(
                grid,
                cx,
                cy,
                car,
                max_left_curvature_1pm=kl,
                max_right_curvature_1pm=kr,
                body_clearance_m=clearance,
            )
            body_clearance_m = clearance
            if line.curvature_violations == 0 and line.clearance_violations == 0:
                break
        px, py = line.x_m, line.y_m
    result = build_raceline_from_centerline(
        px,
        py,
        a_lat_max_mps2=a_lat_mps2,
        a_max_mps2=a_long_mps2,
        v_max_mps=v_max_mps,
        resample_ds_m=0.05,
        smooth_window=3,
    )
    vp = load_vehicle_params()
    write_raceline_csv(
        out_csv,
        result,
        track_id=Path(map_yaml).stem,
        vehicle_params_schema_version=vp.meta.schema_version,
        vehicle_params_sysid_session_id=vp.meta.sysid_session_id,
        generation_params={
            "source_map": Path(map_yaml).name,
            "direction": direction,
            "centre_only": str(centre_only),
            "left_lock_rad": left_lock,
            "right_lock_rad": right_lock,
            "curvature_margin": margin,
            "body_clearance_m": body_clearance_m,
            "v_max_mps": v_max_mps,
            "a_lat_mps2": a_lat_mps2,
            "a_long_mps2": a_long_mps2,
        },
    )
    # The CSV's curvature column is a 5 cm finite difference, noisy at a tight bend; the radius
    # worth quoting is the circle through points 15 cm either side along the line.
    kappa = _chord_curvature(result.x_m, result.y_m, 0.15)
    report = {
        "lane": lane.as_dict(),
        "direction": direction,
        "centre_only": centre_only,
        "curvature_limit_left_1pm": round(kl, 3),
        "curvature_limit_right_1pm": round(kr, 3),
        "line_max_left_curvature_1pm": round(float(max(kappa.max(), 0.0)), 3),
        "line_max_right_curvature_1pm": round(float(max(-kappa.min(), 0.0)), 3),
        "line_min_radius_left_m": round(1.0 / max(float(kappa.max()), 1e-9), 3),
        "line_min_radius_right_m": round(1.0 / max(float(-kappa.min()), 1e-9), 3),
        "min_body_clearance_m": round(line.min_body_clearance_m, 3),
        "curvature_violations": line.curvature_violations,
        "clearance_violations": line.clearance_violations,
        "rounds": line.iterations,
        "length_m": round(
            float(np.sum(np.hypot(np.diff(px, append=px[0]), np.diff(py, append=py[0])))), 3
        ),
        "body_clearance_used_m": body_clearance_m,
        "max_offset_from_centre_m": round(float(np.max(np.abs(line.offsets_m))), 3),
    }
    out_csv.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    return SessionLine(out_csv, report, line, (cx, cy))


def _chord_curvature(x: np.ndarray, y: np.ndarray, reach_m: float) -> np.ndarray:
    """Signed curvature of the circle through each point and the points ``reach_m`` of path
    behind and ahead of it (left positive), on a closed path."""
    seg = np.hypot(np.diff(x, append=x[0]), np.diff(y, append=y[0]))
    k = max(round(reach_m / max(float(seg.mean()), 1e-6)), 1)
    xa, ya = np.roll(x, k), np.roll(y, k)
    xb, yb = np.roll(x, -k), np.roll(y, -k)
    d1 = np.column_stack([x - xa, y - ya])
    d2 = np.column_stack([xb - x, yb - y])
    d3 = np.column_stack([xb - xa, yb - ya])
    cross = d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]
    norms = (
        np.hypot(d1[:, 0], d1[:, 1]) * np.hypot(d2[:, 0], d2[:, 1]) * np.hypot(d3[:, 0], d3[:, 1])
    )
    return 2.0 * cross / np.maximum(norms, 1e-9)


def feasible(report: dict) -> bool:
    return (
        report["curvature_violations"] == 0
        and report["clearance_violations"] == 0
        and not (math.isnan(report["min_body_clearance_m"]))
    )
