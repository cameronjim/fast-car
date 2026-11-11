"""Patrol waypoints to a raceline the tracker can follow (roadmap 5.3). Pure stdlib, no ROS.

A patrol file is YAML:

    name: living-room-loop          # optional
    frame: map                      # required, must be "map" (the AMCL map frame)
    loop: true                      # required, must be true: the route is a CLOSED loop
    speed_mps: 0.6                  # optional, overridden by the launch argument
    waypoints:                      # at least 3, metres in the map frame
      - [0.5, 0.0]
      - [3.0, 0.2]
      ...

The loop is closed from the last waypoint back to the first. `build_raceline` threads a smooth
centripetal Catmull-Rom curve through the waypoints, resamples it every `ds_m` metres and writes
heading, curvature and a constant target speed, the six columns of tools/raceline's CSV, which
racer_control's tracker_node loads (docs/notes/localization-2026-11-03.md: the tracker runs on
the AMCL pose with pose_frame:=map). The spiderbot reuses that tracker instead of a second
path follower; this module is only the waypoints-to-raceline step.

REFUSALS. A route the car cannot drive is refused here, not discovered on the floor:
fewer than 3 waypoints, non-finite values, waypoints closer than `min_spacing_m`, and any point
where the curve's curvature exceeds `max_curvature_1pm` (the caller passes the steering limit's
tightest curve, from vehicle_params, with a margin). A refusal names the waypoint.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

RACELINE_HEADER = ("s_m", "x_m", "y_m", "heading_rad", "curvature_1pm", "target_speed_mps")


class PatrolError(ValueError):
    pass


@dataclass(frozen=True)
class Patrol:
    name: str
    waypoints: tuple[tuple[float, float], ...]
    speed_mps: float | None


def parse_patrol(data: dict) -> Patrol:
    """Validate a parsed patrol document."""
    if not isinstance(data, dict):
        raise PatrolError("a patrol file must be a mapping")
    if data.get("frame") != "map":
        raise PatrolError("frame must be 'map': waypoints are in the AMCL map frame")
    if data.get("loop") is not True:
        raise PatrolError("loop must be true: only closed-loop patrols are supported")
    raw = data.get("waypoints")
    if not isinstance(raw, list) or len(raw) < 3:
        raise PatrolError("waypoints must be a list of at least 3 [x, y] points")
    points: list[tuple[float, float]] = []
    for i, wp in enumerate(raw):
        if (
            not isinstance(wp, (list, tuple))
            or len(wp) != 2
            or not all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in wp)
            or not all(math.isfinite(v) for v in wp)
        ):
            raise PatrolError(f"waypoint {i} must be two finite numbers [x, y], got {wp!r}")
        points.append((float(wp[0]), float(wp[1])))
    speed = data.get("speed_mps")
    if speed is not None and not (
        isinstance(speed, (int, float)) and not isinstance(speed, bool) and 0.0 < speed < 100.0
    ):
        raise PatrolError(f"speed_mps must be a positive number, got {speed!r}")
    return Patrol(
        str(data.get("name", "patrol")), tuple(points), None if speed is None else float(speed)
    )


def load_patrol(path: str | Path) -> Patrol:
    import yaml

    p = Path(path)
    if not p.is_file():
        raise PatrolError(f"patrol file not found: {p}")
    try:
        return parse_patrol(yaml.safe_load(p.read_text(encoding="utf-8")))
    except yaml.YAMLError as exc:
        raise PatrolError(f"{p} is not valid YAML: {exc}") from exc


def _catmull_rom(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    samples: int,
) -> list[tuple[float, float]]:
    """Centripetal Catmull-Rom from p1 to p2 (alpha 0.5), `samples` points, p2 excluded."""

    def knot(a, b):
        return max(math.hypot(b[0] - a[0], b[1] - a[1]) ** 0.5, 1e-6)

    t0 = 0.0
    t1 = t0 + knot(p0, p1)
    t2 = t1 + knot(p1, p2)
    t3 = t2 + knot(p2, p3)
    out = []
    for i in range(samples):
        t = t1 + (t2 - t1) * i / samples
        pt = []
        for d in (0, 1):
            a1 = p0[d] * (t1 - t) / (t1 - t0) + p1[d] * (t - t0) / (t1 - t0)
            a2 = p1[d] * (t2 - t) / (t2 - t1) + p2[d] * (t - t1) / (t2 - t1)
            a3 = p2[d] * (t3 - t) / (t3 - t2) + p3[d] * (t - t2) / (t3 - t2)
            b1 = a1 * (t2 - t) / (t2 - t0) + a2 * (t - t0) / (t2 - t0)
            b2 = a2 * (t3 - t) / (t3 - t1) + a3 * (t - t1) / (t3 - t1)
            pt.append(b1 * (t2 - t) / (t2 - t1) + b2 * (t - t1) / (t2 - t1))
        out.append((pt[0], pt[1]))
    return out


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class RacelinePoint:
    s_m: float
    x_m: float
    y_m: float
    heading_rad: float
    curvature_1pm: float
    target_speed_mps: float


def build_raceline(
    waypoints: Sequence[tuple[float, float]],
    speed_mps: float,
    max_curvature_1pm: float,
    ds_m: float = 0.1,
    min_spacing_m: float = 0.2,
) -> list[RacelinePoint]:
    """Smooth closed loop through `waypoints`, resampled every `ds_m`, at constant speed."""
    n = len(waypoints)
    if n < 3:
        raise PatrolError("need at least 3 waypoints")
    if not (math.isfinite(speed_mps) and speed_mps > 0.0):
        raise PatrolError(f"speed must be positive, got {speed_mps}")
    if not (math.isfinite(max_curvature_1pm) and max_curvature_1pm > 0.0):
        raise PatrolError(f"max_curvature_1pm must be positive, got {max_curvature_1pm}")
    for i in range(n):
        a, b = waypoints[i], waypoints[(i + 1) % n]
        gap = math.hypot(b[0] - a[0], b[1] - a[1])
        if gap < min_spacing_m:
            raise PatrolError(
                f"waypoints {i} and {(i + 1) % n} are {gap:.3f} m apart, closer than {min_spacing_m} m"
            )
    dense: list[tuple[float, float]] = []
    for i in range(n):
        seg = _catmull_rom(
            waypoints[(i - 1) % n], waypoints[i], waypoints[(i + 1) % n], waypoints[(i + 2) % n], 64
        )
        dense.extend(seg)
    # Cumulative arc length around the closed dense polyline.
    cum = [0.0]
    for i in range(len(dense)):
        a, b = dense[i], dense[(i + 1) % len(dense)]
        cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    total = cum[-1]
    count = max(round(total / ds_m), 8)
    step = total / count
    pts: list[tuple[float, float]] = []
    j = 0
    for k in range(count):
        s = k * step
        while cum[j + 1] < s:
            j += 1
        a, b = dense[j % len(dense)], dense[(j + 1) % len(dense)]
        span = cum[j + 1] - cum[j]
        f = 0.0 if span <= 0.0 else (s - cum[j]) / span
        pts.append((a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f))
    headings = []
    for k in range(count):
        a, b = pts[(k - 1) % count], pts[(k + 1) % count]
        headings.append(math.atan2(b[1] - a[1], b[0] - a[0]))
    out: list[RacelinePoint] = []
    for k in range(count):
        kappa = _wrap(headings[(k + 1) % count] - headings[(k - 1) % count]) / (2.0 * step)
        if abs(kappa) > max_curvature_1pm:
            nearest = min(
                range(n),
                key=lambda i: math.hypot(pts[k][0] - waypoints[i][0], pts[k][1] - waypoints[i][1]),
            )
            raise PatrolError(
                f"the route turns tighter than the car can ({abs(kappa):.2f} 1/m, limit "
                f"{max_curvature_1pm:.2f}) near waypoint {nearest} at "
                f"({pts[k][0]:.2f}, {pts[k][1]:.2f}): move waypoints apart or round the corner"
            )
        out.append(RacelinePoint(k * step, pts[k][0], pts[k][1], headings[k], kappa, speed_mps))
    return out


def max_curvature_from_vehicle(
    wheelbase_m: float, max_steering_rad: float, margin_fraction: float = 0.75
) -> float:
    """Tightest curvature a patrol may ask for: `margin_fraction` of the car's own limit.

    The kinematic limit is tan(steering) / wheelbase; both come from vehicle_params (the caller
    reads the binding). A margin below 1 leaves the tracker room to steer without saturating.
    """
    if not (math.isfinite(wheelbase_m) and wheelbase_m > 0.0):
        raise PatrolError(f"wheelbase must be positive, got {wheelbase_m}")
    if not (math.isfinite(max_steering_rad) and 0.0 < max_steering_rad < math.pi / 2):
        raise PatrolError(f"max steering must be in (0, pi/2), got {max_steering_rad}")
    if not 0.0 < margin_fraction <= 1.0:
        raise PatrolError(f"margin_fraction must be in (0, 1], got {margin_fraction}")
    return margin_fraction * math.tan(max_steering_rad) / wheelbase_m


def write_raceline_csv(path: str | Path, points: Sequence[RacelinePoint], name: str) -> None:
    lines = [
        f"# patrol raceline for {name} (racer_apps spiderbot, generated from waypoints)",
        "# columns: " + ",".join(RACELINE_HEADER),
        ",".join(RACELINE_HEADER),
    ]
    for p in points:
        lines.append(
            f"{p.s_m:.6f},{p.x_m:.6f},{p.y_m:.6f},{p.heading_rad:.6f},"
            f"{p.curvature_1pm:.6f},{p.target_speed_mps:.6f}"
        )
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")
