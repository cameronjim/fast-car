"""Python port of racer_control's pure-pursuit base controller (roadmap S.3).

claude-docs/08-learning.md: "Base controller = the tuned classical stack (raceline optimizer
+ tracker from racer_control)." The training env needs that same base controller INSIDE the
sim loop, in Python, so this module ports `ros_ws/src/racer_control/include/racer_control/
raceline.hpp` + `pure_pursuit.hpp` (and their .cpp bodies) line-for-line into Python: same
CSV format, same nearest-point/lookahead search, same curvature-adaptive lookahead formula,
same pure-pursuit curvature-to-steering formula, same clamp.

Logic duplication across languages is a known, named risk here (see this repo's task
description for S.3): C++ and Python WILL silently diverge unless checked. That check is
`tests/test_base_controller_divergence.py` in this package plus the
`test_pure_pursuit_cli_divergence` CTest
(`ros_ws/src/racer_control/test/divergence/compare_divergence.py`, wired into
`ros_ws/src/racer_control/CMakeLists.txt`) on the C++ side -- a
committed fixture of synthetic states run through BOTH implementations, compared within a
stated tolerance. This module is NOT allowed to import or wrap the C++ core (there is no
such binding); it is an independent re-implementation by design, exactly like
`ros_ws/src/racer_control/include/racer_control/raceline.hpp`'s own docstring describes for
its C++ reader ("each deployment target parses the shared interchange file on its own").

Sign convention (claude-docs/06-vehicle-params.md, REP-103), identical to the C++ core:
steering angle is the road-wheel angle in radians, LEFT positive; yaw is counter-clockwise
positive, x forward, y left.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

_COLUMNS = ("s_m", "x_m", "y_m", "heading_rad", "curvature_1pm", "target_speed_mps")


class RacelineLoadError(Exception):
    """Raised on any I/O or format problem loading a raceline CSV. Mirrors
    `racer_control::RacelineLoadError` (raceline.hpp): no silent fallback to an empty or
    degenerate raceline."""


@dataclass(frozen=True)
class RacelinePoint:
    s_m: float
    x_m: float
    y_m: float
    heading_rad: float
    curvature_1pm: float
    target_speed_mps: float


class Raceline:
    """A loaded, closed-loop reference path. Mirrors `racer_control::Raceline`."""

    def __init__(self, points: tuple[RacelinePoint, ...]) -> None:
        if not points:
            raise RacelineLoadError("Raceline: refusing to construct from zero points")
        self._points = points
        # Closed-loop total length, used only by racer_train.reward's progress wraparound --
        # NOT part of the ported pure-pursuit algorithm itself. The raceline CSV's s_m column
        # does not include the closing segment from the last point back to the first (see
        # tools/raceline/io.py), so it is added here once.
        closing_segment_m = math.hypot(
            points[0].x_m - points[-1].x_m, points[0].y_m - points[-1].y_m
        )
        self.length_m = points[-1].s_m + closing_segment_m

    def __len__(self) -> int:
        return len(self._points)

    def at(self, index: int) -> RacelinePoint:
        return self._points[index % len(self._points)]

    @classmethod
    def load_from_csv(cls, path: Path | str) -> Raceline:
        """Parses the tools/raceline-format CSV (see tools/raceline/io.py, the format's
        producer): `#`-commented provenance header, a header row, then
        `s_m,x_m,y_m,heading_rad,curvature_1pm,target_speed_mps` rows. Mirrors
        `racer_control::Raceline::load_from_csv` field-for-field, including CRLF tolerance."""
        path = Path(path)
        try:
            raw_lines = path.read_text(encoding="utf-8").splitlines()
        except OSError as exc:
            raise RacelineLoadError(
                f"Raceline.load_from_csv: could not open '{path}': {exc}"
            ) from exc

        found_header = False
        points: list[RacelinePoint] = []
        for line_number, raw_line in enumerate(raw_lines, start=1):
            # str.splitlines() already strips a trailing "\r\n" or "\n" cleanly, but a lone
            # "\r" mid-content (unlikely, but matches the C++ reader's defense-in-depth) is
            # handled the same way the C++ reader handles it.
            line = raw_line.removesuffix("\r")
            if not line:
                continue
            if line[0] == "#":
                continue  # provenance header line, see tools/raceline/io.py
            fields = next(csv.reader([line]))
            if not found_header:
                if tuple(fields) != _COLUMNS:
                    raise RacelineLoadError(
                        f"{path}: expected CSV header '{','.join(_COLUMNS)}', got '{line}'"
                    )
                found_header = True
                continue
            if len(fields) != 6:
                raise RacelineLoadError(
                    f"{path}:{line_number}: expected 6 columns, got {len(fields)}"
                )
            try:
                values = tuple(float(f) for f in fields)
            except ValueError as exc:
                raise RacelineLoadError(f"{path}:{line_number}: {exc}") from exc
            points.append(RacelinePoint(*values))

        if not found_header:
            raise RacelineLoadError(f"{path}: no CSV header row found")
        if not points:
            raise RacelineLoadError(f"{path}: no raceline data rows found")
        return cls(tuple(points))

    def nearest_index(self, x: float, y: float) -> int:
        """Index of the closest raceline point to (x, y). Mirrors
        `racer_control::Raceline::nearest_index` (O(n) linear scan)."""
        best_index = 0
        best_dist2 = math.inf
        for i, point in enumerate(self._points):
            dx = point.x_m - x
            dy = point.y_m - y
            dist2 = dx * dx + dy * dy
            if dist2 < best_dist2:
                best_dist2 = dist2
                best_index = i
        return best_index

    def advance_to_lookahead(self, from_index: int, x: float, y: float, lookahead_m: float) -> int:
        """Mirrors `racer_control::Raceline::advance_to_lookahead`: walk forward (with
        wraparound) from `from_index` until a point is at least `lookahead_m` away, or return
        the farthest point found if the whole loop is shorter than `lookahead_m`."""
        n = len(self._points)
        idx = from_index % n
        best_index = idx
        best_dist = -1.0
        for _ in range(n):
            point = self._points[idx]
            dist = math.hypot(point.x_m - x, point.y_m - y)
            if dist > best_dist:
                best_dist = dist
                best_index = idx
            if dist >= lookahead_m:
                return idx
            idx = (idx + 1) % n
        return best_index


@dataclass(frozen=True)
class PurePursuitConfig:
    """Mirrors `racer_control::PurePursuitConfig`. `wheelbase_m` and
    `max_steering_angle_rad` must come from the generated vehicle_params binding
    (CLAUDE.md invariant 2) at the call site, never hand-typed here; `lookahead_*` are
    tuning gains, not physical constants (see tracker_node.cpp's ROS param defaults, which
    this module's callers mirror)."""

    wheelbase_m: float
    lookahead_min_m: float
    lookahead_max_m: float
    lookahead_curvature_ref_1pm: float
    max_steering_angle_rad: float


@dataclass(frozen=True)
class PurePursuitCommand:
    steering_angle_rad: float
    speed_mps: float


class PurePursuitController:
    """Mirrors `racer_control::PurePursuitController` exactly (see pure_pursuit.cpp)."""

    def __init__(self, config: PurePursuitConfig) -> None:
        self.config = config

    def lookahead_distance_m(self, curvature_1pm: float) -> float:
        cfg = self.config
        k = abs(curvature_1pm)
        frac = min(k / cfg.lookahead_curvature_ref_1pm, 1.0)
        return cfg.lookahead_max_m - (cfg.lookahead_max_m - cfg.lookahead_min_m) * frac

    def compute_command(
        self, raceline: Raceline, x_m: float, y_m: float, yaw_rad: float
    ) -> PurePursuitCommand:
        nearest = raceline.nearest_index(x_m, y_m)
        nearest_point = raceline.at(nearest)

        lookahead_m = self.lookahead_distance_m(nearest_point.curvature_1pm)
        target_index = raceline.advance_to_lookahead(nearest, x_m, y_m, lookahead_m)
        target = raceline.at(target_index)

        # World-frame vector to the lookahead target, rotated into the vehicle body frame
        # (REP-103: x forward, y left) by -yaw_rad -- identical to pure_pursuit.cpp.
        dx = target.x_m - x_m
        dy = target.y_m - y_m
        cos_neg_yaw = math.cos(-yaw_rad)
        sin_neg_yaw = math.sin(-yaw_rad)
        lx = cos_neg_yaw * dx - sin_neg_yaw * dy
        ly = sin_neg_yaw * dx + cos_neg_yaw * dy

        lookahead_dist2 = lx * lx + ly * ly
        commanded_curvature_1pm = 0.0
        if lookahead_dist2 > 1e-9:
            commanded_curvature_1pm = 2.0 * ly / lookahead_dist2

        steering_angle_rad = math.atan(commanded_curvature_1pm * self.config.wheelbase_m)
        steering_angle_rad = max(
            -self.config.max_steering_angle_rad,
            min(self.config.max_steering_angle_rad, steering_angle_rad),
        )

        return PurePursuitCommand(
            steering_angle_rad=steering_angle_rad, speed_mps=nearest_point.target_speed_mps
        )
