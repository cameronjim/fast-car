"""racer_tools.raceline_loader: parses a tools/raceline-format CSV into plain,
ROS-free point records (roadmap milestone 3).

Pure stdlib (no ROS, no numpy) so it is L1 unit-testable with no ROS install
(claude-docs/12-testing.md). This is a THIRD independent parser of the same shared
interchange file (tools/raceline/io.py's format) alongside
racer_control/include/racer_control/raceline.hpp (C++) and
sim/bridge/racer_gym_bridge/racer_gym_bridge/track_loader.py (Python/numpy, a different
colcon workspace) -- each consumer parses the format independently rather than sharing
code across workspace/language boundaries, the established pattern for this file
(raceline.hpp's own docstring makes the same point).

File format: `#`-commented provenance header, a header row, then
`s_m,x_m,y_m,heading_rad,curvature_1pm,target_speed_mps` rows.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

_EXPECTED_HEADER = ("s_m", "x_m", "y_m", "heading_rad", "curvature_1pm", "target_speed_mps")


class RacelineLoadError(ValueError):
    pass


@dataclass(frozen=True)
class RacelinePoint:
    s_m: float
    x_m: float
    y_m: float
    heading_rad: float
    curvature_1pm: float
    target_speed_mps: float


def load_raceline_points(path: str | Path) -> list[RacelinePoint]:
    """Returns the raceline's points in file order. Raises RacelineLoadError on any I/O or
    format problem (missing file, wrong column header, non-numeric field, empty body) --
    there is no silent fallback to an empty/degenerate raceline."""
    path = Path(path)
    if not path.is_file():
        raise RacelineLoadError(f"raceline file not found: {path}")

    points: list[RacelinePoint] = []
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
            if len(fields) != len(_EXPECTED_HEADER):
                raise RacelineLoadError(
                    f"{path}: expected {len(_EXPECTED_HEADER)} fields, got {len(fields)}: {fields}"
                )
            try:
                values = [float(v) for v in fields]
            except ValueError as e:
                raise RacelineLoadError(f"{path}: non-numeric field in raceline data: {e}") from e
            points.append(RacelinePoint(*values))

    if not header_seen:
        raise RacelineLoadError(f"{path}: no CSV header row found")
    if not points:
        raise RacelineLoadError(f"{path}: no raceline data rows found")
    return points
