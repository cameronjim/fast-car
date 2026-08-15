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
