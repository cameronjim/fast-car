# read and write the gym raceline csv, and load one straight into a RacelineIndex

from __future__ import annotations

import time
import uuid
from pathlib import Path

import numpy as np

from .raceline_index import RacelineIndex

RACELINE_COLUMNS = ("s_m", "x_m", "y_m", "psi_rad", "kappa_radpm", "vx_mps", "ax_mps2")
RACELINE_DELIMITER = ";"
# the gym reads the header off line index 2 and the numbers from line 3 on
RACELINE_HEADER_ROW = 2
CENTERLINE_COLUMNS = ("x_m", "y_m", "w_tr_right_m", "w_tr_left_m")
GENERATED_SUFFIX = "_raceline_gen.csv"


def generated_raceline_path(track) -> Path:
    """where a generated raceline lives, beside the map's shipped one."""
    if track.filepath is None:
        raise ValueError("track was built without a map directory, so it has no raceline path")
    return Path(track.filepath).parent / f"{track.spec.name}{GENERATED_SUFFIX}"


def centerline_path(track) -> Path:
    """the map's centerline csv, which is the only file carrying track widths."""
    if track.filepath is None:
        raise ValueError("track was built without a map directory, so it has no centerline path")
    return Path(track.filepath).parent / f"{track.spec.name}_centerline.csv"


def read_centerline_csv(path) -> np.ndarray:
    """centerline as the [x, y, w_tr_right, w_tr_left] array the tph optimizer expects."""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"no centerline csv at {path}")
    reftrack = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    if reftrack.shape[1] < 4:
        raise ValueError(f"{path}: expected 4 columns {CENTERLINE_COLUMNS}, got {reftrack.shape[1]}")
    reftrack = reftrack[:, :4].astype(np.float64)
    # tph needs the loop left open, and these files sometimes repeat the start point
    if np.hypot(*(reftrack[0, :2] - reftrack[-1, :2])) < 1e-6:
        reftrack = reftrack[:-1]
    return reftrack


def write_raceline_csv(path, raceline: dict, comment: str = "") -> Path:
    """write the seven raceline columns in the gym's `;`-delimited three-header-line format."""
    path = Path(path)
    columns = [np.asarray(raceline[name], dtype=np.float64).reshape(-1) for name in RACELINE_COLUMNS]
    lengths = {name: column.size for name, column in zip(RACELINE_COLUMNS, columns)}
    if len(set(lengths.values())) != 1:
        raise ValueError(f"raceline columns must all be the same length, got {lengths}")
    columns = _repeat_start_point(columns)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as raceline_csv:
        raceline_csv.write(f"# {uuid.uuid4()}\n")
        raceline_csv.write(f"# {time.strftime('%Y-%m-%d %H:%M:%S')} {comment}".rstrip() + "\n")
        raceline_csv.write("# " + "; ".join(RACELINE_COLUMNS) + "\n")
        for row in np.column_stack(columns):
            raceline_csv.write("; ".join(repr(float(value)) for value in row) + "\n")
    return path


def _repeat_start_point(columns: list) -> list:
    """close the lap the way the shipped files do, with s at the full lap length on the repeat."""
    s_m, x_m, y_m = columns[0], columns[1], columns[2]
    if np.hypot(x_m[-1] - x_m[0], y_m[-1] - y_m[0]) < 1e-9:
        return columns
    closed = [np.append(column, column[0]) for column in columns]
    closed[0][-1] = s_m[-1] + float(np.hypot(x_m[0] - x_m[-1], y_m[0] - y_m[-1]))
    return closed


def read_raceline_csv(path) -> dict:
    """the seven raceline columns of a gym-format raceline csv, keyed by column name."""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"no raceline csv at {path}")
    with open(path, "r") as raceline_csv:
        lines = raceline_csv.readlines()
    if len(lines) <= RACELINE_HEADER_ROW:
        raise ValueError(f"{path}: too short to hold a header on line {RACELINE_HEADER_ROW + 1}")
    names = [name.replace("#", "").strip() for name in lines[RACELINE_HEADER_ROW].split(RACELINE_DELIMITER)]
    if names != list(RACELINE_COLUMNS):
        raise ValueError(f"{path}: expected columns {list(RACELINE_COLUMNS)}, got {names}")
    values = np.loadtxt(
        path, delimiter=RACELINE_DELIMITER, skiprows=RACELINE_HEADER_ROW + 1, ndmin=2
    )
    return {name: values[:, index] for index, name in enumerate(names)}


def raceline_index_from_csv(path) -> RacelineIndex:
    """reference line for the planners, read from a raceline csv rather than off a gym track."""
    columns = read_raceline_csv(path)
    return RacelineIndex(columns["x_m"], columns["y_m"], speeds=columns["vx_mps"])
