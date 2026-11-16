"""The runs of a bag: one row per run, from the commands, the planner status and the wheels.

A run is a maximal stretch of /pnp/planner/status messages with no gap above 2 s (racer_eval.runs
is the one run finder). A bag without planner status gets the stretches of non-zero /drive_raw
commands as runs, kind `drive_raw`. A row runs from the first non-zero command of the run to its
last status (or last command), the same definition as the segments of pnp-v0.4.yaml.

Row fields (pre-registration v0.4 section 0.3): kind, start and end on the bag axis, the 99th
percentile of the commanded speed, the direction from the sign of the mean steering command
(ccw for a positive mean: the steering angle is positive to the left), the wheel distance and the
largest wheel speed.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray
from racer_eval.runs import find_runs

from pnp_data import mcapread


@dataclass(frozen=True)
class RunRow:
    run_id: str
    kind: str
    start_s: float
    end_s: float
    #: 99th percentile of the commanded speed over the run's non-zero /drive_raw commands (m/s).
    p99_cmd_mps: float
    direction: str
    wheel_distance_m: float
    max_wheel_mps: float

    @property
    def duration_s(self) -> float:
        return self.end_s - self.start_s

    @property
    def cap_mps(self) -> float:
        """The speed cell: the commanded cap, the 99th percentile rounded to 0.1 m/s."""
        return round(self.p99_cmd_mps, 1)

    def to_dict(self) -> dict[str, Any]:
        return {**asdict(self), "duration_s": self.duration_s, "cap_mps": self.cap_mps}


@dataclass
class RunSignals:
    """The arrays `runs` reads, on the bag axis."""

    draw_t: NDArray[np.float64]
    draw_speed: NDArray[np.float64]
    draw_steer: NDArray[np.float64]
    odom_t: NDArray[np.float64]
    odom_v: NDArray[np.float64]
    status_t: NDArray[np.float64]
    first_scan_log_s: float = 0.0

    def save(self, path: Path) -> None:
        np.savez_compressed(path, **{k: v for k, v in self.__dict__.items()})

    @classmethod
    def load(cls, path: Path) -> RunSignals:
        with np.load(path, allow_pickle=False) as d:
            return cls(
                d["draw_t"],
                d["draw_speed"],
                d["draw_steer"],
                d["odom_t"],
                d["odom_v"],
                d["status_t"],
                float(d["first_scan_log_s"]),
            )


def read_run_signals(bag: Path) -> RunSignals:
    """/drive_raw (speed, steering), /odom/wheel (speed) and /pnp/planner/status times of a bag."""
    t0, cols = mcapread.read_columns(
        bag,
        {
            mcapread.DRIVE_RAW: lambda m: (m.drive.speed, m.drive.steering_angle),
            mcapread.ODOM: lambda m: (m.twist.twist.linear.x,),
            mcapread.STATUS: lambda m: (m.state,),
        },
    )
    d, o, s = cols[mcapread.DRIVE_RAW], cols[mcapread.ODOM], cols[mcapread.STATUS]
    # a topic with no message (a bag with no /drive_raw, or no odometry) is an empty column set
    if d.shape[1] < 3:
        d = np.zeros((0, 3))
    if o.shape[1] < 2:
        o = np.zeros((0, 2))
    if s.shape[1] < 1:
        s = np.zeros((0, 1))
    return RunSignals(d[:, 0], d[:, 1], d[:, 2], o[:, 0], o[:, 1], s[:, 0], t0)


def _distance(t: NDArray[np.float64], v: NDArray[np.float64]) -> float:
    """Trapezoid integral of max(0, speed); a gap above 1 s adds nothing."""
    if len(t) < 2:
        return 0.0
    speed = np.maximum(v, 0.0)
    dt = np.diff(t)
    step = np.where((dt > 0.0) & (dt <= 1.0), 0.5 * (speed[1:] + speed[:-1]) * dt, 0.0)
    return float(np.sum(step))


def run_rows(
    draw_t: ArrayLike,
    draw_speed: ArrayLike,
    draw_steer: ArrayLike,
    odom_t: ArrayLike,
    odom_v: ArrayLike,
    status_t: ArrayLike,
) -> list[RunRow]:
    """The rows of a bag. Runs are named R0, R1, ... in time order."""
    dt = np.asarray(draw_t, dtype=np.float64)
    dv = np.asarray(draw_speed, dtype=np.float64)
    ds = np.asarray(draw_steer, dtype=np.float64)
    ot = np.asarray(odom_t, dtype=np.float64)
    ov = np.asarray(odom_v, dtype=np.float64)
    st = np.asarray(status_t, dtype=np.float64)
    runs = find_runs(dt, dv, st if len(st) else None)
    rows: list[RunRow] = []
    for run in runs:
        start = run.t_first_cmd if run.t_first_cmd is not None else run.t_start
        end = run.t_end
        cmd = (dt >= start) & (dt <= end) & (dv != 0.0)
        speeds, steer = dv[cmd], ds[cmd]
        wheel = (ot >= start) & (ot <= end)
        rows.append(
            RunRow(
                run_id=run.run_id,
                kind=run.kind,
                start_s=float(start),
                end_s=float(end),
                p99_cmd_mps=float(np.percentile(np.abs(speeds), 99.0)) if len(speeds) else 0.0,
                direction="ccw" if len(steer) and float(np.mean(steer)) > 0.0 else "cw",
                wheel_distance_m=_distance(ot[wheel], ov[wheel]),
                max_wheel_mps=float(np.max(np.abs(ov[wheel]))) if wheel.any() else 0.0,
            )
        )
    return rows


def run_rows_all(
    draw_t: ArrayLike,
    draw_speed: ArrayLike,
    draw_steer: ArrayLike,
    odom_t: ArrayLike,
    odom_v: ArrayLike,
    status_t: ArrayLike,
) -> list[RunRow]:
    """The rows of a bag for the v0.5 dynamics rule (`session_2026_11_15`: every run, whatever
    drove it). The PnP runs are the status stretches, exactly as `run_rows` finds them. A bag that
    holds status stretches AND other drivers (gap follow) has non-zero /drive_raw stretches that no
    status stretch covers; each of those is a run of kind `drive_raw`. In a bag with no status this
    is exactly `run_rows`. Runs are named R0, R1, ... in time order over both kinds."""
    dt = np.asarray(draw_t, dtype=np.float64)
    dv = np.asarray(draw_speed, dtype=np.float64)
    pnp = run_rows(draw_t, draw_speed, draw_steer, odom_t, odom_v, status_t)
    if len(np.asarray(status_t)) == 0:
        return pnp
    ds = np.asarray(draw_steer, dtype=np.float64)
    ot = np.asarray(odom_t, dtype=np.float64)
    ov = np.asarray(odom_v, dtype=np.float64)
    st_runs = find_runs(dt, dv, np.asarray(status_t, dtype=np.float64))
    extra: list[RunRow] = []
    for run in find_runs(dt, dv, None):
        covered = any(run.t_start <= r.t_end and r.t_start <= run.t_end for r in st_runs)
        if covered:
            continue
        cmd = (dt >= run.t_start) & (dt <= run.t_end) & (dv != 0.0)
        wheel = (ot >= run.t_start) & (ot <= run.t_end)
        extra.append(
            RunRow(
                run_id="",
                kind="drive_raw",
                start_s=float(run.t_start),
                end_s=float(run.t_end),
                p99_cmd_mps=float(np.percentile(np.abs(dv[cmd]), 99.0)),
                direction="ccw" if float(np.mean(ds[cmd])) > 0.0 else "cw",
                wheel_distance_m=_distance(ot[wheel], ov[wheel]),
                max_wheel_mps=float(np.max(np.abs(ov[wheel]))) if wheel.any() else 0.0,
            )
        )
    rows = sorted([*pnp, *extra], key=lambda r: r.start_s)
    return [RunRow(**{**asdict(r), "run_id": f"R{i}"}) for i, r in enumerate(rows)]


def rows_of_signals_all(sig: RunSignals) -> list[RunRow]:
    return run_rows_all(
        sig.draw_t, sig.draw_speed, sig.draw_steer, sig.odom_t, sig.odom_v, sig.status_t
    )


def rows_of_signals(sig: RunSignals) -> list[RunRow]:
    return run_rows(
        sig.draw_t, sig.draw_speed, sig.draw_steer, sig.odom_t, sig.odom_v, sig.status_t
    )


def format_rows(rows: Sequence[RunRow]) -> str:
    head = (
        f"{'run':<4} {'kind':<9} {'start s':>9} {'end s':>9} {'dur s':>7} {'p99 cmd':>8} "
        f"{'dir':<4} {'dist m':>8} {'max wheel':>9}"
    )
    lines = [head]
    for r in rows:
        lines.append(
            f"{r.run_id:<4} {r.kind:<9} {r.start_s:>9.1f} {r.end_s:>9.1f} {r.duration_s:>7.1f} "
            f"{r.p99_cmd_mps:>8.2f} {r.direction:<4} {r.wheel_distance_m:>8.1f} "
            f"{r.max_wheel_mps:>9.2f}"
        )
    return "\n".join(lines)
