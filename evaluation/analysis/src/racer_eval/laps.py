"""Lap tables (pre-registration sections 4.3 and 5.2).

Sources of lap times, per run:
- camera: the lap counter's `/lap/event` (std_msgs/String JSON: lap, lap_time_s,
  crossing_time_s, distance_m). Primary. It does not use the planner, the pose or the lap memory.
- pnp: D's `/pnp/lap/event` (racer_pnp_msgs/LapEvent: lap_index, lap_time_s). Secondary.
- wheel: path length from `/odom/wheel` over each lap, a cross-check on the camera counter's own
  `distance_m` and the basis of the GP3 distance check.

All functions take plain arrays; bag_io.py fills them from mcap. Laps of the two sources are
matched by their order inside the run (1 for the first completed lap): the camera counter's own
`lap` number need not restart with the run.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import asdict, dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

#: Section 5.2: "any lap where the two differ by more than 0.3 s is listed".
LAP_SOURCE_TOLERANCE_S = 0.3
#: Section 4.3: "wheel distance (at least 0.9 times 20 lap lengths)".
GP3_WHEEL_DISTANCE_FACTOR = 0.9
GP3_LAPS = 20
#: How long after a run's last status a lap event may still arrive and belong to the run.
EVENT_MARGIN_S = 3.0


@dataclass(frozen=True)
class LapRow:
    run_id: str
    lap: int
    camera_time_s: float | None
    pnp_time_s: float | None
    #: camera minus pnp, None when either is missing.
    diff_s: float | None
    wheel_distance_m: float | None
    camera_distance_m: float | None
    #: |diff| above LAP_SOURCE_TOLERANCE_S.
    flag_differs: bool
    flag_missing_camera: bool
    flag_missing_pnp: bool

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def wheel_distance(
    odom_t: ArrayLike, odom_v: ArrayLike, t_from: float, t_to: float
) -> float | None:
    """Path length (m) of |wheel speed| over [t_from, t_to], trapezoid with the end points
    interpolated. None when the odometry does not cover the interval."""
    t = np.asarray(odom_t, dtype=np.float64)
    v = np.abs(np.asarray(odom_v, dtype=np.float64))
    if len(t) < 2 or t_from < t[0] or t_to > t[-1] or t_to < t_from:
        return None
    inner = (t > t_from) & (t < t_to)
    ts = np.concatenate(([t_from], t[inner], [t_to]))
    vs = np.interp(ts, t, v)
    return float(np.sum(0.5 * (vs[1:] + vs[:-1]) * np.diff(ts)))


def in_run_mask(event_t: ArrayLike, t_start: float, t_end: float) -> NDArray[np.bool_]:
    """Events that arrived during a run (up to EVENT_MARGIN_S after its last status)."""
    t = np.asarray(event_t, dtype=np.float64)
    return (t >= t_start) & (t <= t_end + EVENT_MARGIN_S)


def lap_table(
    run_id: str,
    camera_event_t: ArrayLike,
    camera_lap_time_s: ArrayLike,
    camera_distance_m: ArrayLike,
    pnp_event_t: ArrayLike,
    pnp_lap_time_s: ArrayLike,
    odom_t: ArrayLike,
    odom_v: ArrayLike,
) -> list[LapRow]:
    """One row per lap of one run. Event arrays hold only the events of this run, in arrival
    order. A lap's wheel distance is taken over [arrival - lap time, arrival] of the camera event
    when there is one, else of the pnp event."""
    cam_t = np.asarray(camera_event_t, dtype=np.float64)
    cam_s = np.asarray(camera_lap_time_s, dtype=np.float64)
    cam_d = np.asarray(camera_distance_m, dtype=np.float64)
    pnp_t = np.asarray(pnp_event_t, dtype=np.float64)
    pnp_s = np.asarray(pnp_lap_time_s, dtype=np.float64)
    if not (len(cam_t) == len(cam_s) == len(cam_d)) or len(pnp_t) != len(pnp_s):
        raise ValueError("event arrays of one source must have the same length")
    rows = []
    for i in range(max(len(cam_t), len(pnp_t))):
        has_cam, has_pnp = i < len(cam_t), i < len(pnp_t)
        cam = float(cam_s[i]) if has_cam else None
        pnp = float(pnp_s[i]) if has_pnp else None
        diff = None if cam is None or pnp is None else cam - pnp
        if has_cam:
            span = (float(cam_t[i] - cam_s[i]), float(cam_t[i]))
        else:
            span = (float(pnp_t[i] - pnp_s[i]), float(pnp_t[i]))
        rows.append(
            LapRow(
                run_id=run_id,
                lap=i + 1,
                camera_time_s=cam,
                pnp_time_s=pnp,
                diff_s=diff,
                wheel_distance_m=wheel_distance(odom_t, odom_v, *span),
                camera_distance_m=float(cam_d[i]) if has_cam else None,
                flag_differs=diff is not None and abs(diff) > LAP_SOURCE_TOLERANCE_S,
                flag_missing_camera=not has_cam,
                flag_missing_pnp=not has_pnp,
            )
        )
    return rows


def run_wheel_distance(odom_t: ArrayLike, odom_v: ArrayLike, t_from: float, t_to: float) -> float:
    """Wheel path length over a whole run (the GP3 distance check); raises when not covered."""
    d = wheel_distance(odom_t, odom_v, t_from, t_to)
    if d is None:
        raise ValueError("the wheel odometry does not cover the run")
    return d


def lap_times_of(rows: Sequence[LapRow]) -> tuple[float, ...]:
    """Primary lap times of a table: the camera counter's, where a lap has one."""
    return tuple(r.camera_time_s for r in rows if r.camera_time_s is not None)
