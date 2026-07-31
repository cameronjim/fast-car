"""Ties centerline geometry + speed profile into one `Raceline` result."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from raceline.geometry import (
    arc_length_closed,
    curvature_from_points_closed,
    heading_from_points_closed,
    resample_closed_uniform,
    smooth_closed,
)
from raceline.speed_profile import accel_limited_profile_closed, curvature_speed_cap


@dataclass(frozen=True)
class Raceline:
    """A closed-loop raceline: arrays are all the same length, index-aligned, closed
    (index ``n - 1`` connects back to index ``0``)."""

    s_m: np.ndarray
    x_m: np.ndarray
    y_m: np.ndarray
    heading_rad: np.ndarray
    curvature_1pm: np.ndarray
    target_speed_mps: np.ndarray

    def __len__(self) -> int:
        return len(self.s_m)


def build_raceline_from_centerline(
    x_m: np.ndarray,
    y_m: np.ndarray,
    *,
    a_lat_max_mps2: float,
    a_max_mps2: float,
    v_max_mps: float,
    resample_ds_m: float = 0.1,
    smooth_window: int = 5,
    speed_passes: int = 3,
) -> Raceline:
    """Run the full pipeline (resample -> smooth -> geometry -> speed profile) on a raw
    closed-loop centerline. See this package's `__init__.py` docstring for the pipeline
    overview and what each step's simplifications are.
    """
    x_u, y_u = resample_closed_uniform(x_m, y_m, resample_ds_m)
    x_s, y_s = smooth_closed(x_u, y_u, smooth_window)

    heading = heading_from_points_closed(x_s, y_s)
    kappa = curvature_from_points_closed(x_s, y_s)
    s, seg = arc_length_closed(x_s, y_s)

    v_cap = curvature_speed_cap(kappa, a_lat_max_mps2, v_max_mps)
    v = accel_limited_profile_closed(v_cap, seg, a_max_mps2, num_passes=speed_passes)

    return Raceline(
        s_m=s,
        x_m=x_s,
        y_m=y_s,
        heading_rad=heading,
        curvature_1pm=kappa,
        target_speed_mps=v,
    )
