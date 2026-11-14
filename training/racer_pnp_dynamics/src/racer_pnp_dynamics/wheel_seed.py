"""The wheel motion over a scan interval: the start value of the T2 matcher.

T2 is the chained scan-to-scan ICP started from the motion the wheels say the car made (design
doc 13 section 15.4, pre-registration v0.3 section 2.2). For an interval [t0, t1]:

    path     = integral of the /odom/wheel speed v
    heading  = integral of  gain(|delta|) * v * tan(delta) / L

with `delta` the gated `/drive` steering command held between messages (what the actuators were
given, no model of the steering lag), `gain` the steering.yaw_rate_gain_table of vehicle_params
(piecewise linear in |delta|, the same rule the car's wheel_twist_node applies) and `L` the
wheelbase. Both integrals are midpoint sums over SUBSTEPS pieces of the interval. The seed is only
a start value: the matcher result is the whole motion, so a seed that is a little off costs
nothing but iterations. An interval with no wheel speed or no fresh steering command over any
piece has no seed (NaN) and is matched from the identity, exactly as T1 does.
"""

from __future__ import annotations

import numpy as np

from .bags import Run
from .vehicle import Vehicle
from .windows import cmd_at, wheel_at

SUBSTEPS = 10


def interval_seed(
    run: Run, vehicle: Vehicle, t0_s: np.ndarray, t1_s: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """(heading change in rad, path length in m) of each interval [t0_s[i], t1_s[i]], NaN where
    the speed or the steering command is missing anywhere in it."""
    t0 = np.asarray(t0_s, dtype=np.float64)
    t1 = np.asarray(t1_s, dtype=np.float64)
    h = (t1 - t0) / SUBSTEPS
    mid = t0[:, None] + (np.arange(SUBSTEPS)[None, :] + 0.5) * h[:, None]
    v = wheel_at(run, mid)
    delta = cmd_at(run, mid)[..., 1]
    with np.errstate(invalid="ignore"):
        rate = vehicle.yaw_gain(delta) * v * np.tan(delta) / vehicle.wheelbase_m
        dyaw = (rate * h[:, None]).sum(axis=1)
        path = (v * h[:, None]).sum(axis=1)
    return dyaw, path
