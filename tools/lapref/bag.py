"""The two signals `lapref` is allowed to read: the steering angle of the gated `/drive` and the
forward speed of `/odom/wheel`. Nothing else is decoded (the scan topic is not decoded either: its
first log time only gives the zero of the bag axis, see pnp_data.mcapread).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from pnp_data import mcapread

#: The topics lapref reads. A test asserts this tuple is all it asks the reader for.
TOPICS = (mcapread.DRIVE, mcapread.ODOM)


@dataclass
class Signals:
    #: /drive: bag time (s) and steering angle (rad).
    steer_t: NDArray[np.float64]
    steer: NDArray[np.float64]
    #: /odom/wheel: bag time (s) and forward speed (m/s).
    odom_t: NDArray[np.float64]
    odom_v: NDArray[np.float64]

    def window(self, t_from: float, t_to: float) -> Signals:
        """The samples with t_from <= t <= t_to (both ends included)."""
        a = (self.steer_t >= t_from) & (self.steer_t <= t_to)
        b = (self.odom_t >= t_from) & (self.odom_t <= t_to)
        return Signals(self.steer_t[a], self.steer[a], self.odom_t[b], self.odom_v[b])

    def save(self, path: Path) -> None:
        np.savez_compressed(
            path,
            steer_t=self.steer_t,
            steer=self.steer,
            odom_t=self.odom_t,
            odom_v=self.odom_v,
        )

    @classmethod
    def load(cls, path: Path) -> Signals:
        with np.load(path, allow_pickle=False) as d:
            return cls(d["steer_t"], d["steer"], d["odom_t"], d["odom_v"])


def read_signals(bag: Path) -> Signals:
    """Steering of /drive and speed of /odom/wheel on the whole bag, on the bag axis."""
    _t0, cols = mcapread.read_columns(
        bag,
        {
            mcapread.DRIVE: lambda m: (m.drive.steering_angle,),
            mcapread.ODOM: lambda m: (m.twist.twist.linear.x,),
        },
    )
    drive, odom = cols[mcapread.DRIVE], cols[mcapread.ODOM]
    return Signals(drive[:, 0], drive[:, 1], odom[:, 0], odom[:, 1])
