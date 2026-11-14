"""The arrays of one bag that the analysis reads, and their .npz form (the CI fixture).

All times are bag seconds: the recorder's log time minus the log time of the first `/scan`
(the axis of tools/pnp_replay/bag.py and of the split file). bag_io.py fills a BagData from mcap.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

_FLOAT = np.float64


def _f() -> NDArray[np.float64]:
    return np.zeros(0, dtype=_FLOAT)


def _i() -> NDArray[np.int64]:
    return np.zeros(0, dtype=np.int64)


def _s() -> NDArray[np.str_]:
    return np.zeros(0, dtype=str)


@dataclass
class BagData:
    name: str = ""
    #: Log time of the first /scan in seconds (the zero of the axis); 0 when unknown.
    t0_log_s: float = 0.0
    #: /drive_raw: arrival time, speed (m/s), steering (rad).
    draw_t: NDArray[np.float64] = field(default_factory=_f)
    draw_speed: NDArray[np.float64] = field(default_factory=_f)
    #: /pnp/planner/status: arrival time and state.
    st_t: NDArray[np.float64] = field(default_factory=_f)
    st_state: NDArray[np.int64] = field(default_factory=_i)
    #: /safety/events: arrival time, severity, phase and source.
    ev_t: NDArray[np.float64] = field(default_factory=_f)
    ev_severity: NDArray[np.int64] = field(default_factory=_i)
    ev_phase: NDArray[np.int64] = field(default_factory=_i)
    ev_source: NDArray[np.str_] = field(default_factory=_s)
    #: /odom/wheel: arrival time and forward speed (m/s).
    odom_t: NDArray[np.float64] = field(default_factory=_f)
    odom_v: NDArray[np.float64] = field(default_factory=_f)
    #: /lap/event (camera counter): arrival time, lap, lap time (s), path length (m).
    cam_t: NDArray[np.float64] = field(default_factory=_f)
    cam_lap: NDArray[np.int64] = field(default_factory=_i)
    cam_lap_time_s: NDArray[np.float64] = field(default_factory=_f)
    cam_distance_m: NDArray[np.float64] = field(default_factory=_f)
    #: /pnp/lap/event: arrival time, lap index, lap time (s).
    pnp_t: NDArray[np.float64] = field(default_factory=_f)
    pnp_lap_index: NDArray[np.int64] = field(default_factory=_i)
    pnp_lap_time_s: NDArray[np.float64] = field(default_factory=_f)

    def save(self, path: Path) -> None:
        arrays = {f.name: getattr(self, f.name) for f in fields(self) if f.name != "name"}
        arrays["name"] = np.array(self.name)
        np.savez_compressed(path, **arrays)  # type: ignore[arg-type]

    @classmethod
    def load(cls, path: Path) -> BagData:
        out = cls()
        with np.load(path, allow_pickle=False) as data:
            for f in fields(cls):
                if f.name in data.files:
                    value = data[f.name]
                    setattr(out, f.name, str(value) if f.name == "name" else value)
        out.t0_log_s = float(out.t0_log_s)
        return out
