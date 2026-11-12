"""Bags to a 50 Hz run in true units.

All bags up to 2026-11-11 were recorded under vehicle_params 0.13.0 (speeds 1.401 times the true
speed, steering commands in the old scale). `run_from_bag_data` applies
tools/safety_replay/calibration.py `convert_bag` exactly once to those, so everything downstream
is in the units of the current config/vehicle_params.yaml. Bags recorded from 2026-11-12 on are
already in current units and are NOT converted (`recordings.recorded_for` decides per bag).

Commands come from /drive (what the safety gate let through, which is what the actuators
received). Wheel speed comes from /odom/wheel where the bag has it.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

_TOOLS = Path(__file__).resolve().parents[4] / "tools"
if str(_TOOLS) not in sys.path:  # the one place tools/ is put on the path
    sys.path.insert(0, str(_TOOLS))

from follower_replay.bag import BagData, ScanRecord, read_bag
from safety_replay import calibration, motion  # noqa: F401

GRID_HZ = 50.0
GRID_DT_S = 1.0 / GRID_HZ
DRIVE_STALE_S = 0.2  # a /drive older than this at a grid time is a gap, not a held command


@dataclass
class Run:
    """One bag on a 50 Hz grid. NaN marks no data (no odometry, or a gap in /drive)."""

    name: str
    t_s: np.ndarray  # grid times (header stamps)
    speed_cmd_mps: np.ndarray
    steering_cmd_rad: np.ndarray
    wheel_speed_mps: np.ndarray  # NaN everywhere when the bag has no /odom/wheel
    scans: list[ScanRecord] = field(default_factory=list)

    @property
    def has_odom(self) -> bool:
        return bool(np.isfinite(self.wheel_speed_mps).any())

    @property
    def duration_s(self) -> float:
        return float(self.t_s[-1] - self.t_s[0]) if len(self.t_s) else 0.0

    def command_valid(self) -> np.ndarray:
        return np.isfinite(self.speed_cmd_mps) & np.isfinite(self.steering_cmd_rad)


def resample_hold(
    t_src: np.ndarray, values: np.ndarray, t_grid: np.ndarray, max_age_s: float
) -> np.ndarray:
    """Zero order hold of the latest sample at or before each grid time; NaN when it is older
    than max_age_s or there is none yet."""
    out = np.full(len(t_grid), np.nan)
    if len(t_src) == 0:
        return out
    idx = np.searchsorted(t_src, t_grid, side="right") - 1
    ok = idx >= 0
    ok[ok] &= (t_grid[ok] - t_src[idx[ok]]) <= max_age_s
    out[ok] = values[idx[ok]]
    return out


def resample_linear(t_src: np.ndarray, values: np.ndarray, t_grid: np.ndarray) -> np.ndarray:
    """Linear interpolation, NaN outside the source span and across gaps over 0.1 s."""
    out = np.full(len(t_grid), np.nan)
    if len(t_src) < 2:
        return out
    out = np.interp(t_grid, t_src, values)
    idx = np.clip(np.searchsorted(t_src, t_grid), 1, len(t_src) - 1)
    gap = t_src[idx] - t_src[idx - 1]
    outside = (t_grid < t_src[0]) | (t_grid > t_src[-1])
    out[outside | (gap > 0.1)] = np.nan
    return out


def run_from_bag_data(
    name: str,
    bag: BagData,
    recorded: calibration.Recorded | None = calibration.RECORDED_0_13_0,
    current: calibration.Current | None = None,
) -> Run:
    """Convert a decoded bag to true units (exactly once) and resample to the 50 Hz grid.
    `recorded` is the calibration the bag was recorded under; None means it was recorded in the
    units of the current vehicle_params (2026-11-12 bags on) and nothing is converted."""
    if recorded is not None:
        bag = calibration.convert_bag(bag, recorded, current or calibration.load_current())
    drive = sorted(bag.drive, key=lambda d: d.stamp_s)
    if not drive:
        raise ValueError(f"{name}: no /drive messages")
    t_d = np.array([d.stamp_s for d in drive])
    t_grid = np.arange(t_d[0], t_d[-1], GRID_DT_S)
    speed = resample_hold(t_d, np.array([d.speed_mps for d in drive]), t_grid, DRIVE_STALE_S)
    steer = resample_hold(t_d, np.array([d.steering_rad for d in drive]), t_grid, DRIVE_STALE_S)
    odom = sorted(bag.odom, key=lambda o: o.stamp_s)
    wheel = resample_linear(
        np.array([o.stamp_s for o in odom]), np.array([o.speed_mps for o in odom]), t_grid
    )
    return Run(name, t_grid, speed, steer, wheel, sorted(bag.scans, key=lambda s: s.stamp_s))


def load_run(
    bag_dir: Path, recorded: calibration.Recorded | None = calibration.RECORDED_0_13_0
) -> Run:  # pragma: no cover - thin wrapper over the bag reader
    return run_from_bag_data(Path(bag_dir).name, read_bag(bag_dir), recorded)
