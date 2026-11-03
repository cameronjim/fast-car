"""Sensor realism options for bridge_node (localisation tuning, 2026-11-03).

f1tenth_gym's sensors are almost perfect: its wheel speed is the true body speed and its LiDAR
has 1 cm of noise and 30 m of range. Tuning an EKF and AMCL against that would be dishonest, so
bridge_node can, when asked (every default leaves the old behaviour alone):

  * scale, slip and add noise to the simulated /odom/wheel speed (``WheelSpeedNoise``): a tyre
    radius that is a couple of percent off, wheelspin under acceleration, ERPM jitter;
  * add range noise to /scan and cut it at the real sensor's maximum range
    (``degrade_scan``), returning +inf beyond it like sllidar_ros2 does for no return;
  * publish /scan and /odom/wheel at the real sensors' rates instead of every physics step
    (``publish_divider``).

Pure numpy, no ROS or gym imports, so it is L1-tested on its own (test/test_sensor_noise.py).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def publish_divider(step_rate_hz: float, target_rate_hz: float) -> int:
    """Publish every Nth step to get close to ``target_rate_hz``; 0 (or less) means every step.

    Never below 1: a target faster than the step rate cannot be met and is just every step.
    """
    if target_rate_hz <= 0.0 or step_rate_hz <= 0.0:
        return 1
    return max(1, round(step_rate_hz / target_rate_hz))


@dataclass
class WheelSpeedNoise:
    """measured = scale * true + slip_s * acceleration + N(0, noise_std).

    ``slip_s`` (seconds) turns the longitudinal acceleration into extra wheel speed: a wheel that
    spins up faster than the car under throttle and locks a little under braking. All three
    default to a perfect sensor.
    """

    scale: float = 1.0
    noise_std_mps: float = 0.0
    slip_s: float = 0.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.scale) or self.scale <= 0.0:
            raise ValueError(f"wheel speed scale must be positive, got {self.scale}")
        if not math.isfinite(self.noise_std_mps) or self.noise_std_mps < 0.0:
            raise ValueError(f"wheel speed noise std must be >= 0, got {self.noise_std_mps}")
        if not math.isfinite(self.slip_s) or self.slip_s < 0.0:
            raise ValueError(f"wheel slip gain must be >= 0, got {self.slip_s}")

    @property
    def is_perfect(self) -> bool:
        return self.scale == 1.0 and self.noise_std_mps == 0.0 and self.slip_s == 0.0

    def measure(self, true_speed_mps: float, accel_mps2: float, rng: np.random.Generator) -> float:
        if self.is_perfect:
            return float(true_speed_mps)
        noise = float(rng.normal(0.0, self.noise_std_mps)) if self.noise_std_mps > 0.0 else 0.0
        return float(self.scale * true_speed_mps + self.slip_s * accel_mps2 + noise)


def degrade_scan(
    ranges: np.ndarray, noise_std_m: float, range_max_m: float, rng: np.random.Generator
) -> np.ndarray:
    """Range noise and a maximum range. ``noise_std_m`` 0 adds nothing; ``range_max_m`` 0 keeps
    every return. Returns a new float array: negative ranges after noise become 0, returns past
    ``range_max_m`` become +inf (no return)."""
    if noise_std_m < 0.0 or not math.isfinite(noise_std_m):
        raise ValueError(f"scan noise std must be >= 0, got {noise_std_m}")
    if range_max_m < 0.0 or not math.isfinite(range_max_m):
        raise ValueError(f"scan range max must be >= 0, got {range_max_m}")
    out = np.asarray(ranges, dtype=float).copy()
    if noise_std_m > 0.0:
        out = out + rng.normal(0.0, noise_std_m, size=out.shape)
        np.maximum(out, 0.0, out=out)
    if range_max_m > 0.0:
        out[out > range_max_m] = math.inf
    return out
