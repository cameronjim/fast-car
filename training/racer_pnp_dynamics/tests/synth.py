"""Synthetic runs generated from KNOWN parameters with the reference model, with measurement noise
at the real rates (wheel speed at 50 Hz, yaw change per 0.1 s scan interval)."""

from __future__ import annotations

import numpy as np
from racer_pnp_dynamics.bags import GRID_DT_S, Run
from racer_pnp_dynamics.icp_motion import MotionSeries
from racer_pnp_dynamics.model import Nominal, step
from racer_pnp_dynamics.vehicle import Vehicle


def random_commands(
    rng: np.random.Generator, n: int, v_hi: float = 0.6, steer_hi: float = 0.45
) -> tuple[np.ndarray, np.ndarray]:
    speed = np.zeros(n)
    steer = np.zeros(n)
    k = 0
    while k < n:
        hold = int(rng.uniform(1.5, 3.5) * 50)
        speed[k : k + hold] = rng.uniform(0.15, v_hi) if rng.random() > 0.1 else 0.0
        k += hold
    k = 0
    while k < n:
        hold = int(rng.uniform(1.0, 3.0) * 50)
        steer[k : k + hold] = rng.uniform(-steer_hi, steer_hi)
        k += hold
    return speed, steer


def make_run(
    truth: Nominal,
    vehicle: Vehicle,
    seconds: float = 600.0,
    seed: int = 0,
    odom: bool = True,
    speed_noise: float = 0.01,
    yaw_noise: float = 0.003,
    name: str = "synthetic",
    wheel_over_read: float | None = None,
    ramp_mps2: float | None = None,
    dead_zone_mps: float = 0.0,
) -> tuple[Run, MotionSeries]:
    """`wheel_over_read`: ground speed over wheel speed at full lock (1.0 straight, falling
    linearly with |steering command|); the recorded wheel speed is the ground speed divided by
    it. `ramp_mps2` slews the speed command at that rate. `dead_zone_mps`: from rest the car does
    not move while the command is below it. All off by default."""
    rng = np.random.default_rng(seed)
    n = int(seconds * 50)
    speed_cmd, steer_cmd = random_commands(rng, n)
    if ramp_mps2 is not None:
        for k in range(1, n):
            speed_cmd[k] = speed_cmd[k - 1] + np.clip(
                speed_cmd[k] - speed_cmd[k - 1], -ramp_mps2 * GRID_DT_S, ramp_mps2 * GRID_DT_S
            )
    lag = round(truth.command_delay_s / GRID_DT_S)
    state = np.zeros(5)
    states = np.zeros((n, 5))
    for k in range(n):
        states[k] = state
        j = max(k - lag, 0)
        applied = speed_cmd[j]
        if state[3] < 1e-3 and applied < dead_zone_mps:
            applied = 0.0
        state = step(state, [applied, steer_cmd[j]], truth, vehicle, dt=GRID_DT_S)
    t = 1000.0 + np.arange(n) * GRID_DT_S
    ground = states[:, 3]
    if wheel_over_read is not None:
        lock = np.minimum(np.abs(steer_cmd) / vehicle.steer_max_rad, 1.0)
        ground = ground / (1.0 - (1.0 - wheel_over_read) * lock)
    wheel = ground + rng.normal(0.0, speed_noise, n) if odom else np.full(n, np.nan)
    run = Run(name, t, speed_cmd, steer_cmd, wheel, [])
    stride = round(0.1 / GRID_DT_S)
    idx = np.arange(0, n - stride, stride)
    dyaw = states[idx + stride, 2] - states[idx, 2] + rng.normal(0.0, yaw_noise, len(idx))
    motion = MotionSeries(
        t_s=t[idx],
        dt_s=np.full(len(idx), 0.1),
        dx_m=states[idx + stride, 0] - states[idx, 0],
        dy_m=states[idx + stride, 1] - states[idx, 1],
        dyaw_rad=dyaw,
        rms_m=np.zeros(len(idx)),
        inlier_share=np.ones(len(idx)),
        ok=np.ones(len(idx), dtype=bool),
    )
    return run, motion
