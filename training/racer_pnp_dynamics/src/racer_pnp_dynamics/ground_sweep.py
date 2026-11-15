"""Item 5 of GP2e-L: a random sweep of the validity box for a contract 2 model (addendum 1 section
2.6): every state finite, alpha and rho inside their bounds, zero ground motion and zero yaw rate
at zero wheel speed."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .ground_motion import GroundMotion, alpha_rho, step_v2
from .model import Nominal
from .vehicle import Vehicle

SWEEP_SAMPLES = 100_000
SWEEP_STEPS = 30
SWEEP_SEED = 20261114
BOUND_TOL = 1e-12


@dataclass(frozen=True)
class SweepV2:
    samples: int
    nonfinite: int  # samples with a state that is not finite
    alpha_violations: int  # samples with alpha outside its bounds at some step
    rho_violations: int  # samples with |rho| above rho_max at some step
    zero_speed_violations: int  # samples whose pose moved in one step from zero wheel speed

    @property
    def passed(self) -> bool:
        total = (
            self.nonfinite
            + self.alpha_violations
            + self.rho_violations
            + self.zero_speed_violations
        )
        return total == 0

    def to_dict(self) -> dict[str, object]:
        return {**self.__dict__, "passed": self.passed}


def sweep_v2(
    params: Nominal,
    vehicle: Vehicle,
    gm: GroundMotion,
    v_max_mps: float,
    samples: int = SWEEP_SAMPLES,
    steps: int = SWEEP_STEPS,
    seed: int = SWEEP_SEED,
) -> SweepV2:
    """`samples` random states and commands in the validity box (speed 0 to v_max, steering inside
    the limits, a new random command every step) rolled `steps` steps, plus one step from rest."""
    rng = np.random.default_rng(seed)
    lo, hi = vehicle.steer_min_rad, vehicle.steer_max_rad
    state = np.zeros((samples, 5))
    state[:, 2] = rng.uniform(-np.pi, np.pi, samples)
    state[:, 3] = rng.uniform(0.0, v_max_mps, samples)
    state[:, 4] = rng.uniform(lo, hi, samples)
    controls = np.stack(
        [rng.uniform(0.0, v_max_mps, (samples, steps)), rng.uniform(lo, hi, (samples, steps))],
        axis=-1,
    )
    c = gm.constants
    bad_alpha = np.zeros(samples, dtype=bool)
    bad_rho = np.zeros(samples, dtype=bool)
    finite = np.ones(samples, dtype=bool)
    with np.errstate(all="ignore"):
        for i in range(steps):
            alpha, rho = alpha_rho(gm, state, controls[:, i, :], vehicle)
            in_alpha = (alpha >= c.alpha_min - BOUND_TOL) & (alpha <= c.alpha_max + BOUND_TOL)
            bad_alpha |= ~in_alpha
            bad_rho |= ~(np.abs(rho) <= c.rho_max + BOUND_TOL)
            state = step_v2(state, controls[:, i, :], params, vehicle, gm)
            finite &= np.isfinite(state).all(axis=1)
        rest = np.zeros((samples, 5))
        rest[:, 2] = rng.uniform(-np.pi, np.pi, samples)
        rest[:, 4] = rng.uniform(lo, hi, samples)
        after = step_v2(rest, controls[:, 0, :], params, vehicle, gm)
        moved = (after[:, :3] != rest[:, :3]).any(axis=1)
    return SweepV2(
        samples,
        int((~finite).sum()),
        int(bad_alpha.sum()),
        int(bad_rho.sum()),
        int(moved.sum()),
    )
