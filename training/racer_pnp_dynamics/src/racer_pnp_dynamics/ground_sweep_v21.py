"""Item 6 of GP2f-L: a random sweep of the validity box for a contract 2.1 model (pre-registration
v0.5 section 6): every state finite, alpha inside its bounds, gamma inside its bounds, rho inside
its bound, zero ground motion and zero yaw rate at zero wheel speed. Same recipe as
`ground_sweep.sweep_v2` (30 steps, a new random command every step, plus one step from rest), with
gamma added."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .ground_motion_v21 import GroundMotionV21, alpha_gamma_rho, step_v21
from .ground_sweep import BOUND_TOL, SWEEP_SEED, SWEEP_STEPS
from .model import Nominal
from .vehicle import Vehicle


@dataclass(frozen=True)
class SweepV21:
    samples: int
    nonfinite: int  # samples with a state that is not finite
    alpha_violations: int
    gamma_violations: int
    rho_violations: int
    zero_speed_violations: int  # samples whose pose moved in one step from zero wheel speed

    @property
    def violations(self) -> int:
        return (
            self.nonfinite
            + self.alpha_violations
            + self.gamma_violations
            + self.rho_violations
            + self.zero_speed_violations
        )

    @property
    def passed(self) -> bool:
        return self.violations == 0

    def to_dict(self) -> dict[str, object]:
        return {**self.__dict__, "passed": self.passed}


def sweep_v21(
    params: Nominal,
    vehicle: Vehicle,
    gm: GroundMotionV21,
    v_max_mps: float,
    samples: int,
    steps: int = SWEEP_STEPS,
    seed: int = SWEEP_SEED,
) -> SweepV21:
    """`samples` random states and commands in the validity box (speed 0 to v_max, steering inside
    the limits) rolled `steps` steps, plus one step from rest."""
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
    bad_gamma = np.zeros(samples, dtype=bool)
    bad_rho = np.zeros(samples, dtype=bool)
    finite = np.ones(samples, dtype=bool)
    with np.errstate(all="ignore"):
        for i in range(steps):
            alpha, gamma, rho = alpha_gamma_rho(gm, state, controls[:, i, :], vehicle)
            bad_alpha |= ~((alpha >= c.alpha_min - BOUND_TOL) & (alpha <= c.alpha_max + BOUND_TOL))
            bad_gamma |= ~((gamma >= c.gamma_min - BOUND_TOL) & (gamma <= c.gamma_max + BOUND_TOL))
            bad_rho |= ~(np.abs(rho) <= c.rho_max + BOUND_TOL)
            state = step_v21(state, controls[:, i, :], params, vehicle, gm)
            finite &= np.isfinite(state).all(axis=1)
        rest = np.zeros((samples, 5))
        rest[:, 2] = rng.uniform(-np.pi, np.pi, samples)
        rest[:, 4] = rng.uniform(lo, hi, samples)
        after = step_v21(rest, controls[:, 0, :], params, vehicle, gm)
        moved = (after[:, :3] != rest[:, :3]).any(axis=1)
    return SweepV21(
        samples,
        int((~finite).sum()),
        int(bad_alpha.sum()),
        int(bad_gamma.sum()),
        int(bad_rho.sum()),
        int(moved.sum()),
    )
