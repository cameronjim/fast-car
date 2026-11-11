"""L2: for any state and control within wide bounds the step stays finite, v >= 0 and delta
inside the vehicle_params steering limits (hypothesis)."""

from __future__ import annotations

import numpy as np
from hypothesis import given, settings
from hypothesis import strategies as st
from racer_pnp_dynamics.model import MlpWeights, Residual, rollout, step, zero_weights
from racer_pnp_dynamics.vehicle import load_vehicle

VEHICLE = load_vehicle()
finite = {"allow_nan": False, "allow_infinity": False}
RNG = np.random.default_rng(7)
RESIDUAL = Residual(
    True,
    MlpWeights(*(RNG.normal(scale=0.5, size=a.shape) for a in zero_weights().arrays())),
    np.zeros(4),
    np.ones(4),
    np.array([1.0, 1.0]),
)


@given(
    x=st.floats(-100, 100, **finite),
    y=st.floats(-100, 100, **finite),
    yaw=st.floats(-50, 50, **finite),
    v=st.floats(0, 10, **finite),
    delta=st.floats(VEHICLE.steer_min_rad, VEHICLE.steer_max_rad, **finite),
    speed_cmd=st.floats(-20, 20, **finite),
    steer_cmd=st.floats(-3, 3, **finite),
    gain_l=st.floats(0.5, 1.5, **finite),
    gain_r=st.floats(0.5, 1.5, **finite),
    tau=st.floats(0.02, 1.0, **finite),
    use_residual=st.booleans(),
)
@settings(max_examples=300, deadline=None)
def test_step_is_finite_and_inside_limits(
    x, y, yaw, v, delta, speed_cmd, steer_cmd, gain_l, gain_r, tau, use_residual
):
    from racer_pnp_dynamics.model import PLACEHOLDER_NOMINAL

    p = PLACEHOLDER_NOMINAL.replace(
        steer_gain_left=gain_l, steer_gain_right=gain_r, steer_tau_s=tau, speed_tau_s=tau
    )
    out = step(
        [x, y, yaw, v, delta],
        [speed_cmd, steer_cmd],
        p,
        VEHICLE,
        RESIDUAL if use_residual else None,
    )
    assert np.all(np.isfinite(out))
    assert out[3] >= 0.0
    assert VEHICLE.steer_min_rad - 1e-12 <= out[4] <= VEHICLE.steer_max_rad + 1e-12


@given(
    speed=st.floats(-5, 5, **finite),
    steer=st.floats(-2, 2, **finite),
    v0=st.floats(0, 3, **finite),
)
@settings(max_examples=100, deadline=None)
def test_rollouts_stay_finite(speed, steer, v0):
    from racer_pnp_dynamics.model import PLACEHOLDER_NOMINAL

    states = rollout(
        [0, 0, 0, v0, 0], np.tile([speed, steer], (60, 1)), PLACEHOLDER_NOMINAL, VEHICLE
    )
    assert np.all(np.isfinite(states)) and np.all(states[:, 3] >= 0.0)
