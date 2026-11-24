"""L1 and L2: the Python reference of the planner's online speed adaptation (adaptation.py): the
same limits, gating, reset and convergence as racer_planner's test_speed_adaptation.cpp, the
corrected step equal to the contract step at zero correction, and the causal replay."""

from __future__ import annotations

import math

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st
from racer_pnp_dynamics import adaptation as ad
from racer_pnp_dynamics.model import step


def feed(est, bias, gain, n, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    for _ in range(n):
        ap = rng.uniform(-1.5, 1.5)
        est.update(
            ap,
            ap * (1 + gain) + bias + rng.normal(0, noise) if noise else ap * (1 + gain) + bias,
            1.0,
            0.02,
            True,
        )


def test_disabled_does_nothing() -> None:
    est = ad.SpeedAdaptation(ad.AdaptationParams(enabled=False))
    assert not est.update(0.5, 0.0, 1.0, 0.02, True)
    assert est.correction() == (0.0, 0.0)


def test_converges_on_a_known_plant() -> None:
    est = ad.SpeedAdaptation()
    feed(est, -0.2, 0.15, 2000, 0.05, 1)
    b, g = est.correction()
    assert b == pytest.approx(-0.2, abs=0.02) and g == pytest.approx(0.15, abs=0.03)


def test_hard_limits_hold() -> None:
    est = ad.SpeedAdaptation()
    feed(est, 3.0, 0.0, 500)
    assert est.correction()[0] == est.p.bias_max_mps2 and est.at_limit
    est = ad.SpeedAdaptation()
    feed(est, 0.0, -0.8, 500)
    assert est.correction()[1] == -est.p.gain_max


def test_reset_and_bad_samples_zero_it() -> None:
    est = ad.SpeedAdaptation()
    feed(est, -0.3, 0.1, 300)
    assert est.correction() != (0.0, 0.0)
    assert not est.update(math.nan, 0.0, 1.0, 0.02, True)
    assert est.correction() == (0.0, 0.0) and est.resets == 1


def test_gated_samples_are_skipped() -> None:
    est = ad.SpeedAdaptation()
    for args in [
        (0.5, 0.0, 1.0, 0.02, False),
        (0.5, 0.0, 0.1, 0.02, True),
        (0.5, 0.0, 1.0, 0.001, True),
        (0.5, 0.0, 1.0, 0.5, True),
        (0.0, 10.0, 1.0, 0.02, True),
    ]:
        assert not est.update(*args)
    assert est.skipped == 5 and est.updates == 0


@settings(max_examples=50, deadline=None)
@given(
    st.lists(
        st.tuples(
            st.floats(-20, 20),
            st.floats(-20, 20),
            st.floats(-1, 3),
            st.floats(0, 0.3),
            st.booleans(),
        ),
        max_size=300,
    )
)
def test_always_bounded_and_finite(samples) -> None:
    est = ad.SpeedAdaptation()
    for s in samples:
        est.update(*s)
        b, g = est.correction()
        assert math.isfinite(b) and math.isfinite(g)
        assert abs(b) <= est.p.bias_max_mps2 and abs(g) <= est.p.gain_max


def test_zero_correction_step_is_the_contract_step(toy_vehicle, toy_nominal) -> None:
    rng = np.random.default_rng(4)
    s = np.column_stack(
        [rng.normal(size=(50, 3)), rng.uniform(0, 2, 50), rng.uniform(-0.3, 0.3, 50)]
    )
    u = np.column_stack([rng.uniform(-0.5, 2, 50), rng.uniform(-0.4, 0.4, 50)])
    z = np.zeros(50)
    np.testing.assert_array_equal(
        ad.step_corrected(s, u, toy_nominal, toy_vehicle, None, z, z),
        step(s, u, toy_nominal, toy_vehicle),
    )


def test_correction_scales_dv_only_while_driving(toy_vehicle, toy_nominal) -> None:
    s = np.array([0.0, 0.0, 0.0, 0.5, 0.0])
    u = np.array([1.0, 0.0])
    plain = step(s, u, toy_nominal, toy_vehicle)
    corr = ad.step_corrected(s, u, toy_nominal, toy_vehicle, None, np.array(-0.2), np.array(0.1))
    assert corr[3] - 0.5 == pytest.approx((plain[3] - 0.5) * 1.1 - 0.2 * 0.05)
    assert corr[4] == plain[4] and corr[0] == plain[0]
    stop = np.array([0.0, 0.0])
    assert (
        ad.step_corrected(s, stop, toy_nominal, toy_vehicle, None, np.array(-0.2), np.array(0.1))[3]
        == step(s, stop, toy_nominal, toy_vehicle)[3]
    )


def test_replay_learns_a_slow_plant_causally(toy_vehicle, toy_nominal) -> None:
    dt = 0.02
    t = np.arange(0, 60, dt)
    cmds = np.column_stack([0.9 + 0.4 * np.sin(0.26 * t), np.zeros_like(t)])
    v = np.zeros_like(t)
    v[0] = 0.6
    for i in range(1, len(t)):
        s = np.array([0.0, 0.0, 0.0, v[i - 1], 0.0])
        a_pred = (step(s, cmds[i - 1], toy_nominal, toy_vehicle)[3] - v[i - 1]) / 0.05
        v[i] = v[i - 1] + (0.8 * a_pred - 0.1) * dt
    corr = ad.run_corrections(t, v, cmds, toy_nominal, toy_vehicle, None)
    assert corr[0].tolist() == [0.0, 0.0]  # nothing learned before the second sample
    assert corr[-1, 0] == pytest.approx(-0.1, abs=0.02)
    assert corr[-1, 1] == pytest.approx(-0.2, abs=0.02)
    v_gap = v.copy()
    v_gap[100] = np.nan  # a gap in the odometry starts a new chain, it does not reset
    corr_gap = ad.run_corrections(t, v_gap, cmds, toy_nominal, toy_vehicle, None)
    assert np.all(np.isfinite(corr_gap))
