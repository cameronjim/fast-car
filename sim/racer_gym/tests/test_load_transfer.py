"""L1/L2 tests for racer_gym.dynamics.load_transfer (claude-docs/12-testing.md)."""

from __future__ import annotations

import math

from hypothesis import given
from hypothesis import strategies as st
from racer_gym.dynamics.load_transfer import (
    GRAVITY_MPS2,
    lateral_grip_derate,
    longitudinal_load_transfer,
    static_axle_loads,
)

MASS_KG = 3.74
CG_HEIGHT_M = 0.074
LF_M = 0.15875
LR_M = 0.17145
TRACK_WIDTH_M = 0.25


# --------------------------------------------------------------------------------------
# L1: hand-computed cases
# --------------------------------------------------------------------------------------


def test_static_loads_sum_to_weight():
    loads = static_axle_loads(MASS_KG, LF_M, LR_M)
    assert math.isclose(loads.front_n + loads.rear_n, MASS_KG * GRAVITY_MPS2, rel_tol=1e-9)


def test_static_loads_hand_computed():
    # Front axle load is proportional to the REAR cg distance (closer cg -> more load).
    loads = static_axle_loads(MASS_KG, LF_M, LR_M)
    wheelbase = LF_M + LR_M
    expected_front = MASS_KG * GRAVITY_MPS2 * LR_M / wheelbase
    expected_rear = MASS_KG * GRAVITY_MPS2 * LF_M / wheelbase
    assert math.isclose(loads.front_n, expected_front, rel_tol=1e-9)
    assert math.isclose(loads.rear_n, expected_rear, rel_tol=1e-9)


def test_acceleration_shifts_load_to_rear():
    """Positive (forward) longitudinal acceleration: front decreases, rear increases."""
    static = static_axle_loads(MASS_KG, LF_M, LR_M)
    accelerating = longitudinal_load_transfer(MASS_KG, CG_HEIGHT_M, LF_M, LR_M, 5.0)
    assert accelerating.front_n < static.front_n
    assert accelerating.rear_n > static.rear_n


def test_braking_shifts_load_to_front():
    """Negative (braking) longitudinal acceleration: front increases, rear decreases."""
    static = static_axle_loads(MASS_KG, LF_M, LR_M)
    braking = longitudinal_load_transfer(MASS_KG, CG_HEIGHT_M, LF_M, LR_M, -5.0)
    assert braking.front_n > static.front_n
    assert braking.rear_n < static.rear_n


def test_longitudinal_transfer_hand_computed_magnitude():
    ax = 3.0
    wheelbase = LF_M + LR_M
    delta_n = MASS_KG * ax * CG_HEIGHT_M / wheelbase
    static = static_axle_loads(MASS_KG, LF_M, LR_M)
    loads = longitudinal_load_transfer(MASS_KG, CG_HEIGHT_M, LF_M, LR_M, ax)
    assert math.isclose(loads.front_n, static.front_n - delta_n, rel_tol=1e-9)
    assert math.isclose(loads.rear_n, static.rear_n + delta_n, rel_tol=1e-9)


def test_zero_acceleration_matches_static():
    static = static_axle_loads(MASS_KG, LF_M, LR_M)
    loads = longitudinal_load_transfer(MASS_KG, CG_HEIGHT_M, LF_M, LR_M, 0.0)
    assert math.isclose(loads.front_n, static.front_n, rel_tol=1e-9)
    assert math.isclose(loads.rear_n, static.rear_n, rel_tol=1e-9)


def test_extreme_braking_clips_at_zero_not_negative():
    loads = longitudinal_load_transfer(MASS_KG, CG_HEIGHT_M, LF_M, LR_M, -1000.0)
    assert loads.rear_n == 0.0
    assert loads.front_n >= 0.0


def test_lateral_derate_is_one_when_track_width_unknown():
    """Documented fallback: null track_width_m -> no lateral load transfer effect at all,
    identical to stock f1tenth_gym's (non-)treatment of this effect."""
    static = static_axle_loads(MASS_KG, LF_M, LR_M)
    derate = lateral_grip_derate(
        MASS_KG, CG_HEIGHT_M, static.front_n, static.rear_n, lat_accel_mps2=8.0, track_width_m=None
    )
    assert derate.front == 1.0
    assert derate.rear == 1.0


def test_lateral_derate_is_one_at_zero_lateral_accel():
    static = static_axle_loads(MASS_KG, LF_M, LR_M)
    derate = lateral_grip_derate(
        MASS_KG,
        CG_HEIGHT_M,
        static.front_n,
        static.rear_n,
        lat_accel_mps2=0.0,
        track_width_m=TRACK_WIDTH_M,
    )
    assert math.isclose(derate.front, 1.0, rel_tol=1e-9)
    assert math.isclose(derate.rear, 1.0, rel_tol=1e-9)


def test_lateral_derate_reduces_grip_when_cornering_with_known_track_width():
    static = static_axle_loads(MASS_KG, LF_M, LR_M)
    derate = lateral_grip_derate(
        MASS_KG,
        CG_HEIGHT_M,
        static.front_n,
        static.rear_n,
        lat_accel_mps2=8.0,
        track_width_m=TRACK_WIDTH_M,
    )
    assert 0.0 < derate.front < 1.0
    assert 0.0 < derate.rear < 1.0


def test_lateral_derate_is_one_when_static_total_load_is_zero():
    """Degenerate edge case (zero mass): nothing to redistribute, no derate."""
    derate = lateral_grip_derate(
        MASS_KG,
        CG_HEIGHT_M,
        static_front_n=0.0,
        static_rear_n=0.0,
        lat_accel_mps2=8.0,
        track_width_m=TRACK_WIDTH_M,
    )
    assert derate.front == 1.0
    assert derate.rear == 1.0


def test_lateral_derate_is_one_for_an_individually_unloaded_axle():
    """One axle statically unloaded (e.g. a degenerate cg position) but the other isn't:
    the unloaded axle's derate is trivially 1.0 (nothing to redistribute there)."""
    derate = lateral_grip_derate(
        MASS_KG,
        CG_HEIGHT_M,
        static_front_n=0.0,
        static_rear_n=MASS_KG * GRAVITY_MPS2,
        lat_accel_mps2=8.0,
        track_width_m=TRACK_WIDTH_M,
    )
    assert derate.front == 1.0
    assert 0.0 < derate.rear < 1.0


def test_lateral_derate_monotonically_decreases_with_lateral_accel():
    static = static_axle_loads(MASS_KG, LF_M, LR_M)
    derates = [
        lateral_grip_derate(
            MASS_KG,
            CG_HEIGHT_M,
            static.front_n,
            static.rear_n,
            lat_accel_mps2=ay,
            track_width_m=TRACK_WIDTH_M,
        ).front
        for ay in (0.0, 2.0, 4.0, 8.0, 12.0)
    ]
    assert derates == sorted(derates, reverse=True)


# --------------------------------------------------------------------------------------
# L2: property-based tests (hypothesis)
# --------------------------------------------------------------------------------------


@given(
    mass_kg=st.floats(min_value=1.0, max_value=50.0),
    cg_height_m=st.floats(min_value=0.01, max_value=0.5),
    lf_m=st.floats(min_value=0.05, max_value=1.0),
    lr_m=st.floats(min_value=0.05, max_value=1.0),
    ax=st.floats(min_value=-50.0, max_value=50.0, allow_nan=False),
)
def test_longitudinal_conserves_total_load(mass_kg, cg_height_m, lf_m, lr_m, ax):
    """The L2 property explicitly called for by claude-docs/12-testing.md: 'load transfer
    conserves total normal load' -- exactly, at any acceleration, including the saturated
    (one-axle-unloaded) regime (see longitudinal_load_transfer's docstring)."""
    static = static_axle_loads(mass_kg, lf_m, lr_m)
    loads = longitudinal_load_transfer(mass_kg, cg_height_m, lf_m, lr_m, ax)
    total_static = static.front_n + static.rear_n
    total_transferred = loads.front_n + loads.rear_n
    assert math.isclose(total_transferred, total_static, rel_tol=1e-6, abs_tol=1e-6)
    assert loads.front_n >= 0.0
    assert loads.rear_n >= 0.0


@given(
    mass_kg=st.floats(min_value=1.0, max_value=50.0),
    cg_height_m=st.floats(min_value=0.01, max_value=0.5),
    lf_m=st.floats(min_value=0.05, max_value=1.0),
    lr_m=st.floats(min_value=0.05, max_value=1.0),
    ay=st.floats(min_value=-50.0, max_value=50.0, allow_nan=False),
    track_width_m=st.one_of(st.none(), st.floats(min_value=0.05, max_value=1.0)),
)
def test_lateral_derate_always_in_valid_range(mass_kg, cg_height_m, lf_m, lr_m, ay, track_width_m):
    static = static_axle_loads(mass_kg, lf_m, lr_m)
    derate = lateral_grip_derate(
        mass_kg, cg_height_m, static.front_n, static.rear_n, ay, track_width_m
    )
    assert 0.0 < derate.front <= 1.0
    assert 0.0 < derate.rear <= 1.0
