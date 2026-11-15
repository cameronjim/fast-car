"""L1/L2: the slope limit on neighbouring station shifts of `fused-v1` (brief 38a)."""

from __future__ import annotations

import numpy as np
import pytest
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st
from racer_track_perception import fuse
from racer_track_perception import track as tr
from test_fuse import straight

SETTINGS = settings(
    max_examples=300,
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow, HealthCheck.data_too_large],
)
STEP = tr.STATION_SPACING_M
shifts = st.lists(st.floats(-0.35, 0.35), min_size=2, max_size=16).map(np.array)
slopes = st.sampled_from([0.05, 0.10, 0.15, 0.20, 0.30, 1.0])


@SETTINGS
@given(d=shifts, slope=slopes)
def test_the_envelope_never_grows_a_shift_keeps_signs_and_meets_the_bound(d, slope) -> None:
    out = fuse.limit_shift_slope(d, slope)
    assert (np.abs(out) <= np.abs(d) + 1e-12).all()
    assert (out * d >= -1e-15).all()  # same sign or zero
    assert (np.abs(np.diff(out)) <= slope * STEP + 1e-9).all()


@SETTINGS
@given(d=shifts)
def test_a_large_slope_changes_nothing_bit_for_bit(d) -> None:
    assert np.array_equal(fuse.limit_shift_slope(d, 1e9), d)


@SETTINGS
@given(d=shifts, slope=slopes)
def test_the_envelope_is_the_largest_for_same_sign_neighbours(d, slope) -> None:
    """On a profile of one sign the result is the Lipschitz envelope itself: lowering any station
    further than that is never needed, raising it would break the bound or exceed the input."""
    d = np.abs(d)
    out = fuse.limit_shift_slope(d, slope)
    step = slope * STEP
    for i in range(len(d)):
        raised = out[i] + 1e-6
        ok = (
            raised <= d[i]
            and (i == 0 or raised - out[i - 1] <= step)
            and (i == len(d) - 1 or raised - out[i + 1] <= step)
        )
        assert not ok


def test_an_isolated_shifted_station_is_pulled_to_the_bound() -> None:
    d = np.zeros(9)
    d[4] = 0.3
    out = fuse.limit_shift_slope(d, 0.10)
    assert out[4] == pytest.approx(0.10 * STEP)  # one step above its unmoved neighbours
    assert np.count_nonzero(out) == 1


def test_a_ramp_inside_the_slope_is_kept() -> None:
    d = np.linspace(0.0, 0.1, 6)  # 0.02 per station = slope 0.1
    assert np.allclose(fuse.limit_shift_slope(d, 0.10), d)
    assert np.allclose(fuse.limit_shift_slope(d, 0.05), np.minimum(d, 0.05 * STEP * np.arange(6)))


def test_opposite_neighbours_are_brought_within_the_bound() -> None:
    out = fuse.limit_shift_slope(np.array([0.3, -0.3, 0.3]), 0.10)
    assert (np.abs(np.diff(out)) <= 0.10 * STEP + 1e-12).all()
    assert out[0] >= 0.0 >= out[1] and out[2] >= 0.0


def test_fuse_track_applies_the_limit_and_a_huge_slope_reproduces_the_unlimited_output() -> None:
    geo = straight(12, hl=0.9, hr=0.9)
    net = straight(12)
    net.y_m[5] = 0.3  # one station the net pulls aside
    limited = fuse.fuse_track(geo, net, fuse.FuseConfig(append_far=False, max_shift_slope=0.10))
    assert limited.y_m[5] == pytest.approx(0.10 * STEP, abs=1e-6)
    a = fuse.fuse_track(geo, net, fuse.FuseConfig(append_far=False, max_shift_slope=1e9))
    b = fuse.fuse_track(geo, net, fuse.FuseConfig(append_far=False, max_shift_slope=1e12))
    assert a.y_m[5] > 0.05  # the unlimited shift is much larger than the limited one
    for name in ("x_m", "y_m", "half_width_left_m", "half_width_right_m", "sigma_m"):
        assert np.array_equal(getattr(a, name), getattr(b, name)), name


def test_the_edge_margin_still_holds_under_the_limit() -> None:
    geo = straight(12, hl=0.3, hr=0.3)
    net = straight(12, y=0.3)
    out = fuse.fuse_track(geo, net, fuse.FuseConfig(append_far=False, max_shift_slope=0.05))
    assert (out.half_width_left_m[:12] >= 0.15 - 1e-6).all()
    assert (out.half_width_right_m[:12] >= 0.15 - 1e-6).all()


def test_the_slope_is_part_of_the_validated_config() -> None:
    cfg = fuse.FuseConfig(max_shift_slope=0.15)
    assert fuse.FuseConfig.from_dict(cfg.as_dict()).max_shift_slope == 0.15
    for bad in (0.0, -0.1, float("nan"), True, "0.1"):
        with pytest.raises(ValueError, match="max_shift_slope"):
            fuse.FuseConfig.from_dict({**cfg.as_dict(), "max_shift_slope": bad})
    raw = cfg.as_dict()
    del raw["max_shift_slope"]
    with pytest.raises(ValueError):
        fuse.FuseConfig.from_dict(raw)
