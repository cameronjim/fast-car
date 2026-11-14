"""L2 (properties) of the scan deskew core: any ranges (NaN, inf, empty, all invalid, negative) and
any twist in range give an output of the input's length that is finite or inf, never negative,
never below range_min. Needs hypothesis (a dev dependency); conftest.py leaves this file out of a
colcon run where it is absent."""

from __future__ import annotations

import math

import numpy as np
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st
from racer_track_perception import deskew
from racer_track_perception.deskew import DeskewConfig, Twist


def cfg(stamp_is: str = "sweep_start", order: str = "index_ascending", **kw) -> DeskewConfig:
    return DeskewConfig(stamp_is=stamp_is, beam_order=order, **kw)


# -- L2: properties ---------------------------------------------------------------------------------------

range_values = st.one_of(
    st.floats(min_value=-5.0, max_value=40.0),
    st.just(math.nan),
    st.just(math.inf),
    st.just(-math.inf),
    st.just(0.0),
)


@settings(max_examples=200, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(
    ranges=st.lists(range_values, min_size=0, max_size=60),
    v=st.floats(min_value=-3.0, max_value=3.0),
    w=st.floats(min_value=-6.0, max_value=6.0),
    angle_min=st.floats(min_value=-4.0, max_value=4.0),
    inc=st.one_of(
        st.floats(min_value=-0.2, max_value=-1e-3), st.floats(min_value=1e-3, max_value=0.2)
    ),
    scan_time=st.one_of(st.floats(min_value=0.0, max_value=0.3), st.just(math.nan)),
    stamp_is=st.sampled_from(deskew.STAMP_IS_VALUES),
    order=st.sampled_from(deskew.BEAM_ORDERS),
    reference=st.sampled_from(deskew.REFERENCES),
)
def test_any_ranges_and_any_twist_give_a_well_formed_output(
    live, ranges, v, w, angle_min, inc, scan_time, stamp_is, order, reference
) -> None:
    range_min, range_max = 0.05, 16.0
    out, offset = deskew.deskew_scan(
        np.array(ranges, dtype=np.float64),
        angle_min,
        inc,
        scan_time,
        0.0,
        Twist(v, w),
        live.mount,
        cfg(stamp_is, order, reference=reference),
        range_min=range_min,
        range_max=range_max,
    )
    assert out.shape == (len(ranges),)
    assert not np.isnan(out).any()
    finite = out[np.isfinite(out)]
    assert (finite >= range_min).all() and (finite <= range_max).all()
    assert math.isfinite(offset)


@settings(max_examples=100, deadline=None)
@given(
    values=st.lists(st.floats(min_value=0.1, max_value=10.0), min_size=1, max_size=50),
    v=st.floats(min_value=-3.0, max_value=3.0),
    w=st.floats(min_value=-6.0, max_value=6.0),
)
def test_a_zero_sweep_time_never_changes_the_scan(live, values, v, w) -> None:
    r = np.array(values)
    out, offset = deskew.deskew_scan(r, 0.0, 0.05, 0.0, 0.0, Twist(v, w), live.mount, cfg())
    assert np.array_equal(out, r) and offset == 0.0
