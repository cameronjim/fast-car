"""L2: properties of gate v2.1 (brief 48 items 3 and 4), through the engine with a scripted matcher.
Needs hypothesis (a dev dependency): conftest.py leaves this file out where it is absent."""

from __future__ import annotations

import pytest
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st
from scripted_car import BAND_M, LAP_M, Car, park_and_scan


@settings(
    max_examples=40,
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow, HealthCheck.function_scoped_fixture],
)
@given(
    length=st.floats(min_value=4.8, max_value=6.2),
    delay=st.floats(min_value=0.3, max_value=0.6),
)
def test_missed_equals_the_crossing_length_above_the_estimate_plus_the_band(
    length: float, delay: float
) -> None:
    """For any decision delay from 0.3 to 0.6 m after the line (the `past` rule), for crossings
    inside the innovation band and for those beyond it (accepted once the station is uncertain)."""
    # the boundary itself is within the odometry quantisation (0.02 m): not decided by a property
    if abs(length - (LAP_M + BAND_M)) < 0.04:
        return
    with pytest.MonkeyPatch.context() as mp:
        car = Car(mp)
        assert car.lap(LAP_M) is not None and car.lap(LAP_M) is not None
        closed = car.lap(length, delay_m=delay, scan_at_line=True)
        assert closed is not None, (length, delay)
        assert closed.missed_gate == (length > LAP_M + BAND_M), (length, delay)


@settings(
    max_examples=25,
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow, HealthCheck.function_scoped_fixture],
)
@given(past=st.floats(min_value=0.0, max_value=0.25))
def test_any_standstill_inside_the_closing_distance_closes_near_the_line(past: float) -> None:
    """Parked anywhere from the line to 0.25 m past it (before the 0.3 m of the `past` rule)."""
    with pytest.MonkeyPatch.context() as mp:
        car = Car(mp)
        car.steer_for(LAP_M)
        closed, t_parked = park_and_scan(car, LAP_M, LAP_M + past, 0.0, seconds=4.0)
        assert closed is not None, past
        assert abs(closed.length_m - LAP_M) <= 0.05
        assert t_parked is None or car.t - t_parked <= car.gate.cfg.consensus_wait_s + 0.2
