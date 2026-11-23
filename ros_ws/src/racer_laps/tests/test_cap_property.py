"""L2: for any sequence of lap outcomes the cap stays inside its bounds and moves one step a lap."""

from __future__ import annotations

from hypothesis import given, settings
from hypothesis import strategies as st
from racer_laps.cap_schedule import CapSchedule

EPS = 1e-9


@settings(max_examples=300, deadline=None)
@given(
    start=st.floats(0.05, 2.0),
    step=st.floats(0.01, 0.5),
    extra=st.floats(0.0, 3.0),
    outcomes=st.lists(st.booleans(), max_size=80),
)
def test_cap_bounds_and_step_size(
    start: float, step: float, extra: float, outcomes: list[bool]
) -> None:
    ceiling = start + extra
    sched = CapSchedule(start, step, ceiling)
    prev = sched.cap_mps
    assert prev == start
    for clean in outcomes:
        cap = sched.finish_lap(clean)
        assert start - EPS <= cap <= ceiling + EPS
        assert abs(cap - prev) <= step + EPS
        # a clean lap never lowers the cap and an unclean one never raises it
        assert (cap >= prev - EPS) if clean else (cap <= prev + EPS)
        prev = cap


@settings(max_examples=200, deadline=None)
@given(outcomes=st.lists(st.booleans(), min_size=1, max_size=60))
def test_rise_after_an_unclean_lap_needs_two_clean_laps(outcomes: list[bool]) -> None:
    sched = CapSchedule(0.5, 0.1, 5.0)
    since_unclean = None  # clean laps since the last unclean one
    for clean in outcomes:
        before = sched.cap_mps
        after = sched.finish_lap(clean)
        if not clean:
            since_unclean = 0
        elif since_unclean is not None:
            since_unclean += 1
            if since_unclean == 1:
                assert after <= before + 1e-12


@settings(max_examples=300, deadline=None)
@given(outcomes=st.lists(st.sampled_from(["clean", "unclean", "hold"]), max_size=80))
def test_a_held_lap_never_moves_the_cap_and_a_doubtful_one_never_raises_it(
    outcomes: list[str],
) -> None:
    """Owner decision 24: a qualifying camera-only doubtful lap never lowers the cap; any
    doubtful lap (held or not) never raises it; and a hold leaves the future untouched."""
    sched = CapSchedule(0.5, 0.1, 5.0)
    without_holds = CapSchedule(0.5, 0.1, 5.0)
    for o in outcomes:
        before = sched.cap_mps
        after = sched.finish_lap(o == "clean", hold=o == "hold")
        if o == "hold":
            assert after == before
        else:
            without_holds.finish_lap(o == "clean")
            if o == "unclean":
                assert after <= before + EPS
        # holds are invisible: the cap is the one the schedule has without them
        assert after == without_holds.cap_mps
