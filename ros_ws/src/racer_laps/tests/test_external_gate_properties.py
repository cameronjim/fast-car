"""L2 (hypothesis): properties of the lap source rule of brief 51. Left out where hypothesis is absent
(conftest.py), as the other property files are.

1. Any sequence of events (dropped, duplicated, early, late, renumbered, any latency up to 5 s) over
   any odometry of N true laps: the cap never rises at a doubtful lap, and the cap after every lap
   is at most the cap that the confirmed laps alone could have earned (one step per confirmed clean
   lap, from the lap-1 cap).
2. With a perfect source and exact odometry, any speed 0.3 to 2.0 m/s and any heading gain error
   within plus or minus 10 percent: N events give N confirmed laps.
3. A confirmed lap has a length within the band of the true lap length when the true laps are equal
   (no merged or split lap is confirmed), once a confirmed lap exists to measure against. A first
   lap with no reference to measure against is judged by the minimum length and the heading window
   alone (1.2 rad of 2 pi, so a split first lap of 0.81 to 1.0 lap lengths can be confirmed): the
   bound asserted for it is that window. Reported in the PR, not tuned."""

from __future__ import annotations

import math
from dataclasses import dataclass

import pytest
from external_support import DT, LAP_M, EngineRig, Rig
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st

SETTINGS = settings(
    max_examples=40,
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow, HealthCheck.data_too_large],
)
CAP0, STEP, CEILING = 0.5, 0.1, 1.0


@dataclass(frozen=True)
class Action:
    kind: str  # ok, drop, dup, early, late, renumber, false
    amount: float  # the size of the early shift (s), the lateness (s), the renumber, the fraction


actions = st.one_of(
    st.just(Action("ok", 0.0)),
    st.just(Action("drop", 0.0)),
    st.builds(lambda a: Action("dup", a), st.floats(0.05, 1.0)),
    st.builds(lambda a: Action("early", a), st.floats(0.0, 2.0)),
    st.builds(lambda a: Action("late", a), st.floats(0.0, 5.0)),
    st.builds(lambda a: Action("renumber", a), st.integers(-3, 3).map(float)),
    st.builds(lambda a: Action("false", a), st.floats(0.1, 0.95)),
)


def build_events(plan: list[Action], speed: float) -> list[tuple[float, int, float]]:
    """(arrival, number, crossing) for the true crossings of `plan`, numbered as a camera that does
    not know what it missed would (consecutively over what it published); a duplicate repeats the
    number of the event it copies."""
    lap_s = LAP_M / speed
    raw: list[tuple[float, float, int, bool]] = []  # arrival, crossing, renumber shift, duplicate
    shift = 0
    for k, a in enumerate(plan, start=1):
        cross, latency = k * lap_s, 0.16
        if a.kind == "drop":
            continue
        if a.kind == "early":
            cross = max(DT, cross - a.amount)
        if a.kind == "late":
            latency += a.amount
        if a.kind == "renumber":
            shift += int(a.amount)
        if a.kind == "false":
            frac_t = (k - 1 + a.amount) * lap_s
            raw.append((frac_t + 0.16, frac_t, shift, False))
        raw.append((cross + latency, cross, shift, False))
        if a.kind == "dup":
            raw.append((cross + latency + a.amount, cross, shift, True))
    raw.sort()
    events: list[tuple[float, int, float]] = []
    number = 0
    for arrival, cross, sh, dup in raw:
        if not dup:
            number += 1
        events.append((arrival, number + sh, cross))
    return events


@SETTINGS
@given(
    plan=st.lists(actions, min_size=1, max_size=9),
    speed=st.floats(0.5, 2.0),
    first_good=st.booleans(),
)
def test_the_cap_never_rises_at_a_doubtful_lap_nor_above_what_confirmed_laps_earn(
    plan: list[Action], speed: float, first_good: bool
) -> None:
    if first_good:
        plan = [Action("ok", 0.0), *plan]
    rig = EngineRig(speed=speed)
    confirmed_clean = 0
    for arrival, number, cross in build_events(plan, speed):
        rig.drive_until(arrival)
        result = rig.mem.on_lap_source_event(arrival, number, cross)
        if result is None:
            continue
        if result.missed_gate:
            assert result.next_cap_mps <= result.cap_mps + 1e-9
            assert not result.stored and not result.clean
        else:
            confirmed_clean += int(result.clean)
        earned = min(CEILING, CAP0 + STEP * confirmed_clean)
        assert result.next_cap_mps <= earned + 1e-9
        assert result.next_cap_mps >= CAP0 - 1e-9


@SETTINGS
@given(
    speed=st.floats(0.3, 2.0),
    gain_error=st.floats(-0.10, 0.10),
    n=st.integers(1, 5),
    latency=st.floats(0.0, 2.0),
)
def test_a_perfect_source_and_exact_odometry_confirm_every_lap(
    speed: float, gain_error: float, n: int, latency: float
) -> None:
    rig = Rig(speed=speed, gain=1.0 + gain_error)
    lap_s = LAP_M / speed
    for k in range(1, n + 1):
        crossing = rig.event(k * lap_s, k, latency)
        assert crossing is not None and not crossing.missed, rig.gate.doubtful
        assert crossing.length_m == pytest.approx(LAP_M, abs=0.02)
    assert rig.gate.confirmed == n and rig.gate.doubtful == []


@SETTINGS
@given(
    fractions=st.lists(st.one_of(st.just(1.0), st.floats(0.05, 3.2)), min_size=1, max_size=10),
)
def test_a_confirmed_lap_is_never_a_merged_or_split_lap(fractions: list[float]) -> None:
    """Events at the given multiples of the true lap length after the previous event (1.0 is a
    true crossing; 2.0 a merged lap, 0.4 a split one), numbered consecutively."""
    rig = Rig()
    lap_s = LAP_M / 1.0
    t = 0.0
    reference_exists = False
    for number, f in enumerate(fractions, start=1):
        t += f * lap_s
        crossing = rig.event(t, number, 0.16)
        if crossing is None or crossing.missed:
            continue
        ratio = crossing.length_m / LAP_M
        if reference_exists:
            assert abs(crossing.length_m - rig.gate.l_ref_m) <= max(0.5, 0.08 * rig.gate.l_ref_m)
            assert abs(crossing.length_m - LAP_M) <= max(0.5, 0.08 * LAP_M) + 0.5 + 1e-6
        else:
            # no reference yet: the heading window (1.2 rad of 2 pi) is the only bound
            assert abs(ratio - 1.0) <= 1.2 / (2.0 * math.pi) + 1e-6
        reference_exists = True
