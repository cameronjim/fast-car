"""L1: gate v2.1 (brief 48 items 3 and 4), through the engine with a scripted matcher.

* A crossing is missed only if its own wheel length is above the lap length estimate plus the band,
  not because the live station passed that while the gate was still deciding the closure (3 of 40
  laps of the 2026-11-16 car runs: a 5.2 m lap at 1 m/s, the closure is decided 0.3 m or 0.5 s
  after the line, 0.5 m is the band).
* A truly missed crossing with a trusted lap length still closes at the next real crossing.
* A car parked just past the line closes the lap within the wait rule of standstill.
"""

from __future__ import annotations

import pytest
import racer_laps.gate as gate_module
from racer_laps.engine import LapResult
from scripted_car import (
    BAND_M,
    CEILING,
    LAP_M,
    STANDSTILL_AT_LEAST,
    Car,
    park_and_scan,
)
from test_gate import result


def three_laps_decided_past_the_band(
    mp: pytest.MonkeyPatch, before: int
) -> tuple[Car, list[LapResult], int]:
    """`before` ordinary laps of 5.2 m, then three laps of 5.33 m whose closure is decided at
    station 5.74 m: 0.41 m after the line, 0.04 m above the length estimate plus the band.
    Returns the car, the three results and the index of the first tick of the three laps."""
    car = Car(mp)
    for _ in range(before):
        assert car.lap(LAP_M) is not None
    first_tick = len(car.caps)
    laps = []
    for _ in range(3):
        closed = car.lap(5.33)
        assert closed is not None
        laps.append(closed)
    return car, laps, first_tick


def test_a_crossing_decided_past_the_band_is_not_a_missed_gate(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Trusted length 5.2 m, band 0.5 m, crossing at 5.33 m decided at station 5.74 m."""
    car = Car(monkeypatch)
    assert car.lap(LAP_M) is not None and car.lap(LAP_M) is not None
    assert car.gate.l_ref_trusted and car.gate.l_ref_m == pytest.approx(LAP_M, abs=0.03)
    closed = car.lap(5.33)
    assert closed is not None
    assert closed.length_m == pytest.approx(5.33, abs=0.04)
    assert closed.length_m + 0.41 > LAP_M + BAND_M  # decided past the band, crossing inside it
    assert not closed.missed_gate and closed.clean
    assert not any(car.uncertain)  # the live flag stayed down while the closure was pending


def test_three_laps_decided_past_the_band_keep_the_d_v0_cap_at_the_ceiling(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    car, laps, first_tick = three_laps_decided_past_the_band(monkeypatch, before=5)
    assert car.mem.schedule.cap_mps == pytest.approx(CEILING)
    assert [r.missed_gate for r in laps] == [False] * 3 and all(r.clean for r in laps)
    assert [r.next_cap_mps for r in laps] == [pytest.approx(CEILING)] * 3
    # the guidance cap never fell to the lap-1 cap while the closure was decided
    assert len(car.caps) - first_tick > 700  # three laps of about 265 ticks
    assert min(car.caps[first_tick:]) == pytest.approx(CEILING)


def test_the_live_flag_is_up_with_no_candidate_and_down_while_one_is_pending(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    car = Car(monkeypatch)
    assert car.lap(LAP_M) is not None and car.lap(LAP_M) is not None
    car.steer_for(LAP_M)
    line = car.line + LAP_M
    car.move_to(line + BAND_M + 0.2)  # no candidate: the line passed unseen
    assert car.gate.station_uncertain
    assert car.mem.status()["station_uncertain"] is True
    # a candidate appears (it cannot be placed before the next lap, but it is pending): down
    car.gate._cands.append(
        gate_module._Candidate(car.t, car.total, gate_module.MatchResult(0.0, 0, 0, 0.02, 0.9, 99))
    )
    assert not car.gate.station_uncertain
    assert car.mem.status()["station_uncertain"] is False
    car.gate._cands.clear()
    assert car.gate.station_uncertain  # and back up when none is left


def test_a_truly_missed_crossing_with_a_trusted_length_still_closes_at_the_next_one(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """With a trusted length the innovation gate rejects every candidate that is a lap away from
    where the line was expected: after a crossing that passed unseen the gate never closes again.
    While the station is uncertain (the latch), the gate must take the untrusted rule: at least
    `min_lap_length_m` from the last line."""
    car = Car(monkeypatch)
    assert car.lap(LAP_M) is not None and car.lap(LAP_M) is not None
    assert car.gate.l_ref_trusted
    missed = car.lap(2.0 * LAP_M)  # the first line passed unseen, the second is seen
    assert missed is not None, "the gate never closed again"
    assert missed.missed_gate and not missed.clean
    assert missed.length_m == pytest.approx(2.0 * LAP_M, abs=0.05)
    nxt = car.lap(LAP_M)
    assert nxt is not None and not nxt.missed_gate  # and it is back to normal one lap later


# -- the flag follows the crossing, not the decision (the properties are in test_gate_v21_properties.py) --


@pytest.mark.parametrize("length", [5.0, 5.33, 5.55, 5.9])
@pytest.mark.parametrize("v", [0.2, 0.3, 0.45])
def test_missed_follows_the_crossing_when_the_wait_rule_decides_the_lap(
    monkeypatch: pytest.MonkeyPatch, length: float, v: float
) -> None:
    """A closure decided 0.5 s after the last candidate (delay below 0.3 m at these speeds)."""
    car = Car(monkeypatch)
    assert car.lap(LAP_M) is not None and car.lap(LAP_M) is not None
    line = car.line + length
    car.steer_for(length)
    car.move_to(line - 0.2)
    assert car.scan(result(-0.2)) is None
    car.move_to(line - 0.05)
    assert car.scan(result(-0.05)) is None
    car.move_to(line + 0.05, v=v)
    assert car.scan(result(0.05)) is None
    t_last = car.t
    closed = None
    while closed is None and car.t - t_last < 2.0:
        car.move_to(car.total + 0.02, v=v)
        closed = car.scan(None)
    assert closed is not None
    assert closed.missed_gate == (length > LAP_M + BAND_M)


# -- item 4: parked just past the line ----------------------------------------------------------


def test_a_car_parked_0_15_m_past_the_line_closes_the_lap_within_the_wait_and_one_scan(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    car = Car(monkeypatch)
    car.steer_for(LAP_M)
    closed, t_parked = park_and_scan(car, LAP_M, LAP_M + 0.15, 0.0, seconds=6.0)
    assert closed is not None, "the lap never closed (a candidate arrives on every scan)"
    assert t_parked is not None
    assert car.t - t_parked <= car.gate.cfg.consensus_wait_s + 0.1 + 0.03
    assert closed.length_m == pytest.approx(LAP_M, abs=0.05)  # the crossing is on the line


def test_a_creeping_car_does_not_use_the_standstill_rule(monkeypatch: pytest.MonkeyPatch) -> None:
    """Above STANDSTILL_MPS the wait rule is as before: it counts from the LATEST candidate, so a
    candidate on every scan keeps it from closing (the moving cases are unchanged)."""
    car = Car(monkeypatch)
    car.steer_for(LAP_M)
    closed, _ = park_and_scan(car, LAP_M, LAP_M + 0.15, STANDSTILL_AT_LEAST, seconds=2.0)
    assert closed is None
