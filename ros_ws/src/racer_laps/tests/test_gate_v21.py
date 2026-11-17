"""L1 and L2: gate v2.1 (brief 48 items 3 and 4), through the engine with a scripted matcher.

* A crossing is missed only if its own wheel length is above the lap length estimate plus the band,
  not because the live station passed that while the gate was still deciding the closure (3 of 40
  laps of the 2026-11-16 car runs: a 5.2 m lap at 1 m/s, the closure is decided 0.3 m or 0.5 s
  after the line, 0.5 m is the band).
* A truly missed crossing with a trusted lap length still closes at the next real crossing.
* A car parked just past the line closes the lap within the wait rule of standstill.
"""

from __future__ import annotations

import math

import pytest
import racer_laps.gate as gate_module
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st
from racer_laps.engine import LapConfig, LapMemory, LapResult
from racer_laps.gate import GateConfig
from racer_laps.lap_record import STATE_DRIVING
from synth import WHEELBASE_M
from test_gate import PTS, Scripted, result

CEILING = 1.0
#: A speed at the standstill threshold itself (0.05 m/s, `gate.STANDSTILL_MPS`): not standing.
STANDSTILL_AT_LEAST = 0.05
LAP_M = 5.2  # the short loop of the 2026-11-16 bags
BAND_M = 0.5


class Car:
    """A car on a loop whose length is set per lap, run through `LapMemory` with the scripted
    matcher: wheel odometry at 50 Hz, the steering that turns 2 pi per lap length, scans only when
    the test says so."""

    def __init__(self, mp: pytest.MonkeyPatch) -> None:
        cfg = GateConfig(
            wheelbase_m=WHEELBASE_M,
            min_along_support=0,
            dead_reckoning_seed=False,
            seed_search=False,
            along_search_m=0.0,
        )
        self.mem = LapMemory(
            LapConfig(
                lap_source="start_scan", gate=cfg, v_cap_start_mps=0.5, v_cap_ceiling_mps=CEILING
            )
        )
        self.mp = mp
        mp.setattr(gate_module, "freespace_violation_share", lambda *a, **k: 0.0)
        self.t = 0.0
        self.line = 0.0  # wheel total of the last crossing, by construction
        self.caps: list[float] = []  # the guidance cap at every odometry tick
        self.uncertain: list[bool] = []
        self.mem.on_planner_status(STATE_DRIVING, 0.3)
        self.mem.on_wheel_odom(0.0, 0.0)
        assert self.mem.on_scan(0.0, PTS) is None
        assert self.gate.started

    @property
    def gate(self):  # type: ignore[no-untyped-def]
        assert self.mem.gate is not None
        return self.mem.gate

    @property
    def total(self) -> float:
        return float(self.gate.total_m)

    def steer_for(self, length_m: float) -> None:
        """tan(delta) = 2 pi L_wheelbase / length: the heading turns 2 pi over `length_m` at any
        speed."""
        self.mem.on_drive(math.atan(2.0 * math.pi * WHEELBASE_M / length_m))

    def step(self, v: float) -> None:
        """One 50 Hz odometry sample at speed v."""
        self.t += 0.02
        self.mem.on_wheel_odom(self.t, v)
        self.caps.append(self.mem.guidance().v_cap_mps)
        self.uncertain.append(bool(self.mem.status()["station_uncertain"]))

    def move_to(self, total_m: float, v: float = 1.0) -> None:
        while self.total < total_m - 1e-9:
            self.step(v)

    def scan(self, res) -> LapResult | None:  # type: ignore[no-untyped-def]
        self.mp.setattr(gate_module, "match_scans", Scripted([res]))
        self.t += 1e-6
        return self.mem.on_scan(self.t, PTS)

    def lap(self, length_m: float, delay_m: float = 0.41, skip_scans: bool = False):  # type: ignore[no-untyped-def]
        """Drive one lap whose crossing is `length_m` after the last one, with two agreeing
        candidates just before the line, and close it with the scan `delay_m` after the line (the
        `past` rule needs 0.3 m). With `skip_scans` the line passes unseen. Returns the LapResult
        of the closure, or None."""
        line = self.line + length_m
        self.steer_for(length_m)
        if not skip_scans:
            self.move_to(line - 0.2)
            assert self.scan(result(-0.2)) is None
            self.move_to(line - 0.1)
            assert self.scan(result(-0.1)) is None
        self.move_to(line + delay_m)
        closed = self.scan(None)
        if closed is not None:
            self.line = line
        return closed


def three_laps_decided_past_the_band(mp: pytest.MonkeyPatch, before: int) -> tuple[Car, list]:
    """`before` ordinary laps of 5.2 m, then three laps of 5.33 m whose closure is decided at
    station 5.74 m: 0.41 m after the line, 0.04 m above the length estimate plus the band."""
    car = Car(mp)
    for _ in range(before):
        assert car.lap(LAP_M) is not None
    laps = []
    for _ in range(3):
        closed = car.lap(5.33)
        assert closed is not None
        laps.append(closed)
    return car, laps


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
    car, laps = three_laps_decided_past_the_band(monkeypatch, before=5)
    assert car.mem.schedule.cap_mps == pytest.approx(CEILING)
    assert [r.missed_gate for r in laps] == [False] * 3 and all(r.clean for r in laps)
    assert [r.next_cap_mps for r in laps] == [pytest.approx(CEILING)] * 3
    # the guidance cap never fell to the lap-1 cap while the closure was decided
    ceiling_ticks = car.caps[-3 * 270 :]  # the three laps at 50 ticks per 1 m... generous
    assert min(ceiling_ticks) == pytest.approx(CEILING)


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


# -- L2: the flag follows the crossing, not the decision ----------------------------------------


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
    """For any decision delay from 0.3 to 0.6 m after the line (the `past` rule)."""
    # the boundary itself is within the odometry quantisation (0.02 m): not decided by a property
    if abs(length - (LAP_M + BAND_M)) < 0.04:
        return
    with pytest.MonkeyPatch.context() as mp:
        car = Car(mp)
        assert car.lap(LAP_M) is not None and car.lap(LAP_M) is not None
        closed = car.lap(length, delay_m=delay)
        assert closed is not None, (length, delay)
        assert closed.missed_gate == (length > LAP_M + BAND_M), (length, delay)


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


def park_and_scan(
    car: Car, line: float, park_at: float, v_park: float, seconds: float
) -> tuple[LapResult | None, float | None]:
    """Approach the line at 0.5 m/s, stop (speed `v_park`) at wheel total `park_at`, and send a
    scan with a candidate every 0.1 s throughout. Returns (closure or None, time parked)."""
    car.move_to(line - 0.5)
    t_end = car.t + 1.4 + seconds  # the approach takes 1.3 s
    t_scan = car.t
    t_parked: float | None = None
    while car.t < t_end:
        if car.total < park_at - 1e-9:
            car.step(0.5)
        else:
            t_parked = car.t if t_parked is None else t_parked
            car.step(v_park)
        if car.t >= t_scan:
            t_scan = car.t + 0.1
            closed = car.scan(result(car.total - line))
            if closed is not None:
                return closed, t_parked
    return None, t_parked


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
