"""L1, F tests of brief 50b: written first, red on PR 210's head (the gate that winds by the
steering yaw and decides the alias verdict per lap).

* A ring whose car starts turned 0.23 rad against the track tangent (the attitude change of a
  standing start): the steering yaw turns 2 pi minus 0.23 over the first true lap, so waiting for
  2 pi of it closes lap 1 late. Position, not yaw, is what a lap is.
* A ring with the steering gain 3 percent high and low: the yaw over- or under-reads by that much
  every lap; the pair rate does not depend on the gain.
* The alias verdict belongs to the place: two agreeing unique matches before the line do not place
  the line (a reference needs six unique matches), and ambiguous matches that follow latch it
  aliased, so the lap closes by winding."""

from __future__ import annotations

import pytest
import racer_laps.gate as gate_module
from scripted_car import Car
from test_gate import result
from v3_support import RING, RING_SPEED, lap_s, run


@pytest.mark.parametrize("clockwise", [False, True])
def test_f_lap_1_of_a_ring_whose_car_starts_turned_by_0_23_rad_is_within_1_percent(
    clockwise: bool,
) -> None:
    _mem, laps = run(RING, 1.3, RING_SPEED, launch_yaw_rad=0.23, clockwise=clockwise)
    assert laps, "no lap closed"
    assert abs(laps[0].lap_time_s / lap_s(RING, RING_SPEED) - 1.0) <= 0.01, laps[0].lap_time_s


@pytest.mark.parametrize("gain", [1.03, 0.97])
def test_f_a_steering_gain_error_of_3_percent_keeps_every_lap_after_the_launch_within_1_percent(
    gain: float,
) -> None:
    _mem, laps = run(RING, 4.5, RING_SPEED, steer_scale=gain)
    truth = lap_s(RING, RING_SPEED)
    assert len(laps) >= 4
    for r in laps[1:]:
        assert abs(r.lap_time_s / truth - 1.0) <= 0.01, (gain, r.lap_time_s, truth)


def test_f_two_agreeing_unique_matches_do_not_place_the_line_and_ambiguous_ones_latch_the_verdict(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    car = Car(monkeypatch, ring_enabled=True, unique_shift_m=0.4)
    flags = iter([True, True, False, False, False])
    monkeypatch.setattr(
        car.gate,
        "_classify",
        lambda *a, **k: gate_module.UNIQUE if next(flags) else gate_module.AMBIGUOUS,
    )
    length = 5.2
    car.steer_for(length)
    car.move_to(length - 0.2)
    assert car.scan(result(-0.2)) is None
    car.move_to(length - 0.1)
    assert car.scan(result(-0.1)) is None
    car.move_to(length + 0.31)
    assert car.scan(None) is None  # two unique matches are not proof: the closure is held
    for x in (0.35, 0.4, 0.45):
        car.move_to(length + x)
        closed = car.scan(result(x))
        assert closed is None or closed.line_source == "winding"
    assert car.gate.start_aliased
    car.move_to(length + 0.6)
    closed = car.scan(None)
    if closed is None:
        car.move_to(length + 0.9)
        closed = car.scan(None)
    assert closed is not None and closed.line_source == "winding"
