"""The lap counter's time gate is a floor, the distance gate is the double-count guard (brief 39).

Synthetic similarity sequences: a signature is built so its dot product with the reference is
exactly the wanted similarity, so these tests do not depend on any image maths.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from racer_perception.lap_counter import LapConfig, LapCounter

DIM = 8
REF = np.eye(DIM, dtype=np.float32)[0]
FRAME_DT_S = 0.1  # well above the 2 Hz of the recorded bags, so timing is not the variable
SPEED_MPS = 1.0


def sig(similarity: float) -> np.ndarray:
    out = np.zeros(DIM, dtype=np.float32)
    out[0] = similarity
    out[1] = math.sqrt(max(0.0, 1.0 - similarity * similarity))
    return out


def bump(t: float, centre: float, width: float = 0.3) -> float:
    """Similarity 0.95 at `centre`, about 0.1 a couple of widths away."""
    return 0.1 + 0.85 * math.exp(-(((t - centre) / width) ** 2))


def drive(
    cfg: LapConfig,
    crossings: list[float],
    speed: float = SPEED_MPS,
    end: float | None = None,
) -> tuple[LapCounter, list]:
    """Reference at t=0, then a similarity bump at each time in `crossings`."""
    counter = LapCounter(cfg)
    counter.update_odom(0.0, speed, 0.0)
    counter.update_signature(0.0, REF)
    events = []
    end = end if end is not None else crossings[-1] + 1.0
    n = round(end / FRAME_DT_S)
    for i in range(1, n + 1):
        t = i * FRAME_DT_S
        counter.update_odom(t, speed, speed * t)
        s = max(bump(t, c) for c in crossings)
        ev = counter.update_signature(t, sig(s))
        if ev is not None:
            events.append(ev)
    return counter, events


def test_default_time_floor_is_two_seconds():
    assert LapConfig().min_lap_time_s == 2.0
    assert LapConfig().min_lap_distance_m == 3.0


def test_5p6_s_laps_count_every_lap_with_the_new_rule():
    crossings = [5.6 * k for k in range(1, 9)]
    _, events = drive(LapConfig(), crossings)
    assert [e.lap for e in events] == list(range(1, 9))
    for e in events:
        assert e.lap_time_s == pytest.approx(5.6, abs=0.15)


def test_5p6_s_laps_count_every_second_lap_with_the_old_6_s_rule():
    """The bug as the hypothesis states it (GP3 counted 10 of 20)."""
    crossings = [5.6 * k for k in range(1, 9)]
    _, events = drive(LapConfig(min_lap_time_s=6.0), crossings)
    assert len(events) == 4
    for e in events:
        assert e.lap_time_s == pytest.approx(11.2, abs=0.15)


def test_14_s_laps_are_unchanged_by_the_floor():
    crossings = [14.0 * k for k in range(1, 5)]
    new = drive(LapConfig(), crossings)[1]
    old = drive(LapConfig(min_lap_time_s=6.0), crossings)[1]
    assert [e.lap for e in new] == [1, 2, 3, 4]
    assert [(e.lap, round(e.crossing_time_s, 6)) for e in new] == [
        (e.lap, round(e.crossing_time_s, 6)) for e in old
    ]


def test_a_double_peak_one_second_after_a_crossing_is_not_counted():
    # The second peak is under the 2.0 s floor and under the 3 m distance gate.
    _, events = drive(LapConfig(), [5.6, 6.6, 11.2], speed=1.0)
    assert [e.lap for e in events] == [1, 2]
    assert events[0].crossing_time_s == pytest.approx(5.6, abs=0.15)
    assert events[1].crossing_time_s == pytest.approx(11.2, abs=0.15)


def test_a_slow_first_lap_then_fast_laps_counts_every_lap():
    crossings = [13.9]
    for _ in range(5):
        crossings.append(crossings[-1] + 4.6)
    # 4.6 s at 1 m/s is 4.6 m per lap, over the 3 m gate.
    _, events = drive(LapConfig(), crossings)
    assert [e.lap for e in events] == [1, 2, 3, 4, 5, 6]


def test_a_second_peak_after_2p5_s_but_under_3_m_is_not_counted():
    # 2.5 s at 0.5 m/s is 1.25 m of wheel distance: past the floor, under the distance gate.
    _, events = drive(LapConfig(), [10.0, 12.5], speed=0.5)
    assert [e.lap for e in events] == [1]  # the 10.0 s crossing (5 m) counts, 12.5 s does not
    # With the distance gate off the second peak counts: the distance gate is what refused it.
    _, events = drive(LapConfig(min_lap_distance_m=0.0), [10.0, 12.5], speed=0.5)
    assert [e.lap for e in events] == [1, 2]


@pytest.mark.parametrize("bad", [-0.1, float("nan"), float("inf")])
def test_time_floor_validation(bad):
    with pytest.raises(ValueError):
        LapCounter(LapConfig(min_lap_time_s=bad))


def test_a_zero_time_floor_is_allowed():
    LapCounter(LapConfig(min_lap_time_s=0.0))


@pytest.mark.parametrize("speed,period", [(0.6, 5.6), (1.0, 2.2), (1.3, 4.0), (0.4, 9.0)])
def test_the_count_never_exceeds_wheel_distance_over_the_distance_gate(speed, period):
    """L2: whatever the periodic similarity, laps <= distance / min_lap_distance_m."""
    cfg = LapConfig()
    crossings = [period * k for k in range(1, 12)]
    counter, events = drive(cfg, crossings, speed=speed)
    assert len(events) <= counter.distance_m / cfg.min_lap_distance_m
    for e in events:
        assert e.distance_m >= cfg.min_lap_distance_m
        assert e.lap_time_s >= cfg.min_lap_time_s


def test_the_count_bound_holds_for_random_peaks():
    rng = np.random.default_rng(7)
    cfg = LapConfig()
    for _ in range(20):
        gaps = rng.uniform(0.5, 8.0, size=15)
        crossings = list(np.cumsum(gaps))
        speed = float(rng.uniform(0.2, 1.5))
        counter, events = drive(cfg, crossings, speed=speed)
        assert len(events) <= counter.distance_m / cfg.min_lap_distance_m
