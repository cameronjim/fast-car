"""L1/L4, F tests of brief 50c: written first, red on PR 210's head.

* A near miss is not proof. On RD (a ring with a box the start scan sees at grazing range) the
  displaced scan at an alias's pose fails STRONG by a hair (violation 0.021 to 0.031 against 0.02)
  while the match itself passes at 0.014 to 0.019: the head booked that as a unique match, six of
  them proved the start scan and aliases placed the line.
* Above 2.0 m/s scans are more than 0.2 m apart, no scan lies within 0.1 m of 0.4 m back, every
  match was "unknown" and v3 closed no lap where v2.1 did (the L3 harness found it at 3.0 m/s, and
  was slowed to 1.5 m/s, which hid it)."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest
from racer_laps.gate import AMBIGUOUS
from synth import Stadium
from test_gate_v3 import feed_gate, match_to_start
from test_l4_ring_fixture import FIXTURES, replay
from v3_support import STADIUM, lap_s, run


def test_f_a_near_miss_of_the_strong_limit_is_not_a_unique_match() -> None:
    ring = Stadium(straight_m=0.0, lane_m=1.1, island_r_m=0.4)
    gate, pts = feed_gate(ring, 1.2, gate_kw={"ring_enabled": False})
    res = match_to_start(gate, pts, 0.0)
    # the match passes STRONG at 0.015, the displaced scan at the same pose fails it at 0.025:
    # a gap of 0.010, far under what a real place shows (0.040 and more)
    res = replace(res, freespace_violation=0.015)  # type: ignore[type-var]
    gate._freespace = lambda *a, **k: 0.025  # type: ignore[method-assign]
    assert gate._classify(res, gate._start, gate.total_m) == AMBIGUOUS  # type: ignore[arg-type]


def test_f_rd_ccw_dump_every_lap_by_winding_inside_the_tolerance_none_missed() -> None:
    """`c_RD_ccw_11`, laps 1 to 5. On PR 210's head the start scan is proven by near misses and
    lap 2 closes more than 1.0 s early."""
    truth = np.load(FIXTURES / "rd_ccw_11.npz")["truth_lap_s"]
    laps = replay("rd_ccw_11")
    assert len(laps) == len(truth) == 5
    assert [r.line_source for r in laps] == ["winding"] * 5
    assert not any(r.missed_gate for r in laps)
    for r, t in zip(laps, truth, strict=True):
        assert abs(r.lap_time_s - float(t)) <= max(0.3, 0.04 * float(t)), (r.lap_time_s, t)


@pytest.mark.parametrize("speed", [2.5, 3.0])
def test_f_a_stadium_above_2_m_per_s_closes_every_lap_by_start_scan_as_v21_does(
    speed: float,
) -> None:
    _mem, laps = run(STADIUM, 3.4, speed)
    assert [r.lap_index for r in laps] == [1, 2, 3], [r.lap_index for r in laps]
    assert {r.line_source for r in laps} == {"start_scan"}
    for r in laps:
        assert abs(r.lap_time_s - lap_s(STADIUM, speed)) <= 0.3, r.lap_time_s
