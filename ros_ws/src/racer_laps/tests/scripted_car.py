"""A scripted car for the lap gate tests: a car on a loop whose length is set per lap, run through
`LapMemory` with a scripted matcher (no ROS, no hypothesis, so the L3 job's system python can import
it). Used by test_gate_v21.py, test_gate_v21_properties.py and test_lap1_launch.py."""

from __future__ import annotations

import math

import pytest
import racer_laps.gate as gate_module
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
            # The scripted matcher stands in for the start-scan match of gate v2.1 and these scans
            # are sparse (no displaced scan 0.4 m back): the v3 additions are off here, they have
            # their own tests (test_gate_v3.py).
            ring_enabled=False,
            unique_shift_m=0.0,
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

    def lap(
        self,
        length_m: float,
        delay_m: float = 0.41,
        skip_scans: bool = False,
        scan_at_line: bool = False,
    ):  # type: ignore[no-untyped-def]
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
            if scan_at_line:  # a third scan, on the line: the only one the innovation gate takes
                self.move_to(line)  # when the crossing is further than the band from L
                assert self.scan(result(0.0)) is None
        self.move_to(line + delay_m)
        closed = self.scan(None)
        if closed is not None:
            self.line = line
        return closed


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
