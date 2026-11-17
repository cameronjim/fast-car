"""A scripted car for the external lap source tests (brief 51): wheel odometry at 50 Hz at a set
speed, the steering that turns 2 pi per lap length, a source status once a second, and events the
test places by hand. Plain numpy and the standard library, so the L3 job's system python imports
it. Used by test_external_gate.py, test_engine_external.py and the property tests."""

from __future__ import annotations

import math

from racer_laps.engine import LapConfig, LapMemory
from racer_laps.external_gate import ExternalConfig, ExternalLapGate
from racer_laps.gate import GateConfig
from racer_laps.lap_record import STATE_DRIVING
from synth import WHEELBASE_M

DT = 0.02
LAP_M = 5.2


def odom_model(gain: float = 1.0) -> GateConfig:
    return GateConfig(wheelbase_m=WHEELBASE_M, yaw_gain_table=((0.0, gain),))


def steering_for(length_m: float) -> float:
    """tan(delta) = 2 pi wheelbase / length: the heading turns 2 pi over `length_m`."""
    return math.atan(2.0 * math.pi * WHEELBASE_M / length_m)


class Rig:
    """The bare gate, driven by hand."""

    def __init__(
        self,
        lap_m: float = LAP_M,
        speed: float = 1.0,
        cfg: ExternalConfig | None = None,
        gain: float = 1.0,
        ready: bool = True,
        start: bool = True,
        status_period_s: float | None = 1.0,
        source: str = "camera",
    ) -> None:
        self.gate = ExternalLapGate(cfg or ExternalConfig(), odom_model(gain), source)
        self.speed = speed
        self.k = 0  # odometry tick
        self.status_period_s = status_period_s
        self.next_status_s = 0.0
        self.state, self.laps, self.sim = ("running" if ready else "waiting"), 0, 1.0
        self.gate.on_steering(steering_for(lap_m))
        self.gate.on_odom(0.0, speed)
        if status_period_s is not None:
            self.deliver_status(0.0)
        if start:
            assert self.gate.set_start(0.0)

    @property
    def t(self) -> float:
        return self.k * DT

    def deliver_status(self, t: float) -> None:
        self.gate.on_source_status(t, self.state, self.laps, self.sim)
        if self.status_period_s is not None:
            self.next_status_s = t + self.status_period_s

    def drive_until(self, t_end: float) -> None:
        while (self.k + 1) * DT <= t_end + 1e-9:
            self.k += 1
            t = self.k * DT
            if self.status_period_s is not None and t >= self.next_status_s - 1e-9:
                self.deliver_status(t)
            self.gate.on_odom(t, self.speed)

    def drive_to_total(self, total_m: float) -> None:
        while self.gate.total_m < total_m - 1e-9:
            self.drive_until(self.t + DT)

    def event(self, t_cross: float, lap_no: int, latency_s: float = 0.16):  # type: ignore[no-untyped-def]
        """Deliver an event: arrives `latency_s` after its crossing time; odometry runs to the
        arrival first."""
        arrival = t_cross + latency_s
        self.drive_until(arrival)
        return self.gate.on_event(arrival, lap_no, t_cross)


def memory(
    source: str = "camera",
    mode: str = "v0",
    ext: ExternalConfig | None = None,
    **kw: object,
) -> LapMemory:
    """A `LapMemory` on an external source, lap-1 cap 0.5, ceiling 1.0."""
    cfg = LapConfig(
        lap_source=source,
        lap_mode=mode,
        gate=odom_model(),
        external=ext,
        v_cap_start_mps=0.5,
        v_cap_ceiling_mps=1.0,
        **kw,  # type: ignore[arg-type]
    )
    return LapMemory(cfg)


class EngineRig:
    """`LapMemory` on an external source, a car on a loop, clean laps (the planner stays in
    DRIVING with clearance 0.3)."""

    def __init__(
        self,
        lap_m: float = LAP_M,
        speed: float = 1.0,
        source: str = "camera",
        mode: str = "v0",
        ready: bool = True,
        ext: ExternalConfig | None = None,
        status_period_s: float | None = 1.0,
    ) -> None:
        self.mem = memory(source, mode, ext)
        self.lap_m = lap_m
        self.speed = speed
        self.k = 0
        self.status_period_s = status_period_s
        self.next_status_s = 0.0
        self.state = "running" if ready else "waiting"
        self.results: list = []  # type: ignore[type-arg]
        self.caps: list[float] = []
        self.mem.on_drive(steering_for(lap_m))
        if status_period_s is not None:
            self.mem.on_lap_source_status(0.0, self.state, 0, 1.0)
        self.mem.on_planner_status(STATE_DRIVING, 0.3)
        self.mem.on_wheel_odom(0.0, speed)
        assert self.mem.gate is not None and self.mem.gate.started

    @property
    def t(self) -> float:
        return self.k * DT

    def drive_until(self, t_end: float) -> None:
        while (self.k + 1) * DT <= t_end + 1e-9:
            self.k += 1
            t = self.k * DT
            if self.status_period_s is not None and t >= self.next_status_s - 1e-9:
                self.mem.on_lap_source_status(t, self.state, 0, 1.0)
                self.next_status_s = t + self.status_period_s
            self.mem.on_wheel_odom(t, self.speed)
            self.mem.on_planner_status(STATE_DRIVING, 0.3)
            self.caps.append(self.mem.guidance().v_cap_mps)

    def event(self, t_cross: float, lap_no: int, latency_s: float = 0.16):  # type: ignore[no-untyped-def]
        arrival = t_cross + latency_s
        self.drive_until(arrival)
        r = self.mem.on_lap_source_event(arrival, lap_no, t_cross)
        if r is not None:
            self.results.append(r)
        return r

    def laps(self, n: int, first_no: int = 1) -> None:
        """`n` nominal laps, one event per lap length of driving."""
        for i in range(n):
            self.event(self.lap_m / self.speed * (len(self.results) + 1), first_no + i)
