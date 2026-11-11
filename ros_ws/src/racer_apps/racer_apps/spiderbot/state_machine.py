"""The spiderbot state machine: patrol, stop, face the person, alert, hold, resume (roadmap 5.3).

Pure logic, no ROS, no clock of its own: every call to `step` carries the time, so a test drives
it with a list of numbers and gets the same answer every run. spiderbot_node is the plumbing that
feeds it and turns its Command into /drive_raw (through the safety node, like every driver).

PHASES

  PATROL          the patrol command (racer_control's tracker on the waypoint loop) passes
                  through. A rule Trigger (the pose rule held for N frames) starts an engagement
                  unless the cooldown after the last one is still running.
  STOPPING        stop. Leaves when the car is slow enough (a measured speed at or below
                  `stop_speed_eps_mps` after at least `min_stop_s`) or `stop_settle_s` has passed,
                  whichever is first, so a missing speed reading cannot hang it.
  FACING          creep towards the person's bearing at low speed until it is within
                  `face_tolerance_rad` for `face_settle_s`. The car cannot turn on the spot (it
                  steers like a car), so this is a short forward arc, and it ends early without
                  moving at all when the person's range is unknown or under `min_face_range_m`.
                  It also ends when the person is not seen for `face_lost_s` or after
                  `face_timeout_s`. The ALERT is raised either way, with facing_ok saying which.
  HOLD            stopped for `hold_s` after the alert (the operator's window to look, and the
                  place a confirm step will sit), then back to PATROL with a cooldown of
                  `cooldown_s` during which triggers are ignored, so the same person does not
                  re-trigger the moment the car pulls away.
  NO_PERCEPTION   (only with `require_perception`) the detector has gone quiet: stop, and return
                  to PATROL when it has been back for `perception_resume_s`. A patrol with a
                  dead camera pipeline is not doing its job and is not looking where it drives.

COMMANDS. PASS_THROUGH (publish the patrol command), STOP (zero speed, wheels straight), FACE
(creep, steer towards `bearing_rad`; None when the last sighting is too old to steer on, in which
case the node holds still). The machine never says go faster than the patrol command, and it
cannot command anything but stop or a creep: speeds, steering limits and rate limits are the
node's, from vehicle_params, and the safety node is downstream of all of it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

from racer_apps.spiderbot.pose_rules import Trigger


class Phase(str, Enum):
    PATROL = "patrol"
    STOPPING = "stopping"
    FACING = "facing"
    HOLD = "hold"
    NO_PERCEPTION = "no_perception"


class CommandKind(str, Enum):
    PASS_THROUGH = "pass_through"
    STOP = "stop"
    FACE = "face"


class EventKind(str, Enum):
    ENGAGE = "engage"
    ALERT = "alert"
    RESUME = "resume"
    PERCEPTION_LOST = "perception_lost"
    PERCEPTION_RESTORED = "perception_restored"


@dataclass(frozen=True)
class Command:
    kind: CommandKind
    bearing_rad: float | None = None  # FACE only


@dataclass(frozen=True)
class Observation:
    """The person being faced, as last seen: node-clock time, vehicle-frame bearing, range."""

    t_s: float
    bearing_rad: float
    range_m: float
    range_valid: bool


@dataclass(frozen=True)
class Event:
    kind: EventKind
    t_s: float
    data: dict = field(default_factory=dict)


@dataclass(frozen=True)
class MachineConfig:
    stop_settle_s: float = 1.0
    min_stop_s: float = 0.2
    stop_speed_eps_mps: float = 0.05
    face_enabled: bool = True
    face_tolerance_rad: float = 0.12
    face_settle_s: float = 0.3
    face_lost_s: float = 1.0
    face_timeout_s: float = 5.0
    face_obs_max_age_s: float = 0.5
    min_face_range_m: float = 1.0
    hold_s: float = 10.0
    cooldown_s: float = 20.0
    require_perception: bool = True
    perception_resume_s: float = 0.5

    def __post_init__(self) -> None:
        positive = (
            "stop_settle_s",
            "face_tolerance_rad",
            "face_settle_s",
            "face_lost_s",
            "face_timeout_s",
            "face_obs_max_age_s",
            "min_face_range_m",
            "hold_s",
        )
        non_negative = ("min_stop_s", "stop_speed_eps_mps", "cooldown_s", "perception_resume_s")
        for name in positive:
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"{name} must be positive and finite, got {value}")
        for name in non_negative:
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f"{name} must be non-negative and finite, got {value}")


class SpiderbotMachine:
    def __init__(self, config: MachineConfig | None = None) -> None:
        self.config = config or MachineConfig()
        self.phase = Phase.PATROL
        self._phase_t = math.nan
        self._cooldown_until = -math.inf
        self._trigger: Trigger | None = None
        self._last_bearing = math.nan
        self._last_range = math.nan
        self._last_range_valid = False
        self._last_obs_t = math.nan
        self._aligned_since = math.nan
        self._fresh_since = math.nan
        self._engagements = 0

    @property
    def engagements(self) -> int:
        return self._engagements

    def _set_phase(self, phase: Phase, t_s: float) -> None:
        self.phase = phase
        self._phase_t = t_s

    def _aligned(self) -> bool:
        return math.isfinite(self._last_bearing) and (
            abs(self._last_bearing) <= self.config.face_tolerance_rad
        )

    def _alert(self, t_s: float, facing_ok: bool) -> Event:
        trig = self._trigger
        assert trig is not None
        event = Event(
            EventKind.ALERT,
            t_s,
            {
                "person": trig.person,
                "frames": trig.frames,
                "trigger_t_s": trig.t_s,
                "facing_ok": facing_ok,
                "final_bearing_rad": self._last_bearing,
                "engagement": self._engagements,
            },
        )
        self._set_phase(Phase.HOLD, t_s)
        return event

    def step(
        self,
        t_s: float,
        trigger: Trigger | None,
        observation: Observation | None,
        perception_fresh: bool,
        speed_mps: float | None,
    ) -> tuple[Command, list[Event]]:
        cfg = self.config
        events: list[Event] = []
        stop = Command(CommandKind.STOP)

        if self.phase == Phase.PATROL:
            if cfg.require_perception and not perception_fresh:
                self._set_phase(Phase.NO_PERCEPTION, t_s)
                self._fresh_since = math.nan
                events.append(Event(EventKind.PERCEPTION_LOST, t_s))
                return stop, events
            if trigger is not None and t_s >= self._cooldown_until:
                self._engagements += 1
                self._trigger = trigger
                self._last_bearing = trigger.person.bearing_rad
                self._last_range = trigger.person.range_m
                self._last_range_valid = trigger.person.range_valid
                self._last_obs_t = t_s
                self._aligned_since = math.nan
                self._set_phase(Phase.STOPPING, t_s)
                events.append(
                    Event(
                        EventKind.ENGAGE,
                        t_s,
                        {"person": trigger.person, "frames": trigger.frames},
                    )
                )
                return stop, events
            return Command(CommandKind.PASS_THROUGH), events

        if self.phase == Phase.NO_PERCEPTION:
            if perception_fresh:
                if not math.isfinite(self._fresh_since):
                    self._fresh_since = t_s
                if t_s - self._fresh_since >= cfg.perception_resume_s:
                    self._set_phase(Phase.PATROL, t_s)
                    events.append(Event(EventKind.PERCEPTION_RESTORED, t_s))
            else:
                self._fresh_since = math.nan
            return stop, events

        if self.phase == Phase.STOPPING:
            elapsed = t_s - self._phase_t
            slow = speed_mps is not None and abs(speed_mps) <= cfg.stop_speed_eps_mps
            if elapsed >= cfg.stop_settle_s or (slow and elapsed >= cfg.min_stop_s):
                if not cfg.face_enabled or self._aligned():
                    events.append(self._alert(t_s, self._aligned()))
                else:
                    self._set_phase(Phase.FACING, t_s)
            return stop, events

        if self.phase == Phase.FACING:
            fresh = observation is not None and (
                0.0 <= t_s - observation.t_s <= cfg.face_obs_max_age_s
            )
            if fresh:
                assert observation is not None
                self._last_obs_t = max(self._last_obs_t, observation.t_s)
                self._last_bearing = observation.bearing_rad
                self._last_range = observation.range_m
                self._last_range_valid = observation.range_valid
            aligned_now = fresh and self._aligned()
            if aligned_now:
                if not math.isfinite(self._aligned_since):
                    self._aligned_since = t_s
                if t_s - self._aligned_since >= cfg.face_settle_s:
                    events.append(self._alert(t_s, True))
                    return stop, events
            else:
                self._aligned_since = math.nan
            can_creep = self._last_range_valid and self._last_range >= cfg.min_face_range_m
            if (
                not can_creep
                or t_s - self._last_obs_t > cfg.face_lost_s
                or t_s - self._phase_t > cfg.face_timeout_s
            ):
                events.append(self._alert(t_s, self._aligned()))
                return stop, events
            if not fresh:
                return stop, events
            return Command(CommandKind.FACE, self._last_bearing), events

        # HOLD
        if t_s - self._phase_t >= cfg.hold_s:
            self._set_phase(Phase.PATROL, t_s)
            self._cooldown_until = t_s + cfg.cooldown_s
            events.append(Event(EventKind.RESUME, t_s, {"engagement": self._engagements}))
            return Command(CommandKind.PASS_THROUGH), events
        return stop, events
