"""L1: the spiderbot state machine, driven with plain numbers (no ROS, no clock)."""

from __future__ import annotations

import math

import pytest
from racer_apps.spiderbot.pose_rules import Person, Trigger
from racer_apps.spiderbot.state_machine import (
    CommandKind,
    EventKind,
    MachineConfig,
    Observation,
    Phase,
    SpiderbotMachine,
)

CFG = MachineConfig(
    stop_settle_s=1.0,
    min_stop_s=0.2,
    face_tolerance_rad=0.1,
    face_settle_s=0.3,
    face_lost_s=1.0,
    face_timeout_s=5.0,
    min_face_range_m=1.0,
    hold_s=4.0,
    cooldown_s=6.0,
    perception_resume_s=0.5,
)


def trig(bearing=0.3, rng=3.0, valid=True, t=0.0) -> Trigger:
    return Trigger(Person(0.9, bearing, rng if valid else math.nan, valid), 3, t)


def obs(t, bearing, rng=3.0, valid=True) -> Observation:
    return Observation(t, bearing, rng, valid)


def run(m, t, trigger=None, observation=None, fresh=True, speed=None):
    return m.step(t, trigger, observation, fresh, speed)


def to_facing(m, t0=0.0, **kw):
    """Engage at t0 and run until FACING starts; returns the time FACING began."""
    run(m, t0, trigger=trig(**kw))
    assert m.phase == Phase.STOPPING
    t = t0 + CFG.stop_settle_s
    run(m, t, speed=0.0)
    assert m.phase == Phase.FACING
    return t


def test_patrol_passes_the_patrol_command_through():
    m = SpiderbotMachine(CFG)
    cmd, events = run(m, 0.0)
    assert cmd.kind == CommandKind.PASS_THROUGH and events == []
    assert m.phase == Phase.PATROL


def test_trigger_stops_the_car_and_logs_the_engagement():
    m = SpiderbotMachine(CFG)
    cmd, events = run(m, 1.0, trigger=trig())
    assert cmd.kind == CommandKind.STOP
    assert [e.kind for e in events] == [EventKind.ENGAGE]
    assert m.phase == Phase.STOPPING and m.engagements == 1


def test_stopping_waits_for_the_car_to_be_slow_and_not_forever():
    m = SpiderbotMachine(CFG)
    run(m, 0.0, trigger=trig())
    run(m, 0.1, speed=0.0)  # slow, but under min_stop_s
    assert m.phase == Phase.STOPPING
    run(m, 0.5, speed=0.8)  # still moving: keep waiting
    assert m.phase == Phase.STOPPING
    run(m, 0.6, speed=0.02)  # slow enough and past min_stop_s
    assert m.phase == Phase.FACING

    m2 = SpiderbotMachine(CFG)
    run(m2, 0.0, trigger=trig())
    run(m2, 0.9, speed=None)  # no speed reading at all
    assert m2.phase == Phase.STOPPING
    run(m2, 1.0, speed=None)  # settle timeout
    assert m2.phase == Phase.FACING


def test_facing_creeps_towards_the_person_then_alerts_when_aligned():
    m = SpiderbotMachine(CFG)
    t = to_facing(m)
    cmd, events = run(m, t + 0.1, observation=obs(t + 0.1, 0.25))
    assert cmd.kind == CommandKind.FACE and cmd.bearing_rad == pytest.approx(0.25)
    assert events == []
    cmd, _ = run(m, t + 0.5, observation=obs(t + 0.5, 0.05))  # aligned, settling
    assert cmd.kind == CommandKind.FACE
    cmd, events = run(m, t + 0.9, observation=obs(t + 0.9, 0.04))
    assert cmd.kind == CommandKind.STOP
    (alert,) = events
    assert alert.kind == EventKind.ALERT and alert.data["facing_ok"] is True
    assert alert.data["final_bearing_rad"] == pytest.approx(0.04)
    assert m.phase == Phase.HOLD


def test_leaving_alignment_restarts_the_settle_timer():
    m = SpiderbotMachine(CFG)
    t = to_facing(m)
    run(m, t + 0.1, observation=obs(t + 0.1, 0.02))
    run(m, t + 0.2, observation=obs(t + 0.2, 0.3))  # drifted off
    _, events = run(m, t + 0.45, observation=obs(t + 0.45, 0.02))
    assert events == []  # only 0 s aligned since it drifted back


def test_already_facing_the_person_alerts_without_moving():
    m = SpiderbotMachine(CFG)
    run(m, 0.0, trigger=trig(bearing=0.02))
    _, events = run(m, 1.0, speed=0.0)
    (alert,) = events
    assert alert.data["facing_ok"] is True and m.phase == Phase.HOLD


def test_unknown_range_never_creeps():
    m = SpiderbotMachine(CFG)
    run(m, 0.0, trigger=trig(bearing=0.4, valid=False))
    run(m, 1.0, speed=0.0)
    assert m.phase == Phase.FACING
    cmd, events = run(m, 1.05, observation=obs(1.05, 0.4, valid=False))
    assert cmd.kind == CommandKind.STOP
    (alert,) = events
    assert alert.data["facing_ok"] is False


def test_too_close_never_creeps():
    m = SpiderbotMachine(CFG)
    t = to_facing(m, bearing=0.4, rng=0.6)
    _, events = run(m, t + 0.05, observation=obs(t + 0.05, 0.4, rng=0.6))
    assert events[0].kind == EventKind.ALERT and events[0].data["facing_ok"] is False


def test_losing_the_person_ends_facing_with_facing_not_ok():
    m = SpiderbotMachine(CFG)
    t = to_facing(m)
    run(m, t + 0.1, observation=obs(t + 0.1, 0.3))
    cmd, events = run(m, t + 0.7, observation=obs(t + 0.1, 0.3))  # frame too old: hold still
    assert cmd.kind == CommandKind.STOP and events == []
    _, events = run(m, t + 1.2, observation=obs(t + 0.1, 0.3))
    assert events[0].kind == EventKind.ALERT and events[0].data["facing_ok"] is False


def test_facing_times_out():
    m = SpiderbotMachine(CFG)
    t = to_facing(m)
    for i in range(1, 60):
        now = t + i * 0.1
        _, events = run(m, now, observation=obs(now, 0.4))  # never converges
        if events:
            break
    assert events and events[0].data["facing_ok"] is False
    assert now - t == pytest.approx(5.1, abs=0.15)


def test_face_disabled_skips_the_manoeuvre():
    m = SpiderbotMachine(MachineConfig(face_enabled=False, stop_settle_s=1.0, hold_s=2.0))
    run(m, 0.0, trigger=trig(bearing=0.5))
    _, events = run(m, 1.0, speed=0.0)
    assert events[0].kind == EventKind.ALERT and events[0].data["facing_ok"] is False


def test_hold_then_resume_then_cooldown_ignores_new_triggers():
    m = SpiderbotMachine(CFG)
    run(m, 0.0, trigger=trig(bearing=0.0))
    _, events = run(m, 1.0, speed=0.0)  # alert at t=1
    assert events[0].kind == EventKind.ALERT
    cmd, events = run(m, 4.9)
    assert cmd.kind == CommandKind.STOP and events == []
    cmd, events = run(m, 5.0)  # hold_s = 4
    assert cmd.kind == CommandKind.PASS_THROUGH
    assert [e.kind for e in events] == [EventKind.RESUME]
    cmd, events = run(m, 7.0, trigger=trig())  # within cooldown (until 11)
    assert cmd.kind == CommandKind.PASS_THROUGH and events == []
    assert m.phase == Phase.PATROL and m.engagements == 1
    cmd, events = run(m, 11.0, trigger=trig())
    assert cmd.kind == CommandKind.STOP and events[0].kind == EventKind.ENGAGE
    assert m.engagements == 2


def test_triggers_are_ignored_while_engaged():
    m = SpiderbotMachine(CFG)
    run(m, 0.0, trigger=trig())
    _, events = run(m, 0.1, trigger=trig())
    assert events == [] and m.engagements == 1


def test_blind_patrol_stops_and_resumes_after_perception_is_back():
    m = SpiderbotMachine(CFG)
    cmd, events = run(m, 0.0, fresh=False)
    assert cmd.kind == CommandKind.STOP and events[0].kind == EventKind.PERCEPTION_LOST
    cmd, events = run(m, 1.0, fresh=True)  # back, starting the resume clock
    assert cmd.kind == CommandKind.STOP and m.phase == Phase.NO_PERCEPTION
    run(m, 1.2, fresh=False)  # flickers away: clock restarts
    run(m, 1.3, fresh=True)
    cmd, events = run(m, 1.6, fresh=True)
    assert m.phase == Phase.NO_PERCEPTION
    cmd, events = run(m, 1.9, fresh=True)
    assert m.phase == Phase.PATROL and events[0].kind == EventKind.PERCEPTION_RESTORED
    cmd, _ = run(m, 2.0, fresh=True)
    assert cmd.kind == CommandKind.PASS_THROUGH


def test_require_perception_false_patrols_blind():
    m = SpiderbotMachine(MachineConfig(require_perception=False))
    cmd, events = run(m, 0.0, fresh=False)
    assert cmd.kind == CommandKind.PASS_THROUGH and events == []


@pytest.mark.parametrize(
    "field",
    ["stop_settle_s", "face_tolerance_rad", "hold_s", "min_face_range_m", "face_timeout_s"],
)
@pytest.mark.parametrize("bad", [0.0, -1.0, math.nan, math.inf])
def test_config_rejects_bad_positive_values(field, bad):
    with pytest.raises(ValueError, match=field):
        MachineConfig(**{field: bad})


@pytest.mark.parametrize("field", ["min_stop_s", "cooldown_s", "stop_speed_eps_mps"])
def test_config_rejects_negative_values(field):
    with pytest.raises(ValueError, match=field):
        MachineConfig(**{field: -0.1})
    MachineConfig(**{field: 0.0})
