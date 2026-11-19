"""L1: the lap source rule reads the camera's own freshness (outside review of brief 51c, findings 1
and 2). A status that says `running` and similarity 1.0 is not enough: the counter republishes it on
a timer when no image arrives. Ready needs a fresh camera, wheel odometry, and a frame sequence
that advances between statuses; a camera that goes stale during a run makes the source not alive.
A status without these keys (an older counter, a bag, the lap source tests of brief 51) is read as
before: nothing here changes a verdict for it."""

from __future__ import annotations

import math

import pytest
from external_support import LAP_M, EngineRig, Rig
from racer_laps.external_gate import parse_status_camera

CAM = {
    "camera_fresh": True,
    "frame_seq": 10,
    "frame_age_s": 0.05,
    "odom_fresh": True,
    "reset_id": 1,
    "frames_since_reset": 5,
}


def cam(**changes):
    out = dict(CAM)
    out.update(changes)
    return out


def standing_rig() -> Rig:
    return Rig(start=False, status_period_s=None, status_at_start=False)


def status(rig: Rig, t: float, camera: dict | None, state: str = "running") -> None:
    rig.gate.on_source_status(t, state, 0, 1.0, camera)


# -- parse ---------------------------------------------------------------------------------------


def test_parse_status_camera_reads_the_keys_it_knows_with_their_types() -> None:
    body = (
        '{"state":"running","laps":0,"similarity":1.0,"camera":"fresh","camera_fresh":true,'
        '"frame_seq":7,"frame_age_s":0.1,"odom_fresh":false,"reset_id":2,"frames_since_reset":3}'
    )
    out = parse_status_camera(body)
    assert out == {
        "camera_fresh": True,
        "frame_seq": 7,
        "frame_age_s": 0.1,
        "odom_fresh": False,
        "reset_id": 2,
        "frames_since_reset": 3,
    }


def test_parse_status_camera_ignores_wrong_types_and_missing_keys() -> None:
    assert parse_status_camera('{"state":"running"}') == {}
    assert parse_status_camera("not json") is None
    assert parse_status_camera("[1]") is None
    out = parse_status_camera(
        '{"camera_fresh":1,"frame_seq":"7","frame_age_s":NaN,"odom_fresh":null,"reset_id":2.5}'
    )
    assert out == {}  # an int is not a bool, a string not a count, NaN not an age, 2.5 not an id
    assert parse_status_camera('{"frame_seq":true}') == {}  # a bool is not a count


# -- ready ---------------------------------------------------------------------------------------


def test_without_camera_keys_nothing_changes() -> None:
    rig = standing_rig()
    status(rig, 0.0, None)
    assert rig.gate.source_ready_now
    status(rig, 1.0, {})  # an older counter: no keys at all
    assert rig.gate.source_ready_now


def test_the_first_status_with_a_sequence_has_nothing_to_compare_and_is_not_ready() -> None:
    rig = standing_rig()
    status(rig, 0.0, cam(frame_seq=10))
    assert not rig.gate.source_ready_now
    assert "no camera frame progress" in rig.gate.source_not_ready_reason
    status(rig, 1.0, cam(frame_seq=25))
    assert rig.gate.source_ready_now


@pytest.mark.parametrize(
    ("second", "ready", "why"),
    [
        (11, True, ""),
        (10, False, "not advancing"),  # the same count: images stopped
        (9, False, "not advancing"),  # the counter restarted
    ],
)
def test_ready_needs_the_frame_sequence_to_advance(second: int, ready: bool, why: str) -> None:
    rig = standing_rig()
    status(rig, 0.0, cam(frame_seq=10))
    status(rig, 1.0, cam(frame_seq=second))
    assert rig.gate.source_ready_now is ready
    assert why in rig.gate.source_not_ready_reason


def test_a_stale_camera_is_not_ready_whatever_else_the_status_says() -> None:
    rig = standing_rig()
    status(rig, 0.0, cam(frame_seq=1))
    status(rig, 1.0, cam(frame_seq=2, camera_fresh=False, frame_age_s=4.2))
    assert not rig.gate.source_ready_now
    assert "stale" in rig.gate.source_not_ready_reason and "4.2" in rig.gate.source_not_ready_reason


def test_stale_wheel_odometry_at_the_counter_is_not_ready() -> None:
    rig = standing_rig()
    status(rig, 0.0, cam(frame_seq=1))
    status(rig, 1.0, cam(frame_seq=2, odom_fresh=False))
    assert not rig.gate.source_ready_now
    assert "odometry" in rig.gate.source_not_ready_reason


def test_one_frame_then_silence_never_becomes_ready() -> None:
    """The review's case: one frame was captured, images stopped, the process stays up and its status
    timer keeps saying running with similarity 1.0."""
    rig = standing_rig()
    for k in range(10):
        status(rig, float(k), cam(frame_seq=1, camera_fresh=k < 1))
        assert not rig.gate.source_ready_now, k


def test_the_ready_decision_at_the_start_carries_the_camera_check() -> None:
    rig = standing_rig()
    status(rig, 0.0, cam(frame_seq=1))
    status(rig, 1.0, cam(frame_seq=1))  # not advancing
    rig.drive_until(1.2)
    assert rig.gate.set_start(rig.t)
    assert rig.gate.source_ready is False
    assert "not advancing" in rig.gate.source_not_ready_reason


# -- during a run -----------------------------------------------------------------------------------


def test_a_camera_that_goes_stale_during_a_run_makes_the_source_not_alive() -> None:
    rig = Rig(speed=3.0, status_period_s=None, status_at_start=False, start=False)
    status(rig, 0.0, cam(frame_seq=1))
    status(rig, 0.5, cam(frame_seq=9))
    assert rig.gate.set_start(0.5) and rig.gate.source_ready
    rig.drive_until(2.0)
    assert rig.gate.source_alive
    status(rig, 2.0, cam(frame_seq=9, camera_fresh=False, frame_age_s=1.6))
    assert rig.gate.source_alive is False  # the status arrived, the camera did not
    assert rig.gate.station_uncertain is True
    c = rig.event(0.5 + LAP_M / 3.0, 1)  # the lap that closes now, 0.4 s after the last status
    assert c is not None and c.missed is True
    assert "alive" in rig.gate.doubtful[-1]["reasons"]


def test_a_stale_camera_does_not_touch_a_run_whose_status_has_no_camera_keys() -> None:
    rig = Rig()
    c = rig.event(LAP_M, 1)
    assert c is not None and c.missed is False and rig.gate.source_alive


def test_the_status_carries_the_camera_keys_and_none_when_the_counter_gave_none() -> None:
    rig = standing_rig()
    s = rig.gate.status()
    for key in (
        "source_camera_fresh",
        "source_frame_seq",
        "source_reset_id",
        "source_odom_fresh",
        "source_frame_age_s",
    ):
        assert s[key] is None, key
    status(rig, 0.0, cam(frame_seq=3, reset_id=4))
    s = rig.gate.status()
    assert s["source_camera_fresh"] is True and s["source_frame_seq"] == 3
    assert s["source_reset_id"] == 4 and s["source_odom_fresh"] is True
    assert s["source_frame_age_s"] == pytest.approx(0.05)


def test_the_engine_hands_the_camera_keys_to_the_gate() -> None:
    rig = EngineRig(status_period_s=None, ready=False)
    rig.mem.on_lap_source_status(1.0, "running", 0, 1.0, cam(frame_seq=1))
    rig.mem.on_lap_source_status(2.0, "running", 0, 1.0, cam(frame_seq=1))
    assert rig.mem.status()["source_ready_now"] is False
    assert math.isfinite(float(rig.mem.status()["source_status_age_s"]))  # type: ignore[arg-type]
