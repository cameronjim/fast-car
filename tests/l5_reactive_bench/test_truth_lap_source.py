"""L1: the truth oracle of battery B9 (brief 51), `truth_lap_source.TruthLapSource`.

Crossing interpolation on a scripted circle and stadium in both directions, no crossing before
`min_lap_length_m`, the start at the first ground-truth sample at or after the armed stamp, and the
fault plan of the yaml: it publishes exactly what the yaml says, numbered consecutively."""

from __future__ import annotations

import math
from pathlib import Path

import pytest
from truth_lap_source import FaultPlan, OracleEvent, TruthLapSource

REPO_ROOT = Path(__file__).resolve().parents[2]
YAML = REPO_ROOT / "evaluation" / "prereg" / "pnp-v0.5-laps-camera.yaml"
HZ = 50.0
SPEED = 1.0


def circle(radius: float, ccw: bool = True):
    """Poses of a car on a circle that starts at (radius, 0) heading along +y (ccw) or -y (cw)."""
    length = 2.0 * math.pi * radius

    def pose(t: float) -> tuple[float, float, float]:
        a = (SPEED * t / radius) * (1.0 if ccw else -1.0)
        x, y = radius * math.cos(a), radius * math.sin(a)
        yaw = a + (math.pi / 2.0 if ccw else -math.pi / 2.0)
        return x, y, yaw

    return pose, length


def stadium(straight: float, radius: float, ccw: bool = True):
    """A stadium centreline: the car starts at the origin heading +x on the bottom straight."""
    arc = math.pi * radius
    length = 2.0 * straight + 2.0 * arc

    def pose(t: float) -> tuple[float, float, float]:
        s = (SPEED * t) % length
        if s < straight:
            x, y, yaw = s, 0.0, 0.0
        elif s < straight + arc:
            a = (s - straight) / radius
            x, y, yaw = straight + radius * math.sin(a), radius - radius * math.cos(a), a
        elif s < 2.0 * straight + arc:
            x, y, yaw = straight - (s - straight - arc), 2.0 * radius, math.pi
        else:
            a = (s - 2.0 * straight - arc) / radius
            x, y, yaw = -radius * math.sin(a), radius + radius * math.cos(a), math.pi + a
        return x, y, yaw

    def pose_dir(t: float) -> tuple[float, float, float]:
        x, y, yaw = pose(t)
        if ccw:
            return x, y, yaw
        # mirror: clockwise, same start pose and line
        return x, -y, -yaw

    return pose_dir, length


def run(source: TruthLapSource, pose, seconds: float, arm: float = 0.0, t0: float = 0.0):
    """Feed poses at 50 Hz. Returns every OracleEvent in the order published."""
    out: list[OracleEvent] = []
    source.arm(arm)
    k = 0
    while k / HZ <= seconds:
        t = t0 + k / HZ
        x, y, yaw = pose(k / HZ)
        out.extend(source.on_truth_odom(t, x, y, yaw))
        k += 1
    return out


@pytest.mark.parametrize("ccw", [True, False])
def test_crossings_on_a_circle_are_interpolated_to_a_few_ms(ccw: bool) -> None:
    pose, length = circle(1.5, ccw)
    events = run(TruthLapSource(), pose, 3.4 * length / SPEED)
    assert [e.payload["lap"] for e in events] == [1, 2, 3]
    for k, e in enumerate(events, start=1):
        assert e.payload["crossing_time_s"] == pytest.approx(k * length / SPEED, abs=0.005)
        assert e.payload["distance_m"] == pytest.approx(length, abs=0.01)
        assert e.payload["lap_time_s"] == pytest.approx(length / SPEED, abs=0.01)
        assert e.payload["peak_similarity"] == 1.0 and e.payload["source"] == "truth"
        assert e.kind == "true" and e.truth_crossing == k


@pytest.mark.parametrize("ccw", [True, False])
def test_crossings_on_a_stadium_both_directions(ccw: bool) -> None:
    pose, length = stadium(2.0, 0.9, ccw)
    events = run(TruthLapSource(), pose, 4.2 * length / SPEED)
    assert [e.payload["lap"] for e in events] == [1, 2, 3, 4]
    for k, e in enumerate(events, start=1):
        assert e.payload["crossing_time_s"] == pytest.approx(k * length / SPEED, abs=0.01)
        assert e.payload["distance_m"] == pytest.approx(length, abs=0.02)


def test_no_crossing_before_the_minimum_lap_length() -> None:
    pose, length = circle(0.3)  # a loop of 1.9 m: shorter than min_lap_length_m
    assert length < 3.0
    assert run(TruthLapSource(), pose, 5.0 * length / SPEED) == []
    # with a shorter minimum the same loop counts
    pose, length = circle(0.3)
    got = run(TruthLapSource(min_lap_length_m=1.0), pose, 3.4 * length / SPEED)
    assert len(got) == 3


def test_nothing_happens_until_armed_and_the_start_is_the_first_sample_at_or_after_the_stamp() -> (
    None
):
    pose, length = circle(1.5)
    s = TruthLapSource()
    assert s.on_truth_odom(0.0, *pose(0.0)) == []  # not armed: nothing, not started
    assert not s.started and s.truth_start_t is None
    s.arm(0.55)
    for k in range(40):
        s.on_truth_odom(k / HZ, *pose(k / HZ))
    assert s.started
    assert s.truth_start_t == pytest.approx(0.56, abs=1e-9)  # the first sample at or after 0.55
    # crossing 1 comes one lap after the START, not after t = 0
    events = []
    k = 40
    while k / HZ < 2.2 * length:
        events.extend(s.on_truth_odom(k / HZ, *pose(k / HZ)))
        k += 1
    assert events[0].payload["crossing_time_s"] == pytest.approx(0.56 + length / SPEED, abs=0.01)


def test_the_status_is_the_cameras_json_shape() -> None:
    pose, length = circle(1.5)
    s = TruthLapSource()
    st = s.status()
    assert st["state"] == "running" and st["laps"] == 0 and st["similarity"] == 1.0
    run(s, pose, 2.2 * length / SPEED)
    st = s.status()
    assert st["laps"] == 2 and set(st) >= {"state", "similarity", "laps"}


# -- the fault plan ----------------------------------------------------------------------------------


def yaml_fault_plan() -> FaultPlan:
    return FaultPlan.from_prereg(YAML)


def test_the_fault_plan_is_read_from_the_registered_yaml() -> None:
    plan = yaml_fault_plan()
    assert plan.drop_crossings == (4, 7)
    assert plan.false_lap == 9 and plan.false_share == 0.6
    assert plan.duplicate_crossing == 10 and plan.duplicate_delay_s == 0.2
    assert plan.consecutive is True
    with pytest.raises(ValueError):
        FaultPlan.from_dict({"numbering": "random"})


def test_the_fault_plan_publishes_exactly_what_the_yaml_says() -> None:
    pose, length = circle(1.5)
    plan = yaml_fault_plan()
    clean = run(TruthLapSource(), pose, 13.4 * length / SPEED)
    assert len(clean) == 13
    events = run(TruthLapSource(plan), pose, 13.4 * length / SPEED)
    true = [e for e in events if e.kind == "true"]
    false = [e for e in events if e.kind == "false"]
    dup = [e for e in events if e.kind == "duplicate"]
    # crossings 4 and 7 are not published
    assert [e.truth_crossing for e in true] == [1, 2, 3, 5, 6, 8, 9, 10, 11, 12, 13]
    # one false event, inside lap 9 at 0.6 of the median ground-truth lap path
    assert len(false) == 1
    f = false[0].payload
    assert f["crossing_time_s"] == pytest.approx((8 + 0.6) * length / SPEED, abs=0.02)
    # crossing 10 published twice, 0.2 s apart, with the same content
    assert len(dup) == 1
    d10 = next(e for e in true if e.truth_crossing == 10)
    assert dup[0].payload == d10.payload
    assert dup[0].publish_t - d10.publish_t == pytest.approx(0.2, abs=1.0 / HZ + 1e-9)
    # published events numbered consecutively over what was published; the duplicate repeats one
    numbers = [e.payload["lap"] for e in events if e.kind != "duplicate"]
    assert numbers == list(range(1, len(numbers) + 1))
    assert len(numbers) == 13 - 2 + 1  # 13 crossings, two dropped, one false
    assert dup[0].payload["lap"] == d10.payload["lap"]
    # the lap a camera reports after a missed crossing is the merged one
    merged = next(e for e in true if e.truth_crossing == 5).payload
    assert merged["distance_m"] == pytest.approx(2.0 * length, abs=0.03)
    assert merged["lap_time_s"] == pytest.approx(2.0 * length / SPEED, abs=0.03)
    # the false event and the lap after it split lap 9
    after_false = next(e for e in true if e.truth_crossing == 9).payload
    assert f["distance_m"] == pytest.approx(0.6 * length, abs=0.05)
    assert after_false["distance_m"] == pytest.approx(0.4 * length, abs=0.05)


def test_the_published_events_are_in_time_order() -> None:
    pose, length = circle(1.5)
    events = run(TruthLapSource(yaml_fault_plan()), pose, 13.4 * length / SPEED)
    times = [e.publish_t for e in events]
    assert times == sorted(times)
    cross = [e.payload["crossing_time_s"] for e in events if e.kind != "duplicate"]
    assert cross == sorted(cross)


def test_the_median_is_over_the_truth_laps_so_far() -> None:
    """A lap that is not the same length every time: 0.6 of the median of the laps so far."""
    pose, length = stadium(2.0, 0.9)
    plan = FaultPlan(false_lap=3, false_share=0.5)
    events = run(TruthLapSource(plan), pose, 4.2 * length / SPEED)
    false = [e for e in events if e.kind == "false"]
    assert len(false) == 1
    assert false[0].payload["crossing_time_s"] == pytest.approx(
        (2 + 0.5) * length / SPEED, abs=0.03
    )
    # a plan with no faults is the clean oracle
    assert [e.kind for e in run(TruthLapSource(FaultPlan()), pose, 2.2 * length / SPEED)] == [
        "true",
        "true",
    ]
