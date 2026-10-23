"""L1 tests for racer_perception.person_tracker: nearest person, hold-over, edge cases."""

from __future__ import annotations

import math

import pytest
from racer_perception.fusion import STATUS_NO_RETURNS, STATUS_OK, FusedObject
from racer_perception.person_tracker import PersonTracker


def obj(name="person", range_m=2.0, score=0.9, stamp=0.0, bearing=0.0):
    valid = math.isfinite(range_m)
    return FusedObject(
        stamp_s=stamp,
        class_name=name,
        score=score,
        bearing_rad=bearing,
        bearing_half_width_rad=0.05,
        range_m=range_m,
        range_valid=valid,
        range_status=STATUS_OK if valid else STATUS_NO_RETURNS,
        range_samples=5 if valid else 0,
        cx_px=640.0,
        cy_px=360.0,
        width_px=100.0,
        height_px=300.0,
    )


def test_nearest_person_wins():
    t = PersonTracker()
    far, near = obj(range_m=4.0, bearing=0.2), obj(range_m=1.5, bearing=-0.3)
    target, held = t.update([far, near], now_s=0.0)
    assert target is near and not held


def test_other_classes_are_ignored():
    t = PersonTracker()
    target, _ = t.update([obj("chair", 0.5), obj("person", 3.0)], now_s=0.0)
    assert target is not None and target.class_name == "person" and target.range_m == 3.0


def test_no_person_no_target():
    t = PersonTracker()
    assert t.update([obj("chair", 1.0)], 0.0) == (None, False)
    assert t.update([], 0.1) == (None, False)


def test_ranged_person_beats_a_closer_looking_unranged_one():
    t = PersonTracker()
    ranged = obj(range_m=5.0, score=0.5)
    unranged = obj(range_m=math.nan, score=0.99)
    target, _ = t.update([unranged, ranged], 0.0)
    assert target is ranged


def test_without_any_range_the_highest_score_wins_and_stays_unranged():
    t = PersonTracker()
    low, high = obj(range_m=math.nan, score=0.4), obj(range_m=math.nan, score=0.8)
    target, held = t.update([low, high], 0.0)
    assert target is high and not target.range_valid and not held


def test_min_score_filters_candidates():
    t = PersonTracker(min_score=0.5)
    assert t.update([obj(range_m=1.0, score=0.49)], 0.0) == (None, False)
    target, _ = t.update([obj(range_m=1.0, score=0.5)], 0.1)
    assert target is not None


def test_hold_over_within_a_second_returns_the_last_target_unchanged():
    t = PersonTracker(hold_s=1.0)
    seen = obj(range_m=2.0, stamp=10.0)
    t.update([seen], now_s=10.0)
    target, held = t.update([], now_s=10.9)
    assert target is seen and held
    assert target.stamp_s == 10.0  # history is not rewritten


def test_hold_over_expires_after_hold_s():
    t = PersonTracker(hold_s=1.0)
    t.update([obj()], now_s=10.0)
    assert t.update([], now_s=11.0)[1] is True  # exactly at the limit still holds
    assert t.update([], now_s=11.01) == (None, False)
    assert t.target is None
    assert t.update([], now_s=11.02) == (None, False)  # and stays gone


def test_hold_over_is_measured_from_the_last_sighting_not_the_last_call():
    t = PersonTracker(hold_s=1.0)
    t.update([obj()], now_s=0.0)
    t.update([], now_s=0.6)
    assert t.update([], now_s=1.2) == (None, False)


def test_a_new_sighting_replaces_a_held_target_and_restarts_the_clock():
    t = PersonTracker(hold_s=1.0)
    t.update([obj(range_m=2.0)], now_s=0.0)
    t.update([], now_s=0.5)
    newer = obj(range_m=3.0, stamp=0.9)
    assert t.update([newer], now_s=0.9) == (newer, False)
    assert t.update([], now_s=1.8)[1] is True


def test_clock_going_backwards_drops_the_hold_over():
    t = PersonTracker()
    t.update([obj()], now_s=100.0)
    assert t.update([], now_s=50.0) == (None, False)


def test_zero_hold_never_holds_over():
    t = PersonTracker(hold_s=0.0)
    t.update([obj()], now_s=5.0)
    assert t.update([], now_s=5.0)[1] is True  # same instant only
    t.update([obj()], now_s=6.0)
    assert t.update([], now_s=6.001) == (None, False)


def test_custom_class():
    t = PersonTracker(class_name="dog")
    assert t.update([obj("person", 1.0)], 0.0) == (None, False)
    assert t.update([obj("dog", 1.0)], 0.1)[0] is not None


def test_reset_clears_the_target():
    t = PersonTracker()
    t.update([obj()], 0.0)
    t.reset()
    assert t.target is None and t.update([], 0.1) == (None, False)


@pytest.mark.parametrize("hold", [-1.0, math.nan, math.inf])
def test_bad_hold_is_rejected(hold):
    with pytest.raises(ValueError):
        PersonTracker(hold_s=hold)
