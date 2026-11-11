"""L1: the spiderbot pose rules on synthetic and on real (recorded) keypoints.

The synthetic cases are exact (a wrist this far above a shoulder, a joint not seen). The real
ones are yolov8n-pose output on three public domain photographs, committed as JSON
(racer_perception/test/fixtures/keypoints), so the rules are checked against what the detector
actually produces: noisy joints, hands that are not where a body model puts them.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest
from racer_apps.spiderbot.pose_rules import (
    ArmSwingRule,
    HeldForFrames,
    Person,
    PredicateRule,
    RuleSettings,
    arms_raised,
    body_scale_px,
    build_rule,
    choose_person,
    load_rule_factory,
    one_arm_raised,
    person_present,
    register_rule,
    rule_names,
)
from racer_apps.spiderbot.synthetic import POSES, synthetic_keypoints, with_confidence
from racer_perception.keypoints import Keypoint

FIXTURE = (
    Path(__file__).resolve().parents[2]
    / "racer_perception"
    / "test"
    / "fixtures"
    / "keypoints"
    / "yolov8n_pose_images.json"
)
S = RuleSettings()


def person(pose="standing", score=0.9, rng=3.0, valid=True, **kw) -> Person:
    height = 300.0
    return Person(
        score=score,
        bearing_rad=0.0,
        range_m=rng if valid else math.nan,
        range_valid=valid,
        box_height_px=height,
        keypoints=synthetic_keypoints(pose, 640.0, 100.0, height, **kw),
    )


# -- stateless geometry ---------------------------------------------------------------------


@pytest.mark.parametrize(
    ("pose", "both", "either"),
    [
        ("standing", False, False),
        ("arms_raised", True, True),
        ("one_arm_raised", False, True),
        ("arms_wide", False, False),
        ("guard", False, False),
    ],
)
def test_synthetic_poses(pose, both, either):
    assert arms_raised(person(pose), S) is both
    assert one_arm_raised(person(pose), S) is either


def test_all_named_poses_are_tested():
    assert set(POSES) == {"standing", "arms_raised", "one_arm_raised", "arms_wide", "guard"}


def test_wrist_barely_above_shoulder_is_not_raised_but_well_above_is():
    base = person("arms_raised")
    kps = list(base.keypoints)
    shoulder_y = kps[5].y_px
    scale = body_scale_px(base, 0.5)
    just_below_margin = shoulder_y - 0.45 * scale
    just_above_margin = shoulder_y - 0.55 * scale
    for y, expected in ((just_below_margin, False), (just_above_margin, True)):
        kps[9] = Keypoint(kps[9].x_px, y, 0.9)
        kps[10] = Keypoint(kps[10].x_px, y, 0.9)
        assert arms_raised(Person(0.9, 0, 3, True, 300.0, tuple(kps)), S) is expected


def test_unseen_wrist_never_counts_as_raised():
    p = person("arms_raised")
    hidden = Person(
        p.score,
        p.bearing_rad,
        p.range_m,
        p.range_valid,
        p.box_height_px,
        with_confidence(p.keypoints, ("left_wrist",), 0.0),
    )
    assert arms_raised(hidden, S) is False
    assert one_arm_raised(hidden, S) is True  # the right arm is still up


def test_missing_shoulders_or_no_keypoints_fail_closed():
    p = person("arms_raised")
    no_shoulder = Person(
        0.9, 0, 3, True, 300.0, with_confidence(p.keypoints, ("left_shoulder",), 0.1)
    )
    assert arms_raised(no_shoulder, S) is False
    assert one_arm_raised(Person(0.9, 0, 3, True), S) is False
    assert body_scale_px(Person(0.9, 0, 3, True), 0.5) is None


def test_nan_keypoint_is_ignored():
    p = person("arms_raised")
    kps = list(p.keypoints)
    kps[9] = Keypoint(math.nan, math.nan, 0.9)
    assert arms_raised(Person(0.9, 0, 3, True, 300.0, tuple(kps)), S) is False


def test_side_on_person_scale_is_floored_by_box_height():
    p = person("standing")
    kps = list(p.keypoints)
    kps[6] = Keypoint(kps[5].x_px + 1.0, kps[5].y_px, 0.9)  # shoulders overlap
    squeezed = Person(0.9, 0, 3, True, 300.0, tuple(kps))
    assert body_scale_px(squeezed, 0.5) == pytest.approx(0.2 * 300.0)


def test_person_present_score_and_range():
    assert person_present(person(score=0.9), S)
    assert not person_present(person(score=0.3), S)
    far = RuleSettings(max_range_m=4.0)
    assert person_present(person(rng=3.0), far)
    assert not person_present(person(rng=5.0), far)
    assert not person_present(person(valid=False), far)  # no range, cannot prove it is near


def test_choose_person_prefers_nearest_valid_range_then_score():
    near, far = person(rng=2.0, score=0.6), person(rng=5.0, score=0.95)
    assert choose_person([far, near]) is near
    unranged = [person(valid=False, score=0.6), person(valid=False, score=0.8)]
    assert choose_person(unranged).score == 0.8
    assert choose_person([]) is None


# -- rule objects ---------------------------------------------------------------------------


def test_predicate_rule_applies_min_score_and_returns_the_matching_person():
    rule = build_rule("arms_raised", RuleSettings(min_score=0.6))
    low = person("arms_raised", score=0.5)
    ok = person("arms_raised", score=0.8)
    assert rule.update(0.0, [low]) is None
    assert rule.update(0.1, [person("standing"), ok]) is ok


def test_unknown_rule_is_refused_with_the_choices():
    with pytest.raises(ValueError, match="arms_raised"):
        build_rule("moonwalk")


def test_register_and_load_pluggable_rules():
    class Always:
        name = "always"

        def update(self, t_s, people):
            return people[0] if people else None

        def reset(self):
            pass

    register_rule("always_test_rule", lambda settings: Always())
    assert "always_test_rule" in rule_names()
    assert build_rule("always_test_rule").update(0.0, [person()]) is not None
    with pytest.raises(ValueError, match="already registered"):
        register_rule("always_test_rule", lambda s: Always())
    with pytest.raises(ValueError, match="rule name"):
        register_rule("bad name!", lambda s: Always())


def test_load_rule_factory_from_module_path_and_its_refusals():
    factory = load_rule_factory("racer_apps.spiderbot.pose_rules:build_rule")
    assert callable(factory)
    with pytest.raises(ValueError, match="package.module:function"):
        load_rule_factory("nocolon")
    with pytest.raises(TypeError, match="not a callable"):
        load_rule_factory("racer_apps.spiderbot.pose_rules:__doc__")
    assert build_rule("test_pose_rules:_predicate_demo", S).name == "demo"


def _predicate_demo(settings: RuleSettings):
    return PredicateRule("demo", lambda p, s: True, settings)


# -- temporal rule --------------------------------------------------------------------------


def test_held_for_frames_needs_n_consecutive_matches():
    rule = HeldForFrames(build_rule("arms_raised"), frames=3)
    up = [person("arms_raised")]
    assert rule.update(0.0, up) is None
    assert rule.update(0.1, up) is None
    trig = rule.update(0.2, up)
    assert trig is not None and trig.frames == 3
    assert rule.update(0.3, up).frames == 4  # keeps reporting while it holds


def test_one_bad_frame_resets_the_count():
    rule = HeldForFrames(build_rule("arms_raised"), frames=3)
    up, down = [person("arms_raised")], [person("standing")]
    rule.update(0.0, up)
    rule.update(0.1, up)
    assert rule.update(0.2, down) is None
    assert rule.count == 0
    assert rule.update(0.3, up) is None and rule.update(0.4, up) is None
    assert rule.update(0.5, up) is not None


def test_empty_frame_counts_as_a_miss():
    rule = HeldForFrames(build_rule("person_present"), frames=2)
    assert rule.update(0.0, [person()]) is None
    assert rule.update(0.1, []) is None
    assert rule.update(0.2, [person()]) is None
    assert rule.update(0.3, [person()]) is not None


def test_a_gap_between_frames_resets_even_if_every_frame_matched():
    rule = HeldForFrames(build_rule("person_present"), frames=2, max_gap_s=0.5)
    assert rule.update(0.0, [person()]) is None
    assert rule.update(5.0, [person()]) is None  # stalled detector, not consecutive
    assert rule.update(5.1, [person()]) is not None


def test_time_going_backwards_resets():
    rule = HeldForFrames(build_rule("person_present"), frames=2)
    rule.update(10.0, [person()])
    assert rule.update(9.0, [person()]) is None


def test_held_for_frames_validates_and_resets():
    with pytest.raises(ValueError):
        HeldForFrames(build_rule("person_present"), frames=0)
    with pytest.raises(ValueError):
        HeldForFrames(build_rule("person_present"), max_gap_s=math.nan)
    rule = HeldForFrames(build_rule("person_present"), frames=1)
    assert rule.name == "person_present"
    assert rule.update(0.0, [person()]) is not None
    rule.reset()
    assert rule.count == 0


# -- arm swing ------------------------------------------------------------------------------


def _swing_frames(reaches, dt):
    return [(i * dt, [person("guard", reach=r, punching_side=-1)]) for i, r in enumerate(reaches)]


def test_fast_fist_fires_and_slow_reach_does_not():
    fast = ArmSwingRule(RuleSettings(swing_speed_scale_per_s=4.0))
    hits = [fast.update(t, ps) for t, ps in _swing_frames([0.0, 0.0, 1.0, 1.0], 0.1)]
    assert any(h is not None for h in hits)
    slow = ArmSwingRule(RuleSettings(swing_speed_scale_per_s=4.0))
    reaches = [i / 40.0 for i in range(41)]  # one second from guard to full reach
    assert all(slow.update(t, ps) is None for t, ps in _swing_frames(reaches, 0.1 / 4))


def test_keypoint_jitter_does_not_fire_arm_swing():
    rule = ArmSwingRule()
    base = person("standing")
    for i in range(30):
        wobble = 0.5 * (-1) ** i
        kps = tuple(
            Keypoint(k.x_px + wobble, k.y_px - wobble, k.confidence) for k in base.keypoints
        )
        jittery = Person(0.9, 0, 3, True, 300.0, kps)
        assert rule.update(i * 0.1, [jittery]) is None


def test_arm_swing_history_clears_without_a_person_and_resets():
    rule = ArmSwingRule()
    rule.update(0.0, [person("guard", reach=0.0, punching_side=-1)])
    rule.update(0.1, [])
    assert rule.update(0.2, [person("guard", reach=1.0, punching_side=-1)]) is None  # no history
    rule.reset()
    assert rule.update(0.3, [person("standing", score=0.1)]) is None


# -- real detector output -------------------------------------------------------------------


@pytest.fixture(scope="module")
def real():
    data = json.loads(FIXTURE.read_text(encoding="utf-8"))

    def people(name):
        out = []
        for d in data[name]:
            kps = tuple(Keypoint(*k) for k in d["keypoints"])
            out.append(Person(d["score"], 0.0, math.nan, False, d["box_cxcywh_px"][3], kps))
        return out

    return people


def test_real_hiker_with_both_arms_up_is_arms_raised(real):
    (hiker,) = real("arms_raised_hiker.jpg")
    assert arms_raised(hiker, S) and one_arm_raised(hiker, S)


def test_real_airman_with_arms_out_wide_is_not_arms_raised(real):
    (airman,) = real("arms_wide_airman.jpg")
    assert not arms_raised(airman, S)


def test_real_group_with_arms_down_triggers_nothing(real):
    group = real("four_people_standing.jpg")
    assert len(group) == 4
    assert not any(arms_raised(p, S) or one_arm_raised(p, S) for p in group)
    rule = build_rule("arms_raised")
    assert rule.update(0.0, group) is None


def test_real_hiker_held_over_three_frames_triggers(real):
    rule = HeldForFrames(build_rule("arms_raised"), frames=3)
    frames = [real("arms_raised_hiker.jpg")] * 3
    trig = None
    for i, people in enumerate(frames):
        trig = rule.update(i * 0.1, people)
    assert trig is not None and trig.frames == 3
