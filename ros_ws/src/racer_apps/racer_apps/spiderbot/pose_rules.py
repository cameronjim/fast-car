"""Pose rules for the spiderbot: pluggable, no ROS, no numpy (roadmap 5.3).

A RULE looks at the people the camera sees in ONE frame (and, if it keeps state, the frames
before it) and says whether the pattern it watches for is happening. The spiderbot acts on a rule
only after it has HELD for N consecutive frames (`HeldForFrames`), which is the temporal rule
that keeps a one-frame misdetection from stopping the car.

Interface (anything with these two methods and a `name` is a rule):

    update(t_s, people) -> Person | None     the person that satisfied the rule this frame, or None
    reset() -> None                          forget any history (the car moved on, a run began)

Built in:

  person_present   any person with a good enough score (optionally within a maximum range)
  arms_raised      both wrists above their shoulders by a margin of the body's own scale
  one_arm_raised   either wrist above its shoulder (a wave, or a hand up)
  arm_swing        a wrist moving fast relative to the body: the temporal rule behind punches and
                   kicks' upper-body half, stateful (history of wrist positions)

Adding a rule: write a class with that interface and either call `register_rule(name, factory)`
or pass `rule:=package.module:factory` to the spiderbot launch (`load_rule_factory`). A factory
takes the same keyword settings as the built-ins (RuleSettings) and returns a rule.

CONVENTIONS. Keypoints are in image pixels, y DOWN, so a raised wrist has a SMALLER y than its
shoulder. Body scale is the shoulder width in pixels, floored at a fraction of the box height so
a side-on person (shoulders overlapping) does not collapse the scale to nothing. Keypoint
confidence below `keypoint_confidence` means "not seen": the rule treats that joint as absent and
the check as failed (a missing wrist never counts as a raised arm).

CHOICE OF PERSON. When several people satisfy a rule the one returned is the nearest by valid
LiDAR range, else the highest score, the same preference as racer_perception's person tracker.
"""

from __future__ import annotations

import importlib
import math
from collections import deque
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Protocol

from racer_perception.keypoints import (
    LEFT_SHOULDER,
    LEFT_WRIST,
    RIGHT_SHOULDER,
    RIGHT_WRIST,
    Keypoint,
)


@dataclass(frozen=True)
class Person:
    """One person in one frame: what the rules (and the state machine) read."""

    score: float
    bearing_rad: float  # vehicle frame, left positive
    range_m: float  # NaN when range_valid is False
    range_valid: bool
    box_height_px: float = 0.0
    keypoints: tuple[Keypoint, ...] = ()


class FrameRule(Protocol):
    name: str

    def update(self, t_s: float, people: Sequence[Person]) -> Person | None: ...

    def reset(self) -> None: ...


@dataclass(frozen=True)
class RuleSettings:
    """Keyword settings every built-in rule factory accepts (a custom factory may ignore some)."""

    min_score: float = 0.5
    keypoint_confidence: float = 0.5
    #: How far above the shoulder a wrist must be, in body scales, to count as raised.
    raise_margin: float = 0.5
    #: person_present only: ignore people farther than this (NaN: no limit).
    max_range_m: float = math.nan
    #: arm_swing: wrist speed (body scales per second) that counts as fast, and the window.
    swing_speed_scale_per_s: float = 4.0
    swing_window_s: float = 0.6


# -- geometry ----------------------------------------------------------------------------


def _seen(person: Person, index: int, min_conf: float) -> Keypoint | None:
    if index >= len(person.keypoints):
        return None
    k = person.keypoints[index]
    if not (math.isfinite(k.x_px) and math.isfinite(k.y_px)) or k.confidence < min_conf:
        return None
    return k


def body_scale_px(person: Person, min_conf: float) -> float | None:
    """Shoulder width in pixels, floored at 0.2 x the box height. None without both shoulders."""
    left = _seen(person, LEFT_SHOULDER, min_conf)
    right = _seen(person, RIGHT_SHOULDER, min_conf)
    if left is None or right is None:
        return None
    width = math.hypot(left.x_px - right.x_px, left.y_px - right.y_px)
    return max(width, 0.2 * max(person.box_height_px, 0.0)) or None


def wrist_above_shoulder(
    person: Person, wrist_index: int, shoulder_index: int, settings: RuleSettings
) -> bool:
    wrist = _seen(person, wrist_index, settings.keypoint_confidence)
    shoulder = _seen(person, shoulder_index, settings.keypoint_confidence)
    scale = body_scale_px(person, settings.keypoint_confidence)
    if wrist is None or shoulder is None or scale is None:
        return False
    return wrist.y_px < shoulder.y_px - settings.raise_margin * scale


def arms_raised(person: Person, settings: RuleSettings) -> bool:
    return wrist_above_shoulder(
        person, LEFT_WRIST, LEFT_SHOULDER, settings
    ) and wrist_above_shoulder(person, RIGHT_WRIST, RIGHT_SHOULDER, settings)


def one_arm_raised(person: Person, settings: RuleSettings) -> bool:
    return wrist_above_shoulder(
        person, LEFT_WRIST, LEFT_SHOULDER, settings
    ) or wrist_above_shoulder(person, RIGHT_WRIST, RIGHT_SHOULDER, settings)


def person_present(person: Person, settings: RuleSettings) -> bool:
    if person.score < settings.min_score:
        return False
    if math.isfinite(settings.max_range_m):
        return person.range_valid and person.range_m <= settings.max_range_m
    return True


def choose_person(people: Sequence[Person]) -> Person | None:
    """Nearest by valid range, else the highest score; None for an empty list."""
    if not people:
        return None
    ranged = [p for p in people if p.range_valid and math.isfinite(p.range_m)]
    if ranged:
        return min(ranged, key=lambda p: p.range_m)
    return max(people, key=lambda p: p.score)


# -- rules -------------------------------------------------------------------------------


@dataclass
class PredicateRule:
    """A stateless rule from a per-person test. Needs `min_score` from the settings too."""

    name: str
    predicate: Callable[[Person, RuleSettings], bool]
    settings: RuleSettings = field(default_factory=RuleSettings)

    def update(self, t_s: float, people: Sequence[Person]) -> Person | None:
        return choose_person(
            [
                p
                for p in people
                if p.score >= self.settings.min_score and self.predicate(p, self.settings)
            ]
        )

    def reset(self) -> None:
        return None


@dataclass
class ArmSwingRule:
    """A wrist moving fast relative to the body (the upper-body half of a punch).

    Each frame the chosen person's wrists are expressed relative to the shoulder midpoint and
    divided by the body scale, so walking, approaching the car and camera zoom cancel out. The
    rule fires when, within `swing_window_s`, a wrist's speed between two consecutive frames
    reaches `swing_speed_scale_per_s` body scales per second. Slow reaching and keypoint jitter
    stay below it (jitter is at most a few hundredths of a scale per frame). The tracked person
    is the one `choose_person` picks each frame, so with several people it follows the nearest.
    A frame with no usable person clears the history.
    """

    settings: RuleSettings = field(default_factory=RuleSettings)
    name: str = "arm_swing"
    _history: deque = field(default_factory=deque, init=False, repr=False)

    def reset(self) -> None:
        self._history.clear()

    def _wrist_positions(self, person: Person) -> tuple[tuple[float, float] | None, ...] | None:
        conf = self.settings.keypoint_confidence
        left = _seen(person, LEFT_SHOULDER, conf)
        right = _seen(person, RIGHT_SHOULDER, conf)
        scale = body_scale_px(person, conf)
        if left is None or right is None or scale is None:
            return None
        mid_x, mid_y = (left.x_px + right.x_px) / 2.0, (left.y_px + right.y_px) / 2.0
        out: list[tuple[float, float] | None] = []
        for idx in (LEFT_WRIST, RIGHT_WRIST):
            wrist = _seen(person, idx, conf)
            out.append(
                None
                if wrist is None
                else ((wrist.x_px - mid_x) / scale, (wrist.y_px - mid_y) / scale)
            )
        return tuple(out)

    def update(self, t_s: float, people: Sequence[Person]) -> Person | None:
        person = choose_person([p for p in people if p.score >= self.settings.min_score])
        positions = self._wrist_positions(person) if person is not None else None
        if person is None or positions is None:
            self._history.clear()
            return None
        while self._history and (
            t_s - self._history[0][0] > self.settings.swing_window_s or self._history[0][0] > t_s
        ):
            self._history.popleft()
        fired = False
        if self._history:
            prev_t, prev_pos = self._history[-1]
            dt = t_s - prev_t
            if dt > 0.0:
                for now, before in zip(positions, prev_pos, strict=True):
                    if now is None or before is None:
                        continue
                    speed = math.hypot(now[0] - before[0], now[1] - before[1]) / dt
                    if speed >= self.settings.swing_speed_scale_per_s:
                        fired = True
        self._history.append((t_s, positions))
        return person if fired else None


@dataclass(frozen=True)
class Trigger:
    """A rule that held for enough frames: the person, and how many frames it held."""

    person: Person
    frames: int
    t_s: float


@dataclass
class HeldForFrames:
    """The temporal rule: `rule` must match on `frames` CONSECUTIVE frames.

    One `update` call is one frame (one detector output, empty ones included: an empty frame
    is a frame where the rule did not match). The count also resets if more than `max_gap_s`
    passes between frames (a stalled detector must not let two matches minutes apart count as
    consecutive). Returns a Trigger on every frame from the Nth consecutive match on.
    """

    rule: FrameRule
    frames: int = 3
    max_gap_s: float = 1.0
    _count: int = field(default=0, init=False, repr=False)
    _last_t_s: float = field(default=math.nan, init=False, repr=False)

    def __post_init__(self) -> None:
        if self.frames < 1:
            raise ValueError(f"frames must be at least 1, got {self.frames}")
        if not (math.isfinite(self.max_gap_s) and self.max_gap_s > 0.0):
            raise ValueError(f"max_gap_s must be positive and finite, got {self.max_gap_s}")

    @property
    def name(self) -> str:
        return self.rule.name

    @property
    def count(self) -> int:
        return self._count

    def reset(self) -> None:
        self._count = 0
        self._last_t_s = math.nan
        self.rule.reset()

    def update(self, t_s: float, people: Sequence[Person]) -> Trigger | None:
        if math.isfinite(self._last_t_s) and not 0.0 <= t_s - self._last_t_s <= self.max_gap_s:
            self._count = 0
            self.rule.reset()
        self._last_t_s = t_s
        hit = self.rule.update(t_s, people)
        if hit is None:
            self._count = 0
            return None
        self._count += 1
        if self._count >= self.frames:
            return Trigger(person=hit, frames=self._count, t_s=t_s)
        return None


# -- registry ----------------------------------------------------------------------------

RuleFactory = Callable[[RuleSettings], FrameRule]

_REGISTRY: dict[str, RuleFactory] = {
    "person_present": lambda s: PredicateRule("person_present", person_present, s),
    "arms_raised": lambda s: PredicateRule("arms_raised", arms_raised, s),
    "one_arm_raised": lambda s: PredicateRule("one_arm_raised", one_arm_raised, s),
    "arm_swing": lambda s: ArmSwingRule(s),
}


def rule_names() -> list[str]:
    return sorted(_REGISTRY)


def register_rule(name: str, factory: RuleFactory) -> None:
    """Add a rule under `name`. Refuses to replace an existing one (a typo must not shadow)."""
    if not name or not name.replace("_", "").isalnum():
        raise ValueError(f"rule name must be letters, digits and underscores, got {name!r}")
    if name in _REGISTRY:
        raise ValueError(f"a rule called {name!r} is already registered")
    _REGISTRY[name] = factory


def load_rule_factory(spec: str) -> RuleFactory:
    """Import `package.module:function`, the pluggable-rule hook for `rule:=pkg.mod:func`."""
    module_name, sep, attr = spec.partition(":")
    if not sep or not module_name or not attr:
        raise ValueError(f"rule factory must look like 'package.module:function', got {spec!r}")
    factory = getattr(importlib.import_module(module_name), attr, None)
    if not callable(factory):
        raise TypeError(f"{spec!r} is not a callable")
    return factory


def build_rule(name_or_spec: str, settings: RuleSettings | None = None) -> FrameRule:
    """A rule by built-in or registered name, or by `package.module:function`."""
    settings = settings or RuleSettings()
    if ":" in name_or_spec:
        return load_rule_factory(name_or_spec)(settings)
    factory = _REGISTRY.get(name_or_spec)
    if factory is None:
        raise ValueError(f"unknown rule {name_or_spec!r}: choose one of {rule_names()}")
    return factory(settings)
