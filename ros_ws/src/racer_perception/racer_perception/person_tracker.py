"""Pure target selection: the nearest person, held over short dropouts (roadmap 5.1).

Feeds /perception/target for the follow-me and approach behaviours to come (roadmap 5.2,
5.6). No ROS: operates on fusion.FusedObject records, unit-tested without rclpy.

RULE. Each update takes the fused objects of one frame and keeps those of class `class_name`
(default "person") with score >= `min_score`. Among them:

  1. persons with a valid LiDAR range: the NEAREST wins;
  2. otherwise (no ranges at all): the highest-score person wins, and its range_valid stays
     False so a consumer can refuse to drive on it.

HOLD-OVER. When a frame has no qualifying person, the last target is returned unchanged for
up to `hold_s` seconds (default 1.0) after it was last SEEN, then None. The held target keeps
its original stamp, so a consumer computes its age from the stamp; nothing here rewrites
history. `update` returns (target, held) so the node can log the transition.

This is nearest-person selection, not identity tracking: two people at similar range can swap
(the follow-me task, 5.6, will need appearance or an operator lock). Documented in
docs/notes/perception-2026-10-23.md.
"""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass, field

from racer_perception.fusion import FusedObject


@dataclass
class PersonTracker:
    class_name: str = "person"
    hold_s: float = 1.0
    min_score: float = 0.0
    _target: FusedObject | None = field(default=None, init=False, repr=False)
    _seen_at_s: float = field(default=math.nan, init=False, repr=False)

    def __post_init__(self) -> None:
        if not (math.isfinite(self.hold_s) and self.hold_s >= 0.0):
            raise ValueError(f"hold_s must be finite and >= 0, got {self.hold_s}")

    @property
    def target(self) -> FusedObject | None:
        return self._target

    def _choose(self, objects: Iterable[FusedObject]) -> FusedObject | None:
        people = [
            o for o in objects if o.class_name == self.class_name and o.score >= self.min_score
        ]
        ranged = [o for o in people if o.range_valid]
        if ranged:
            return min(ranged, key=lambda o: o.range_m)
        if people:
            return max(people, key=lambda o: o.score)
        return None

    def update(
        self, objects: Iterable[FusedObject], now_s: float
    ) -> tuple[FusedObject | None, bool]:
        """Return (target, held). held is True when the target is a hold-over."""
        chosen = self._choose(objects)
        if chosen is not None:
            self._target = chosen
            self._seen_at_s = now_s
            return chosen, False
        if self._target is not None and 0.0 <= now_s - self._seen_at_s <= self.hold_s:
            return self._target, True
        self._target = None
        return None, False

    def reset(self) -> None:
        self._target = None
        self._seen_at_s = math.nan
