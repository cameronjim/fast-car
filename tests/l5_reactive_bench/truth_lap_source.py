"""The truth oracle of battery B9 (brief 51): a lap source for the simulator that reads the
ground-truth pose and speaks the camera lap counter's JSON. Pure, no ROS.

D9 feeds the lap memory (`lap_source truth`) the truth line crossing, so that the battery tests
the learning through the new path and the fallback, and nothing about lap counting: the oracle is
the answer key, D9-S0 only checks that it matches the registered truth and survives the wiring.

* The start line is the ground-truth pose at the first ground-truth odometry sample at or after
  the stamp it was armed with (the runner arms it with the first `/scan` stamped at or after the
  planner's first DRIVING report: the registered crossing 0).
* Crossings are counted by `racer_laps.lap_detector.LapDetector` on the ground-truth pose; the
  crossing time is interpolated.
* An event is the camera's `/lap/event` JSON: `lap`, `lap_time_s`, `crossing_time_s`,
  `distance_m`, `peak_similarity` 1.0 and `source` truth. A status is the camera's `/lap/status`:
  state running, laps, similarity 1.0.
* The fault plan (condition f, the yaml's `fault_plan`): the listed truth crossings are not
  published; one false event comes at a share of the median ground-truth lap path since the last
  crossing of the lap before the named one; one crossing is published twice, a delay apart, with
  the same content. Published events are numbered consecutively over what was published, as a
  camera that does not know what it missed would. Lap time and distance of an event are measured
  from the previous published event.

The registered truth is computed after the run from the recording and never reads this module.
"""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

try:
    from racer_laps.lap_detector import LapDetector
except ImportError:  # the L1 job of this package: racer_laps from the checkout, numpy only
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "ros_ws/src/racer_laps/src"))
    from racer_laps.lap_detector import LapDetector


@dataclass(frozen=True)
class FaultPlan:
    """The `fault_plan` of the pre-registration (`lap_battery_b9.fault_plan`)."""

    drop_crossings: tuple[int, ...] = ()
    false_lap: int | None = None
    false_share: float = 0.6
    duplicate_crossing: int | None = None
    duplicate_delay_s: float = 0.2
    consecutive: bool = True

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> FaultPlan:
        if d.get("numbering", "consecutive") != "consecutive":
            raise ValueError("the fault plan numbers published events consecutively")
        false = d.get("false_event") or {}
        dup = d.get("duplicate_truth_crossing") or {}
        return cls(
            drop_crossings=tuple(int(k) for k in d.get("drop_truth_crossings", ())),
            false_lap=int(false["lap"]) if false else None,
            false_share=float(false.get("share_of_median_truth_lap_path", 0.6)),
            duplicate_crossing=int(dup["crossing"]) if dup else None,
            duplicate_delay_s=float(dup.get("delay_s", 0.2)),
        )

    @classmethod
    def from_prereg(cls, path: Path) -> FaultPlan:
        """From `lap_battery_b9.fault_plan` of the pre-registration yaml (the caller has checked
        the file's hash)."""
        import yaml

        data = yaml.safe_load(Path(path).read_text())
        return cls.from_dict(data["lap_battery_b9"]["fault_plan"])

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "drop_truth_crossings": list(self.drop_crossings),
            "numbering": "consecutive",
        }
        if self.false_lap is not None:
            out["false_event"] = {
                "lap": self.false_lap,
                "share_of_median_truth_lap_path": self.false_share,
            }
        if self.duplicate_crossing is not None:
            out["duplicate_truth_crossing"] = {
                "crossing": self.duplicate_crossing,
                "delay_s": self.duplicate_delay_s,
            }
        return out


@dataclass(frozen=True)
class OracleEvent:
    """One message the oracle publishes."""

    payload: dict[str, Any]
    kind: str  # true, false or duplicate
    truth_crossing: int | None  # the ground-truth crossing it stands for (None for a false event)
    publish_t: float  # the ground-truth stamp of the sample that produced it


@dataclass
class TruthLapSource:
    plan: FaultPlan = field(default_factory=FaultPlan)
    min_lap_length_m: float = 3.0
    line_length_m: float = 1.5
    heading_tolerance_rad: float = math.radians(60.0)

    def __post_init__(self) -> None:
        self._detector = LapDetector(
            self.min_lap_length_m, self.line_length_m, self.heading_tolerance_rad
        )
        self._arm_t: float | None = None
        self._start_t: float | None = None
        self._last: tuple[float, float, float] | None = None  # t, x, y
        self._cum_m = 0.0  # ground-truth path since the start
        self._cum_at_crossing: list[float] = []  # cumulative path at truth crossing k (index k-1)
        self._lap_lengths: list[float] = []  # ground-truth lap paths so far
        self._published = 0
        self._last_pub_t = math.nan  # crossing time of the last published event
        self._last_pub_cum = 0.0
        self._false_done = False
        self._pending_dup: list[tuple[float, OracleEvent]] = []
        self._speed = 0.0
        self.truth_crossings: list[float] = []

    # -- state --------------------------------------------------------------------------------

    @property
    def started(self) -> bool:
        return self._start_t is not None

    @property
    def truth_start_t(self) -> float | None:
        """The ground-truth stamp the start line was taken at (None before the start)."""
        return self._start_t

    def arm(self, t_stamp: float) -> None:
        """Take the start line at the first ground-truth sample at or after `t_stamp`."""
        if self._arm_t is None:
            self._arm_t = float(t_stamp)

    def status(self) -> dict[str, Any]:
        """The camera's `/lap/status`: the oracle is always `running` and always sees its
        reference (similarity 1.0)."""
        return {
            "state": "running",
            "similarity": 1.0,
            "laps": self._published,
            "armed": self.started,
            "speed_mps": self._speed,
            "distance_m": self._cum_m,
        }

    # -- the ground truth ------------------------------------------------------------------------

    def on_truth_odom(self, t: float, x: float, y: float, yaw: float) -> list[OracleEvent]:
        """One ground-truth pose. Returns the events due now, in publishing order."""
        out: list[OracleEvent] = []
        if self._pending_dup:
            due = [p for p in self._pending_dup if p[0] <= t]
            self._pending_dup = [p for p in self._pending_dup if p[0] > t]
            out.extend(OracleEvent(p[1].payload, p[1].kind, p[1].truth_crossing, t) for p in due)
        if self._arm_t is None or (self._start_t is None and t < self._arm_t):
            return out
        if self._start_t is None:
            self._start_t = t
            self._detector.set_start(t, x, y, yaw)
            self._last = (t, x, y)
            return out
        assert self._last is not None
        t0, x0, y0 = self._last
        if t <= t0:
            return out
        seg = math.hypot(x - x0, y - y0)
        self._speed = seg / (t - t0)
        path_before = self._detector.path_since_crossing_m
        cum_before = self._cum_m
        self._cum_m += seg
        crossing = self._detector.update(t, x, y, yaw)
        self._last = (t, x, y)
        k_next = len(self._cum_at_crossing) + 1
        # the false event: part way through lap `false_lap`, by ground-truth path since the last
        # crossing, against the median of the ground-truth lap paths so far
        if (
            not self._false_done
            and self.plan.false_lap is not None
            and k_next == self.plan.false_lap
            and self._lap_lengths
        ):
            target = self.plan.false_share * statistics.median(self._lap_lengths)
            if crossing is None and path_before + seg >= target > path_before:
                f = (target - path_before) / seg if seg > 0.0 else 1.0
                t_false = t0 + f * (t - t0)
                cum_false = cum_before + f * seg
                out.append(self._publish_false(t_false, cum_false, t))
        if crossing is not None:
            out.extend(self._on_crossing(crossing.t_cross_s, crossing.length_m, cum_before, seg, t))
        return out

    def _on_crossing(
        self, t_cross: float, length_m: float, cum_before: float, seg: float, t_now: float
    ) -> list[OracleEvent]:
        k = len(self._cum_at_crossing) + 1
        # cumulative path at the crossing: this lap's path ends where the detector says
        previous_cum = self._cum_at_crossing[-1] if self._cum_at_crossing else 0.0
        cum_cross = previous_cum + length_m
        self._cum_at_crossing.append(cum_cross)
        self._lap_lengths.append(length_m)
        self.truth_crossings.append(t_cross)
        if k in self.plan.drop_crossings:
            return []
        event = self._event(t_cross, cum_cross, "true", k, t_now)
        out = [event]
        if k == self.plan.duplicate_crossing:
            copy = OracleEvent(
                dict(event.payload), "duplicate", k, t_now + self.plan.duplicate_delay_s
            )
            self._pending_dup.append((t_cross + self.plan.duplicate_delay_s, copy))
        return out

    def _event(
        self, t_cross: float, cum: float, kind: str, truth_crossing: int | None, t_now: float
    ) -> OracleEvent:
        self._published += 1
        start = self._start_t if math.isnan(self._last_pub_t) else self._last_pub_t
        assert start is not None
        payload = {
            "lap": self._published,
            "lap_time_s": t_cross - start,
            "crossing_time_s": t_cross,
            "distance_m": cum - self._last_pub_cum,
            "peak_similarity": 1.0,
            "source": "truth",
        }
        self._last_pub_t = t_cross
        self._last_pub_cum = cum
        return OracleEvent(payload, kind, truth_crossing, t_now)

    def _publish_false(self, t_false: float, cum_false: float, t_now: float) -> OracleEvent:
        self._false_done = True
        return self._event(t_false, cum_false, "false", None, t_now)
