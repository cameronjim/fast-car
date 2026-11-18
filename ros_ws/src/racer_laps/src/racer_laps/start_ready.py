"""The start rule of brief 51c: the planner is not started until the lap node on the external lap
source says the source is ready.

`StartReadyGate` is one pure rule used by both starters: `pnp.launch.py` on the car (through the
`lap_source_ready_wait` node) and `run_startup` in the simulator's runner. The planner process, the
only publisher of `/drive_raw`, is not started until the lap node has published `source_ready_now`
and `source_alive` on `ready_statuses_required` consecutive statuses, and the wait is bounded: when
it runs out the starter refuses, the planner is never started, and the reason is the lap node's own
`source_not_ready_reason`.

Why: the lap node fixes `source_ready` once, at the first wheel odometry after the planner's first
DRIVING report, and nothing ordered that instant after the source's first status (design 13
section 21.6). The node's own rule (a run that starts not ready is doubtful on every lap) stays as
the second layer, unchanged.

Constants: the `start_gate` block of `evaluation/prereg/pnp-v0.5-laps-camera-2.yaml`, fixed by the
pre-registration; a test pins every default to the yaml. Nothing here is a parameter of a node or a
launch argument: there is no way to shorten the wait, skip it or start on a refusal.

A status says ready when `lap_source` is `camera` or `truth`, `phase` is a string other than
`running`, `source_ready_now` is the boolean true and `source_alive` is the boolean true. Anything
else (a missing key, a value of the wrong type, not a mapping) resets the count. Two statuses
further apart than `status_gap_max_s` are not consecutive: the count restarts at the later one. A
status older than that at decision time resets the count (a lap node that died after one good
status never gives `start`). A lap node whose `phase` is already `running` refuses at once: the run
it would gate has started. `start` and `refuse` are final. The deadline wins: a status that arrives after `wait_s` is not
counted, it ends the wait with a refusal (outside review of brief 51c, finding 8).

Numpy-free, ROS-free, standard library only.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

#: The sources the rule serves (a status of any other lap source never says ready).
READY_SOURCES = ("camera", "truth")
#: Tolerance of "exactly at the limit" against the float noise of a difference of two times (the
#: convention of `racer_laps.external_gate.EPS`).
EPS = 1e-9

VERDICT_WAIT = "wait"
VERDICT_START = "start"
VERDICT_REFUSE = "refuse"

REFUSAL_NO_STATUS = "no lap node status"
REFUSAL_RUNNING = "lap node already running"
#: The reason of a refusal when the last status says not ready but gives no `source_not_ready_reason`
#: of its own: the first condition it fails.
REFUSAL_NO_REASON = "the lap node's status does not say the source is ready"


@dataclass(frozen=True)
class StartGateConfig:
    """The `start_gate` block of `evaluation/prereg/pnp-v0.5-laps-camera-2.yaml`."""

    #: Consecutive lap node statuses that must say ready.
    ready_statuses_required: int = 2
    #: Two statuses further apart than this are not consecutive: the count restarts.
    status_gap_max_s: float = 2.5
    #: From the gate's creation; then the verdict is `refuse`.
    wait_s: float = 30.0

    def __post_init__(self) -> None:
        if self.ready_statuses_required < 1:
            raise ValueError("ready_statuses_required must be at least 1")
        for name in ("status_gap_max_s", "wait_s"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"StartGateConfig.{name} must be finite and positive")


def says_ready(status: object) -> bool:
    """True when a parsed lap node status says the source is ready (the rule's whole reading of a
    status, apart from `phase` running which refuses)."""
    if not isinstance(status, Mapping):
        return False
    source, phase = status.get("lap_source"), status.get("phase")
    if not isinstance(source, str) or source not in READY_SOURCES:
        return False
    if not isinstance(phase, str) or phase == "running":
        return False
    return status.get("source_ready_now") is True and status.get("source_alive") is True


def _why_not_ready(status: object) -> str:
    """The reason a status that does not say ready gives: the lap node's own, else the first
    condition it fails."""
    if isinstance(status, Mapping):
        reason = status.get("source_not_ready_reason")
        if isinstance(reason, str) and reason:
            return reason
        source = status.get("lap_source")
        if not isinstance(source, str) or source not in READY_SOURCES:
            return f"lap node source is {source!r}, not one of {list(READY_SOURCES)}"
    return REFUSAL_NO_REASON


class StartReadyGate:
    """See the module docstring. `t0_s` is the gate's creation time; every time the gate is given
    is on one clock (the node's, or the wall clock). Non-finite times are ignored and counted, and
    so is a status stamped earlier than the one before it."""

    def __init__(self, cfg: StartGateConfig, t0_s: float) -> None:
        if not math.isfinite(t0_s):
            raise ValueError("the gate's creation time must be finite")
        self.cfg = cfg
        self._t0 = float(t0_s)
        self._verdict = VERDICT_WAIT
        self._reason = ""
        self._count = 0  # consecutive ready statuses so far
        self._last_ready_t = math.nan  # time of the last status counted
        self._last_status: object = None
        self._last_t = -math.inf  # time of the last status taken in
        self._statuses = 0
        self._first_status_t: float | None = None
        self._ready_first_t: float | None = None
        self._start_t: float | None = None
        self._refuse_t: float | None = None
        self._ignored_non_finite = 0
        self._ignored_non_monotone = 0

    # -- inputs -------------------------------------------------------------------------------

    def on_status(self, t_s: float, status: Mapping[str, Any]) -> None:
        """One lap node status (a parsed `<prefix>/status`) that arrived at `t_s`."""
        if not math.isfinite(t_s):
            self._ignored_non_finite += 1
            return
        if self._verdict != VERDICT_WAIT:
            return  # final
        if t_s - self._t0 > self.cfg.wait_s + EPS:
            # the wait was over before this status arrived: the deadline wins, whatever the count
            # (a caller that polled late must not turn two late statuses into a start). The late
            # status is not counted; it still gives the refusal its reason, the freshest one.
            self._statuses += 1
            self._last_status = status
            self._expire(t_s)
            return
        if t_s < self._last_t:
            self._ignored_non_monotone += 1
            return
        self._last_t = float(t_s)
        self._statuses += 1
        self._last_status = status
        if self._first_status_t is None:
            self._first_status_t = float(t_s)
        if isinstance(status, Mapping) and status.get("phase") == "running":
            self._count = 0
            self._finish(VERDICT_REFUSE, REFUSAL_RUNNING, t_s)
            return
        if not says_ready(status):
            self._count = 0
            return
        if self._ready_first_t is None:
            self._ready_first_t = float(t_s)
        consecutive = (
            self._count > 0 and t_s - self._last_ready_t <= self.cfg.status_gap_max_s + EPS
        )
        self._count = self._count + 1 if consecutive else 1
        self._last_ready_t = float(t_s)

    def decide(self, t_s: float) -> tuple[str, str]:
        """`(verdict, reason)` at `t_s`: `wait`, `start` (reason empty) or `refuse`."""
        if self._verdict != VERDICT_WAIT:
            return self._verdict, self._reason
        if not math.isfinite(t_s):
            self._ignored_non_finite += 1
            return VERDICT_WAIT, ""
        if self._count > 0 and t_s - self._last_ready_t > self.cfg.status_gap_max_s + EPS:
            self._count = 0  # the lap node went quiet after its last good status
        if self._count >= self.cfg.ready_statuses_required:
            self._finish(VERDICT_START, "", t_s)
        elif t_s - self._t0 > self.cfg.wait_s + EPS:
            self._expire(t_s)
        return self._verdict, self._reason

    def _expire(self, t_s: float) -> None:
        reason = REFUSAL_NO_STATUS if self._statuses == 0 else _why_not_ready(self._last_status)
        self._finish(VERDICT_REFUSE, reason, t_s)

    def _finish(self, verdict: str, reason: str, t_s: float) -> None:
        self._verdict, self._reason = verdict, reason
        if verdict == VERDICT_START:
            self._start_t = float(t_s)
        else:
            self._refuse_t = float(t_s)

    # -- the record ---------------------------------------------------------------------------

    def trace(self) -> dict[str, Any]:
        """What the gate saw and decided, for the run's report (JSON-ready)."""
        return {
            "created_s": self._t0,
            "statuses": self._statuses,
            "first_status_s": self._first_status_t,
            "ready_first_s": self._ready_first_t,
            "ready_count": self._count,
            "start_s": self._start_t,
            "refuse_s": self._refuse_t,
            "verdict": self._verdict,
            "reason": self._reason,
            "ignored_non_finite": self._ignored_non_finite,
            "ignored_non_monotone": self._ignored_non_monotone,
            "config": {
                "ready_statuses_required": self.cfg.ready_statuses_required,
                "status_gap_max_s": self.cfg.status_gap_max_s,
                "wait_s": self.cfg.wait_s,
            },
        }
