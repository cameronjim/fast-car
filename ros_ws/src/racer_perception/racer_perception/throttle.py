"""Pure frame-rate limiter: process `rate_hz` frames per second, drop the rest."""

from __future__ import annotations

import math

#: Float slack on the due time, as a fraction of the period, so that a frame on the grid
#: (0.3 = 30 * 0.01) is not dropped for a rounding error.
_EPS_PERIODS = 1e-6


class FrameThrottle:
    """accept(now_s) is True when a frame should be processed.

    The throttle keeps a PHASE: frames are due on a grid `origin + k * period`, and accepting a
    frame moves the next due time one period along the grid, not one period after the frame that
    was accepted. The delivered rate therefore equals `rate_hz` (S1, fix-checklist 2026-11-22)
    for any source at least that fast, including a source at exactly `rate_hz` with jitter (the
    old "period since the last accepted frame" rule dropped every frame that arrived a little
    early and delivered about half). A source slower than `rate_hz` is never dropped.

    After a stall (the next due time is a whole period or more behind now) the grid restarts at
    the current frame, so a camera that comes back does not release a burst. If the clock steps
    back the grid restarts too, rather than stalling.

    Uses the receive time the caller passes (the node's clock), not the image stamp, so a camera
    restart with a jumped stamp cannot freeze the detector. rate_hz <= 0 means "no limit".
    """

    def __init__(self, rate_hz: float) -> None:
        if not math.isfinite(rate_hz):
            raise ValueError(f"rate_hz must be finite, got {rate_hz}")
        self.period_s = 1.0 / rate_hz if rate_hz > 0.0 else 0.0
        self._origin_s: float | None = None
        self._k = 0
        self._last_s = -math.inf
        self.accepted = 0
        self.dropped = 0

    def _accept(self, now_s: float) -> bool:
        self._last_s = now_s
        self.accepted += 1
        return True

    def accept(self, now_s: float) -> bool:
        if self.period_s <= 0.0:
            return self._accept(now_s)
        if now_s < self._last_s:  # clock stepped back: restart rather than stall
            self._origin_s = None
        if self._origin_s is None:
            self._origin_s, self._k = now_s, 1
            return self._accept(now_s)
        due_s = self._origin_s + self._k * self.period_s
        if now_s < due_s - _EPS_PERIODS * self.period_s:
            self.dropped += 1
            return False
        self._k += 1
        next_due_s = self._origin_s + self._k * self.period_s
        if now_s - next_due_s >= self.period_s:  # a whole period behind: do not catch up
            self._origin_s, self._k = now_s, 1
        return self._accept(now_s)
