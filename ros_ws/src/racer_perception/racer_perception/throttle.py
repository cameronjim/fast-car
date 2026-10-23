"""Pure frame-rate limiter: process at most `rate_hz` frames per second, drop the rest."""

from __future__ import annotations

import math


class FrameThrottle:
    """accept(now_s) is True when a frame should be processed.

    Uses the receive time the caller passes (the node's monotonic clock), not the image stamp,
    so a camera restart with a jumped stamp cannot freeze the detector. rate_hz <= 0 means
    "no limit" (process every frame).
    """

    def __init__(self, rate_hz: float) -> None:
        if not math.isfinite(rate_hz):
            raise ValueError(f"rate_hz must be finite, got {rate_hz}")
        self.period_s = 1.0 / rate_hz if rate_hz > 0.0 else 0.0
        self._last_s = -math.inf
        self.accepted = 0
        self.dropped = 0

    def accept(self, now_s: float) -> bool:
        if now_s < self._last_s:  # clock stepped back: restart rather than stall
            self._last_s = -math.inf
        if now_s - self._last_s >= self.period_s:
            self._last_s = now_s
            self.accepted += 1
            return True
        self.dropped += 1
        return False
