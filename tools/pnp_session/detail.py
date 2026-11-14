"""Numbers out of the `detail` strings of /safety/events.

The formats are racer_safety's (gate_logic_formatting.cpp, std::to_string of doubles, so six
decimals):

    rate_limit  command rate-limited relative to previous output (dt=0.006800s)
    watchdog    /drive_raw stale or age invalid (timeout=0.060000s, age=0.349454s); braking

A string that does not match exactly is `None`: it is counted as unparsed and shown, never
guessed. A non-finite number ("inf", "nan", which std::to_string prints for a missing command) does
not match either.
"""

from __future__ import annotations

import re
from dataclasses import dataclass

_NUM = r"(\d+(?:\.\d+)?)"
_RATE_LIMIT = re.compile(
    r"^command rate-limited relative to previous output \(dt=" + _NUM + r"s\)$"
)
_WATCHDOG = re.compile(
    r"^/drive_raw stale or age invalid \(timeout=" + _NUM + r"s, age=" + _NUM + r"s\); braking$"
)


@dataclass(frozen=True)
class WatchdogDetail:
    timeout_s: float
    age_s: float


def parse_rate_limit_dt(detail: str) -> float | None:
    """The limiter's own dt in seconds, or None when the string does not parse."""
    m = _RATE_LIMIT.match(detail.strip())
    return float(m.group(1)) if m else None


def parse_watchdog(detail: str) -> WatchdogDetail | None:
    """The watchdog's timeout and the age it saw, or None when the string does not parse."""
    m = _WATCHDOG.match(detail.strip())
    return WatchdogDetail(float(m.group(1)), float(m.group(2))) if m else None
