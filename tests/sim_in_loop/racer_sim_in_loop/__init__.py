"""L5 sim-in-loop test harness scaffold (roadmap task 0.9, claude-docs/12-testing.md).

``runner`` launches a headless, seeded scenario (an env + a controller
callable) and records the resulting trajectory. ``assertions`` checks that
trajectory against the regression properties L5 cares about: lap
completion, wall contact, lap-time band, and trajectory-vs-reference
tolerance. Neither module imports f1tenth_gym or any real controller --
that wiring belongs to the tasks that actually have a tracker and a
reference track (S.2) or a dynamics model to regress against (S.6).
"""

from __future__ import annotations
