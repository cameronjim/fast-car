"""L4 replay/golden test harness (roadmap task 0.9, claude-docs/12-testing.md).

Two independent pieces live here:

- ``golden``: a golden-comparison engine with stated per-field tolerances
  (never exact float equality) and an explicit, loud regeneration flow.
- ``streams`` / ``mutators``: an abstract message-stream interface and
  composable bag-mutation fault injectors (NaNs, timestamp jumps, dropped
  frames, out-of-order messages, frozen sensors) that work today on
  synthetic in-memory streams and are meant to work unmodified on rosbag2
  readers once real bags exist (see ``tests/bags/``, roadmap task 2.8).
"""

from __future__ import annotations
