"""racer_sim_regression: the S.6 sim dynamics regression battery.

Roadmap task S.6 (claude-docs/01-roadmap.md), claude-docs/12-testing.md L5
"Model-upgrade regression": a fixed, seeded battery of sysid-style
maneuvers (throttle step, steering step, constant-radius circle at a few
speeds, coastdown -- claude-docs/07-sim-and-sysid.md's real-world battery,
run here in racer_gym instead of on the venue surface) run through
sim/racer_gym's dynamics and checked against committed references
(racer_sim_regression/references/) using tests/replay_harness's
golden/tolerance engine, so every racer_gym dynamics change is caught by CI
(see .github/workflows/ci.yml's sim-regression-battery job).

See battery.py for the top-level API (`run_battery`,
`compare_battery_to_references`), maneuvers.py for what each maneuver does
and why, and tolerances.py for the tolerance calibration story.
"""

from __future__ import annotations
