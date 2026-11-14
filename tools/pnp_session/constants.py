"""Constants of the session report. Each quotes the section it comes from; none is a tuning knob.

Pre-registration files (docs/notes/preregistration-pnp-v0.3.md and preregistration-pnp-v0.2.md)
are frozen. A number here that is not from them is marked "convention" and says what it is for.
"""

from __future__ import annotations

#: Version of the JSON the report writes (what the session note cites).
SCHEMA_VERSION = "pnp-session/1"

#: v0.3 section 5: "the count above each of 40, 60, 80, 100 and 150 ms. The count above 60 ms is
#: what the current watchdog sees; the other counts say what a different limit would have seen."
GAP_LIMITS_S: tuple[float, ...] = (0.040, 0.060, 0.080, 0.100, 0.150)

#: The watchdog limit of safety_node at the time of writing (3 cycles of 20 ms, 13 section 14.1
#: item 4 "safety's 60 ms watchdog"). Used only to pick the row of GAP_LIMITS_S that is compared
#: with the recorded `watchdog` events; it is not read from or written to any safety parameter.
CURRENT_WATCHDOG_LIMIT_S = 0.060

#: Tolerance of "above a limit" (convention): a gap of exactly 60 ms is not above 60 ms even when
#: two stamps in float seconds differ by 0.060000000000000005. Far below any stamp resolution.
ABOVE_EPS_S = 1e-9

#: Design 13 section 4.2 (PlannerStatus: "One per control tick ... (50 Hz)") and 14.7 ("A timer
#: callback that arrives under half a tick after the last publish republishes the last command
#: and does not solve"). A planner status gap under half this is a bunched tick.
PLANNER_TICK_S = 0.020
BUNCHED_BELOW_S = PLANNER_TICK_S / 2.0

#: v0.2 section 4.2: "in the first 2.0 s after the first non-zero command" (the value is
#: racer_eval.interventions.START_WINDOW_S; the report reads it from there).

#: PlannerStatus.msg state values (racer_pnp_msgs).
STATE_NAMES = {
    0: "STARTING",
    1: "DRIVING",
    2: "NO_TRACK",
    3: "NO_SCAN",
    4: "NO_ODOM",
    5: "BLOCKED",
    6: "FAULT",
    7: "RECOVERING",
}
STATE_STARTING = 0
STATE_FAULT = 6

#: Convention of the watchdog agreement table: a recorded `watchdog` engagement at cycle time t is
#: matched to a /drive_raw receive gap [a, b] above the limit when a + limit - tol <= t <= b + tol.
#: One gate cycle (20 ms) plus ten milliseconds for the recorder's own latency. Reporting only.
WATCHDOG_MATCH_TOL_S = 0.030

#: Convention of the rate limiter reproduction check: a replayed engagement is the same event as a
#: recorded one when their cycle stamps are within this. Both come from the safety node's own
#: cycle stamp (the /drive header stamp and the event stamp are the same `now`), so they agree to
#: the nanosecond when the replay's cycle list is the node's.
LIMITER_MATCH_TOL_S = 1e-3

#: v0.3 section 1.5 (D3-C0): "the lap count equals the camera lap counter's in both runs and at
#: least 18 of the 20 lap times agree with it within 0.3 s".
D3C0_TOLERANCE_S = 0.3
D3C0_MIN_WITHIN = 18
D3C0_OF_LAPS = 20
D3C0_MIN_RUNS = 2

#: v0.2 section 4.2, quoted by the baseline check. racer_eval.cli.HAND_COUNT is the same set.
BASELINE_BAG = "2026-11-13T22-20-18_car_teleop"
