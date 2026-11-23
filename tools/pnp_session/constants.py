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

#: Lap gate v3 (brief 50): the D7-C0 rule and D8-C0 read their thresholds from the pre-registration
#: yaml files, `d7_c0` of the first and `d8_c0` of the second, each refused unless its SHA-256 is
#: the registered one (the same constants as tests/l5_pnp_laps/lap_battery_b7.py and
#: lap_battery_b8.py).
D7C0_LAPS_YAML = "evaluation/prereg/pnp-v0.5-laps.yaml"
D7C0_LAPS_YAML_SHA256 = "35949c7d86dbdd3c5a3dde11eb0551465ea893b5f26e78dc7de0f1a0890428de"
D8C0_RING_YAML = "evaluation/prereg/pnp-v0.5-laps-ring.yaml"
D8C0_RING_YAML_SHA256 = "74b1dccd0d30910a7e96a45af65a00de62446902380bfb2bae76512d0498dc36"

#: Brief 51 (lap learning fed by the camera lap counter): the pre-registration constants of battery B9,
#: the offline gate D9-B0 and the car gates D9-P0 and D9-C0, `pnp_session lap-src` reads `d9_c0` and
#: `d9_b0` from this file, refused unless its SHA-256 is the registered one. The freeze file is
#: written by PR B; an evaluation bag of D9-B0 is not opened before it exists.
D9_YAML = "evaluation/prereg/pnp-v0.5-laps-camera.yaml"
D9_YAML_SHA256 = "013cb2f8bec3ce96d2abf456361c3de93d8f85da48c01900c374287d7473807f"
D9_FREEZE_FILE = "evaluation/prereg/freeze/b9.json"

#: Brief 51c (the lap source is ready before the planner exists): the second attempt, battery B10,
#: car gates D10-P0 and D10-C0. `pnp_session lap-src --d10` reads its thresholds through this file
#: (refused unless its SHA-256 is the registered one), which names the D9 yaml above and the
#: hash it must have: D10-C0 is D9-C0's items i to v with the same thresholds, plus item vi.
D10_YAML = "evaluation/prereg/pnp-v0.5-laps-camera-2.yaml"
D10_YAML_SHA256 = "823f641c8a40337e1a81aaa440d55976585d19984ae2db55f06f0eee1289bdcd"
