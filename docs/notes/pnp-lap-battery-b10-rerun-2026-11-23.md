# B10 rerun on main cce6bd3, regression result, 2026-11-23

Owner decision 23 (`claude-docs/owner-decisions-2026-11-16.md`): S1 to S5 left the B10 freeze, so a B10 rerun on the
new main commit is a regression result (decision 17), not new held-out evidence. Original run and constants:
`docs/notes/pnp-lap-battery-b10-2026-11.md`, `evaluation/prereg/pnp-v0.5-laps-camera-2.yaml` (SHA-256
`823f641c8a40337e1a81aaa440d55976585d19984ae2db55f06f0eee1289bdcd`, unchanged).

Same cases (V1 to V3), seeds (81, 82), directions, conditions (a, b, c, f, r), thresholds and scorer
(`battery_b10.py`) as the original. Nothing registered was changed.

## How it was run

- Commit `cce6bd39d30eabda1dbb6b3cd53fc93a05d9493d`, worktree `.claude/worktrees/b10-rerun`, clean.
- `evaluation/prereg/freeze/b10.json` names the original freeze commit, and `check-freeze` refuses it on this commit
  because guarded `racer_laps` code changed (S1 to S5). The rerun used a second freeze record
  (`car-data/pnp/battery/b10-rerun-freeze.json`, outside the repo) that is identical except `freeze_commit` is
  cce6bd3, passed with `--freeze-path`. `check-freeze` on the host passed against it. The repo's freeze file is untouched.
- ros-dev image, built in a container with `ROS_LOCALHOST_ONLY=1`, `battery_b10.py run --workers 2 --domain-base 200`
  (the original used 3 workers, default domain base) because battery B11 was running in another container.
- Run files: `car-data/pnp/battery/v0.5-b10-rerun-cce6bd3/`, log and report alongside.
- 45 runs. One starved attempt: `f_V2_ccw_81` attempt 1 (starved, truth valid). Per the registered rule it was rerun
  alone; attempt 2 was clean and is the run scored. Both attempts are kept. No other attempt was starved, none
  failed to execute, nothing dropped (`drops.json`: dropped [], decided).

## Result

| gate | original B10 | rerun cce6bd3 |
|---|---|---|
| D10-S0 | PASS | PASS (all six items) |
| D10-S1 | PASS | PASS (12 of 12 reach, ratios 0.996 to 1.009, median r 0.438, 0 contacts) |
| D10-S2 | PASS | PASS (all seven items) |
| D9-B0 | carried | not rescored here, still carried from the original note (the event-to-lap check was not rerun on this commit) |

`start_refused`: 3, the three condition r runs, as before (reason `source state 'waiting', not 'running'`).

## What moved

No pass/fail criterion moved. Reported numbers that did:

- Condition f, late lap time (median of laps 8 to 10), f against c: was 1.87 to 2.28, now 1.15 to 1.37
  (V1 about 1.36, V2 about 1.15, V3 about 1.35). T_late(f) fell from about 25.4 s to 15.3 s on V1/V2 and from about
  15.4 s to 9.5 s on V3. Cause: S5 (PR 238, decision 24) makes a lap doubtful only by length, heading or sequence
  with a clean drive hold the cap instead of dropping it to the lap-1 cap, so a camera-only doubtful lap no longer
  slows the car. S2 and S3 planner changes are not separated out here.
- Condition f reference length: V1 about 12.4 to 12.5 m (was 12.0 to 12.2, now close to c), V2 12.1 to 12.2 m (was 12.0 to 12.1), V3 7.5 to 7.6 m (was 7.3). Likely the same S5 cause, not isolated.
- Doubtful laps in f are unchanged: 4, 6, 7, 8 (length, heading) in every run, one `not_monotone` rejection, same confirmed counts (V1 8, V2 7, V3 7 or 8).
- Condition c: T_late within about 0.1 s of the original in all 12 runs, reach ratios 0.996 to 1.009 (was 0.995 to 1.012), median r identical 0.438.
- Reference length of c: unchanged to 0.02 m. Event latency max: 0.016 to 0.026 s (was 0.016 to 0.020), c_V3_cw_82 the high one.
- Drop-rule lap counts: V3_ccw condition a drove 12 laps (original 13); the rule needs 3, so nothing dropped.
- Source health at start over 24 c and f runs: largest status gap 1.01 s (was 1.00), largest age 0.87 s (was 0.91), within the same limits.

## D10-S0: PASS

- ok: (1) the planner was started only after the start gate said start, the first DRIVING report is later than that, and the lap node started ready: 0 exceptions in 12 runs []
- ok: the oracle publishes one event per registered truth crossing 1 to 12, each within 0.3 s of it: 0 exceptions in 12 runs []
- ok: the system closes laps 1 to 12, all confirmed, none doubtful, no event rejected: 0 exceptions []
- ok: every system crossing within 0.3 s of the truth crossing of the same number: 0 exceptions []
- ok: no run failed to execute: 0 errors []
- ok: (6) condition r: the starter refused within 35.0 s, the planner was never started, no /drive_raw message, the ground-truth path at most 0.05 m, and the refusal names the source's reason: 0 exceptions in 3 runs []

## D10-S1: PASS

- ok: 1. 0 wall contacts in the 12 runs of condition c: 0 contacts, runs with a contact: []
- ok: 2. runs of condition c complete 10 laps: 12 of 12 complete, not complete: []
- ok: 3. reach: T_late(c) <= 1.05 x T_late(b) in at least 11 of 12 runs: 12 of 12 reach; ratios [0.996, 0.997, 0.998, 1.002, 1.002, 1.003, 1.003, 1.003, 1.003, 1.006, 1.009, 1.009]
- ok: 4a. T_late below T1 in every completing run: 0 runs not below []
- ok: 4b. median r <= 0.8: median r 0.438 over 12 completing runs

## D10-S2: PASS

- ok: (1) 0 wall contacts: 0 exceptions in 12 runs []
- ok: (2) 12 truth laps completed: 0 exceptions in 12 runs []
- ok: (3) every confirmed system lap is one truth lap (two consecutive truth crossings within 0.3 s): 0 exceptions in 12 runs []
- ok: (4a) the laps that close at truth crossings 5 and 8, the lap closed by the false event and the lap after it are doubtful: 0 exceptions in 12 runs []
- ok: (4b) the duplicate of crossing 10 is rejected: 0 exceptions in 12 runs []
- ok: (5) the cap of the lap after a doubtful lap is at most that of the doubtful lap: 0 exceptions in 12 runs []
- ok: (6) the lowest guidance cap while overdue is the lap-1 cap: 0 exceptions in 12 runs []
- ok: (7) the run started ready through the start gate (D10-S0 item 1): 0 exceptions in 12 runs []

## Reported, no threshold

Late lap time (median of laps 8 to 10), condition f against c:

| run | T_late(f) s | T_late(c) s | f / c |
|---|---|---|---|
| V1/ccw/81 | 15.34 | 11.25 | 1.364 |
| V1/ccw/82 | 15.36 | 11.26 | 1.364 |
| V1/cw/81 | 15.22 | 11.17 | 1.363 |
| V1/cw/82 | 15.23 | 11.19 | 1.361 |
| V2/ccw/81 | 15.72 | 13.52 | 1.163 |
| V2/ccw/82 | 15.67 | 13.62 | 1.151 |
| V2/cw/81 | 15.60 | 13.59 | 1.148 |
| V2/cw/82 | 15.60 | 13.56 | 1.150 |
| V3/ccw/81 | 9.56 | 7.08 | 1.350 |
| V3/ccw/82 | 9.56 | 7.07 | 1.352 |
| V3/cw/81 | 9.51 | 6.95 | 1.368 |
| V3/cw/82 | 9.49 | 7.01 | 1.354 |

Event latency, reference length and every doubtful lap with its reasons:

| run | latency max s | reference length m | confirmed | doubtful laps | rejected |
|---|---|---|---|---|---|
| c_V1_ccw_81 | 0.019 | 12.57 | 13 | {} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 0} |
| c_V1_ccw_82 | 0.017 | 12.56 | 13 | {} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 0} |
| c_V1_cw_81 | 0.019 | 12.45 | 13 | {} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 0} |
| c_V1_cw_82 | 0.019 | 12.45 | 13 | {} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 0} |
| c_V2_ccw_81 | 0.018 | 12.16 | 12 | {} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 0} |
| c_V2_ccw_82 | 0.016 | 12.15 | 12 | {} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 0} |
| c_V2_cw_81 | 0.017 | 12.10 | 12 | {} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 0} |
| c_V2_cw_82 | 0.020 | 12.11 | 12 | {} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 0} |
| c_V3_ccw_81 | 0.019 | 7.64 | 12 | {} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 0} |
| c_V3_ccw_82 | 0.019 | 7.63 | 12 | {} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 0} |
| c_V3_cw_81 | 0.020 | 7.53 | 13 | {} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 0} |
| c_V3_cw_82 | 0.026 | 7.54 | 13 | {} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 0} |
| f_V1_ccw_81 | 0.202 | 12.53 | 8 | {4: ['length', 'heading'], 6: ['length', 'heading'], 7: ['length', 'heading'], 8: ['length', 'heading']} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 1} |
| f_V1_ccw_82 | 0.212 | 12.54 | 8 | {4: ['length', 'heading'], 6: ['length', 'heading'], 7: ['length', 'heading'], 8: ['length', 'heading']} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 1} |
| f_V1_cw_81 | 0.209 | 12.39 | 8 | {4: ['length', 'heading'], 6: ['length', 'heading'], 7: ['length', 'heading'], 8: ['length', 'heading']} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 1} |
| f_V1_cw_82 | 0.219 | 12.40 | 8 | {4: ['length', 'heading'], 6: ['length', 'heading'], 7: ['length', 'heading'], 8: ['length', 'heading']} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 1} |
| f_V2_ccw_81 | 0.207 | 12.15 | 7 | {4: ['length', 'heading'], 6: ['length', 'heading'], 7: ['length', 'heading'], 8: ['length', 'heading']} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 1} |
| f_V2_ccw_82 | 0.205 | 12.15 | 7 | {4: ['length', 'heading'], 6: ['length', 'heading'], 7: ['length', 'heading'], 8: ['length', 'heading']} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 1} |
| f_V2_cw_81 | 0.211 | 12.08 | 7 | {4: ['length', 'heading'], 6: ['length', 'heading'], 7: ['length', 'heading'], 8: ['length', 'heading']} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 1} |
| f_V2_cw_82 | 0.217 | 12.09 | 7 | {4: ['length', 'heading'], 6: ['length', 'heading'], 7: ['length', 'heading'], 8: ['length', 'heading']} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 1} |
| f_V3_ccw_81 | 0.220 | 7.64 | 7 | {4: ['length', 'heading'], 6: ['length', 'heading'], 7: ['length', 'heading'], 8: ['length', 'heading']} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 1} |
| f_V3_ccw_82 | 0.202 | 7.63 | 7 | {4: ['length', 'heading'], 6: ['length', 'heading'], 7: ['length', 'heading'], 8: ['length', 'heading']} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 1} |
| f_V3_cw_81 | 0.213 | 7.53 | 8 | {4: ['length', 'heading'], 6: ['length', 'heading'], 7: ['length', 'heading'], 8: ['length', 'heading']} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 1} |
| f_V3_cw_82 | 0.212 | 7.54 | 8 | {4: ['length', 'heading'], 6: ['length', 'heading'], 7: ['length', 'heading'], 8: ['length', 'heading']} | {'before_start': 0, 'duplicate': 0, 'future': 0, 'latency': 0, 'malformed': 0, 'not_finite': 0, 'not_monotone': 1} |

The start of every run (node time, s; None where it did not happen): lap node first status, first status that said ready, gate start, planner spawn, first DRIVING report, lap 1 start, oracle first status, verdict and reason, the source's health at the start:

| run | lap node first | ready first | gate start | planner spawn | driving first | lap 1 start | oracle first | verdict | reason | ready | age at start s | statuses before | gap max s | stopped |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| c_V1_ccw_81 | 1.64 | 4.00 | 5.00 | 5.00 | 5.62 | 5.63 | 0.00 | start |  | True | 0.63 | 4 | 1.00 | laps |
| c_V1_ccw_82 | 1.38 | 9.00 | 10.00 | 10.00 | 10.63 | 10.65 | 0.00 | start |  | True | 0.65 | 9 | 1.00 | laps |
| c_V1_cw_81 | 1.32 | 4.12 | 5.00 | 5.00 | 5.66 | 5.67 | 0.00 | start |  | True | 0.67 | 4 | 1.00 | laps |
| c_V1_cw_82 | 1.30 | 9.00 | 10.00 | 10.00 | 10.62 | 10.63 | 0.00 | start |  | True | 0.63 | 9 | 1.00 | laps |
| c_V2_ccw_81 | 1.30 | 4.00 | 5.00 | 5.00 | 5.71 | 5.73 | 0.00 | start |  | True | 0.73 | 4 | 1.00 | laps |
| c_V2_ccw_82 | 1.30 | 9.00 | 10.00 | 10.00 | 10.62 | 10.63 | 0.00 | start |  | True | 0.63 | 9 | 1.00 | laps |
| c_V2_cw_81 | 1.33 | 4.00 | 5.00 | 5.00 | 5.66 | 5.67 | 1.00 | start |  | True | 0.67 | 4 | 1.00 | laps |
| c_V2_cw_82 | 4.00 | 11.00 | 12.00 | 12.00 | 12.62 | 12.63 | 0.00 | start |  | True | 0.63 | 9 | 1.00 | laps |
| c_V3_ccw_81 | 4.00 | 6.00 | 7.00 | 7.00 | 7.77 | 7.79 | 0.00 | start |  | True | 0.79 | 4 | 1.00 | laps |
| c_V3_ccw_82 | 1.31 | 9.00 | 10.00 | 10.00 | 10.63 | 10.65 | 0.00 | start |  | True | 0.65 | 9 | 1.01 | laps |
| c_V3_cw_81 | 1.28 | 3.00 | 4.00 | 4.00 | 4.64 | 4.65 | 0.00 | start |  | True | 0.65 | 3 | 1.00 | laps |
| c_V3_cw_82 | 1.30 | 9.00 | 10.00 | 10.00 | 10.64 | 10.65 | 0.00 | start |  | True | 0.65 | 9 | 1.00 | laps |
| f_V1_ccw_81 | 1.33 | 3.00 | 4.00 | 4.00 | 4.62 | 4.63 | 0.00 | start |  | True | 0.63 | 3 | 1.00 | laps |
| f_V1_ccw_82 | 1.29 | 9.00 | 10.00 | 10.00 | 10.67 | 10.69 | 0.00 | start |  | True | 0.69 | 9 | 1.00 | laps |
| f_V1_cw_81 | 4.00 | 6.00 | 7.00 | 7.00 | 7.63 | 7.65 | 0.00 | start |  | True | 0.65 | 4 | 1.00 | laps |
| f_V1_cw_82 | 1.38 | 9.00 | 10.00 | 10.00 | 13.00 | 13.01 | 0.00 | start |  | True | 0.01 | 12 | 1.00 | laps |
| f_V2_ccw_81 | 2.00 | 4.00 | 5.00 | 5.00 | 5.64 | 5.65 | 0.00 | start |  | True | 0.65 | 4 | 1.00 | laps |
| f_V2_ccw_82 | 1.33 | 9.00 | 10.00 | 10.00 | 10.62 | 10.63 | 0.00 | start |  | True | 0.63 | 9 | 1.00 | laps |
| f_V2_cw_81 | 4.20 | 6.00 | 7.00 | 7.00 | 7.71 | 7.73 | 3.86 | start |  | True | 0.87 | 3 | 1.00 | laps |
| f_V2_cw_82 | 1.32 | 9.00 | 10.00 | 10.00 | 10.61 | 10.63 | 0.00 | start |  | True | 0.63 | 9 | 1.00 | laps |
| f_V3_ccw_81 | 1.33 | 3.00 | 4.00 | 4.00 | 4.70 | 4.71 | 0.00 | start |  | True | 0.71 | 3 | 1.01 | laps |
| f_V3_ccw_82 | 1.35 | 9.00 | 10.00 | 10.00 | 10.63 | 10.65 | 0.00 | start |  | True | 0.65 | 9 | 1.00 | laps |
| f_V3_cw_81 | 1.36 | 4.00 | 5.00 | 5.00 | 5.63 | 5.65 | 0.00 | start |  | True | 0.65 | 4 | 1.00 | laps |
| f_V3_cw_82 | 1.35 | 9.00 | 10.00 | 10.00 | 10.62 | 10.63 | 0.00 | start |  | True | 0.63 | 9 | 1.00 | laps |
| r_V1_ccw_81 | 1.33 | - | - | - | - | - | 0.00 | refuse | source state 'waiting', not 'running' | - | - | 29 | - | start_refused |
| r_V2_ccw_81 | 4.00 | - | - | - | - | - | 0.00 | refuse | source state 'waiting', not 'running' | - | - | 27 | - | start_refused |
| r_V3_ccw_81 | 4.00 | - | - | - | - | - | 0.00 | refuse | source state 'waiting', not 'running' | - | - | 27 | - | start_refused |

Source health over 24 c and f runs: largest status gap 1.01 s (limit 2.0 for the development check), largest status age at the start 0.87 s (limit 1.5).
