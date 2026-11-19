# Lap source ready before the planner exists: development acceptance (brief 51c, PR A), 2026-11-18

Pre-registration: `docs/notes/preregistration-pnp-v0.5-laps-camera-2.md`, constants
`evaluation/prereg/pnp-v0.5-laps-camera-2.yaml` (SHA-256
`823f641c8a40337e1a81aaa440d55976585d19984ae2db55f06f0eee1289bdcd`), committed alone as commit 1
(`1426c46`). Everything below is development. No V case and no seed 81 or 82 was run, no U case,
no seed 71 or 72, no run file of `v0.5-b9` was opened, neither evaluation bag was opened. Development
runs are in `car-data/pnp/battery/v0.5-ready-dev/`. B10 was not frozen and not run.

## D1: forced, as built

P1 ccw seed 11, condition c, the oracle's first status held back until 1.0 s after the first DRIVING
report (a test hook of the runner, since removed: with the start gate that stimulus is a refusal).
Head `c1b611d` (instrument only). The reasons are exactly the ones the brief expects, so the
mechanism is the one the brief assumes:

| | |
|---|---|
| `source_ready` | false ("no source status yet"), `source_statuses_before_start` 0 |
| lap 1 | doubtful: `ready`, `alive` |
| laps 2 and 3 | doubtful: `ready` |
| cap | the lap-1 cap, 0.5 m/s, on every lap |
| start trace | lap node first status 1.34 s, planner spawned 1.34 s, DRIVING 2.37 s, oracle first status 3.38 s |

The cw run of the same case: identical reasons (DRIVING 4.13 s, oracle first status 5.14 s).

## D2: natural, 60 start-only runs, as built

`--laps 1`, P1, P2, P4, both directions, seeds 11 to 13, condition c, three passes of 18 and one of
P1 (6): 60 runs, none starved, none with an error.

| | |
|---|---|
| started not ready | 1 of 60 (P4 cw seed 11: oracle first status 3.99 s, lap 1 start 1.87 s on the lap node's clock, "no source status yet") |
| oracle first status minus lap-1 start, s | minimum -7.35, median -0.58, maximum +2.12 |
| runs with the oracle's first status after the lap-1 start | 1 |

So the hole exists in the simulator in about 1 start in 60 even before any hold. Why the B9 run
started not ready stays unproven (this is a natural reproduction of the mechanism, not of that run).

## Outside review (claude-docs/review-codex-2026-11-18.md section 4), fixed on this branch

Each with a red test committed first.

| # | finding | fix |
|---|---|---|
| 8 | the ready count was checked before the deadline | `StartReadyGate`: a status that arrives after `wait_s` is not counted, it ends the wait with a refusal (it still gives the refusal the freshest reason). The wall-time bound of the runner's wait (`99e0059`) is a different thing: it stopped the runner running out of spin polls and does not cover #8 |
| 1 | the lap counter says `running` and a similarity on a timer when no image arrives | `racer_perception`: `LapCounter.freshness(now)` and the status keys `camera` (fresh, stale, no_frames), `camera_fresh`, `frame_seq` (advances only with processed images), `frame_age_s`, `frames_since_reset`, `reset_id`, `odom_age_s`, `odom_fresh`; parameter `frame_stale_s` (1.0 s). The lap node: `source_ready_now` needs a fresh camera, fresh odometry and a frame sequence that advanced since the status before; `source_alive` and the lap's liveness need a fresh camera, so frames that stop during a run make the lap doubtful (`alive`) and the station uncertain. A status without these keys is read as before |
| 2 | post-reset ordering | the counter's reset answer names `reset_id=N`; the launch passes it to the waiter as `min_reset_id`, and a status that carries an older `source_reset_id` (a queued pre-reset status, an earlier life of the counter) is not ready. Order on the car: counter in the graph, reset acknowledged, fresh post-reset frames and odometry (the lap node's ready check), two consecutive ready statuses, planner. The oracle speaks the same keys in sim |
| | lidar gate shadow not started by default | it never was (`lap_shadow` defaults empty); the D10 runbook command no longer names it |

These tighten what "ready" means after the pre-registration was committed (`start_gate` and
`source_rule` constants are untouched, the yaml is unchanged): the registered `ready_keys` are read
from the lap node's `source_ready_now`, and that now includes the camera's own freshness. Written
here so the freeze commit's reader knows; a status without the keys is judged exactly as before, so
the G1 identity holds.

## G1 to G6

Head under test for G2 to G5: `a757afb` (all review fixes in). G3 and G5 were first run on `9a35d6d`
(before findings 1 and 2) and passed there as well; the numbers below are the final head's.

| | check | result |
|---|---|---|
| G1 | identity of the lap computation | PASS. `git diff origin/main` of `racer_laps/src` is additive: the three brief keys and the camera keys in `external_gate.py` (read-only bookkeeping, a status without camera keys is read exactly as before), `start_ready.py`, `lap_source_ready_wait.py`, the engine's optional `camera` argument. Every existing `racer_laps` test and golden passes unchanged (macOS 807, Linux 876 passed, 3 skipped; the new tests included), `tests/l5_pnp_laps`, `tools/tests`, `racer_bringup` too. `lap-src --offline --assume-ready` on bags `22-38-47` and `02-18-54` gives byte-identical JSON (16462 and 13156 bytes) on `origin/main` and on this branch. L2 property: verdicts, reasons, caps and the reference length are identical with the diagnostic keys present and stubbed out |
| G2 | D1 after the fix | PASS (canary `test_d1_...`, P1 ccw seed 11, oracle ready 6 s after the lap node's first status): gate start 10.0 s, planner spawned 10.0 s, first DRIVING 11.46 s, `source_ready` true, no doubtful lap, cap rises from 0.5 |
| G3 | development sim, 34 runs | PASS. D10-S0 (18 c runs, 2 r runs), D10-S1 (6 of 6 on seed 11's pairs: reach ratios 0.990 to 1.013, median r 0.484, 0 contacts), D10-S2 (8 f runs, all seven items). 26 of 26 c and f runs started ready through the gate. Both r runs refused at 31.0 s sim time after the lap node's first status, planner never spawned, 0 `/drive_raw` messages |
| G4 | source health | PASS. Largest `source_status_gap_max_s` 1.01 s (limit 2.0), largest `source_status_age_at_start_s` 0.99 s (limit 1.5), over the 26 c and f runs |
| G5 | 60 start-only runs, delays 0 and 6 s alternating | PASS. 60 of 60 started ready, 60 of 60 through the gate, every first DRIVING report after the gate's start, oracle first status always before lap 1 (minus 13.63 to minus 3.84 s), none refused, none with an error. The same 60 as built (D2) had 1 not ready |
| G6 | launch | PASS (Linux, ros-dev image, real `LaunchService`, real waiter, a fake lap node): ready after 8 s starts the planner after it; never ready refuses at 30 s with the reason in the log and no planner process created; a reset that fails refuses; a lap node that dies after one ready status refuses. `lap_memory:=false` node list and parameters are the pinned golden; `start_scan` or `map_pose` without a camera shadow has no chain and starts the planner as today |

Targeted cases of the review, all red first: delayed or no source (D1, r canary, G6, `ReadyWaiter`),
a late poll (deadline precedence tests, property), camera frames stop (counter L1, counter node L3,
lap node L1, and two real nodes in `test_camera_freshness_ros.py`: one frame then no images with both
processes alive is never ready; frames flowing is ready and frames stopping is not ready and not alive;
frames stopping during a run is not alive and the station uncertain), duplicate and drop (D10-S2, the
fault plan, all 8 f runs), shutdown (the waiter's clean shutdown, the launch's Shutdown on a refusal).

## Tests that changed on purpose

* `test_pnp_launch.py`, two tests that read the planner out of the top-level actions
  (`test_lap_source_camera_starts_the_lap_node_on_the_camera_source`, the shadow lap node test) now
  read it through `_planner_of`, because a camera lap node, primary or shadow, holds the planner
  behind the start chain. Same assertions on its parameters.
* `test_pnp_session_lap_src.py::test_the_command_line_drive_table_and_the_refusals` was red on main
  (B9's freeze file exists); main's own fix is taken in the merge.
* `test_start_trace.py`: the item-1 hook tests were removed with the hook.

## Not done here, for the launcher

B10 is not frozen. The freeze file, the one B10 run and its note are PR B. The lap node's ready rule now
includes the camera's freshness (above): whoever freezes should read that paragraph. Timings of the
development runs: D1 and D2 about 25 min with the container warm, G3 about 40 min, G5 about 15 min,
the Linux `racer_laps` suite 31 to 36 min (single process).
