# Lap battery B9, 2026-11: D9-S0 FAIL, D9-S1 FAIL, D9-S2 PASS, D9-B0 INSUFFICIENT

Run once at freeze commit 7098fcda5906f56287188e0be41d1dc676cf48e0 (main after PR 213, 211 and 214),
freeze file `evaluation/prereg/freeze/b9.json` (commit 94fa27a on branch feat/pnp-lap-battery-b9),
3 workers, yaml sha256 013cb2f8bec3ce96d2abf456361c3de93d8f85da48c01900c374287d7473807f, results in
car-data/pnp/battery/v0.5-b9. Thresholds unchanged, nothing re-scored, no run file opened beyond the
battery's own report. D does not drive on the camera source: pre-registration section 6 needs D9-S0,
D9-S1 and D9-S2 all passed. D3-S0 and D7-S0 stay FAIL as registered.

## Verdicts

| gate | verdict | what failed |
|---|---|---|
| D9-S0 (oracle and wiring, condition c, 12 runs) | FAIL | one run, U2 ccw seed 72: all 12 laps doubtful, `ready` on every lap and `alive` on lap 1 |
| D9-S1 (learning through the new path, c against b) | FAIL | item 4a, T_late below T1 in every completing run: the same run U2 ccw seed 72 |
| D9-S2 (fallback in closed loop, condition f, 12 runs) | PASS | all seven items |
| D9-B0 (camera events through D's rule, bags `22-45-39`, `23-04-09`) | INSUFFICIENT | items (i), (ii) PASS over 23 judged laps; no run has a lapref period |

D9-S0 items: the oracle published one event per truth crossing 1 to 12 within 0.3 s in all 12 runs (PASS);
the system closed laps 1 to 12 all confirmed with no event rejected in 11 of 12 runs (FAIL, U2/ccw/72);
every system crossing within 0.3 s of the truth crossing of its number (PASS); no run failed to execute
(PASS).

D9-S1 items: 0 contacts in 12 c runs (ok); 12 of 12 complete 10 laps (ok); reach, T_late(c) at most 1.05
times T_late(b) in 11 of 12 runs, needs 11 (ok; ratios 0.992 to 1.006 in 11 runs, 1.729 in U2/ccw/72);
4a T_late below T1 in every completing run: 1 run not below, U2/ccw/72 (FAIL); 4b median r 0.465 against
0.80 (ok).

D9-S2 items: 0 contacts; 12 truth laps in every run; every confirmed system lap is one truth lap; the
laps closing at truth crossings 5 and 8, the false event's lap and the lap after it are doubtful; the
duplicate of crossing 10 is rejected; the cap after a doubtful lap never above it; the lowest guidance cap
while overdue is the lap-1 cap. Run f_U1_ccw_71 also started not ready (see below) and has doubtful laps
for `ready` as well; the items as registered hold on it.

## The one failure

In two of 24 runs that use the oracle (c U2 ccw 72 and f U1 ccw 71) the lap node's start found the
oracle's status absent or stale: lap 1 is doubtful for `ready` and `alive`, and every later lap for
`ready` (pre-registration D9-P0: a run that starts not ready stays not ready). The fallback did what it
is built to do: the cap stayed at the lap-1 cap, no contact, no lap confirmed that the rule does not
confirm. U2 ccw 72 then has no confirmed lap, so its late laps stay at the lap-1 cap, which is what fails
D9-S1 4a and the reach ratio of that run. D9-S0 asks for 12 confirmed laps in every c run, so it fails.
Not investigated, not re-run, no code or constant changed (stop rule 7a). What is in the report: the
oracle in the sim publishes its status from the runner's odometry loop, so a status that had not arrived
at the planner's first DRIVING report would give exactly `ready` and `alive` on lap 1. That is a hypothesis,
not a finding. A
registered next attempt would fix that wiring and run B9 again under a new freeze.

## Runs

42 run files, 43 attempts: one starved first attempt (a_U3_ccw_71), rerun alone as registered and kept in
`attempts/`. Drop rule: no case dropped (condition a, seed 71, 13 truth laps in all six runs). Reported
with no threshold: T_late(f) / T_late(c) 1.69 to 2.14 (U2 ccw 72 0.98, where c never left the lap-1 cap);
event latency of the oracle at most 0.020 s in c, at most 0.22 s in f (the duplicate); reference length 10.2
to 10.4 m (U1), 11.4 (U2), 7.6 to 7.8 (U3); doubtful laps in the 10 clean f runs 4, 6, 7, 8 (length and
heading) as registered.

## D9-B0

`lap-src --offline` on the two evaluation bags after the freeze file existed, once. Runs of the bags:
`22-45-39` R0 (3.7 s, 0 camera events, 1.14 m), R1 (12 camera events), `23-04-09` R0 (13 camera events).
Judged laps 23 (every camera event after the first of its run). (i) PASS, 0 of 23 over tolerance:
crossing stamps equal the events', lap times within 0.02 s. (ii) PASS, 0 of 23: wheel lengths within 0.10
m of the events' distance. (iii) INSUFFICIENT: lapref found no period in R1 (autocorrelation peak 0.283)
nor in `23-04-09` R0 (0.401), both under the 0.5 minimum. The registered rule makes the gate INSUFFICIENT,
neither pass nor fail; section 6 allows PASS or INSUFFICIENT for D9-B0 and a FAIL stops the path. The
path stops anyway on D9-S0 and D9-S1.

## Order of events

Freeze committed and `check-freeze` run on the host, D9-B0 once, then B9 once in the ros-dev container
(8 CPUs, 3 workers, sim time) after B8 finished. The host slept once mid-run; the starved attempt it may
have caused was rerun by the machinery. `caffeinate` was started after that.
