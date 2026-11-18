# Main's red L5 canaries, 2026-11-18

Main CI runs 44199589402 (on 6483291) and 44202710978 (on 16e491d) failed three L5 jobs that were
green on PR 209's own CI. Nothing in racer_safety, racer_drivers, firmware or vehicle_params is
touched here: the fixes are in the test harness and the CI script.

## 1. L5 Plug-and-Play canary, contract 2 planner bundle, cw: `rate_limit_events` 1 != 0

Not the startup fault of PRs 208 and 209 (those are fixed: no `rate_limit` at t = 0.3 s in any of
the 26 traces of run 44199589402). The `l5-pnp-canary-trace` artefact of that run shows:

    /drive stamps  ... 3.65, 3.67, 3.69, 3.69, 4.03, 4.05 ...        a 0.34 s gap
    /drive_raw     3.71, 3.73, ... 4.01, 4.03 (20 ms cadence, no gap)
    safety events  watchdog engage "age=0.341653s" (stamp 3.69), release 4.03,
                   rate_limit engage dt=0.013541 s at 4.03

safety_node was not scheduled for 0.34 s while the planner kept publishing every 20 ms. It measures
its /drive_raw watchdog on the steady clock and ticks on a wall timer, and the simulator's clock
never runs faster than the wall, so the first cycle after the stall reads a 0.34 s old command: the
watchdog brakes (one brake, inside the allowed 3) and the ramp back to speed from 0 is clipped by the
rate limit (the one `rate_limit` engagement). Both are the gate working as designed on a late
cycle. The other 25 traces have a longest /drive gap of 0.01 to 0.03 s. Under a 2 CPU limit with a
70 to 100 per cent stress load the same watchdog plus rate_limit pairs appear in bulk (local runs).

Fix (harness): `pnp_timing_summary` now reports `safety_drive` (the gaps of /drive). A run that
fails its judgement and shows a gap of four sim steps (40 ms) or more in /drive_raw or /drive was starved by the machine
and is run again, at most 3 attempts, with the same judgement every time
(`tests/l5_reactive_bench/starvation.py`, the rule the planner timing cases already had for
/drive_raw alone, now shared). A failing run without such a gap fails at once, a defect that fails
every attempt still fails the test, and nothing is relaxed.

## 2. L5 lap learning canary, lap 1: system 21.24 s, truth 20.84 s

The printed lines of the failing job (the only data there is: that job kept no artefact):

    truth   [20.84, 17.73, 15.58, 14.23, 13.50, 13.35]
    system  [21.24, 18.05, 15.73, 14.33, 13.56, 13.34]   differences .40 .31 .15 .10 .06 -.01

and of a passing local run: truth [21.26, 18.01 ...], system [21.31, 17.98 ...], lap 1 difference
0.03 to 0.055 in four clean 4 CPU runs. The system's laps are the normal ones; the truth's are
short, by an amount that shrinks as the lap speed settles, which points at the truth's start line
(its first odometry sample) and not at the lap node. Which sample the truth starts from is the first
odometry sample at or after the system's start scan, recorded by the runner's own spin loop. That is
a guess, not a measurement: the root cause of this one is NOT proven. Under a 2 CPU limit and a
stress load the truth validity checks of the runner fire instead (`truth_starts_after_crossing_0`,
`no_scan_at_start`, and watchdog brakes), which are the starved-machine failures.

Fix (harness, what can be done without the data): the two lap time cases run through the same
starvation rule (also a truth recording the runner itself calls invalid), the lap time assertion
now prints the per lap differences, both crossing lists, the truth's start, the lap lengths, the
/drive_raw and /drive gaps and the brake count, and the lap job uploads its event traces
(`l5-pnp-laps-canary-trace`) like the Plug-and-Play job does. The next occurrence will say what it
was. If it recurs without a gap, it is a measurement problem in the truth start and wants its own
brief.

## 3. L5 localisation canary, mapping: `wall_err_mean_m` 0.1258 > 0.075

A bad map: coverage 0.78, 2266 occupied cells against about 1870 in a good one, slam pose mean error
0.256 m against 0.10. The trajectory is the same as in a good run (lap times [17.51, 34.76], 684
samples in both), so the sim did the same thing and slam_toolbox built a different map from it. It
is the "wrong loop closure on the plain lane" of docs/notes/sim-time-lockstep-2026-11-14.md.

Measured locally (ros-dev, arm64, 2 CPUs, stress-ng 70 per cent): 2 bad maps in 8 runs at the real
time factor 1, 0 in 14 more, then 1 in 8 at the real time factor 0.5, so pacing the sim slower does
not cure it. The lockstep gate timeouts of the bad CI run (20) are the same as in good runs (17 to
24), the number of /pose updates slam publishes varies by a few between good runs (170 to 178), so
no signal in the run separates a bad map from a good one, and a starved-run rule cannot be written.

Fix (CI script): the mapping canary is run on its own with ctest `--repeat until-pass:3`
(`.github/scripts/l5_localization_test.sh`); the AMCL canaries are judged once as before. This is a
bounded rerun, not a looser threshold: a stack that builds a wrong map every time fails all three
attempts. It is the one judgement call in this change; it matches the mapping canary's own header
("a rerun is the right answer"). The cause (slam_toolbox's single-slot scan queue dropping scans while
the node is busy, or the order the EKF's odometry and the scans pair up) is still open under issue
156. The canary now records `slam_pose_updates` so a future change can look for a signal.

## 4. L5 lap learning canary again, PR 212 run 44270021089 (traces in hand)

Job 134683997950, two failures, both on the first attempt (`attempt 1` in the log: the starvation
rule did not fire, `drive_raw` and `safety_drive` gaps were 0.02 s, no brakes). The
`l5-pnp-laps-canary-trace` artefact has one trace per test (drive_raw, drive, planner status,
safety events). Nothing below touches racer_laps or the planner: it is a reading of the harness.

### 4a. Lap 1: system 21.34 s, truth 20.98 s (0.36 s over the 0.3 s tolerance)

Differences per lap [0.36, 0.16, 0.12, 0.05, -0.05, 0.08]; the truth crossings are later than the
system's by 1.09, 0.93, 0.81, 0.76, 0.81, 0.73 s. The system's lap 1 started at
23.78 - 21.34 = 2.44 s (its start scan). The truth's crossing 0 is 3.89 s. Lap 1 error is the start
gap (1.45 s) minus the first crossing gap (1.09 s); later laps only see the crossing gap change
(1.09 to 0.93 is 0.16, 0.93 to 0.81 is 0.12, and so on: each printed difference reproduces
exactly). So the system and the truth count lap 1 from two different points on the track, about
0.3 m apart, and the time cost of that gap is large at the launch (slow) and small at the end of
the lap. That is also why the error shrinks as the cap rises. A passing run has no gap, so the
two cancel (the 0.03 to 0.055 s of the passing local runs).

Why the truth starts late (measured in the trace, not guessed): the runner took the planner's first
DRIVING report at 3.82 s, but the car was already moving.

- `/drive` (safety_node output) leaves zero at 1.86 s and ramps to 0.33 m/s by 3.8 s (0.35 m of
  commanded travel between 1.86 and 3.82 s). The planner was therefore publishing from about 1.8 s.
- The runner's `/drive_raw` and planner status recordings both begin at exactly 3.82 s, the first
  status it saw is already DRIVING (`planner_states {'1': 4821}`, no STARTING record at all), and
  the `/safety/events` ENGAGE at t = 0 is missing (only the release at 1.72 s is recorded). In the
  other five traces of the same job the first `/drive_raw` is at or before the first `/drive`
  motion (lag -0.1 to -4.5 s) and the t = 0 ENGAGE is present. Here it is 1.96 s after.
- The lap node, on the same topics, started its lap at 2.44 s.

Root cause hypothesis: the runner's subscriptions to `/drive_raw` and the planner status matched
the planner's publishers about 2 s late (DDS discovery in a loaded container, the planner being a
phase two node), so its first DRIVING receipt, the registered crossing 0 rule ("first scan stamped
at or after the receipt of the first DRIVING report"), fell 1.4 s after the lap node's start. The
car had moved 0.3 m by then, so the truth line is 0.3 m ahead of the system's. This is the same
family as the two cases documented in `first_scan_at_or_after` (the spin loop lag of
a_S1_ccw_13, the late first DRIVING of a_T2_ccw_12), reached by a third route. Not proven: the
trace has no discovery timestamps. What would prove it: the runner logging the time it first sees
a publisher on `/drive_raw` and the planner status topic (`get_publishers_info_by_topic`), which is cheap.

The existing starvation rule cannot see it (no gap in either topic after the first message), and
`truth_first_odom_s < driving_first_s` holds, so `assert_truth_valid` passes.

Suggested brief (needs the owner's decision, it touches the registered truth rule so it is not
done here): add a validity condition to `registered_truth` that the car is at rest at crossing 0,
from the recorded ground-truth odometry the runner already has (displacement between the first
odometry sample and the sample at crossing 0 under a few centimetres, or `|v|` under the
standstill threshold at crossing 0). That reads nothing from the system, matches the rule's own
statement that an invalid run is rerun, never scored, and the run goes through the existing
`starved` retry. The alternative, anchoring crossing 0 to the first `/drive` motion instead of the
receipt, changes the registered definition and would need a pre-registration amendment.

### 4b. Oracle test: `no_scan_at_start` at line 150

A different failure. In the trace the planner status goes NO_ODOM (2.03 s), NO_SCAN (2.04 s),
STARTING (2.12 s) and stays in STARTING until the run ends at 15.55 s (677 statuses, never DRIVING,
so `driving_first_s` is None and the runner reports `no_scan_at_start`). `/drive_raw` is steady
(longest gap 0.03 s), speed 0 (noise under 0.09 m/s), steering 0; the car never moved and the run
stopped on the stall rule. This is the planner's warm start gate not passing for 13 s (conditions
after scan and odom: track, 25 publish ticks inside `start_tick_gap_s`, 25 solves, gated
`/drive`), not a late subscription. Which condition is unknown: the planner logs the missing one
after `start_timeout_s`, and that log is in the runner's stdout/stderr, which the test throws away
unless the result file is missing. This case is also not retried: `run_truth` goes through
`run_runner` with `check_validity=True`, outside `run_judged`.

Open, needs data: the runner's captured stdout/stderr (the planner's `start_missing` line) kept
next to the trace on failure. Do not add a retry before that: a planner that never leaves
STARTING is a product symptom until shown to be starvation. A second occurrence with that log
decides it.

Both failed in the same job, 4 minutes apart, which fits a loaded runner; 4a does not need a load
to explain it, only a late match.
