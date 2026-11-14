# Pre-registration: Plug-and-Play Racer v0.2 (gates GP1 to GP4, sim battery for lap learning)

Written 2026-11-13, after the GP0 floor session and BEFORE any v0.2 training, refit or evaluation.
It is committed unchanged by the first v0.2 implementer PR (the analysis PR, roadmap P5.1 and
P5.2), together with `evaluation/prereg/pnp-v0.2-split.yaml`. No result may change this file. A
later correction is a new dated section appended at the end that says what changed and why, and
every number produced before the correction stays reported.

Split file SHA-256 at the time of writing:
`bc5185c912ece67a6177581f1b37adaaa9101b34ee1efe9881af7723681aef83`.

## 0. What was already known when this was written

Stated so the reader can judge what the authors had seen.

- GP0 passed on 2026-11-13: PnP v0.1 (tracer `geometric-v1`, bundle `dyn-v0.1-2026-11-12`, MPPI
  512 samples) drove 9 runs, about 170 m, both directions, caps 0.5, 0.7 and 1.0 m/s, no TTC or
  obstacle brake, no wall contact.
- Per-run safety event counts, planner solve times and tick gaps of those 9 runs were read
  (they motivate the planner work). No perception or dynamics model was trained on, fitted to
  or scored against any 2026-11-13 bag before this file was written. The existing bundles saw
  only bags up to 2026-11-12.
- Same layout, classical drivers: mapped raceline tracker 1.5 m/s line, mean 1.31 m/s, about
  5.6 s per lap, 11 TTC events; gap follow race profile about 1.0 to 1.3 m/s caps; PnP at cap
  1.0 about 8.4 s per lap.
- The run boundaries in the split file come from `/drive_raw` and `/pnp/planner/status` timing
  only (when a driver was commanding), plus one wheel-speed timeline to cut the tracker's wall
  crash out. No model output was used to choose them.

## 1. Held-out split of the real bags

The exact time ranges are in `evaluation/prereg/pnp-v0.2-split.yaml`. Summary of the 2026-11-13
bags (bag seconds after the first `/scan`):

| Segment | Bag | Range (s) | Driver | Direction | Cap (m/s) | Role |
|---|---|---|---|---|---|---|
| G1a | 22-00-40 | 109.2 to 540.0 | gap follow | ccw | 0.60 | train |
| G1b | 22-00-40 | 542.0 to 650.0 | gap follow | ccw | 0.60 | held-out |
| G2 | 22-00-40 | 667.2 to 825.4 | gap follow | cw | 0.56 | train |
| G3a | 22-00-40 | 856.3 to 877.6 | gap follow race | cw | 1.51 | train |
| G3b | 22-00-40 | 879.6 to 895.0 | gap follow race | cw | 1.51 | held-out |
| P0 | 22-20-18 | 16.6 to 113.9 | gap follow | cw | 0.54 | held-out |
| R0 | 22-20-18 | 430.1 to 481.4 | PnP | cw | 0.5 | train |
| R1 | 22-20-18 | 501.5 to 552.9 | PnP | ccw | 0.5 | train |
| R2 | 22-20-18 | 626.1 to 645.8 | PnP | ccw | 1.0 | train |
| R3 | 22-20-18 | 2729.7 to 2786.3 | PnP | ccw | 0.5 | held-out |
| R4 | 22-20-18 | 2812.7 to 2852.8 | PnP | cw | 0.5 | held-out |
| R5 | 22-20-18 | 2868.6 to 2895.5 | PnP | cw | 0.7 | held-out |
| R6 | 22-20-18 | 2914.6 to 2940.6 | PnP | ccw | 0.7 | held-out |
| R7a | 22-20-18 | 2967.2 to 2980.6 | PnP | cw | 1.0 | train |
| R7b | 22-20-18 | 2982.6 to 2991.6 | PnP | cw | 1.0 | held-out |
| R8 | 22-20-18 | 3012.4 to 3029.7 | PnP | ccw | 1.0 | held-out |
| M0 | 23-19-51 | 343.0 to 356.2 | PnP (mapping lap) | ccw | 0.5 | train |
| T0 | 23-27-23 | 458.7 to 683.6 | tracker | ccw | 0.5 | train |
| T1 | 23-27-23 | 1158.0 to 1222.3 | tracker | ccw | 1.0 | train |
| T2a | 23-27-23 | 1230.9 to 1244.1 | tracker | ccw | 1.5 | train |
| T2b | 23-27-23 | 1246.1 to 1254.9 | tracker | ccw | 1.5 | held-out |
| T3a | 23-27-23 | 1264.3 to 1310.1 | tracker | ccw | 2.0 line | train |
| T3b | 23-27-23 | 1312.1 to 1347.6 | tracker | ccw | 2.0 line | held-out |

Held-out is 441 s of driving, train is 1107 s (2026-11-13 only).

Rules that produced it:

1. A speed and direction cell with two or more runs gives one whole run to held-out (R3, R4,
   R8, P0). Both 0.7 m/s PnP runs (R5, R6) are held out whole, so 0.7 m/s is a cap the models
   never trained on with this driver: an interpolation test.
2. A cell with a single run is cut in time: the first 55 percent is train, then a 2.0 s guard
   that nobody uses, the rest is held-out (R7, G3, T2, T3). The long gap follow run G1 is cut
   80 to 20 the same way.
3. The tracker's wall crash and the stand-still after it (23-27-23, 1107.5 to 1158.0 s) are
   unused. So is everything between segments (the car being carried, teleop, idle).
4. A sample is used only if every message it was built from lies inside one segment. A
   self-label whose submap reaches across a segment edge is dropped, in train and in held-out.
5. Held-out therefore holds both directions at 0.5, 0.7 and 1.0 m/s under PnP, gap follow in
   both directions, and the fast data in both directions (cw from the gap follow race run, ccw
   from the tracker).

Older bags: for perception the two 2026-11-11 oval bags and the 2026-10 bags are train; the two
2026-11-12 bags (a different layout build: foam oval, lanes 1.3 to 1.5 m, round island ends) are
never trained on and form the cross-layout held-out set. For dynamics the older bags keep the
dyn-v0.1 rule (first 70 percent of each bag's moving data train); their last 30 percent is
reported as "legacy held-out" and decides nothing.

Limits, stated plainly:

- All 2026-11-13 data is ONE layout. The in-layout held-out set shares every corner with the
  train set; it measures generalisation over runs, speeds and directions, not over tracks. The
  cross-layout set and the simulator's unseen track families are the only offline evidence about
  new tracks, and the real answer comes from the first session on a new layout.
- The self-labels are traced by `geometric-v1` on a multi-scan submap. Scoring single-scan
  `geometric-v1` against them favours the baseline (same algorithm, more data). The bias is
  against the net.
- Pose truth for dynamics is chained scan-to-scan ICP at 10 Hz. There is no IMU and no motion
  capture.
- Held-out at 0.7 m/s is about 27 s per direction, and the fast cw held-out part is 15 s. Cells
  with too few samples are merged as section 3 says, and that is reported.

Model selection (architecture, epochs, loss weights, residual size, early stopping, which seed
ships) uses the train role only, with an internal validation part cut from train. The held-out
roles are scored once per candidate that is proposed for shipping. If a bug forces a rerun, both
runs are reported.

## 2. GP1: track perception net against the geometric tracer

Compared on the same scans: the net of the candidate bundle and single-scan `geometric-v1`, both
through `TrackPerceiver.process` (the code the node runs).

Sets: H-run = labelled scans of the `heldout` segments. H-layout = labelled scans of the
`heldout_layout` bags. A-run and A-layout = ALL moving scans (wheel speed at least 0.2 m/s) of the
same segments, labelled or not, for the label-free numbers.

Metrics (definitions as in `racer_pnp_perception.eval_selflabels` and `eval_bags` on main at the
time of writing):

- E_c: centreline error, mean over scans of the mean distance between predicted and labelled
  centreline points over the stations valid in both.
- E_far: the same over stations 8 to 15 only (reported, not gated).
- V: mean `valid_count`.
- C95: share of scans whose driven-path containment is at least 0.95.
- Label-free on A sets: mean `intrusion_fraction`, median `boundary_residual_m`, clearance
  deficit p90.

Uncertainty: paired block bootstrap over 2.0 s blocks of consecutive scans within a segment,
2000 resamples, seed 0, 95 percent percentile interval of the ratio E_c(net) / E_c(geometric).

GP1 PASSES only if all of these hold:

1. H-run holds at least 300 labelled scans. Fewer: GP1 is undecided, the geometric tracer stays.
2. In-layout accuracy: E_c(net) at most 0.90 times E_c(geometric) on H-run, and the upper end
   of the bootstrap interval of the ratio is below 1.00.
3. In-layout range: V(net) at least V(geometric) on H-run.
4. Cross-layout: on H-layout, E_c(net) at most 1.00 times E_c(geometric) and V(net) at least
   V(geometric) minus 0.5. If H-layout holds fewer than 100 labelled scans this item is judged
   on the label-free numbers of A-layout alone (item 5) and that is said.
5. Label-free non-inferiority on A-run and on A-layout, each: mean intrusion at most
   max(0.02, geometric + 0.005); boundary residual median at most 0.10 m; clearance deficit p90
   at most 0.05 m; C95(net) at least C95(geometric) minus 0.02.
6. Per cell (direction by speed bin [0.2, 0.6), [0.6, 0.85), 0.85 and up, by wheel speed) with
   at least 40 labelled scans: E_c(net) at most 1.10 times E_c(geometric). No cell may be much
   worse even when the pooled number wins.

Outcomes, all reported:

- Pass: the net is the candidate backend for the next car session. It still goes on the car in
  shadow first (it publishes, the planner does not read it) and must hold its latency budget
  there (scan to `LocalTrack` p99 under 10 ms on the Orin) before it drives.
- Fails 2 or 3 only: "no better than geometric in the layout". Geometric stays deployed.
- Passes 2 and 3, fails 4, 5 or 6: "better where it trained, not shown elsewhere". Geometric
  stays deployed for unseen layouts. Reported as such, not as a pass.

On-car confirmation (not part of the offline gate, pre-registered so it cannot be redefined
later): on the first layout the net never trained on, with the net driving at the same cap as a
geometric run in the same direction, mean track confidence at least the geometric run's minus
0.03 and no new TTC or obstacle brake.

## 3. GP2: dynamics

Windows: open-loop rollouts from the recorded `/drive` commands, started every 0.25 s inside a
segment where the wheel speed at the start is at least 0.25 m/s, horizons 0.5, 1.0 and 1.5 s.
Speed truth is the wheel speed, pose and yaw truth is chained ICP. Windows with a failed ICP
interval are dropped for every model alike. The unresponsive-window filter of the pipeline on
main stays as it is.

Strata: direction (from the split file) by start-speed bin [0.25, 0.6), [0.6, 0.85),
[0.85, 1.25), 1.25 and up. A stratum with fewer than 30 held-out windows is merged into the next
lower bin of the same direction and listed as merged. The headline numbers are MACRO averages:
the unweighted mean over strata of the per-stratum RMSE, so 100 s of slow gap follow cannot hide
the fast data.

Primary metric: P1 = macro position RMSE at 1.0 s. Secondary: Y1 = macro yaw RMSE at 1.0 s,
S1 = macro wheel speed RMSE at 1.0 s. The 0.5 and 1.5 s values are reported.

Uncertainty: paired block bootstrap over 5.0 s blocks within a segment, 2000 resamples, seed 0,
on the ratio of P1 between two models.

### 3.1 Refit acceptance (which nominal model ships)

`dyn-v0.2` nominal (fit on the `train` and legacy train data) against `dyn-v0.1-2026-11-12`
(never saw 2026-11-13), on the `heldout` segments. The refit ships if P1(v0.2) is at most
P1(v0.1), S1(v0.2) at most 1.05 times S1(v0.1) and Y1(v0.2) at most 1.05 times Y1(v0.1).
Otherwise v0.1 stays the deployed nominal and the refit is reported as not better. Parameters on
a bound or with a flat profile are listed as unidentified, as before.

### 3.2 GP2a: residual on top of the nominal (decided tonight)

Three seeds (0, 1, 2), trained on train only, early stopping on the internal validation part.
The seed that would ship is the one with the lowest VALIDATION P1, chosen before held-out is
read. The residual ships enabled only if all of these hold on the `heldout` segments:

1. P1(residual, shipping seed) at most 0.95 times P1(nominal), and the upper end of the
   bootstrap interval of that ratio is below 1.00.
2. Y1(residual) at most 1.00 times Y1(nominal) and S1(residual) at most 1.02 times S1(nominal).
3. Every seed: P1 at most 1.00 times P1(nominal).
4. Every stratum (after merging): position RMSE at 1.0 s at most 1.05 times the nominal's.
5. Rollouts stay finite and inside the bundle's state bounds on a 10^5 sample random sweep of
   the validity box (a residual that explodes off the data is refused whatever its error).

Fail: `residual.enabled: false` ships and the table is the result.

### 3.3 GP2b: online adaptation (roadmap GP2 proper, not decided tonight)

When an adapted model exists (latent from the last 1 to 2 s, roadmap P2.4) it is scored on the
same `heldout` segments AND on the first later car session's bags (never trained on by anyone),
with the latent run causally from each window's own history. GP2 passes if P1(adapted) is at
most 0.90 times P1(nominal) on both sets with the bootstrap interval's upper end below 1.00, and
no stratum is worse than 1.05 times the nominal. In sim the same rule applies on parameter draws
held out from training (grip, mass, steering gain and offset, lags, battery voltage: 200 draws,
seeds 1000 to 1199). Fail: the nominal ships and the result is reported.

### 3.4 Yaw rate from scan matching (P2.2, a measurement, no gate)

The map pose arrives at about 2 Hz (`/pose` from slam_toolbox or AMCL); between two pose updates
the map to base_link transform is carried by wheel odometry, so it is not an independent
reference at 10 Hz. The reference is therefore the yaw change between consecutive `/pose`
messages, and the scan-matching yaw is integrated over the same interval. Reported on the
segments that have `/pose` (G1a, G1b, T0, T1, T2a, T2b, T3a, T3b), per speed bin:

- bias and RMS error of the interval-mean yaw rate (rad/s);
- 10 Hz noise from closure: the yaw of scan i to i+2 matched directly against the sum of the
  two single steps, as an RMS in rad/s;
- delay at the cross-correlation peak against the reference where the pose rate resolves it,
  otherwise against the yaw rate of the steering model from the gated command, and labelled as
  such (that lag includes the steering lag);
- share of intervals rejected by the quality flag.

The same numbers are reported for the wheel-model yaw rate the car already has, as the baseline.
A signal is called usable for adaptation if, at 0.85 m/s and above, the interval-mean RMS error
is at most 0.10 rad/s, the closure noise at most 0.10 rad/s and the delay at most 0.15 s.
Otherwise the limit in section 5.B of the design stands and is restated.

## 4. Planner hardening (C) and GP3

### 4.1 Before the car (replay and sim)

- Replay of R0 to R8 (bag 22-20-18) through the new planner, open loop: zero-speed share with
  fresh inputs under 2 percent outside RECOVERING, no command between 0 and the dead zone,
  steering change std under 0.004 rad per tick (the 13.9 limits, unchanged).
- Sim, car-like plant, the canary cases plus stadium lane 1.3 m, each 10 laps, both directions,
  at 0.6, 1.0 and the bundle ceiling: 0 wall contacts, 0 `rate_limit` events, 0 `watchdog`
  events between the first non-zero command and the last planner status, at most 1 TTC or
  obstacle brake per 10 laps.
- The same with tick jitter injected into the planner's timer (gaps of 40 to 90 ms at 0.2 per
  second and one 400 ms stall in the first second, the distribution measured on 22-20-18): 0
  wall contacts, 0 `rate_limit` events, at most 1 `watchdog` event per run, and no speed command
  jump after a stall.
- Gate predictor against the real safety gate logic: over at least 10^5 seeded random scan and
  command cases, the planner's predictor flags every case the safety gate brakes (no false
  negative). False positives are reported as a share.

### 4.2 On the car (next session)

Counting rule for every on-car number in this file: `/safety/events` records with
`phase == PHASE_ENGAGE`, stamped between the first non-zero `/drive_raw` command of a run and its
last `/pnp/planner/status` message. The watchdog engagement after the planner has stopped
publishing is the stop mechanism and is not counted; one that lands before the last status is.
Sources are reported separately: `ttc` and obstacle (primary), `watchdog`, `rate_limit`.

Baseline from 2026-11-13 (R0 to R8, about 315 s of PnP driving, counted by that rule from the
event listing): 0 TTC or obstacle, `watchdog` 8 (1.5 per minute), `rate_limit` 13 (2.5 per
minute, one more lands within 0.05 s of a run's last status). 7 of the 9 starts had an event of
one of the two kinds in their first 2.0 s. The analysis code recomputes these; if its count
differs, both are reported and the thresholds below do not move.

C is accepted on the car if, over at least 300 s of PnP driving and at least 8 standing starts at
caps 0.5 and 1.0: `watchdog` at most 0.2 per minute, `rate_limit` at most 0.2 per minute, no
event of either kind in the first 2.0 s of at least 7 of 8 starts, TTC and obstacle brakes still
0 at caps up to 1.0, and no wall contact. Miss: the counts are reported and the next step
(real-time scheduling of the planner process) goes to the owner as a decision.

### 4.3 GP3

One continuous run, one launch, nobody touches the car: 20 consecutive laps on one layout with
no wall contact. Fixed before the attempt: cap 1.0 m/s, backend and bundles as validated earlier
in the session, direction ccw first. Laps are counted by the camera lap counter (`/lap/event`)
and cross-checked by wheel distance (at least 0.9 times 20 lap lengths). A contact is any touch
of a wall seen by either person or on the phone video; the video decides a disagreement. A
recovery (reverse) without contact does not fail the run and is counted. A BLOCKED hold longer
than 5 s fails it. The FIRST attempt at a configuration is the GP3 result for it. Later attempts
are reported as repeats with their own outcome.

## 5. GP4 and the lap learning evaluation (D)

### 5.1 Sim battery (decided before D goes on the car)

D may be tuned on ONE track only: the canary stadium (lane 1.0 m, straight 2.0 m, island radius
0.4 m). The battery below is run once with frozen parameters:

| Case | Track | Directions |
|---|---|---|
| S1 | stadium, lane 1.3 m, straight 2.0 m, island radius 0.4 m | ccw, cw |
| S2 | rounded rectangle, lane 1.0 m, 3.0 by 2.0 m island, radius 0.4 m | ccw, cw |
| S3 | stadium, lane 0.9 m, straight 3.0 m, island radius 0.5 m | ccw, cw |
| S4 | the saved real map `car-data/maps/2026-11-13-pnp-lap1` as the world (`--track map`) | ccw, cw |

Seeds 11, 12 and 13 per case and direction: 24 runs per condition. Car-like plant (dead zone,
steering lag, command delay as the fitted bundle), scan noise on, 12 laps from a standing start,
lap-1 cap 0.5 m/s, ceiling the planner bundle's validity. Lap position inside the system comes
from the source that will run on the car (start-scan gate plus odometry station, design section
14.5). Ground-truth pose is used only to score: lap times and contacts.

Conditions: (a) D off, cap fixed at 0.5; (b) D off, cap fixed at the ceiling; (c) D v0 cap
schedule; (d) D v1 safe set. If S4 cannot be loaded (map conversion fails) it is replaced by
nothing: the battery is then 18 runs per condition and the thresholds below scale (22 of 24
becomes 16 of 18).

Per run: T1 = time of lap 1, T_late = median time of laps 8 to 10, r = T_late / T1.

- D-S0 (lap bookkeeping): over all runs of (a), the system's lap count equals the ground-truth
  lap count and every system lap time is within 0.3 s of ground truth. Fail: D does not go on
  the car with that lap source.
- D-S1 (D v0 may go on the car): 0 wall contacts in 24 runs; at least 22 of 24 runs complete 10
  laps with r at most 0.80.
- D-S2 (D v1 may go on the car, and "D v1 beats D v0 in sim"): 0 wall contacts in 24 runs; at
  least 22 of 24 complete 10 laps; the median over runs of T_late(d) / T_late(c), paired by case,
  direction and seed, at most 0.95; mean TTC or obstacle brakes per lap over laps 8 to 10 at
  most 0.2. If the safety items hold and the 0.95 does not, D v1 is reported as "no better than
  the schedule in sim" and D v0 is what runs on the car.
- Condition (b) is the control for "is this just a slow start": its contacts, completion and
  T_late are reported next to (c) and (d). No threshold.

### 5.2 GP4 on the car

Unseen layout: built for the session, never driven, never mapped, in no training set of any
deployed model. Weights frozen. Only D's on-board memory changes during a run.

A run: 10 consecutive laps from a standing start, one launch, lap-1 cap 0.5 m/s, ceiling
min(dynamics bundle validity, 1.5 m/s). At least 5 runs with D on, direction fixed for the
layout, interleaved in a seeded random order with the other conditions (no condition more than
twice in a row), every run starting with the VESC input voltage (5 s mean at rest) inside a
0.8 V window declared before the first run.

Lap times: the camera lap counter (`/lap/event`) is primary. It is on board but does not use the
planner, the pose or the lap memory. D's own `LapEvent` times are secondary; any lap where the
two differ by more than 0.3 s is listed. One run per condition is also timed from phone video
as the outside check, and the difference is reported.

GP4 PASSES if, over the D-on runs (the D variant chosen by 5.1, declared before the session):

1. Every run completes 10 laps (a DNF is reported with its cause and fails this item).
2. Delta = T1 - T_late is positive in every run, the mean Delta has a 95 percent t interval
   that excludes 0, and the median r is at most 0.80.
3. Wall contacts over the D-on runs are no more than over the D-off runs at the lap-1 cap
   (condition a, same number of runs, interleaved). Both counts are reported with the Wilson
   interval of the per-lap contact rate.

Reported next to it, same protocol, no threshold: D off at the ceiling (condition b), the other
D variant if both passed the sim battery, gap follow race profile, the mapped tracker with its
setup time (mapping lap, raceline, localisation), ablations A geometric against net and B
residual off against on when those parts passed their gates. Completion with Wilson intervals,
interventions per lap by source (counting rule of 4.2), time of lap 1, time to complete 10 laps
including setup.

### 5.3 Headline evaluation (P5)

As design section 9, with these fixed: two unseen layouts; conditions PnP full, gap follow race
profile, mapped tracker, and the ablations above; 10 lap runs from a standing start; at least 5
runs per condition per layout; seeded randomised interleaving with no condition more than twice
in a row; the battery window of 5.2; direction fixed per layout. Primary endpoint: lap time as a
function of lap index, mean and 95 percent t interval per lap index over runs, completed laps
only, next to the completion rate. Rates get Wilson intervals and are not hypothesis tested.

Three named outcomes, all written up: improves lap over lap and beats gap follow; drives but does
not improve; does not complete laps.

## 6. What counts as learned in every report

- A net: learned (supervised on sim geometry and on self-labels from the car's own later scans).
  A geometric: classical.
- B nominal: an identified physical model (system identification, classical). B residual and B
  latent: learned.
- C: classical sampling MPC. Nothing in it is learned.
- D v0: a rule (cap schedule). D v1: memory-based learning from the car's own laps (stored
  states and their measured time to go, no gradient training, no demonstration from anyone).
- No part imitates a driver. Labels are geometry seen later by the same sensor; `/drive` and
  `/drive_raw` are never a training target.

A report never says "the learned system" for a configuration whose learned parts failed their
gates and were switched off. It names the configuration.
