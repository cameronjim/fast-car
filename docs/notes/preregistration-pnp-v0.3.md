# Pre-registration: Plug-and-Play Racer v0.3 (new attempts after the v0.2 gates)

Written 2026-11-14, after the v0.2 results (PRs 162 to 169) and BEFORE any v0.3 code, training,
refit, battery run or evaluation. No result may change this file. A later correction is a new
dated section appended at the end that says what changed and why, and every number produced
before the correction stays reported.

Constants file: `evaluation/prereg/pnp-v0.3.yaml`, SHA-256
`4415b3719a29cfb08d5f5242ac6cdc5b35fee87e132c6d2939dfa0758808d7a1`.
Real-bag split: `evaluation/prereg/pnp-v0.2-split.yaml`, unchanged, SHA-256
`bc5185c912ece67a6177581f1b37adaaa9101b34ee1efe9881af7723681aef83`.

`docs/notes/preregistration-pnp-v0.2.md` stays in force and is not edited. Every v0.2 gate keeps
its v0.2 result. Nothing below replaces a v0.2 threshold. Where a v0.2 gate failed, the gate
here is a NEW attempt with its own name, its own data where fresh data exists, and a plain
statement of what was already seen.

## 0. What was already known when this was written

The v0.2 results. All of them stay on the record as they are.

| v0.2 gate | Result | Where |
|---|---|---|
| GP1 perception net v1 | "better where it trained, not shown elsewhere". Passed items 1 to 4 and 6. FAILED item 5 on A-layout (intrusion 0.023 vs 0.02, deficit p90 0.067 vs 0.05 m, C95 0.491 vs 0.564 minus 0.02) | `training/pnp_models/trackperc-v1-2026-11-14/README.md` |
| 3.1 dynamics refit | FAILED (S1 ratio 1.073, Y1 ratio 1.055, limit 1.05) | `training/pnp_models/dyn-v0.2-2026-11-14/README.md` |
| GP2a residual on dyn-v0.1 | FAILED (S1 ratio 1.089 vs 1.02; stratum ccw 0.6 to 0.85 ratio 1.156 vs 1.05). P1 ratio 0.922, Y1 ratio 0.403 | `training/pnp_models/dyn-v0.1r-2026-11-14/README.md` |
| 3.4 yaw rate from scan matching | No signal usable (RMS 0.135 to 0.219 rad/s at 0.85 m/s and above, limit 0.10). Identity-seeded ICP under-reads yaw rate (bias -0.162 rad/s); wheel-seeded ICP does not (bias 0.007) | `docs/notes/yaw-rate-scan-matching-2026-11-13.md` |
| D-S0 lap bookkeeping | FAILED. Lap count right in 24 of 24 runs, 11 lap times off by more than 0.3 s | sim battery run 3, PR 169 |
| D-S1 D v0 | FAILED (20 of 24 runs with r at most 0.80, needs 22) | same |
| D-S2 D v1 | PASSED (median T_late(d) / T_late(c) 0.939, 0 contacts), with the deviation below | same |
| 4.2, GP3, GP4 | not attempted yet (car work) | |

What the author of this file had read before writing it, so the reader can judge:

- Every per-run row of battery run 3 (conditions a to d, cases S1 to S4, seeds 11 to 13), the
  system `LapEvent` lengths and times of the failing runs, and the gate source code.
- D-S0 detail. Ten of the 11 laps are in two S3 runs (stadium, 3.0 m straights). The loop is
  11.97 m and the correct laps measure 11.72 to 11.99 m of wheel distance. Lap 1 closed at 9.95 m
  (ccw) and 9.54 m (cw): about 2 m early, on the straight. That length became the gate's
  reference length and was never revised, so later laps closed alternately short and long
  (10.27 m then 13.53 m). The errors cancel in pairs, which is why the count is right. The 11th
  lap (S1, cw, seed 11, lap 1) looks like a scoring artefact of the ground truth, not a gate
  error: the system and ground-truth crossing stamps agree within 0.10 s and the run starts
  with a recovery. That reading is not verified yet; brief 29 must test it.
- D-S1 detail. The four runs over 0.80 are S3 runs. On S3 the control with D off at the ceiling
  (condition b) laps in 21.9 s against 27.3 s at the lap-1 cap: a ratio of 0.80. The lane and the
  corners bound the speed there, not the cap. So "r at most 0.80 in 22 of 24 runs" asked D v0
  for more than the ceiling control delivers on that track. Two of the four were also slowed by
  the lap gate error (unclean "missed" laps lower the cap).
- The deviation in D-S2. Condition d ran with two changes made after the pre-registration and
  during the battery work: a floor of 0.3 m/s on the D v1 speed bound and a merge of caution
  zones created at the same spot on the same lap. The owner has not confirmed it. Section 1
  runs D v1 both ways.
- GP2a per-stratum and per-seed tables, and the P2.2 tables.
- GP1 per-set tables. The held-out sets H-run, A-run, H-layout and A-layout have each been
  scored once by net v1.
- The held-out dynamics segments have been scored by three v0.2 candidates (the v0.2 refit, the
  residual on v0.2, the residual on dyn-v0.1).

Consequence, stated once: every real-bag held-out set of the v0.2 split is now a set that has
been read. A v0.3 pass on such a set makes a model a CANDIDATE. It is confirmed only on data
that did not exist when this file was written (sections 2.4 and 3.4). The simulator can still
give fresh tests, and section 1 uses new tracks and new seeds for that reason.

## 1. Lap learning, second sim battery (B4)

### 1.1 What changed in the system under test

- Lap gate v2 (design doc 13, section 15.2): free-space consistency of a match, a lap length
  estimate that is a running median and can be revised, an odometry innovation gate, and a
  crossing placed by consensus of several matches plus wheel distance.
- D v0 and D v1 are unchanged, except that the two deviation items become parameters
  (`v_bound_floor_mps`, `zone_merge`).

### 1.2 Development data and the freeze

Development data (may be run, read, debugged and tuned on without limit): the tuning stadium T,
a new long-straight development stadium T2 (lane 1.0 m, straight 5.0 m, island radius 0.4 m),
and the whole v0.2 battery S1 to S4 with seeds 11 to 13, which has been read and is no longer a
test. The 2026-11-13 real bags are development data for the gate as well (they were used to
develop gate v1).

Freeze: the commit on main that merges the lap gate v2 PR. Battery B4 is run once on that
commit or a later main commit that changes nothing under `ros_ws/src/racer_laps/`,
`ros_ws/src/racer_planner/` or the runner's lap scoring. The commit SHA is written into the
report. If a bug forces a second run, both runs are reported and the second one says which bug.

### 1.3 The battery

Cases (each ccw and cw, seeds 21, 22 and 23, so 24 runs per condition):

| Case | Track |
|---|---|
| N1 | stadium, lane 1.1 m, straight 4.0 m, island radius 0.45 m |
| N2 | rounded rectangle, lane 0.9 m, straight 4.0 m, island 1.5 m wide, radius 0.4 m |
| N3 | stadium, lane 1.4 m, straight 2.5 m, island radius 0.6 m |
| N4 | the saved real map `car-data/maps/2026-11-10-oval` as the world |

None of these tracks and none of these seeds has been run with the lap memory. N1 and N2 have
longer straights than anything the gate was developed on, on purpose. N4 is dropped if the map
cannot be loaded or if condition a cannot complete 3 laps on it in either direction with seed
21; the battery is then 18 runs per condition and 22 of 24 becomes 16 of 18.

Everything else as v0.2 section 5.1: car-like plant, scan noise on, 12 laps from a standing
start, lap-1 cap 0.5 m/s, ceiling the planner bundle's validity, lap position from the source
that will run on the car (start-scan gate v2 plus odometry station). Ground truth is used only
to score.

Conditions:

| | Condition |
|---|---|
| a | D off, cap fixed at 0.5 |
| b | D off, cap fixed at the ceiling |
| c | D v0 cap schedule |
| d | D v1 with the bound floor 0.3 m/s and the zone merge (as merged in PR 169) |
| e | D v1 with no floor and no merge (as pre-registered in v0.2) |

Ground-truth lap times in B4: lap k is the time between ground-truth crossing k-1 and crossing
k of the start line, where crossing 0 is the stamp of the first `/scan` after the planner first
reported DRIVING and the start line is the ground-truth pose at that stamp. The lap time the
simulator's own lap tracker reports is not used. Per run: T1 is lap 1, T_late the median of
laps 8 to 10, r = T_late / T1.

### 1.4 Gates

**D3-S0 (lap bookkeeping), new attempt of D-S0.**

- S0a, same rule as v0.2 on the new battery: over all runs of condition a, the system lap count
  equals the ground-truth count and every system lap time is within 0.3 s of ground truth.
- S0b, new: in each of conditions c, d and e, the lap count is equal in every run, at least 98
  percent of the laps are within 0.3 s, and no lap is off by more than 1.0 s. v0.2 only checked
  the gate at the lap-1 cap; D changes the speed, and wheel distance per lap changes with it.
- D3-S0 passes if S0a and S0b both hold. Fail: D does not go on the car with this lap source.

**D3-S1 (D v0 may go on the car), a new gate, not a replacement of D-S1.**

Why a new rule: section 0 shows the v0.2 threshold (r at most 0.80 in 22 of 24 runs) could not
be met on a track where the ceiling control itself only reaches 0.80. The fixed ratio ignored
what the track allows. The new rule ties D v0 to the ceiling control on the same track.
The v0.2 rule is still computed on B4 and reported next to it, labelled as the v0.2 rule.

1. 0 wall contacts in the runs of condition c.
2. At least 22 of 24 runs complete 10 laps.
3. Reach: in at least 22 of 24 runs, T_late(c) is at most 1.05 times T_late(b) of the paired
   run (same case, direction, seed).
4. Improvement: T_late is below T1 in every completing run, and the median r over runs is at
   most 0.80.

Disclosure: the numbers of battery run 3 under this rule were visible when it was written
(paired T_late(c) / T_late(b) between 1.00 and 1.07, median r 0.42 to 0.65). The tracks and
seeds of B4 are new.

**D3-S2 (D v1 may go on the car), same thresholds as D-S2, judged for d and for e separately.**

0 wall contacts; at least 22 of 24 complete 10 laps; median over runs of T_late(v1) /
T_late(c), paired, at most 0.95; mean TTC or obstacle brakes per lap over laps 8 to 10 at most
0.2. Reported next to it with no threshold: the paired ratio T_late(d) / T_late(e), contacts
and brakes of each, and how often the floor and the merge were active in d.

Which D v1 goes on the car:

| Owner on the deviation | d | e | On the car |
|---|---|---|---|
| confirms it | pass | any | d |
| confirms it | fail | pass | e |
| rejects it | any | pass | e |
| rejects it | any | fail | D v0 |
| no decision yet | | | D v0 only, both results reported |

In every row D3-S0 and D3-S1 must have passed first. If neither d nor e passes, D v1 is reported
as "no better than the schedule in sim" or as unsafe in sim, whichever items failed, and D v0
is what runs.

Condition b is the control as before: reported, no threshold.

### 1.5 Real bags and the car (the gate itself)

- M-R, regression on the 2026-11-13 bags (development data, so no claim of a fresh test): on
  every PnP, gap follow and tracker segment of the split with at least two laps, gate v2 closes
  at least as many laps as gate v1 at commit f45fd09, and on the segments with a map pose (G1a,
  G1b, T0, T1, T2a plus T2b) every lap time is within 0.3 s of the pose-based detector. A lap
  that v1 closed and v2 does not is acceptable only if the pose detector shows it was a false
  closure. Reported per segment.
- D3-C0, on the car, the first fresh test of the gate on real data: with the lap memory in
  observe mode (it publishes `LapEvent`, no guidance, the planner's cap is fixed) over at least
  two runs of 10 laps, one per direction: the lap count equals the camera lap counter's in both
  runs and at least 18 of the 20 lap times agree with it within 0.3 s. If the owner does not
  accept the camera counter as the reference, the phone video of the same runs is the reference
  and that is said. Fail: D does not drive the cap in that session.

## 2. Dynamics: yaw channel (GP2c), new attempt after 3.1 and GP2a failed

### 2.1 Why this is narrower

The v0.2 residual cut yaw error to 0.40 of the nominal's and made speed error worse (S1 ratio
1.089, which is 3 mm/s of RMSE). The nominal already fits speed. So the new candidates may
change the yaw channel only. Two candidates, the classical one first:

- Y (classical): refit of the four yaw parameters of the nominal (`steer_gain_left`,
  `steer_gain_right`, `steer_offset_rad`, `understeer_s2_per_m2`). Every other parameter is
  frozen at its dyn-v0.1 value.
- R (learned): a residual whose speed output is switched off exactly (`output_scale[0] = 0`),
  trained on top of whichever nominal ships (Y if Y passes, else dyn-v0.1).

For both, the speed channel is identical to the nominal's by construction. S1 is therefore not
evidence of anything and is only checked for equality.

### 2.2 Truth

P2.2 measured that the identity-seeded ICP chain, which was the yaw and pose truth of v0.2
("T1"), under-reads yaw rate by 0.16 rad/s at 0.85 m/s and above, and that the wheel-seeded
matcher does not. A yaw-only model fitted to T1 would learn to under-turn. So:

- T2 (primary in v0.3): chained scan-to-scan ICP started from the wheel motion over the
  interval (wheel speed, gated steering, the vehicle_params yaw gain table), same quality flags
  as v0.2 (RMS at most 0.02 m, inlier share at least 0.7). Training targets and held-out
  scoring both use T2.
- T1 is still computed and every table is also printed under T1, labelled, deciding nothing.
- M-B1, reported before any v0.3 model is trained: dyn-v0.1 nominal and the v0.2 residual
  `dyn-v0.1r` scored under T2. This says how much of the 0.40 was the truth's own bias.
- Independent check, because T2 is seeded by a wheel model: on the held-out segments with a map
  pose (G1b, T2b), for every pair of consecutive `/pose` messages at most 1.5 s apart and
  passing the P2.2 jump filter, the model is rolled open loop over the interval from the
  recorded commands. Yp = RMS of (model yaw change minus pose yaw change), per segment, then
  the mean over the two segments.

Windows, strata, merging, macro averaging, horizons, P1, Y1, S1 and the bootstrap are exactly
those of v0.2 section 3, on the `heldout` segments (403 s of driving; T3b is empty).

### 2.3 Gates

**GP2c-Y: the yaw refit ships as the nominal** if, on held-out under T2, against dyn-v0.1:

1. P1(Y) at most 1.00 times P1(v0.1) and the upper end of the bootstrap interval of the ratio
   below 1.00.
2. Y1(Y) at most 1.00 times Y1(v0.1).
3. Every stratum: position RMSE at 1.0 s at most 1.05 times dyn-v0.1's.
4. Yp(Y) at most 1.00 times Yp(v0.1).
5. S1(Y) equals S1(v0.1) within 1e-6 (the speed channel was not touched).
6. No refit parameter on a bound. A parameter on a bound is listed as unidentified and fails
   this item.

This is stricter than v0.2 section 3.1 (which allowed 1.05 on Y1). Fail: dyn-v0.1 stays the
nominal.

**GP2c-R: the yaw-only residual becomes a candidate** if, on held-out under T2, against the
shipping nominal (the strict reading of v0.2 addendum 2), with seeds 0, 1 and 2 and the
shipping seed chosen by validation P1 before held-out is read:

1. P1(R) at most 0.95 times P1(nominal) and the bootstrap upper end below 1.00.
2. Y1(R) at most 1.00 times Y1(nominal). S1(R) equals S1(nominal) within 1e-6.
3. Every seed: P1 at most 1.00 times P1(nominal).
4. Every stratum: position RMSE at 1.0 s at most 1.05 times the nominal's.
5. Rollouts finite and inside the state bounds on a 10^5 sample sweep of the validity box.
6. Yp(R) at most 1.00 times Yp(nominal).

Items 1 to 5 are the GP2a thresholds, unchanged. Item 6 is added. Fail: the residual stays
off and the table is the result.

### 2.4 Confirmation on fresh data (before either drives the car)

The held-out segments have been read before (section 0). A pass above is therefore a
candidate. At the next car session the first PnP runs are driven with the deployed model
(dyn-v0.1, residual off). On at least 120 s of that driving, both directions, scored with the
same code under T2: the candidate must have P1 at most 1.00 times and Y1 at most 1.00 times
the deployed model's. Only then is it switched on, and the session's event counts decide
whether it stays (design doc 13, section 14.8 steps 2 and 3, unchanged). If the confirmation
fails, the offline pass is reported as not confirmed.

## 3. Perception: scan-grounded net (GP1b), new attempt after GP1 item 5 failed

### 3.1 The candidate

One candidate, fixed before any fresh scoring: a net with the v1 architecture whose output is
refined against the scan at run time (stations the scan supports on both sides are recentred
by the Euclidean rule of `geometric-v1`; a station never claims more half width than the scan
shows; stations the scan does not support keep the net's value), trained with an isotropic
scale augmentation of the real samples in [0.75, 1.4] in addition to the v1 recipe. The
refinement is part of the bundle contract (`io.arch.type`), so a bundle says whether it runs.

Development data for choosing the recipe and its settings: sim, the `train` roles, and the
2026-11-12 layout bags (A-layout and H-layout), which were scored once in v0.2 and are no
longer a test.

### 3.2 Fresh layouts: leave one layout out

Two folds. In fold F1 the two 2026-11-11 oval bags are removed from training and are the test;
in fold F2 the two 2026-10 bags are. The recipe is frozen (git SHA recorded) before either fold
is trained. Each fold trains the same recipe, same seed rule, without its held-out bags, and is
scored once. These bags were training data in v0.2, so they were never used to judge a model,
and no setting may be chosen on them. In bags with no `/odom/wheel`, a scan counts as moving
when the ICP ground speed is at least 0.2 m/s.

### 3.3 Gate GP1b

The full model (all `train` roles) is scored on H-run and A-run; fold models on their own
held-out bags. All thresholds are those of v0.2 section 2.

1. Items 1, 2, 3 and 6 of v0.2 section 2 hold for the full model on H-run (second read of
   H-run, stated).
2. Item 5 holds for the full model on A-run.
3. In each fold with at least 100 labelled scans: E_c(net) at most 1.00 times E_c(geometric)
   and V(net) at least V(geometric) minus 0.5 (v0.2 item 4).
4. In each fold, on all moving scans: mean intrusion at most max(0.02, geometric + 0.005);
   boundary residual median at most 0.10 m; clearance deficit p90 at most 0.05 m; C95(net) at
   least C95(geometric) minus 0.02 (v0.2 item 5).
5. The same label-free rule on the 2026-11-12 layout for the full model. It must hold, and it
   is reported as development data, not as evidence.

Reported with no threshold: every number for the unrefined net next to the refined one, so
the report says what the refinement did and what the retraining did.

Outcomes: pass means the net is the candidate backend; it still goes on the car in shadow
first, and the on-car confirmation of v0.2 section 2 (first layout it never trained on, track
confidence, no new brake) is unchanged and is the real test. Fails 1 or 2: "no better than
geometric in the layout". Passes 1 and 2, fails 3, 4 or 5: "better where it trained, not shown
elsewhere", again. Geometric stays deployed in both.

## 4. Scan deskew (M-DSK), a measurement with an acceptance rule

Deskew moves every beam of a sweep to the car's pose at one reference time, using wheel speed
and the wheel-model yaw rate. Two things are not known and are chosen on the `train` role only:
whether the driver's header stamp is the start or the end of the sweep, and the sweep's
rotation sense. The choice is the one that gives the lowest metric on the selection pairs.

Metric: a fast run and a slow run over the same loop in the same direction. For every scan of
the fast run taken in a fast corner (wheel speed at least 0.6 m/s and wheel-model yaw rate at
least 0.5 rad/s in magnitude), find the scan of the slow run that matches it best by ICP and
keep that match's RMS. C is the median of those RMS values. Distortion grows with speed, so a
fast scan and a slow scan of the same place disagree when the sweep is not compensated. Both
runs get the same processing (both raw, or both deskewed).

Pairs (yaml): selection R2 against R1 (ccw) and R7a against R0 (cw); result R8 against R3 (ccw)
and R7b against R4 (cw), scored once.

Deskew may be switched on at the car (as its own step, two runs per direction, compared with
the step before) if on the result pairs:

1. C(deskewed) at most 0.90 times C(raw), pooled over the two pairs, with the upper end of the
   paired block bootstrap interval of the ratio below 1.00.
2. On the fast run's scans that are not fast corner scans: the same median at most 1.02 times
   raw.
3. `geometric-v1` on A-run with deskewed scans: mean intrusion at most raw plus 0.002 and
   clearance deficit p90 at most raw plus 0.005 m.
4. Replay of R0 to R8 through the planner with deskewed scans: no NO_SCAN and no FAULT tick
   that the raw replay does not have.
5. Node latency p99 under 3 ms on the dev machine (under 5 ms on the Orin, measured at the car
   before it drives).

Fail on 1: reported as "no measurable gain", deskew stays off. Fail on 2 to 5: off, with the
reason.

## 5. Timing report for the open safety-timing questions (M-T), no gate

The owner has open decisions on real-time scheduling and on safety_node timing. v0.3 changes
neither. It fixes what will be measured at the next session so the numbers cannot be chosen
after the fact. From each bag, inside the counting window of v0.2 section 4.2 (first non-zero
`/drive_raw` command of a run to its last planner status):

- `/drive_raw` inter-arrival gaps (recorder receive time) and header stamp gaps: p50, p99, max,
  and the count above each of 40, 60, 80, 100 and 150 ms. The count above 60 ms is what the
  current watchdog sees; the other counts say what a different limit would have seen.
- Planner solve time and tick gap percentiles, plan age at publish, bunched ticks.
- `/safety/events` ENGAGE counts by source, per minute and per start (first 2.0 s), and for
  `rate_limit` events the limiter's own dt parsed from the event detail.
- Rate limiter what-if: the recorded `/drive_raw` replayed through the unchanged safety gate
  code offline, once with the recorded dt per cycle and once with the nominal period, counting
  clips in each.
- CPU load and the scheduling policy of the planner process as found (read, not set).

The v0.2 acceptance for C on the car (section 4.2) is unchanged and is judged from the same
report. M-T decides nothing by itself; it is the evidence for the owner's decisions.

## 6. Unchanged, carried over

- v0.2 section 4.2 (C on the car), 4.3 (GP3), 5.2 (GP4), 5.3 (headline evaluation) and 6 (what
  counts as learned) apply as written. For GP4 "the D variant chosen by 5.1" now reads "the D
  variant chosen by section 1.4 of this file"; the thresholds do not move.
- GP2b (online adaptation) is not attempted in v0.3: no yaw signal passed 3.4.
- What counts as learned, added: the yaw refit Y is system identification (classical). The
  yaw-only residual R is learned. The scan refinement of the net is classical geometry applied
  to a learned output; a refined net is reported as "net plus scan refinement", never as the
  net alone. Lap gate v2 and deskew are classical.

## 7. Open owner decisions this file does not take

1. Real-time scheduling of the planner process on the Orin.
2. Any change to safety_node timing (rate limiter dt, the 60 ms watchdog).
3. rf2o or any other third-party scan-matching package on the car (rf2o is GPL-3.0).
4. Whether the camera lap counter is the primary lap timing (v0.2 5.2 assumes it; D3-C0 names
   the phone video as the fallback reference).
5. Whether the D v1 zone floor and zone merge are accepted (section 1.4 runs both).
6. The multi-algorithm benchmark. Not designed here.
