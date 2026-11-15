# Pre-registration: Plug-and-Play Racer v0.4 (new attempts after the v0.3 gates and the 2026-11-14 car session)

Written 2026-11-14, after the v0.3 results (PRs 176 to 184) and the car session of the same day,
and BEFORE any v0.4 code, any fit or score on the 2026-11-14 bags, and before the result of
battery B4 was known. No result may change this file. A later correction is a new dated section
appended at the end that says what changed and why, and every number produced before the
correction stays reported.

Constants file: `evaluation/prereg/pnp-v0.4.yaml`, SHA-256
`78510c1423278f80c42511e6caab095818598af6db6a284dd5479f74a03a1d63`. It holds the split of the
2026-11-14 bags.
Earlier files, unchanged and in force: `evaluation/prereg/pnp-v0.2-split.yaml` (SHA-256
`bc5185c912ece67a6177581f1b37adaaa9101b34ee1efe9881af7723681aef83`),
`evaluation/prereg/pnp-v0.3.yaml` (SHA-256
`4415b3719a29cfb08d5f5242ac6cdc5b35fee87e132c6d2939dfa0758808d7a1`),
`docs/notes/preregistration-pnp-v0.2.md`, `docs/notes/preregistration-pnp-v0.3.md`.

Every earlier gate keeps its result. Nothing below replaces or loosens an earlier threshold.
Where an earlier gate failed, the gate here is a NEW attempt with its own name and fresh data.
Battery B4 and its gates D3-S0, D3-S1, D3-S2 and D3-C0 stay exactly as v0.3 section 1 defines
them; this file adds nothing to them and takes nothing away.

## 0. What was already known when this was written

### 0.1 v0.3 results (all stay on the record)

| v0.3 gate or measurement | Result | Where |
|---|---|---|
| GP1b perception net v2 with scan refinement | FAILED. E_c ratio 0.472 on H-run, intrusion about 0. Failed on range and coverage: V 15.31 against 16.00 (H-run), fold F1 V 14.39 against 14.78 and C95 0.603 against 0.616, 2026-11-12 layout C95 0.480 against 0.544 | PR 177, `training/pnp_models/trackperc-v2-2026-11-14/` |
| M-B1 truth re-read | Under the wheel-seeded truth T2 the v0.2 residual's yaw error is 1.38 times the nominal's, not 0.40. The 0.40 was the bias of the v0.2 truth | PR 180 |
| GP2c-Y yaw refit | FAILED (P1 ratio 1.019, Y1 ratio 1.224, Yp ratio 1.053) | PR 180 |
| GP2c-R yaw-only residual | FAILED on item 1 only (P1 ratio 0.972 against 0.95, bootstrap upper end 1.019). Y1 ratio 0.611, Yp ratio 0.694, speed channel identical | PR 180 |
| M-DSK scan deskew | PASSED all five items, default off. Item 2 was judged on 2 scans. The stamp convention was chosen by a margin of 0.4 percent | PR 176 |
| M-R lap gate v2 on the 2026-11-13 bags | FAILED as written: lap count on R6 (the segment ends 0.33 s after the crossing), lap times on G1a and T0 against the pose detector. Gate v1 has the same differences | PR 178 |
| Lap gate v2 development battery (S1 to S4, seeds 11 to 13, conditions a and c) | 48 of 48 runs right, worst lap error 0.17 s. Development data, not a test. `strong_max_violation` was tuned from 0.04 to 0.02 on development case T2 | PR 178 |
| M-T baseline count | reproduced (0 / 8 / 13) | PR 179 |
| M-T limiter replay | FAILED "must reproduce": 11 of 13 `rate_limit` engagements matched | PR 179 |
| B4 (D3-S0, D3-S1, D3-S2) | running when this was written. Result not known | PR 185 |

### 0.2 The car session of 2026-11-14 (driven on main, geometric-v1, dyn-v0.1, lap memory off)

- GP3 PASSED on the first attempt: 20 laps ccw at cap 1.0 in 115 s, lap times 5.50 to 5.80 s,
  0 TTC, 0 FAULT, no wall contact (owner observed, not filmed). The camera lap counter counted
  10 of the 20 laps, so the lap count of GP3 rests on the owner's count and on wheel distance,
  and that is said wherever GP3 is reported.
- Speed sweep, both directions, mean lap time: cap 0.5: 13.85 s ccw, 13.87 s cw; 0.7: 8.10 and
  8.99; 1.0: 5.63 and 6.11; 1.3: 4.57 and 5.00. Lap times were recovered offline by matching
  the steering signal of `/drive`, not by a validated tool.
- Safety timing: 2.6 to 4.5 ENGAGE events per minute, mostly `rate_limit`, against the 0.2 per
  minute of v0.2 section 4.2. C on the car (4.2) is therefore NOT accepted; the miss is
  reported and the next step is the owner's decision, as 4.2 says.
- A cap of 1.5 was refused by `dyn-v0.1` (validity 1.3 m/s), as designed.
- Shadow perception (trackperc-v2 with refinement) was recorded on `/pnp/shadow/*`.

### 0.3 What the author had read from the 2026-11-14 bags before writing

The topic lists and message counts; for each PnP run its start and end, the 99th percentile of
the commanded speed, the sign of the mean steering command, the wheel distance and the largest
wheel speed. Plus the session summary above. No model, no lap gate and no perception code was
run on these bags by the author. The lap times in 0.2 cover every run, including the runs
given the lap role `test` below; that is a limit of M-L1 and is repeated there.

Whether the donut of 2026-11-14 is the same build as the loop of 2026-11-13 is not recorded.
Both give 5.2 to 6.1 m of wheel distance per lap. This file therefore treats 2026-11-14 as the
SAME layout family as 2026-11-13: a fresh session, not an unseen layout. No claim about unseen
layouts is made from these bags.

### 0.4 Consequence

The held-out sets of the v0.2 split have been read by five dynamics candidates and two nets.
The folds F1 and F2 and the 2026-11-12 layout have been read by net v2. From now on they are
development data. The 2026-11-14 bags are the only real data that has judged nothing. Section 1
gives every part of them one role before anyone fits on them.

## 1. Split of the 2026-11-14 bags

The table is `split_2026_11_14` in the yaml. The rule is the v0.2 rule: a speed and direction
cell with two or more runs gives whole runs to held-out; a cell with one run is cut in time,
the first 55 percent train, 2.0 s unused, the rest held-out.

| Segment | Bag | Cap, direction | Dynamics | Perception | Lap (whole run) |
|---|---|---|---|---|---|
| K1 | 21-37-23 | 1.0 ccw | heldout | heldout | test |
| K2 | 21-37-23 | 1.0 cw | heldout | heldout | test |
| L0 (the GP3 run) | 21-49-21 | 1.0 ccw | train | reserve | dev |
| X0 | 22-07-50 | 1.0 ccw | train | reserve | dev |
| X1 | 22-07-50 | 1.0 cw | train | reserve | dev |
| X2a, X2b | 22-07-50 | 0.5 cw | train, heldout | reserve, heldout | test |
| X3a, X3b | 22-07-50 | 0.5 ccw | train, heldout | reserve, heldout | dev |
| X4a, X4b | 22-07-50 | 0.7 ccw | train, heldout | reserve, heldout | test |
| X5a, X5b | 22-07-50 | 0.7 cw | train, heldout | reserve, heldout | dev |
| X6a, X6b | 22-07-50 | 1.3 cw | train, heldout | reserve, heldout | dev |
| X7a, X7b | 22-07-50 | 1.3 ccw | train, heldout | reserve, heldout | test |

Held-out is about 355 s of driving, train about 552 s. The 1.3 m/s held-out data is 14.4 s cw
and 28.3 s ccw: thin, and every table says so.

- Dynamics `heldout` decides GP2d. It is scored once, in one pass, after every v0.4 dynamics
  candidate is frozen.
- Perception `heldout` decides GP1c. Perception `reserve` is not read by any perception code in
  v0.4: not trained on, not scored, not used for a setting. It stays fresh for the next round.
- The lap role is per whole run, because a lap tool reads a run from start to end. A lap tool
  reads steering, wheel odometry, scans and camera frames; it fits no model, so a `dev` lap run
  may contain a dynamics or perception held-out segment without harm to those gates.
- Everything outside the segments is unused.

Bags recorded after 2026-11-14 get their roles by rule, not by hand (yaml `future_sessions`):
first the frozen candidates and the deployed models are scored on the session's PnP runs (the
confirmation of sections 2.4 and 3.4); then a tool writes the split of that session by the cut
rule above and the file is committed with its SHA-256; only then may anything be fitted on the
train part. A fit that reads a bag with no committed split is a bug.

## 2. Dynamics (GP2d), new attempt after GP2c failed

### 2.1 What is different

- Fresh held-out data at four speeds and both directions, and train data up to 1.36 m/s of
  wheel speed in both directions. Before today the clockwise train data above 1.25 m/s was 1.7 s.
- The classical baseline is refitted on the same new data and with the objective the gate
  scores, so the learned candidate has to beat a baseline that had the same chance. In v0.3 the
  yaw refit was fitted on one-step yaw intervals driven by ICP ground speed and then scored on
  open-loop rollouts from wheel speed.
- A budget measurement first, so the gate cannot be failed for a reason known in advance.

Truth, windows, strata, merging, macro averaging, horizons, P1, Y1, S1 and the bootstrap are
those of v0.3 section 2.2 (T2, the wheel-seeded chain) and v0.2 section 3, on the dynamics
`heldout` segments of section 1. The v0.2 held-out segments are scored too and printed,
labelled "read before", deciding nothing.

### 2.2 M-B3, the error budget (a measurement, committed before any v0.4 fit)

On the train data only, for `dyn-v0.1`: the position error at 1.0 s split into the component
along the true heading at the window start and the component across it; and two oracle
rollouts, one with the true yaw at every step in place of the model's (perfect yaw) and one
with the true wheel speed in place of the model's (perfect speed). Reported: P1 of each oracle
over P1 of `dyn-v0.1`, macro and per stratum.

If the perfect yaw ratio on train is above 0.95, a model that changes only the yaw channel
cannot be expected to meet item 1 of GP2d-R. That is then written into the report BEFORE the
held-out pass as "item 1 is out of reach for a yaw-only model on this data". The candidate is
still trained and scored and the gate is still applied as written.

### 2.3 M-B2, direction asymmetry (a measurement, train only, before any fit)

Clockwise laps were 8 to 10 percent slower than counter-clockwise above 0.5 m/s. Per direction
and speed bin on the train segments of 2026-11-14: mean wheel speed, share of time with the
steering command at the limit, mean yaw rate of the truth against `dyn-v0.1`'s, and the lap
radius from wheel distance per lap. No threshold. It says whether the model's yaw error has a
side.

### 2.4 Candidates and gates

Two candidates, frozen before the held-out pass.

- N4 (classical): refit of the nominal's parameters on the train data, the yaw parameters
  fitted on open-loop rollout windows built like the gate's windows (wheel speed as the speed
  state), the speed parameters by the pipeline's speed fit. No new parameter, no schema change.
- R4 (learned): the yaw-only residual of v0.3 (`output_scale[0] = 0`), trained on the same
  train data on top of the nominal that ships (N4 if GP2d-N passes, else `dyn-v0.1`), seeds 0,
  1 and 2, config and shipping seed chosen by validation P1 before held-out is read. The
  steering state is an input of the residual, so it can learn a difference between left and
  right without a new field.

**GP2d-N: N4 ships as the nominal** if on held-out under T2, against `dyn-v0.1`:

1. P1(N4) at most 1.00 times P1(v0.1) and the upper end of the bootstrap interval of the
   ratio below 1.00.
2. Y1(N4) at most 1.00 times Y1(v0.1) and S1(N4) at most 1.00 times S1(v0.1).
3. Every stratum: position RMSE at 1.0 s at most 1.05 times `dyn-v0.1`'s.
4. No refit parameter on a bound.
5. Yp(N4) at most 1.00 times Yp(v0.1) on G1b and T2b (the pose check of v0.3; a re-read, the
   only pose data there is).

Not looser than v0.2 section 3.1 or GP2c-Y in any item. Fail: `dyn-v0.1` stays.

**GP2d-R: the yaw-only residual becomes a candidate** if on held-out under T2, against the
shipping nominal: items 1 to 5 of GP2c-R with their thresholds unchanged (P1 at most 0.95 and
bootstrap upper end below 1.00; Y1 at most 1.00 and S1 equal within 1e-6; every seed P1 at most
1.00; every stratum at most 1.05; the 10^5 sample sweep), and Yp at most 1.00 times the
nominal's on G1b and T2b. Fail: the residual stays off and the table is the result.

Named outcome, reported whether or not GP2d-R passes, and NOT a deployment rule: "improves
heading, not position" if Y1 ratio at most 0.80, the lateral position RMSE ratio at 1.0 s
(the across component of 2.2, macro) at most 0.90 with bootstrap upper end below 1.00, P1
ratio at most 1.00 and every stratum at most 1.05. Whether a model with that outcome may drive
is the owner's decision (section 9).

**Validity above 1.3 m/s.** A v0.4 bundle's `v_max_mps` is the validity rule of the pipeline on
main applied to the new train data, capped at 1.5. It may exceed `dyn-v0.1`'s 1.3 only if the
bundle's model (N4, or `dyn-v0.1` with R4) passed its gate AND, on the held-out windows with a
start speed of at least 1.15 m/s, in each direction with at least 30 such windows, its position
RMSE at 1.0 s is at most 1.00 times `dyn-v0.1`'s. A direction with fewer than 30 windows makes
the item "cannot be applied" and the validity stays 1.3. No bundle gets a validity above the
speeds it was fitted on by decision.

Also printed in the same held-out pass, deciding nothing: the frozen v0.3 candidates
`dyn-v0.3y` and `dyn-v0.3yr`.

### 2.5 Confirmation before anything drives

Unchanged from v0.3 section 2.4: at the next session the first PnP runs are driven with the
deployed model; on at least 120 s of that driving, both directions, a candidate must have P1
at most 1.00 times and Y1 at most 1.00 times the deployed model's before it is switched on.

## 3. Perception (GP1c), new attempt after GP1b failed

### 3.1 The candidate

One candidate, `fused-v1`, with NO retraining: the frozen weights of `trackperc-v2-2026-11-14`.
Per scan, the geometric tracer and the refined net both run. The output keeps the geometric
tracer's stations and its corridor edges. The centre of each station is moved towards the
net's refined centreline where the net has one within reach, and stays inside the geometric
corridor; the half widths are recomputed against the geometric edges. Stations the net has
beyond the tracer's last valid station are appended. The fusion is declared in the bundle
(`io.arch.type`), so an old loader refuses it.

Disclosed: with this construction the valid count is at least the tracer's and the corridor on
shared stations is the tracer's. So the range item and the label-free items hold largely BY
CONSTRUCTION, and every report flags them so. What the gate can still fail on is whether the
net's better centre survives the fusion, on fresh data, in every cell. `fused-v1` is reported
as "geometric plus net centre", never as the net.

The fusion's settings are chosen on development data only: sim, the v0.2 `train` roles,
H-run, A-run, the 2026-11-12 layout bags and the folds F1 and F2 with their fold models. All
of these have been read before. The code is frozen (git SHA recorded) before any perception
`heldout` scan of section 1 is read.

### 3.2 Gate GP1c

Sets: H14 = labelled scans of the perception `heldout` segments of section 1; A14 = all moving
scans of them (wheel speed at least 0.2 m/s). Metrics and bootstrap as v0.2 section 2.

1. H14 holds at least 300 labelled scans. Fewer: undecided, geometric stays.
2. E_c(fused) at most 0.90 times E_c(geometric) on H14, bootstrap upper end below 1.00.
3. V(fused) at least V(geometric) on H14.
4. Every cell (direction by speed bin [0.2, 0.6), [0.6, 0.85), 0.85 and up) with at least 40
   labelled scans: E_c(fused) at most 1.10 times E_c(geometric).
5. Label-free on A14: mean intrusion at most max(0.02, geometric + 0.005); boundary residual
   median at most 0.10 m; clearance deficit p90 at most 0.05 m; C95(fused) at least
   C95(geometric) minus 0.02.
6. The rule of v0.3 section 3.3 items 1 to 5 holds for `fused-v1` on the development sets
   (H-run, A-run, the folds with their fold models, the 2026-11-12 layout). Reported as
   development data. A fold whose model was not kept is "cannot be applied" and is said.
7. Latency p99 under 10 ms on the dev machine, preprocess plus tracer plus net plus fusion.
8. The L5 PnP canary with the fused backend: every canary case completes with 0 wall contacts
   and a median lap time at most 1.03 times the geometric backend's on the same case and seed.

All thresholds of items 2 to 5 are those of v0.2 section 2, unchanged.

Outcomes: pass means "candidate backend in the layout family it trained on, on a fresh
session". It is not evidence for unseen layouts. Fails 2 or 3: "no better than geometric in
the layout". Passes 2 and 3 and fails another item: "better where it trained, not shown
elsewhere". Geometric stays deployed in every case until the on-car rule of 3.4.

### 3.3 M-A1 (a measurement, same pass, no threshold)

The frozen `trackperc-v2-2026-11-14`, refined and unrefined, on H14 and A14, next to the fused
numbers. And the shadow output recorded on the car on 2026-11-14 (`/pnp/shadow/*`): its
latency on the Orin from the shadow status, and whether the recorded shadow tracks equal the
offline ones on the same scans.

### 3.4 On the car

Unchanged from v0.2 section 2: shadow first, p99 under 10 ms on the Orin, then on the first
layout the net never trained on, driving at the same cap as a geometric run in the same
direction, mean track confidence at least the geometric run's minus 0.03 and no new TTC or
obstacle brake. A variant layout of section 6.2 counts as "a layout the net never trained on"
for this rule only if the owner says so (section 9); until then it is reported as a variant.

### 3.5 M-A2, does a better track matter (a sim measurement with a stated reading)

Nobody has measured whether a more accurate centreline changes lap time. In sim, on the
development cases T and S1 to S4, both directions, seed 11, 6 laps at the ceiling cap with the
lap memory off: condition g drives on `geometric-v1`, condition o on a `LocalTrack` built from
the ground truth track (same message, same 16 stations). Per pair: the median lap time of laps
3 to 6 of o over that of g.

Reading, fixed now: median ratio at least 0.97 and no pair below 0.93 means "a perfect track
buys under 3 percent of lap time in sim at these speeds; accuracy has no measured path to lap
time". Median ratio at most 0.95 means "track accuracy matters". Between: undecided. Contacts
and completion of both are reported. This is a measurement on development tracks. It gates
nothing; it is the evidence for the owner's decision on perception as a target (section 9).

## 4. Lap timing

### 4.1 The camera lap counter (M-CAM)

Hypothesis, written before anyone tested it: the counter's `min_lap_time_s` is 6.0 s and the
GP3 laps took 5.50 to 5.80 s, so the counter could only take every second crossing, which
gives exactly 10 of 20. If the replay of the recorded frames of L0 shows another cause, the
hypothesis is reported as wrong.

A changed counter (any change of its parameters or code) is developed on the lap `dev` runs
and then replayed once on the lap `test` runs (K1, K2, X2, X4, X7) at the recorded frame rate.
It is accepted for use at the next session if, on every test run, its lap count equals the
count of 4.2's reference and it has no false count. The frames were recorded at about 2 Hz; if
a run cannot be decided at that rate, that is reported and the run is "cannot be applied", not
a pass. On the car the registered rules that name the camera counter (v0.2 4.3 and 5.2, v0.3
D3-C0) are unchanged.

### 4.2 The offline lap reference (M-LR)

`lapref` is an offline tool that finds the laps of one whole run from the gated steering
command and wheel distance only (no scan, no camera, no pose, no planner or lap memory
output). It is not causal and cannot run on the car.

Development: the 2026-11-13 bags and the lap `dev` runs. On the 2026-11-13 segments with a map
pose (G1a, G1b, T0, T1, T2a plus T2b) its lap count and lap times are printed against the pose
detector; on L0 it must find 20 laps. These are development numbers.

Frozen (git SHA) before the lap `test` runs are read.

### 4.3 Lap gate v2 on fresh real runs (M-L1)

Lap gate v2 at the commit battery B4 ran on, replayed once over the five lap `test` runs, and
`lapref` over the same runs. They share no input except wheel distance. M-L1 holds if in every
test run the two lap counts are equal and at least 95 percent of the laps differ by at most
0.3 s. If it holds, both are reported as "agree on fresh runs". If it does not, every
differing lap is listed and neither is trusted on the strength of the other; the phone video
of the next session decides.

Limits, stated: the mean lap times of these runs were known to the author (0.2). `lapref` and
the gate are not checked against an outside clock here. M-L1 does not replace D3-C0.

### 4.4 D3-C0 at the next session

Unchanged (v0.3 section 1.5): two 10 lap runs in observe mode, one per direction, lap count
equal to the reference and at least 18 of 20 lap times within 0.3 s. The reference is the
camera lap counter if the owner accepts it as primary and M-CAM held; otherwise the phone
video of the same two runs, which is therefore filmed in any case. `lapref` on the same runs
is reported next to it and decides nothing unless the owner makes it the reference
(section 9).

## 5. Lap learning on obstacle variants in sim (battery B5)

B5 runs only if D3-S0 and D3-S1 of B4 passed, on the same frozen code. If either failed, the
B5 tooling may be merged but no B5 case with seeds 31 or 32 is run by anyone until a later
pre-registration says so.

Cases, seeds, conditions and thresholds: yaml `lap_battery_b5`. O1 is a plain round donut, the
worst case for a scan gate because every place looks like every other. O2 to O4 add solid
boxes against the walls. Scoring is that of v0.3 section 1.3 (ground truth crossing stamps).
16 runs per condition (12 if O4 is dropped).

- D4-S0, the rule of D3-S0 on B5: S0a on condition a (count equal, every lap within 0.3 s);
  S0b in c and v (count equal in every run, at least 98 percent within 0.3 s, none off by more
  than 1.0 s).
- D4-S1, the rule of D3-S1 on B5 with 15 of 16 in place of 22 of 24 (11 of 12 if O4 is
  dropped): 0 contacts in c; completion; reach against the paired ceiling control at most 1.05;
  T_late below T1 in every completing run and median r at most 0.80.
- D4-S2, the rule of D3-S2 for condition v, same thresholds, 15 of 16.

15 of 16 is a share of 0.94, above the 0.92 of 22 of 24. A pass of D4-S0 and D4-S1 is required
before D drives the cap on a variant layout on the car (section 6.2). They do not gate D on the
plain donut; B4 does.

## 6. The car

### 6.1 Order

D may drive the cap only after D3-S0, D3-S1 (B4) and D3-C0 (on the car), as v0.3 says. D v1
only in the variant the table of v0.3 section 1.4 allows.

### 6.2 GP4-V, lap learning on an obstacle variant (a new gate; GP4 is unchanged)

The owner has one donut and no material for another track. A variant layout is the donut with
at least two solid objects placed so that the lane changes by at least 0.15 m at two or more
places, in positions not used in any earlier session, not driven before the first run of the
protocol, and in no training set of a deployed model.

GP4-V is the protocol and the thresholds of v0.2 section 5.2, word for word, on a variant
layout, with D4-S0 and D4-S1 passed first. It is reported as GP4-V, "lap learning on an
obstacle variant of a known track". GP4 itself asks for a layout "built for the session" and
stays not attempted; whether a variant satisfies GP4 is the owner's decision and is not taken
here.

### 6.3 C on the car, again

v0.2 section 4.2 is unchanged and was missed on 2026-11-14. It is judged again from the M-T
report of the next session. No threshold moves.

### 6.4 A cap above 1.3

A cap above 1.3 m/s is driven only with a bundle whose validity allows it by section 2.4, and
first as two runs per direction at that cap with the lap memory off, compared with the runs at
1.3 of the same session: no TTC or obstacle brake, no contact, mean wheel speed not lower.
GP3 was registered at 1.0 and stays so; a 20 lap run at another cap is reported as a repeat at
that cap.

### 6.5 The tracker comparison

The mapped tracker's 5.6 s was driven on 2026-11-13. A statement that the mapless driver is
faster than the mapped tracker needs both driven in the same session on the same layout,
same direction, at least 5 laps each. Until then the two numbers are reported with their
dates and no comparison is claimed.

## 7. Unchanged, carried over

- v0.2 sections 4.2, 4.3, 5.2, 5.3 and 6, and v0.3 sections 1 (B4, D3 gates), 2.4, 5 (M-T) and
  6 apply as written.
- GP2b (online adaptation) is not attempted in v0.4.
- What counts as learned, added: N4 is system identification (classical). R4 is learned.
  `fused-v1` is classical geometry with a learned centre and is named "geometric plus net
  centre". `lapref`, the camera counter and lap gate v2 are classical.

## 8. Cut from v0.4 (not attempted, so no gate)

A retrained net; a loss aimed at V or C95; a speed-channel residual; RMA; D v2; a keyframe
ring for the lap gate; the limiter replay reproduction; any change to `racer_safety`.

## 9. Open owner decisions this file does not take

1. Real-time scheduling of the planner process, and any change to safety_node timing.
2. rf2o (GPL-3.0) on the car.
3. The camera lap counter as primary lap timing, and whether `lapref` may be a reference.
4. The D v1 zone floor and zone merge.
5. Lap gate v2: whether flagged laps feed the length estimate; how strictly M-R is read;
   whether T2 canary seeds are added.
6. Whether V and C95 are a target for perception at all (M-A2 is the evidence).
7. Whether a dynamics model that improves heading and not position may drive (2.4).
8. Whether an obstacle variant counts as an unseen layout for GP4 and for the on-car net rule.
9. The left steering stop (0.504 rad) in `vehicle_params` in place of the symmetric 0.449.
10. The multi-algorithm benchmark. Not designed here.
