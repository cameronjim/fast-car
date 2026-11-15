# Pre-registration: Plug-and-Play Racer v0.4, addendum 1 (after the outside review of 2026-11-14)

Written 2026-11-14, after an outside static review of checkout 5e531eb and after the author
checked its code claims against main f5db7f1. Written BEFORE any v0.4 dynamics candidate was
fitted, before any 2026-11-14 held-out or test segment was read by any model, lap tool or
perception code, before `fused-v1` was frozen, and before the result of battery B4 was known.
No result may change this file. A later correction is a new dated section appended at the end.

Constants file: `evaluation/prereg/pnp-v0.4-addendum-1.yaml`, SHA-256
`ada5a83675e5c843fb8e121a1190e4458f823b1ed2ff32a34a46c5fb4ff76824`.

This addendum belongs to `docs/notes/preregistration-pnp-v0.4.md` and
`evaluation/prereg/pnp-v0.4.yaml` (SHA-256
`78510c1423278f80c42511e6caab095818598af6db6a284dd5479f74a03a1d63`). It is in force only once
those two files are on main with that SHA. It replaces and loosens nothing in them or in the
v0.2 and v0.3 files. Every gate and measurement there stays as written and keeps its result.
Everything below is a NEW name. Battery B4 (D3 gates) and battery B5 (D4 gates) are untouched.

## 0. What was known when this was written

- The v0.3 results and the 2026-11-14 car session, as section 0 of the v0.4 file states them.
- The review (static, no replays, no files changed). Its code claims were checked by reading
  main: the final MPPI sequence is not checked for collision; odometry and lap guidance
  freshness use arrival time; the planner's actuator estimate follows published commands; dead
  reckoning between scans omits a learned yaw term; the dynamics model has one speed state for
  wheel speed and ground motion; the lap station inside a rollout advances by path length.
- From development data only, by the agent of brief 38 (draft PR 187): with its blend at 1 the
  fused corridor, rebuilt as a consumer rebuilds it, does not cover 4 to 8 percent of the
  geometric corridor's area on average, 20 to 30 percent at the 99th percentile, about 33
  percent on the worst scan; at blend 0 the loss is exactly 0. No 2026-11-14 scan was read.
- Brief 37 (dynamics) was stopped before any fit. Its train-only measurements M-B3 and M-B2
  may exist when this is merged; they read no held-out data.
- B4 was running. No B4 number was known.

## 1. GP3, stated as registered (a record, no new gate)

v0.2 section 4.3 counts GP3's laps by the camera lap counter, cross-checked by wheel distance,
and says the first attempt at a configuration is the result. On 2026-11-14 the first attempt
(cap 1.0, ccw, geometric-v1, dyn-v0.1) gave 20 laps without contact as observed by the owner,
and the camera counter counted 10. GP3 is therefore reported as: "not met as registered; 20
clean laps observed, camera count 10 of 20". The sentence "GP3 PASSED" in section 0.2 of the
v0.4 file is read with this wording wherever GP3 is reported. A later 20 lap run is a repeat,
named GP3-R1, GP3-R2 and so on, with its own outcome under the unchanged rule of 4.3, and is
never reported as GP3. No pre-registered rule names GP3 as a precondition.

## 2. Dynamics: wheel motion and ground motion (GP2e), in the same held-out pass as GP2d

GP2d (v0.4 section 2) is unchanged: M-B3, M-B2, N4, R4, the validity rule, every item. The
dynamics `heldout` segments are still read once, in one pass, after every v0.4 dynamics
candidate is frozen. That pass now also scores the candidates below. This is possible because
no v0.4 candidate had been fitted and no held-out segment had been read when this was written.

### 2.1 Why

The model's one speed state is initialised from wheel odometry, is scored against wheel
telemetry (S1), and also advances position. In tight turns the car covers less ground than the
wheels report. A yaw term cannot repair that, and a speed term that repairs position must fail
S1. The candidates below keep the wheel-speed channel as it is and give ground motion its own
quantity.

### 2.2 Step contract 2

The wheel channel (steering lag, speed lag, limits, dead zone) is the contract of the bundle
schema 1.x, unchanged, with no learned term acting on it. Position and yaw advance by:

```
alpha    = clamp(table(delta, v), 0.60, 1.05)
rho      = 0
if learned: alpha = clamp(alpha + 0.10 * tanh(m[0]), 0.60, 1.05) ; rho = 0.25 * tanh(m[1])
v_g      = alpha * v
yaw_rate = (1 + rho) * v * tan(delta) / L / (1 + understeer * v^2)
x_next   = x + v_g * cos(yaw) * dt ; y_next = y + v_g * sin(yaw) * dt ; yaw_next = yaw + yaw_rate * dt
```

`v` is the wheel-speed state, `m` the output of a 4-32-32-2 network on (v, delta, speed
command, steering command). `table` is bilinear on the grid of the yaml. Both corrections
multiply: at zero wheel speed ground motion and yaw rate are zero for any weights. A bundle of
this contract carries `bundle_schema_version` 2.0.0, so every loader that does not implement
it refuses it. The additive residual of 1.x is not allowed in a 2.0.0 bundle. No
vehicle_params field changes.

### 2.3 Candidates and training

- T4 (classical): the nominal plus the table. Table nodes by least squares with a second
  difference penalty, the weight chosen from the yaml list on the validation part of train. A
  node with fewer than 30 supporting windows is tied to its nearest supported node.
- TL4 (learned): T4 plus the bounded network, seeds 0, 1 and 2, config and shipping seed by
  validation P1 before held-out is read.

Both are fitted on the GP2d train data only, on windows built like the gate's windows (wheel
speed as the initial speed, the recorded gated commands, open loop, horizons 0.5, 1.0 and
1.5 s), with a robust loss on the relative motion split into along, cross and heading parts,
weighted by the truth's quality flags. As with R4, the base nominal cannot be known before
held-out is read, so both are fitted on `dyn-v0.1` and on N4, and the pass uses the ones on
the nominal that ships.

### 2.4 M-B4, label sensitivity (a measurement, train only, committed before any fit)

The truth T2 is seeded with wheel motion and may lean towards "ground equals wheel". Per
stratum on train: the median of ground distance over wheel distance in 1.0 s windows under T2,
under T2s (the same chain seeded with 0.80 of the wheel translation), and under T2 with the
scan clock shifted by 10 ms each way; and the share of windows rejected by the quality flags,
by speed bin and direction. A stratum with at least 30 windows whose T2 and T2s medians differ
by more than 0.03 is reported as "label depends on the seed". No threshold; the gate items
under T2s below are the guard.

### 2.5 Metrics

P1, Y1, S1, strata, macro averaging and the bootstrap are those of GP2d. Added: A1 and C1, the
RMSE at 1.0 s of the position error along and across the true heading at the window start
(the components of M-B3); Pp, the position RMSE at 1.0 s against the map pose on G1b and T2b,
next to Yp. For a contract 1 model, position is what its one speed state integrates, as in
GP2d.

### 2.6 Gate GP2e

**GP2e-T: the table is a candidate ground-motion model** if on held-out, against the shipping
nominal (N4 if GP2d-N passes, else `dyn-v0.1`):

1. P1(T4) at most 0.95 times P1(nominal) under T2, bootstrap upper end of the ratio below 1.00.
2. A1(T4) at most 1.00 times A1(nominal) and C1(T4) at most 1.00 times C1(nominal).
3. Y1 and S1 equal to the nominal's within 1e-6 (the table changes neither channel).
4. Every stratum: position RMSE at 1.0 s at most 1.05 times the nominal's.
5. No supported table node on a bound.
6. Under T2s on the same windows: P1(T4) at most 1.00 times P1(nominal).
7. Pp(T4) at most 1.00 times Pp(nominal) on G1b and T2b (a re-read, the only pose data).

**GP2e-L: the learned correction is used on top of the table** if GP2e-T passed and, against
T4 on the same base:

1. P1(TL4) at most 0.95 times P1(T4) under T2, bootstrap upper end below 1.00.
2. Y1(TL4) at most 1.00 times Y1(T4). S1 equal within 1e-6.
3. Every seed: P1 at most 1.00 times P1(T4).
4. Every stratum at most 1.05 times T4's.
5. A 10^5 sample sweep of the validity box: finite, alpha and rho inside their bounds, zero
   ground motion and zero yaw rate at zero wheel speed.
6. Under T2s: P1(TL4) at most 1.00 times P1(T4).
7. Pp and Yp at most 1.00 times T4's on G1b and T2b.

Outcomes. GP2e-T fails: the one-speed contract stays, and the table of all candidates is the
result. GP2e-T passes and GP2e-L fails or is not evaluated: "the table wins; the learned
correction is not used". Both pass: TL4 is the candidate. If GP2e-T fails, TL4 is printed and
decides nothing.

Printed in the same pass on identical windows, every metric: `dyn-v0.1`, N4, R4, T4, TL4.

A candidate of contract 2 drives nothing until the planner implements contract 2 and
reproduces the bundle's golden rollouts, and until the confirmation of v0.4 section 2.5 holds
for it. Validity above 1.3 m/s follows v0.4 section 2.4 unchanged. T4 counts as classical
(system identification), TL4 as learned.

Limits, stated: T2, T2s and the map pose all share sensors with the model's inputs; none is
independent truth. Windows overlap; the block bootstrap is the only guard. The held-out data
is the same layout family as train.

## 3. Perception

### 3.1 No training before the oracle

No perception model is trained, fine-tuned or re-exported with new weights until M-A2 (v0.4
section 3.5) has been run and reported. M-A2's reading is unchanged.

### 3.2 `fused-v1`: a slope limit on the shift, and gate GP1c-K

Disclosed change to the candidate of v0.4 section 3.1, made before its freeze and before any
2026-11-14 scan was read. The fused corridor is what a consumer rebuilds from the output:
each edge point is the station plus or minus its half width along the normal of the output's
own polyline. When neighbouring centres move by different amounts those normals turn and the
rebuilt corridor is no longer the tracer's (section 0). The statement of 3.1 that range and
the label-free items hold "by construction" is withdrawn; reports do not use it.

The candidate gets one more rule. With d_i the shift of station i along the tracer's normal
as 3.1 defines it, the applied shift is the largest profile that is nowhere larger in size
than d_i, keeps its sign, and has |d_{i+1} - d_i| at most `max_shift_slope` times the station
spacing. A station that does not move counts as zero. The rule can only shrink a shift.
Everything else of 3.1 stands. `max_shift_slope` is the largest value of the yaml grid for
which every development set of 3.1 meets the development margin of the yaml (mean and p99 of
coverage loss and of excess); if none does, no candidate is frozen and GP1c is reported as not
attempted. The other fusion settings are chosen as 3.1 says. Nothing is tuned after.

Coverage loss: the share of the geometric corridor's area the fused corridor does not cover.
Excess: the area of the fused corridor over the tracer's stations that lies outside the
geometric corridor, as a share of the geometric area. Both on a 0.01 m grid, appended far
stations left out, per scan.

**GP1c-K** (a new gate; the eight items of GP1c are unchanged), on A14 in the same single
pass: mean coverage loss at most 0.01 and p99 at most 0.05; mean excess at most 0.01 and p99
at most 0.05. The worst scan of each is printed.

`fused-v1` is a candidate backend only if GP1c and GP1c-K both pass. GP1c passed and GP1c-K
failed is reported as "better centre, corridor not kept".

### 3.3 M-A2x and M-A2w (diagnostics next to M-A2; they decide nothing)

M-A2x: on M-A2's pairs and on development case OD, a third condition f with the frozen learned
candidate through the same message, stations, frame and timing; per run the mean track
confidence, the cost terms of the planner's final sequence, and which term limited speed.
M-A2w, only if M-A2 reads "no measured path": conditions g and o again with the planner's
boundary weight at 0.5 and at 2.0 times its value. A median ratio at most 0.95 at either
setting is reported as "headroom only with other planner weights". No default changes.
Stated limit: an oracle with fixed planner settings measures the value of a better input to
this planner, not a bound on perception.

## 4. Planner (measurements; v0.2 section 4.2 is unchanged and is judged again as written)

- M-C1, final sequence. The planner's final command sequence is checked for collision in the
  solver's own evaluation and replaced by the best evaluated collision-free sample when it
  fails. Reported from a replay of segments L0, X0 and X1 and from the sim canary: the share
  of ticks where the unchecked sequence collided although a collision-free sample existed,
  before and after the change. No threshold.
- M-C2, timing chain, at the next session: per tick the source stamp and receipt time of scan,
  track and odometry, solve start and end, publish time, and the safety cycle that first used
  the command. safety_node is not changed: it stamps `/drive` with its cycle time and passes
  an unaltered field through bit for bit, so the first `/drive` equal to a `/drive_raw` in a
  passed-through field bounds the receipt to one cycle. Reported: p50, p99 and max of each
  leg and the share of commands so identified. No threshold.
- M-C3, command quantisation: from recorded runs, the set of distinct commanded speeds and the
  steady wheel speeds they produced. No threshold.
- M-T2, limiter replay, a new attempt next to M-T (whose result stays): the replay of v0.3
  section 5 with the command latched at each safety cycle taken from the echo above instead of
  the recorder's receive time. It "reproduces" if every `rate_limit` ENGAGE of the bag is
  reproduced at the same cycle. Cycles where the echo is ambiguous are listed.

## 5. Lap learning

### 5.1 Which code a battery result holds for

A lap battery result holds for the commit it ran on and for later commits that change no file
under `ros_ws/src/racer_laps/`, `ros_ws/src/racer_planner/` or the lap scoring. B5 runs as
registered, on the code B4 ran on.

### 5.2 Battery B6 (after a planner or lap change)

If the car is to run a planner or lap node that differs from the commit B4 ran on, D may drive
the cap only after B6 on that code: part 1, the B4 cases, conditions, laps, plant, scorer and
thresholds with seeds 41 and 42, gates D5-S0, D5-S1, D5-S2 by the rules of D3-S0, D3-S1,
D3-S2 word for word; part 2, the B5 cases O1 to O4 with seeds 41 and 42, gates D6-S0, D6-S1,
D6-S2 by the rules of D4. D5-S0 and D5-S1 are required for the plain donut, D6-S0 and D6-S1
in addition for a variant layout. B6 runs once, on one frozen commit (SHA reported), only if
D3-S0 and D3-S1 passed, and nobody runs a B4 or B5 case with seed 41 or 42 before that. B4,
B5 and their gates keep their results whatever B6 says.

### 5.3 M-D1, what the memory buys (development diagnostics; decide nothing)

On the development cases and seeds of the yaml, conditions a, b, c, m, f, u, t and z. Reported
per condition: contacts, completion, late-lap time against the paired ceiling control. Reading
fixed now, for design only: a learned value profile is part of a later D only if u or f has a
median late-lap time at most 0.97 times m's on the same pairs with no more contacts; otherwise
the later D keeps the planner's own progress reward and uses memory as a speed bound. No
development number is reported as evidence that D works.

### 5.4 Wording

The stored per-station speeds are an empirical speed envelope: speeds that were driven without
contact at that station. No report calls it a safe set or claims a guarantee.

## 6. Next session, added

- M-SID, excitation runs: per direction, the PnP planner with its cap stepped through the
  yaml list, held 8 s each, three cycles, plus the data laps of v0.4. Commands are model
  inputs, not imitation targets. The bags get their roles by the `future_sessions` rule of the
  v0.4 yaml like any other bag.
- M-C2 is recorded in every PnP run.
- A 20 lap run is GP3-R1 (section 1).

## 7. Notes on earlier gates (no threshold is edited)

- GP1b: the baseline's V was at its maximum of 16, so item V asked for saturation on every
  scan, and V mixes range with trust. GP1b also failed C95 on fold F1 and on the 2026-11-12
  layout.
- GP2c-R item 1: cutting RMSE by 5 percent means removing 9.75 percent of squared error. The
  perfect yaw oracle of M-B3 is a diagnostic and not a bound; its sentence means "not
  expected", not "impossible".
- S1: the allowance of 1.02 on a baseline of 0.037 m/s is 0.74 mm/s. The 3 mm/s of v0.3
  section 2.1 is the excess of the failed candidate.
- D3-S1, D4-S1 and GP4 may ask for an improvement the track does not offer; the ceiling and
  schedule controls are what make them readable.
- A round donut gives no absolute lap phase from scans.
- Obstacle variants support "adaptation on a perturbed known track", never "generalisation
  over tracks".

## 8. Owner decisions of 2026-11-14 (given after the first commit of this file, before any result it governs; recorded here, nothing was scored in between)

1. safety_node may record the receipt time of `/drive_raw`, as instrumentation only, with no
   change to any safety behaviour, threshold or the gating path. Consequence for section 4:
   M-C2 and M-T2 use that record where a bag holds it and the echo where it does not; every
   table says which. Until that record is on main and proven bit-identical, the echo is used.
2. The command pulse grid goes into `config/vehicle_params.yaml` (schema bump, generated
   bindings). M-C3 stays a measurement and is the calibration evidence for the value. The
   planner may predict with the reachable command set only after that bump; such a change is
   a planner change in the sense of section 5.1.
3. Dynamics bundle schema 2.0.0 (section 2.2) is approved. No vehicle_params change.
4. The GP3 wording of section 1 is accepted. GP3 will be attempted again at the next session
   and is recorded as GP3-R1, a repeat.
5. Inside a caution zone the cap wins; a bound below the drivable floor means hold (stop or
   recover), never a raise. This binds every lap learning variant designed from now on. B4,
   B5 and B6 run the registered code; their conditions are not changed. In M-D1 the
   conditions keep the registered zone rule and the new rule is reported as an extra
   condition z (D v1 with the new rule), development only.
6. Brief 42 and every other change under `racer_planner`, `racer_laps`, `racer_safety`,
   `config/vehicle_params.yaml` or the lap scoring merges after the B5 run is finished or
   declared not run. Section 5.2 (B6) then applies to the changed code.
7. About 10 minutes of excitation runs at the next session are approved (M-SID, section 6):
   the cap schedule of the yaml in both directions, on the lane centre and with the lateral
   offsets of the yaml, at most 600 s of driving in total. If the planner has no lateral
   offset parameter on the session's code, the steering variation is driven by the gap
   follow race profile instead and that is said.

Still open, not taken here: real-time scheduling and any safety_node timing change; rf2o; the
camera lap counter as primary timing and `lapref` as a reference; the D v1 zone-floor
deviation of v0.3; the three lap gate v2 questions; the multi-algorithm benchmark.
