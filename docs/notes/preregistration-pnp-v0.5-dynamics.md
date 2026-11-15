# Pre-registration: Plug-and-Play Racer v0.5 dynamics (after the v0.4 held-out pass, before the 2026-11-15 car session)

Written 2026-11-14 at night, after the v0.4 dynamics held-out pass (PR 193, freeze commit
`416e68501c424caeb1f6dc80b6a0ee9edc7a288c`) and BEFORE the car session of 2026-11-15 was
driven, before any v0.5 dynamics code and before any v0.5 fit. No result may change this file.
A later correction is a new dated section appended at the end that says what changed and why,
and every number produced before the correction stays reported.

Constants file: `evaluation/prereg/pnp-v0.5-dynamics.yaml`, SHA-256
`2767f297d951c11683b40834b864b832e8cbd0b01343e2765681e11b22e86a82`.

Earlier files, unchanged and in force: `preregistration-pnp-v0.2.md`, `-v0.3.md`, `-v0.4.md`,
`-v0.4-addendum-1.md` and their yaml files (`pnp-v0.4.yaml` SHA-256
`78510c1423278f80c42511e6caab095818598af6db6a284dd5479f74a03a1d63`, `pnp-v0.4-addendum-1.yaml`
SHA-256 `ada5a83675e5c843fb8e121a1190e4458f823b1ed2ff32a34a46c5fb4ff76824`). Every earlier
gate keeps its text and its result: GP2d-N FAIL, GP2d-R FAIL, GP2e-T FAIL, GP2e-L not
evaluated. Everything below is a NEW name. This file covers dynamics only; it gives no
perception or lap role and changes no session step other than the excitation runs.

## 0. What was known when this was written

All of it is in PR 193 (`training/pnp_models/dyn-v0.4-2026-11-14/`).

- Held-out pass of 2026-11-14 (1281 windows, read once). P1 at 1.0 s: `dyn-v0.1` 0.0854 m,
  N4 0.0826, R4 0.0812, T4 0.0535, TL4 0.0440 (printed, decided nothing).
- GP2d-N failed on S1 (ratio 1.00256 against 1.00) and on `understeer_s2_per_m2` at its lower
  bound 0. The refit also moved the dead band from 0.22 to 0.33 m/s and the steering time
  constant from 0.028 to 0.014 s, below the 0.02 s model step.
- GP2d-R failed on P1 (0.9517 against 0.95).
- GP2e-T failed on item 5 only: two supported table nodes on the upper bound 1.05, steering 0
  at 0.25 m/s and steering +0.667 at 1.5 m/s. P1 ratio 0.627, every other item passed.
- M-B3 (train): along error 0.076 m, across 0.058 m; perfect yaw ratio 0.895, perfect speed
  0.971 overall, but 0.729 and 0.825 in the two slowest strata.
- M-B4 (train): ground over wheel distance 0.88 to 0.93 per stratum (clockwise 0.88 to 0.89,
  counter-clockwise 0.90 to 0.93), unchanged by the label seed and the scan clock shift.
- M-B2 (train): clockwise the model's yaw rate is too large by about 0.06 rad/s at every
  speed; counter-clockwise it is 0.02 too large below 0.6 m/s and 0.07 to 0.12 too small above
  0.85 m/s. The steering command is at the limit 3 to 17 percent of the time clockwise, never
  counter-clockwise.
- From the fit summary and the code of the v0.4 table fit: the largest wheel speed in the data
  is 1.36 m/s, so the 1.5 m/s nodes had no data at or above them; a window counted as support
  for a node when it put one step of interpolation weight on it; and the table multiplied the
  model's predicted wheel speed, not the measured one.
- Every 2026-11-14 held-out segment has now been read in the v0.4 pass. It cannot judge a new
  candidate.
- Not known: anything about the 2026-11-15 session. It had not been driven.

## 1. Why the v0.4 candidates failed, as read here

1. **The 1.5 m/s node was outside the data.** Its 99 "supporting" windows lay between 1.25 and
   1.36 m/s, less than half way to the node, so its value was the slope of that stretch
   carried over 0.25 m/s. The bound caught an extrapolation.
2. **The 0.25 m/s row fitted the wheel channel's error, not the ground ratio.** The table
   multiplied the model's own speed. At low speed that speed is wrong (perfect speed is worth
   17 to 27 percent of P1 there), so the row took values from 0.73 to 1.05 where the measured
   ratio is 0.89 to 0.90.
3. **The bounds were not too tight and no physics lies outside them.** The directly measured
   ratio is never above 0.93 in any stratum.
4. **Understeer sat on 0 because 0 was the edge of its range,** and because the yaw channel is
   driven by wheel speed while the ground ratio rises with speed at lock, which acts against a
   positive understeer term. The refit of the other scalars was the one-speed model bending
   its yaw and lag terms to reduce a position error that is a ground-motion error.

## 2. Step contract 2.1 (bundle schema 2.1.0)

Contract 2.0 (addendum 1 section 2.2) with one classical term added, a yaw gain table:

```
alpha    = clamp(table_a(delta, v), 0.70, 1.05)
gamma    = clamp(table_y(delta, v), -0.25, 0.25)
rho      = 0
if learned: alpha = clamp(alpha + 0.10 * tanh(m[0]), 0.70, 1.05) ; rho = 0.25 * tanh(m[1])
v_g      = alpha * v
yaw_rate = (1 + gamma) * (1 + rho) * v * tan(delta) / L / (1 + understeer * v^2)
x_next   = x + v_g * cos(yaw) * dt ; y_next = y + v_g * sin(yaw) * dt ; yaw_next = yaw + yaw_rate * dt
```

- The wheel channel, the steering channel (per-side gains, offset, lag, command delay) and
  `understeer` are those of `dyn-v0.1`, byte for byte. Nothing in v0.5 refits a scalar of the
  nominal. So S1 cannot move, the dead band stays 0.22 m/s, the steering time constant stays
  0.028 s, and no scalar has a one-sided range to sit on.
- Both tables are bilinear on one grid: steering as a share of each side's limit at -1, -0.5,
  0, 0.5, 1 and wheel speed at 0.35, 0.6, 0.85, 1.1, 1.3, 1.5 m/s, held constant outside the
  grid. The steering axis is signed, so every term is per side. `gamma` at steering 0 is fixed
  at 0 (a gain has no effect there and the data cannot identify it).
- The neutral value of each fitted quantity (alpha 1, gamma 0) and the measured values
  (alpha 0.88 to 0.93) are inside the ranges. Alpha range: the lower end 0.70 is two and a
  half times the largest loss measured in a stratum (0.12); the upper end 1.05 allows 5
  percent of calibration error above 1, since driven wheels do not turn slower than the
  ground outside braking. Gamma range 0.25: M-B2's largest bias is about 14 percent of the
  yaw rate.
- Both corrections multiply: at zero wheel speed ground motion and yaw rate are zero.
- Schema 2.1.0: a loader that does not implement it refuses the bundle. No vehicle_params
  field changes. Whether 2.1.0 is approved is the owner's decision; until then every v0.5
  bundle is `do-not-deploy` and the gates below are still scored.

## 3. Candidates and fits

- **T5 (classical): `dyn-v0.1` plus both tables.** Each table is fitted against its own
  measured target on 0.5 s windows, with the MEASURED wheel speed and the nominal's steering
  state as inputs: alpha against the ground distance along the heading, gamma against the yaw
  change. A window with any wheel speed sample under 0.30 m/s is not used. Huber loss, label
  quality weights, a second difference penalty whose weight is chosen from the yaml list on
  the selection set (alpha by A1, gamma by Y1).
- **Support.** A window belongs to the node nearest to its mean steering share and mean
  measured wheel speed. A node is supported with at least 60 windows. An unsupported node
  copies the nearest supported node of its steering column (else of its row on the same side)
  and is no free parameter.
- **TL5 (learned): T5 plus the bounded 4-32-32-2 network of contract 2.0,** trained on top of
  the frozen T5 on open-loop windows built like the gate's, seeds 0, 1, 2, two configurations,
  chosen by P1 on the selection set.
- No classical refit of the nominal (no N5) and no contract 1 candidate.
- Training data: yaml `data_roles.train`. Selection data: the 2026-11-14 held-out segments,
  never fitted on. Every candidate is frozen (git SHA in the report) before the held-out pass.

## 4. Measurements before the fit (no threshold; one stop rule)

- **M-B5.** The T5 method on the data that exists tonight (train without 2026-11-15), run and
  committed before the 2026-11-15 split file is committed. Reported: both tables, support,
  nodes on a bound, split-half differences, P1, A1 and Y1 on the selection set. If a supported
  node is on a bound, the reading of section 1 is wrong: the work stops and the design lead
  is told. The method of section 3 is not changed after M-B5 except by a dated correction
  appended here before the split file is committed.
- **M-B6.** The frozen v0.4 models (`dyn-v0.1`, N4, R4, T4, TL4 at the freeze commit) scored
  on every PnP run of 2026-11-15, whole, before the split file is written. They were frozen
  before that session existed. Printed, deciding nothing.

## 5. The 2026-11-15 session

### 5.1 Excitation runs (M-SID2)

Yaml block `excitation`. At most 600 s of driving (owner decision 7 of 2026-11-14), 552 s
planned, every run through safety_node and bagged, clockwise blocks first. Commands are model
inputs, not imitation targets.

| Block | What drives | Runs |
|---|---|---|
| P, speed transitions | PnP planner, cap schedule 0.5, 1.0, 0.7, 1.3 m/s, 8 s each, three cycles | cw 96 s, ccw 96 s |
| G, steering variation | gap follow race profile, lateral limit 2.0 m/s^2 | cw at 1.0 for 30 s, cw at 1.3 for 45 s, ccw at 1.3 for 45 s |
| L, steering at the limit | gap follow lane profile, deepest-ray target, 0.6 m/s, reverse escape off | cw 40 s, ccw 40 s |
| F, above 1.3 | gap follow race profile, 1.4 m/s for 20 s then 1.5 m/s for 60 s | cw, then ccw |

A wall contact ends the blocks of that direction. The 1.5 rung is driven only if the 1.4 rung
had no contact. Block F needs the owner's yes at the track; without it F is not driven and
nothing else changes. Runs that were not driven are simply absent.

### 5.2 Dynamics roles of the session's runs, fixed now

One rule for every run of every bag of the session, whoever drove it and on whatever layout
(yaml `session_2026_11_15`): a run shorter than 15 s or with under 1.0 m of wheel distance is
unused; otherwise the first 55 percent of its duration is train, the next 2.0 s unused, the
rest held-out. There are no cells and no labels, so nothing is chosen after looking. A tool
writes `evaluation/prereg/sessions/2026-11-15-dynamics-v05.yaml` from the bags; it is
committed with its SHA-256 before any v0.5 fit reads the session. Order: M-B6, the split
file, fits.

This rule gives dynamics roles for v0.5 only. The split that `pnp-v0.4.yaml`
(`future_sessions`) asks for is still written by its own tool and still governs everything
else.

## 6. Gates (all new; held-out is the 2026-11-15 held-out parts, scored once, in one pass)

Truth, windows, strata, merging, macro averaging, P1, A1, C1, Y1, S1, Pp, Yp and the bootstrap
are those of v0.4 and addendum 1. T2s is built for the 2026-11-15 held-out parts. Added
metrics: per direction P1 and Y1; the yaw rate bias per direction (model minus truth over
1.0 s windows); near-limit windows (mean absolute steering share at least 0.85, per side);
transition windows (the speed command changes by at least 0.2 m/s inside the window). The
comparison model is `dyn-v0.1`.

### GP2f-S: the tables may be trusted where they are used

1. No supported node of either table within 0.005 of a bound.
2. Split half: the tables refitted on the odd and on the even 10 s blocks of train differ at
   every supported node by at most 0.05 (alpha) and 0.08 (gamma).
3. At least 95 percent of the held-out rollout steps fall in the cell of a supported node.

This replaces what item 5 of GP2e-T was for, in two parts: a bound now reports a wrong model
(item 1), and support reports extrapolation (items 2 and 3, and GP2f-V).

### GP2f-T: T5 is a candidate

GP2f-S passed, and on held-out under T2, against `dyn-v0.1`:

1. P1 at most 0.90 times, bootstrap upper end of the ratio below 0.95.
2. A1 at most 1.00 times and C1 at most 1.00 times.
3. Y1 at most 0.90 times. S1 equal within 1e-6.
4. Every stratum: position RMSE at 1.0 s at most 1.05 times.
5. Each direction by itself: P1 at most 0.95 times and Y1 at most 1.00 times.
6. Each direction: the absolute yaw rate bias is at most 0.03 rad/s, or at most half of
   `dyn-v0.1`'s in that direction.
7. Near-limit windows, each side with at least 30: position RMSE at most 1.00 times.
8. Transition windows, if at least 30: position RMSE at most 1.00 times.
9. Under T2s: P1 at most 1.00 times.
10. Pp and Yp at most 1.00 times on G1b and T2b (a re-read, the only pose data).

An item whose minimum count is not met is "cannot be applied" and is reported; it does not
fail the gate. Items 1 to 6, 9 and 10 can always be applied.

### GP2f-L: the learned correction is used on top of T5

GP2f-T passed, and against T5:

1. P1 at most 0.95 times, bootstrap upper end below 1.00.
2. Y1 at most 1.00 times. S1 equal within 1e-6.
3. Every seed: P1 at most 1.00 times.
4. Every stratum at most 1.05 times. Each direction: P1 at most 1.00 times.
5. Near-limit and transition windows at most 1.05 times.
6. A 10^5 sample sweep of the validity box: finite, alpha inside its bounds, rho inside its
   bound, zero ground motion and zero yaw rate at zero wheel speed.
7. Saturation: the share of held-out steps where the network's alpha or rho output is beyond
   0.9 of its limit is at most 1 percent.
8. Under T2s: P1 at most 1.00 times.
9. Pp and Yp at most 1.00 times.

### GP2f-V: validity above 1.3 m/s

A v0.5 bundle's `v_max_mps` is 1.3 unless all of these hold, and then it is the smallest of
1.5, the pipeline's validity rule on train, and the top supported speed. The top supported
speed is the highest speed node such that it and every lower node from 0.6 m/s up have, on
each side, a supported node at share 0.5 or 1.0 in both tables.

1. The bundle's model passed its gate (GP2f-T for T5; GP2f-T and GP2f-L for TL5).
2. Each direction has at least 30 held-out windows with a start speed of at least 1.15 m/s,
   and their position RMSE at 1.0 s is at most 1.00 times `dyn-v0.1`'s.
3. The same with a start speed of at least 1.30 m/s.

A direction with too few windows makes the item "cannot be applied" and the validity stays 1.3.

### Outcomes

GP2f-S or GP2f-T fails: the one-speed contract stays, `dyn-v0.1` stays, the table of all
models is the result. GP2f-T passes and GP2f-L fails or is not evaluated: T5 is the candidate.
Both pass: TL5 is the candidate. Printed in the same pass on identical windows, every metric:
`dyn-v0.1`, T5, TL5 per seed, and the frozen v0.4 models N4, R4, T4, TL4 (deciding nothing).
A failed gate is a result. Nothing is tuned or re-scored after the pass.

## 7. GP2f-C: confirmation on the car, before a candidate drives

No v0.5 candidate can drive at the 2026-11-15 session: it does not exist yet. At the first
session after the fit, in this order, and the first failure switches the candidate off for
that session:

1. The planner on main implements contract 2.1 and reproduces the bundle's golden rollouts to
   1e-9; a 1.x bundle runs bit-identically. If lap learning is to drive the cap, battery B6
   (addendum 1 section 5.2) has passed on that code.
2. Fresh driving. The first PnP runs are driven with the deployed model. On at least 120 s of
   them, at least 40 s in each direction, the candidate has P1 at most 1.00 times and Y1 at
   most 1.00 times the deployed model's, overall and in each direction, and at least 95
   percent of the steps fall in supported cells.
3. Switch-on at cap 1.0, lap memory off: per direction 5 laps with the deployed model, then 5
   with the candidate. No contact, no more safety brakes than the deployed model had, mean
   wheel speed at least 0.98 times the deployed model's.
4. A cap above 1.3 only with a validity from GP2f-V, as v0.4 section 6.4 says, one rung at a
   time: 1.4 then 1.5 m/s, two runs per direction at each.

## 8. Not attempted in v0.5 dynamics

A refit of the wheel channel (it waits for the command pulse grid in vehicle_params), a
speed-channel learned term, a lateral slip output, RMA, online adaptation.
