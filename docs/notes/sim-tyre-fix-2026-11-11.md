# Sim tyre units fix, 2026-11-11

## The bug

`racer_gym/params.py` synthesised the placeholder Pacejka tyre with small-slip slope equal to
f1tenth_gym's `C_Sf = 4.7` / `C_Sr = 5.5`, as if they were N/rad. They are dimensionless: stock
`vehicle_dynamics_st` computes `F_y = mu * C_S * Fz * alpha`. The slope in N/rad is
`mu * C_S * Fz_static`, 94 N/rad front and 101 N/rad rear for this car (mu 1.0489, 3.74 kg), about
20x the value used. Result: 1.8 m rear-axle radius at full lock and 2 m/s.

## Fix

- `params.py`: `gym_cornering_stiffness_n_per_rad(C_S, mu, Fz)`; the placeholder still reads the
  gym's own defaults programmatically. Still a placeholder (a different, larger car's tyre), now
  unit-correct. No `vehicle_params` change: this PR does not touch any field PR 118 changes, and
  the schema version is not bumped.
- **Second defect, exposed by the first.** With realistic stiffness the tyre modes relax at about
  1/V, so fixed-step RK4 at dt = 0.01 s is unstable below about 0.4 m/s (it does not blow up, the
  force saturates; it settles into a wrong state: 0.57 m instead of 0.60 m at 0.3 m/s, 1.07 m at
  0.1 m/s). `RacerRK4Integrator` now takes stiffness-aware substeps (1 above about 1.7 m/s, so no
  cost at racing speed; at most 64). A 0.001 s reference agrees to 1%.

## Before and after (rear-axle radius, full lock, steady state)

| | 0.3 m/s left (meas. 0.60) | 0.3 m/s right (meas. 0.69) | 2 m/s left | 2 m/s right |
|---|---|---|---|---|
| before | 0.64 | 0.73 | 1.76 | 1.85 |
| after | 0.60 | 0.69 | 0.70 | 0.78 |
| kinematic L/tan(delta) | 0.599 | 0.685 | | |

The 0.3 m/s car measurement pins geometry only: at crawl speed the radius is kinematic and
independent of tyre stiffness (the old tyre was already near it at 0.3 m/s). Stiffness is
constrained by nothing measured yet; it shows up as understeer at speed (Phase 3).

## Tests

- `sim/racer_gym/tests/test_low_speed_turning.py`: radius within 0.04 m (left) / 0.05 m (right)
  of 0.60 / 0.69, which is half the measured p25 to p75 (0.56 to 0.64, 0.64 to 0.74), using the
  effective angles 0.504 / 0.449 on the 0.3302 m wheelbase (steering limit widened on an in-memory
  copy); within 2% of kinematic at 0.1 and 0.3 m/s; understeer grows with speed and is under 10%
  at 1 m/s (old: 61%); stiffness equals `mu*C_S*Fz`; small-slip accelerations agree with stock
  `vehicle_dynamics_st` within 3% (was a factor of about 20); step-size independence; substep logic.
  Verified to fail on the old unit (9 of 15 fail). Note the measurement pin alone barely bites
  (old left radius 0.641 vs band edge 0.64); the 2% kinematic and stock-gym tests are the ones that do.
- `tests/sim_regression/tests/test_low_speed_turning_env.py`: same pin through the full env (lag,
  delay, bang-bang steering). Kinematic check is 5% there because `pid_steer` chatters one rate
  step (0.032 rad).
- `test_params_fallback.py`: the old test asserted `B*C*D == C_S` and encoded the bug; it now
  asserts `mu*C_S*Fz`. Not a weakened tolerance, a corrected statement.

## S.6 references regenerated (deliberate, owner asked for it)

`steering_step` and `constant_radius_circle` (the only tyre-dependent maneuvers) were
regenerated; `throttle_step` and `coastdown` pass unchanged and were left alone. The reason is
stored in each file's `meta.regeneration_reason` (`regenerate_references.py` now records it).
Examples: steering step at 3 m/s, 0.15 rad: yaw rate 0.40 to 1.20 rad/s (kinematic 1.37), overshoot
70% to 0.4%. No tolerance was changed.

## v0 policy re-evaluation (no retraining), 120 paired episodes

| Condition | Completion tracker / +residual | Paired lap time |
|---|---|---|
| held-out family | 114 / 114 of 120 (was 115 / 113) | -1.42% (was -1.43%) |
| unseen shapes | 120 / 120 (same) | -0.60% (was -0.61%) |
| x1.5 widened | 100 / 90 (was 100 / 91) | -1.24% (was -1.21%) |
| nominal | 120 / 120 (same) | -1.68% (was -1.64%) |

Conclusion unchanged: about 1% faster, no completion gain, worse out of range. Expected, because v0
randomised stiffness over 80 to 400 N/rad per episode (which already contained the corrected value);
only the nominal draw and the low-speed integration changed. Numbers are not a claim about the car.

## Recommended stiffness randomisation

Front 50 to 200 N/rad (about 0.5x to 2x the 94 N/rad placeholder), rear ratio 0.8 to 1.3 unchanged
(placeholder 1.07). At full lock and 2 m/s that spans R/R_kin of 1.09 to 1.30 (0.5x: 1.30, 1x: 1.17,
2x: 1.09; 4x, i.e. the old 400 end: 1.05, near-kinematic and the stiffest, slowest-to-integrate case).
Not applied here: it changes the training distribution, so it belongs with the retrain (roadmap
5.1); the integrator handles the old range too (tested at 4x). Phase 3 should replace it with fit residuals.

## Follow-up (needs PR 118 first)

`vehicle_params.yaml` `tires.linear_cornering_stiffness_*_n_per_rad` hold the gym's dimensionless
4.718 / 5.4562 under an N/rad name, and the schema caps them at 50 N/rad, so correct values do not
validate. Nothing consumes them. Fix with a schema bump after 118 merges (or drop the fields when
the Pacejka fit lands). Kept out of this PR to avoid the 0.13.0 to 0.14.0 conflict and the v0
contract refusal.

## Merge note

Main gained PR 118 (vehicle_params 0.14.0) while this was open. The v0 re-evaluation above ran on
0.13.0, before that merge; the v0 `policy.pt` contract names 0.13.0 and the loader will now refuse
it against main, as designed. `test_overrides_apply_to_a_copy` hard-coded the old committed
steering limit (0.4189) and failed on merged main; it now compares against the committed value
read from the binding (the override must still differ from it).
