Deployment verdict: the deployed model stays dyn-v0.1 with the residual disabled (GP2a failed on top of dyn-v0.1)

# dyn-v0.1r-2026-11-14

dyn-v0.1 nominal parameters (repo copy, vehicle_params 0.17.0) with a residual trained on top of THEM, train segments only, early stopping and selection of the config and seed on validation P1 only. Split SHA-256 `bc5185c912ece67a6177581f1b37adaaa9101b34ee1efe9881af7723681aef83`.

## Addendum

Addendum 2026-11-13, written before stage 2 scoring. Segment T3b (bag 2026-11-13T23-27-23, 1312.1 to 1347.6 s) has no driving: the car is at rest there (speed command 0, wheel speed 0), so it contributes no windows. Held-out driving is 403 s, not the 441 s of the pre-registration. Held-out windows (1453 in all, counted before any model was scored): ccw 641 / 93 / 46 / 1 and cw 527 / 91 / 51 / 3 in the bins 0.25-0.6 / 0.6-0.85 / 0.85-1.25 / 1.25+. The 1.25+ bins are merged into 0.85-1.25 as the rule says (ccw 1 window, cw 3 windows), so no stratum above 1.25 m/s exists: the fast held-out data is too thin to decide anything about speeds above 1.25 m/s. Strata after merging: ccw 0.25-0.6, 0.6-0.85, 0.85+; cw 0.25-0.6, 0.6-0.85, 0.85+. The pre-registration and the split file are unchanged.

Addendum 2 (2026-11-14), written before any scoring of the strict residual: 3.2 nominal ambiguity resolved strictly: the residual is evaluated on top of the deployed nominal dyn-v0.1 because 3.1 failed; the residual on v0.2 is reported for information only, it cannot ship.

## 3.2 GP2a residual on dyn-v0.1 (held-out segments)

Shipping config `multi_step_wd1e-2`, seed 1 (lowest validation P1, frozen before held-out was read).

| item | value | threshold | result |
|---|---|---|---|
| 1a P1 ratio residual / nominal | 0.9219 | <= 0.95 | pass |
| 1b upper end of the bootstrap interval of the P1 ratio | 0.9397 | < 1 | pass |
| 2a Y1 ratio residual / nominal | 0.4026 | <= 1 | pass |
| 2b S1 ratio residual / nominal | 1.0894 | <= 1.02 | FAIL |
| 3 seed 0 P1 ratio to nominal | 0.8610 | <= 1 | pass |
| 3 seed 1 P1 ratio to nominal | 0.9219 | <= 1 | pass |
| 3 seed 2 P1 ratio to nominal | 0.9152 | <= 1 | pass |
| 4 stratum ccw:0.25-0.6 position RMSE ratio | 0.7815 | <= 1.05 | pass |
| 4 stratum ccw:0.6-0.85 position RMSE ratio | 1.1558 | <= 1.05 | FAIL |
| 4 stratum ccw:0.85+ position RMSE ratio | 0.9266 | <= 1.05 | pass |
| 4 stratum cw:0.25-0.6 position RMSE ratio | 0.8478 | <= 1.05 | pass |
| 4 stratum cw:0.6-0.85 position RMSE ratio | 0.8217 | <= 1.05 | pass |
| 4 stratum cw:0.85+ position RMSE ratio | 0.9770 | <= 1.05 | pass |
| 5 random sweep of the validity box: finite, inside the state bounds | 0.0000 | <= 0 | pass |

Gate 3.2 GP2a residual: FAIL.

Paired block bootstrap (5.0 s blocks, 2000 resamples, seed 0), P1 ratio shipping residual over dyn-v0.1: 0.9219, 95 percent interval [0.8961, 0.9397].

Macro RMSE at each horizon, position (m) / yaw (rad) / wheel speed (m/s):

| model | 0.5 s | 1.0 s | 1.5 s |
|---|---|---|---|
| dyn-v0.1 nominal (deployed) | 0.075 / 0.071 / 0.037 | 0.148 / 0.140 / 0.037 | 0.218 / 0.206 / 0.037 |
| dyn-v0.1 + residual seed 0 | 0.067 / 0.033 / 0.052 | 0.127 / 0.064 / 0.056 | 0.187 / 0.095 / 0.058 |
| dyn-v0.1 + residual seed 1 | 0.071 / 0.033 / 0.040 | 0.136 / 0.056 / 0.040 | 0.198 / 0.077 / 0.042 |
| dyn-v0.1 + residual seed 2 | 0.070 / 0.033 / 0.039 | 0.135 / 0.057 / 0.039 | 0.197 / 0.078 / 0.041 |

Validation P1 per candidate (metres, macro):

| config | mean | min | seed 0 | seed 1 | seed 2 |
|---|---|---|---|---|---|
| one_step_wd1e-2 | 0.1224 | 0.1197 | 0.1197 | 0.1237 | 0.1238 |
| multi_step_wd1e-3 | 0.1150 | 0.1144 | 0.1144 | 0.1152 | 0.1153 |
| multi_step_wd1e-2 | 0.1143 | 0.1138 | 0.1143 | 0.1138 | 0.1147 |
| multi_step_from_one_wd1e-2 | 0.1148 | 0.1135 | 0.1156 | 0.1153 | 0.1135 |
| multi_step_capped_wd1e-2 | 0.1152 | 0.1138 | 0.1157 | 0.1138 | 0.1160 |
| multi_step_nonorm_wd1e-2 | 0.1161 | 0.1148 | 0.1167 | 0.1169 | 0.1148 |

Per stratum position RMSE at 1.0 s (m):

| stratum | windows | dyn-v0.1 (reference) | dyn-v0.1 nominal (deployed) | dyn-v0.1 + residual seed 0 | dyn-v0.1 + residual seed 1 | dyn-v0.1 + residual seed 2 |
|---|---|---|---|---|---|---|
| ccw:0.25-0.6 | 641 | 0.104 | 0.104 | 0.082 | 0.081 | 0.082 |
| ccw:0.6-0.85 | 93 | 0.105 | 0.105 | 0.107 | 0.121 | 0.101 |
| ccw:0.85+ | 47 | 0.185 | 0.185 | 0.173 | 0.172 | 0.186 |
| cw:0.25-0.6 | 527 | 0.120 | 0.120 | 0.101 | 0.102 | 0.100 |
| cw:0.6-0.85 | 91 | 0.144 | 0.144 | 0.079 | 0.118 | 0.120 |
| cw:0.85+ | 54 | 0.227 | 0.227 | 0.220 | 0.222 | 0.221 |

Strata merges: ccw 1.25+ (1 windows) into 0.85-1.25; cw 1.25+ (3 windows) into 0.85-1.25.

## Information only: the residual on the v0.2 nominal

Reported in training/pnp_models/dyn-v0.2-2026-11-14: 3.1 failed, so that nominal cannot ship and neither can a residual on it. Its GP2a numbers: P1 ratio 0.9388, bootstrap upper end 0.9632, passed GP2a on its own nominal.
