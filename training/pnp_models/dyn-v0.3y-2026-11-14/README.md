do-not-deploy: GP2c-Y (yaw refit) FAILED on the held-out segments under T2. The deployed model does not change; this table is the result.

# dyn-v0.3y-2026-11-14

Candidate Y: refit of the four yaw parameters of dyn-v0.1 (`steer_gain_left`, `steer_gain_right`, `steer_offset_rad`, `understeer_s2_per_m2`), every other parameter held at its dyn-v0.1 value (read from the dyn-v0.1 manifest). Train role of the split plus the legacy train part with a T2 truth (the two October bags have no /odom/wheel and so no T2: they are not in the fit). Truth T2 (wheel seeded ICP). Pre-registration docs/notes/preregistration-pnp-v0.3.md section 2.3, constants `evaluation/prereg/pnp-v0.3.yaml`, split SHA-256 `bc5185c912ece67a6177581f1b37adaaa9101b34ee1efe9881af7723681aef83`.

vehicle_params 0.17.0 / none-preliminary.

## Refit parameters

| parameter | dyn-v0.1 | Y | bounds | on a bound |
|---|---|---|---|---|
| steer_gain_left | 0.9166 | 0.9154 | [0.5, 1.5] | no |
| steer_gain_right | 1.0115 | 1.0459 | [0.5, 1.5] | no |
| steer_offset_rad | 0.0152 | 0.0228 | [-0.1, 0.1] | no |
| understeer_s2_per_m2 | 0.1094 | 0.0422 | [0, 2] | no |

Unidentified (on a bound or a flat profile): none.

Profiles (validation yaw RMSE per 0.1 s interval against each parameter, the other three refit) are in `profiles.png`. Validation yaw RMSE: dyn-v0.1 0.0075, Y 0.0073 rad. Intervals: train 13351, validation 1985.

## GP2c-Y decision (held-out segments, truth T2)

| item | value | threshold | result |
|---|---|---|---|
| 1a P1 ratio Y / v0.1 | 1.0191 | <= 1 | FAIL |
| 1b upper end of the bootstrap interval of the P1 ratio | 1.0899 | < 1 | FAIL |
| 2 Y1 ratio Y / v0.1 | 1.2243 | <= 1 | FAIL |
| 3 stratum ccw:0.25-0.6 position RMSE ratio | 1.0038 | <= 1.05 | pass |
| 3 stratum ccw:0.6-0.85 position RMSE ratio | 1.0264 | <= 1.05 | pass |
| 3 stratum ccw:0.85+ position RMSE ratio | 0.9764 | <= 1.05 | pass |
| 3 stratum cw:0.25-0.6 position RMSE ratio | 1.0075 | <= 1.05 | pass |
| 3 stratum cw:0.6-0.85 position RMSE ratio | 1.0261 | <= 1.05 | pass |
| 3 stratum cw:0.85+ position RMSE ratio | 1.0486 | <= 1.05 | pass |
| 4 Yp ratio Y / v0.1 | 1.0530 | <= 1 | FAIL |
| 5 S1(Y) minus S1(v0.1) | 0.0000 | <= 1e-06 | pass |
| 6 refit parameters on a bound | 0.0000 | <= 0 | pass |

Gate GP2c-Y yaw refit: FAIL.

Paired block bootstrap (5.0 s blocks, 2000 resamples, seed 0), P1 ratio Y over dyn-v0.1: 1.0191, 95 percent interval [0.9833, 1.0899].

## Truth T2 (primary, decides)

Held-out windows 1479; strata after merging: ccw:0.25-0.6, ccw:0.6-0.85, ccw:0.85+, cw:0.25-0.6, cw:0.6-0.85, cw:0.85+. Merges: ccw 1.25+ (7 windows) into 0.85-1.25; cw 1.25+ (3 windows) into 0.85-1.25.

Macro RMSE at each horizon, position (m) / yaw (rad) / wheel speed (m/s), and Yp:

| model | 0.5 s | 1.0 s | 1.5 s | Yp (rad) |
|---|---|---|---|---|
| dyn-v0.1 | 0.044 / 0.039 / 0.037 | 0.089 / 0.074 / 0.039 | 0.133 / 0.105 / 0.038 | 0.0257 |
| Y | 0.044 / 0.048 / 0.037 | 0.090 / 0.091 / 0.039 | 0.136 / 0.129 / 0.038 | 0.0270 |

Per stratum position RMSE at 1.0 s (m):

| stratum | windows | dyn-v0.1 | Y |
|---|---|---|---|
| ccw:0.25-0.6 | 641 | 0.063 | 0.063 |
| ccw:0.6-0.85 | 93 | 0.061 | 0.062 |
| ccw:0.85+ | 59 | 0.087 | 0.084 |
| cw:0.25-0.6 | 527 | 0.077 | 0.077 |
| cw:0.6-0.85 | 92 | 0.094 | 0.097 |
| cw:0.85+ | 67 | 0.151 | 0.158 |

Yp per segment (rad, usable pose intervals):

- dyn-v0.1: G1b 0.0317 (185), T2b 0.0196 (87)
- Y: G1b 0.0367 (185), T2b 0.0174 (87)

## Truth T1 (labelled: the v0.2 truth, decides nothing)

Held-out windows 1453; strata after merging: ccw:0.25-0.6, ccw:0.6-0.85, ccw:0.85+, cw:0.25-0.6, cw:0.6-0.85, cw:0.85+. Merges: ccw 1.25+ (1 windows) into 0.85-1.25; cw 1.25+ (3 windows) into 0.85-1.25.

Macro RMSE at each horizon, position (m) / yaw (rad) / wheel speed (m/s), and Yp:

| model | 0.5 s | 1.0 s | 1.5 s | Yp (rad) |
|---|---|---|---|---|
| dyn-v0.1 | 0.075 / 0.071 / 0.037 | 0.148 / 0.140 / 0.037 | 0.218 / 0.206 / 0.037 | 0.0257 |
| Y | 0.075 / 0.091 / 0.037 | 0.151 / 0.179 / 0.037 | 0.228 / 0.267 / 0.037 | 0.0270 |

Per stratum position RMSE at 1.0 s (m):

| stratum | windows | dyn-v0.1 | Y |
|---|---|---|---|
| ccw:0.25-0.6 | 641 | 0.104 | 0.104 |
| ccw:0.6-0.85 | 93 | 0.105 | 0.108 |
| ccw:0.85+ | 47 | 0.185 | 0.195 |
| cw:0.25-0.6 | 527 | 0.120 | 0.121 |
| cw:0.6-0.85 | 91 | 0.144 | 0.145 |
| cw:0.85+ | 54 | 0.227 | 0.231 |

Yp per segment (rad, usable pose intervals):

- dyn-v0.1: G1b 0.0317 (185), T2b 0.0196 (87)
- Y: G1b 0.0367 (185), T2b 0.0174 (87)

do-not-deploy: the gate failed.
