do-not-deploy: GP2c-R (yaw-only residual) FAILED on the held-out segments under T2. The deployed model does not change; this table is the result.

# dyn-v0.3yr-2026-11-14

Candidate R: the MLP residual of the bundle contract with `output_scale: [0.0, s_yaw]`, trained on top of the nominal `dyn-v0.1`, truth T2, train segments only, config and shipping seed chosen on validation P1 and frozen before held-out was read. Shipping config `yaw_multi_step_from_one_wd1e-2`, seed 1. Split SHA-256 `bc5185c912ece67a6177581f1b37adaaa9101b34ee1efe9881af7723681aef83`.

## Addendum

- Base nominal decided before any residual was trained: GP2c-Y FAILED (pointer dyn-v0.3y-2026-11-14), so the residual is trained on top of dyn-v0.1.

## GP2c-R decision (held-out segments, truth T2)

| item | value | threshold | result |
|---|---|---|---|
| 1a P1 ratio R / nominal | 0.9718 | <= 0.95 | FAIL |
| 1b upper end of the bootstrap interval of the P1 ratio | 1.0194 | < 1 | FAIL |
| 2a Y1 ratio R / nominal | 0.6105 | <= 1 | pass |
| 2b S1(R) minus S1(nominal) | 0.0000 | <= 1e-06 | pass |
| 3 seed 0 P1 ratio to nominal | 0.9724 | <= 1 | pass |
| 3 seed 1 P1 ratio to nominal | 0.9718 | <= 1 | pass |
| 3 seed 2 P1 ratio to nominal | 0.9728 | <= 1 | pass |
| 4 stratum ccw:0.25-0.6 position RMSE ratio | 0.9918 | <= 1.05 | pass |
| 4 stratum ccw:0.6-0.85 position RMSE ratio | 1.0022 | <= 1.05 | pass |
| 4 stratum ccw:0.85+ position RMSE ratio | 0.8927 | <= 1.05 | pass |
| 4 stratum cw:0.25-0.6 position RMSE ratio | 0.9785 | <= 1.05 | pass |
| 4 stratum cw:0.6-0.85 position RMSE ratio | 0.9910 | <= 1.05 | pass |
| 4 stratum cw:0.85+ position RMSE ratio | 0.9812 | <= 1.05 | pass |
| 5 random sweep of the validity box: finite, inside the state bounds | 0.0000 | <= 0 | pass |
| 6 Yp ratio R / nominal | 0.6940 | <= 1 | pass |

Gate GP2c-R yaw-only residual: FAIL.

Paired block bootstrap (5.0 s blocks, 2000 resamples, seed 0), P1 ratio shipping seed over the nominal: 0.9718, 95 percent interval [0.9346, 1.0194].

Validation P1 per candidate (metres, macro, internal validation part of train):

| config | mean | min | seed 0 | seed 1 | seed 2 |
|---|---|---|---|---|---|
| yaw_one_step_wd1e-2 | 0.0717 | 0.0715 | 0.0715 | 0.0721 | 0.0717 |
| yaw_multi_step_wd1e-3 | 0.0711 | 0.0709 | 0.0710 | 0.0709 | 0.0713 |
| yaw_multi_step_wd1e-2 | 0.0712 | 0.0710 | 0.0710 | 0.0711 | 0.0713 |
| yaw_multi_step_from_one_wd1e-2 | 0.0710 | 0.0707 | 0.0712 | 0.0707 | 0.0712 |
| yaw_multi_step_capped_wd1e-2 | 0.0712 | 0.0711 | 0.0712 | 0.0711 | 0.0714 |
| yaw_multi_step_nonorm_wd1e-2 | 0.0720 | 0.0719 | 0.0719 | 0.0719 | 0.0722 |

## Truth T2 (primary, decides), per seed

Held-out windows 1479; strata after merging: ccw:0.25-0.6, ccw:0.6-0.85, ccw:0.85+, cw:0.25-0.6, cw:0.6-0.85, cw:0.85+. Merges: ccw 1.25+ (7 windows) into 0.85-1.25; cw 1.25+ (3 windows) into 0.85-1.25.

Macro RMSE at each horizon, position (m) / yaw (rad) / wheel speed (m/s), and Yp:

| model | 0.5 s | 1.0 s | 1.5 s | Yp (rad) |
|---|---|---|---|---|
| dyn-v0.1 | 0.044 / 0.039 / 0.037 | 0.089 / 0.074 / 0.039 | 0.133 / 0.105 / 0.038 | 0.0257 |
| R seed 0 | 0.044 / 0.028 / 0.037 | 0.086 / 0.046 / 0.039 | 0.126 / 0.060 / 0.038 | 0.0171 |
| R seed 1 | 0.044 / 0.028 / 0.037 | 0.086 / 0.045 / 0.039 | 0.126 / 0.059 / 0.038 | 0.0178 |
| R seed 2 | 0.044 / 0.027 / 0.037 | 0.086 / 0.044 / 0.039 | 0.126 / 0.057 / 0.038 | 0.0177 |

Per stratum position RMSE at 1.0 s (m), per seed:

| stratum | windows | dyn-v0.1 | R seed 0 | R seed 1 | R seed 2 |
|---|---|---|---|---|---|
| ccw:0.25-0.6 | 641 | 0.063 | 0.062 | 0.062 | 0.062 |
| ccw:0.6-0.85 | 93 | 0.061 | 0.061 | 0.061 | 0.061 |
| ccw:0.85+ | 59 | 0.087 | 0.077 | 0.077 | 0.078 |
| cw:0.25-0.6 | 527 | 0.077 | 0.075 | 0.075 | 0.075 |
| cw:0.6-0.85 | 92 | 0.094 | 0.093 | 0.093 | 0.093 |
| cw:0.85+ | 67 | 0.151 | 0.149 | 0.148 | 0.148 |

Yp per segment (rad, usable pose intervals):

- dyn-v0.1: G1b 0.0317 (185), T2b 0.0196 (87)
- R seed 0: G1b 0.0207 (185), T2b 0.0136 (87)
- R seed 1: G1b 0.0204 (185), T2b 0.0152 (87)
- R seed 2: G1b 0.0210 (185), T2b 0.0145 (87)

## Truth T1 (labelled: the v0.2 truth, decides nothing)

Held-out windows 1453; strata after merging: ccw:0.25-0.6, ccw:0.6-0.85, ccw:0.85+, cw:0.25-0.6, cw:0.6-0.85, cw:0.85+. Merges: ccw 1.25+ (1 windows) into 0.85-1.25; cw 1.25+ (3 windows) into 0.85-1.25.

Macro RMSE at each horizon, position (m) / yaw (rad) / wheel speed (m/s), and Yp:

| model | 0.5 s | 1.0 s | 1.5 s | Yp (rad) |
|---|---|---|---|---|
| dyn-v0.1 | 0.075 / 0.071 / 0.037 | 0.148 / 0.140 / 0.037 | 0.218 / 0.206 / 0.037 | 0.0257 |
| R seed 0 | 0.075 / 0.059 / 0.037 | 0.148 / 0.113 / 0.037 | 0.219 / 0.167 / 0.037 | 0.0171 |
| R seed 1 | 0.075 / 0.056 / 0.037 | 0.148 / 0.109 / 0.037 | 0.218 / 0.159 / 0.037 | 0.0178 |
| R seed 2 | 0.075 / 0.057 / 0.037 | 0.148 / 0.111 / 0.037 | 0.218 / 0.164 / 0.037 | 0.0177 |

Per stratum position RMSE at 1.0 s (m), per seed:

| stratum | windows | dyn-v0.1 | R seed 0 | R seed 1 | R seed 2 |
|---|---|---|---|---|---|
| ccw:0.25-0.6 | 641 | 0.104 | 0.104 | 0.104 | 0.103 |
| ccw:0.6-0.85 | 93 | 0.105 | 0.106 | 0.105 | 0.106 |
| ccw:0.85+ | 47 | 0.185 | 0.190 | 0.190 | 0.190 |
| cw:0.25-0.6 | 527 | 0.120 | 0.119 | 0.119 | 0.119 |
| cw:0.6-0.85 | 91 | 0.144 | 0.143 | 0.142 | 0.142 |
| cw:0.85+ | 54 | 0.227 | 0.227 | 0.227 | 0.226 |

Yp per segment (rad, usable pose intervals):

- dyn-v0.1: G1b 0.0317 (185), T2b 0.0196 (87)
- R seed 0: G1b 0.0207 (185), T2b 0.0136 (87)
- R seed 1: G1b 0.0204 (185), T2b 0.0152 (87)
- R seed 2: G1b 0.0210 (185), T2b 0.0145 (87)

do-not-deploy: the gate failed.
