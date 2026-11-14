# M-B1: dyn-v0.1 and the v0.2 residual dyn-v0.1r, rescored (before any v0.3 fit)

Pre-registration v0.3 section 2.2. Held-out segments of the v0.2 split, windows, strata, merging, macro averaging and horizons exactly as v0.2 section 3. Truth T2 (wheel seeded ICP) is primary; T1 (identity seeded ICP, the v0.2 truth) is printed labelled and decides nothing. No v0.3 model existed when this table was produced.

- dyn-v0.1 nominal read from the committed pointer manifest training/pnp_models/dyn-v0.1-2026-11-12/manifest.yaml, residual disabled.
- The dyn-v0.1r bundle was never exported (it failed GP2a, v0.2 exported passing bundles only); the same weights are read from the frozen state car-data/pnp/cache/dyn-v0.1r-frozen.pkl (config multi_step_wd1e-2, seed 1, trained on the dyn-v0.1 nominal).
- Held-out windows differ between T1 and T2 (a failed ICP interval drops a window), so the T1 block is not the v0.2 table over again; it is the v0.2 rule on the current code.

Cells: macro position RMSE (m) / yaw RMSE (rad) / wheel speed RMSE (m/s). Yp: RMS of (model yaw change minus /pose yaw change) over consecutive pose pairs, mean of G1b and T2b.

## Truth T2 (primary)

Held-out windows: 1479. Strata after merging: ccw:0.25-0.6, ccw:0.6-0.85, ccw:0.85+, cw:0.25-0.6, cw:0.6-0.85, cw:0.85+.

| model | 0.5 s | 1.0 s | 1.5 s | Yp (rad) |
|---|---|---|---|---|
| dyn-v0.1 | 0.044 / 0.039 / 0.037 | 0.089 / 0.074 / 0.039 | 0.133 / 0.105 / 0.038 | 0.0257 |
| dyn-v0.1r (v0.2 residual, seed 1) | 0.041 / 0.052 / 0.040 | 0.085 / 0.102 / 0.044 | 0.139 / 0.154 / 0.044 | 0.0383 |

Ratios dyn-v0.1r (v0.2 residual, seed 1) / dyn-v0.1: P1 0.9593, Y1 1.3821, S1 1.1444.

Per stratum position RMSE at 1.0 s (m):

| stratum | windows | dyn-v0.1 | dyn-v0.1r (v0.2 residual, seed 1) |
|---|---|---|---|
| ccw:0.25-0.6 | 641 | 0.063 | 0.042 |
| ccw:0.6-0.85 | 93 | 0.061 | 0.077 |
| ccw:0.85+ | 59 | 0.087 | 0.110 |
| cw:0.25-0.6 | 527 | 0.077 | 0.059 |
| cw:0.6-0.85 | 92 | 0.094 | 0.072 |
| cw:0.85+ | 67 | 0.151 | 0.150 |

Strata merges: ccw 1.25+ (7 windows) into 0.85-1.25; cw 1.25+ (3 windows) into 0.85-1.25.

Yp per segment (rad, usable pose intervals):

- dyn-v0.1: G1b 0.0317 (185), T2b 0.0196 (87)
- dyn-v0.1r (v0.2 residual, seed 1): G1b 0.0503 (185), T2b 0.0263 (87)

## Truth T1 (labelled, v0.2 truth, decides nothing)

Held-out windows: 1453. Strata after merging: ccw:0.25-0.6, ccw:0.6-0.85, ccw:0.85+, cw:0.25-0.6, cw:0.6-0.85, cw:0.85+.

| model | 0.5 s | 1.0 s | 1.5 s | Yp (rad) |
|---|---|---|---|---|
| dyn-v0.1 | 0.075 / 0.071 / 0.037 | 0.148 / 0.140 / 0.037 | 0.218 / 0.206 / 0.037 | 0.0257 |
| dyn-v0.1r (v0.2 residual, seed 1) | 0.071 / 0.033 / 0.040 | 0.136 / 0.056 / 0.040 | 0.198 / 0.077 / 0.042 | 0.0383 |

Ratios dyn-v0.1r (v0.2 residual, seed 1) / dyn-v0.1: P1 0.9219, Y1 0.4026, S1 1.0894.

Per stratum position RMSE at 1.0 s (m):

| stratum | windows | dyn-v0.1 | dyn-v0.1r (v0.2 residual, seed 1) |
|---|---|---|---|
| ccw:0.25-0.6 | 641 | 0.104 | 0.081 |
| ccw:0.6-0.85 | 93 | 0.105 | 0.121 |
| ccw:0.85+ | 47 | 0.185 | 0.172 |
| cw:0.25-0.6 | 527 | 0.120 | 0.102 |
| cw:0.6-0.85 | 91 | 0.144 | 0.118 |
| cw:0.85+ | 54 | 0.227 | 0.222 |

Strata merges: ccw 1.25+ (1 windows) into 0.85-1.25; cw 1.25+ (3 windows) into 0.85-1.25.

Yp per segment (rad, usable pose intervals):

- dyn-v0.1: G1b 0.0317 (185), T2b 0.0196 (87)
- dyn-v0.1r (v0.2 residual, seed 1): G1b 0.0503 (185), T2b 0.0263 (87)
