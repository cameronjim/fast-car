# GP2g-R held-out pass: FAIL

Freeze commit: `3c4540c5875b6f717f09c6d5b84378d8f96297de`. Constants SHA-256 db62eaac18ba655bf00e4ff7c8e8b287cd7e042b6ea671014ebb404032d00e0c; session file SHA-256 6b791ac5bcc5c0b150b86420574f4d704453963e9d1fdb1502aa09f0447516f5.

Windows: 525 on identical windows under T2 and T2s (0 only under T2, 0 only under T2s, dropped); per direction {'ccw': 285, 'cw': 240}; strata {'ccw:0.25-0.6': 8, 'ccw:0.6-0.85': 37, 'ccw:0.85-1.25': 240, 'cw:0.25-0.6': 46, 'cw:0.6-1.25': 194}.

Shipping: yaw_multi_step_from_one_wd1e-2 seed 2 (R6 seed 2).

## Item 0 enough data

| item | value | threshold | result | note |
|---|---|---|---|---|
| 0a windows | 525 | >= 400 | PASS |  |
| 0b windows ccw | 285 | >= 150 | PASS |  |
| 0b windows cw | 240 | >= 150 | PASS |  |
| 0c strata with 30 windows | 4 | >= 4 | PASS |  |

## GP2g-R items

| item | value | threshold | result | note |
|---|---|---|---|---|
| 1a P1 ratio R6 / dyn-v0.1 | 0.992888 | <= 0.95 | FAIL |  |
| 1b upper end of the bootstrap interval of the P1 ratio | 1.00087 | < 1 | FAIL |  |
| 2a Y1 ratio | 0.960876 | <= 0.9 | FAIL |  |
| 2b S1(R6) minus S1(dyn-v0.1) | 0 | <= 1e-06 | PASS | absolute difference in m/s |
| 3 seed 0 P1 ratio | 0.993122 | <= 1 | PASS |  |
| 3 seed 1 P1 ratio | 0.996814 | <= 1 | PASS |  |
| 3 seed 2 P1 ratio | 0.992888 | <= 1 | PASS |  |
| 4 stratum ccw:0.25-0.6 position RMSE ratio | 8 | n/a 30 | n/a | cannot be applied: 8 windows, at least 30 needed |
| 4 stratum ccw:0.6-0.85 position RMSE ratio (37 windows) | 0.962518 | <= 1.05 | PASS |  |
| 4 stratum ccw:0.85-1.25 position RMSE ratio (240 windows) | 1.02354 | <= 1.05 | PASS |  |
| 4 stratum cw:0.25-0.6 position RMSE ratio (46 windows) | 0.993557 | <= 1.05 | PASS |  |
| 4 stratum cw:0.6-1.25 position RMSE ratio (194 windows) | 0.998197 | <= 1.05 | PASS |  |
| 5 random sweep of the validity box: finite, inside the state bounds | 0 | <= 0 | PASS | 100000 rollouts of 30 steps; max v 1.300 m/s |
| 6 Yp ratio (G1b, T2b; a re-read) | 0.705215 | <= 1 | PASS |  |
| 7 ccw P1 ratio | 0.99186 | <= 1 | PASS |  |
| 7 ccw Y1 ratio | 1.01561 | <= 1 | FAIL |  |
| 7 cw P1 ratio | 0.9961 | <= 1 | PASS |  |
| 7 cw Y1 ratio | 0.721789 | <= 1 | PASS |  |
| 8a C1 ratio | 1.04609 | <= 1 | FAIL |  |
| 8b A1 ratio | 0.981128 | <= 1.02 | PASS |  |
| 9 near-limit left position RMSE ratio | 0 | n/a 30 | n/a | cannot be applied: 0 windows, at least 30 needed |
| 9 near-limit right position RMSE ratio (163 windows) | 0.990581 | <= 1 | PASS |  |
| 9 transition position RMSE ratio (78 windows) | 1.0013 | <= 1 | FAIL |  |
| 10 P1 ratio under T2s | 0.993308 | <= 1 | PASS |  |
| 11 share of held-out steps with the residual output at or above 0.9 of its bound | 0 | <= 0.01 | PASS |  |
| 12 mppi_bench p50 solve time ratio R6 / dyn-v0.1 | 13.8457 | <= 1.25 | FAIL | 7.449 ms against 0.538 ms. mppi_bench 'planner solve()' p50, default settings (512 samples, fast MLP off), Linux ros-dev:ci container on the Mac, Release build, median of 3 interleaved runs of 1000 solves (R6 7.395, 7.449, 8.071; dyn-v0.1 0.580, 0.530, 0.538). Every scene is 13 to 15 times. For information only: with --fast-mlp R6 4.438 ms against 0.572 ms (7.8 times). Bench copy of the frozen weights, cache/v06-bench. |
| 13 L5 PnP canary with the R6 bundle: every case completes, 0 wall contacts | 0 | <= 0 | PASS | 22 of 22 cases completed, 0 wall contacts. tests/l5_pnp unedited, run by .github/scripts/l5_pnp_test.sh in ros-dev:ci with the R6 bundle bind-mounted read-only over config/pnp/fixtures/dyn-car-2026-11-12 (the planner bundle the canary loads; it is dyn-v0.1's nominal with the residual off). 22 passed, 36 skipped, 0 failed, 0 wall contacts; one starvation rerun of the no-perception case (attempt 3 passed). The 2 contract 2 cases load their own bundle, not R6. |

Named outcome (not a deployment rule): no named outcome.

## Metrics at 1.0 s (macro over strata)

| model | P1 (m) | A1 (m) | C1 (m) | Y1 (rad) | S1 (m/s) | P1 under T2s (m) | Yp (rad) |
|---|---|---|---|---|---|---|---|
| dyn-v0.1 | 0.1632 | 0.1395 | 0.0825 | 0.1847 | 0.1592 | 0.1637 | 0.0257 |
| R6 seed 0 | 0.1621 | 0.1371 | 0.0859 | 0.1738 | 0.1592 | 0.1626 | 0.0179 |
| R6 seed 1 | 0.1627 | 0.1378 | 0.0858 | 0.1794 | 0.1592 | 0.1632 | 0.0190 |
| R6 seed 2 | 0.1620 | 0.1369 | 0.0863 | 0.1775 | 0.1592 | 0.1626 | 0.0181 |

P1 ratio bootstrap: 0.9929 (0.9534 to 1.0009).
Saturation: 0.0000% of 15750 held-out steps; bound [0.0, 1.286968353759187], largest |output| [0.0, 0.5785784509774958].

## Reported only: v0.2 split held-out segments under T2 (deciding nothing)

| model | P1 | Y1 | S1 |
|---|---|---|---|
| dyn-v0.1 | 0.0887 | 0.0739 | 0.0387 |
| R6 seed 0 | 0.0860 | 0.0399 | 0.0387 |
| R6 seed 1 | 0.0854 | 0.0403 | 0.0387 |
| R6 seed 2 | 0.0858 | 0.0407 | 0.0387 |

