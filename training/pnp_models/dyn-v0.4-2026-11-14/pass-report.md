# v0.4 dynamics: the held-out pass (GP2d and GP2e, one pass)

Freeze commit: `416e68501c424caeb1f6dc80b6a0ee9edc7a288c`.

- Split SHA-256 of 2026-11-14: 78510c1423278f80c42511e6caab095818598af6db6a284dd5479f74a03a1d63; addendum constants SHA-256: ada5a83675e5c843fb8e121a1190e4458f823b1ed2ff32a34a46c5fb4ff76824.
- Windows: 1281 on identical windows under T2 and T2s (0 only under T2, 0 only under T2s, dropped).
- Torch device of the learned fits: cpu, float64 (Apple MPS is available but has no float64 and the residual recipe is float64).
- Pp and Yp are a re-read of G1b and T2b (the only pose data), labelled as such.

## Comparator, identical windows, T2 unless the column says T2s

| model | P1 (m) | A1 (m) | C1 (m) | Y1 (rad) | S1 (m/s) | P1 under T2s (m) | Pp (m) | Yp (rad) |
|---|---|---|---|---|---|---|---|---|
| dyn-v0.1 | 0.0854 | 0.0636 | 0.0542 | 0.0728 | 0.0336 | 0.0868 | 0.0982 | 0.0257 |
| N4 | 0.0826 | 0.0661 | 0.0482 | 0.0476 | 0.0337 | 0.0842 | 0.0880 | 0.0238 |
| R4[dyn-v0.1] seed 0 | 0.0812 | 0.0657 | 0.0462 | 0.0405 | 0.0336 | 0.0828 | 0.0779 | 0.0172 |
| R4[dyn-v0.1] seed 1 | 0.0812 | 0.0664 | 0.0453 | 0.0413 | 0.0336 | 0.0828 | 0.0785 | 0.0175 |
| R4[dyn-v0.1] seed 2 | 0.0812 | 0.0654 | 0.0465 | 0.0399 | 0.0336 | 0.0828 | 0.0783 | 0.0182 |
| T4[dyn-v0.1] | 0.0535 | 0.0380 | 0.0369 | 0.0728 | 0.0336 | 0.0533 | 0.0795 | 0.0257 |
| TL4[dyn-v0.1] seed 0 | 0.0441 | 0.0277 | 0.0332 | 0.0455 | 0.0336 | 0.0442 | 0.0653 | 0.0207 |
| TL4[dyn-v0.1] seed 1 | 0.0443 | 0.0278 | 0.0335 | 0.0446 | 0.0336 | 0.0444 | 0.0629 | 0.0195 |
| TL4[dyn-v0.1] seed 2 | 0.0440 | 0.0275 | 0.0333 | 0.0452 | 0.0336 | 0.0442 | 0.0662 | 0.0208 |
| R4[N4] seed 0 | 0.0808 | 0.0658 | 0.0454 | 0.0397 | 0.0337 | 0.0824 | 0.0801 | 0.0185 |
| R4[N4] seed 1 | 0.0813 | 0.0660 | 0.0459 | 0.0396 | 0.0337 | 0.0829 | 0.0787 | 0.0189 |
| R4[N4] seed 2 | 0.0810 | 0.0652 | 0.0464 | 0.0400 | 0.0337 | 0.0826 | 0.0768 | 0.0187 |
| T4[N4] | 0.0480 | 0.0331 | 0.0337 | 0.0476 | 0.0337 | 0.0480 | 0.0720 | 0.0238 |
| TL4[N4] seed 0 | 0.0447 | 0.0280 | 0.0337 | 0.0456 | 0.0337 | 0.0448 | 0.0664 | 0.0208 |
| TL4[N4] seed 1 | 0.0454 | 0.0287 | 0.0341 | 0.0455 | 0.0337 | 0.0457 | 0.0675 | 0.0211 |
| TL4[N4] seed 2 | 0.0450 | 0.0280 | 0.0342 | 0.0460 | 0.0337 | 0.0453 | 0.0673 | 0.0207 |
| dyn-v0.3y | 0.0876 | 0.0550 | 0.0650 | 0.0916 | 0.0336 | 0.0891 | 0.0865 | 0.0270 |
| dyn-v0.3yr | 0.0838 | 0.0684 | 0.0462 | 0.0513 | 0.0336 | 0.0853 | 0.0755 | 0.0178 |

Shipping nominal (GP2d): dyn-v0.1.

### GP2d-N classical refit: FAIL

| item | value | threshold | result | note |
|---|---|---|---|---|
| 1a P1 ratio N4 / v0.1 | 0.968161 | <= 1 | PASS |  |
| 1b upper end of the bootstrap interval of the P1 ratio | 0.980003 | < 1 | PASS |  |
| 2a Y1 ratio N4 / v0.1 | 0.653842 | <= 1 | PASS |  |
| 2b S1 ratio N4 / v0.1 | 1.00256 | <= 1 | FAIL |  |
| 3 stratum ccw:0.25-0.6 position RMSE ratio | 0.995187 | <= 1.05 | PASS |  |
| 3 stratum ccw:0.6-0.85 position RMSE ratio | 1.01047 | <= 1.05 | PASS |  |
| 3 stratum ccw:0.85+ position RMSE ratio | 0.900398 | <= 1.05 | PASS |  |
| 3 stratum cw:0.25-0.6 position RMSE ratio | 0.958478 | <= 1.05 | PASS |  |
| 3 stratum cw:0.6-0.85 position RMSE ratio | 0.985764 | <= 1.05 | PASS |  |
| 3 stratum cw:0.85+ position RMSE ratio | 0.971264 | <= 1.05 | PASS |  |
| 4 refit parameters on a bound | 1 | <= 0 | FAIL | understeer_s2_per_m2 |
| 5 Yp ratio N4 / v0.1 | 0.925342 | <= 1 | PASS |  |

### GP2d-R yaw-only residual: FAIL

| item | value | threshold | result | note |
|---|---|---|---|---|
| 1a P1 ratio R / nominal | 0.951719 | <= 0.95 | FAIL |  |
| 1b upper end of the bootstrap interval of the P1 ratio | 0.964552 | < 1 | PASS |  |
| 2a Y1 ratio R / nominal | 0.548404 | <= 1 | PASS |  |
| 2b S1(R) minus S1(nominal) | 0 | <= 1e-06 | PASS | absolute difference in m/s |
| 3 seed 0 P1 ratio to nominal | 0.951438 | <= 1 | PASS |  |
| 3 seed 1 P1 ratio to nominal | 0.951343 | <= 1 | PASS |  |
| 3 seed 2 P1 ratio to nominal | 0.951719 | <= 1 | PASS |  |
| 4 stratum ccw:0.25-0.6 position RMSE ratio | 0.991184 | <= 1.05 | PASS |  |
| 4 stratum ccw:0.6-0.85 position RMSE ratio | 0.995744 | <= 1.05 | PASS |  |
| 4 stratum ccw:0.85+ position RMSE ratio | 0.851947 | <= 1.05 | PASS |  |
| 4 stratum cw:0.25-0.6 position RMSE ratio | 0.955615 | <= 1.05 | PASS |  |
| 4 stratum cw:0.6-0.85 position RMSE ratio | 0.974512 | <= 1.05 | PASS |  |
| 4 stratum cw:0.85+ position RMSE ratio | 0.957011 | <= 1.05 | PASS |  |
| 5 random sweep of the validity box: finite, inside the state bounds | 0 | <= 0 | PASS | 100000 rollouts of 30 steps; max v 1.300 m/s |
| 6 Yp ratio R / nominal | 0.707254 | <= 1 | PASS |  |

Named outcome (a label, deploys nothing): **improves heading, not position**.

### Heading outcome items: PASS

| item | value | threshold | result | note |
|---|---|---|---|---|
| Y1 ratio | 0.548404 | <= 0.8 | PASS |  |
| lateral position RMSE ratio at 1.0 s | 0.857393 | <= 0.9 | PASS |  |
| upper end of the bootstrap interval of the lateral ratio | 0.872051 | < 1 | PASS |  |
| P1 ratio | 0.951719 | <= 1 | PASS |  |
| stratum ccw:0.25-0.6 position RMSE ratio | 0.991184 | <= 1.05 | PASS |  |
| stratum ccw:0.6-0.85 position RMSE ratio | 0.995744 | <= 1.05 | PASS |  |
| stratum ccw:0.85+ position RMSE ratio | 0.851947 | <= 1.05 | PASS |  |
| stratum cw:0.25-0.6 position RMSE ratio | 0.955615 | <= 1.05 | PASS |  |
| stratum cw:0.6-0.85 position RMSE ratio | 0.974512 | <= 1.05 | PASS |  |
| stratum cw:0.85+ position RMSE ratio | 0.957011 | <= 1.05 | PASS |  |

Validity: v_max 1.3 m/s (pipeline rule 1.4); the bundle's model failed its gate: validity held at min(rule, 1.3).

- bundle model passed its gate: 0 >= 1 FAIL
- fast ccw: P1 ratio to dyn-v0.1 (36 windows): 1 <= 1 PASS
- fast cw: windows with start speed >= 1.15 m/s: 24 >= 30 FAIL the fast item cannot be applied

### GP2e-T ground motion table: FAIL

| item | value | threshold | result | note |
|---|---|---|---|---|
| 1a P1 ratio T4 / nominal | 0.62654 | <= 0.95 | PASS |  |
| 1b upper end of the bootstrap interval of the P1 ratio | 0.663672 | < 1 | PASS |  |
| 2a A1 ratio T4 / nominal | 0.597678 | <= 1 | PASS |  |
| 2b C1 ratio T4 / nominal | 0.680265 | <= 1 | PASS |  |
| 3a Y1(T4) minus Y1(nominal) | 0 | <= 1e-06 | PASS | absolute rad |
| 3b S1(T4) minus S1(nominal) | 0 | <= 1e-06 | PASS | absolute m/s |
| 4 stratum ccw:0.25-0.6 position RMSE ratio | 0.560937 | <= 1.05 | PASS |  |
| 4 stratum ccw:0.6-0.85 position RMSE ratio | 0.642149 | <= 1.05 | PASS |  |
| 4 stratum ccw:0.85+ position RMSE ratio | 0.763553 | <= 1.05 | PASS |  |
| 4 stratum cw:0.25-0.6 position RMSE ratio | 0.587841 | <= 1.05 | PASS |  |
| 4 stratum cw:0.6-0.85 position RMSE ratio | 0.560879 | <= 1.05 | PASS |  |
| 4 stratum cw:0.85+ position RMSE ratio | 0.630477 | <= 1.05 | PASS |  |
| 5 supported table nodes on a bound | 2 | <= 0 | FAIL | s+0.0000/v0.25, s+0.6667/v1.5 |
| 6 P1 ratio T4 / nominal under T2s | 0.613508 | <= 1 | PASS |  |
| 7 Pp ratio T4 / nominal | 0.80947 | <= 1 | PASS |  |

GP2e-L learned correction (not evaluated: GP2e-T failed): not evaluated.

GP2e outcome: **the one-speed contract stays**.

Validity of the contract 2 bundles:

T4: Validity: v_max 1.3 m/s (pipeline rule 1.4); the bundle's model failed its gate: validity held at min(rule, 1.3).

- bundle model passed its gate: 0 >= 1 FAIL
- fast ccw: P1 ratio to dyn-v0.1 (36 windows): 0.715384 <= 1 PASS
- fast cw: windows with start speed >= 1.15 m/s: 24 >= 30 FAIL the fast item cannot be applied

TL4: Validity: v_max 1.3 m/s (pipeline rule 1.4); the bundle's model failed its gate: validity held at min(rule, 1.3).

- bundle model passed its gate: 0 >= 1 FAIL
- fast ccw: P1 ratio to dyn-v0.1 (36 windows): 0.449833 <= 1 PASS
- fast cw: windows with start speed >= 1.15 m/s: 24 >= 30 FAIL the fast item cannot be applied

## The v0.2 held-out segments (read before, decide nothing)

| model | P1 (m) | A1 (m) | C1 (m) | Y1 (rad) | S1 (m/s) | P1 under T2 (no T2s for these segments) (m) | Pp (m) | Yp (rad) |
|---|---|---|---|---|---|---|---|---|
| dyn-v0.1 | 0.0887 | 0.0690 | 0.0540 | 0.0739 | 0.0387 | 0.0887 | nan | nan |
| N4 | 0.0869 | 0.0718 | 0.0478 | 0.0506 | 0.0387 | 0.0869 | nan | nan |
| R4[dyn-v0.1] seed 0 | 0.0859 | 0.0708 | 0.0468 | 0.0416 | 0.0387 | 0.0859 | nan | nan |
| R4[dyn-v0.1] seed 1 | 0.0857 | 0.0714 | 0.0458 | 0.0419 | 0.0387 | 0.0857 | nan | nan |
| R4[dyn-v0.1] seed 2 | 0.0858 | 0.0706 | 0.0469 | 0.0421 | 0.0387 | 0.0858 | nan | nan |
| T4[dyn-v0.1] | 0.0557 | 0.0377 | 0.0397 | 0.0739 | 0.0387 | 0.0557 | nan | nan |
| TL4[dyn-v0.1] seed 0 | 0.0495 | 0.0331 | 0.0350 | 0.0435 | 0.0387 | 0.0495 | nan | nan |
| TL4[dyn-v0.1] seed 1 | 0.0494 | 0.0329 | 0.0351 | 0.0425 | 0.0387 | 0.0494 | nan | nan |
| TL4[dyn-v0.1] seed 2 | 0.0495 | 0.0328 | 0.0353 | 0.0434 | 0.0387 | 0.0495 | nan | nan |
| R4[N4] seed 0 | 0.0855 | 0.0706 | 0.0465 | 0.0406 | 0.0387 | 0.0855 | nan | nan |
| R4[N4] seed 1 | 0.0855 | 0.0706 | 0.0466 | 0.0410 | 0.0387 | 0.0855 | nan | nan |
| R4[N4] seed 2 | 0.0857 | 0.0702 | 0.0474 | 0.0422 | 0.0387 | 0.0857 | nan | nan |
| T4[N4] | 0.0526 | 0.0365 | 0.0357 | 0.0506 | 0.0387 | 0.0526 | nan | nan |
| TL4[N4] seed 0 | 0.0500 | 0.0333 | 0.0353 | 0.0434 | 0.0387 | 0.0500 | nan | nan |
| TL4[N4] seed 1 | 0.0495 | 0.0324 | 0.0357 | 0.0433 | 0.0387 | 0.0495 | nan | nan |
| TL4[N4] seed 2 | 0.0497 | 0.0326 | 0.0357 | 0.0434 | 0.0387 | 0.0497 | nan | nan |
| dyn-v0.3y | 0.0904 | 0.0636 | 0.0619 | 0.0905 | 0.0387 | 0.0904 | nan | nan |
| dyn-v0.3yr | 0.0862 | 0.0713 | 0.0464 | 0.0451 | 0.0387 | 0.0862 | nan | nan |

Pp and Yp are not computed here (see the main table).

