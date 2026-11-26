# dyn-v0.9-2026-11-25 pass against dyn-v0.7p: FAIL

Freeze commit `1321fcd5bd4081a5161dff9b0daa8d8dd44539aa`; rule SHA-256 24eb652eafcc2f81f6ac1a33a421beb74bd29ea1b449f37552d38f955a94767a.

## H1: GP2g-R held-out parts of 2026-11-22 (decides)

Windows: 525 (identical under T2 and T2s); per direction {'ccw': 285, 'cw': 240}; strata {'ccw:0.25-0.6': 8, 'ccw:0.6-0.85': 37, 'ccw:0.85-1.25': 240, 'cw:0.25-0.6': 46, 'cw:0.6-1.25': 194}.

Shipping: multi_step_from_one_capped_wd1e-2_gs seed 0 (hidden 32).

| item | value | threshold | result | note |
|---|---|---|---|---|
| 0a windows | 525 | >= 400 | PASS |  |
| 0b windows ccw | 285 | >= 150 | PASS |  |
| 0b windows cw | 240 | >= 150 | PASS |  |
| 0c strata with 30 windows | 4 | >= 4 | PASS |  |

| item | value | threshold | result | note |
|---|---|---|---|---|
| 1a P1 ratio / dyn-v0.7p | 0.860064 | <= 0.95 | PASS |  |
| 1b upper end of the bootstrap interval of the P1 ratio | 0.900262 | < 1 | PASS |  |
| 2a Y1 ratio | 1.04089 | <= 0.9 | FAIL |  |
| 2b S1 ratio (replaces GP2g-R's equality) | 0.978735 | <= 1 | PASS |  |
| 3 seed 0 P1 ratio | 0.860064 | <= 1 | PASS |  |
| 3 seed 1 P1 ratio | 0.8686 | <= 1 | PASS |  |
| 3 seed 2 P1 ratio | 0.85897 | <= 1 | PASS |  |
| 4 stratum ccw:0.25-0.6 position RMSE ratio | 8 | n/a 30 | n/a | cannot be applied: 8 windows, at least 30 needed |
| 4 stratum ccw:0.6-0.85 position RMSE ratio (37 windows) | 0.719942 | <= 1.05 | PASS |  |
| 4 stratum ccw:0.85-1.25 position RMSE ratio (240 windows) | 0.858195 | <= 1.05 | PASS |  |
| 4 stratum cw:0.25-0.6 position RMSE ratio (46 windows) | 0.776593 | <= 1.05 | PASS |  |
| 4 stratum cw:0.6-1.25 position RMSE ratio (194 windows) | 0.68596 | <= 1.05 | PASS |  |
| 5 random sweep of the validity box: finite, inside the state bounds | 0 | <= 0 | PASS | 100000 rollouts of 30 steps; max v 1.317 m/s |
| 6 Yp ratio (G1b, T2b; a re-read) | 0.862133 | <= 1 | PASS |  |
| 7 ccw P1 ratio | 0.89836 | <= 1 | PASS |  |
| 7 ccw Y1 ratio | 1.02928 | <= 1 | FAIL |  |
| 7 cw P1 ratio | 0.727316 | <= 1 | PASS |  |
| 7 cw Y1 ratio | 1.11277 | <= 1 | FAIL |  |
| 8a C1 ratio | 0.932514 | <= 1 | PASS |  |
| 8b A1 ratio | 0.818299 | <= 1.02 | PASS |  |
| 9 near-limit left position RMSE ratio | 0 | n/a 30 | n/a | cannot be applied: 0 windows, at least 30 needed |
| 9 near-limit right position RMSE ratio (163 windows) | 0.708521 | <= 1 | PASS |  |
| 9 transition position RMSE ratio (78 windows) | 0.801799 | <= 1 | PASS |  |
| 10 P1 ratio under T2s | 0.857419 | <= 1 | PASS |  |
| 11 share of held-out steps with the residual output at or above 0.9 of its bound | 0 | <= 0.01 | PASS |  |
| 12 planner solve p50 with fast_mlp / dyn-v0.7p's (replaces GP2g-R's 1.25 x dyn-v0.1) | 1.07532 | <= 1 | FAIL | 3.569 ms against 3.319 ms. mppi_bench planner solve p50, track c=1.0, 512 samples, --fast-mlp, Linux ros-dev:ci on the Mac, median of 5 interleaved runs of 1000 solves (candidate [5.043, 3.569, 3.151, 3.902, 3.218], dyn-v0.7p [4.844, 3.619, 3.319, 3.295, 3.227]) |

Metrics at 1.0 s (macro over strata):

| model | P1 (m) | A1 (m) | C1 (m) | Y1 (rad) | S1 (m/s) | P1 T2s (m) | Yp (rad) | ccw P1 | ccw Y1 | cw P1 | cw Y1 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| dyn-v0.1 | 0.1632 | 0.1395 | 0.0825 | 0.1847 | 0.1592 | 0.1637 | 0.0257 | 0.2060 | 0.2505 | 0.0989 | 0.0860 |
| dyn-v0.7p | 0.1437 | 0.1217 | 0.0748 | 0.1622 | 0.1555 | 0.1442 | 0.0202 | 0.1859 | 0.2327 | 0.0804 | 0.0564 |
| dyn-v0.9-2026-11-25 seed 0 | 0.1236 | 0.0996 | 0.0697 | 0.1688 | 0.1521 | 0.1236 | 0.0174 | 0.1670 | 0.2395 | 0.0585 | 0.0627 |
| dyn-v0.9-2026-11-25 seed 1 | 0.1248 | 0.1002 | 0.0712 | 0.1691 | 0.1540 | 0.1249 | 0.0175 | 0.1685 | 0.2431 | 0.0592 | 0.0581 |
| dyn-v0.9-2026-11-25 seed 2 | 0.1234 | 0.0994 | 0.0698 | 0.1685 | 0.1518 | 0.1235 | 0.0175 | 0.1667 | 0.2399 | 0.0586 | 0.0614 |

P1 ratio to dyn-v0.7p, bootstrap: 0.8601 (0.8343 to 0.9003).
For information, P1 ratio to dyn-v0.1: 0.7573 (0.6921 to 0.8012).
Saturation: 0.0000% of 15750 steps; largest |output| [1.4072470466437192, 0.16152189693531824].

## H2: dyn-v0.7p's 2026-11-15 development parts (reported only, decides nothing)

Windows: 566. Ratios to dyn-v0.7p.

| group | model | windows | P1 m | ratio | Y1 rad | ratio | S1 m/s | ratio |
|---|---|---|---|---|---|---|---|---|
| all | dyn-v0.1 | 566 | 0.11623 | 1.3084 | 0.10465 | 1.4923 | 0.09483 | 1.2266 |
| all | dyn-v0.7p | 566 | 0.08883 | 1.0 | 0.07013 | 1.0 | 0.07732 | 1.0 |
| all | dyn-v0.9-2026-11-25 seed 0 | 566 | 0.07608 | 0.8565 | 0.06834 | 0.9745 | 0.07065 | 0.9139 |
| all | dyn-v0.9-2026-11-25 seed 1 | 566 | 0.07685 | 0.8651 | 0.06784 | 0.9673 | 0.07147 | 0.9244 |
| all | dyn-v0.9-2026-11-25 seed 2 | 566 | 0.07584 | 0.8537 | 0.06746 | 0.962 | 0.07082 | 0.916 |
| cw | dyn-v0.1 | 182 | 0.1397 | 1.3013 | 0.11035 | 1.4562 | 0.10737 | 1.1963 |
| cw | dyn-v0.7p | 182 | 0.10735 | 1.0 | 0.07578 | 1.0 | 0.08975 | 1.0 |
| cw | dyn-v0.9-2026-11-25 seed 0 | 182 | 0.08918 | 0.8307 | 0.07761 | 1.0242 | 0.08328 | 0.9278 |
| cw | dyn-v0.9-2026-11-25 seed 1 | 182 | 0.08943 | 0.833 | 0.0751 | 0.991 | 0.08306 | 0.9254 |
| cw | dyn-v0.9-2026-11-25 seed 2 | 182 | 0.08812 | 0.8208 | 0.07639 | 1.0081 | 0.08289 | 0.9236 |
| ccw | dyn-v0.1 | 384 | 0.08494 | 1.3243 | 0.09705 | 1.5505 | 0.07811 | 1.2862 |
| ccw | dyn-v0.7p | 384 | 0.06414 | 1.0 | 0.0626 | 1.0 | 0.06073 | 1.0 |
| ccw | dyn-v0.9-2026-11-25 seed 0 | 384 | 0.05863 | 0.9141 | 0.05598 | 0.8943 | 0.05383 | 0.8863 |
| ccw | dyn-v0.9-2026-11-25 seed 1 | 384 | 0.06007 | 0.9366 | 0.05815 | 0.9291 | 0.05602 | 0.9224 |
| ccw | dyn-v0.9-2026-11-25 seed 2 | 384 | 0.05947 | 0.9272 | 0.05556 | 0.8876 | 0.05472 | 0.901 |

H2 P1 ratio to dyn-v0.7p, bootstrap: 0.8565 (0.7765 to 0.9750).

