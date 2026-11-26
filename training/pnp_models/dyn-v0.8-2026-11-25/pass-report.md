# dyn-v0.8-2026-11-25 pass against dyn-v0.7p: FAIL

Freeze commit `2647a9ef532abd4b040749f0c3eae0f8c97fd45b`; rule SHA-256 c7b87874d94b4899165b48ab95f2443ad9dfe8207fabe0f8e9253fc00a7699b4.

## H1: GP2g-R held-out parts of 2026-11-22 (decides)

Windows: 525 (identical under T2 and T2s); per direction {'ccw': 285, 'cw': 240}; strata {'ccw:0.25-0.6': 8, 'ccw:0.6-0.85': 37, 'ccw:0.85-1.25': 240, 'cw:0.25-0.6': 46, 'cw:0.6-1.25': 194}.

Shipping: multi_step_from_one_capped_wd1e-2 seed 1 (hidden 32).

| item | value | threshold | result | note |
|---|---|---|---|---|
| 0a windows | 525 | >= 400 | PASS |  |
| 0b windows ccw | 285 | >= 150 | PASS |  |
| 0b windows cw | 240 | >= 150 | PASS |  |
| 0c strata with 30 windows | 4 | >= 4 | PASS |  |

| item | value | threshold | result | note |
|---|---|---|---|---|
| 1a P1 ratio / dyn-v0.7p | 1.02648 | <= 0.95 | FAIL |  |
| 1b upper end of the bootstrap interval of the P1 ratio | 1.03456 | < 1 | FAIL |  |
| 2a Y1 ratio | 1.04114 | <= 0.9 | FAIL |  |
| 2b S1 ratio (replaces GP2g-R's equality) | 1.00498 | <= 1 | FAIL |  |
| 3 seed 0 P1 ratio | 1.03206 | <= 1 | FAIL |  |
| 3 seed 1 P1 ratio | 1.02648 | <= 1 | FAIL |  |
| 3 seed 2 P1 ratio | 1.02236 | <= 1 | FAIL |  |
| 4 stratum ccw:0.25-0.6 position RMSE ratio | 8 | n/a 30 | n/a | cannot be applied: 8 windows, at least 30 needed |
| 4 stratum ccw:0.6-0.85 position RMSE ratio (37 windows) | 1.12656 | <= 1.05 | FAIL |  |
| 4 stratum ccw:0.85-1.25 position RMSE ratio (240 windows) | 1.14852 | <= 1.05 | FAIL |  |
| 4 stratum cw:0.25-0.6 position RMSE ratio (46 windows) | 0.970981 | <= 1.05 | PASS |  |
| 4 stratum cw:0.6-1.25 position RMSE ratio (194 windows) | 0.991395 | <= 1.05 | PASS |  |
| 5 random sweep of the validity box: finite, inside the state bounds | 0 | <= 0 | PASS | 100000 rollouts of 30 steps; max v 1.306 m/s |
| 6 Yp ratio (G1b, T2b; a re-read) | 0.904508 | <= 1 | PASS |  |
| 7 ccw P1 ratio | 1.03929 | <= 1 | FAIL |  |
| 7 ccw Y1 ratio | 1.04092 | <= 1 | FAIL |  |
| 7 cw P1 ratio | 0.98208 | <= 1 | PASS |  |
| 7 cw Y1 ratio | 1.04248 | <= 1 | FAIL |  |
| 8a C1 ratio | 1.06277 | <= 1 | FAIL |  |
| 8b A1 ratio | 1.01291 | <= 1.02 | PASS |  |
| 9 near-limit left position RMSE ratio | 0 | n/a 30 | n/a | cannot be applied: 0 windows, at least 30 needed |
| 9 near-limit right position RMSE ratio (163 windows) | 0.996812 | <= 1 | PASS |  |
| 9 transition position RMSE ratio (78 windows) | 1.04973 | <= 1 | FAIL |  |
| 10 P1 ratio under T2s | 1.02674 | <= 1 | FAIL |  |
| 11 share of held-out steps with the residual output at or above 0.9 of its bound | 0 | <= 0.01 | PASS |  |
| 12 planner solve p50 with fast_mlp / dyn-v0.7p's (replaces GP2g-R's 1.25 x dyn-v0.1) | 1.07227 | <= 1 | FAIL | 3.635 ms against 3.390 ms. mppi_bench planner solve p50, track c=1.0, 512 samples, --fast-mlp, Linux ros-dev:ci on the Mac, median of 3 interleaved runs of 1000 solves (candidate [3.638, 3.243, 3.635], dyn-v0.7p [3.39, 3.176, 3.895]) |

Metrics at 1.0 s (macro over strata):

| model | P1 (m) | A1 (m) | C1 (m) | Y1 (rad) | S1 (m/s) | P1 T2s (m) | Yp (rad) | ccw P1 | ccw Y1 | cw P1 | cw Y1 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| dyn-v0.1 | 0.1632 | 0.1395 | 0.0825 | 0.1847 | 0.1592 | 0.1637 | 0.0257 | 0.2060 | 0.2505 | 0.0989 | 0.0860 |
| dyn-v0.7p | 0.1437 | 0.1217 | 0.0748 | 0.1622 | 0.1555 | 0.1442 | 0.0202 | 0.1859 | 0.2327 | 0.0804 | 0.0564 |
| dyn-v0.8-2026-11-25 seed 0 | 0.1483 | 0.1234 | 0.0805 | 0.1711 | 0.1531 | 0.1489 | 0.0182 | 0.1920 | 0.2439 | 0.0827 | 0.0619 |
| dyn-v0.8-2026-11-25 seed 1 | 0.1475 | 0.1232 | 0.0795 | 0.1689 | 0.1562 | 0.1481 | 0.0182 | 0.1932 | 0.2422 | 0.0790 | 0.0588 |
| dyn-v0.8-2026-11-25 seed 2 | 0.1469 | 0.1225 | 0.0793 | 0.1686 | 0.1528 | 0.1475 | 0.0175 | 0.1908 | 0.2405 | 0.0811 | 0.0609 |

P1 ratio to dyn-v0.7p, bootstrap: 1.0265 (1.0174 to 1.0346).
For information, P1 ratio to dyn-v0.1: 0.9038 (0.8184 to 0.9317).
Saturation: 0.0000% of 15750 steps; largest |output| [1.5602841724197096, 0.20653008883451704].

## H2: dyn-v0.7p's 2026-11-15 development parts (reported only, decides nothing)

Windows: 566. Ratios to dyn-v0.7p.

| group | model | windows | P1 m | ratio | Y1 rad | ratio | S1 m/s | ratio |
|---|---|---|---|---|---|---|---|---|
| all | dyn-v0.1 | 566 | 0.11623 | 1.3084 | 0.10465 | 1.4923 | 0.09483 | 1.2266 |
| all | dyn-v0.7p | 566 | 0.08883 | 1.0 | 0.07013 | 1.0 | 0.07732 | 1.0 |
| all | dyn-v0.8-2026-11-25 seed 0 | 566 | 0.09084 | 1.0226 | 0.06693 | 0.9544 | 0.07573 | 0.9795 |
| all | dyn-v0.8-2026-11-25 seed 1 | 566 | 0.08943 | 1.0067 | 0.06796 | 0.9691 | 0.07765 | 1.0043 |
| all | dyn-v0.8-2026-11-25 seed 2 | 566 | 0.0901 | 1.0142 | 0.0666 | 0.9496 | 0.07666 | 0.9915 |
| cw | dyn-v0.1 | 182 | 0.1397 | 1.3013 | 0.11035 | 1.4562 | 0.10737 | 1.1963 |
| cw | dyn-v0.7p | 182 | 0.10735 | 1.0 | 0.07578 | 1.0 | 0.08975 | 1.0 |
| cw | dyn-v0.8-2026-11-25 seed 0 | 182 | 0.10848 | 1.0104 | 0.07471 | 0.9859 | 0.08802 | 0.9807 |
| cw | dyn-v0.8-2026-11-25 seed 1 | 182 | 0.10601 | 0.9874 | 0.07538 | 0.9948 | 0.09052 | 1.0086 |
| cw | dyn-v0.8-2026-11-25 seed 2 | 182 | 0.10749 | 1.0013 | 0.07502 | 0.99 | 0.08924 | 0.9943 |
| ccw | dyn-v0.1 | 384 | 0.08494 | 1.3243 | 0.09705 | 1.5505 | 0.07811 | 1.2862 |
| ccw | dyn-v0.7p | 384 | 0.06414 | 1.0 | 0.0626 | 1.0 | 0.06073 | 1.0 |
| ccw | dyn-v0.8-2026-11-25 seed 0 | 384 | 0.06732 | 1.0497 | 0.05656 | 0.9037 | 0.05933 | 0.977 |
| ccw | dyn-v0.8-2026-11-25 seed 1 | 384 | 0.06733 | 1.0499 | 0.05807 | 0.9278 | 0.06049 | 0.996 |
| ccw | dyn-v0.8-2026-11-25 seed 2 | 384 | 0.0669 | 1.0431 | 0.05537 | 0.8846 | 0.05987 | 0.9859 |

H2 P1 ratio to dyn-v0.7p, bootstrap: 1.0067 (0.9936 to 1.0295).

