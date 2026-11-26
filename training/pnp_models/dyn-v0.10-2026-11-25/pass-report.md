# dyn-v0.10-2026-11-25 pass against dyn-v0.7p: FAIL

Freeze commit `4e94cd9ab9e1c76a81c63da2b7d2c7dcfff45ad4`; rule SHA-256 e942788459662043c8f9f71378be1d2cbf61a44aba261954d7401f077fb21e6b.

## H1: GP2g-R held-out parts of 2026-11-22 (decides)

Windows: 525 (identical under T2 and T2s); per direction {'ccw': 285, 'cw': 240}; strata {'ccw:0.25-0.6': 8, 'ccw:0.6-0.85': 37, 'ccw:0.85-1.25': 240, 'cw:0.25-0.6': 46, 'cw:0.6-1.25': 194}.

Shipping: multi_step_from_one_capped_wd1e-2_gs_h8 seed 0 (hidden 8).

| item | value | threshold | result | note |
|---|---|---|---|---|
| 0a windows | 525 | >= 400 | PASS |  |
| 0b windows ccw | 285 | >= 150 | PASS |  |
| 0b windows cw | 240 | >= 150 | PASS |  |
| 0c strata with 30 windows | 4 | >= 4 | PASS |  |

| item | value | threshold | result | note |
|---|---|---|---|---|
| 1a P1 ratio / dyn-v0.7p | 0.850547 | <= 0.95 | PASS |  |
| 1b upper end of the bootstrap interval of the P1 ratio | 0.889729 | < 1 | PASS |  |
| 2a Y1 ratio | 0.983235 | <= 0.9 | FAIL |  |
| 2b S1 ratio (replaces GP2g-R's equality) | 0.986418 | <= 1 | PASS |  |
| 3 seed 0 P1 ratio | 0.850547 | <= 1 | PASS |  |
| 3 seed 1 P1 ratio | 0.849867 | <= 1 | PASS |  |
| 3 seed 2 P1 ratio | 0.851438 | <= 1 | PASS |  |
| 4 stratum ccw:0.25-0.6 position RMSE ratio | 8 | n/a 30 | n/a | cannot be applied: 8 windows, at least 30 needed |
| 4 stratum ccw:0.6-0.85 position RMSE ratio (37 windows) | 0.780863 | <= 1.05 | PASS |  |
| 4 stratum ccw:0.85-1.25 position RMSE ratio (240 windows) | 0.838933 | <= 1.05 | PASS |  |
| 4 stratum cw:0.25-0.6 position RMSE ratio (46 windows) | 0.755968 | <= 1.05 | PASS |  |
| 4 stratum cw:0.6-1.25 position RMSE ratio (194 windows) | 0.622738 | <= 1.05 | PASS |  |
| 5 random sweep of the validity box: finite, inside the state bounds | 0 | <= 0 | PASS | 100000 rollouts of 30 steps; max v 1.300 m/s |
| 6 Yp ratio (G1b, T2b; a re-read) | 0.979697 | <= 1 | PASS |  |
| 7 ccw P1 ratio | 0.898727 | <= 1 | PASS |  |
| 7 ccw Y1 ratio | 0.980564 | <= 1 | PASS |  |
| 7 cw P1 ratio | 0.683532 | <= 1 | PASS |  |
| 7 cw Y1 ratio | 0.99977 | <= 1 | PASS |  |
| 8a C1 ratio | 0.907845 | <= 1 | PASS |  |
| 8b A1 ratio | 0.812544 | <= 1.02 | PASS |  |
| 9 near-limit left position RMSE ratio | 0 | n/a 30 | n/a | cannot be applied: 0 windows, at least 30 needed |
| 9 near-limit right position RMSE ratio (163 windows) | 0.626427 | <= 1 | PASS |  |
| 9 transition position RMSE ratio (78 windows) | 0.784043 | <= 1 | PASS |  |
| 10 P1 ratio under T2s | 0.847261 | <= 1 | PASS |  |
| 11 share of held-out steps with the residual output at or above 0.9 of its bound | 0 | <= 0.01 | PASS |  |
| 12 planner solve p50 with fast_mlp / dyn-v0.7p's (replaces GP2g-R's 1.25 x dyn-v0.1) | 0.479161 | <= 1 | PASS | 1.897 ms against 3.959 ms. mppi_bench planner solve p50, track c=1.0, 512 samples, --fast-mlp, Linux ros-dev:ci on the Mac, median of 5 interleaved runs of 1000 solves (candidate [1.897, 2.002, 1.7, 2.024, 1.701], dyn-v0.7p [3.959, 4.203, 3.606, 4.264, 3.605]) |

Metrics at 1.0 s (macro over strata):

| model | P1 (m) | A1 (m) | C1 (m) | Y1 (rad) | S1 (m/s) | P1 T2s (m) | Yp (rad) | ccw P1 | ccw Y1 | cw P1 | cw Y1 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| dyn-v0.1 | 0.1632 | 0.1395 | 0.0825 | 0.1847 | 0.1592 | 0.1637 | 0.0257 | 0.2060 | 0.2505 | 0.0989 | 0.0860 |
| dyn-v0.7p | 0.1437 | 0.1217 | 0.0748 | 0.1622 | 0.1555 | 0.1442 | 0.0202 | 0.1859 | 0.2327 | 0.0804 | 0.0564 |
| dyn-v0.10-2026-11-25 seed 0 | 0.1222 | 0.0989 | 0.0679 | 0.1595 | 0.1533 | 0.1222 | 0.0197 | 0.1670 | 0.2282 | 0.0550 | 0.0564 |
| dyn-v0.10-2026-11-25 seed 1 | 0.1221 | 0.0988 | 0.0678 | 0.1615 | 0.1546 | 0.1220 | 0.0207 | 0.1673 | 0.2311 | 0.0543 | 0.0572 |
| dyn-v0.10-2026-11-25 seed 2 | 0.1223 | 0.0987 | 0.0684 | 0.1618 | 0.1529 | 0.1223 | 0.0202 | 0.1672 | 0.2324 | 0.0551 | 0.0559 |

P1 ratio to dyn-v0.7p, bootstrap: 0.8505 (0.8260 to 0.8897).
For information, P1 ratio to dyn-v0.1: 0.7489 (0.6858 to 0.7917).
Saturation: 0.0000% of 15750 steps; largest |output| [1.4863416347534772, 0.1998510772510608].

## H2: dyn-v0.7p's 2026-11-15 development parts (reported only, decides nothing)

Windows: 566. Ratios to dyn-v0.7p.

| group | model | windows | P1 m | ratio | Y1 rad | ratio | S1 m/s | ratio |
|---|---|---|---|---|---|---|---|---|
| all | dyn-v0.1 | 566 | 0.11623 | 1.3084 | 0.10465 | 1.4923 | 0.09483 | 1.2266 |
| all | dyn-v0.7p | 566 | 0.08883 | 1.0 | 0.07013 | 1.0 | 0.07732 | 1.0 |
| all | dyn-v0.10-2026-11-25 seed 0 | 566 | 0.07639 | 0.86 | 0.0696 | 0.9925 | 0.07348 | 0.9504 |
| all | dyn-v0.10-2026-11-25 seed 1 | 566 | 0.07568 | 0.8519 | 0.06927 | 0.9878 | 0.07285 | 0.9422 |
| all | dyn-v0.10-2026-11-25 seed 2 | 566 | 0.07498 | 0.8441 | 0.06881 | 0.9812 | 0.07085 | 0.9163 |
| cw | dyn-v0.1 | 182 | 0.1397 | 1.3013 | 0.11035 | 1.4562 | 0.10737 | 1.1963 |
| cw | dyn-v0.7p | 182 | 0.10735 | 1.0 | 0.07578 | 1.0 | 0.08975 | 1.0 |
| cw | dyn-v0.10-2026-11-25 seed 0 | 182 | 0.0901 | 0.8393 | 0.07739 | 1.0213 | 0.08752 | 0.9751 |
| cw | dyn-v0.10-2026-11-25 seed 1 | 182 | 0.0895 | 0.8336 | 0.07627 | 1.0065 | 0.08544 | 0.9519 |
| cw | dyn-v0.10-2026-11-25 seed 2 | 182 | 0.088 | 0.8197 | 0.0745 | 0.9831 | 0.08345 | 0.9298 |
| ccw | dyn-v0.1 | 384 | 0.08494 | 1.3243 | 0.09705 | 1.5505 | 0.07811 | 1.2862 |
| ccw | dyn-v0.7p | 384 | 0.06414 | 1.0 | 0.0626 | 1.0 | 0.06073 | 1.0 |
| ccw | dyn-v0.10-2026-11-25 seed 0 | 384 | 0.05812 | 0.9062 | 0.05922 | 0.9461 | 0.05476 | 0.9016 |
| ccw | dyn-v0.10-2026-11-25 seed 1 | 384 | 0.05726 | 0.8928 | 0.05994 | 0.9576 | 0.05606 | 0.923 |
| ccw | dyn-v0.10-2026-11-25 seed 2 | 384 | 0.05762 | 0.8985 | 0.06122 | 0.9781 | 0.05404 | 0.8898 |

H2 P1 ratio to dyn-v0.7p, bootstrap: 0.8600 (0.7689 to 0.9742).

