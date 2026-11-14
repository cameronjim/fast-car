GP1 verdict: better where it trained, not shown elsewhere (items 2 and 3 passed, item 4, 5 or 6 failed), geometric stays deployed for unseen layouts

# trackperc-v1-2026-11-14

Track perception net (`circ_conv1d_v0`) trained on simulated lanes and on self-labels of
the car's own scans (geometry seen later by the same sensor), role `train` of the
pre-registered split only. The weights are not committed; the bundle lives in
`/Users/cameronjim/code/car-data/pnp/models/` on the dev machine. Decision rule:
`docs/notes/preregistration-pnp-v0.2.md` section 2, evaluated once on the held-out sets.

Split SHA-256: `bc5185c912ece67a6177581f1b37adaaa9101b34ee1efe9881af7723681aef83`.

| set | backend | labelled scans | E_c (m) | E_far (m) | V | C95 |
|---|---|---|---|---|---|---|
| H-run | net | 2597 | 0.051 | 0.070 | 16.000 | 0.935 |
| H-run | geometric | 2597 | 0.136 | 0.193 | 16.000 | 0.934 |
| H-layout | net | 1088 | 0.122 | 0.170 | 15.505 | 0.718 |
| H-layout | geometric | 1088 | 0.150 | 0.217 | 15.955 | 0.799 |

| set | backend | moving scans | mean intrusion | residual median (m) | deficit p90 (m) | C95 |
|---|---|---|---|---|---|---|
| A-run | net | 4009 | 0.002 | 0.064 | 0.018 | 0.884 |
| A-run | geometric | 4009 | 0.003 | 0.026 | 0.021 | 0.889 |
| A-layout | net | 2715 | 0.023 | 0.081 | 0.067 | 0.491 |
| A-layout | geometric | 2715 | 0.007 | 0.031 | 0.025 | 0.564 |

Bootstrap of E_c(net) / E_c(geometric) on H-run: 0.374, 95 percent interval [0.355, 0.395] (169 blocks of 2.0 s, 2000 resamples, seed 0).

| item | result | value |
|---|---|---|
| 1. H-run has at least 300 labelled scans | pass | `{"labelled_scans": 2597}` |
| 2. in-layout accuracy | pass | `{"e_c_net_m": 0.05073116134262434, "e_c_geometric_m": 0.1355333784055033, "ratio": 0.37430750962940956, "ratio_ci_upper": 0.39486013676175413}` |
| 3. in-layout range | pass | `{"v_net": 16.0, "v_geometric": 16.0}` |
| 4. cross-layout | pass | `{"e_c_net_m": 0.12203702894600996, "e_c_geometric_m": 0.14998033866723068, "ratio": 0.8136868474259148, "v_net": 15.504595588235293, "v_geometric": 15.954963235294118}` |
| 5. label-free non-inferiority on A-run and A-layout | FAIL | `{"a_run": {"intrusion_mean": 0.0015580707962275127, "residual_median_m": 0.06380921319249921, "deficit_p90_m": 0.017983137550180023, "c95": 0.8841068917018284, "geometric_intrusion_mean": 0.0030071895218507115, "geometric_c95": 0.889451476793249}, "a_layout": {"intrusion_mean": 0.02274346084359733, "residual_median_m": 0.08067818709849095, "deficit_p90_m": 0.0672450514734842, "c95": 0.4908561928512053, "geometric_intrusion_mean": 0.006523744443497586, "geometric_c95": 0.5635910224438903}}` |
| 6. per cell | pass | `{"cells_judged": 8, "cells_not_judged": 1, "worst_ratio": 0.9133180093895644, "failing_cells": []}` |

| cell | labelled scans | E_c ratio | judged |
|---|---|---|---|
| layout unknown 0.85+ | 269 | 0.625 | yes |
| layout unknown [0.2, 0.6) | 797 | 0.913 | yes |
| layout unknown [0.6, 0.85) | 22 | 0.754 | no |
| run ccw 0.85+ | 42 | 0.659 | yes |
| run ccw [0.2, 0.6) | 1224 | 0.379 | yes |
| run ccw [0.6, 0.85) | 156 | 0.420 | yes |
| run cw 0.85+ | 48 | 0.403 | yes |
| run cw [0.2, 0.6) | 958 | 0.361 | yes |
| run cw [0.6, 0.85) | 169 | 0.334 | yes |

Numpy forward latency on the dev Mac, single thread, 2000 real scans: p50 0.049 ms, p99 0.062 ms (a proxy; the Orin number is taken at the car).

Simulation, held-out family tracks, net raw output, median error in metres (v1 next to v0, no threshold):

| track | centreline 0-7 v1 / v0 | centreline 8-15 v1 / v0 | half width v1 / v0 |
|---|---|---|---|
| stadium_l1.0_s2.0_r0.4 | 0.012 / 0.010 | 0.044 / 0.039 | 0.020 / 0.010 |
| rounded_rect_l1.0_3.0x2.0_r0.4 | 0.011 / 0.009 | 0.038 / 0.033 | 0.017 / 0.009 |
| rounded_rect_l1.2_2.0x1.6_r0.8 | 0.010 / 0.008 | 0.022 / 0.018 | 0.014 / 0.009 |

Everything, as stored in the bundle manifest (`metrics.json` is the same):

