GP1b verdict: no better than geometric in the layout (item 1 or 2 failed), geometric stays

# trackperc-v2-2026-11-14

Net plus scan refinement (`io.arch.type: circ_conv1d_v0_refine_v1`): the v1 network with
scale-augmented real samples, followed by `racer_track_perception.refine`. Weights are not
committed (`/Users/cameronjim/code/car-data/pnp/models/`). Decision rule:
`docs/notes/preregistration-pnp-v0.3.md` section 3.3, constants
`evaluation/prereg/pnp-v0.3.yaml`; thresholds are those of v0.2 section 2.
Recipe frozen at git `c66ce014dabac8340239252f70128d7992e17c51`. Split SHA-256 `bc5185c912ece67a6177581f1b37adaaa9101b34ee1efe9881af7723681aef83`.
Moving scans: wheel speed >= 0.2 m/s; ICP ground speed >= 0.2 m/s without /odom/wheel; speed source per fold bag: {'2026-11-11T03-54-11_car_teleop_cw': 'odom', '2026-11-11T04-20-19_car_teleop_v3': 'odom', '2026-10-24T23-13-18_car_teleop': 'icp', '2026-10-25T19-27-08_car_teleop': 'icp'}.

The clearance deficit of the refined output holds BY CONSTRUCTION (per-side clearance cap
in the refinement); the variant without the cap and the unrefined net are printed next to
it. The decision below is the registered one, applied as written.

### refined (candidate): no better than geometric in the layout (item 1 or 2 failed), geometric stays

| item | result | note |
|---|---|---|
| 1. v0.2 items 1, 2, 3 and 6 on H-run, full model | FAIL | second read of H-run (read once by net v1 in v0.2) |
| 2. label-free rule on A-run, full model | pass | clearance deficit p90 holds by construction for the refined output (per-side clearance cap of refine.py); see the variant without the cap |
| 3. each fold with 100 or more labelled scans: accuracy and range | FAIL |  |
| 4. each fold, all moving scans: the label-free rule | FAIL | clearance deficit p90 holds by construction for the refined output (per-side clearance cap of refine.py); see the variant without the cap |
| 5. label-free rule on the 2026-11-12 layout, full model | FAIL | development data, reported and not evidence; clearance deficit p90 holds by construction for the refined output (per-side clearance cap of refine.py); see the variant without the cap |

| set | backend | scans | E_c (m) | V | intrusion | residual median (m) | deficit p90 (m) | C95 |
|---|---|---|---|---|---|---|---|---|
| H-run | net | 2597 | 0.064 | 15.310 | n/a | n/a | n/a | 0.945 |
| H-run | geometric | 2597 | 0.136 | 16.000 | n/a | n/a | n/a | 0.934 |
| A-run | net | 4009 | n/a | n/a | 0.000 | 0.041 | 0.005 | 0.897 |
| A-run | geometric | 4009 | n/a | n/a | 0.003 | 0.026 | 0.021 | 0.889 |
| A-layout-2026-11-12 | net | 2715 | n/a | n/a | 0.000 | 0.045 | 0.005 | 0.480 |
| A-layout-2026-11-12 | geometric | 2715 | n/a | n/a | 0.007 | 0.031 | 0.025 | 0.564 |
| fold F1 | net | 1634 labelled / 4826 moving | 0.113 | 14.392 | 0.000 | 0.050 | 0.006 | 0.603 |
| fold F1 | geometric | 1634 labelled / 4826 moving | 0.174 | 15.280 | 0.005 | 0.033 | 0.024 | 0.636 |
| fold F2 | net | 3271 labelled / 5201 moving | 0.102 | 15.237 | 0.000 | 0.042 | 0.006 | 0.833 |
| fold F2 | geometric | 3271 labelled / 5201 moving | 0.169 | 15.038 | 0.007 | 0.035 | 0.043 | 0.748 |

### refined, without the clearance cap (report only): no better than geometric in the layout (item 1 or 2 failed), geometric stays

| item | result | note |
|---|---|---|
| 1. v0.2 items 1, 2, 3 and 6 on H-run, full model | FAIL | second read of H-run (read once by net v1 in v0.2) |
| 2. label-free rule on A-run, full model | pass |  |
| 3. each fold with 100 or more labelled scans: accuracy and range | FAIL |  |
| 4. each fold, all moving scans: the label-free rule | FAIL |  |
| 5. label-free rule on the 2026-11-12 layout, full model | FAIL | development data, reported and not evidence |

| set | backend | scans | E_c (m) | V | intrusion | residual median (m) | deficit p90 (m) | C95 |
|---|---|---|---|---|---|---|---|---|
| H-run | net | 2597 | 0.064 | 15.310 | n/a | n/a | n/a | 0.945 |
| H-run | geometric | 2597 | 0.136 | 16.000 | n/a | n/a | n/a | 0.934 |
| A-run | net | 4009 | n/a | n/a | 0.000 | 0.041 | 0.005 | 0.897 |
| A-run | geometric | 4009 | n/a | n/a | 0.003 | 0.026 | 0.021 | 0.889 |
| A-layout-2026-11-12 | net | 2715 | n/a | n/a | 0.000 | 0.043 | 0.006 | 0.480 |
| A-layout-2026-11-12 | geometric | 2715 | n/a | n/a | 0.007 | 0.031 | 0.025 | 0.564 |
| fold F1 | net | 1634 labelled / 4826 moving | 0.113 | 14.392 | 0.001 | 0.049 | 0.013 | 0.603 |
| fold F1 | geometric | 1634 labelled / 4826 moving | 0.174 | 15.280 | 0.005 | 0.033 | 0.024 | 0.636 |
| fold F2 | net | 3271 labelled / 5201 moving | 0.102 | 15.237 | 0.001 | 0.040 | 0.011 | 0.833 |
| fold F2 | geometric | 3271 labelled / 5201 moving | 0.169 | 15.038 | 0.007 | 0.035 | 0.043 | 0.748 |

### unrefined (same weights, report only): better where it trained, not shown elsewhere (items 1 and 2 passed, item 3, 4 or 5 failed), geometric stays deployed

| item | result | note |
|---|---|---|
| 1. v0.2 items 1, 2, 3 and 6 on H-run, full model | pass | second read of H-run (read once by net v1 in v0.2) |
| 2. label-free rule on A-run, full model | pass |  |
| 3. each fold with 100 or more labelled scans: accuracy and range | pass |  |
| 4. each fold, all moving scans: the label-free rule | FAIL |  |
| 5. label-free rule on the 2026-11-12 layout, full model | FAIL | development data, reported and not evidence |

| set | backend | scans | E_c (m) | V | intrusion | residual median (m) | deficit p90 (m) | C95 |
|---|---|---|---|---|---|---|---|---|
| H-run | net | 2597 | 0.057 | 16.000 | n/a | n/a | n/a | 0.908 |
| H-run | geometric | 2597 | 0.136 | 16.000 | n/a | n/a | n/a | 0.934 |
| A-run | net | 4009 | n/a | n/a | 0.003 | 0.070 | 0.023 | 0.881 |
| A-run | geometric | 4009 | n/a | n/a | 0.003 | 0.026 | 0.021 | 0.889 |
| A-layout-2026-11-12 | net | 2715 | n/a | n/a | 0.014 | 0.089 | 0.056 | 0.506 |
| A-layout-2026-11-12 | geometric | 2715 | n/a | n/a | 0.007 | 0.031 | 0.025 | 0.564 |
| fold F1 | net | 1634 labelled / 4826 moving | 0.122 | 15.066 | 0.066 | 0.117 | 0.112 | 0.616 |
| fold F1 | geometric | 1634 labelled / 4826 moving | 0.174 | 15.280 | 0.005 | 0.033 | 0.024 | 0.636 |
| fold F2 | net | 3271 labelled / 5201 moving | 0.106 | 15.758 | 0.052 | 0.089 | 0.083 | 0.871 |
| fold F2 | geometric | 3271 labelled / 5201 moving | 0.169 | 15.038 | 0.007 | 0.035 | 0.043 | 0.748 |

Latency on the dev machine, 2000 real scans, preprocess + net + refinement:
p50 0.727 ms, p99 0.774 ms. The Orin budget is p99 under 10 ms
scan to message; this is the dev machine (Apple M5 Pro, one thread), not the Orin; the Orin number is taken at the car. The measured p99 is about 13 times under the 10 ms budget.

Everything, as stored in the bundle manifest (`metrics.json` is the same):

