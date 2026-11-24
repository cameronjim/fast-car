# trackperc-v3 (2026-11-23): not shipped, v2 stays the published net

Unregistered development candidate (owner decision 29), overnight 2026-11-23 workstream W2. Roles,
held-out sets and the ship rule were committed and pushed before any v3 training:
`evaluation/prereg/w2-trackperc-v3-2026-11-23.yaml`, SHA-256
`03ab38fac9ed0aa629a3b4331c855cc1a943f78847e1eaa988030a2a43780b01` (commit 9577d04). Scored once,
after the freeze, with `racer_pnp_perception.compare_v3`.

Outcome by the stated rule: **not shipped**. v3 has the better centre on the unseen layout but a
shorter valid range, which the rule does not allow. No v3 bundle was put under
`data/pnp/models`; the candidate stays in `car-data/pnp/models-candidate/trackperc-v3-2026-11-23`
(weights not committed) and is not for the car.

## What v3 is

Same architecture and contract as v2 (`circ_conv1d_v0_refine_v1`, 16 x 5 output, scan refinement),
initialised from the v2 weights, the v2 recipe (6 epochs, batch 256, lr 1e-3, real share 0.75,
scale augmentation), seeds 0 1 2, seed 0 chosen on internal validation E_c (0.0632; 0.0632 and
0.0633 for the other seeds).

* Simulation: a fresh 300000 / 20000 sample set of random tracks (seed 23) with the new LiDAR-model
  randomisation (`dataset --lidar-dr`: mount yaw error up to 1.5 deg, mount offset up to 1.5 cm,
  range scale 1.5 percent, range bias 2 cm, range-dependent dropout up to 0.4 at 6 m, up to three
  dark sectors that return nothing beyond 0.8 to 3.0 m) on top of the v0 nuisances.
* Real self-labels (`selflabel --split`, `car-data/pnp/selflabels-v3`): 13213 train and 2184
  internal validation labels, from the v0.2 train bags, the whole 2026-11-13 loop (its old
  held-out segments included), the 2026-11-16 bags and the first 55 percent of each 2026-11-23 run.
* Not touched at all (registered held-out data stays fresh): every 2026-11-14 bag and the
  2026-11-22 bag.

## Numbers

Every backend through `TrackPerceiver.process`, refinement as each bundle declares. E_c, V, C95
and the label-free numbers are the GP1 definitions.

| set | backend | scans | E_c (m) | V | C95 | intrusion | deficit p90 (m) |
|---|---|---|---|---|---|---|---|
| H-layout (2026-11-12, labelled / all moving) | geometric | 1070 / 2715 | 0.154 | 15.96 | 0.798 | 0.0065 | 0.025 |
| | v2 | 1070 / 2715 | 0.139 | 15.13 | 0.733 | 0.0000 | 0.005 |
| | v3 | 1070 / 2715 | **0.127** | **14.58** | 0.722 | 0.0000 | 0.005 |
| H-session (2026-11-23, last 45 percent) | geometric | 11 / 2112 | 0.269 | 16.00 | 0.455 | 0.0096 | 0.049 |
| | v2 | 11 / 2112 | 0.183 | 15.45 | 1.000 | 0.0000 | 0.005 |
| | v3 | 11 / 2112 | 0.166 | 13.82 | 0.909 | 0.0000 | 0.005 |

Latency p99, preprocess + net + refinement, dev Mac: v2 0.90 ms, v3 0.86 ms.

| item | value | limit | result |
|---|---|---|---|
| H-layout E_c(v3) <= E_c(v2) | 0.127 | 0.139 | pass |
| H-layout V(v3) >= V(v2) - 0.25 | 14.58 | 14.88 | **FAIL** |
| H-layout C95(v3) >= C95(v2) - 0.02 | 0.722 | 0.713 | pass |
| H-layout intrusion <= max(0.02, v2 + 0.005) | 0.000 | 0.020 | pass |
| H-session E_c(v3) <= 1.05 E_c(v2) | 0.166 | 0.192 | pass |
| H-session V(v3) >= V(v2) - 0.25 | 13.82 | 15.20 | **FAIL** |
| latency p99 <= 1.2 v2 | 0.86 | 1.08 | pass |

## Read this before using the numbers

* H-session has only 11 labelled scans. At 1.3 m/s the scan-to-scan ICP of the self-labeller
  fails on most intervals (3346 of 4930 moving scans of the 2026-11-23 bag dropped as
  `icp_failed`), so the fast runs give almost no labels. Its E_c and V items rest on 11 scans and
  mean little either way; the label-free numbers use all 2112 moving scans. The rule was stated
  without a minimum count and is applied as written.
* The H-layout result is the informative one: the centre improves by 9 percent on a layout no net
  trained on, and the valid range drops by about half a station (0.11 m of the 3 m horizon).
  Whether a better centre is worth a shorter horizon is a question for the planner, not this rule.
* Self-labels at racing speed are the bottleneck for using new sessions: a wheel-odometry seed for
  the ICP (as `racer_pnp_dynamics` uses for its truth) would likely recover most of them.
