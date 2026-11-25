# trackperc-v4 (2026-11-25): not shipped, v2 stays the published net

Unregistered development candidate (owner decisions 29 and 35). Prestatement committed and pushed
before training: `evaluation/prereg/w2-trackperc-v4-2026-11-25.yaml`, SHA-256
`67047d6550ffae1c0cfe1661bb75d87a7047bd3511bad895a73d714538139692` (b57cd18). Scored once with
`racer_pnp_perception.compare_v4`. Candidate bundle:
`car-data/pnp/models-candidate/trackperc-v4-2026-11-25` (weights not committed), shown as
"(candidate)" in the Racer app Net bundle picker. No default changes; the prototype profile keeps v2.

## Labels: fast RMS gate (decision 35)

`selflabel --rms-gate-fast 0.025 --rms-fast-speed 0.8` (default off): an ICP interval faster than
0.8 m/s (moving-test speed at the interval midpoint) passes the RMS item at 0.025 m instead of
0.02 m. ICP, seeding, inlier share, dt and every label gate unchanged; cached ICP results reused.
With the flag off the labeller reproduces `selflabels-v3` array for array.

Kept labels, v3 split bags, moving threshold 0.2 m/s (default gate -> fast gate):

| speed band (m/s) | moving | kept default | kept fast | icp_failed | deficit | path_outside |
|---|---|---|---|---|---|---|
| 0.0-0.5 | 22980 | 11786 | 11864 | 1234 -> 952 | 718 -> 757 | 3272 -> 3386 |
| 0.5-0.8 | 6362 | 2854 | 3001 | 1151 -> 830 | 510 -> 591 | 479 -> 539 |
| 0.8-1.0 | 5746 | 2145 | 2822 | 1723 -> 432 | 261 -> 580 | 1032 -> 1266 |
| 1.0-1.2 | 3291 | 262 | 1036 | 2244 -> 479 | 110 -> 657 | 341 -> 612 |
| 1.2+ | 2339 | 154 | 402 | 1517 -> 365 | 94 -> 440 | 310 -> 777 |

Session 2 (2026-11-24T04-36-54): 394 -> 1505 (train 383 -> 758, H-session 11 -> 747); 0.8-1.0
16 -> 374, 1.0-1.2 0 -> 611, 1.2+ 0 -> 74. After the ICP gate, the deficit gate is now the main
drop at speed (1261 of 4930 moving scans). 2026-11-16 bags: 2756 -> 3431. H-layout: 1070 -> 1122.
No label the default keeps is lost.

Kept-label quality, all bags (default / fast / fast-only labels): intrusion mean 0.0004 / 0.0004 /
0.0008, deficit p90 mean 0.0122 / 0.0131 / 0.0210 m, path containment mean 0.981 / 0.981 / 0.978.
The new labels sit at faster, harder parts of the lap and are a little worse, all inside the
unchanged gates.

## Training

v3 recipe: init from v2 (runs-v2/full/seed1), the v3 simulation set (seed 23, lidar-dr) reused,
6 epochs, batch 256, lr 1e-3, real share 0.75, scale augmentation, seeds 0 1 2. Real: 14099 train
and 2341 internal validation labels. Seed 2 chosen (internal val E_c 0.0664; 0.0667, 0.0665).

## Score (v4 label set, the stated rule)

| item | value | limit | result |
|---|---|---|---|
| H-layout E_c(v4) <= E_c(v2) | 0.1288 | 0.1372 | pass |
| H-layout V(v4) >= V(v2) - 0.25 | 14.78 | 14.86 | **FAIL** |
| H-layout C95(v4) >= C95(v2) - 0.02 | 0.721 | 0.704 | pass |
| H-layout intrusion <= max(0.02, v2 + 0.005) | 0.0000 | 0.020 | pass |
| H-session labelled scans >= 300 | 747 | 300 | pass |
| H-session E_c(v4) <= 1.05 E_c(v2) | 0.1516 | 0.1870 | pass |
| H-session V(v4) >= V(v2) - 0.25 | 13.85 | 15.02 | **FAIL** |
| latency p99 <= 1.2 v2 | 0.567 ms | 0.689 ms | pass |

Reference (decides nothing): v3 on the same labels: H-layout E_c 0.125, V 14.57; H-session E_c
0.151, V 13.12. geometric H-session E_c 0.290. On the v3 label set v4 is H-layout E_c 0.131, V
14.79 (v2 0.139, 15.13).

Reading: the fast labels moved v4 back towards v2 on valid range (V +0.20 on H-layout and +0.73 on
H-session against v3) with most of v3's centre gain kept (H-session E_c 0.152 vs v2 0.178), but V
is still short of the 0.25 margin on both sets. The V loss is in the recipe (v3 had it with almost
no fast labels), not in the labels.

## CPU and latency (v4 vs v2, same architecture)

* Dev Mac, `racer_pnp_perception.latency`, 2000 session 2 scans, single thread: net forward p50
  0.096 / 0.096 ms; scan to track (net) p50 0.505 / 0.506, p99 0.556 / 0.577 ms.
* arm64 Linux container (ros-dev:ci, the W2 bench2 tool, 1500 scans, 2 reps): net wall p50
  0.668-0.668 / 0.669-0.675 ms, CPU mean 0.668-0.670 / 0.658-0.662 ms; geometric about 1.4 ms.
  No difference beyond noise.
