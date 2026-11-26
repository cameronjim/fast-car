# trackperc-v5 (2026-11-25): not shipped, v2 stays the published net

Unregistered development candidate (owner decisions 29 and 37a). Prestatement committed and pushed
before any held-out scoring: `evaluation/prereg/w2-trackperc-v5-2026-11-25.yaml`, SHA-256
`f465f61663183325671424bdccb26fe290aa99229952f2221c05dbcd8bff8971` (37ea3e7). Scored once with
`racer_pnp_perception.compare_v5` (the v4 rule unchanged). Candidate bundle:
`car-data/pnp/models-candidate/trackperc-v5-2026-11-25` (weights not committed), shown as
"(candidate)" in the Racer app Net bundle picker. No default changes; the prototype profile keeps v2.

## Diagnosis of the V loss (train-role internal validation and simulation only)

V is the leading run of stations whose predicted sigma is under the node's 0.25 m gate, so it is
set by the sigma head. Internal validation cut of the v4 split (2341 labels, last 15 percent of
each train segment); no held-out set was read for any of this.

| net | V all | V old bags | V 2026-11-16 | V session 2 | E_c all |
|---|---|---|---|---|---|
| v2 | 15.97 | 15.99 | 15.84 | 16.00 | 0.0740 |
| v4 seed 2 | 15.80 | 15.96 | 14.97 | 15.12 | 0.0664 |
| v5 (chosen) | 15.98 | 16.00 | 15.91 | 15.93 | 0.0675 |

* Not label class balance: 99.6 percent of train labels have all 16 stations valid.
* Not the simulation set or the init: the v4 recipe with v2's v0 simulation set gives V 15.82;
  initialised from the sim-only v0 net, 15.85 (v4 seed 0: 15.83). Simulation V is the same for v2
  and v4 (15.78 / 15.80).
* Not early stopping as such: V only drifts a little over the 6 epochs (seed 2: 15.87 to 15.80),
  and the three v4 seeds' final V sit within 0.03, so selecting on E_c alone costs at most 0.03.
* Where: only on bags added after v2 (2026-11-16, session 2). There v4's x, y are better than v2's
  (E_c 0.108 vs 0.154 on 2026-11-16) but its sigma is higher. The Gaussian likelihood on noisy
  self-labels teaches sigma the label error, and v2 never saw these bags, so its sigma stayed low.
  Two parts: (a) far-station sigma over 0.25 m (about a third of the loss); (b) whole-track
  collapse to V = 0 when station 0's sigma passes 0.25 m in tight turns (15 of 337 2026-11-16
  validation scans, about two thirds).
* Rejected on the same data: real labels never training sigma (V 15.55, old bags lose the last
  station); a label-noise term in the full 6-epoch training (fixes (a) but not (b), and makes E_c
  worse because it also down-weights the real x, y fit).

## Recipe (two changes)

1. Label-noise-aware likelihood for real labels: variance sigma^2 + s^2, s = 0.10 m at every
   station (`train --real-label-sigma-near 0.10`). Chosen as the smallest s tried (0.05, 0.10,
   with and without a far ramp) whose internal V reached v2's 15.97 (0.10: 15.98; 0.05: 15.92).
2. Applied as a 1-epoch sigma-head refit (`train --sigma-head-only`): only the fc1 rows that output
   log_sigma train; everything else is frozen.

Stage 1 is the v4 recipe: the three v4 seed runs, reused unchanged. Seed chosen by final internal
E_c: seed 2 again. So v5 has exactly v4's x, y and half widths (checked weight by weight: only the
16 log_sigma rows of fc1 differ). Every E_c change against v4 comes from the stations v5 now keeps.

## Score (the stated rule, v4 label set)

| item | value | limit | result |
|---|---|---|---|
| H-layout E_c(v5) <= E_c(v2) | 0.1345 | 0.1372 | pass |
| H-layout V(v5) >= V(v2) - 0.25 | 15.21 | 14.86 | pass |
| H-layout C95(v5) >= C95(v2) - 0.02 | 0.740 | 0.704 | pass |
| H-layout intrusion <= max(0.02, v2 + 0.005) | 0.0000 | 0.020 | pass |
| H-session labelled scans >= 300 | 747 | 300 | pass |
| H-session E_c(v5) <= 1.05 E_c(v2) | 0.1644 | 0.1870 | pass |
| H-session V(v5) >= V(v2) - 0.25 | 14.85 | 15.02 | **FAIL** |
| latency p99 <= 1.2 v2 | 0.825 ms | 1.125 ms | pass |

Reference (decides nothing): v4 H-layout E_c 0.129, V 14.78; H-session E_c 0.152, V 13.85.
geometric H-session E_c 0.290, V 15.99.

Reading: the sigma refit fixes H-layout. V there goes from 14.78 to 15.21 (v2 15.11), and E_c
stays under v2's even with the extra far stations. On H-session V recovers by 1.0 (13.85 to 14.85)
but stays 0.17 under the limit. E_c there is still 8 percent better than v2. The internal session-2
cut has only 43 labels (V 15.93 there), so it could not show this. The rule is applied as
written: v5 does not ship.

## CPU and latency (v5 vs v2, same architecture)

Dev Mac, `racer_pnp_perception.latency`, 2000 session 2 scans, single thread, machine shared
(load about 6): net forward p50 0.103 / 0.102 ms; scan to track (net) p50 0.546 / 0.547 ms, p99
0.692 / 0.699 ms. Equal within noise, as expected: v5 only changes weights. The compare tool's p99
(0.83 vs 0.94 ms) was taken under the same load.
