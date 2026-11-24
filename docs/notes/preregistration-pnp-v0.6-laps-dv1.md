**Pre-registration: D v1 against D v0 in sim (development ablations M-D1b, the candidate rule, battery B11, gates D11-S0 to D11-S3).**
Written 2026-11-23 by the designer after owner decision 27, before any D v1 code of this attempt. Constants: `evaluation/prereg/pnp-v0.6-laps-dv1.yaml` (SHA-256 `a75b088fb15f791d0ba23087708e0e8ebdfbbe759af6e4342e620f1b38d06c4b`). The section below is copied word for word from brief 53.

## The gate (pre-registered; copy this section into the note unchanged)

Battery B11, run once on the frozen branch commit. Held-out: cases W1 to W4 with seeds 91 and
92, both directions, never run before the freeze. Development is seeds 11 to 13 on cases read
before. Truth lap source. 88 runs: a (8), b, c, k, t, kf (16 each). Pairing is same case,
direction and seed. T_late is the median truth lap time of laps 8 to 10. Drop rule as B10 (one
case may drop, three must remain; counts then read "of 12").

- **D11-S0, bookkeeping.** D10-S0 items 1 to 5, unchanged, on every c and k run.
- **D11-S1, the baseline is healthy.** D10-S1 on the c runs: 0 contacts, 15 of 16 complete 10
  laps, T_late(c) at most 1.05 times T_late(b) in 15 of 16, median r at most 0.80. Fail means
  "no result: the D v0 baseline failed". D v1 is then not judged.
- **D11-S2, D v1 against D v0 at equal safety.** All of:
  1. 0 wall contacts in the k runs.
  2. At least 15 of 16 k runs complete 10 laps, and not fewer than the c runs. No k run stalls
     for more than 5 s.
  3. Median over pairs of T_late(k) / T_late(c) at most 0.95 (the D-S2 and D3-S2 threshold).
  4. No pair above 1.03.
  5. Mean TTC or obstacle brakes per lap over laps 8 to 10 at most 0.2 (unchanged).
  6. Safety engagements (`PHASE_ENGAGE` on `/safety/events`, laps 2 to 12) summed over the k
     runs at most the sum over the paired c runs.
  7. Median over k runs of the lowest body to wall clearance on laps 8 to 10 at least the c
     runs' median minus 0.03 m.
  8. The speed command never exceeds the lap 1 cap on lap 1 or the ceiling at any time (0.01).
- **D11-S3, the candidate under lap source faults** (kf, B10's fault plan): 0 contacts, 15 of
  16 complete 12 truth laps, brakes per lap at most 0.2 over all laps, D10-S2 items 5 and 6
  unchanged, and no station's speed bound in the lap after a doubtful lap above its bound in
  the doubtful lap.

Pass is S0, S1, S2 and S3 together: "D v1 faster than the schedule in sim at equal safety".
Fail on item 3 only: "no better than the schedule in sim". Fail on a safety item or S3: "unsafe
in sim". If T_late(t) is within 2 percent of T_late(k), the outcome also says "gain not shown
to need memory". A run that cannot execute after 3 attempts counts as failed. Reported with no
threshold: the ratios to b and t, the sum of laps 2 to 10, laps to reach the ceiling, resets.
After B11 the W cases and seeds 91, 92 are spent. D-S0, D-S1, D3-S0, D3-S2(d) stay FAIL.

