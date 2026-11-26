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


## Addendum 1: D11-S4, the sum of laps 2 to 10 (owner decision 28 item 2)

Written 2026-11-23 by the implementer, before any D v1 code, any M-D1b run and any B11 run.
Constants: `evaluation/prereg/pnp-v0.6-laps-dv1-addendum-1.yaml`. Nothing above changes: every
threshold, run, case, seed, pairing, the drop rule and the registered outcome stand as written.

- **D11-S4, D v1 learns faster than D v0.** On the k runs of B11 and their paired c runs (no
  new run), the sum of the truth lap times of laps 2 to 10 per run. All of:
  1. Median over pairs of sum(k) / sum(c) at most 0.95.
  2. No pair above 1.03. A pair where either run lacks one of laps 2 to 10 counts as above 1.03.
  3. D11-S0, D11-S1, D11-S2 items 1, 2, 4, 5, 6, 7 and 8, and D11-S3 pass.

Pass: "D v1 learns faster than the schedule in sim at equal safety". Fail on items 1 or 2 only:
"D v1 does not learn faster than the schedule in sim". S0 or S1 failed, or an item of 3 failed:
"not judged". The S4 outcome is reported next to the registered outcome, never in its place.

Why 0.95 and 1.03. They are the bars the attempt already uses: 0.95 is the D-S2, D3-S2 and
D11-S2 item 3 threshold (a 5 percent gain is the smallest the project has registered as worth a
change), and 1.03 is D11-S2 item 4 (no single pairing much worse). Reusing them keeps the second
criterion from being tuned to this attempt. It is reachable but not free: D v0 rises one 0.1 m/s
step per clean lap from 0.5 m/s, so if lap time scaled with 1 / cap until the ceiling, a memory
that adds `explore_dv_mps` (0.2) per lap would sit near 0.83 and one at the ceiling from lap 2
lower still, while equal learning gives 1.0. Laps at the ceiling are bounded by the track and the
planner rather than the cap, so the real gap is smaller; 0.95 asks for a clear part of it. Item 3
asks for equal safety and no late-lap pair worse than 1.03, so a faster ramp cannot pass by
paying for it on laps 8 to 10 or in contacts. Item 3 of D11-S2 (late-lap ratio) is not needed for
S4: D v0 already reaches the ceiling control on late laps (B10), and decision 28 asks whether
learning speed alone is a gain.
