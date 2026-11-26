# Pre-registration: battery B12, D v1 candidate K6 against D v0 (pnp-v0.7-laps-k6-b12)

Constants: `evaluation/prereg/pnp-v0.7-laps-k6-b12.yaml` (its SHA-256 is checked by
`tests/l5_pnp_laps/lap_battery_b12.py` on every run). Written 2026-11-25 before any B12 run, after
the dev runs that chose K6 (`docs/notes/pnp-dv1-k4-dev-2026-11.md`). Owner decision 37b, extended by
the coordinator to later versions: B12 runs once, when a version clearly beats K3s on dev runs.

## What is new and what is not

New: the cases X1 to X4 and the seeds 93 and 94 (written down before the first K4 run and never
run since), and the candidate K6. Everything else is B11's (`pnp-v0.6-laps-dv1.yaml`), unchanged:
conditions a, b, c, k, t, kf, 88 runs, pairing, drop rule, validity, the B10 fault plan, every
threshold of D11-S0 to D11-S3 and D11-S4 of addendum 1. A test (`test_battery_b12.py`) compares the
blocks. The tooling is a copy of B11's (`battery_b12.py`, `lap_battery_b12.py`) with only the
paths, hashes, the block name and the candidate changed.

## The gate (pre-registered)

Battery B12, run once on the frozen branch commit (`feat/dv1-b12`, not merged). Held-out: cases X1
to X4 with seeds 93 and 94, both directions. Truth lap source. 88 runs: a (8, seed 93), b, c, k,
t, kf (16 each). Pairing is same case, direction and seed. T_late is the median truth lap time of
laps 8 to 10. Drop rule as B11 (one case may drop, three must remain; counts then read "of 12").

- **D12-S0, bookkeeping.** D10-S0 items 1 to 5, unchanged, on every c and k run.
- **D12-S1, the baseline is healthy.** D10-S1 on the c runs: 0 contacts, 15 of 16 complete 10
  laps, T_late(c) at most 1.05 times T_late(b) in 15 of 16, median r at most 0.80. Fail means
  "no result: the D v0 baseline failed". K6 is then not judged.
- **D12-S2, K6 against D v0 at equal safety.** All of:
  1. 0 wall contacts in the k runs.
  2. At least 15 of 16 k runs complete 10 laps, and not fewer than the c runs. No k run stalls
     for more than 5 s.
  3. Median over pairs of T_late(k) / T_late(c) at most 0.95.
  4. No pair above 1.03.
  5. Mean TTC or obstacle brakes per lap over laps 8 to 10 at most 0.2.
  6. Safety engagements (`PHASE_ENGAGE` on `/safety/events`, laps 2 to 12, every source) summed
     over the k runs at most the sum over the paired c runs.
  7. Median over k runs of the lowest body to wall clearance on laps 8 to 10 at least the c
     runs' median minus 0.03 m.
  8. The speed command never exceeds the lap 1 cap on lap 1 or the ceiling at any time (0.01).
- **D12-S3, the candidate under lap source faults** (kf, B10's fault plan): 0 contacts, 15 of
  16 complete 12 truth laps, brakes per lap at most 0.2 over all laps, D10-S2 items 5 and 6
  unchanged, and no station's speed bound in the lap after a doubtful lap above its bound in
  the doubtful lap.
- **D12-S4, learning speed** (addendum 1 D11-S4, unchanged): median over pairs of the sum of
  laps 2 to 10, k over c, at most 0.95, no pair above 1.03, with S0, S1, S2 items 1, 2, 4 to 8
  and S3.

Pass is S0, S1, S2 and S3 together: "D v1 (K6) faster than the schedule in sim at equal safety".
Fail on item 3 only: "no better than the schedule in sim". Fail on a safety item or S3: "unsafe
in sim". If T_late(t) is within 2 percent of T_late(k), the outcome also says "gain not shown to
need memory". A run that cannot execute after 3 attempts counts as failed. Reported with no
threshold: the ratios to b and t, the sum of laps 2 to 10, laps to reach the ceiling, resets, and
item 6 split by source (TTC against timing events). After B12 the X cases and seeds 93, 94 are
spent. B11's result ("unsafe in sim" for K3) stands.
