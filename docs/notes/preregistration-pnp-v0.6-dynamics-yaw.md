# Pre-registration: Plug-and-Play Racer v0.6 dynamics, yaw-only residual R6 (gate GP2g-R)

Written by the designer on 2026-11-23 after owner decision 27, before any fit of this attempt.
Owner decision 31 (2026-11-24) approved GP2g-R and GP2g-C as written. No result may change this
file. A later correction is a new dated section appended at the end.

Constants file: `evaluation/prereg/pnp-v0.6-dynamics-yaw.yaml`, SHA-256
`db62eaac18ba655bf00e4ff7c8e8b287cd7e042b6ea671014ebb404032d00e0c`. Code checks this SHA on every run.

Every earlier prereg file and result is unchanged and stands: GP2a FAIL, GP2c-Y FAIL, GP2c-R FAIL,
GP2d-N FAIL, GP2d-R FAIL (P1 0.9517 against 0.95), GP2e-T FAIL, GP2f-S FAIL, GP2f-T FAIL.

## The gate (pre-registered, copied from brief 54 unchanged)

**GP2g-R.** Candidate R6 (yaw-only residual on dyn-v0.1, schema 1.x) against dyn-v0.1 with the
residual off. Train: every v0.5 train role plus the first 55 percent of each 2026-11-22 run.
Selection (never fitted on): the 2026-11-14 and 2026-11-15 held-out sets, both read before.
**Held-out: the last part of each 2026-11-22 run (R0 to R4, after a 2.0 s guard), about 220 s,
both directions, read by no dynamics model.** Truth T2. Three seeds. One pass.

0. Enough data: at least 400 windows, 150 per direction, 4 strata with 30. If not:
   INSUFFICIENT, which is not a pass.
1. P1 at most 0.95 times dyn-v0.1's, bootstrap upper end of the ratio below 1.00.
2. Y1 at most 0.90 times. S1 equal within 1e-6.
3. Every seed: P1 at most 1.00 times.
4. Every stratum with 30 windows: position RMSE at 1.0 s at most 1.05 times.
5. The 100000 rollout sweep of the validity box: finite, inside the state bounds.
6. Yp at most 1.00 times on G1b and T2b (a re-read).
7. Each direction by itself: P1 at most 1.00 times, Y1 at most 1.00 times.
8. C1 at most 1.00 times, A1 at most 1.02 times.
9. Near-limit windows (each side with 30) and transition windows (if 30): at most 1.00 times.
   Fewer windows: "cannot be applied", reported, not a fail.
10. Under T2s: P1 at most 1.00 times.
11. Residual output at or above 0.9 of its bound on at most 1 percent of held-out steps.
12. `mppi_bench` p50 solve time with R6 at most 1.25 times dyn-v0.1's.
13. L5 PnP canary with the R6 bundle: every case completes, 0 wall contacts.

Items 1, 3, 4, 5, 6 and S1 are GP2c-R and GP2d-R unchanged; the rest are added or tighter.
Pass: "candidate: yaw-only residual beats dyn-v0.1 on fresh real driving". Fail: the residual
stays off, dyn-v0.1 stays, the table is the result. Validity stays 1.3 m/s in every case.
Named outcome, reported either way, not a deployment rule: "improves heading, not position"
(v0.4 section 2.4, unchanged). GP2a, GP2c-R and GP2d-R stay FAIL.

**GP2g-C, before it drives** (car, later): planner rollouts equal the Python reference within
1e-9; on at least 120 s of fresh driving on dyn-v0.1 (40 s per direction, bags after
2026-11-22) P1 and Y1 at most 1.00 times; then 5 laps per model per direction at cap 1.0,
interleaved, 0 contacts, no extra safety brake, mean wheel speed at least 0.98 times.
