# Battery B12: D v1 candidate K6 against D v0 in sim (run once), 2026-11-26

Pre-registration `evaluation/prereg/pnp-v0.7-laps-k6-b12.yaml` (SHA-256 ce8de8be...fd03), note
`docs/notes/preregistration-pnp-v0.7-laps-k6-b12.md`. Frozen at 23465e0 (freeze file 8ab482a,
main_sha_contained corrected to f6b83fd in 550f6f8 before any run), run once on 550f6f8 in the
ros-dev image (container dv1-k4), 3 workers, starved attempts rerun alone. Runs and the full report:
`car-data/pnp/battery/v0.7-b12/` (`battery_b12.py report --out ...`). 88 runs, no case dropped.

Host load during the run: 1-minute load from about 6 up to 25 at times, mostly from other
agents' jobs (training, CI); 18 starved attempts were rerun alone. The owner's later load rule
(under 10) was not met for part of this run. B12 is not rerun: it runs once.

## Outcome (registered): "unsafe in sim"

- D12-S0 PASS (bookkeeping, c and k).
- D12-S1 PASS (baseline healthy: 16 of 16 reach, median r 0.538).
- D12-S2 FAIL:
  1. contacts 0: PASS
  2. completion 16 of 16, no stall: PASS
  3. median T_late(k) / T_late(c) 0.921 (bar 0.95): PASS
  4. no pair above 1.03: FAIL (X3 ccw 93 1.089, X3 ccw 94 1.051)
  5. brakes per lap, laps 8 to 10, 0.000: PASS
  6. engagements laps 2 to 12, k 2 against c 0: FAIL. Both are in one run (k X4 ccw 93, lap 4):
     one watchdog and one rate_limit (a /drive_raw timing gap), 0 TTC engagements in all k runs.
     The registered count includes every source, so the item fails as registered.
  7. clearance laps 8 to 10, median k 0.2568 m against c 0.2902 m minus 0.03 (0.2602): FAIL by
     0.003 m.
  8. lap 1 cap and ceiling: PASS
- D12-S3 PASS, including the station bound item that failed in B11 (0 exceptions).
- D12-S4 (sum of laps 2 to 10): median 0.920, no pair above 1.03, but not judged (needs S2 items
  4, 6 and 7).

Reported, no threshold: T_late(t) / T_late(k) median 1.089 (memory needed for the gain); k
fastest on X4 (long straights, 0.715 to 0.867 of c) and X2; slower than c on X3 ccw (lane 1.2,
island 0.62, short straights), where k took 10 to 12 laps to reach 1.05 x T_late(b) against c's 8.
Every kf run reset at lap 7 (length_out_of_band_twice), as in B11.

## Reading

K6 is faster than D v0 on held-out cases (median 0.921, sum of laps 2 to 10 0.920) and fixed both
B11 failures (0 TTC engagements against B11's 122; S3 station bound now passes), but it is not
"equal safety" as registered: it drives about 0.033 m closer to the walls late in the run than D
v0, it is slower than D v0 on one wide, short case (X3 ccw), and one run had two timing
engagements under host load. X cases and seeds 93, 94 are now spent.
