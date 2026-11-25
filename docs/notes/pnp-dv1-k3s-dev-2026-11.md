# D v1 K3s: a development fix for K3 after B11 (unregistered), 2026-11-25

Development runs only (owner decision 29). Nothing here is a registered result and no gate is
judged. B11 (`pnp-lap-battery-b11-2026-11.md`) stays "unsafe in sim". The W cases and seeds 91, 92
were not used. Every run is a development case of `evaluation/prereg/pnp-v0.6-laps-dv1.yaml`
(T, S1, S2, S3, N1, N3) on development seeds 11 to 13, both directions, truth lap source, 13 laps,
run from the M-D1b commands with only the lap node and planner parameters changed. Run files,
traces and the scripts (`dev_k3s.py`, `diag.py`, `summary.py`, `s3check.py`) are in
`car-data/pnp/battery/v0.6-k3s-dev/`. Ratios are against the M-D1b c (D v0) run of the same case,
direction and seed (code bf4ff89; c was not rerun on this main). Starved attempts were rerun.

## Diagnosis (K3 on main, 6 dev runs with traces, 66 TTC events)

- Every K3 engagement is a TTC **warning** (advisory, no command change): TTC 0.63 to 0.71 s
  against the 0.71 s warning band; none reached the 0.60 s brake.
- Where: inside the corners, at turn-in and mid corner (path curvature about 1.0 to 1.2 1/m), car
  centre 0.42 to 0.50 m from the nearest wall, so mid lane, not near a wall. Speed 0.85 to 1.17 m/s.
- When: laps 6 to 13, rising each lap (laps 6-8: 9, 9-10: 19, 11-13: 38). The bound ratchets up by
  the exploration step every lap, and the value term (the slow lap-1 cost to go, times the cap)
  rewards about 2.6 times what the progress term does, so the planner drives at the bound. Its
  gate aware cost (weight 3, margin 1.1 times 0.71 s) is calibrated for the progress term and is
  too weak against that push. D v0 never enters the band.
- Mostly ccw on these cases (S2 ccw, T ccw, N1 ccw); several cw runs had none.
- S3 station bound item: reproduced on dev kf runs (K3, B10 fault plan): 2 of 2 runs, up to 0.011
  m/s above the doubtful lap.

## Mechanisms tried

1. Planner gate cost (no code): `w_gate` 9 (3x), and `w_gate` 9 with `gate_ttc_margin` 1.3.
2. TTC warning zones (lap node, new switch `ttc_warning_zone_laps`): a warning makes a caution
   zone (speed driven there less 0.1, no exploration) held for the run.
3. Bound hold (lap node, new switch `bound_hold_after_doubtful`): the lap after a doubtful lap
   publishes no station bound above what the doubtful lap published in that 0.10 m bin.

Selection set (6 runs: N1 ccw 12, S2 ccw 12, S2 cw 13, S3 cw 12, T ccw 13, T cw 11):

| variant | median T_late / c | TTC engagements laps 2-12 | brakes | contacts | median clearance laps 8-10 (c) |
|---|---|---|---|---|---|
| K3 on main | 0.767 | 52 | 0 | 0 | 0.244 (0.245) |
| K3 + w_gate 9 | 0.825 | 6 | 0 | 0 | 0.238 |
| K3 + w_gate 9, margin 1.3 | 0.857 | 1 | 0 | 0 | 0.232 |
| K3 + w_gate 9 + TTC zones | 0.779 | 8 | 0 | 0 | 0.234 |

The zones stop repeats at most corners but not on N1 (long straights, the planner turns in at 1.1
to 1.2 m/s; a 0.1 m/s lower bound is not enough), so they do not get to 0 alone.

Check set (12 other dev runs: T cw 12, T ccw 11, S2 ccw 13, S2 cw 11, N1 cw 11, N1 ccw 13, S3 ccw
13, S3 cw 11, N3 ccw 11, N3 cw 12, S1 ccw 12, S1 cw 13), rule fixed before the second variant ran:
fewest TTC engagements, then the faster median.

| variant | median T_late / c | worst pair | sum laps 2-10 / c | TTC eng laps 2-12 | TTC brakes | contacts | clearance 8-10 median (c 0.275) | lowest clearance |
|---|---|---|---|---|---|---|---|---|
| K3 (M-D1b runs) | 0.805 | 1.054 | 0.861 | 26 | 0 | 0 | 0.241 | 0.150 |
| w_gate 9 + zones + hold | 0.830 | 1.122 | 0.891 | 7 | 1 | 0 | 0.243 | 0.186 |
| **K3s**: w_gate 9, margin 1.3 + zones + hold | **0.927** | 1.101 | 0.924 | **0** | 0 | 0 | 0.243 | 0.156 |

Bound hold on dev kf runs (K3 + hold, T ccw 11, S2 cw 12): 0 station bins above the doubtful lap
(K3 alone: 2 of 2 runs above), 0 contacts, 13 laps each. One watchdog brake (a /drive_raw timing
gap while the host was loaded), no TTC brake.

## Reading

- **K3s reaches 0 TTC engagements on the 12 check runs and 1 on the 6 selection runs, but it keeps
  only about 37 percent of K3's late lap gain** (median 0.927 of D v0 against K3's 0.805 on the
  same runs). So the target "most of the gain with 0 engagements" is NOT met: the gain and the
  engagements come from the same push into the corners. The variant that kept most of the gain
  (w_gate 9 + zones, 0.830) still had 7 engagements and one TTC brake.
- Clearance: K3s median on laps 8 to 10 is 0.243 m against c 0.275 m (K3 0.241): the B11 item 7
  margin (c minus 0.03 m) would be missed by about 0.002 m on these runs; T cw 12 is 0.157 m.
- On the wide cases (N3, S1) every K3 variant is slower than D v0 (up to 1.10 to 1.12).
- Run to run spread is large (the same K3 run on main and in M-D1b differ by up to 20 percent), so
  6 to 12 runs per variant decide little.

Shipped as `dv1_variant:=K3s` (lap_mode v1 only, default off; D v0 and every other variant
unchanged). A registered claim would need a new battery (B12) with its own prereg, held-out cases
and seeds and owner approval; proposed, not started.
