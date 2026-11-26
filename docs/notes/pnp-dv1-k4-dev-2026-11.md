# D v1 K4: development candidate after K3s (unregistered), 2026-11-25

Development runs only (owner decisions 29 and 37b). Nothing here is a registered result and no
gate is judged. B11 stays "unsafe in sim". The W cases, seeds 91 and 92, and the seeds and cases
held back for a possible B12 (seeds 93, 94, cases X1 to X4, written down before the first K4 run)
were not used. Every run is a development case of `evaluation/prereg/pnp-v0.6-laps-dv1.yaml` on
development seeds 11 to 13, both directions, truth lap source, 13 laps, run from the M-D1b
commands with only the lap node and planner parameters changed, exactly as the K3s note did. Run
files and scripts (`dev_k4.py`, `summary.py`) are in `car-data/pnp/battery/v0.6-k4-dev/`. Ratios
are against the M-D1b c (D v0) run of the same case, direction and seed (code bf4ff89).

## The problem K4 tries to fix

K3s (`pnp-dv1-k3s-dev-2026-11.md`) has 0 TTC engagements on its 12 check runs but keeps only about
37 percent of K3's late-lap gain (0.927 of D v0 against K3's 0.805). Its fix is a heavier and
wider soft gate cost (w_gate 9, margin 1.3 times the 0.71 s warning time), which charges every plan
whose predicted TTC is under 0.92 s, so the car slows well before the gate would warn.

## Mechanisms tried (3)

All variants keep K3's lap memory (frozen lap-1 value profile, cap wins, station uncertainty,
reset on change, track station) and K3s's two lap node parts (TTC warning zones held for the run,
bound held after a doubtful lap).

1. **Gate barrier** (new planner switches `w_gate_barrier`, `gate_barrier_margin`,
   `gate_barrier_steps`, default off): a fixed cost for each step of the first 0.3 s of a plan
   whose predicted TTC is under margin times the warning time; nothing outside that band. The
   soft gate cost stays at its defaults (3, 1.1).
2. **Value gain** (new planner switch `lap_value_gain`, default 1): scales the value term, which
   pays about v_cap / v_lap1 (2.6 at 1.3 / 0.5) times the progress term per metre, toward it.
3. **Updating value profile** (`value_profile_mode update`, M-D1b's u) in place of the frozen
   lap-1 profile, with the K3s gate cost. Note: M-D1b's f is the FROZEN lap-1 profile, the same
   as K3 (f 0.773 and K3 0.774 of m); only u is "learned beyond lap 1".

Selection set (the K3s note's 6 runs: N1 ccw 12, S2 ccw 12, S2 cw 13, S3 cw 12, T ccw 13, T cw 11),
rule written down before the finalist ran: fewest TTC engagements, then the lower median.

| variant | median T_late / c | TTC eng laps 2-12 | brakes | contacts | clearance 8-10 median (c 0.245) |
|---|---|---|---|---|---|
| K3 on main (K3s note) | 0.767 | 52 | 0 | 0 | 0.244 |
| K3s gate cost alone (K3s note) | 0.857 | 1 | 0 | 0 | 0.232 |
| barrier w 30, margin 1.1, 6 steps | 0.998 | 0 | 0 | 0 | 0.235 |
| **barrier w 10, margin 1.0, 6 steps** | **0.906** | **0** | 0 | 0 | 0.239 |
| barrier w 20, margin 1.05, 2 steps | 0.840 | 4 | 0 | 0 | 0.229 |
| barrier w 20, margin 1.2, 2 steps | 0.816 | 1 | 0 | 0 | 0.213 |
| value gain 0.6 | 0.880 | 1 | 0 | 0 | 0.246 |
| updating profile (u) + K3s gate cost | 0.995 | 0 | 0 | 0 | 0.243 |

Check set (the K3s note's 12 other dev runs: T cw 12, T ccw 11, S2 ccw 13, S2 cw 11, N1 cw 11, N1
ccw 13, S3 ccw 13, S3 cw 11, N3 ccw 11, N3 cw 12, S1 ccw 12, S1 cw 13), the finalist only. The bar
was written down before it ran: below K3s's 0.927 on these runs AND 0 TTC engagements on all 18
selection and check runs, 0 contacts.

| variant | median T_late / c | worst pair | TTC eng laps 2-12 | brakes | contacts | clearance 8-10 median (c 0.275) | lowest clearance |
|---|---|---|---|---|---|---|---|
| K3 (M-D1b runs) | 0.805 | 1.054 | 26 | 0 | 0 | 0.241 | 0.150 |
| K3s | 0.927 | 1.101 | 0 | 0 | 0 | 0.243 | 0.156 |
| **K4** (barrier w 10, margin 1.0, 6 steps) | **0.967** | 1.077 | **1** (S1 cw 13) | 0 | 0 | 0.249 | 0.187 |

Over all 18 runs K4 is 0.937 of D v0 with 1 engagement, 0 brakes, 0 contacts. Two starved attempts
(drive_raw gap) were rerun, as in M-D1b.

## Reading

- **K4 does not meet the bar**: slower than K3s on the same runs (0.967 against 0.927) and not 0
  TTC engagements (1 warning, 0 brakes). K3s stays the prototype default; no B12 prereg was
  written and B12 was not run (decision 37b). Seeds 93, 94 and cases X1 to X4 stay unread.
- None of the three mechanisms broke the speed against TTC trade of K3s. A barrier wide enough
  (all 6 steps) to stop every warning is slower than K3s's soft margin. Likely cause (not measured): the gate
  table is built from the current pose, so its distances on steps 1 to 5 of a turning plan are
  pessimistic and the barrier charges plans the gate would never warn on. Limited to the first 2 steps it is
  fast (0.816 to 0.840 on the selection set) but lets warnings through (1 to 4 per 6 runs, TTC 0.65
  to 0.71 s) and costs clearance (median 0.229 and 0.213 against c 0.245; 0.213 is under the B11
  item 7 margin of c minus 0.03 m). Scaling the value term down (0.6) gives 0.880 with 1 warning. The updating profile
  learns the faster laps and so pays about the progress term per metre: no gain (0.995).
- The barrier needs a better prediction of the later steps (the arc table from the predicted pose,
  not the current one) to keep the speed it should; that is a planner change for a later attempt.
- Run to run spread stays large (6 to 12 runs per variant decide little).

Shipped as `dv1_variant:=K4` (lap_mode v1 only, default off, app variant select; D v0, K3s and
every other variant unchanged; planner switches default off and bit for bit as before).
