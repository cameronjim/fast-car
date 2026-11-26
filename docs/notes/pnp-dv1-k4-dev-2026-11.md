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

## K5: the barrier judged from the predicted pose (after K4)

K4's reading said the barrier over-charges because steps 1 to 5 reuse the scan table of the
current pose. K5 (new planner switch `gate_barrier_predicted`, default off) keeps step 0 on the
table (what the gate judges next) and judges steps 1 and later from the predicted pose of that
step: the commanded arc (steering clamped like the gate) is sphere traced in the planner's exact
distance field, and the step is in the band when a leading footprint circle (front body circle,
front corners) comes within the gate's corridor margin (corridor half width less the body half
width, both from vehicle_params) inside margin times warning time times the judged speed. Same
weight and margin as K4 (10, 1.0, 6 steps).

A first version marched every 0.04 m and was slow (45 sample-count halvings per run against 0 for
K4); under host load (load about 19 from other agents) 5 of its 12 check runs starved three times
and it was not judged (7 valid check runs, 0 engagements; selection 0.861, 0 engagements). The
sphere traced version (K5b) had 0 halvings and was rerun on all 18 dev runs.

Engagements below are TTC only. K5b's runs also had 7 timing engagements (4 watchdog, 3
rate_limit: /drive_raw gaps under host load, counted by B11 item 6 but not TTC); K3s's and
K4's runs had none.

| variant | runs | median T_late / c | check-set median (K3s 0.927) | paired vs K3s (check set) | TTC eng | brakes | contacts | clearance 8-10 median (c) |
|---|---|---|---|---|---|---|---|---|
| K4 | 18 | 0.937 | 0.967 | | 1 | 0 | 0 | 0.240 (0.246) |
| **K5b** | 18 | **0.873** | **0.909** | median 0.976, faster in 7 of 12 | **0** | 0 | 0 | 0.231 (0.246) |

K5b meets the bar written for K4 (below 0.927 on the check set, 0 TTC engagements on all 18, 0
contacts), but the paired gain over K3s is within the run to run spread (7 of 12, median 0.976).
After the coordinator asked that B12 wait for a version that CLEARLY beats K3s, "clearly" was
written down before the next runs: paired median K / K3s at most 0.95 and faster in at least 9 of
the 12 check runs, 0 TTC on all 18, 0 contacts. K5b does not meet it, so B12 was not run.

Where K5b loses: on the wide cases (S1, N3) K3 itself has 0 TTC warnings, yet K5b is slower than
K3 there (S1 cw 13 late laps 10.0 s against 9.2 s, the planner below the cap on 58 percent of
ticks against D v0's 33): the barrier still charges later plan steps that MPPI will replan.

## K6: the predicted barrier on the first 4 plan steps (after K5)

K6 is K5 with `gate_barrier_steps` 4 (0.2 s) instead of 6, so plan steps that MPPI replans before
they are commanded are no longer charged. All 18 dev runs, 3 workers (one run, T cw 11, starved on
3 attempts and was rerun alone, as M-D1b does).

| variant | runs | median T_late / c | check-set median (K3s 0.927) | paired vs K3s (check set) | TTC eng | other eng | brakes | contacts | clearance 8-10 median (c 0.246) | lowest |
|---|---|---|---|---|---|---|---|---|---|---|
| K5b | 18 | 0.873 | 0.909 | 0.976, faster in 7 of 12 | 0 | 7 timing | 0 | 0 | 0.231 | 0.177 |
| **K6** | 18 | **0.872** | **0.881** | **0.944, faster in 9 of 12** | **0** | 0 | 0 | 0 | 0.227 | 0.168 |

K6 meets the "clearly beats K3s" rule written before it ran (paired median at most 0.95, faster
in at least 9 of 12, 0 TTC on all 18, 0 contacts), so B12 was pre-registered and run once on its
own branch (`feat/dv1-b12`, not merged; decision 28 style).

## B12 (registered, run once): "unsafe in sim"

`docs/notes/pnp-lap-battery-b12-2026-11.md` on `feat/dv1-b12`. Cases X1 to X4, seeds 93, 94, 88
runs. S0, S1, S3 PASS (the station bound item that failed in B11 passes); S2 FAIL on item 4 (X3 ccw
pairs 1.089, 1.051), item 6 (k 2 against c 0: one watchdog and one rate_limit in one run under
host load, 0 TTC engagements in all k runs) and item 7 (clearance k 0.2568 m against c 0.2902 m
minus 0.03, short by 0.003 m); item 3 passes (median 0.921). S4 ratio 0.920, not judged. Host load
during B12 was 6 to 25 (other agents).

## K6 on the prototype stack (after B12)

`track_family_runner --pnp-profile prototype --dv1-variant K6 --plant-mirror-reverse`
(net perception, dyn-v0.7p with the fast MLP), 22 laps, one sim at a time, host load 1.8 to 4.4
for every run, same build for K6, K3s and D v0. Dev cases S2 cw, P1 ccw (11, 12), N1 ccw (12,
13); probe cases N1 ccw 11, P2 ccw 11, 12 (confirm only, run once per variant, not tuned on).
Runs in `car-data/pnp/battery/v0.7-k6-proto/`.

| run | K6 TTC warn / brake | K3s TTC warn / brake | K6 / K3s laps 8-10 | K6 / K3s laps 18-22 |
|---|---|---|---|---|
| S2 cw 11 | 0 / 0 | 2 / 0 | 1.059 | 1.097 |
| S2 cw 12 | 0 / 0 | 4 / 0 | 1.066 | 0.991 |
| P1 ccw 11 | 0 / 0 | 0 / 0 | 1.089 | 1.096 |
| P1 ccw 12 | 0 / 0 | 3 / 0 | 1.073 | 0.990 |
| N1 ccw 12 | 0 / 0 | 5 / 0 | 1.065 | 0.947 |
| N1 ccw 13 | 0 / 0 | 5 / 0 | 1.035 | 0.965 |
| N1 ccw 11 (probe) | 0 / 0 | 7 / 0 | 1.046 | 0.911 |
| P2 ccw 11 (probe) | 0 / 0 | 4 / 0 | 1.002 | 0.919 |
| P2 ccw 12 (probe) | 0 / 0 | 4 / 0 | 1.010 | 0.939 |

0 contacts everywhere. K4 for reference (proto-sim's runs, same harness, load 3 to 4): N1 ccw 11
0 events, late laps 14.5 s (K6 13.16); P2 ccw 11 2 warnings, 11.2 s (K6 10.16); P2 ccw 12 1
warning, 10.9 s (K6 10.13); S2 cw 11 0 events, 15.3 s (K6 13.03). D v0 (lap_mode v0, same
harness and build) on the probes: N1 ccw 11 0 warnings, 0 brakes, laps 8-10 15.13 s; P2 ccw 11 0, 0,
11.08 s; P2 ccw 12 0, 0, 11.04 s.

Reading: K6 passes the safety half of the coordinator's gate (0 TTC warnings and 0 brakes on all
9, probes included, where K3s has 34 warnings) and is faster than K4 everywhere, but it does not
beat K3s on the registered late-lap measure (laps 8 to 10: median 1.059, slower in 9 of 9). It is
faster on laps 18 to 22 (median 0.965; 0.991 on the 6 dev runs), mostly because K3s's warnings
make caution zones that slow its later laps. By the coordinator's rule K6 is therefore NOT the
prototype default: it stays a switch and the profile keeps K3s. Owner question: K6 trades about 6
percent of lap 8-10 speed for 0 TTC warnings on the prototype stack.
Not measured: the cost of the predicted barrier's arc tracing on the Jetson (solve p50 3.3 ms in
these sims on the Mac, 0 sample cuts).
