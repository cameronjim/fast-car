# Oracle track measurements and the obstacle tooling (2026-11-15, briefs 40 and 40a)

Pre-registration: `docs/notes/preregistration-pnp-v0.4.md` sections 3.5 and 5 and addendum 1
section 3.3. Constants: `evaluation/prereg/pnp-v0.4.yaml` (SHA-256 `78510c14...ca3a1d63`, checked in
code on every run) and `pnp-v0.4-addendum-1.yaml`. Code: PR 199, branch `feat/pnp-sim-variants`.
Runs: `car-data/pnp/battery/v0.4-ma2`, `-ma2-rerun`, `-ma2x`, `-ma2w-0.5`, `-ma2w-2.0`, every
attempt kept under `attempts/`. Run commit `ca78981` (M-A2) and `d6704ff` (the rest; only the report
and a `--only` option differ).

## What was built

- Sim obstacles: `sim/bridge/racer_gym_bridge/track_obstacles.py`. A solid box against the inner or
  outer wall (`at`, `side`, `protrusion_m`, `length_m`), drawn into the walls the simulator ray casts
  and the scorer's body clearance sees, for generated tracks and for a map (drawn into its grid
  along the map's centreline). The ground truth centreline moves to the midline of the free lane.
  The start pose rule (0.5 m) is enforced and refuses, it moves nothing. All four registered B5
  cases and OD build in both directions, O4 on the real map included
  (`battery_b5.py track-check`).
- The round donut needed no fix: `straight_length_m: 0.0` already gave uniform 5 cm steps and no
  duplicate points; a test pins it.
- The oracle: `tests/l5_pnp_oracle/` (test only, never installed; a test asserts the launch, the
  bringup package and the car's directories cannot name or start it). `--backend oracle` in the
  runner. Stamps and publish timing match `track_perception_node` on the same scans (a ROS test in
  both message orders).
- `tests/l5_pnp_laps/battery_b5.py` and `lap_battery_b5.py`: the D4-S0, D4-S1, D4-S2 decisions (15
  of 16, 11 of 12, thresholds from the yaml), the M-A2 reading, M-A2x and M-A2w reports, and the B5
  runner. The runner refuses B5 unless the B4 directory it is pointed at is complete and its D3-S0
  and D3-S1 are computed here as PASS, and unless `freeze-check` shows nothing changed under
  `racer_laps`, `racer_planner` or the lap scoring since B4's commit `1bdaaf8`.
- Report items without a threshold: the innovation gate rejections per lap on O1
  (`lap_node_counting.py` wraps the gate from outside, racer_laps is unchanged), planned clearance
  past each box, per case medians, mean track confidence, share of ticks at the speed cap.

## B5 was not run

B4 (PR 185) reported D3-S0 FAIL. By brief 40 and v0.4 section 5 no O1 to O4 case ran with seed 31 or
32, anywhere. `battery_b5.py b5 --b4-dir` on the real B4 directory prints both decisions and refuses.
Nothing was run on seeds 11 to 13 beyond tooling checks (below).

## Finding: case OD stalls the frozen planner

OD (round donut, lane 1.0 m, one box 0.25 m deep on the outer wall) leaves a 0.67 m gap for a 0.31 m
wide car on a 1.0 m radius turn. In sim time, counter-clockwise, seed 11, 3 laps at 0.5 m/s, with the
geometric backend and with the oracle: the planner goes BLOCKED (below the speed floor, or recovery
exhausted) in front of the box and often stalls: of 13 development runs 8 stalled without a lap, 4
completed 3 laps cleanly (some slowly, lap 1 up to 79 s) and 1 completed with a wall contact.
Geometric: 3 stalls in 7 runs; oracle: 5 stalls in 6 runs. A perfect track does not help, so this is
not perception. With a 0.15 m box both backends lap it cleanly (13.5 s a lap); at 0.20 m it
mostly passes; at 0.10 m two runs stalled at once (non monotone, so a start up effect of the planner
also shows). At the ceiling (M-A2x, OD, 6 laps) none of the three conditions completes in either
direction. Nothing in `racer_planner` or `racer_laps` was changed; this is a result for the owner:
B5 on these cases is likely to fail completion, and a lane of 0.67 m free is at the limit of this
planner. The CI canary therefore uses a 0.15 m box (passes, both backends, 0 contacts) and the
registered 0.25 m box is an expected failure (`xfail`, not strict) that keeps the expectation
unchanged.

## M-A2: does a perfect track change lap time

Cases T, S1, S2, S3, S4 of `pnp-v0.3.yaml` development, ccw and cw, seed 11, 6 laps at the ceiling,
lap memory off, car-like plant, noise on. Median of laps 3 to 6 (ground truth crossings). Runs were
real time stepped as in B4: a run with a /drive_raw gap over 40 ms or a real-time factor under 0.8
was repeated alone (up to 3 attempts, all kept; 3 runs were accepted starved after 3 attempts and
are flagged in the report). `o_S3_ccw_11` attempt 3 was a host sleep (real-time factor 0.008); it was
rerun once more alone (`v0.4-ma2-rerun`, first run kept) and that rerun is the one scored.

| case | dir | g s | o s | o / g |
|---|---|---|---|---|
| S1 | ccw | 9.02 | 9.01 | 0.999 |
| S1 | cw | 8.95 | 8.99 | 1.005 |
| S2 | ccw | 17.84 | 17.72 | 0.993 |
| S2 | cw | 17.56 | 17.55 | 0.999 |
| S3 | ccw | 22.24 | 21.84 | 0.982 |
| S3 | cw | 21.90 | 21.83 | 0.997 |
| S4 | ccw | 4.52 | 4.53 | 1.002 |
| S4 | cw | 4.29 | 4.29 | 0.998 |
| T | ccw | 13.48 | 13.66 | 1.013 |
| T | cw | 13.48 | 13.45 | 0.998 |

Median ratio 0.999, min 0.982, max 1.013 over 10 of 10 pairs; 0 contacts, all 20 runs completed.
Registered reading (median at least 0.97 and no pair below 0.93): **no measured path**. A perfect
track buys under 3 percent of lap time in sim at these speeds with this planner.

## M-A2x: the learned candidate as a third condition (diagnostic, decides nothing)

Condition f is `trackperc-v2-2026-11-14` refined, through the same message and timing. `fused-v1` is
not on main (PR 187), so it was not used.

- f over g: median 1.009, min 0.983, max 1.079 (10 of 10 pairs). f over o: median 1.011, min 1.001,
  max 1.075. The candidate is slower than the tracer on S1 (1.07, 1.05), T ccw (1.08) and S4 (1.03,
  1.02), equal elsewhere. 0 contacts, all completed.
- Mean track confidence over the M-A2 cases: g 0.994, o 0.994, f 0.999. Share of ticks held at the
  speed cap: g 0.054, o 0.056, f 0.099.
- OD, all three conditions, both directions, 6 laps at the ceiling: no run completed (f ccw 6 laps
  with one contact; the rest stalled or hit the wall). See the finding above.
- Not available: the cost terms of the planner's final sequence and the share of ticks per speed
  limiting term. PlannerStatus carries none, brief 42's PlannerTiming is not on main and
  `planner_replay_cli` writes none; the planner was not changed to produce them. The per-tick cap
  limited share above is the only limiter information there is.

## M-A2w: boundary weight at half and double (M-A2 read "no measured path")

`w_bound` default 200 (read from `mppi.hpp`), launch parameter only, no default changed.

| weight | median o / g | min | max |
|---|---|---|---|
| 0.5 x (100) | 1.000 | 0.996 | 1.005 |
| 2.0 x (400) | 0.999 | 0.988 | 1.002 |

Neither setting reaches 0.95: **not** "headroom only with other planner weights".

## Limit

A fixed-planner oracle measures the value of a better input to this planner with these weights, not
a bound on perception.

## Process notes

- An earlier M-A2 batch (`v0.4-ma2-discarded-domain-overlap`, never read as a result) was thrown away:
  its ROS domain ids (120 to 123) were the default of `battery_b4.py` too, which another agent was
  running in a second container on the same Docker network, so two stacks shared one DDS domain
  (real-time factors near 2, runs failing for no reason). All reported runs used domains 200 to 203.
  The B4 development regression that ran at the same time used domains 120 to 124 and may have been
  affected by the overlap while mine ran.
- M-A2 and the dependent measurements ran with 4 workers while the B4 agent's regression ran one.
