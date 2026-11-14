# Scan deskew (P1.4), measurement M-DSK, 2026-11

Brief 33, design `claude-docs/13-plug-and-play.md` 15.6, pre-registration
`docs/notes/preregistration-pnp-v0.3.md` section 4 and `evaluation/prereg/pnp-v0.3.yaml` block
`deskew` (both frozen, read from main after PR 170 merged; split file SHA-256 `bc5185c9...aef83`).

## What was built

`racer_track_perception/deskew.py` (numpy core), `deskew_relay.py` (the node's decisions, ROS-free),
`scan_deskew_node.py` (relay, `/scan` to `/pnp/scan_deskewed`), `pnp.launch.py deskew:=false|true`
(default false), `tools/pnp_deskew/` and `tools/pnp_deskew_eval.py` (this measurement).

Two points where the sweep gives no answer, so they are inputs chosen on the train role and not
assumptions:

* `stamp_is`: header.stamp is the first (`sweep_start`) or the last (`sweep_end`) beam's time.
* `beam_order`: time runs with the beam index (`index_ascending`) or against it
  (`index_descending`). The Slamtec driver stores the head's clockwise sweep reversed, so the
  second case is plausible. The brief names only `stamp_is`; the pre-registration (section 4) says
  the sweep's rotation sense is chosen on the train role as well, so both are hypotheses here
  (four in all).

How a sweep gets its twist (the brief leaves this open): the wheel speed and the gated steering are
kept as short histories on their header stamps, and a sweep gets the sample at its middle, which is
`stamp + T/2` under `sweep_start` and `stamp - T/2` under `sweep_end`. Without this `stamp_is`
changes only the output stamp and not one beam of the output, and metric C, which compares scans,
could not tell the two apart (the L1 test shows the geometry does not depend on it).

The car's own parts (about a dozen beams per scan return from inside the body rectangle at about
0.17 m) are not carried: they are fixed in the car frame, and carrying them would throw them up to
`v T` out of the body.

Real scans: `scan_time` is 0.0996 s and `time_increment` 1.38e-4 s per beam (720 beams), so the
sweep takes the 0.1 s the brief says.

## Step A: selection on the train role

Pairs R2 against R1 (ccw) and R7a against R0 (cw), fast corner scans of the fast run (wheel speed at
least 0.6 m/s and wheel-model yaw rate at least 0.5 rad/s: 191 of 197 scans of R2, 118 of 134 of
R7a), C = median of the RMS of the best ICP match among the scans of the slow run (inlier share at
least 0.6), both runs processed the same way. All 309 scans scored in every condition.

| condition | R2/R1 C (m) | R7a/R0 C (m) | pooled C (m) |
|---|---|---|---|
| raw | 0.01354 | 0.01295 | 0.01342 |
| sweep_start / index_ascending | 0.01883 | 0.01939 | 0.01917 |
| sweep_start / index_descending | 0.01113 | 0.01011 | 0.01083 |
| sweep_end / index_ascending | 0.01876 | 0.01930 | 0.01896 |
| sweep_end / index_descending | 0.01111 | 0.01011 | 0.01078 |

Chosen by the rule (lowest pooled C): **`sweep_end` / `index_descending`**.

Reading of the table, none of it used to choose:

* `index_descending` is clearly right: 0.0108 against 0.019 for ascending, and against 0.0134 raw.
  Ascending is worse than raw, as the wrong sign of the sweep should be.
* The two stamp meanings differ by 0.4 percent (0.01078 against 0.01083), inside what a median of 309
  scans resolves. The rule picks `sweep_end` by that margin.
* Independent evidence on the same train segments that is NOT part of the rule: a scan message
  arrives one sweep after its header stamp (arrival minus stamp, minus `scan_time`: 1.5, 1.3, 1.3 and
  1.8 ms on R0, R1, R2, R7a). That is what `sweep_start` predicts. If the stamp were the end of the
  sweep the message would arrive a few milliseconds after it.
* The consequence of a wrong `stamp_is` is in the output stamp (0.1 s, a car pose off by `v` times
  that) and in which odometry a sweep gets, not in the shape of the scan, which is why C barely
  sees it. The applied rule gives `sweep_end`; the arrival evidence says `sweep_start`. This is an
  owner question (see the PR comment). Nothing was changed in the rule.

The choice is committed (launch constants `DESKEW_STAMP_IS`, `DESKEW_BEAM_ORDER`) before step B.

## Step B: result on the held-out role, scored once

Pairs R8 against R3 (ccw) and R7b against R4 (cw), raw against `sweep_end` / `index_descending`.
JSON next to the bags: `/Users/cameronjim/code/car-data/pnp/deskew/step_b.json`. The first run of
step B stopped at item 4 (a bug: the split names R7 as R7a and R7b, the code asked for R7) after
items 1 to 3 were computed in memory and before anything was written; nobody read it. The code was
fixed and the step run again from the start, saving after every item. This is the run reported.
Worker count does not enter the numbers (each scan is scored on its own).

| item | measured | rule | |
|---|---|---|---|
| 1 C, fast corner scans, pooled (262 scans) | raw 0.01316 m, deskewed 0.01051 m, ratio 0.799, bootstrap 95 percent interval 0.778 to 0.814 (14 blocks of 2.0 s, 2000 resamples, seed 0) | ratio at most 0.90, upper end below 1.00 | PASS |
| 2 C, fast run scans that are not fast corner scans | ratio 0.796 | at most 1.02 | PASS, on 2 scans (see below) |
| 3 `geometric-v1` on A-run (4016 moving scans, 10 held-out segments) | intrusion mean raw 0.00300, deskewed 0.00275 (excess -0.00026); deficit p90 raw 0.02078 m, deskewed 0.01995 m (excess -0.00083 m) | excess at most 0.002 and 0.005 m | PASS |
| 4 planner replay R0 to R8 (10 segments, uniform ticks, caps from the split) | NO_SCAN or FAULT ticks: raw 0, deskewed 0, none the raw replay lacks; same tick counts | none that raw does not have | PASS |
| 5 core plus rebin over 2000 real scans, this Mac | p50 0.065 ms, p99 0.22 ms, max 0.42 ms | p99 under 3 ms | PASS |

Per pair, fast corner scans (C raw to C deskewed): R8 against R3 0.01395 to 0.01109 m (172 scans),
R7b against R4 0.01265 to 0.01008 m (90 scans). Every scan found a match in both conditions and no
scan passed through.

Verdict by the pre-registered rule: all five items hold, so deskew MAY be switched on at the car, as
its own step (two runs per direction, compared with the step before). Not done here, and the car
decides: item 5 on the Orin (under 5 ms, brief 34's check), and the open `stamp_is` question.

Things the numbers do not say:

* Item 2 is weak. 172 of the 174 scans of R8 and all 90 of R7b are fast corner scans (the loop is
  7.4 m round at 0.83 m minimum radius, so a fast run is almost all corner), which leaves 2 scans
  for the check on the rest. It passes, on 2 scans.
* Item 4 is open loop: the planner reacts to what the deskewed scans would have shown at the
  recorded poses, not a lap. The chosen configuration is `sweep_end`, whose stamp offset is 0, so the
  replay does not test a shifted stamp at all. If the car runs `sweep_start` (the owner question
  below) the output stamp moves by the sweep time and the planner dead-reckons from it (design 13.4);
  that has not been replayed.
* Metric C compares scans of runs at two speeds and so shows the shape of the scan, not its stamp.
  The stamp question is not settled by it (step A).
* The simulator's scan has no sweep, so there is no L5 case and a sim lap cannot show a gain;
  deskewing an instantaneous scan would only add error. That gap is stated, not closed.

## L6 check for brief 34 (not run here)

On the Orin, with the car moving: `/pnp/deskew/status` `process_ms.p99` under 5 ms, and
`scans_passed_through` 0 (`passed_through_by_reason` all 0) while the wheel speed is above 0.02 m/s.
