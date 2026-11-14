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
