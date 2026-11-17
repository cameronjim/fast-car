# Lap gate v3: development acceptance (brief 50, PR A), 2026-11-17

Pre-registration: `docs/notes/preregistration-pnp-v0.5-laps-ring.md`, constants
`evaluation/prereg/pnp-v0.5-laps-ring.yaml` (SHA-256
`74b1dccd0d30910a7e96a45af65a00de62446902380bfb2bae76512d0498dc36`), committed alone as commit 1.
Everything below is development: no R or Q case, no seed 61 to 63, nothing under `v0.5-b7/` was run
or opened. Development runs are in `car-data/pnp/battery/v0.5-ring-dev/` (A5) and
`v0.5-ring-dev-dumps/` (the fixture and a replay of ND). Code under test: gate v3 at the head of
`feat/pnp-lap-keyframe-ring`, constants of the brief's table untouched.

Result: A1, A2, A3, A4, A6 hold. **A5 fails** with the design built as written. By the stop rule of
the pre-registration (7b) there is no freeze and no B8 run until the launcher decides.

## A1: the synthetic ring on gate v2.1 (premise)

F1 (`tests/test_gate_v3_f.py`, island 0.40 m, lane 1.1 m, 0.5 m/s, exact odometry, 5.97 m round,
11.94 s per lap): on main (71ae56d) lap 1 closes at 10.95 s, 91.7 percent of a lap (the brief:
before 95 percent), and four laps are counted before the fourth true lap is complete. **Red on
main, so the premise holds and the work went on.** On gate v3: 12 laps for 12, each wheel length
within 2 percent of the circle, every `line_source` `winding`, `phase_observable` false; with the
heading gain off by plus and minus 3 percent every lap is within 4 percent and none is gained or
lost. The second F test (every scan within 0.6 m of the line of lap 3 has odometry 0.35 s stale)
was red on main as well: lap 3 lost, the next closure flagged missed; on v3 it closes by `anchor`
within 0.05 s of the clean run's crossing.

## A2: existing tests and goldens

| suite | result |
|---|---|
| `racer_laps` (with the replay extra) | 427 passed, 2 skipped, line coverage 90.03 percent (gate 90) |
| `tests/l5_pnp_laps` (no ROS) | 405 passed, 1 skipped; `lap_battery_b8.py` 98 percent, `battery_b8.py` 94 percent |
| `tools/tests` lap replay, pnp_session | 190 passed, 1 skipped; `pnp_session` package 97 percent |
| `tests/l5_reactive_bench/test_registered_truth.py` | 14 passed |

No tolerance, golden or coverage gate was changed. With the defaults on the L4 car fixtures
(`fixtures_car_2026_11_16`: 4 laps, 0 missed, cap unchanged; the parked last lap closes), the M-R
golden, the T2 fixture and the alias fixture pass unchanged. Stadium crossings of two synthetic
drives equal gate v2.1's exactly with the ring and uniqueness off and to the printed digits with
them on (goldens captured from main, `test_gate_v3.py`).

Changed expectations (setup, not thresholds), listed with the reason in the PR comment:

- `tests/test_gate.py::make_gate` and `tests/scripted_car.py::Car` now pass `ring_enabled=False,
  unique_shift_m=0.0`. Those fixtures stand a scripted matcher in for the start-scan match and send
  a scan or two near the line; with the uniqueness test on there is no displaced scan 0.4 m back, so
  nothing is unique and nothing would close (the brief's own rule). They test gate v2.1 mechanics,
  which the flags select exactly; v3 has its own tests.
- `test_l4_alias_fixture.py::test_the_design_starting_limit_is_too_loose_on_this_input` passes the
  same two flags: v3's uniqueness test rejects the 2 m alias by itself, so with 0.04 it no longer
  closes early. A companion test asserts that v3 closes lap 1 at the line even with 0.04.

## A3: the 13 dumps of `v0.5-lap3-dev`

Every lap time is identical to v2.1's (largest difference 0.000 s), every `line_source` is
`start_scan`, start alias share 0.000 in every lap, 5 laps each: PASS (13 of 13, N1 ccw and N2 cw).

## A4: the two car bags, full replay

| bag | laps v3 | laps v2.1 | largest lap difference | missed | sources | alias share |
|---|---|---|---|---|---|---|
| 2026-11-16T22-38-47 | 20 | 20 | 0.000 s | 0 | start_scan x20 | 0.000 |
| 2026-11-16T02-18-54 | 20 | 20 | 0.000 s | 0 | start_scan x20 | 0.000 |

PASS.

## A5: development sim runs (conditions a and c, both directions)

P4 seeds 11 to 13 (12 runs), OD and ND seed 11 (8 runs), 13 truth laps each, B7 command, aliased
class tolerance (the larger of 0.3 s and 4 percent of the lap). Starved attempts were rerun alone
(my own concurrent jobs starved five first attempts; every attempt is kept in `attempts/`).

| run | truth laps | system laps | count rule | judged laps outside | largest error s | sources | missed |
|---|---|---|---|---|---|---|---|
| a_ND_ccw_11 | 13 | 12 | ok | none | 0.59 | winding 12 | none |
| a_ND_cw_11 | 13 | 12 | ok | none | 0.48 | winding 12 | none |
| a_OD_ccw_11 | 3 | 3 | no truth crossing 12 | none | 0.09 | start_scan 3 | none |
| a_OD_cw_11 | 1 | 1 | no truth crossing 12 | none | 2.17 | winding 1 | none |
| a_P4_ccw_11 | 13 | 12 | ok | lap 1 | 0.65 | winding 12 | none |
| a_P4_ccw_12 | 13 | 12 | ok | lap 1 | 0.64 | winding 12 | none |
| a_P4_ccw_13 | 13 | 12 | ok | lap 1 | 0.63 | winding 12 | none |
| a_P4_cw_11 | 13 | 13 | ok | none | 0.49 | winding 13 | none |
| a_P4_cw_12 | 13 | 13 | ok | none | 0.48 | winding 12 | none |
| a_P4_cw_13 | 13 | 13 | ok | none | 0.45 | winding 12 | none |
| c_ND_ccw_11 | 13 | 13 | ok | none | 0.56 | winding 8, anchor 5 | none |
| c_ND_cw_11 | 13 | 13 | ok | laps 1, 3 to 9 | 1.45 | winding 3, start_scan 3, anchor 7 | lap 5 |
| c_OD_ccw_11 | 13 | 13 | ok | none | 0.38 | start_scan 12, anchor 1 | laps 8, 9 |
| c_OD_cw_11 | 1 | 1 | no truth crossing 12 | none | 1.79 | winding 1 | none |
| c_P4_ccw_11 | 13 | 13 | ok | lap 1 | 0.63 | winding 13 | none |
| c_P4_ccw_12 | 13 | 13 | ok | lap 1 | 0.87 | winding 13 | none |
| c_P4_ccw_13 | 13 | 13 | ok | lap 1 | 0.62 | winding 13 | none |
| c_P4_cw_11 | 13 | 13 | ok | laps 1, 3, 10 | 0.51 | winding 13 | none |
| c_P4_cw_12 | 13 | 13 | ok | lap 3 | 0.47 | winding 13 | none |
| c_P4_cw_13 | 13 | 13 | ok | lap 10 | 0.47 | winding 13 | none |

**FAIL.** 20 runs; the count rule holds in all 17 runs that drove 13 laps and cannot be applied to
the three OD runs that stalled; 19 of 209 judged laps are outside the class tolerance (9.1 percent
against at most 2 percent). What the numbers say:

1. **The count rule works.** On every ring and near ring run that completed, 12 crossings by truth
   crossing 12 plus half a lap, no lap gained or lost, no missed gate, in both conditions.
2. **Lap 1 of a ring closes late by the heading alone.** Counter-clockwise P4 runs: system lap 1 is
   13.16 s against a truth lap of 12.51 s (+0.65 s, 5.2 percent; the next laps are +0.06 to +0.28 s),
   the same in all six ccw runs (a and c). The tolerance is 4 percent of the lap and lap 1 has no
   allowance. The winding line is where the gated heading reaches 2 pi: lap 1 starts from a standstill
   at the lap-1 cap and the heading read over that lap is about 5 percent under the plant's. No
   constant of the brief's table acts on that (`anchor_wait_m` only decides when the winding crossing
   is announced, not where it is placed), so a retuning cannot fix A5.
3. **Near ring (ND), condition c, clockwise: v3 is worse than v2.1.** On the same input
   (`c_ND_cw_11`, replayed from its dump) gate v2.1 places every crossing within 0.3 s of the truth
   except lap 1 (-0.84 s) and lap 10 (-0.51 s). Gate v3 calls the start scan aliased on part of the
   laps (more than a quarter of its near-line matches are ambiguous, the straight is 0.3 m), and
   then places laps by winding or by an anchor, with errors of -0.77, -0.57, -1.58 and +0.81 s. The
   design's trade (an honest 4 percent against a precise scan) is the cause, not a defect of one
   rule: where a scan can place the line, v3 gives it up when the verdict says aliased.
4. **The three OD runs that stall** (a ccw 3 laps, a cw 1 lap, c cw 1 lap; c ccw completed 13):
   `stopped_because: stalled`, the car stopped making progress at the obstacle (one wall contact, in
   c cw). The lap node only counts laps, so this is not the gate; I did not look into the planner.
   Two of them were also starved on their first attempt by load I made; the reruns stalled the same.

Replay of `c_ND_cw_11` with other settings (diagnosis only; the registered design and the brief's
constants are what this note scores, and none of these is applied): `kf_seed_yaw` (below) leaves the
errors as they are; `kf_spacing_m` 0.25 with `kf_window_m` 0.15 brings the anchor laps to within
0.35 s except laps 1 and 3 (lap 1 +0.48, lap 3 -0.77). Lap 1 of the ccw P4 runs does not depend on
any of them.

## A6: cost over the replays of A3 and A4 (development machine idle)

All 22 803 scans after the start scan of the 13 dumps and the two bags:

| gate | mean ms | p99 ms |
|---|---|---|
| v3 | 4.76 | 14.93 |
| v2.1 (measured the same way) | 2.95 | 13.84 |

Limits: mean at most 12 ms, p99 at most 25 ms: PASS. Armed scans of the car bags alone: mean 12.5 to
12.7 ms, p99 16.2 to 16.4 ms (v2.1: 11.2 to 11.3 ms, 16.2 to 16.3 ms). Memory: a keyframe is
about 8.3 kB in float32, 120 of them 1.0 MB (a test). The Orin Nano is not measured: item (v) of
D8-C0 is.

## `pnp_session lap-c0` on the two 2026-11-16 bags (`--replay`, REGRESSION, NOT A GATE)

Not the registered procedure (not observe mode, not one per direction): a regression record that
decides nothing. The reference validity rule, applied to the camera counter, says what the notes
said before: bag `T22-38-47` has 20 reference laps against a wheel count of 20 (valid), bag
`T02-18-54` has 10 reference laps (median lap distance 7.76 m: the camera missed every other lap)
against a wheel count of 13 (not a reference). D7-C0 and D8-C0 are therefore REPEAT for the pair,
D3-C0 FAIL (the count differs). For `T22-38-47` alone every lap of the replay is `start_scan`, none
missed, laps 2 to 20 within 0.14 s of the camera's, and the crossing instants sit a constant
0.28 to 0.42 s before the camera's (the offset lap 1 is judged against). Recorded events (without
`--replay`) are not in these bags: the lap node did not run, so the items read INSUFFICIENT.

## What the build found that the brief did not say

- **No displaced scan is "unknown", not "ambiguous".** Counting a match with no scan 0.4 m back as
  ambiguous made a stadium aliased as soon as scans dropped out (a property test with 75 percent of
  scans lost: winding fired on a stadium and closed a lap 2 m short, because heading reaches 2 pi
  at the end of the last bend, 1.5 m before the line). Built as: not unique (it cannot place a
  crossing) and not counted in the verdict. Cost: with heavy scan loss the line passage is lost
  (a missed gate, as in v2.1 at lap 2 in the same test) instead of invented. The speed envelope
  matters: the displaced scan must be within 0.1 m of W - 0.4 m, which needs scans less than 0.2 m
  apart: at 10 Hz that is 2.0 m/s.
- **The keyframes of the last 40 percent of a lap are never matched** (the start scan is tracked
  from 62 percent of a turn on, and the brief says no keyframe while it is tracked). They stay
  live and are never visited, so the re-teach rule ("live keyframes, no fix on two laps") counts
  visited keyframes only, or a ring would be taught again every second lap.
- **The zero-yaw seed of the first keyframe match biases the anchor.** On a curved ring ICP from a
  seed with the right along offset and yaw zero pulls the along offset toward zero (a displacement
  d reads as about d / 3). The anchor laps of the one-box ring are 1.9 percent short on lap 2 and
  the next laps lose their keyframes (the station moved by 0.14 m). With the yaw the odometry
  predicts (the keyframe's stored heading against the heading now) fixes are within 0.02 m and
  every anchor lap is within 0.1 percent. Built as written (default off); the variant is
  `GateConfig.kf_seed_yaw`, tested, not registered, not used by any run here.
- B7's raw runs were not opened. The note `lap-gate-lap3-2026-11.md` was not edited: its column
  "line - truth m" before the line is the table's own artefact (`total_at` asked for a truth
  crossing in the future), as the brief says.

## Not done

Nothing after A6: no freeze file, no B8 case, no seed 61 to 63. PR 207 (the B7 result) was still
open when this branch was cut from `origin/main` (71ae56d); the two PRs touch no common file.
