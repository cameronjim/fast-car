# Lap gate v3: development acceptance (brief 50 and its revision 50b, PR A), 2026-11-17

Pre-registration: `docs/notes/preregistration-pnp-v0.5-laps-ring.md`, constants
`evaluation/prereg/pnp-v0.5-laps-ring.yaml` (SHA-256
`74b1dccd0d30910a7e96a45af65a00de62446902380bfb2bae76512d0498dc36`), committed alone as commit 1 and
not edited since. Owner decisions given (items 11 and 14 of `owner-decisions-2026-11-16.md`) are in
`preregistration-pnp-v0.5-laps-ring-appendix-2026-11-17.md`. Everything below is development: no R or
Q case, no seed 61 to 63, nothing under `v0.5-b7/` was run or opened.

**Two rounds.** Round 1 (brief 50 as written, gate at `a90f38a`): A1 to A4 and A6 held, A5 failed;
its numbers are kept below under "Round 1". Round 2 (brief 50b, design 20.5, this PR's head): the
winding counts position, the alias verdict belongs to the place, one source per track. All six checks
rerun on the revised gate; **A1 to A6 hold** (round 3 below reruns them with C1 and C2). Development runs of round 2 are in
`car-data/pnp/battery/v0.5-ring-dev-50b/` (with the lap node's dumps); round 1's in
`v0.5-ring-dev/` and `v0.5-ring-dev-dumps/`.

## Round 3 (brief 50c: the box ring RD), the head of this PR

Round 2 left RD (a ring with one box the start scan sees) failing in two condition c runs. Root cause
(50c): the uniqueness test booked a near miss of STRONG as proof. Now a match is unique only if the
displaced scan, at the same pose, is worse than the match itself by at least `unique_gap` of
free-space violation (`gate._classify`); the displaced scan is the buffered one nearest 0.4 m back among
those more than 0.3 m back (above 2.0 m/s the nearest further back instead of none).

**Tuned constants (listed, under 50c's rule):** `unique_gap` 0.0325 (brief 0.035): at 0.035 a stadium
with a 5 m straight (gaps 0.029 to 0.039, gate v2's long straight tests) latched aliased and wound; 0.0325
keeps that stadium, the box ring (alias gaps up to 0.031) and the placed tracks (0.040 and more).
`proof_min_unique` 4 (50b: 6): at 3.0 m/s a stadium approach gives 3 to 4 matches, lap 1 waited 1.5 s
and was placed 0.8 s late; all 28 RD, ND and P4 dumps are as clean with 4. The displaced scan must be
*more than* 0.3 m back (not 0.3): the brief's table has 0.30 m spacing giving 0.6 m back; at exactly 0.3
the nearer scan would have been taken, whose smaller displacement shows a smaller gap.
The L3 harness is back at 3.0 m/s (and passes there).

**Red first** (`3514a27`, on the head before the fix): the near-miss classification (unique, should be
ambiguous), the RD ccw dump (`start_scan` on lap 1) and a stadium at 3.0 m/s (2 of 3 laps) were red.

| | result | numbers |
|---|---|---|
| A1 | PASS (as recorded) | F1 red on main, 91.7 percent of a lap |
| A2 | PASS | racer_laps 507 passed, 2 skipped, coverage 90.65 percent; L3 node tests 26 passed in ros-dev at 3.0 m/s; stadium, alias and car fixtures unchanged |
| A3 | PASS | 13 of 13 dumps identical to v2.1 (0.000 s); smallest unique gap per dump 0.040 to 0.064, no ambiguous match |
| A4 | PASS | 20 and 20 laps, 0.000 s from v2.1, 0 missed; the displaced scan does not fit at all (no gap) |
| A5 | PASS | live, 16 P4 and ND runs (OD dropped by the rule), count rule in all, 0 of 192 laps outside, largest error 0.20 s |
| A6 | PASS | mean 5.56 ms, p99 17.14 ms over 22,803 scans (v2.1 3.24 and 15.46; host not idle) |
| C1 | PASS | 12 fresh RD runs: 13 laps each, count rule holds, 0 of 144 outside, none missed, one source (winding) per run, largest error 0.21 s |
| C2 | PASS | the 12 recorded RD dumps: all winding, none outside, none missed, largest errors 0.06 to 0.30 s |

Violation gaps (reported, no rule): the lane between a real place and an alias is thin. Placed tracks
A3: smallest unique gap 0.040 to 0.064 per dump; below 0.045 in `c_N1_ccw_21__r2` (0.041), `c_N1_ccw_22` and its `__r3` and `__r4` (0.043) and `c_N1_ccw_23` (0.040); no ambiguous match in any. RD: largest ambiguous gap 0.031
(`c_RD_ccw_11`), above the brief's 0.030 mark. ND (a near ring that is latched aliased in every run):
unique gaps from 0.033 and ambiguous up to 0.031, so ND sits on `unique_gap` itself; the verdict
needs a quarter of matches ambiguous, which it has (latched in every run). The stadium with a 5 m
straight sits at 0.029 to 0.039 and is the reason for 0.0325.

Other: the ring cost guard of `test_gate_v3_ring.py` was a wall-clock bound (60 ms) that read 72 ms on a
slow shared runner; it now counts ICP calls per scan (never a keyframe search, at most 8 per scan, mean
at most 4). Not a regression: A6 above is 5.56 mean and 17.14 p99 here. A guidance-property test
(`test_guidance_property.py`, seed 4) failed on main and on this PR for a float rounding at a caution
zone's edge; fixed in `station_set.py` (test red first).

## Round 2 (brief 50b)

### Red first

Three F tests (`tests/test_gate_v3b_f.py`) were committed red on PR 210's head (`6e99c4d`) before
the fix: a ring whose car starts turned 0.23 rad against the tangent closes lap 1 3.3 percent late
(12.33 s of 11.94 s; clockwise the same); with the steering gain 3 percent high or low every later
lap is 3.1 and 3.3 percent off; two agreeing unique matches before the line closed by `start_scan`.
Green on the revised gate.

### Item 4: the prototype table, reproduced by the built gate (stop rule: it must be)

Lap time minus truth, every lap by `winding` (`tests/test_l4_ring_fixture.py`, four fixtures cut by
`fixtures_ring/make_ring_fixture.py` from the dumps, 0.4 to 1.6 MB):

| dump | laps | prototype | built gate | lap 1 d_beta |
|---|---|---|---|---|
| a_P4_ccw_11 | 4 | +0.12 +0.09 -0.02 +0.21 | +0.12 +0.08 -0.03 +0.21 | +0.230 |
| a_P4_cw_11 | 4 | +0.08 +0.01 -0.08 +0.08 | +0.07 +0.01 -0.08 +0.09 | -0.231 |
| c_ND_ccw_11 | 13 | +0.19, then -0.06 to +0.10 | +0.20, then -0.06 to +0.10 | +0.167 |
| c_ND_cw_11 | 13 | +0.16, then -0.08 to +0.09 | +0.17, then -0.08 to +0.10 | -0.180 |

The built gate reproduces the table to 0.03 s (the brief allows 0.10 s). The attitude correction has
the sign and size the design predicts (0.23 rad on the P4 ring).

### A1 to A6

| | result | numbers |
|---|---|---|
| A1 | PASS (as recorded) | F1 red on main, lap 1 at 10.95 s of 11.94 s, 91.7 percent of a lap |
| A2 | PASS | racer_laps 472 passed, 2 skipped, coverage 90.6 percent (gate 90); l5_pnp_laps 411 passed (`lap_battery_b8` 98, `battery_b8` 94 percent); tools 90 passed in the files touched; registered_truth 14 passed. Nothing weakened; rewritten tests listed in the PR comment |
| A3 | PASS | 13 of 13 dumps identical to v2.1 (0.000 s), all `start_scan`, alias share 0.000 |
| A4 | PASS | 20 and 20 laps, 0.000 s from v2.1, 0 missed, all `start_scan`, alias share 0.000 |
| A5 | PASS | 16 runs judged, count rule holds in all 16, 0 of 192 judged laps outside the class tolerance (limit 2 percent), largest error 0.25 s |
| A6 | PASS | mean 5.41 ms, p99 16.51 ms over 22,803 scans (v2.1 measured the same way 3.14 and 14.97; the machine was not idle: other jobs ran) |

A3 and A4 show that a placed track is proven before v2.1 would close: its crossings did not move.
A6 is higher than round 1 (4.76 and 14.93) because lap 1 now computes a pair ICP on every scan.

### A5, owner decision 14 (B8's drop rule applies to the development cases)

Condition a, seed 11, either direction, fewer than 3 truth laps drops the case: **OD is dropped**
(`a_OD_cw_11` 2 laps, stalled; the other three OD runs stalled too: a ccw 12 laps, c ccw 0, c cw 2).
A5 judges the 16 P4 and ND runs:

| run | count rule | laps outside | largest error s | sources | verdicts | lap 1 d_beta | pair share lap 1 |
|---|---|---|---|---|---|---|---|
| a_ND_ccw_11 | ok | none | 0.19 | winding 12 | aliased | +0.17 | 0.72 |
| a_ND_cw_11 | ok | none | 0.17 | winding 13 | aliased | -0.18 | 0.72 |
| a_P4_ccw_11, 12, 13 | ok | none | 0.20, 0.25, 0.19 | winding 12 | aliased | +0.23 | 0.67 |
| a_P4_cw_11, 12, 13 | ok | none | 0.14, 0.13, 0.15 | winding 12 or 13 | aliased | -0.23 | 0.66 |
| c_ND_ccw_11 | ok | none | 0.15 | winding 12 | aliased | +0.17 | 0.72 |
| c_ND_cw_11 | ok | none | 0.11 | winding 13 | aliased | -0.18 | 0.72 |
| c_P4_ccw_11, 12, 13 | ok | none | 0.09, 0.09, 0.18 | winding 13 | aliased | +0.23 | 0.67 |
| c_P4_cw_11, 12, 13 | ok | none | 0.12, 0.10, 0.06 | winding 13 | aliased | -0.23 | 0.66 |

The pair share is the share of lap 1's wheel distance wound on the pair rate (the first 2 m are on
the steering rate); every later lap is wound entirely on it. The start scan is latched aliased on
every run before the first closure. Several first attempts were starved by other load on the host
(another builder's jobs and a stray process); they were rerun alone, every attempt kept.

### RD stall probe (report only, decides nothing by rule)

Case RD: ring, lane 1.2 m, island radius 0.50 m, one box 0.20 m into the lane at 0.35, seeds 11 to
13, conditions a and c, both directions, 13 truth laps.

- **No RD run stalled**: all 12 reached 13 truth laps, 0 contacts. The planner does not stall on a
  box this size.
- **The gate does not hold on RD in condition c.** Of the 12 runs, 10 are clean (winding, largest
  error 0.06 to 0.29 s). `c_RD_ccw_11` and `c_RD_ccw_12` are not: the start scan is called proven
  (the box, close to the start, makes its matches unique), laps close by `start_scan` and `anchor`
  with errors up to 1.67 and 1.32 s, 3 and 4 laps flagged missed gate, 13 crossings by truth crossing
  12 plus half a lap (the count rule broken). `a_RD_ccw_11` is mixed too (one `start_scan` lap, then
  latched aliased, 11 winding laps, all within tolerance).

R3 has the same geometry class (a 0.20 m box in a 1.2 m lane) and is a B8 case. If R3's box sits
where the start scan sees it, the same failure is a risk for D8-S0. I did not look into it further
(report only). It is for the launcher and the owner before any freeze.

## What the build found in round 2

- The pair centre is the fixed point of the match of a scan to the one 0.4 m back; on the P4 ring it
  is (0.00, 0.95) within 0.05 m from any seed along the valley, clockwise (0.00, -0.95). A straight
  gives none (the match yaw is under 0.1 rad) and the steering rate is used.
- The attitude correction `d_beta` is +0.23 rad ccw and -0.23 cw on P4, +0.17 and -0.18 on ND.
  The sign, as written in the brief, gives the prototype table.
- `kf_seed_yaw` is no longer a flag: the seed always takes the odometry's yaw. `kf_retire_visits`
  is gone; an aliased keyframe (3 ambiguous, over a quarter, over its life) is retired.
- The brief's constants table lists `kf_seed_yaw | true`; its text says no flag stays behind. I
  followed the text.
- A stadium whose start scan is latched aliased (a violation limit of 0.04 on the T2 alias input, which
  is not the registered 0.02) now closes by winding, and winding on a stadium with straights is not the
  line. The rewritten fixture test says so; the registered limit does not do it.

## Round 1 (brief 50 as written, the gate of `a90f38a`; A5 failed)

### A1: the synthetic ring on gate v2.1 (premise)

F1 (`tests/test_gate_v3_f.py`, island 0.40 m, lane 1.1 m, 0.5 m/s, exact odometry, 5.97 m round,
11.94 s per lap): on main (71ae56d) lap 1 closes at 10.95 s, 91.7 percent of a lap (the brief:
before 95 percent), and four laps are counted before the fourth true lap is complete. **Red on
main, so the premise holds and the work went on.** On gate v3: 12 laps for 12, each wheel length
within 2 percent of the circle, every `line_source` `winding`, `phase_observable` false; with the
heading gain off by plus and minus 3 percent every lap is within 4 percent and none is gained or
lost. The second F test (every scan within 0.6 m of the line of lap 3 has odometry 0.35 s stale)
was red on main as well: lap 3 lost, the next closure flagged missed; on v3 it closes by `anchor`
within 0.05 s of the clean run's crossing.

### A2: existing tests and goldens

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

### A3: the 13 dumps of `v0.5-lap3-dev`

Every lap time is identical to v2.1's (largest difference 0.000 s), every `line_source` is
`start_scan`, start alias share 0.000 in every lap, 5 laps each: PASS (13 of 13, N1 ccw and N2 cw).

### A4: the two car bags, full replay

| bag | laps v3 | laps v2.1 | largest lap difference | missed | sources | alias share |
|---|---|---|---|---|---|---|
| 2026-11-16T22-38-47 | 20 | 20 | 0.000 s | 0 | start_scan x20 | 0.000 |
| 2026-11-16T02-18-54 | 20 | 20 | 0.000 s | 0 | start_scan x20 | 0.000 |

PASS.

### A5: development sim runs (conditions a and c, both directions)

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

### A6: cost over the replays of A3 and A4 (development machine idle)

All 22 803 scans after the start scan of the 13 dumps and the two bags:

| gate | mean ms | p99 ms |
|---|---|---|
| v3 | 4.76 | 14.93 |
| v2.1 (measured the same way) | 2.95 | 13.84 |

Limits: mean at most 12 ms, p99 at most 25 ms: PASS. Armed scans of the car bags alone: mean 12.5 to
12.7 ms, p99 16.2 to 16.4 ms (v2.1: 11.2 to 11.3 ms, 16.2 to 16.3 ms). Memory: a keyframe is
about 8.3 kB in float32, 120 of them 1.0 MB (a test). The Orin Nano is not measured: item (v) of
D8-C0 is.

### `pnp_session lap-c0` on the two 2026-11-16 bags (`--replay`, REGRESSION, NOT A GATE)

Not the registered procedure (not observe mode, not one per direction): a regression record that
decides nothing. The reference validity rule, applied to the camera counter, says what the notes
said before: bag `T22-38-47` has 20 reference laps against a wheel count of 20 (valid), bag
`T02-18-54` has 10 reference laps (median lap distance 7.76 m: the camera missed every other lap)
against a wheel count of 13 (not a reference). D7-C0 and D8-C0 are therefore REPEAT for the pair,
D3-C0 FAIL (the count differs). For `T22-38-47` alone every lap of the replay is `start_scan`, none
missed, laps 2 to 20 within 0.14 s of the camera's, and the crossing instants sit a constant
0.28 to 0.42 s before the camera's (the offset lap 1 is judged against). Recorded events (without
`--replay`) are not in these bags: the lap node did not run, so the items read INSUFFICIENT.

### What the build found that the brief did not say

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

### Not done (round 1)

Nothing after A6: no freeze file, no B8 case, no seed 61 to 63. PR 207 (the B7 result) was still
open when this branch was cut from `origin/main` (71ae56d); the two PRs touch no common file.

## Round 4 (brief 50d: a dead band for the alias verdict)

Built as written: `alias_gap` 0.025 (`0 < alias_gap <= unique_gap`, checked), the match of a same-pose
displaced scan is UNIQUE at a gap of 0.0325 and over, AMBIGUOUS under 0.025, UNKNOWN between (counted
for neither verdict, places no candidate); `ring.is_proven` (4 unique over life, ambiguous at most a
quarter of unique plus ambiguous, no minimum count) used by `start_proven` and by the keyframe fix.
Status: `gaps_unique`, `gaps_unknown`, `gaps_ambiguous`, `latched_at_scan`, life counts. Development
data only; no R or Q case, seeds 61 to 63 or B7/B8 run directory was opened. `unique_gap`,
`proof_min_unique` and every other threshold are where round 3 left them, `alias_gap` was not tuned.

### Development checks

| check | result |
|---|---|
| A1 / A2 (full local suite) | one registered L1 test red, see findings 2 below; everything else green |
| A3, A4 | PASS (A4: 20 laps each, 0.000 s from v2.1, none missed) |
| A5 | PASS (16 runs, 0 of 192 judged laps outside the tolerance) |
| A6 | PASS (mean 5.2 ms, p99 16.0 ms; v2.1 3.1 and 15.6 ms) |
| C1 | PASS (12 fresh RD runs, 12 of 12 clean, every lap by winding, 0 of 144 laps outside) |
| C2 | PASS (the 12 recorded RD dumps) |
| M1 | PASS on the 13 stadium dumps and the two car bags (0 ambiguous; smallest same-pose gap 0.040, so 0.015 over `alias_gap`; the bags have 178 and more unique matches whose displaced scan did not match at the same pose, so no gap at all); FAIL on the synthetic 5 m stadium, see finding 1 |
| M2 | PASS on the 28 RD, ND and P4 dumps: all latched aliased, every lap by winding, ambiguous share 0.95 to 1.00; latched the same scan as round 3 in 27, one scan later in c_ND_cw_11 |
| M3 | PASS: all 43 dumps and bags replayed at -0.003 and +0.003, same verdict, same line sources, lap times identical to 0.000 s |
| L3 at 3.0 m/s | 24 of 26 on the first run, then the failing test passed twice; flaky, see finding 3 |

Gap classes per data set (count and range), the three classes:

- 13 stadium dumps: unique 78 to 93 each (0.040 to 0.207), unknown 0, ambiguous 0.
- 2 car bags: no same-pose displaced scan at all (life unique 178 and more, ambiguous 0).
- ND (4 dumps): unique 2 to 9 (0.033 to 0.062), unknown 1 to 13 (0.026 to 0.031), ambiguous 163 to 240 (-0.017 to 0.024).
- P4 (12 dumps): unique 0, unknown 0, ambiguous 161 to 246 (-0.002 to 0.002).
- RD (12 dumps): unique 0, unknown 0 to 72 (0.026 to 0.031), ambiguous 67 to 249 (-0.019 to 0.024).

### Findings the brief did not say, with the design as written

1. **The 5 m stadium has ambiguous matches when the speed rises.** The v2 test matrix (straight 5 m,
   speed rising 0.1 m/s a lap to 1.2 m/s) gives 1 to 4 ambiguous gaps (0.019 to 0.0238) in three of its six (all counter-clockwise)
   rising runs (the brief expected a smallest gap of 0.029); worst case counter-clockwise, scale 1.05:
   4 of 46 matches, nothing latches (share 0.09), every lap by `start_scan`. Constant 0.5 m/s: none.
   `alias_gap` may not go under 0.020 and 0.019 would still be ambiguous there. The F test
   (`test_the_5_m_stadium_has_no_ambiguous_match...`) is kept as written and marked xfail on the rising runs.
2. **The box ring clockwise is not latched.** `test_a_ring_with_a_box_the_start_scan_may_see...[True]`
   (clockwise, the box out of sight) is red: 26 unique matches (0.033 to 0.038), 4 unknown (0.026 to 0.031),
   3 ambiguous (0.017 to 0.019): proven, every lap `start_scan`, lap times within 0.1 s of truth. Round 3
   latched it only because the four band matches counted as ambiguous at that moment (7 of 33). No value of
   `alias_gap` in 0.020 to 0.028 changes it (at most one more ambiguous match). Left red.
3. **A stadium at 3.0 m/s can latch in L3.** One of three runs of the L3 lap test: 5 unique (0.040 to 0.112)
   and 3 ambiguous (0.007, 0.019, 0.021) in the first approaches, share 0.375, latched at scan 65, laps 2 to
   4 by winding, lap 2 to 4 times 0.4 to 1.0 s short. All three gaps are under 0.025, so the band does not
   touch them: round 3 has the same exposure (the run to run difference is scan timing in the node). The latch
   is for good, which is what the brief's own reasoning says is dangerous.
4. The two 50c table rows "gap 0.032" and "gap 0.031, the largest alias of RD" now expect UNKNOWN (they are
   the rows the brief replaces); no other row, test, tolerance or golden changed.

### Not done

No freeze, no B8. Not merged: finding 2 leaves a registered L1 test red and finding 3 is an L3 flake; M1
fails on the 5 m stadium (finding 1). `alias_gap` was not tuned because no value in 0.020 to 0.028 changes
any of the three.

## Round 5 (brief 50e: the verdict is a majority, held, not for good)

Built as written: aliased entered at 6 or more ambiguous and a share over 0.75 of counted matches,
released at a share of 0.5 or under (`alias_release_share`), proven at 4 unique and more unique than
ambiguous, no start-scan close and no hold expiry while ambiguous matches are at least the unique ones.
`alias_min_scans` was not tuned. Both gap constants and the 50d band are unchanged.

| check | result |
|---|---|
| A3, A4, A6, C2 | pass (A6 mean 5.1 ms, p99 15.4 ms) |
| A5, C1 | pass (16 runs; 12 of 12 fresh RD runs, every lap by winding) |
| M1' | pass: 13 stadium dumps and 2 bags, never aliased, share 0.00 at every scan with 3 ambiguous |
| M2/M4 | pass: 28 ring dumps, aliased 3 to 11 scans later than round 4, never released, share over the run 0.95 to 1.00, lap times identical to round 4 |
| M3 | pass |
| M5 | NOT met: 24 of 30 and 21 of 30 green, the third loop stopped at 12 |
| Linux (aarch64, ros-dev) | the registered long-straight case (lap 7 = 11.446 on main) passes; full racer_laps suite one failure, a superseded 50b test, fixed |

Margin, with no rule: the largest ambiguous share a placed run shows once 3 are ambiguous is 0.00 on the
dumps and bags, 0.09 on the 5 m stadium and 0.38 on the L3 stadium; the smallest a ring shows from its entry
is 0.75 (ND), 1.00 on RD and P4.

M5: every red run is a missing 4th lap (`laps=[1, 2, 3]` or 3 laps in the station set) of the L3 lap test.
The third loop printed the counts on every run, failures included: `aliased_entries` 0 in all 12, u/a 10/5, 11/5
and 12/4 on the three red ones. The same test on main fails 7 of 30 in the same way, so it is not the verdict;
the cause (lap 4 closing late under host load in the container) is not found. Not rerun until green.

Why the registered Linux case failed on main: a few early matches decided the latch ("3 and over a quarter")
and the count of counted matches differs by 1 to 2 between macOS and Linux (38 against 40). The 50e rule
needs 6 and over 3/4, and the same case shows at most 4 ambiguous of 46 on both platforms.

## Round 6 (brief 50f: displacement is a floor, proof by span, the late laps)

Built as written: the displaced scan is the nearest to `unique_shift_m` among those at least that far back
(`unique_shift_tol_m` removed; under 0.4 m of travel there is none); proven is not aliased, more unique than
ambiguous, and 4 unique or 3 unique, none ambiguous, whose first and latest unique matches are at least 0.4 m
of wheel distance apart (start scan and keyframes). The span is measured over the whole life of the reference,
so after a lap it is long; a reference with ambiguous matches never takes the span arm. All numbers below are
measured again at the final head; none is carried over from round 5.

The late laps 2 and 3 and the missing lap 3 of round 5 were one cause, in the gate. At a scan spacing just over
0.30 m (3.0 m/s with wall-clock jitter) the displaced scan was 0.30 to 0.35 m back and a placed track showed
30 to 40 percent ambiguous matches: the hold ran to `candidate_ttl_s` (1.7 s late in L1 at 3.03 to 3.3 m/s,
seeds 1 and 7), or, with a clear majority ambiguous, a lap never closed. Not starvation (scans late by at most
0.03 s, start delay constant at 0.055 s, docker otherwise idle) and not the node. At 0.4 m and over the placed
stadium has 0 ambiguous matches at every spacing tested. What remained (3 unique matches on lap 1's approach,
proof needing 4, hold to the ttl) was lap 1 only, 1.4 s late, in 2 of 30 runs; proof by span removes it.

| check | result at the final head |
|---|---|
| M5 (L3 lap test, 3.0 m/s, ros-dev, aarch64) | 60 of 60 green in two loops of 30: four lap events every run, the latest 0.26 s after its crossing, u/a counted every run with a = 0 and `aliased_entries` 0 |
| A3, A4, A6, C2 | pass (A6 mean 5.1 ms, p99 15.4 ms) |
| A5, C1 | pass (16 runs, 0 of 192 laps outside; 12 of 12 fresh RD runs). c_OD_cw_11 (stalled, dropped by the rule) shows one release, aliased then proven, reported |
| M1' | pass: 13 stadium dumps, 2 bags: never aliased, share 0.00 at every scan with 3 ambiguous |
| M2/M4 | pass: 28 ring dumps aliased 3 to 11 scans later than round 4, never released, never proven at any scan, share over the run 0.95 to 1.00, lap times within 0.03 s of round 4 |
| M3 | pass |
| L1 | the 3.12 and 3.22 m/s seed 7 reproduction closes lap 1 on time; (u, a, span) table; no ring (plain, box ccw, four fixtures) proven at any scan |

Gap classes against round 4: the smallest placed unique gap is 0.043 (round 4: 0.040), the largest ring
ambiguous gap 0.024 (the same); neither moved by 0.005.

Full suites at the final head: the whole local suite on macOS exit 0 (after merging main); the whole racer_laps suite including the L3 node tests inside ros-dev on Linux aarch64: 609 passed, 3 skipped. The registered long-straight case that failed on main on Linux (lap 7 = 11.446) passes there.
