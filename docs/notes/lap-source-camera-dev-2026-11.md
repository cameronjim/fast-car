# Camera lap source: development acceptance (brief 51, PR A), 2026-11-17

Pre-registration: `docs/notes/preregistration-pnp-v0.5-laps-camera.md`, constants
`evaluation/prereg/pnp-v0.5-laps-camera.yaml` (SHA-256
`013cb2f8bec3ce96d2abf456361c3de93d8f85da48c01900c374287d7473807f`), committed alone as commit 1.
Everything below is development. No U case, no seed 71 or 72 was run, neither evaluation bag
(`2026-11-16T22-45-39`, `2026-11-16T23-04-09`) was opened or decoded, nothing under `v0.5-b7/` or
`v0.5-b8/` was touched. Development runs are in `car-data/pnp/battery/v0.5-camera-dev/`. Code under
test: the head of `feat/pnp-lap-source-camera`, stacked on `feat/pnp-lap-keyframe-ring` (PR 210, head
`6e99c4d`, merged again before the end: no new commits there). The source rule's constants are the
yaml's and were not tuned.

Result: E1 to E7 hold (E1 with one failing test of D v1 that fails identically on PR 210's head, see
below). The design is built as written; two things it implies are in "For the owner".

## E1: existing tests and goldens, unchanged

| suite | result |
|---|---|
| `racer_laps` (replay extra) | 569 passed, 1 failed, 3 skipped; line coverage 91.65 percent (gate 90) |
| `tests/l5_pnp_laps` | 457 passed, 1 skipped (the ROS canary skips off a workspace) |
| `tests/l5_reactive_bench` | 104 passed, 1 skipped |
| `tools/tests` | 750 passed, 53 skipped (the ones that need built binaries) |
| `racer_bringup/test/test_pnp_launch.py` (ros-dev container) | 75 passed |
| `racer_bringup/test` all (ros-dev container) | 246 passed, 2 failed: `test_car_teleop_launch` and `test_car_teleop_bag_launch`, timing assertions of the car_teleop launch (a bag size and a stamp offset); this PR touches neither file |
| `racer_laps` node tests (ros-dev container) | `test_node_ros`, `test_node_scan_ros` and the new `test_node_external_ros` pass |

The one failing `racer_laps` test is `test_guidance_property.py::test_every_guidance_is_valid_and_bounded_whatever_happens`
(D v1, start-scan source): a zone bound at a wrapped station (zone centre 0.51 m, length 1.0 m,
bound 0.75 to 1.25 against a limit of 0.83). Hypothesis found it on this branch; the same example run
on PR 210's head fails the same way, so it is not from this branch and D v1 is untouched here.

Identity with `lap_source:=start_scan`: the two car slices of `tests/fixtures_car_2026_11_16`
(GP3-R1 laps 14 to 17 and the parked last lap) replayed through the engine on PR 210's head and on this
branch give identical `LapResult`s and an identical node status after every lap (every key except the
`on_scan_ms_*` timings): byte for byte the same JSON.

The expectations changed on purpose are listed in the first PR comment.

## E2: bag `22-38-47`, `lap-src --offline` (regression, not a gate)

`source_ready` true; 20 events give 20 laps, 20 confirmed, 0 doubtful, 0 rejected. Every crossing stamp
equals the event's `crossing_time_s`; the wheel length is within 0.036 m of the event's `distance_m`
(laps 2 to 20 largest: lap 19, 0.036 m; lap 18 0.018, lap 17 0.015, the rest at most 0.014); the lap
time equals the event's. D9-B0 items on this bag: (i) PASS, (ii) PASS, (iii) PASS (lapref length 5.25
m, 19 of 19 inside the band confirmed). Lap 1 (reported, not judged): 6.34 s, 5.50 m, confirmed.

## E3: bag `02-18-54`, `lap-src --offline`

Real ready rule: `source_ready` false (the camera's first status arrived 2.4 s after the planner's
first DRIVING report: "no source status yet"), all 10 laps doubtful with reason `ready`. With
`--assume-ready`:

| event | camera s | camera m | replay s | replay m | confirmed | reasons | cap after |
|---|---|---|---|---|---|---|---|
| 1 (lap 1, reported) | 6.20 | 5.26 | 9.23 | 7.20 | no | heading | 0.50 |
| 2 | 5.78 | 5.23 | 5.78 | 5.23 | yes | | 0.50 |
| 3 | 5.57 | 5.12 | 5.57 | 5.12 | yes | | 0.60 |
| 4 | 11.39 | 10.40 | 11.39 | 10.40 | no | length, heading | 0.50 |
| 5 | 5.56 | 5.09 | 5.56 | 5.09 | yes | | 0.50 |
| 6 | 11.20 | 10.25 | 11.20 | 10.25 | no | length, heading | 0.50 |
| 7 | 11.16 | 10.25 | 11.16 | 10.25 | no | length, heading | 0.50 |
| 8 | 5.66 | 5.19 | 5.66 | 5.21 | yes | | 0.50 |
| 9 | 11.45 | 10.49 | 11.45 | 10.48 | no | length, heading | 0.50 |
| 10 | 22.51 | 20.62 | 22.51 | 20.63 | no | length, heading | 0.50 |

Laps 4, 6, 7, 9 and 10 doubtful by length and heading, laps 2, 3, 5, 8 confirmed, lap 1 reported. The
what-if cap never rises at a doubtful lap (it falls from 0.6 to 0.5 at lap 4 and stays there). Lap 1 of
this run is doubtful by heading alone: the car started 3 s before the camera's reference frame existed
and the lap node's lap 1 is the whole start, 7.2 m for 1.15 turns.

## E4: development sim, seed 11, ccw and cw

P1, P2, P4 in conditions b and c (12 runs), P1 and P4 in condition f (4 runs), 13 truth laps each, 16 run
files, `car-data/pnp/battery/v0.5-camera-dev/`. Two attempts were starved (a real-time factor under
0.8 while two test suites ran on the same host: `b_P1_ccw_11`, `c_P2_ccw_11`) and were rerun alone, as
the machinery says; every attempt is kept in `attempts/`. D9-S0, D9-S1 (6 of 6) and D9-S2 as written:

| gate | result | numbers |
|---|---|---|
| D9-S0 (6 c runs) | PASS | 12 oracle events per run, one per truth crossing 1 to 12; 12 confirmed laps, none doubtful, 0 events rejected; every system crossing within 0.3 s of the truth crossing of its number |
| D9-S1 (b and c, 6 of 6) | PASS | 0 contacts; 6 of 6 complete 10 laps; T_late(c) / T_late(b) 0.996 to 1.003 (limit 1.05) in 6 of 6; T_late below T1 in all 6; median r 0.486 (limit 0.80) |
| D9-S2 (4 f runs) | PASS | 0 contacts; 12 truth laps; every confirmed lap is one truth lap; laps 4 and 6 (merged, closing at truth crossings 5 and 8), 7 (the false event) and 8 (the lap after it) doubtful, each by length and heading; the duplicate of crossing 10 rejected (`not_monotone`, once per run); the cap after a doubtful lap never above it; the lowest guidance cap in both overdue windows is the lap-1 cap |

Reported with no threshold: T_late(f) / T_late(c) is 2.22 and 2.22 (P1 ccw, cw), 2.00 and 1.93 (P4
ccw, cw): after four doubtful laps in nine the schedule stays near the lap-1 cap, as built. Event
latency of the oracle at most 0.03 s in c, at most 0.23 s in f (the duplicate). Reference length per run
13.4 to 13.7 m (P1), 12.8 (P2), 5.6 to 5.7 (P4).

## E5: the oracle against the registered truth (condition c runs of E4)

| run | oracle events for crossings 1 to 12 | worst difference to the truth crossing |
|---|---|---|
| P1 ccw | 12 | 0.053 s |
| P1 cw | 12 | 0.044 s |
| P2 ccw | 12 | 0.048 s |
| P2 cw | 12 | 0.048 s |
| P4 ccw | 12 | 0.051 s |
| P4 cw | 12 | 0.047 s |

Same count in every run; every crossing within 0.10 s (the gate asks 0.3 s). The oracle's crossing is
the start line plane of `LapDetector` on the ground-truth pose, the registered truth's is the
centreline progress of `bench_metrics.LapTracker`: the 0.05 s offset is the difference between the
two ways of cutting the same loop.

## E6: launch

`racer_bringup/test/test_pnp_launch.py`, 75 passed with the additions:

- `lap_memory:=false` (the default): the node list and every parameter pinned against the golden of
  the deskew test, whatever `lap_source` says (start_scan, map_pose, camera); four actions, unchanged.
- The three refusals refuse: `lap_source:=truth` (with and without lap memory, "the car has no truth"),
  `lap_source:=camera lap_counter:=false` with no counter in the graph (the wording of
  `require_lap_counter`, after the five second wait), `lap_shadow` equal to `lap_source`. Also refused:
  `lap_shadow` without `lap_memory:=true` and `lap_mode:=v1` with the camera.
- `lap_source:=camera` with the counter started by the launch waits for it like `require_lap_counter`.
- The shadow node is `lap_memory_shadow_node`, `lap_mode` observe, `output_prefix` `/pnp/lap_shadow`,
  `output_dir` `.../shadow`; the planner's and the primary node's parameters are those of a run
  without it; its exit is logged and does not stop the launch.
- On a live graph (`test_node_external_ros`): the shadow node has no publisher on `/pnp/lap/guidance`
  or `/pnp/lap_shadow/guidance`.

## E7: the reference_suspect diagnostic changes nothing (brief 51b)

`reference_suspect` (status keys `reference_suspect`, `reference_suspect_lap`, one WARN line, the
`lap-src --drive` header, two runbook sentences) is set when the reference rests on exactly one
confirmed lap and a later lap fails only `length`. It reads the verdict and feeds nothing.

| check | result |
|---|---|
| existing tests and goldens | `racer_laps` 586 passed, 3 skipped (the D v1 property test of E1 deselected, unchanged); the new gate, property, node and `lap-src` tests pass; ruff clean |
| the L2 property | for any event sequence, verdicts, reasons, caps and the reference length are identical with the flag logic present and with it stubbed out |
| E1 to E6 numbers | unchanged: E2, E3 and the L4 replays are the same code path and the same goldens |
| seed 11 development runs, 16 runs again | D9-S0 PASS, D9-S1 PASS (6 of 6, ratios 0.990 to 1.019, median r 0.483), D9-S2 PASS; doubtful laps and reasons identical in all 4 f runs (laps 4, 6, 7, 8, length and heading), 0 rejected beyond the one duplicate, 0 contacts, the flag false in every c and f run |

The sim is not bit-reproducible run to run on this host: lap times differ by up to 0.36 s between two
runs of the same case, also in condition b where no lap logic is involved, and `safety_brakes` moved
in 2 of 16 runs (b_P4_cw 0 to 4, c_P1_ccw 1 to 4: timing events, not obstacle brakes). Compared run for
run: lap count, caps per lap, clean flags, doubtful laps with reasons, rejected events, contacts
identical in 14 of 16; the two that differ are those timing and snapshot effects (b_P4_cw has no lap
logic in play; c_P1_ccw: 12 system events in the status snapshot against 13 before, the last lap
closing after the snapshot).

### Heading per lap with the real gain table (E2 bag `22-38-47`, the car's vehicle_params table)

Single laps read 0.927 to 0.971 turns (mean 0.938; 20 laps, lengths 5.14 to 5.50 m), not the 1.085 to
1.105 of gain 1.0: the car's table under-reads. On `02-18-54` the single laps read 0.922 to 0.951, the
merged ones 1.857 to 1.870 (two laps) and 3.737 (four); lap 1 (launch, 7.20 m) 1.293.

The registered window is 2 pi plus or minus 1.2 rad, 0.809 to 1.191 turns. With 0.938 turns per true
lap that passes a lap 1 of 0.86 to 1.27 lap lengths (worst case of the 20 laps, 0.927 turns: 0.87 to
1.28; best case 0.971: 0.83 to 1.23). So on the car a split first lap below about 0.86 of a lap is
caught by the heading, one of 0.86 to 1.0 passes, and the upper edge is a lap of about 1.27 lap
lengths, not 1.1. Nothing changed: the numbers say where the registered edge sits.

## Other evidence

- L5 canary (ros-dev container): P1, seed 11, `lap_source truth`, 4 truth laps: system crossings 33.22,
  57.68, 78.78 s against truth 33.17, 57.64, 78.74 s, 3 judged laps confirmed, caps 0.5, 0.6, 0.7, 0
  contacts, `/drive_raw` from `mppi_planner_node` only and `/drive` from `safety_node` only. With one
  dropped crossing: the merged lap (29.27 s, 52.40 s) is doubtful (`length`, `heading`) and the cap handed
  out after it is not above the cap it was driven at.
- `lap-src` on the dev bags: `22-38-47` and `02-18-54` carry no lap node, so D9-C0 refuses them with
  "no lap node on the camera source" and `--drive` with "no lap node events": the refusal the tests pin.

## For the owner

1. A first lap, with no reference length yet, is judged by the minimum length (3.0 m) and the heading
   window (1.2 rad of 2 pi) alone, as written. A split first lap of 0.81 to 1.0 lap lengths passes both
   and becomes the reference length; every later full lap then misses the band and is doubtful (the cap
   stays at the lap-1 cap: safe, but not a lap count). The L2 property "a confirmed lap is within the
   band of the true lap length" holds once a reference exists, and for the first lap the bound asserted
   is the heading window. Not tuned; a stricter first-lap rule is a new registered attempt.
2. D9-C0 item (i) with no owner count uses the wheel rule; a bag with no odometry gives REPEAT (the
   count cannot be checked), which the brief lists as a verdict but does not define: take the count at
   the stop.
3. The one failing D v1 property test above is on PR 210's head too.
