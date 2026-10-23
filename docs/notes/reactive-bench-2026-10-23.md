# Reactive benchmark battery for gap_follow_node, 2026-10-23

A simulator battery that reproduces the corner and open-side failures of the owner's
living-room floor (bags 2026-10-06), so gap_follow_node can be tuned without the car. Sim only:
nothing here ran on the Jetson or the car.

The tables below are the output of `tools/reactive_bench.py` (generated), copied in with the
prose around them written by hand. Every run is 100 Hz sim time at a real-time factor of 1.00
(checked per run, `real_time_factor` in the JSON), four to seven runs side by side in one
container.

## What was built

| Piece | Where |
|---|---|
| Five parametric synthetic tracks as wall segments, rasterised for the gym's LiDAR caster; bridge parameter `synthetic_track` selects one | `sim/bridge/racer_gym_bridge/racer_gym_bridge/track_loader.py` (`build_synthetic_track`, `rasterize_walls`), `bridge_node.py` |
| Runner: one track, one parameter set, JSON result | `tests/l5_reactive_bench/reactive_bench_runner.py` |
| Metric maths (pure, unit-tested) | `tests/l5_reactive_bench/bench_metrics.py` |
| Battery CLI: all tracks, markdown table, note, sweep, ranking | `tools/reactive_bench.py` |
| The same in the ros-dev container on an isolated network | `tools/reactive_bench_docker.sh` |
| Reduced battery in CI | job `l5-reactive-bench`, `.github/scripts/l5_reactive_bench_test.sh` |

The runner starts `bridge_node`, `safety_node` and `gap_follow_node` as processes on the real
command path (`/drive_raw` to `safety_node` to `/drive`, no test-only remap), watches the
topics for a fixed time and prints JSON. A wall contact (f1tenth_gym's own collision handler,
or the body rectangle overlapping a wall face) is counted and the car is put back at the start
with `/sim/reset`, so one contact does not end a run; the lap in progress is lost.

## The tracks

All lane tracks are one counter-clockwise rectangular loop (5.5 x 4.0 m centreline, 19 m),
lane width W = 1.1 m, the car starting mid bottom straight facing +x. The walls are the
centreline offset by W/2: outside corner radius r + W/2, inside max(r - W/2, 0).

| Track | What it is |
|---|---|
| `lane_square` | Four square corners (centreline radius 0): square inside corners, an arc of radius 0.55 m outside. The geometry of the L5 escape canary, where the car ends nose-in as it did on the floor. |
| `lane_round` | The same with centreline corner radius 0.25 m (outside radius 0.80 m, inside square). |
| `lane_open_side` | Easy 0.8 m corners; on the bottom straight the LEFT wall is replaced by 3 boxes 0.4 m long (0.3 m deep) with 0.6 m gaps, and the open 4.4 x 2.9 m infield behind them. |
| `chicane` | Easy corners; two jogs of 0.5 m (left, then back) on the bottom straight, each over 1.0 m of track, 0.8 m plateau. |
| `open_room` | Not a loop. A 4 x 4 m room, three 0.5 m boxes, the car starting bottom left facing +x. |

Parameters (bridge `synthetic_lane_width_m`, `synthetic_corner_radius_m`, `synthetic_box_length_m`,
`synthetic_box_gap_m`, `synthetic_jog_offset_m`, `synthetic_room_size_m`; the CLI takes them as
`--track-params-json '{"box_gap_m": 1.0}'`). Existing options (`raceline_path`,
`track_half_width_m`, `pocket_*`, `reverse_direction`, the LiDAR mount) are unchanged;
`reverse_direction` also runs the synthetic loops clockwise.

Square outside walls were tried first and rejected: they leave a 1.56 m diagonal pocket in each
corner that the car drives round in one sweep, which is not the floor failure.

## Metrics

Per run: `laps`, `lap_time_s` (mean of the completed laps), `obstacle_brakes` (safety_node
`/safety/events` ENGAGE records with severity BRAKE from the `ttc` and `ttc_reverse` gates),
`escapes` (entries into a reverse request on `/drive_raw`), `min_body_clearance_m` (the
vehicle_params bounding box about the rear axle against the wall faces, exact, at 50 Hz; 0 or
less is contact), `wall_contacts`, `mean_abs_steering_rad` and `steering_reversals_per_min` (of
the follower's `/drive_raw` request; a reversal is a peak or trough confirmed by a 0.1 rad
swing). Also distance, mean speed, the real-time factor and the contact points.

## Result 1: the floor profile and the node defaults

Floor profile, `profile:=floor-2026-10-06` (max 0.9 m/s):

| Track | Laps | Lap time (s) | Brakes | Escapes | Min clearance (m) | Contacts | Mean abs steer (rad) | Reversals/min | Run (s) |
|---|---|---|---|---|---|---|---|---|---|
| lane_square | 3 | 39.8 | 15 | 15 | 0.038 | 0 | 0.180 | 42.0 | 150 |
| lane_round | 2 | 40.8 | 13 | 12 | 0.013 | 0 | 0.210 | 24.0 | 120 |
| lane_open_side | 3 | 24.9 | 0 | 0 | 0.167 | 0 | 0.195 | 62.0 | 90 |
| chicane | 3 | 24.7 | 0 | 0 | 0.219 | 0 | 0.187 | 43.3 | 90 |
| open_room | - | - | 39 | 8 | -0.001 | 1 | 0.286 | 41.0 | 60 |

Node defaults, `profile:=none` (what `gap_follow.launch.py` starts: max speed 2.0 m/s):

| Track | Laps | Lap time (s) | Brakes | Escapes | Min clearance (m) | Contacts | Mean abs steer (rad) | Reversals/min | Run (s) |
|---|---|---|---|---|---|---|---|---|---|
| lane_square | 12 | 12.0 | 0 | 0 | 0.204 | 0 | 0.178 | 39.6 | 150 |
| lane_round | 10 | 12.0 | 0 | 0 | 0.193 | 0 | 0.176 | 39.5 | 120 |
| lane_open_side | 7 | 11.4 | 0 | 0 | 0.180 | 0 | 0.166 | 42.0 | 90 |
| chicane | 7 | 11.8 | 0 | 0 | 0.166 | 0 | 0.183 | 40.0 | 90 |
| open_room | - | - | 1 | 0 | 0.100 | 0 | 0.008 | 3.0 | 60 |

Node defaults with the speed capped at the floor's 0.9 m/s (`--params-json '{"max_speed_mps": 0.9}'`):

| Track | Laps | Lap time (s) | Brakes | Escapes | Min clearance (m) | Contacts | Mean abs steer (rad) | Reversals/min | Run (s) |
|---|---|---|---|---|---|---|---|---|---|
| lane_square | 6 | 24.2 | 0 | 0 | 0.166 | 0 | 0.161 | 19.6 | 150 |
| lane_round | 4 | 24.2 | 0 | 0 | 0.159 | 0 | 0.160 | 19.5 | 120 |
| lane_open_side | 3 | 23.3 | 0 | 0 | 0.155 | 0 | 0.157 | 20.7 | 90 |
| chicane | 3 | 24.0 | 0 | 0 | 0.148 | 0 | 0.170 | 20.0 | 90 |
| open_room | - | - | 11 | 0 | -0.025 | 11 | 0.183 | 43.0 | 60 |

What the three say:

- The floor profile reproduces the known corner behaviour: one obstacle brake and one reverse
  escape per tight corner (15 in 3 laps of `lane_square`, 12 in 2 laps of `lane_round`), laps
  of about 40 s against 24 s without corners to escape from, and a minimum clearance of 1 to 4
  cm in the corners. No contact on any lane track.
- The node defaults go round all four lanes without a brake, at 0.9 m/s and at 2 m/s, with 15
  to 20 cm clear. They have no reverse escape and no lane centring, and in the open room they
  hit walls (11 contacts at 0.9 m/s, none of them recovered by an escape). **This does not say
  the defaults are better on the floor**: they were tried there first and did not work, and
  the sim's `lane_open_side` with the specified 0.6 m gaps does not reproduce that failure (next
  section). Treat the lane comparison as "the sim corner is easier for a follower that takes it
  in one sweep than for the floor profile's clamped, slower approach", not as a floor result.
- `open_room` is a wander test, not a lap: every set touches a wall in it now and then (1 to
  3 contacts per 60 s for the floor-profile family), so its contact count is noisy. It is kept
  because it is the only track where a follower can wedge itself against a box.

## Result 2: the open side

The specified `lane_open_side` (0.4 m boxes, 0.6 m gaps) is closed by the disparity bubble
(half width 0.155 m plus margin on each side leaves a 0.09 m gap), so neither the floor
profile nor the defaults dive into it. Widening the gaps reproduces the floor failure; 60 s runs
(`--track-params-json '{"box_gap_m": g}'`, defaults at 0.9 m/s):

| Gap | Floor profile | Node defaults at 0.9 m/s |
|---|---|---|
| 0.6 m (specified) | 3 laps in 90 s, 0 brakes | 3 laps in 90 s, 0 brakes |
| 1.0 m | 2 laps, 0 brakes, 0.124 m clear | 0 laps, 19 brakes, 0.012 m clear |
| 1.4 m | 2 laps, 0 brakes, 0.124 m clear | 0 laps, 20 brakes, 1 wall contact |
| 2.0 m | box layout does not fit the 3.9 m straight (refused) | same |

With gaps of 1.0 m and more the defaults chase the gaps into the infield and get stuck
braking, as on the floor, and the floor profile (lane centring, 1.2 rad cone) drives past
them. The CI battery and the sweeps below use the specified 0.6 m gaps except where stated;
`box_gap_m` 1.0 is the setting that tests the open side.

## Result 3: sweep

Grid on top of the floor profile: `steering_gain` {1.0, 1.6, 2.4}, `cone_half_angle_rad` {0.8,
1.2}, `centering_gain` {0.0, 0.4, 0.6}, `safety_margin_m` {0.05, 0.1}: 36 sets, the full
battery each, 180 runs, stopped at 3 wall contacts. Ranked as asked: wall contacts (all
tracks) first, then a lap on every loop track, then obstacle brakes per lap (loop tracks, the
open room has no laps), then lap time (the loops' mean lap times summed). First pass, one run
per set (best 12 and worst 6 of 36; the floor profile is `1.6 / 1.2 / 0.6 / 0.1`, row 4):

| # | steering_gain | cone_half_angle_rad | centering_gain | safety_margin_m | Contacts | Loop contacts | Loop runs without a lap | Laps | Loop brakes | Brakes/lap | Lap time, loops summed (s) | Escapes | Min clearance (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 1.6 | 0.8 | 0.6 | 0.05 | 0 | 0 | 0 | 11 | 137 | 12.45 | 161.7 | 42 | 0.002 |
| 2 | 2.4 | 0.8 | 0.4 | 0.05 | 0 | 0 | 0 | 10 | 130 | 13.00 | 163.5 | 41 | 0.002 |
| 3 | 1.0 | 1.2 | 0.4 | 0.1 | 1 | 0 | 0 | 12 | 27 | 2.25 | 131.9 | 34 | -0.014 |
| 4 | 1.6 | 1.2 | 0.6 | 0.1 | 1 | 0 | 0 | 11 | 28 | 2.55 | 130.3 | 33 | -0.002 |
| 5 | 1.0 | 1.2 | 0.6 | 0.05 | 2 | 0 | 0 | 12 | 26 | 2.17 | 132.9 | 35 | -0.012 |
| 6 | 1.6 | 1.2 | 0.4 | 0.05 | 2 | 0 | 0 | 12 | 27 | 2.25 | 131.3 | 34 | -0.007 |
| 7 | 1.0 | 1.2 | 0.4 | 0.05 | 2 | 0 | 0 | 12 | 27 | 2.25 | 131.8 | 34 | -0.010 |
| 8 | 1.0 | 1.2 | 0.6 | 0.1 | 2 | 0 | 0 | 12 | 27 | 2.25 | 132.3 | 34 | -0.005 |
| 9 | 1.6 | 1.2 | 0.6 | 0.05 | 2 | 0 | 0 | 11 | 27 | 2.45 | 130.4 | 35 | -0.011 |
| 10 | 1.6 | 1.2 | 0.4 | 0.1 | 2 | 0 | 0 | 11 | 27 | 2.45 | 130.5 | 35 | -0.002 |
| 11 | 2.4 | 1.2 | 0.0 | 0.05 | 2 | 0 | 0 | 11 | 30 | 2.73 | 131.0 | 38 | -0.002 |
| 12 | 2.4 | 1.2 | 0.4 | 0.1 | 2 | 0 | 0 | 11 | 61 | 5.55 | 136.7 | 33 | -0.012 |
| ... | | | | | | | | | | | | | |
| 31 | 1.0 | 0.8 | 0.0 | 0.05 | 9 | 6 | 1 | 4 | 424 | 106.00 | 152.1 | 46 | -0.014 |
| 32 | 1.6 | 0.8 | 0.0 | 0.05 | 9 | 6 | 2 | 5 | 153 | 30.60 | 59.1 | 27 | -0.021 |
| 33 | 1.0 | 0.8 | 0.6 | 0.05 | 9 | 6 | 2 | 4 | 139 | 34.75 | 91.2 | 34 | -0.012 |
| 34 | 1.0 | 0.8 | 0.4 | 0.1 | 9 | 6 | 2 | 3 | 185 | 61.67 | 127.0 | 43 | -0.009 |
| 35 | 1.6 | 0.8 | 0.0 | 0.1 | 10 | 7 | 2 | 4 | 210 | 52.50 | 64.0 | 28 | -0.011 |
| 36 | 1.0 | 0.8 | 0.4 | 0.05 | 12 | 9 | 3 | 2 | 150 | 75.00 | 56.3 | 31 | -0.012 |

Averages over the sets sharing a value:

| Value | Sets | Mean contacts (loop tracks) | Median brakes/lap | Loops without a lap |
|---|---|---|---|---|
| cone 0.8 | 18 | 5.0 (2.6) | 17.4 | 14 |
| cone 1.2 | 18 | 2.2 (0.0) | 2.5 | 0 |
| steering gain 1.0 | 12 | 5.1 (2.8) | 28.7 | 10 |
| steering gain 1.6 | 12 | 3.4 (1.1) | 8.5 | 4 |
| steering gain 2.4 | 12 | 2.2 (0.0) | 9.1 | 0 |
| centering 0.0 | 12 | 4.7 (1.9) | 7.0 | 6 |
| centering 0.4 | 12 | 3.4 (1.2) | 9.1 | 5 |
| centering 0.6 | 12 | 2.7 (0.8) | 11.2 | 3 |
| safety margin 0.05 | 18 | 3.7 (1.5) | 9.1 | 8 |
| safety margin 0.1 | 18 | 3.4 (1.1) | 8.1 | 6 |

By the stated order, rows 1 and 2 win (no contact on any track). They are not a winner:

- They are two cone-0.8 sets that happened not to touch a wall in `open_room` in that one
  run. Run three times each (8 sets, three repeats, summed), row 1's set (cone 0.8, centering
  0.6, margin 0.05, gain 1.6) has 9 contacts and brakes 8.9 times per lap against 2.5 for the
  floor profile:

| # | cone_half_angle_rad | centering_gain | safety_margin_m | Contacts | Loop contacts | Loop runs without a lap | Laps | Loop brakes | Brakes/lap | Lap time, loops summed (s) | Escapes | Min clearance (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 1.2 | 0.6 | 0.1 | 2 | 0 | 0 | 33 | 83 | 2.52 | 129.9 | 100 | -0.003 |
| 2 | 1.2 | 0.6 | 0.05 | 4 | 0 | 0 | 33 | 86 | 2.61 | 130.7 | 100 | -0.009 |
| 3 | 1.2 | 0.4 | 0.05 | 6 | 0 | 0 | 34 | 81 | 2.38 | 131.2 | 101 | -0.011 |
| 4 | 1.2 | 0.4 | 0.1 | 6 | 0 | 0 | 33 | 81 | 2.45 | 130.6 | 103 | -0.011 |
| 5 | 0.8 | 0.6 | 0.1 | 6 | 0 | 0 | 32 | 232 | 7.25 | 156.5 | 110 | -0.012 |
| 6 | 0.8 | 0.6 | 0.05 | 9 | 0 | 0 | 32 | 284 | 8.88 | 159.0 | 102 | -0.009 |
| 7 | 0.8 | 0.4 | 0.05 | 11 | 3 | 2 | 26 | 384 | 14.77 | 163.3 | 104 | -0.006 |
| 8 | 0.8 | 0.4 | 0.1 | 13 | 4 | 1 | 28 | 265 | 9.46 | 159.5 | 88 | -0.019 |

  (steering gain 1.6 throughout, so row 1 is exactly the floor profile.) Ranked on the
  repeats the floor profile is first.
- Cone 0.8 is 5 to 7 times worse on brakes per lap (median 17.4 against 2.5), about a quarter
  slower, and is the only value with wall contacts on the lane tracks and with loops it never
  finishes.
- Within cone 1.2 the 12 sets are indistinguishable on the loops (2.2 to 2.7 brakes per lap,
  129 to 133 s of laps) except that steering gain 2.4 with centering brakes much more (5.6 to
  21 per lap); the ranking among them is decided by 0 to 3 open-room contacts per run, which
  is noise.

A second sweep on `lane_open_side` alone with 1.0 m gaps (the setting that exercises the open
side), same 36 sets, two runs each: every set with cone 1.2 laps twice in each 60 s run with
no brake and no contact, whatever the steering gain, floor profile included (row 13 of 36:
4 laps in two runs, 0 brakes, 0.127 m clear). Cone 0.8 is clean only with steering gain 2.4;
below that it brakes, wedges or touches walls, and steering gain 1.0 with cone 0.8 fails
outright (up to 18 contacts in two runs, no lap). The fastest sets (23.7 s against 25.2 s per
lap) are cone 0.8 with centering 0.0, which the full battery above rules out.

## Recommendation

**Keep `floor-2026-10-06`. No `sim-2026-10-23` profile was added.** Nothing in the grid beats
it beyond run-to-run noise: it is first of the 8 sets run three times, and its cone (1.2 rad),
steering gain (1.6) and centering (0.6) sit on the plateau where the loops are clean (no wall
contact in over 100 loop runs with cone 1.2). What the sweep does show, to carry to the floor:

1. `cone_half_angle_rad` 1.2 is the one setting that matters here; 0.8 loses laps and walls.
2. `steering_gain` below 1.6 is only safe with the wide cone; 2.4 is as clean as 1.6 on the
   loops but brakes more with centering on.
3. `centering_gain` 0.4 to 0.6 versus 0.0 and `safety_margin_m` 0.05 versus 0.1 do not
   separate on the loops. If the floor shows one hugging a wall, try them there.

To tune beyond this, the lane tracks need a harder variant so the sets separate (narrower
lane: `--track-params-json '{"lane_width_m": 0.95}'`, a wider-gap open side, speed above 0.9
m/s) and `--repeats 3` to average out the open room.

## Limits of the sim, stated

- The bridge's /scan is cast from the gym pose with a 4.7 rad field of view, not the real C1
  at its mount with yaw pi; the canaries share this limit. The sim has no rear coverage, and
  f1tenth_gym's own collision handler does not fire when reversing, so contact is also judged
  from the body rectangle against the wall faces (the `body_overlaps_wall` reason).
- The default `lane_open_side` does not reproduce the gap-diving failure, only wider gaps do;
  the geometry matches the specification, not the floor.
- Wall thickness is 0.08 m, the gym's map is 0.05 m per cell: clearances below about 2 cm are
  at the map's resolution.
- The sim runs wall-clock real time and the controllers use wall-clock timers, so runs are not
  bit-reproducible (the open-room contact count varies from run to run); use `--repeats`.
- The 2 m/s default-profile result comes from the gym's dynamic tyre model; do not read it as
  a floor prediction.

## How to run

```sh
tools/reactive_bench_docker.sh --profile floor-2026-10-06,none --jobs 4
tools/reactive_bench_docker.sh --battery ci --check --no-note
tools/reactive_bench_docker.sh --profile floor-2026-10-06 --jobs 7 --stop-after-contacts 3 \
  --repeats 3 --sweep cone_half_angle_rad=0.8,1.2 --sweep centering_gain=0.4,0.6
tools/reactive_bench.py --from-json runs.json --note note.md      # re-rank without running
```

The docker script builds the workspace into a volume once, runs on an `--internal` network and
gives each run its own `ROS_DOMAIN_ID` (`--domain-base`). Four to seven runs side by side kept
the real-time factor at 1.00 on a 4-core laptop; check `real_time_factor` before trusting a
batch run harder than that.

## CI

Job `l5-reactive-bench` (next to `l5-reactive-lap`): the runner smoke test, then the floor
profile on `lane_square`, `lane_open_side` and `chicane`, each stopped at its first lap (limits
120, 80, 80 s of sim time). It fails on any wall contact or a missing lap. Measured locally the
battery takes 91 s of sim time (laps in 40.6, 25.1 and 25.3 s, 4 brakes and 4 escapes in the
square lane, none elsewhere) plus start-up, well inside the 5 minute bound. The L1 tests
(`tests/l5_reactive_bench/test_bench_metrics.py`, `tools/tests/test_reactive_bench.py`,
`sim/bridge/racer_gym_bridge/test/test_track_loader.py`) run in the python-tests job.
