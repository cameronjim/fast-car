# Lane-centre target for gap_follow_node, 2026-10-24

Floor finding that evening (bag `2026-10-24T23-13-18_car_teleop`, a closed foam-board loop about
1.0 to 1.1 m wide with corners near the car's 0.74 m turning radius): with the floor profile the
car rode the inside edge for the whole lap. Replayed, the gap-centre target aims at the outer
wall ahead of each corner, and the deepest-ray target sits at full lock whatever the gain. This
note records the new `target_mode lane_centre`, how it was checked, and why no new profile was
added. Nothing here ran on the Jetson or the car.

## What changed

| Piece | Where |
|---|---|
| `target_mode` (`gap_centre` default, `gap_deepest`, `lane_centre`) replaces the bool `target_deepest_ray`, which is still accepted (true means `gap_deepest`, warns once; true with `lane_centre` refuses to start) | `gap_follow.hpp` `TargetMode`, `resolve_target_mode`; `gap_follow_node.cpp` |
| Lane-centre target: returns split into wall chains, left and right boundary chains beside the car, lane width measured beside the car, a centreline trace (the curve midway between the two walls, or h from the one visible wall) walked `lane_lookahead_m` (0.6 m) from the car's place on it, pure pursuit to its end: `atan(2 L sin(alpha) / l_d)`, L from the binding | `gap_follow.hpp` (maths in the comment above `LaneGeometry`), `gap_follow.cpp` |
| New parameters `lane_lookahead_m` (0.6, [0.1, 3] m), `lane_default_half_width_m` (0.55, [0.1, 2] m); launch arguments for both and for `target_mode` | `gap_follow_node.cpp`, `gap_follow.launch.py` |
| Replay CLI and tool: `target_mode`, the aim point, trace and lane width per scan, purple trace and aim in `scans.png`, a steering-stats line (full lock share, mean abs steering, sign changes) | `gap_follow_replay_cli.cpp`, `tools/follower_replay/` |
| Bench metric: distance of the car from the lane centreline, mean per lap and over the run (`mean_centreline_offset_m`, `lap_centreline_offsets_m`, schema 2) | `tests/l5_reactive_bench/bench_metrics.py`, `tools/reactive_bench.py` |
| Seven corner scans of the floor bag as a fixture | `tools/tests/fixtures/floor_2026-10-24_corners.txt` |

The disparity bubble, the swept-path clamp (after the target) and the corner override are
unchanged and still run in `lane_centre`. `steering_gain` and `centering_gain` are not used in
that mode.

## 1. Replay of the floor bag (floor profile, `lane_lookahead_m` 0.6)

`tools/replay_follower.py data/bags/2026-10-24T23-13-18_car_teleop --profile floor-2026-10-06
--param target_mode=<mode> --plot --plot-at 0,46,70,93`. 1170 scans; the car is parked for the
first 40 s and wedged from 90 s, so the 40 to 90 s column is the driving.

| Steering after the clamps, before the low-pass | Full lock, all | Mean abs, all (rad) | Sign changes, all | Full lock, 40-90 s | Mean abs, 40-90 s | Sign changes, 40-90 s |
|---|---|---|---|---|---|---|
| `gap_centre` (today) | 27.4 % | 0.187 | 8 | 64.2 % | 0.318 | 8 |
| `gap_deepest` | 74.2 % | 0.314 | 2 | 93.4 % | 0.399 | 2 |
| `lane_centre` | 16.5 % | 0.175 | 12 | 38.6 % | 0.284 | 10 |

Before the swept-path clamp, `gap_deepest` asks for full lock on 100 % of the driving scans,
`gap_centre` on 68.6 % and `lane_centre` on 43.0 %. The low-passed `lane_centre` command is at
full lock on 24.2 % of the driving scans (gap_centre 43.6 %, gap_deepest 88.2 %). Its sign
changes are short dips to about -0.1 to -0.2 rad where the replayed car (which drove the gap
target's line, open loop) sits angled across the lane; it turns left (the loop is
counter-clockwise) everywhere else, and the 40 s parked section holds a steady 0.16 rad.

At the four corner timestamps (`scans.png`, purple line and diamond):

| t (s) | `gap_centre` target | `lane_centre` aim point (rear-axle frame) | `lane_centre` steering |
|---|---|---|---|
| 0.0 | the outer wall ahead, 1.26 m out at 0.17 rad | (0.59, +0.09) m, in the lane before the bend, h 0.51 m | 0.161 rad |
| 46.0 | the outer wall ahead, 0.93 m out, 0.41 rad wanted | (0.60, 0.00) m, mid lane short of the bend | -0.003 rad |
| 70.0 | the outer wall, 0.74 m out, full lock | (0.30, +0.33) m, mid lane round the bend | 0.419 (lock: the bend is tighter than the car) |
| 93.0 | the outer wall, 0.68 m, full lock | (0.06, +0.67) m, mid lane to the left (car wedged nose-in) | 0.419 wanted, 0 after the clamp |

Lookahead on the replay (driving window, after the clamps, before the low-pass): 0.4 m 29 sign
changes and 40.2 % at lock, 0.5 m 18 and 38.0 %, 0.6 m 10 and 38.6 %, 0.8 m 0 and 43.0 %, 1.0 m
0 and 41.0 %. The longer lookaheads lose the dips; 1.0 m is also the only one that lapped every
sim track without contact (below), so it is the value to try first on the floor.

## 2. Simulator bench (`lane_square`, `lane_round`, `lane_open_side`, `chicane`)

Floor profile with `target_mode lane_centre` on top, one run each, against the floor profile
(gap centre). Brakes are safety_node TTC engagements; the centre offset is the car's CG from
the lane centreline.

| Set | Track | Laps | Brakes | Brakes/lap | Contacts | Min clearance (m) | Centre offset mean (m) | per lap (m) |
|---|---|---|---|---|---|---|---|---|
| floor-2026-10-06 | lane_square | 3 | 15 | 5.00 | 0 | 0.030 | 0.048 | 0.048, 0.048, 0.049 |
| floor-2026-10-06 | lane_round | 2 | 12 | 6.00 | 0 | 0.014 | 0.055 | 0.053, 0.056 |
| floor-2026-10-06 | lane_open_side | 3 | 0 | 0.00 | 0 | 0.188 | 0.016 | 0.014, 0.016, 0.018 |
| floor-2026-10-06 | chicane | 3 | 0 | 0.00 | 0 | 0.212 | 0.016 | 0.018, 0.017, 0.016 |
| lane 0.6 | lane_square | 2 | 16 | 8.00 | 1 | -0.001 | 0.057 | 0.056, 0.055 |
| lane 0.6 | lane_round | 2 | 13 | 6.50 | 1 | -0.002 | 0.060 | 0.061, 0.062 |
| lane 0.6 | lane_open_side | 3 | 23 | 7.67 | 0 | 0.002 | 0.077 | 0.070, 0.076, 0.077 |
| lane 0.6 | chicane | 3 | 0 | 0.00 | 0 | 0.077 | 0.057 | 0.059, 0.056, 0.059 |
| lane 0.8 | lane_square | 3 | 15 | 5.00 | 0 | 0.013 | 0.060 | 0.060, 0.059, 0.058 |
| lane 0.8 | lane_round | 0 | 14 | - | 2 | -0.003 | 0.065 | - |
| lane 0.8 | lane_open_side | 3 | 2 | 0.67 | 0 | 0.016 | 0.090 | 0.085, 0.088, 0.090 |
| lane 0.8 | chicane | 3 | 0 | 0.00 | 0 | 0.087 | 0.078 | 0.075, 0.078, 0.080 |
| lane 1.0 | lane_square | 3 | 16 | 5.33 | 0 | 0.011 | 0.062 | 0.059, 0.060, 0.059 |
| lane 1.0 | lane_round | 3 | 12 | 4.00 | 0 | 0.012 | 0.056 | 0.055, 0.058, 0.056 |
| lane 1.0 | lane_open_side | 3 | 4 | 1.33 | 0 | 0.005 | 0.099 | 0.095, 0.105, 0.101 |
| lane 1.0 | chicane | 3 | 0 | 0.00 | 0 | 0.077 | 0.092 | 0.086, 0.096, 0.094 |

`lane_open_side` with 1.0 m gaps (the setting that tests the open side): floor profile 3 laps,
0 brakes, 0 contacts; `lane_centre` 0 laps, 19 brakes, 14 contacts at 0.6 m and 3 laps, 1
brake, 3 contacts at 1.0 m.

The 0.6 m contacts are grazes (-1 and -2 mm, at the map's 5 cm resolution) but they count.

**Result: `lane_centre` does not win the bench, so no `lane-2026-10-24` profile was added.**
At 1.0 m it is clean on the four tracks with fewer brakes per lap on `lane_round` (4.0 against
6.0) and the same on `lane_square` and `chicane`, but it brakes on the open side where the floor
profile does not, touches walls at wide gaps, and sits further from the centreline on every
track (6 to 10 cm against 2 to 5 cm). The centre offset is the surprise: on the straights the
trace is on the centreline (checked open loop on the same synthetic tracks), so the closed loop
is weaving about it, which the pure pursuit through the 0.15 s steering low-pass and 10 Hz
scans does not damp; that is the next thing to look at.

## How the trace got here (dead ends, for the next person)

- Climbing to the point of largest clearance circled in the wide pocket of a corner (aim points
  behind the car on 1 scan in 20 of the floor bag). Following the curve midway between the walls
  fixed it.
- Turning at most 20 degrees per 5 cm step made the trace overshoot and weave; any heading
  within a right angle, every 5 degrees, lets it walk along the curve.
- Counting the walk to the centreline as lane distance made an offset car aim short and
  saturate; the trace now starts at the car's place on the centreline.
- Gaps between boxes pulled the midway curve 15 cm into them (87 brakes in 3 sim laps). The
  fix, continuing a wall straight past its end or past a convex corner, had to be restricted to
  corners beside a straight parallel opposite wall that does not close the lane ahead: applied
  everywhere it steered the floor replay right at every inside corner and drove the sim's square
  and round lanes into the outside wall (13 to 23 contacts per run).

## Tests

gtest `LaneCentreTarget.*` (straight lane centred gives zero, offset right steers left by the
pure-pursuit amount, curved lanes give atan(L / R) and change smoothly with offset, one-sided
lane holds the offset, an opening past a wall's end and boxes along an open side do not pull the
trace, LEFT positive with the real C1 layout, clamp and corner override after the target);
replay CLI ctest (lane_centre on a centred lane, `target_mode` parsing and refusals, the floor
excerpt); pytest for the replay tool and the bench metric; launch L1 for the new arguments and
L3 for the new parameter ranges.
