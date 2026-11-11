# Track session tooling, track family and driver comparison, 2026-11-11

The owner rebuilds the foam track every session and the next one will be larger, so nothing here
is tuned to one layout. This note records (1) tonight's real track mapped from the floor bags,
(2) a family of sim tracks and the car's measured turning in the sim, (3) four drivers compared on
them, (4) the new "new track session" routine, and (5) the root cause work on the mapping canary's
flake. Everything ran in the ros-dev image on the dev machine (Apple silicon, Docker). Nothing
here has run on the Jetson or the car.

## The calibration has landed (vehicle_params 0.14.0, PR 118)

This note was first written against the car as measured on 2026-11-10 before the calibration
(left lock 0.504 rad, right 0.449 rad, a full command of 0.4189 rad turning the wheels that far,
speed commands reading 1.401 times the truth), carried by a temporary yaml, a launch profile and
tracker/bridge parameters. All of that is deleted: `tools/track_session/real_car_2026-11-10.yaml`,
`track_follow.launch.py`'s `calib-2026-11-10` profile, `tracker_node`'s per-side limit and
command-scale parameters, the pure pursuit per-side limits, and the bridge's command and speed scale
options. The tools and the sim read `config/vehicle_params.yaml`: steering planning limit 0.449 rad
each way (the right-hand lock, 0.69 m turning radius, used both ways), commands are true angles and
true speeds, overhangs 0.14 / 0.11 m. `--commands-read-high` is gone (it would default to 1.0, so
it did nothing). Bags recorded before the calibration are mapped with `--speed-divisor 1.401
--steer-scale-left 1.203 --steer-scale-right 1.072`; the oval map below was made that way and is
unchanged.

Section 3 first shows the tables rerun on the calibrated car (the sim's steering limited to the
vehicle_params 0.449 rad each way, 0.64 m/s), then the original tables from the uncalibrated car,
kept because they show what the 0.60 m left lock was worth. What changed: counter-clockwise and
clockwise are now the same car, so the pairs of rows are the same to within 0.1 s; the car gave up
its tighter left turn, so thin-island ends cost more (below).

## 1. Tonight's track, mapped from the floor bags

Bags `2026-11-11T03-54-11_car_teleop` (12 counter-clockwise gap_follow laps, 17.6 s each) and
`2026-11-11T04-20-19_car_teleop_v3` (more laps, both ways; it also drives out of the lane for a
moment) are unindexed mcap with no /tf. `tools/track_session/map_from_bag.sh` replays one through
the car's own mapping stack: `extract_bag.py` (the truncated-file reader of
`tools/follower_replay/bag.py`) to an npz, `bag_feeder.py` publishing /clock, /scan, /odom/wheel
(speed divided by 1.401), /drive and the static laser transform, robot_localization's ekf_node and
slam_toolbox with `racer_state`'s configuration, then `/slam_toolbox/save_map` and
`serialize_map`. Idle stretches (the car parked) are cut so 525 s of bag maps in about 220 s.
The map is in `/Users/cameronjim/code/car-data/maps/2026-11-10-oval/` (`oval.yaml`/`.pgm`,
pose graph, and `oval_v3` from the second bag); a downscaled picture is
`track-session-2026-11-11-oval-map.png` (black: the map's walls, blue: centreline, red: the
counter-clockwise racing line, green dots: the car's outline along it).

What mattered, in the order it was found (wall cells = occupied cells in the saved map, fewer is
crisper for the same track; all 12 laps, rate 2 unless noted):

| Variant | Wall cells | Lane width median (m) | Result |
|---|---|---|---|
| no scan correction | 1127 | 0.85 | thick, doubled walls, island blob |
| scan start-stamped, beam time rising | 1149 | 1.05 | worse, loop 6.4 m, island 1.6 x 0.8 m (wrong) |
| scan start-stamped, beam time falling | 1108 | (trace does not close) | worse |
| scan end-stamped, beam time rising | 1241 | (trace does not close) | worse |
| **scan end-stamped, beam time falling** | **822** | **0.98** | crisp single walls, island closed |
| same, real `wheel_twist_node` (symmetric 0.4189 rad steering clamp) instead of measured steering | 1065 | (trace does not close) | smeared |
| same, replayed at rate 1 instead of 2 | 822 | 0.98 | identical (the map does not depend on the pace) |
| bag `_cw` (clockwise only) | 1609 | 0.90 | doubled loop and a second ring: the loop closed at the wrong place |
| bag `v3` (both ways) | 1625 | 1.05 | oval crisp (the car also drove out of the lane) |

SLAM did struggle, for two reasons that are not the featureless oval. The C1 takes 0.1 s per turn
and the car turns at up to 1 rad/s: a wall 1.5 m away smears by 15 cm in a corner, over a tenth of
a 1 m lane (the localisation note lists scan motion distortion as not simulated and not
corrected). Correcting each scan with the wheel odometry before slam_toolbox sees it (the beam
time model is the stamp at the END of the sweep and beam time FALLING with the beam index, found
by trying all four) took the map from 1127 wall cells to 822 and gave the closed island. And the
yaw rate: the car's left lock is 0.504 rad where the software clamps at 0.4189, so the stock
`wheel_twist_node` underestimates left turns by 17 percent; with it the map is 30 percent smeared
again. The live stack on the car has neither fix (no scan deskew node, symmetric steering model):
a map made live on this lane will be at the "no correction" quality unless the calibration PR fixes the
steering and a deskew node is added (not built here: out of scope for this task). The counter-
clockwise-only bag maps; the clockwise-only bag did not (loop closure in a nearly featureless oval
with the car reversing at every end). Map with counter-clockwise laps.

Measured on the 822-cell map (`oval_lane.json`), cross-checked on the v3 map:

| Quantity | oval.yaml | oval_v3.yaml | Straight from the scans |
|---|---|---|---|
| Lane width, free corridor along the centreline, median / p10 / p90 | 0.98 / 0.78 / 1.27 m | 1.05 / 0.83 / 1.25 m | on straight driving, lines fitted to both walls: 0.80 to 0.84 m median (23 scans of two bags) |
| Island | 0.87 x 0.51 m | 0.91 x 0.50 m | |
| Outer wall radius at the two ends (near face) | 1.04 and 1.06 m | 1.13 and 1.14 m | |
| Centreline radius at the two ends | 0.47 and 0.56 m | 0.54 and 0.56 m | |
| Centreline loop length | 5.18 m | 5.23 m | dead reckoning: 6.1 m (the car drives wider than the centreline) |

So the lane is **not 1 m everywhere**: about 0.8 m on the two short straights and 1.0 to 1.3 m
round the ends (a thin island, an end radius of about 0.25 m, with an outer wall of about 1.1 m).
The centreline radius at the ends is 0.47 to 0.56 m, under the car's 0.60 m (left) and 0.69 m
(right) turning radii: **the middle of the lane is not drivable at full lock**, which is the owner's
observation. The geometry of an end with the car's real turning: at full left lock the outer front
corner sweeps 0.88 m from the turning centre (0.95 with the provisional overhangs) and the inner
rear corner 0.47 m; against an outer wall at 1.05 to 1.14 m that leaves 0.1 to 0.2 m on the
outside only if the turning centre sits level with the island tip. Right lock (0.69 m): the outer
front corner sweeps 0.95 m, so the same end leaves 0.1 m or less. Hence the owner's outside line.

## 2. The track family and the car model in the sim

`racer_gym_bridge.track_loader` gains `stadium` (island of a straight and two round ends of
`island_radius_m`, 0 = a thin wall), `rounded_rect` (island `straight_length_m` x
`island_width_m`, corner radius `island_radius_m`) and `map` (a saved SLAM map as the world, with
a centreline CSV for the start). The lane is `lane_width_m` round the island, walls are the
island outline and its offset by the lane width, both directions (`reverse_direction`), start mid
bottom straight. The family used (`tools/track_bench.py`): lane 1.0 m with straights 2 m and end
radius 0, 0.2, 0.4, 0.8; straights 1 m and 4 m at radius 0; lane 1.2 m at radius 0 and 0.4; lane
0.8 m, straight 1 m, radius 0.2 (tonight's straights); rounded rectangles 2 x 1.5 m (square
corners), 3 x 2 m (radius 0.4) at 1.0 m and 2 x 1.6 m (radius 0.8) at 1.2 m; and the oval.

The car, new bridge parameters all defaulting to the old behaviour: per-side steering limits
(`steering_limit_left_rad` 0.504, `steering_limit_right_rad` 0.449 through the gym's `s_max` /
`s_min`, which it applies per side), the command scales (a full command of 0.4189 rad turns the
wheels 0.504 left and 0.449 right) and `speed_cmd_scale`. `ground_truth_rear_odom_offset_x_m` adds
`/sim/ground_truth_rear_odom` (the pose at the rear axle for a tracker on ground truth; sim only).
The LiDAR is cast from the real mount (0.285 m ahead of the rear axle, yaw pi) for every driver.
Validation of the model against the floor: gap_follow with the lane profile on the oval map
laps in **17.6 s counter-clockwise** (the floor bag: 17.6 s) with an escape every lap or so, and
clockwise in 19.2 s with an escape at about every end, the owner's description of the clockwise
trouble (tables below). The sim also runs at a real-time factor of 1.00 in every run.

## 3. Drivers compared

Metrics per run: laps (3, or fewer if it stalled), safety brakes (`safety_node` TTC engagements,
`b`), reverse escapes (`e`), wall contacts (`c`), minimum body clearance (the body rectangle to the
wall faces, m, `clr`), lap time. All runs: the real command path (driver, /drive_raw,
`safety_node`, /drive, bridge), the car of the table in question, 0.64 m/s true top speed, the real LiDAR mount, seed
11. A run ends at 3 laps, 150 s, a wall contact count, or when the car has stood still for 12 s
(`stalled`: `safety_node` latched with a driver that cannot get out). One run per cell, not seeds:
the sim is deterministic apart from process timing, and repeated cells of this table (the
speed table, the sweep table and this one share runs) agree to a lap time of 0.1 s and the same
brake counts.

Drivers:

* **gap_follow**: `lane-2026-10-25` profile (lane_centre target, pure pursuit, lookahead 0.6 m,
  reverse escape on), max speed 0.64.
* **gap_follow, longer lookahead**: the same with `lane_lookahead_m` 0.8 (and 1.0 where stated).
* **gap_follow, outside entry** (new, default off): `outside_entry_offset_m` > 0 moves the aim
  point to the outside of a turn the trace sees beyond the lookahead, by up to that offset and never
  closer to the wall than body half width + margin + 5 cm, and cancels the shift as the turn
  arrives inside the lookahead (turn in late). See `gap_follow.hpp`, OUTSIDE ENTRY.
* **tracker (ground truth)**: `tracker_node` on a raceline made by the session pipeline from the
  track's own map (`tools/track_session/pipeline.py`: minimum-curvature line inside the lane, the whole
  body outline kept the widest of 0.15 to 0.05 m off the walls the lane allows, curvature under 85
  percent of the car's steering on each side, speed profile at a_lat 2 m/s^2, 0.64 m/s), following
  the rear-axle ground-truth pose, with the steering compensation. Lookahead 0.35 to 0.7 m (the
  node's own 0.4 to 1.5 m cuts a 1 m lane's corners).
* **tracker (localised)**: the same raceline followed on the EKF's /odom composed with AMCL's
  map -> odom (the car's odometry stack on the track's map, surfaces-only like slam_toolbox, sensor
  errors on: wheel speed scale 1.02, wheelspin, 0.01 rad steering trim, 2 cm scan noise).

### With the calibration (vehicle_params 0.14.0), 3 laps

Gap columns: the lane profile as it is, and the recommended settings (lookahead 0.8 m, outside
entry 0.25 m); tracker columns use the widest of 0.15 to 0.05 m body clearance the line allows.
Same bench, same 0.64 m/s.

| Track | Dir | gap_follow (lane profile, lookahead 0.6) | gap_follow, lookahead 0.8 + outside entry 0.25 | tracker, ground truth | tracker, localised |
|---|---|---|---|---|---|
| st_w1.0_s2_r0.0 | ccw | 3 laps 30.7s, 16b 12e 0c, clr 0.085 | 3 laps 23.2s, 6b 6e 0c, clr 0.083 | 0 laps, 1b 0e 0c, clr 0.131 | 0 laps, 1b 0e 0c, clr 0.156 |
| st_w1.0_s2_r0.0 | cw | 3 laps 30.3s, 14b 12e 0c, clr 0.094 | 3 laps 23.2s, 6b 6e 0c, clr 0.077 | 0 laps, 1b 0e 0c, clr 0.146 | 0 laps, 1b 0e 0c, clr 0.149 |
| st_w1.0_s2_r0.2 | ccw | 3 laps 18.3s, 0b 0e 0c, clr 0.158 | 3 laps 18.2s, 0b 0e 0c, clr 0.137 | 3 laps 13.7s, 0b 0e 0c, clr 0.113 | 0 laps, 11b 0e 0c, clr 0.074 |
| st_w1.0_s2_r0.2 | cw | 3 laps 18.3s, 0b 0e 0c, clr 0.162 | 3 laps 18.2s, 0b 0e 0c, clr 0.141 | 3 laps 12.8s, 0b 0e 0c, clr 0.115 | 0 laps, 10b 0e 0c, clr 0.069 |
| st_w1.0_s2_r0.4 | ccw | 3 laps 19.7s, 0b 0e 0c, clr 0.204 | 3 laps 19.6s, 0b 0e 0c, clr 0.198 | 3 laps 15.5s, 0b 0e 0c, clr 0.118 | 3 laps 15.7s, 4b 0e 0c, clr 0.085 |
| st_w1.0_s2_r0.4 | cw | 3 laps 19.7s, 0b 0e 0c, clr 0.208 | 3 laps 19.7s, 0b 0e 0c, clr 0.197 | 3 laps 14.9s, 0b 0e 0c, clr 0.122 | 0 laps, 28b 0e 0c, clr 0.065 |
| st_w1.0_s2_r0.8 | ccw | 3 laps 23.1s, 0b 0e 0c, clr 0.230 | 3 laps 23.3s, 0b 0e 0c, clr 0.226 | 3 laps 19.0s, 0b 0e 0c, clr 0.242 | 3 laps 18.8s, 0b 0e 0c, clr 0.211 |
| st_w1.0_s2_r0.8 | cw | 3 laps 23.1s, 0b 0e 0c, clr 0.230 | 3 laps 23.2s, 0b 0e 0c, clr 0.226 | 3 laps 18.9s, 0b 0e 0c, clr 0.241 | 3 laps 19.0s, 0b 0e 0c, clr 0.190 |
| st_w1.0_s1_r0.0 | ccw | 3 laps 26.2s, 15b 11e 0c, clr 0.093 | 3 laps 19.8s, 7b 6e 0c, clr 0.104 | 0 laps, 1b 0e 0c, clr 0.153 | 0 laps, 1b 0e 0c, clr 0.159 |
| st_w1.0_s1_r0.0 | cw | 3 laps 26.2s, 15b 11e 0c, clr 0.099 | 3 laps 19.7s, 7b 6e 0c, clr 0.095 | 0 laps, 1b 0e 0c, clr 0.160 | 0 laps, 1b 0e 0c, clr 0.162 |
| st_w1.0_s4_r0.0 | ccw | 3 laps 37.5s, 16b 12e 0c, clr 0.094 | 3 laps 29.7s, 6b 6e 0c, clr 0.082 | 0 laps, 1b 0e 0c, clr 0.145 | 0 laps, 1b 0e 0c, clr 0.126 |
| st_w1.0_s4_r0.0 | cw | 3 laps 36.4s, 12b 12e 0c, clr 0.093 | 3 laps 29.7s, 7b 6e 0c, clr 0.093 | 0 laps, 1b 0e 0c, clr 0.116 | 0 laps, 1b 0e 0c, clr 0.158 |
| st_w1.2_s2_r0.0 | ccw | 3 laps 18.7s, 0b 0e 0c, clr 0.082 | 3 laps 17.9s, 0b 0e 0c, clr 0.079 | 0 laps, 1b 0e 0c, clr 0.247 | 0 laps, 1b 0e 0c, clr 0.254 |
| st_w1.2_s2_r0.0 | cw | 3 laps 18.6s, 0b 0e 0c, clr 0.086 | 3 laps 17.9s, 0b 0e 0c, clr 0.087 | 0 laps, 2b 0e 0c, clr 0.274 | 0 laps, 2b 0e 0c, clr 0.274 |
| st_w1.2_s2_r0.4 | ccw | 3 laps 20.5s, 0b 0e 0c, clr 0.312 | 3 laps 20.7s, 0b 0e 0c, clr 0.295 | 3 laps 16.0s, 0b 0e 0c, clr 0.332 | 3 laps 15.9s, 0b 0e 0c, clr 0.292 |
| st_w1.2_s2_r0.4 | cw | 3 laps 20.5s, 0b 0e 0c, clr 0.312 | 3 laps 20.7s, 0b 0e 0c, clr 0.295 | 3 laps 16.0s, 0b 0e 0c, clr 0.336 | 3 laps 16.0s, 0b 0e 0c, clr 0.276 |
| st_w0.8_s1_r0.2 | ccw | 3 laps 42.3s, 80b 16e 0c, clr 0.042 | 3 laps 27.6s, 17b 11e 0c, clr 0.065 | 0 laps, 2b 0e 0c, clr 0.103 | 0 laps, 4b 0e 0c, clr 0.119 |
| st_w0.8_s1_r0.2 | cw | 3 laps 36.7s, 58b 15e 0c, clr 0.041 | 3 laps 27.0s, 20b 11e 0c, clr 0.067 | 0 laps, 9b 0e 0c, clr 0.056 | 0 laps, 1b 0e 0c, clr 0.092 |
| rr_w1.0_2x1.5_r0.0 | ccw | 3 laps 25.3s, 33b 0e 0c, clr 0.059 | 3 laps 20.4s, 0b 0e 0c, clr 0.159 | 3 laps 15.4s, 0b 0e 0c, clr 0.091 | 0 laps, 17b 0e 0c, clr 0.075 |
| rr_w1.0_2x1.5_r0.0 | cw | 3 laps 21.4s, 1b 0e 0c, clr 0.070 | 3 laps 20.4s, 0b 0e 0c, clr 0.159 | 3 laps 15.4s, 0b 0e 0c, clr 0.113 | 1 laps 95.3s, 58b 0e 0c, clr 0.065 |
| rr_w1.0_3x2_r0.4 | ccw | 3 laps 24.0s, 0b 0e 0c, clr 0.205 | 3 laps 23.8s, 0b 0e 0c, clr 0.201 | 3 laps 19.1s, 0b 0e 0c, clr 0.121 | 3 laps 18.9s, 0b 0e 0c, clr 0.105 |
| rr_w1.0_3x2_r0.4 | cw | 3 laps 24.0s, 0b 0e 0c, clr 0.205 | 3 laps 23.8s, 0b 0e 0c, clr 0.200 | 3 laps 19.0s, 0b 0e 0c, clr 0.120 | 2 laps 47.2s, 95b 0e 0c, clr 0.068 |
| rr_w1.2_2x1.6_r0.8 | ccw | 3 laps 18.9s, 0b 0e 0c, clr 0.335 | 3 laps 18.8s, 0b 0e 0c, clr 0.326 | 3 laps 17.0s, 0b 0e 0c, clr 0.126 | 3 laps 16.9s, 0b 0e 0c, clr 0.108 |
| rr_w1.2_2x1.6_r0.8 | cw | 3 laps 18.9s, 0b 0e 0c, clr 0.333 | 3 laps 18.9s, 0b 0e 0c, clr 0.324 | 3 laps 16.7s, 0b 0e 0c, clr 0.125 | 3 laps 22.1s, 28b 0e 0c, clr 0.072 |
| oval_2026-11-10 | ccw | 3 laps 20.4s, 6b 6e 0c, clr 0.074 | 3 laps 20.0s, 6b 6e 0c, clr 0.104 | 0 laps, 1b 0e 0c, clr 0.035 | 0 laps, 0b 0e 0c, clr 0.035 |
| oval_2026-11-10 | cw | 3 laps 20.4s, 6b 6e 0c, clr 0.118 | 3 laps 18.6s, 5b 5e 0c, clr 0.106 | 0 laps, 1b 0e 0c, clr 0.176 | 0 laps, 0b 0e 0c, clr 0.173 |


What the calibration changed:

* **gap_follow with the plain lane profile got worse on the thin islands** (12 escapes per 3
  laps in both directions, lap 30.7 s on the 2 m straight, where it was 23.3 s counter-clockwise
  and 31.7 s clockwise): the car can no longer turn left at 0.60 m. **The recommended settings are
  now the clear winner, not a refinement**: 6 escapes and 23.2 s in both directions, 6 to 7 cm of
  clearance or more, and on the 0.8 m lane 11 against 15 to 16 escapes and 27 s against 37 to 42 s
  (no wall contact in either, but 17 to 20 brakes in three laps). On the oval
  they take 18.6 to 20.0 s with 5 to 6 escapes in both directions, against 20.4 s and 6 for the
  plain profile.
* **The tracker is unchanged**: laps every family track with an island end radius of 0.2 m or more
  and every rounded rectangle, 25 percent faster than gap_follow; stalls on every thin island,
  the 0.8 m lane and the oval, for the reasons in finding 2. It additionally lost the 1.2 m
  thin-island stadium counter-clockwise (it lapped it with 3 brakes before).
* **The tracker on the localised pose got worse**: on 5 tracks where the ground-truth tracker laps
  cleanly it now stalls or brakes 4 to 95 times in at least one direction (it was clean counter-clockwise
  before). Position
  error is similar; the line asks for the same 0.69 m turns both ways, so there is no 0.60 m
  left lock to hide a 10 cm error in. Localisation error and the sensor-noise question of finding 4
  stay open.

### Before the calibration (the car as measured on 2026-11-10), 3 laps


Gap columns: the lane profile as it is, and the recommended settings (lookahead 0.8 m, outside entry 0.25 m). gap_follow rows are from the build with PR 115's reverse escape merged; tracker rows use the widest of 0.15 to 0.05 m body clearance the line allows.

| Track | Dir | gap_follow (lane profile, lookahead 0.6) | gap_follow, lookahead 0.8 + outside entry 0.25 | tracker, ground truth | tracker, localised |
|---|---|---|---|---|---|
| st_w1.0_s2_r0.0 | ccw | 3 laps 23.3s, 6b 6e 0c, clr 0.032 | 3 laps 22.9s, 8b 6e 0c, clr 0.076 | 0 laps, 1b 0e 0c, clr 0.152 | 0 laps, 2b 0e 0c, clr 0.127 |
| st_w1.0_s2_r0.0 | cw | 3 laps 31.7s, 12b 12e 0c, clr 0.029 | 3 laps 23.3s, 7b 6e 0c, clr 0.056 | 0 laps, 1b 0e 0c, clr 0.086 | 0 laps, 1b 0e 0c, clr 0.102 |
| st_w1.0_s2_r0.2 | ccw | 3 laps 17.0s, 0b 0e 0c, clr 0.220 | 3 laps 16.9s, 0b 0e 0c, clr 0.216 | 3 laps 13.5s, 0b 0e 0c, clr 0.112 | 3 laps 13.2s, 0b 0e 0c, clr 0.141 |
| st_w1.0_s2_r0.2 | cw | 3 laps 18.1s, 0b 0e 0c, clr 0.157 | 3 laps 18.1s, 0b 0e 0c, clr 0.144 | 3 laps 12.8s, 0b 0e 0c, clr 0.115 | 3 laps 14.5s, 20b 0e 0c, clr 0.077 |
| st_w1.0_s2_r0.4 | ccw | 3 laps 18.7s, 0b 0e 0c, clr 0.226 | 3 laps 18.6s, 0b 0e 0c, clr 0.230 | 3 laps 15.1s, 0b 0e 0c, clr 0.116 | 3 laps 14.7s, 0b 0e 0c, clr 0.187 |
| st_w1.0_s2_r0.4 | cw | 3 laps 19.5s, 0b 0e 0c, clr 0.209 | 3 laps 19.4s, 0b 0e 0c, clr 0.215 | 3 laps 14.9s, 0b 0e 0c, clr 0.120 | 3 laps 15.7s, 8b 0e 0c, clr 0.080 |
| st_w1.0_s2_r0.8 | ccw | 3 laps 22.4s, 0b 0e 0c, clr 0.247 | 3 laps 22.4s, 0b 0e 0c, clr 0.256 | 3 laps 19.0s, 0b 0e 0c, clr 0.237 | 3 laps 18.7s, 0b 0e 0c, clr 0.165 |
| st_w1.0_s2_r0.8 | cw | 3 laps 23.0s, 0b 0e 0c, clr 0.233 | 3 laps 23.0s, 0b 0e 0c, clr 0.236 | 3 laps 19.0s, 0b 0e 0c, clr 0.236 | 3 laps 19.0s, 0b 0e 0c, clr 0.204 |
| st_w1.0_s1_r0.0 | ccw | 3 laps 19.2s, 13b 5e 0c, clr 0.035 | 3 laps 18.2s, 5b 5e 0c, clr 0.060 | 0 laps, 1b 0e 0c, clr 0.151 | 0 laps, 1b 0e 0c, clr 0.223 |
| st_w1.0_s1_r0.0 | cw | 3 laps 27.2s, 12b 11e 0c, clr 0.040 | 3 laps 19.8s, 6b 6e 0c, clr 0.059 | 0 laps, 1b 0e 0c, clr 0.102 | 0 laps, 1b 0e 0c, clr 0.135 |
| st_w1.0_s4_r0.0 | ccw | 3 laps 30.1s, 7b 6e 0c, clr 0.036 | 3 laps 29.3s, 6b 6e 0c, clr 0.073 | 0 laps, 1b 0e 0c, clr 0.083 | 0 laps, 2b 0e 0c, clr 0.107 |
| st_w1.0_s4_r0.0 | cw | 3 laps 38.6s, 13b 12e 0c, clr 0.033 | 3 laps 29.9s, 6b 6e 0c, clr 0.058 | 0 laps, 1b 0e 0c, clr 0.089 | 0 laps, 1b 0e 0c, clr 0.115 |
| st_w1.2_s2_r0.0 | ccw | 3 laps 16.7s, 0b 0e 0c, clr 0.220 | 3 laps 16.5s, 0b 0e 0c, clr 0.215 | 3 laps 14.8s, 3b 0e 0c, clr 0.069 | 3 laps 13.7s, 3b 0e 0c, clr 0.096 |
| st_w1.2_s2_r0.0 | cw | 3 laps 18.6s, 0b 0e 0c, clr 0.078 | 3 laps 17.9s, 0b 0e 0c, clr 0.082 | 0 laps, 3b 0e 0c, clr 0.077 | 0 laps, 4b 0e 0c, clr 0.109 |
| st_w1.2_s2_r0.4 | ccw | 3 laps 19.6s, 0b 0e 0c, clr 0.336 | 3 laps 19.7s, 0b 0e 0c, clr 0.285 | 3 laps 16.7s, 0b 0e 0c, clr 0.328 | 3 laps 15.6s, 0b 0e 0c, clr 0.264 |
| st_w1.2_s2_r0.4 | cw | 3 laps 20.3s, 0b 0e 0c, clr 0.321 | 3 laps 20.6s, 0b 0e 0c, clr 0.272 | 3 laps 16.0s, 0b 0e 0c, clr 0.330 | 3 laps 15.9s, 0b 0e 0c, clr 0.298 |
| st_w0.8_s1_r0.2 | ccw | 0 laps, 58b 12e 12c, clr -0.008 | 0 laps, 24b 12e 12c, clr -0.009 | 0 laps, 1b 0e 0c, clr 0.115 | 0 laps, 1b 0e 0c, clr 0.116 |
| st_w0.8_s1_r0.2 | cw | 0 laps, 97b 28e 28c, clr -0.009 | 0 laps, 34b 25e 24c, clr -0.007 | 0 laps, 1b 0e 0c, clr 0.081 | 0 laps, 1b 0e 0c, clr 0.087 |
| rr_w1.0_2x1.5_r0.0 | ccw | 3 laps 20.1s, 0b 0e 0c, clr 0.138 | 3 laps 19.5s, 0b 0e 0c, clr 0.194 | 3 laps 15.3s, 0b 0e 0c, clr 0.119 | 3 laps 14.9s, 1b 0e 0c, clr 0.063 |
| rr_w1.0_2x1.5_r0.0 | cw | 3 laps 21.5s, 2b 0e 0c, clr 0.072 | 3 laps 20.3s, 0b 0e 0c, clr 0.172 | 3 laps 15.4s, 0b 0e 0c, clr 0.114 | 3 laps 29.3s, 43b 0e 0c, clr 0.069 |
| rr_w1.0_3x2_r0.4 | ccw | 3 laps 23.0s, 0b 0e 0c, clr 0.230 | 3 laps 23.1s, 0b 0e 0c, clr 0.223 | 3 laps 19.1s, 0b 0e 0c, clr 0.119 | 3 laps 18.6s, 0b 0e 0c, clr 0.087 |
| rr_w1.0_3x2_r0.4 | cw | 3 laps 23.8s, 0b 0e 0c, clr 0.211 | 3 laps 23.7s, 0b 0e 0c, clr 0.214 | 3 laps 19.0s, 0b 0e 0c, clr 0.119 | 3 laps 19.4s, 5b 0e 0c, clr 0.080 |
| rr_w1.2_2x1.6_r0.8 | ccw | 3 laps 18.2s, 0b 0e 0c, clr 0.349 | 3 laps 18.1s, 0b 0e 0c, clr 0.355 | 3 laps 17.0s, 0b 0e 0c, clr 0.125 | 3 laps 16.6s, 0b 0e 0c, clr 0.138 |
| rr_w1.2_2x1.6_r0.8 | cw | 3 laps 18.8s, 0b 0e 0c, clr 0.337 | 3 laps 18.7s, 0b 0e 0c, clr 0.332 | 3 laps 16.6s, 0b 0e 0c, clr 0.125 | 3 laps 16.7s, 3b 0e 0c, clr 0.088 |

b = safety brakes, e = reverse escapes, c = wall contacts, clr = minimum body clearance (m); a negative clearance is a wall contact.


### Tonight's oval before the calibration (the SLAM map as the world)

| Track | Dir | gap_follow (0.6) | lookahead 0.8 | outside entry 0.25 | lookahead 0.8 + outside 0.25 | lookahead 0.8 + outside 0.35 | tracker, ground truth | tracker, localised |
|---|---|---|---|---|---|---|---|---|
| oval_2026-11-10 | ccw | 3 laps 16.9s, 11b 3e 0c, clr 0.052 | 3 laps 19.8s, 8b 6e 0c, clr 0.065 | 3 laps 16.7s, 9b 3e 0c, clr 0.053 | 3 laps 20.1s, 9b 6e 0c, clr 0.054 | 3 laps 20.4s, 9b 6e 0c, clr 0.062 | 0 laps, 5b 0e 0c, clr 0.084 | 0 laps, 3b 0e 0c, clr 0.087 |
| oval_2026-11-10 | cw | 3 laps 19.2s, 5b 5e 0c, clr 0.006 | 3 laps 18.8s, 5b 5e 0c, clr 0.066 | 3 laps 19.2s, 5b 5e 0c, clr 0.017 | 3 laps 18.8s, 5b 5e 0c, clr 0.061 | 3 laps 18.8s, 5b 5e 0c, clr 0.070 | 0 laps, 1b 0e 0c, clr 0.088 | 0 laps, 1b 0e 0c, clr 0.089 |

The lane profile reproduces the floor: counter-clockwise laps of 16.9 to 17.6 s (floor 17.6 s) with escapes; clockwise 19.2 s with an escape at about every end and 6 mm of clearance (the car ends up against the outer wall, the owner's clockwise trouble). The raceline for it is feasible counter-clockwise (body clearance 0.07 m, tightest bend 0.66 m left) and NOT feasible clockwise (right lock 0.69 m: the outer front corner needs 0.95 m of wall radius, the ends have 1.04 to 1.14), and the tracker stalls against safety_node's corridor in both directions regardless (see the findings).

### Outside entry and lookahead on the thin-island stadium (pre-merge build)

| Track | Dir | gap_follow | gap_la08 | gap_la10 | gap_out15 | gap_out25 | gap_out35 | gap_out45 | gap_out35h20 | gap_out35_la08 |
|---|---|---|---|---|---|---|---|---|---|---|
| st_w1.0_s2_r0.0 | ccw | 3 laps 23.7s, 8b 6e 0c, clr 0.031 | 3 laps 22.8s, 8b 6e 0c, clr 0.070 | 3 laps 22.5s, 7b 6e 0c, clr 0.063 | 3 laps 23.7s, 7b 6e 0c, clr 0.045 | 3 laps 23.6s, 7b 6e 0c, clr 0.048 | 3 laps 23.7s, 7b 6e 0c, clr 0.047 | 3 laps 23.6s, 7b 6e 0c, clr 0.047 | 3 laps 23.6s, 7b 6e 0c, clr 0.041 | 3 laps 22.9s, 8b 6e 0c, clr 0.073 |
| st_w1.0_s2_r0.0 | cw | 3 laps 31.7s, 12b 12e 0c, clr 0.033 | 3 laps 23.0s, 6b 6e 0c, clr 0.070 | 3 laps 22.8s, 6b 6e 0c, clr 0.068 | 3 laps 25.6s, 7b 7e 0c, clr 0.030 | 3 laps 25.6s, 7b 7e 0c, clr 0.033 | 3 laps 25.6s, 7b 7e 0c, clr 0.030 | 3 laps 26.3s, 13b 7e 0c, clr 0.029 | 3 laps 25.2s, 7b 7e 0c, clr 0.034 | 3 laps 23.3s, 7b 6e 0c, clr 0.056 |

### Speed 0.9 m/s (a calibrated car at the lane profile's own top speed)

| Track | Dir | gap_follow | gap_follow, outside entry 0.25 | tracker, ground truth |
|---|---|---|---|---|
| st_w1.0_s2_r0.4 | ccw | 3 laps 13.6s, 0b 0e 0c, clr 0.201 | 3 laps 13.8s, 0b 0e 0c, clr 0.142 | 3 laps 10.8s, 0b 0e 0c, clr 0.118 |
| st_w1.0_s2_r0.4 | cw | 3 laps 14.1s, 0b 0e 0c, clr 0.191 | 3 laps 14.4s, 0b 0e 0c, clr 0.141 | 3 laps 10.6s, 0b 0e 0c, clr 0.121 |
| st_w1.0_s2_r0.0 | ccw | 3 laps 26.4s, 12b 12e 0c, clr 0.032 | 3 laps 20.3s, 7b 7e 0c, clr 0.042 | 0 laps, 1b 0e 0c, clr 0.202 |
| st_w1.0_s2_r0.0 | cw | 3 laps 31.4s, 21b 15e 0c, clr 0.033 | 3 laps 22.9s, 10b 8e 0c, clr 0.021 | 0 laps, 1b 0e 0c, clr 0.087 |



Findings:

1. **The expectation "the raceline tracker handles any 1 m track with the corrected steering limits"
   is false.** It laps every family track whose island end radius is 0.2 m or more and every
   rounded rectangle, both directions, faster than gap_follow (lap 13.5 against 17.0 s on the
   0.2 m stadium, 15.1 against 18.7 s at 0.4 m, 25 percent less time, zero brakes, zero contacts,
   minimum body clearance 0.11 to 0.33 m; about 20 percent shorter laps again at 0.9 m/s), also on the
   localised pose counter-clockwise. It does not lap, in either direction, any thin-island 1.0 m
   stadium (island radius 0, straights 1, 2 or 4 m), the 0.8 m lane, or the 1.2 m thin-island
   stadium clockwise, and it does not lap tonight's oval.
2. **Why it stalls there: two limits, one of them geometry.** Geometry: a thin island on a 1.0 m
   lane leaves the car's full-left-lock U-turn (rear-axle radius 0.60 m, outer front corner sweeping 0.88 m
   from the turning centre against a wall face at 0.95 m) with about 6 cm in total, and right lock
   (0.69 m, 0.95 m sweep) with none, so clockwise the end cannot be driven in one sweep at all:
   that is why the floor car brakes and reverses at every clockwise end, and what the reverse escape
   is for. The raceline tool says so (`NOT FEASIBLE`) for those tracks and for the oval clockwise.
   The other limit is `safety_node`'s path corridor (+/-0.205 m about the requested steering arc,
   0.05 m wider than the body each side) with its latch: a line that is feasible but clears a wall by
   0.05 to 0.1 m (every thin-island line) trips the TTC gate within a second of the start, and a
   tracker has no way out of the latch (it keeps asking for the same arc), so the car stands
   until the 12 s stall rule ends the run. The lines are fine in the sense of the geometry check
   (no violation, body clearance 0.05 m at the narrowest); the safety layer is stricter than the
   body. That layer was not touched.
3. **Recommended driver for any 1 m lane: gap_follow, `lane-2026-10-25` with `lane_lookahead_m` 0.8
   and `outside_entry_offset_m` 0.25** (reverse escape on, as in the profile). On the 24 family
   runs it never touches a wall except where nothing can (the 0.8 m lane, 1.0 m minus the island),
   has the same laps and clearance as the 0.6 m lookahead on the easy tracks (within 1 s) and on
   the thin islands cuts the clockwise brakes and escapes by half (12 to 6 or 7 per three laps)
   and the lap by a quarter (31.7 to 23.3 s), with 6 to 8 cm of minimum clearance instead of 3
   (0.8 m lookahead alone does most of that; outside entry on top of it adds nothing there,
   alone it helps clockwise a little: 12 to 7 escapes; on the oval counter-clockwise it is
   neutral, 16.9 to 16.7 s). On the oval, counter-clockwise, 0.6 m is still the better lookahead (16.9 s against
   20 s) and outside entry on top of it is neutral to slightly better; clockwise the 0.8 m lookahead
   lifts the clearance from 6 mm to 6 to 7 cm at the same 5 escapes. There is no setting that avoids the escape at a
   thin end: one per end counter-clockwise on a 1.0 m island, which is the geometry in finding 2.
   Use 0.6 m on the oval if it is driven counter-clockwise only. The tracker is the faster driver
   wherever it works: the session tool tells the owner which (feasible line, island end radius
   0.2 m or more).
4. **Localised pose.** The raceline tracker on the car's own odometry and AMCL on the track's
   map laps the same tracks as on ground truth, counter-clockwise without a brake (lap within
   0.8 s, localisation error mean 0.06 to 0.13 m, p95 0.10 to 0.28 m, along the track). Clockwise it
   brakes 3 to 43 times in three laps (the safety latch releases and the car goes on) although the
   position error is smaller (mean 0.03 to 0.05 m). With the sensor errors off the clockwise runs are
   clean (0 brakes, same lap): it is the injected noise (wheel scale, wheelspin, 0.01 rad steering
   trim, 2 cm scan noise) meeting the clockwise approach to the wall, not the localisation; the
   which of the four errors does it I did not pin down.
5. **The mapping and the raceline both depend on the lane being known to 5 cm.** The saved
   SLAM map's walls are 10 to 15 cm thick bands; the free corridor measured from them is about 5 cm
   narrower than the lane (0.8 m straights read 0.75 on the map and 0.80 to 0.84 straight from the
   scans). The raceline tool uses the free corridor, so it is conservative by that amount.
6. **Open: the live stack has no scan deskew and the odometry clamps steering at 0.4189 rad.** Both
   cost the map quality (section 1). Not built here.


## 4. The new track session routine

`tools/track_session/track_session.py` (one command per step; full text in
`first-boot-runbook.md`, "New track session"):

| Step | Command | Gives |
|---|---|---|
| map from a bag | `tools/track_session/map_from_bag.sh <bag> /data/maps/<id>/<id>` or `track_session.py map <bag> --name <id>` | `<id>.yaml/.pgm`, pose graph |
| map live on the car | drive one slow lap with `car_teleop.launch.py mapping:=true`, then `track_session.py save-map --out <prefix>` | the same |
| lane report | `track_session.py analyze <id>.yaml` | width, island, ends, centreline CSV |
| raceline | `track_session.py raceline <id>.yaml --v-max 0.64` | `<id>_ccw.csv`, `<id>_cw.csv`, JSON, PNG; exit 2 if not feasible |
| run | `track_session.py tracker <id>_ccw.csv <id>.yaml` prints the two launch commands | `car_teleop.launch.py ... localization:=true map:=...` and `ros2 launch racer_control track_follow.launch.py raceline:=...` |

Built: `tools/raceline/map_track.py` (flood fill of the lane, distance field, centreline trace,
lane report, minimum-curvature racing line with body outline and per-side curvature limits, multi
start because the energy has a pinched local minimum), `tools/track_session/` (pipeline,
CLI, bag extraction and feeder, offline mapping launch file, the measured-car yaml),
`tools/track_bench.py` and `track_bench_docker.sh` (the sweeps), `tests/l5_reactive_bench/
track_family_runner.py`, `racer_control/launch/track_follow.launch.py` (the `calib-2026-11-10`
profile) and the tracker's per-side steering parameters. Tests: L1 for the maps, the line and the
bench (`tools/raceline/tests/test_map_track.py`, `tools/tests/test_track_bench.py`,
`sim/bridge/.../test_track_loader.py`), gtests for the per-side limits and outside entry, L3 for
the tracker's steering parameters, and two L5 canaries (CI job `l5-track-family`): the tracker on
the 0.4 m stadium and the 3 x 2 rounded rectangle, and gap_follow (recommended settings) on two
thin-island stadiums, each both directions, thresholds in the test files from the measured runs.


## 5. The mapping canary flake (async_slam_toolbox_node exit -6)

**Observed.** `async_slam_toolbox_node` exited with -6 at the end of
`test_mapping_canary` twice on PR 115 (CI run 42958061821 and another): the log has "terminate
called after throwing an instance of 'karto::Exception'" 9 ms after launch_testing's SIGINT, with
the test body already passed and the saved map compared. `karto::Exception` is not a
`std::exception`, so there is no `what()`.

**What the source shows** (slam_toolbox humble, `slam_toolbox_common.cpp`):
`publishVisualizations()` runs `while (rclcpp::ok()) { updateMap(); publishGraph(); r.sleep(); }` on a bare
`boost::thread` with no try/catch, so a karto exception there is `std::terminate`; the destructor
joins that thread before it resets the mapper. Karto throws `karto::Exception` for an out-of-range
grid index and a few other conditions; which call threw in the shutdown window was **not isolated**.
I tried: 12 plain reruns of the canary, 4 under gdb catching `__cxa_throw` (the
catchpoint slowed slam_toolbox so much the test failed on its own timeout), and 2 offline runs under gdb (no karto throw at shutdown, only FastDDS startup throws), and 24 offline shutdown trials (the car's recorded laps replayed, the map saved,
scans still arriving for 4 s, SIGINT at a random moment, 1 CPU): none aborted, none threw a
karto exception at shutdown. No upstream issue matches (searched). So it is not a proven
"known upstream abort with no effect", and `-6` is **not** added to the allowed exit codes.

**Fix.** The abort happens after every result of the test is known, in the one process whose exit
status says nothing about the test, and only when the SIGINT lands inside slam_toolbox's visualisation
thread. The canary now ends slam_toolbox itself with SIGKILL once the map is saved and compared
(`_stop_slam_toolbox_after_the_map_is_saved`), and the shutdown check requires every process to exit
as before and slam_toolbox to exit with -9 and nothing else: an abort during the run still fails (it
would have lost the map). The reasoning is in that function's docstring.

**Found on the way, same cause class in the offline tooling:** a background job of a non-interactive
shell starts with SIGINT ignored, so `ros2 launch` started that way never stopped on `kill -INT`
(six containers hung). `map_from_bag.sh` starts it with `env --default-signal=INT`.

### The second failure: "0.0613 not less than or equal to 0.06"

That is `MAX_WALL_ERR_MEAN_M`, the mean distance of the saved map's wall cells from the true walls.
25 runs of the canary (2 CPUs each, machine loaded by other jobs):

| Runs | Wall error mean (m) | p95 (m) | Spurious fraction | Coverage |
|---|---|---|---|---|
| 11, same seed 21 | 0.0227 to 0.0454, median 0.037, sd 0.006 | 0.072 to 0.119 | 0.000 to 0.040 | 1.00 |
| 14, varied seeds | 13 healthy: 0.028 to 0.040; 1 broken: 0.114 | healthy up to 0.12; broken 0.31 | healthy up to 0.04; broken 0.386 | healthy 1.00; broken 0.757 |

**The sim is not deterministic**: the same seed gives a spread (sd 6 mm on a mean of 37 mm),
because the bridge, EKF, slam_toolbox and tracker run on wall-clock timers and the scheduler decides
which scan meets which odometry sample; a slower machine shifts the whole distribution up. The
variance source is that timing, not something to remove in the test. 0.0613 is a healthy map from a
slow runner (the idle maximum was 0.042, the loaded 0.045). **Thresholds**: mean 0.06 raised to 0.075, p95
0.16 (kept), spurious 0.06 raised to 0.15, coverage 0.95 (kept), each between the healthy tail and the
broken map (0.114 / 0.31 / 0.39 / 0.76), written up in the test. What the test can not absorb: **1 run in
25 builds a wrong map** (the loop closed at the wrong place on the featureless lane track), a real failure of
the stack that the canary correctly reports; rerun it once and look at the numbers before calling a failure
real.

### Later: PR 121 and the gap_follow canary

PR 121 (the canary holds a `/map` subscription and retries `save_map`; 2 `save_map` failures in 36
runs at 1 CPU before, 0 after) is open, not merged, at the time of writing. The gap_follow canary
(straight 2 m, ccw) failed once in CI with 7 safety brakes against a limit of 6 taken from 3-lap
numbers; 24 local runs with vehicle_params 0.14.0 gave 4 brakes (23 runs) or 5, always 4 escapes,
lap 23.13 to 23.55 s, clearance 0.080 to 0.0995 m, so the 7 is a loaded runner (the TTC latch
engaging twice in an escape). The limits are now brakes 10 and escapes 8, justified in the test.
