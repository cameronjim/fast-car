# Reactive controllers ported to C++, 2026-10-05

GitHub issue 26. The old Python reactive controllers from the Foxy project
(`f1tenth-autonomous-racing/reactive_control/`, moving to `legacy/f1tenth-autonomous-racing/`
with the same contents) are now C++ in `ros_ws/src/racer_control`. The owner asked for
corrections and improvements rather than a literal translation, so the maths changes below
are deliberate. The old folder is untouched.

## What was ported

| Old Python | New C++ |
|---|---|
| `gap_logic.py` | `include/racer_control/gap_follow.hpp`, `src/gap_follow.cpp` |
| `wall_logic.py` | `include/racer_control/wall_follow.hpp`, `src/wall_follow.cpp` |
| `pid.py` | `include/racer_control/pid.hpp`, `src/pid.cpp` (wall follower only) |
| `gap_follow_node.py` | `src/gap_follow_node.cpp`, `launch/gap_follow.launch.py` |
| `wall_follow_node.py` | `src/wall_follow_node.cpp`, `launch/wall_follow.launch.py` |
| `test_gap_logic.py`, `test_wall_logic.py` | `test/test_gap_follow.cpp`, `test/test_wall_follow.cpp` |

Shared pieces: `laser_scan.hpp` (scan geometry, bearing lookup, invalid-return policy, LiDAR
yaw resolution) and `reactive_speed.hpp` (speed laws). All of it is ROS-free and gtest-covered;
the nodes are thin plumbing.

Both nodes subscribe `/scan` (KeepLast 10, best_effort) and publish `/drive_raw` (KeepLast 10,
reliable) at 50 Hz from a fixed timer, never `/drive`. They use tracker_node's clock policy
(steady clock for elapsed time, ROS clock only for stamps), its float32 wire margin, and its
watchdog rule: on `/scan` silence they stop publishing and safety_node brakes.

Physical constants come only from the generated vehicle_params binding: half width is
`chassis.width_m / 2` (0.155 m; the old code hard-coded 0.5 m), steering clamp is
`steering.max_angle_rad`, speed is capped by `limits.global_speed_cap_mps` (the `max_speed_mps`
parameter is range-limited below it) and ramped through `SpeedRateLimiter` from
`actuation.max_acceleration_mps2`. The LiDAR yaw comes from `sensors.lidar.mount_yaw_rad` once
it is measured; until then a `laser_yaw_offset_rad` parameter (default 0) is used, and a
parameter that disagrees with a measured binding value refuses to start.

## What was dropped, and why

- `safety_node.py` and `safety_logic.py`: racer_safety/safety_node already owns TTC braking,
  the kill latch and the `/drive_raw` to `/drive` gate. A second safety node would duplicate
  layer 3.
- `cv_node.py`: out of scope for this issue.
- The `/kys` and `/speed` topics: they were the old safety node's kill latch and speed
  command. Kill is layer 1 (the mux) and layer 3 (safety_node); speed is now computed by each
  controller (below).
- Publishing `/drive` directly: forbidden by CLAUDE.md invariant 1.
- The SIGINT wind-down (coast for 2 s, then publish a zero): on shutdown the nodes just stop
  publishing and safety_node brakes on `/drive_raw` silence.
- The `/odom` subscription: the old nodes read it only for the wall follower's speed * dt
  lookahead, which is gone.

## Maths changes

a. **Forward ray and cone.** The old code took ray `num_rays / 2` as straight ahead and defined
   the cone and corner sectors as fractions of the ray count. That is wrong for a 360 degree
   RPLIDAR C1 (`angle_min = -pi`) and for any scan not centred on forward. The forward ray is
   now the ray nearest vehicle bearing 0, from `angle_min` and `angle_increment`; the cone is a
   half-angle in radians (`cone_half_angle_rad`, default 1.57) and wraps across the seam of a
   full-circle scan; corner sectors are radians too. `laser_yaw_offset_rad` handles a LiDAR
   mounted backwards (pi).
b. **Invalid returns.** inf and above-`range_max` are free space, clipped to
   `clip_max_range_m`. NaN, -inf, zero and below-`range_min` are invalid and filled from the
   nearest valid ray (smaller range on a tie). The RPLIDAR driver reports no-return as inf; the
   old Hokuyo path reported 0. A scan with no valid returns produces no command. No NaN reaches
   the output; the nodes also re-check before publishing.
c. **Disparity extension.** The bubble is `ceil(atan2(half_width + safety_margin_m, near) /
   angle_increment)` rays starting at the first ray on the far side of the edge. The old
   `int()` truncation is replaced by `ceil` because the real edge lies somewhere inside the
   near ray's angular bin; `near == 0` is guarded (a quarter turn); `safety_margin_m` defaults
   to 0.1 m. Edges are found on the input ranges, so the result does not depend on edge
   order. Work is O(n + total bubble length).
d. **Gap selection.** Still the widest free gap in the cone, tie-broken toward forward, aimed
   at its centre. `target_deepest_ray` (default off) aims at the gap's deepest ray instead,
   tie-broken toward forward. Both are tested. Two optional scoring terms were added after
   the 2026-10-06 floor test; both default to off, which is exactly this behaviour (see
   "Gap selection after the 2026-10-06 floor test" below).
e. **Steering.** A PID on a target bearing is not meaningful (its I term winds up on any
   steady curve), so it is gone: `steering = clamp(steering_gain * bearing, +/- max_angle)`,
   then a first-order low-pass (`steering_time_constant_s`, default 0.1 s) at the 50 Hz
   command rate. The corner override is kept with its sector in radians, and its side is
   fixed: the old code checked the last sixth of the rays for a right turn, but in LaserScan
   ordering those are on the LEFT, so it guarded the wrong side.
f. **Speed.** `speed = clamp(k_speed * range along the target ray, min_speed, max_speed)`,
   times `1 - k_steer * |steering| / max_angle`, then `SpeedRateLimiter`. The wall follower uses
   `max(min_speed, max_speed * (1 - k_steer * |steering| / max_angle))`.
g. **Wall follow.** Same two-ray geometry (alpha, D, D + L sin alpha). L was speed * dt, one
   timestep of travel, which is near zero; it is now `lookahead_m` (default 0.5 m). Deadband
   is a parameter. Ray bearings are parameters in the vehicle frame: negative follows the
   right wall, positive the left wall, and the steering sign flips with the side. Too far from
   the right wall steers right (negative), pinned in a test. A ray outside the scan's coverage
   now gives no measurement instead of silently reading the end ray; with no measurement the
   node steers straight and resets the PID. The PID has no magic first-step dt, skips the
   derivative on the first sample, takes its integral clamp as a parameter, and gets its dt
   from the steady clock between scans.

## Gap selection after the 2026-10-06 floor test

Bag `2026-10-06T22-12-40_car_teleop`: a lane with a continuous wall on one side and scattered
objects on the other. The follower picked a gap between the objects, into the open room, in 62
of 63 recorded scans under every parameter set tried. "Widest run of free rays" prefers an
opening close to the car on the side, because it subtends a wider angle than the lane ahead.
The owner wants the lane ahead unless a side opening is much bigger. Two parameters, both in
[0, 1], both default 0 (off), both range-checked by the node (out of range refuses to start)
and exposed as `gap_follow.launch.py` arguments:

- `forward_preference` p. Each gap scores
  `angular_width_rad * max(0, 1 - p * (1 - cos(centre_bearing)))`, with the centre bearing in
  the vehicle frame (laser yaw applied). The best score wins; exact ties still go to the gap
  nearer forward, and the centre or deepest target is still chosen inside the winning gap.
  p = 0 is exactly the widest gap (pinned bit-identical against a copy of the old code on the
  existing fixtures and 800 seeded random scans). p = 1 scores a gap at 90 degrees zero. The
  weighting is even in the bearing, never rewards turning away, and is quadratic near 0, so a
  lane a few degrees off axis is barely penalised (10 degrees costs p * 1.5 percent). The
  clamp at 0 only matters past 90 degrees with p > 0.5. With p = 0.6 a gap centred at 60
  degrees needs 1.43 times the width of one dead ahead to win.
- `gap_switch_margin` m. If a gap contains the previous scan's target bearing, it is kept
  unless the best gap's score exceeds its score by more than the fraction m. This stops the
  target flipping between two similar gaps on alternate scans.

The launch arguments default to empty, meaning "not set here", so a value in a params file is
not overridden by a launch default.

In an idealised copy of the floor scene (1.2 m lane, 2 m opening centred at 70 degrees on one
side, real C1 geometry with yaw pi), p = 0 picks the opening on either side and p = 0.6 picks
the lane. That is a synthetic test, not a tuning recommendation: pick p on the floor. The L5
canary runs with the defaults and is unaffected.

## Swept-path clamp (2026-10-06 floor finding)

Bag `2026-10-06T22-12-40_car_teleop`, replayed: a counter-clockwise loop with loose objects on
the inside of every left corner. The follower steered to a geometrically correct gap, but the
car's inside flank swept across the apex object: 62 of 63 scans chose a gap whose turn passed
within the car's width of an object beside the car. The disparity bubble only inflates
obstacle edges angularly from the near edge inside the search cone, so an object alongside the
car, 60 to 100 degrees off the LiDAR's axis, never constrains the turn.

The fix is a steering clamp after `steering_from_bearing` and before the corner override
(`clamp_steering_to_swept_path` in `gap_follow.hpp`). Geometry, in the rear-axle frame (x
forward, y left), worked for a left turn (a right turn is the mirror image):

- For steering delta the rear axle follows a circle of radius R = L / tan(delta) about (0, R),
  L = `chassis.wheelbase_m`. Same model as racer_safety's arc corridor; no code is shared.
- With c = `chassis.width_m / 2` + `safety_margin_m`, the inside flank sweeps the circle of
  radius R - c (the body point nearest the centre is (0, half width)), and the outside front
  corner sweeps hypot(body_front_x, R + c), with body_front_x = `cg_to_rear_axle_m` +
  `length_m / 2` (the bounding box taken as centred on the CG, the f1tenth_gym convention its
  values come from).
- Returns are taken from the raw scan (invalid returns ignored, not filled), turned to the
  vehicle frame with the laser yaw and shifted by `sensors.lidar.mount_x_m / mount_y_m`. The
  node refuses to start while those are null.
- A return counts only if it is on the turn-in side outside the car's width (y > half width;
  a return straight ahead inside the car's width is the TTC gate's job), ahead along the turn
  (arc angle atan2(x, R - y) in (0, pi/2)), within `swept_path_lookahead_m` (default 1.0 m) of
  rear-axle arc length, and inside the swept annulus R - c < rho <= outer radius.
- Closed form: a return clears the inside flank by the margin exactly when the curvature
  k = 1 / R satisfies k <= 2 (y - c) / (x^2 + y^2 - c^2) (0 when y <= c, so a return within
  the margin beside the car forbids any turn toward it). Starting from the wanted curvature,
  k drops to the smallest such bound among the returns in the swept area, and repeats with
  the window re-evaluated at the new k until nothing violates (one pass per return at most,
  usually one or two). Output delta = atan(k L), with the wanted sign.

The steering is unchanged bit for bit when nothing violates or the clamp is off
(`swept_path_clamp`, default true). `GapFollowResult` carries `wanted_steering_rad` and
`swept_path_clamped`; the node logs the clamp at DEBUG at most once per second. If the clamp
drives the steering near zero while the target bearing is large, the corner override and
safety_node take over. Both parameters are also `gap_follow.launch.py` arguments (empty =
node default).

Example from the tests (real C1 geometry, margin 0.05 m): an object 0.5 m left of the head at
x 0.1 to 0.4 m ahead of the head. Full left (0.4189 rad, R 0.742 m) sweeps the inside flank
through it; the clamp gives about 0.28 rad (R 1.15 m), set by the object's far corner at
(0.685, 0.5) in the rear-axle frame, and the inside flank circle then misses every return by
exactly the margin.

Sim note: racer_gym_bridge raycasts from the car's pose with no LiDAR offset, while the node
applies the car's measured 0.285 m mount (rear axle to head). In the canary the clamp
therefore places returns forward of where the sim saw them, by the difference between the
mount and the gym's pose reference point. The clamp is a car feature; the canary only checks
it does not break the lap.

With the clamp on by default the L5 canary below still passes: 26.04 s and 25.95 s for two
laps on two runs, worst distance from the raceline 0.165 m and 0.163 m (was about 26.0 s and
0.163 m).

## Checkpoint 2026-10-06: first working floor laps

On the evening of 2026-10-06 gap_follow_node completed its first working laps on the floor, on
the owner's living-room lane, with the node parameters below. They are saved as the
`floor-2026-10-06` profile of `gap_follow.launch.py` (`FLOOR_2026_10_06_PROFILE`), so the
working state can always be brought back:

```sh
ros2 launch racer_control gap_follow.launch.py profile:=floor-2026-10-06
```

This is a checkpoint, not a tuned optimum: one evening of hand tuning on one lane at 0.5 to
0.9 m/s. To record a better set later, add a new dated profile rather than editing this one.
Any launch argument given explicitly still wins over the profile (for example
`max_speed_mps:=0.7`), and the profile wins over `params_file`. `profile:=none`, the default,
passes nothing extra and keeps the node defaults (and `max_speed_mps` 2.0) exactly as before.

| Parameter | Profile | Node default |
|---|---|---|
| `min_speed_mps` | 0.5 | 0.5 |
| `max_speed_mps` | 0.9 | 2.0 |
| `k_steer` | 0.4 | 0.5 |
| `free_space_threshold_m` | 0.6 | 1.5 |
| `steering_gain` | 1.6 | 1.0 |
| `steering_time_constant_s` | 0.15 | 0.1 |
| `disparity_threshold_m` | 0.3 | 0.5 |
| `cone_half_angle_rad` | 1.2 | 1.57 |
| `forward_preference` | 0.0 | 0.0 |
| `gap_switch_margin` | 0.0 | 0.0 |
| `swept_path_clamp` | true | true |
| `swept_path_lookahead_m` | 0.6 | 1.0 |
| `corner_sector_inner_rad` | 0.4 | pi/2 |
| `corner_sector_outer_rad` | 1.6 | 3 pi/4 |
| `corner_min_clearance_m` | 0.35 | 0.2 |
| `speed_rate_limit_margin_fraction` | 0.1 | 0.5 |
| `target_deepest_ray` | false | false |
| `speed_time_constant_s` (new, see below) | 0.5 | 0.0 (off) |
| `target_range_median_scans` (new, see below) | 5 | 1 (off) |

Everything not listed (`k_speed_per_s` 1.0, `safety_margin_m`, `clip_max_range_m`, the control
rate, the watchdog) stays at the node default.

### Speed smoothing

The one complaint left after the laps: the speed surged and slowed all the time. Speed is
`clamp(k_speed * range along the target bearing, min, max)` times the steering slowdown,
recomputed for every scan at 10 Hz, and the target range flickers between scans. With
`speed_rate_limit_margin_fraction` 0.1 the rate limiter already holds acceleration to about
0.95 m/s^2, but it never limits deceleration, so every short target range dropped the speed at
once and the limiter then ramped it back up. Two new parameters, both off by default, both in
the profile, both `gap_follow.launch.py` arguments:

- `target_range_median_scans` (integer, 1 to 15, default 1 = off): the speed law uses the
  median of the last N scans' target ranges instead of this scan's. With 5 a target range has
  to persist for 3 of the last 5 scans (0.3 s) before it changes the speed, so a one or two scan
  flicker never reaches it. Updated once per scan, not per control cycle.
- `speed_time_constant_s` (0 to 5 s, default 0 = off): a first-order low-pass (the same
  `FirstOrderLowPass` as the steering) on the speed command, applied BEFORE the
  `SpeedRateLimiter`, so the limiter still bounds the acceleration of what is published. 0.5 s
  spreads a step over about a second in both directions.

Both reset with the rate limiter when the `/scan` watchdog trips. With both off the speed
chain is bit-identical to before (pinned by a gtest against the unfiltered chain), and the L5
canary, which runs on the node defaults, still passes: 26.035 s for two laps, worst distance
from the raceline 0.163 m.

The smoothing has not been on the floor yet; the laps above were driven without it.

### Known limits

- Turning circle about 0.8 m (full lock, 0.4189 rad on the 0.3302 m wheelbase, is a 0.742 m
  radius at the rear axle centre, more at the body), while the corners of the lane need about
  1 m of radius. There is little margin in the corners, which is why the corner and swept-path
  settings above matter, and a tighter corner than this lane's will not go round.
- No reverse. A car that ends up nose-in to a wall waits for safety_node's latch and cannot back
  out. The next step per the roadmap (2.3b) is a rear corridor check in safety_node, then a
  reverse escape behaviour in gap_follow_node (latched for a few seconds, back straight out a
  short distance, resume).
- One lane, one evening, one speed band. Nothing here says the profile works elsewhere.

## L5 canary

`tests/l5_reactive_lap` runs `bridge_node` and `gap_follow_node` (which sees only `/scan`) on
`config/tracks/gym_oval` and asserts two laps inside a time band without leaving the track.
The bridge's raceline tracks had no walls (`Track.from_refline` builds a map that is free
everywhere), so `bridge_node` gained a `track_half_width_m` parameter. The default 0 keeps the
old map for every existing caller; a positive value builds a walled corridor around the
raceline. The canary uses 0.8 m (a 1.6 m wide track) and `max_speed_mps` 3.0. Measured locally
in the ros-dev image: about 26.0 s for two laps on three runs, with the car never more than
0.163 m from the raceline (the limit before the chassis touches a wall is 0.645 m). The
committed band is 20 s (physical floor at the speed cap) to 60 s. Same test-only shim as the
tracker canary: `/drive_raw` is remapped to `/drive`, no safety_node in the loop.

## Not done here

- The roadmap note: `claude-docs/01-roadmap.md` is gitignored, so it is not updated in this PR.
- No hardware run. The usual L6 wheels-off-ground sweep applies before either node drives the
  car, and the tuning defaults are sim values.
- The canary only covers gap_follow_node. wall_follow_node has L1 and L3 coverage but no lap
  test.
