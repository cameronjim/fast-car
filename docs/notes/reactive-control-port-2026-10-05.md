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
  corner sweeps hypot(body_front_x, R + c), with body_front_x = `chassis.wheelbase_m` +
  `chassis.front_overhang_m` (schema 0.11.0, PROVISIONAL 0.13 m; until then it was
  `cg_to_rear_axle_m` + `length_m / 2`, the bounding box taken as centred on the CG, which the
  provisional value matches within 1.3 mm).
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
| `centering_gain` (new, night, see "Lane centring") | 0.6 | 0.0 (off) |
| `centering_sector_half_angle_rad` (new, night) | 1.0 | 1.0 |
| `centering_max_range_m` (new, night) | 1.5 | 1.5 |
| `reverse_escape` (new, night, see "Reverse escape") | true | false |
| `escape_probe_distance_m` (new, night, see "Reverse escape") | 0.5 | 0.3 |

Everything not listed (`k_speed_per_s` 1.0, `safety_margin_m`, `clip_max_range_m`, the control
rate, the watchdog, the other `escape_*` timings) stays at the node default.

The laps were driven with every row down to `target_deepest_ray`. The rows marked "new" were
added afterwards, at the owner's request, and none of them has been on the floor yet. To drive
exactly the laps' parameters: `profile:=floor-2026-10-06 speed_time_constant_s:=0
target_range_median_scans:=1 centering_gain:=0 reverse_escape:=false`.

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
- No reverse (until the night of 2026-10-06). A car that ended up nose-in to a wall waited for
  safety_node's latch and could not back out. Now: a rear corridor in safety_node
  (docs/notes/ttc-limit-cycle-2026-10-06.md, "Rear corridor") and the reverse escape below.
- One lane, one evening, one speed band. Nothing here says the profile works elsewhere.

## Lane centring (2026-10-06 night floor finding)

Bags `2026-10-06T23-19-41` and `23-25-38`, replayed through the follower core: the follower is
EDGE-BIASED. It aims at the angular centre of the widest run of free rays; rays grazing the
wall the car is already near stay long and count as free, so the gap centre drags toward that
wall. The owner watched it hug the edges in both lap directions.

The fix adds a push away from the nearer side wall, measured from the scan, to the gap steering
before the steering clamp (`measure_lane_walls` and `centering_steering` in
`include/racer_control/gap_follow.hpp`; the result also carries the walls and the push):

- Returns from the RAW scan (invalid ones ignored), in the vehicle frame (laser yaw applied).
  Left side: vehicle bearing in (0, `centering_sector_half_angle_rad`]; right side: [-half
  angle, 0). Only returns within `centering_max_range_m` count.
- Each return's perpendicular distance from the centreline is `r sin(b) + mount_y` (sign per
  side), and a side's wall distance is the MEDIAN of those, not the nearest return's. On a
  straight wall every ray gives the same distance, so the median, the minimum and the nearest
  return agree. The median is the "cleaner estimate" because of the wall ACROSS the lane at
  the next corner: its rays just off the centreline have a perpendicular distance near zero,
  so the nearest return (or the smallest `r |sin b|`) reads a wall at about 0 m on one side and
  saturates the push away from the turn on the way into every corner. A gtest pins this: a
  centred car 0.8 m from a corner's outside wall gets no push from the median, while the
  smallest perpendicular distance on the left is under 2 cm.
- `centering = centering_gain * (d_left - d_right) / (d_left + d_right)`: positive (left) when
  the right wall is nearer. The task text wrote `(d_right - d_left)`, which with the
  left-positive steering convention steers TOWARD the nearer wall, the opposite of its own
  "positive = steer left when the right wall is nearer"; the code and the tests follow the
  stated intent (a car offset right steers left).
- An OPEN side (no return within the range) gives NO push at all: centring is relative to a
  lane with two walls. Substituting the maximum range for the open side would push the car
  toward a doorway or toward the room beyond a row of scattered objects, which is the first
  floor test's "into the open room" failure.
- The push is added to `steering_gain * bearing` before the clamp, so the sum saturates at
  `steering.max_angle_rad` (a hard gap turn is not undone by it), then the swept-path clamp and
  the corner override run as before.
- `centering_gain` 0 (the node default) skips the measurement and is bit-identical to the
  follower without centring (a gtest compares every output field on 200 seeded random scans
  against the pipeline rebuilt from its parts).

Floor profile value 0.6. In a straight lane of width W an offset e from the centre gives
`(d_left - d_right) / (d_left + d_right) = 2 e / W`, so the push is `2 * gain / W` rad per metre:
about 1.1 rad/m at the bags' 1.0 to 1.2 m lane (1.0 to 1.2 rad/m across that range). That is the
steering a pure-pursuit point about 0.8 m ahead would ask for (`2 L e / Ld^2` with L = 0.33 m),
about the turning radius and the profile's 0.6 m swept-path lookahead: strong enough to pull the
car off a wall within a car length or two, not so strong that a 5 cm offset (0.05 rad) fights
the gap. Not yet driven on the floor.

## Reverse escape (2026-10-06 night floor finding)

Same bags: in both lap directions the car ended nose-in to a corner tighter than its turning
circle (full lock 0.4189 rad on the 0.3302 m wheelbase, about 0.8 m across). safety_node's arc
corridor braked correctly; then no steering gave a clear forward arc (an object 0.2 m ahead and
0.16 m off the front corner) and the latch held for ever. The follower never reversed.

**With `reverse_escape` true (the floor profile) gap_follow_node can command the car to REVERSE
on its own.** The state machine is `ReverseEscape` in `include/racer_control/reverse_escape.hpp`
(ROS-free, gtest-covered); the node feeds it once per control cycle and logs every transition
at INFO:

- **Trigger.** safety_node has refused the node's forward request (its own last request > 0,
  the GATED `/drive` speed 0) for `escape_after_s` (1.5 s) AND the follower sees no way
  forward: its corner override fired, or no steering gives a clear forward arc for
  `escape_probe_distance_m` (node default 0.3 m, 0.5 m in the floor profile). The node subscribes `/drive` (read only) for this; it
  never publishes it. Without safety_node in the loop there is no gated `/drive` and the escape
  never fires.
- **Clear arc.** `any_forward_arc_clear` in `gap_follow.hpp`: nine steering angles evenly over
  +/- full lock; for each, the body (`wheelbase_m + front_overhang_m` ahead
  and `chassis.rear_overhang_m` behind the rear axle, inflated by `safety_margin_m` on every
  side) is moved along the arc in 2 cm steps of rear-axle travel and must not contain a return
  at any step. Returns already inside the body at the start are ignored (the car itself, or
  contact the probe cannot judge).
- **Escape.** `escape_speed_mps` (-0.5, ramped like the forward speed; the VESC does nothing
  below about 0.44 m/s, so do not go slower) with the steering at full lock OPPOSITE to the
  sign of the follower's wanted forward steering: backing up with the wheels turned the other
  way swings the nose toward where the follower wanted to go (yaw rate `v tan(delta) / L`,
  both signs flipped), the second leg of a three-point turn. A zero wanted steering backs
  straight out. It lasts until `escape_distance_m` (0.4 m) of travel or `escape_max_s` (2.0 s),
  then the node resumes forward with its speed ramp restarted from rest.
- **Distance, without odometry.** The integral of the COMMANDED escape speed. Assumption,
  stated: the car moves at the commanded speed. It does not quite (nothing happens below the
  VESC's 0.44 m/s, and a gate brake stops it early), so the real escape is SHORTER than
  `escape_distance_m`, never longer, and `escape_max_s` bounds it in time.
- **Rear blocked.** If safety_node refuses the reverse request too (its new rear corridor: the
  gated speed is not negative while the request is) for `escape_block_debounce_s` (0.1 s, the
  cycle or two `/drive` lags `/drive_raw`), or `/drive` goes stale (`drive_feedback_timeout_s`
  0.2 s), the escape is ABORTED and the next one waits `escape_retry_after_s` (3.0 s).
- **Attempts.** At most `escape_max_attempts` (3) escapes without forward progress (the gated
  forward speed integrated since the last escape reaching `escape_distance_m`), then the node
  HOLDS: no more escapes, the follower runs as before and safety_node's latch holds the car,
  until forward progress resets the count.

All parameters are declared with ranges; `reverse_escape` and `escape_probe_distance_m` are
also `gap_follow.launch.py` arguments. The node logs a WARN at startup when the escape is on.

Known limits:

- The probe judges "can the body drive 0.3 m" while safety_node releases its latch only once the
  REQUESTED arc is clear for 1.5 x `limits.min_forward_clearance_m` (0.60 m from the head,
  about 0.45 m from the bumper). In the narrow band where some arc gives 0.3 m but the
  follower's requested arc is not clear for the gate, the node waits instead of backing up,
  exactly as before this change. The floor profile therefore sets `escape_probe_distance_m`
  to 0.5 m (the sim escape scenario on a round corner waited instead of escaping at 0.3 m),
  which closes most of that band; the node default stays 0.3 m.
- No odometry, so no closed-loop distance (above). No rear check in the follower itself: the
  rear is safety_node's job.
- Sim only so far (L5 below). The floor check: park the car nose-in to a corner it cannot make,
  with the kill switch in a second person's hand, and watch for one "reverse escape 1/3" line,
  a short reverse with the wheels turned away from the turn, and "reverse escape complete".

## L5 canary

Since the night of 2026-10-06 the canary runs the corridor lap in BOTH directions, with the
floor profile's lane centring on (`test_gap_follow_lap_canary.py` counter-clockwise, the
raceline's own direction, and `test_gap_follow_lap_canary_cw.py` clockwise; shared harness in
`l5_reactive_common.py`). f1tenth_gym's reset puts the car on the raceline's first waypoint
facing along it, so `bridge_node` gained `reverse_direction` (default false), which reverses the
waypoint order before the track is built; the walls do not change. Measured locally in the
ros-dev image: 26.09 to 26.12 s counter-clockwise and 25.82 to 25.84 s clockwise for two laps,
worst distance from the raceline 0.152 to 0.154 m (0.163 m without centring). The harness now
also fails on wall contact: f1tenth_gym's collision handler sets `state[3:]` to zero, which
zeroes the car's yaw, so contact shows in ground truth as the heading snapping between two
100 Hz samples.

`test_gap_follow_escape_canary.py` is the second scenario: a 5 x 3.5 m rounded rectangle with a
1.1 m lane, three 1.2 m corners and one SQUARE corner first after the start (centreline radius
0.02 m, outside wall a 0.57 m arc), the full floor profile, and safety_node in the loop with no
remap (the escape needs the gated `/drive`). It passes when the escape fires at least once (a
reverse request on `/drive_raw` and a reverse command on the gated `/drive`) and the car still
finishes the lap inside 150 s without leaving the corridor or touching a wall. Measured locally:
25.94 to 26.05 s for the lap on three runs, one escape each, worst distance from the centreline
0.18 to 0.24 m (limit 0.395 m). The control run, the same scene with `reverse_escape:=false`,
reproduces the floor finding: braked nose-in at 0.40 m, latched, the follower asking for full
lock for the rest of the run.

Round corner (`test_gap_follow_escape_canary_round.py`, vehicle_params 0.11.0): the same loop
with the tight corner at a 0.25 m centreline radius. Before 0.11.0 the follower turned in early
and the car's outer FRONT CORNER scraped the outside wall before safety_node braked, because
the arc corridor ended at |R| + 0.205 m about the rear-axle path and the outer front corner
sweeps about 6 cm further out at full lock. safety_node's band now ends at the outer front
corner's sweep (ttc-limit-cycle note, Arc corridor, "Outer boundary"), and the floor profile's
`escape_probe_distance_m` is 0.5 m. Measured locally after both changes, two runs: round
corner 36.94 s and 28.12 s, one escape each, worst distance from the centreline 0.196 to
0.203 m, no wall contact; square corner on the same build 36.84 s and 36.93 s, two escapes
each, worst 0.263 to 0.267 m. Control run, the same build with only
safety_node's band put back to the old outer edge (probe still 0.5 m): the round corner fails
on wall contact at (5.32, 0.48), and the square corner is 26.03 s with one escape. So the
wider band is what lets the round corner pass, and it is also what costs the square corner a
second escape and about 11 s (it brakes earlier in the hard turn). The square corner stays as
the floor's head-on nose-in case.

The original canary text, still true: `tests/l5_reactive_lap` runs `bridge_node` and `gap_follow_node` (which sees only `/scan`) on
`config/tracks/gym_oval` and asserts two laps inside a time band without leaving the track.
The bridge's raceline tracks had no walls (`Track.from_refline` builds a map that is free
everywhere), so `bridge_node` gained a `track_half_width_m` parameter. The default 0 keeps the
old map for every existing caller; a positive value builds a walled corridor around the
raceline. The canary uses 0.8 m (a 1.6 m wide track) and `max_speed_mps` 3.0. Measured locally
in the ros-dev image: about 26.0 s for two laps on three runs, with the car never more than
0.163 m from the raceline (the limit before the chassis touches a wall is 0.645 m). The
committed band is 20 s (physical floor at the speed cap) to 60 s. Same test-only shim as the
tracker canary: `/drive_raw` is remapped to `/drive`, no safety_node in the loop (the lap
canaries; the escape canary runs safety_node).

## Not done here

- The roadmap note: `claude-docs/01-roadmap.md` is gitignored, so it is not updated in this PR.
- No hardware run. The usual L6 wheels-off-ground sweep applies before either node drives the
  car, and the tuning defaults are sim values.
- The canary only covers gap_follow_node. wall_follow_node has L1 and L3 coverage but no lap
  test.
