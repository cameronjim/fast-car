# TTC brake / rate limiter limit cycle (2026-10-06)

Found on the car on the evening of 2026-10-06, fixed in racer_safety the same night. Nothing
in this fix has run on the car yet: the bench check at the end is still to do.

## What was seen

Bag `2026-10-06T18-51-39_car_teleop` (on the Jetson, not copied off yet). Wheels off the
ground, `gap_follow_node` feeding `/drive_raw` at a steady requested speed of about 0.48 m/s,
an obstacle about 0.22 m ahead of the LiDAR, `safety_node` at 50 Hz, vehicle_params 0.6.1
(`limits.ttc_brake_s` 0.5 s, `ttc_warning_s` 1.0 s, `actuation.max_acceleration_mps2` 9.51).

- The gated `/drive` speed cycled 0.000, 0.190, 0.380, 0.000, 0.190, 0.380, ... at 50 Hz: a
  3-cycle limit cycle.
- 780 brake/release flips over 286 s, 34 in the worst second.
- `/safety/events` alternated "time-to-collision 0.459s <= brake threshold 0.500s; braking"
  and "command rate-limited relative to previous output (dt=0.02s)".
- The motor pulsed at about 17 Hz (50 Hz / 3).

## Mechanism

`SafetyGateLogic::evaluate` (ros_ws/src/racer_safety/src/gate_logic.cpp, as of commit
eb77c3f) ran bounds clamp, then rate limit, then TTC, and the TTC check used the
rate-limited OUTPUT speed:

- gate_logic.cpp 196-211: step 3b rate-limits the command against the PREVIOUS output and
  replaces it (`cmd = rate_limited;`, line 211). `rate_limit` (lines 130-139) only allows a
  speed increase of `max_acceleration_mps2 * dt` = 9.51 * 0.02 = 0.19 m/s per cycle.
- gate_logic.cpp 222: `forward_speed_mps = cmd.speed_mps`, i.e. the rate-limited output.
- gate_logic.cpp 230-232: `ttc_s = min_scan_range_m / forward_speed_mps`; at or below the
  brake threshold it sets `cmd.speed_mps = 0.0`.
- safety_node.cpp 336: `previous_output_ = result.output`, so the braked 0 becomes the base
  the limiter ramps from on the next cycle.
- gate_logic.hpp 52-57 documented exactly that order.

So, with a 0.48 m/s request and 0.22 m range:

| cycle | limiter output | TTC on that output | gate |
|---|---|---|---|
| n | 0.19 (0 + 0.19) | 1.16 s | passes |
| n+1 | 0.38 (0.19 + 0.19) | 0.58 s | passes |
| n+2 | 0.48 (0.38 + 0.10, capped at the request) | 0.46 s | brakes to 0 |
| n+3 | 0.19 again | ... | ... |

The third step is the request itself (0.38 + 0.19 would overshoot it), which is why the event
says 0.459 s (0.22 / 0.48) rather than the 0.39 s a 0.57 m/s step would give. The gate was
judging the command it had itself shrunk, not the command it had been asked to pass, so a
request that violated TTC got through two cycles in three.

Two things made it worse:

- `safety_node` took the minimum valid range over the whole 360 degree scan
  (`compute_min_scan_range_m`, safety_node.cpp 83-101), so anything beside or behind the car
  counted as straight ahead.
- 0.5 s of TTC at 0.48 m/s is 0.24 m. At crawl speed that is about the distance from the
  LiDAR head to the bumper, and a layer-3 "brake" is a coast (gate_logic.hpp, GateResult).

## Fix (racer_safety, vehicle_params 0.7.0)

1. **TTC on the request.** The obstacle gate now runs on the bounds-clamped REQUEST, before
   the rate limiter; the limiter then runs on whatever the gate leaves, so the output is
   still rate-limited. For a forward command the output never exceeds the request, so
   request-TTC is never larger than output-TTC: this is the conservative side.
2. **Latch with hysteresis.** Once the gate brakes, forward speed stays at zero until the
   request's TTC exceeds `ttc_warning_s` (or, with no warning threshold, `ttc_brake_s *
   SafetyLimits::ttc_release_hysteresis_factor`, 2.0). The release produces one
   `PHASE_RELEASE` record whose detail starts "ttc brake released". Garbage range never
   releases. Zero and reverse requests still pass while latched.
3. **Distance floor.** New `limits.min_forward_clearance_m` (PROVISIONAL 0.30 m, measured
   from the LiDAR head): a forward request at any speed with the forward minimum range below
   it brakes, on the same latch, released above 1.5 times the floor
   (`SafetyLimits::clearance_release_factor`).
4. **Forward sector.** New `limits.ttc_forward_sector_half_angle_rad` (PROVISIONAL 1.0 rad).
   `safety_node` takes the minimum only over returns whose vehicle bearing (laser bearing +
   `sensors.lidar.mount_yaw_rad`, pi on this car) is within that half angle, ignoring
   non-finite, zero, negative, below-`range_min` and above-`range_max` returns
   (`forward_sector.hpp`). `laser_yaw_from_vehicle_params` (default true) is false only in the
   sim launch files and the sim/synthetic-scan tests, whose `/scan` has yaw 0.
5. **Thresholds raised.** `limits.ttc_brake_s` 0.5 -> 1.0 s, `ttc_warning_s` 1.0 -> 2.0 s,
   both still PROVISIONAL.

The L1 suite reproduces the bag case exactly (0.48 m/s request, 0.22 m, 50 Hz, 286 s): the
output is 0 on every cycle, one TTC engagement, no rate_limit fight, and release happens only
once the request's TTC is above the warning threshold. Branch coverage of gate_logic.cpp and
forward_sector.cpp stays at 100 percent.

## Still to do on the car

Not run on the car (nothing runs there until the owner is back). Before `lidar:=true` is used
for a drive: wheels off the ground, the same gap_follow setup and an obstacle at about 0.2 m,
check that `/drive` stays at 0 with ONE `ttc` brake engage record, then move the obstacle away
and check one "ttc brake released" record and a smooth ramp. Also check that nothing on the car
itself is inside the forward sector within the floor distance.

## Steering hold while parked on the latch (2026-10-06, late)

Third wheels-off pass with gap_follow_node, vehicle_params 0.7.2. The latch held the car at
zero throttle as intended, but the steering servo kept hunting back and forth: the gate keeps
steering live while latched, and gap_follow_node's steering request wandered while it looked
for a gap it could not take.

Change (racer_safety, vehicle_params 0.8.0, not yet run on the car):

- Steering stays live while the latch is fresh, because a car braking at speed must keep
  steering authority. Once the latch has held the gated output speed at exactly zero for
  `limits.obstacle_steering_hold_after_s` (new, PROVISIONAL 0.5 s), the gate freezes the
  steering output at the angle it had when the hold started, until the latch releases. A
  reverse request (allowed while latched) moves the car, so it gets its steering back and the
  timer restarts.
- The timer uses the gate's own dt, threaded through `GateInput` / `GateResult` like the latch.
  Watchdog, command-sanity and internal-fault short-circuits hold it as it is.
- `/safety/events`: one `ttc` INFO engage, "steering held while obstacle-latched" with the held
  angle, and one release, "steering hold released", normally on the same cycle as "ttc brake
  released".
- Zero speed is a coast and there is no `/odom`, so 0.5 s stands in for "the car has stopped".
  It needs to be longer than the coast to rest from the speeds the car is driven at.

Same commit, release tuning: `limits.ttc_warning_s` (the release line) 0.6 -> 0.45 s and
`limits.ttc_forward_sector_half_angle_rad` 1.0 -> 0.6 rad. A person standing beside the front
corner sat inside the 1.0 rad sector and held the latch, and releasing at 0.6 s with a
1.0 m/s request needed 0.6 m of clear road (0.45 m at 0.45 s). `ttc_brake_s` 0.35 s and
`min_forward_clearance_m` 0.20 m are unchanged.

Bench check still to do: park the car on an obstacle with gap_follow_node running, check that
the servo stops moving about 0.5 s after `/drive` goes to zero and that it steers again when
the obstacle is moved away; stand beside the front corner and check the latch no longer holds.

## Corridor, not wedge (2026-10-06, floor test)

First floor run, on a tight track of backpacks about 1 m wide, gap_follow_node at 0.8 to
1.0 m/s, vehicle_params 0.8.0. The obstacle gate took the nearest return anywhere in the
+/- 0.6 rad wedge ahead of the car, so a bag 0.3 m to the side of the car's path, which the
car would pass cleanly, tripped the TTC brake (0.35 s) and the clearance floor (0.20 m)
exactly like a bag straight ahead. The owner found the safety node far too aggressive on that
track.

Change (racer_safety, vehicle_params 0.9.0, not yet run on the car):

- A return at vehicle bearing theta and range r becomes x = r cos(theta) ahead and
  y = r sin(theta) left (same mount yaw conversion as before). It counts as in the path only
  if x > 0 and |y| <= `chassis.width_m` / 2 + `limits.obstacle_corridor_margin_m` (new required
  field, PROVISIONAL 0.05 m, so 0.155 + 0.05 = 0.205 m). The corridor is the primary filter.
- The distance handed to the gate, for TTC and for the floor and its release clearance, is x,
  the along-track distance, not r.
- `limits.ttc_forward_sector_half_angle_rad` (0.6 rad) stays as an outer bound only: nothing
  outside it is ever considered, whatever the margin.
- Invalid returns are ignored as before, and garbage scan geometry or a garbage corridor
  width still falls back to the whole-scan minimum slant range. The latch, hysteresis,
  steering hold and release line are unchanged.

Two things to know. The corridor is straight along +x; it does not bend with the steering,
so in a corner it is a short-horizon approximation of the swept path. And with these values
the 0.6 rad sector, not the corridor, is the limit closer than 0.205 / tan(0.6), about 0.30 m
from the head: a return right at a front corner that close (for example x 0.20 m, y 0.15 m)
is outside the sector and is not seen. Widening the sector would close that gap without
bringing back the wedge problem, because the corridor now does the filtering; that is a
separate tuning decision.

Floor check still to do: the same backpack track at the same speeds, check that bags beside
the path no longer produce `ttc` engage records and that a bag placed in the path still
latches the car.

## Arc corridor (2026-10-06, late floor test)

Second floor run on the same backpack track, about 1 m wide, gap_follow_node at 0.8 to
1.0 m/s, vehicle_params 0.9.1. Bag `2026-10-06T22-12-40_car_teleop` (on the Jetson).

What the bag shows:

- The obstacle gate braked correctly three times, each at TTC 0.33 s with the wall about
  0.33 m ahead.
- Each latch then lasted 20 to 37 s.
- The car was at full steering lock for 85 percent of the run.

Mechanism: the corridor was straight ahead in the vehicle frame and ignored the requested
steering. A car stopped against a wall and asking for full lock away from it still had the
wall in its straight corridor, so the release test never passed. Once the latch had held the
speed at zero for 0.5 s the steering hold froze the output steering too, so nothing changed
until the planner happened to ask for something else.

Change (racer_safety, vehicle_params 0.9.2, not yet run on the car):

- The corridor follows the arc the requested steering sweeps. The request's steering angle,
  clamped to `steering.max_angle_rad`, gives a rear-axle turn radius R = L / tan(delta) with
  L = `chassis.wheelbase_m`. Below 1e-3 rad the old straight corridor is used unchanged.
- A return is moved from the LiDAR head to the rear-axle frame with
  `sensors.lidar.mount_x_m` / `mount_y_m`. It is in the path if its distance from the turn
  centre is within `chassis.width_m` / 2 + `limits.obstacle_corridor_margin_m` (0.205 m) of |R|,
  and its arc angle from the rear axle is between 0 and pi/2 (nothing behind the car, nothing
  past a quarter turn). The outer sector bound still applies first.
- The distance for TTC, the floor and the release clearance is the arc length from the LiDAR
  head (the same reference as the straight corridor's x and the clearance floor, not the
  bumper), never the straight-line range. At full lock R is about 0.74 m, the swept band runs
  0.54 to 0.95 m from the turn centre (to 1.05 m since schema 0.11.0, see "Outer boundary"
  below), and a quarter turn is about 0.89 m of arc ahead of the head.
- Which steering: the REQUEST's, not the gate's output. The output lags the request through
  the steering rate limiter, and while parked it is frozen by the steering hold; judging the
  path on the output would keep a latched car latched for exactly the reason seen in the bag.
- Where it is computed: `safety_node` used to reduce each scan once, in the scan callback. It
  now keeps the last scan and reduces it on every gate cycle with that cycle's requested
  steering, then hands the distance to the gate as before. The gate logic is unchanged: latch,
  hysteresis, floor and steering hold are as they were. `test_arc_corridor.cpp` composes the
  two exactly that way, and the L3 launch test checks it through the node.
- `limits.ttc_warning_s` (the release line) 0.45 -> 0.36 s, just above the 0.35 s brake; the
  owner wants the car released as soon as its path is clear. At 1.0 m/s that is 0.36 m of
  clear arc (and the floor's own release, 1.5 x 0.20 = 0.30 m, binds below about 0.83 m/s).

The L1 suite reproduces the bag: latched on a wall 0.33 m ahead at 1.0 m/s, parked past the
steering hold, then the request goes to full lock away with a scan whose arc is clear. The
latch releases on the first such cycle with one "ttc brake released" record and one "steering
hold released" record, and the output ramps out of the latch rate-limited. A straight request,
or full lock into the wall, stays latched for the bag's full 37 s. A return further round the
arc at 0.355 m (TTC 0.355 s, between brake and release) holds the latch; at 0.40 m it releases.

Two things to know. The arc assumes the car follows the requested steering at once; in the
first tenth of a second after a release the wheels are still slewing toward it (3.2 rad/s
rate limit, about 0.13 s centre to lock) while the speed ramps from zero, so the car is slow
while it catches up. And the simulator's scan comes from the vehicle reference point, while
the node places the head 0.285 m ahead of the rear axle from the binding; the sim tests pass,
but sim arc distances are off by that offset.

Floor check still to do: same track, same speeds. Park the car on a wall with gap_follow_node
running and check that it releases and drives off as soon as the planner steers away from the
wall, with one "ttc brake released" record; check that a bag on the inside of a turn still
latches it.

### Outer boundary: the outer front corner's sweep (2026-10-06 night, vehicle_params 0.11.0)

Sim finding (the L5 escape scenario on a ROUND tight corner, 0.25 m centreline radius in a
1.1 m lane): the follower turned in at full lock and the car's outer FRONT CORNER scraped the
outside wall before safety_node braked. The band above is centred on the rear-axle path, so its
outer edge was |R| + 0.205 m, 0.947 m from the turn centre at full lock. But the body point
furthest from the turn centre is the outer front corner, at (front x, -/+ half width) in the
rear-axle frame, and it sweeps the circle of radius `hypot(front_x, |R| + half_width)`: about
1.008 m without the margin. A round wall between 0.947 and 1.008 m was never in the corridor.

Change (racer_safety and racer_control, vehicle_params 0.11.0, not yet run on the car):

- **The band's outer edge is the outer front corner's sweep**, with the corridor margin on the
  half width as everywhere else: `|R| - 0.205 <= rho <= hypot(front_x, |R| + 0.205)`, 0.54 to
  1.053 m at full lock (forward_sector.hpp "OUTER BOUNDARY"). The inner edge (the inside flank)
  is unchanged. The straight corridor is unchanged: going straight the corner moves along the
  side line, already inside the half width. The distance for a return in the new outer strip
  is still the arc length to the head's arc angle, like every arc return.
- **front_x = `chassis.wheelbase_m` + `chassis.front_overhang_m`**, the rear axle to the front
  bumper line. `front_overhang_m` is a NEW REQUIRED field, PROVISIONAL 0.13 m, NOT measured
  (schema 0.10.0 -> 0.11.0). 0.13 m keeps front_x (0.4602 m) within 1.3 mm of the old
  `cg_to_rear_axle_m + length_m / 2` (0.46145 m) that racer_control's swept-path clamp used for
  the same point, which assumed the bounding box is centred on the CG. The clamp and the
  follower's forward arc probe now take front_x from the new field too, so safety_node and the
  follower agree on where the nose is.
- **The rear corridor mirrors it**: backing up, the rear bumper line leads, so the rear band's
  outer edge is the outer REAR corner's sweep, `hypot(rear_overhang_m, |R| + 0.205)`, 0.954 m
  at full lock (it was 0.947 m).
- **Consequence for the bag geometry above.** The L1 reproduction of bag 22-12-40 had the wall
  0.33 m ahead of the head from 2 cm right of the centreline outwards, and the car released by
  steering full lock left. With the corner's sweep the wall's near end (2 to 5.7 cm right of the
  centreline, inside the no-margin 1.008 m circle) is in the full-lock-left path at about
  0.19 m: steering away, the right front corner would have clipped it. That latch now holds
  (`TheBagsOwnWallHoldsTheLatchAtFullLockAwayBecauseTheCornerWouldClipIt`) and the car has to
  back out first (gap_follow_node's reverse escape). The release test now uses the same wall
  with its near end 15 cm right of the centreline, clear of the sweep. On the floor this means
  more latches in tight spots that steering alone used to clear, each one a case where the
  nose would have touched.

Tests: `test_arc_corridor.cpp` "ArcCorridorOuterCorner" (a return inside the corner's sweep but
outside the old band is in the full-lock path, left and right; the outer edge is exactly the
sweep with the margin; the inner edge and the straight corridor are unchanged for any front x;
a ray-cast round wall at the corner's no-margin sweep is in the path, and was not with the old
band), the garbage fallback for a non-finite or negative front x, and in
`test_rear_corridor.cpp` the mirrored rear case on `rear_overhang_m` (the front x plays no part
behind the car). Branch coverage of gate_logic.cpp and forward_sector.cpp stays at 100 percent
(325 of 325). L5: `test_gap_follow_escape_canary_round.py` drives the round corner with the
floor profile (`escape_probe_distance_m` 0.5 m, see the reactive-control note) and safety_node
in the loop; the brake now fires before contact, one escape backs the car out, and the lap
completes without wall contact. The square-corner canary still runs.

### Outer-corner horizon (2026-10-07, vehicle_params 0.12.0)

Floor finding (bags `2026-10-07T05-43-31` and later, with the 0.11.1 lane pass: floor 0.30 m,
brake 0.45 s): on a 1.1 m lane the car still braked mid-corner. At full lock the brake band
spanned radii 0.54 to 1.05 m from the turn centre, the whole lane, and the outer front corner's
sweep was projected up to a quarter turn ahead (about 0.89 m of arc from the head). The
follower straightens the steering within a few tenths of a second, so the outer wall two car
lengths round the arc is never reached, yet it tripped the brake. The owner reports the car
could have stepped on the gas and cleared these corners; the escape then fired for nothing.

Change (racer_safety, vehicle_params 0.12.0, not yet run on the car):

- **Two bands, two horizons** (forward_sector.hpp "OUTER-CORNER HORIZON"). The BODY band,
  `|R| - 0.205 <= rho <= |R| + 0.205` (the rear-axle path swept by the half width plus margin,
  0.54 to 0.947 m at full lock), is checked to the quarter turn as before. The OUTER-CORNER
  band, `|R| + 0.205 < rho <= hypot(front_x, |R| + 0.205)` (0.947 to 1.053 m at full lock), is
  checked only while the return's arc distance is at most `limits.outer_corner_horizon_m`.
  Beyond it the return is ignored.
- **`limits.outer_corner_horizon_m`** is a NEW REQUIRED field, PROVISIONAL 0.45 m (about one
  body length), NOT tuned (schema 0.11.1 -> 0.12.0). The horizon is measured in the same arc
  distance the gate uses for TTC and the floor (from the head forward, from the rear bumper line
  backward), and that distance is unchanged.
- **The rear corridor mirrors it**: the band between `|R| + 0.205` and the outer rear corner's
  sweep (0.947 to 0.954 m at full lock) counts only within the same horizon of reverse arc.
- **Unchanged**: the straight corridor, the inner edge, the outer edge itself, the sector bound,
  the invalid-return policy, and everything in gate_logic (latches, hysteresis, steering hold,
  release line). A NaN, zero or negative horizon is garbage and falls back to the whole-scan
  minimum like every other bad path input; +infinity is accepted and gives the 0.11.x band.

What it does to the lane case: a wall across the lane 0.98 m ahead of the rear axle (about
0.7 m ahead of the head) is outside the body band's sweep at full lock and inside the corner's
only near a quarter turn, about 0.62 m of arc from the head. It was in the brake band; it is
not now. The same wall 0.85 m ahead of the rear axle is in the body band's sweep and still
counts (`TheLanesOuterWallFarRoundTheArcNoLongerBrakes`). The 2026-10-06 round-corner wall is
still seen, its nearest return well inside the horizon
(`TheRoundCornerWallIsStillSeenWithinTheHorizon`).

Tests: `test_arc_corridor.cpp` "ArcCorridorOuterCornerHorizon" (an outer-band return inside the
horizon counts at its arc length and beyond it is ignored out to the quarter turn; body-band
returns beyond the horizon still count, outer side, centre and inner flank; left and right are
mirror images; an infinite horizon is the 0.11.x band; the straight corridor is the same for any
horizon; the lane wall and the round-corner wall above), the garbage fallback for a NaN, zero,
negative or -infinite horizon, and in `test_rear_corridor.cpp` the mirrored rear split (inside
and beyond the horizon, the body band beyond it, left/right, the straight reverse corridor, the
garbage fallback). The existing rear outer-edge test moved from 0.9 rad of arc (0.55 m from the
bumper line, now beyond the horizon) to 0.6 rad (0.33 m). Branch coverage of gate_logic.cpp and
forward_sector.cpp stays at 100 percent (333 of 333). L5: the round-corner escape canary passes
with no wall contact at 0.45 m, and both lap directions pass; numbers in the build log.

The L3 release test (`test_latch_releases_when_the_request_steers_away_and_the_arc_is_clear`)
was failing on PR 102 before this change: with the 0.30 m floor its wall moved in to 0.345 m,
and its near end, 2 cm right of the centreline, came inside the outer front corner's sweep
(arc distance about 0.24 m, inside the horizon too), so the latch held. Its near end is now
15 cm right of the centreline, clear of the sweep, like the gtest's `clear_wall()`.

Floor check still to do: the same 1.1 m lane at the 0.11.1 speeds, with the kill switch in a
second person's hand. Count TTC and clearance brakes per lap against bag 05-43-31 (36 brakes in
160 s); the mid-corner brakes with the outer wall 0.3 to 0.4 m away should be gone, and a car
that turns in late toward the outer wall must still brake before the nose touches it.

## Rear corridor (2026-10-06, night floor test)

Bags `2026-10-06T23-19-41` and `23-25-38`, replayed through the follower core: in both lap
directions the car ended nose-in to a corner tighter than its turning circle (full lock
0.4189 rad on the 0.3302 m wheelbase). The arc corridor braked correctly; then no steering angle
gave a clear forward arc (an object 0.2 m ahead and 0.16 m off the front corner), so the latch
held for ever. The follower never reverses, and safety_node had no rear check at all: a reverse
request passed whatever was behind the car. gap_follow_node now has a reverse escape
(reactive-control-port-2026-10-05.md, "Reverse escape"), so a reverse request is now judged like
a forward one.

Change (racer_safety, vehicle_params 0.10.0, not yet run on the car):

- **Geometry** (`min_rear_path_distance_m`, forward_sector.hpp "REAR CORRIDOR"). The arc corridor
  mirrored behind the car: a return moves to the rear-axle frame and is mirrored front to back
  (`xm = -(x + mount_x)`, `ym = y + mount_y`). Backing up with steering delta, the rear axle
  rides the same circle of radius `L / tan(delta)` about `(0, R)` as driving forward, the other
  way round, which in the mirrored frame is the forward case exactly; so the same band (half the
  chassis width plus `limits.obstacle_corridor_margin_m`), the same arc-angle window (0, pi/2)
  and the same straight corridor below 1e-3 rad apply. The outer bound is the same half angle
  (`limits.ttc_forward_sector_half_angle_rad`, 1.2 rad) centred on the car's -x axis.
- **Reference: the rear bumper line**, `chassis.rear_overhang_m` behind the rear axle. That is a
  NEW REQUIRED field, PROVISIONAL 0.12 m, NOT measured (schema 0.10.0; `cg_to_rear_axle_m` runs
  from the CG to the axle, forward of it, and `length_m` is only a bounding box). The forward
  corridor is measured from the LiDAR head, about 0.15 m behind the front bumper; measuring the
  rear from the head would put 0.4 m of car inside the distance. Returns inside the car's own
  footprint behind the head have a distance of zero or less and never count.
- **The same gate, mirrored** (gate_logic.hpp "THE REAR OBSTACLE GATE"). TTC on |requested
  speed| for a reverse request (infinite for a forward or zero one), the same `ttc_brake_s`
  (0.6 s), release line `ttc_warning_s` (0.75 s), `min_forward_clearance_m` floor (0.40 m, here
  measured from the rear bumper) with its 1.5 x release clearance, trip-wins rule and
  garbage-range handling. While latched, REVERSE speed is held at zero; forward and zero requests
  pass. The forward gate is the same code with the direction flipped, so every forward property
  and detail string is unchanged.
- **Two latches, not one keyed by direction.** `ttc_brake_latched` (forward) and
  `reverse_brake_latched` (rear) trip, hold and release independently on their own corridors, and
  each zeroes only motion toward its own side: the forward latch never blocks a reverse request
  (backing out is what a car parked nose-in needs) and the rear latch never blocks a forward one.
  One latch keyed by direction would have to drop or re-key the forward latch the moment a
  reverse request arrives and judge the forward release afresh when the request turns forward,
  losing the hysteresis that stops flicker; and a car boxed in front and back needs both held at
  once. They are separate engagements on `/safety/events`: the rear gate is a new source,
  `ttc_reverse` (`GateSource::kTtcReverse`), so a car latched both ways is two interventions and
  neither release can hide behind the other still being engaged. The steering hold stays tied to
  the forward latch only.
- **Rate limiter in both directions.** The speed rate limit compared signed speeds: any increase
  was limited, any decrease passed. Backwards that limited BRAKING (-0.5 -> 0 is an increase:
  three 50 Hz cycles to stop from -0.5 m/s) and passed any reverse acceleration. It now works on
  the magnitude: a change toward zero passes at once in either direction, growth away from zero
  in either direction is limited to `max_acceleration_mps2 * dt`, and a request across zero is a
  free brake to zero plus limited growth from zero. Forward behaviour is bit-identical. One
  existing gtest pinned the old reverse behaviour (a latched zero request while rolling backwards
  at -1.0 m/s stayed negative for a cycle, so the steering-hold timer did not start); it now pins
  the new one (zero at once, timer running) and says why in its comment.
- **safety_node** reduces the last scan a second time each gate cycle, along the reverse arc of
  the cached request's steering, and threads the rear latch like the forward one (held across an
  internal fault, reported as a `ttc_reverse` engagement).

Tests: `test_rear_corridor.cpp` mirrors the forward suites (reversing into a wall brakes,
reversing with nothing behind is free and rate-limited, the arc mirrored for left and right
lock, mount yaw pi and 0 agree, the rear sector bound, garbage falls back to the whole-scan
minimum, the car's own footprint ignored) plus the two-latch properties, release hysteresis with
one release record, a reverse brake from speed reaching the output on the same cycle, boxed in
both ways as two engagements, short-circuits holding the rear latch, and the rate limit in both
directions. The L3 launch test drives a reverse request at a wall 0.25 m behind the bumper
(braked, one `ttc_reverse` engage, no forward `ttc` engage), a forward request past the rear
latch, and the release. Branch coverage of gate_logic.cpp and forward_sector.cpp stays at 100
percent (319 of 319 branches; the rear code lives in forward_sector.cpp, so no new file needed
adding to the gate).

Finding, not changed in this step (forward behaviour was to be kept): the forward arc band was
centred on the REAR-AXLE path, half width 0.205 m. At full lock (rear-axle radius 0.742 m) its
outer edge was 0.947 m from the turn centre, but the body's outer front corner (0.46 m ahead of
the rear axle, 0.155 m out) sweeps `hypot(0.46, 0.742 + 0.155)` = 1.008 m, about 6 cm outside
the band. In the sim, on a corner of 0.25 m centreline radius in a 1.1 m lane, the car turned
in at full lock and scraped the outside wall with that corner before safety_node braked
(f1tenth_gym's collision handler then zeroed its heading). FIXED in vehicle_params 0.11.0: the
band's outer edge is now that corner's sweep (Arc corridor, "Outer boundary" above), and the
escape canary runs a round corner alongside the square one.

Sim limits: racer_gym_bridge's scan covers 4.7 rad, so in the sim the rear corridor sees only
the two rear-quarter wedges (bearings 1.94 to 2.35 rad off ahead), not straight behind. The L5
escape canary exercises the rear latch only if something sits in those wedges; the L1 and L3
tests are what pin the rear corridor.

Floor check still to do: wheels off the ground first, then on the floor with the kill switch in
a second person's hand. Back the car toward a wall with keyboard teleop: `/drive` must go to
zero about 0.4 m from the rear bumper with one `ttc_reverse` engage record, and forward must
still work. Then park it nose-in to a corner it cannot make with gap_follow_node running the
floor profile and watch one escape go through safety_node. Measure `chassis.rear_overhang_m`
on the car before relying on the rear floor.
