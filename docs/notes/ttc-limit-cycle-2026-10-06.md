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
  0.54 to 0.95 m from the turn centre, and a quarter turn is about 0.89 m of arc ahead of the
  head.
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

Finding, NOT changed here (forward behaviour was to be kept): the forward arc band is centred on
the REAR-AXLE path, half width 0.205 m. At full lock (rear-axle radius 0.742 m) its outer edge
is 0.947 m from the turn centre, but the body's outer front corner (0.46 m ahead of the rear
axle by the gym convention, 0.155 m out) sweeps `hypot(0.46, 0.742 + 0.155)` = 1.008 m, about
6 cm outside the band. In the sim, on a corner of 0.25 m centreline radius in a 1.1 m lane, the
car turned in at full lock and scraped the outside wall with that corner before safety_node
braked (f1tenth_gym's collision handler then zeroed its heading). Widening the band's OUTER edge
to the swept outer front corner (`hypot(front_x, |R| + half_width) + margin`) would close it;
that changes when the forward gate brakes in every hard turn, so it is the owner's call and a
separate change. The escape canary uses a square corner, met head-on inside the band, for that
reason.

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
