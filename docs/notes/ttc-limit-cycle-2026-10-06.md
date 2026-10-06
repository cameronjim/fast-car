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

Change (racer_safety, vehicle_params 0.7.3, not yet run on the car):

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
