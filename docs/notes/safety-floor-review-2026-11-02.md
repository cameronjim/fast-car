# Safety node floor review, 2026-11-02 (analysis only, nothing changed)

**Update 2026-11-10:** the speeds in this note (and the thresholds chosen from them) were 1.40
times the true speeds, and the steering angles were off by 7 to 20 per cent; see
`docs/notes/calibration-2026-11-10.md` for the corrected re-tune (TTC 0.60 s, floor 0.10 m).

Owner report: "safety node is causing car to run into things, it should be slightly more
forgiving". Checked against the two track bags from the last driving sessions.

## Bags

| Bag | Driver | Length | TTC/floor brakes | Reverse escapes | Brakes ending <= 2 cm off a wall |
|---|---|---|---|---|---|
| `2026-10-24T23-13-18_car_teleop` | gap_follow (centre), then wall_follow (right, then left) | 806 s | 19 (15 floor, 4 TTC) | 11 | 3 |
| `2026-10-25T19-27-08_car_teleop` | gap_follow, `lane_centre` target (before PR 111) | 409 s | 37 (31 TTC, 6 floor) | 34 | 11 |

On 2026-10-25 the car braked, sat latched for about 2.6 s, reversed 0.4 m, drove on and braked
again, every 5 to 7 s for 210 s. Only 25% of scans in that bag have the car moving forward.

Method: bags streamed with `mcap` + `mcap_ros2`; every `/safety/events` engage matched to
`/drive_raw` and `/drive`; the LiDAR scan transformed into the rear-axle frame (laser yaw pi,
mount x 0.285 m) with the provisional body box (front 0.46 m, rear 0.12 m, half width 0.155 m).
"End clearance" is the smallest gap between any return and that box in the 2.5 s after a brake.
"Swept-body distance" is the arc the rear axle travels until the body box (plus the 5 cm side
margin) reaches the return along the requested steering arc. Scripts:
`/private/tmp/claude-501/-Users-cameronjim-code-car/0cf95989-cb0c-4556-a90b-84707e5753a7/scratchpad/trackana/`.

## What happens at a brake (56 brakes)

- 50 of 56 brakes are at full LEFT lock (+0.37 to +0.419 rad) on a CCW lap, with the closest
  return at the RIGHT front corner: the outside of the turn is running into the outer wall.
- At the moment of the brake, in 51 of 56 cases NO steering angle within limits leaves 0.30 m of
  swept path. The car was already inside the corner it cannot make at its 0.74 m rear-axle
  turning radius. The brake was justified; it was not cutting off a turn the car could make.
- After the brake the car keeps going about `0.13 s x speed` (0.07 m at 0.59 m/s, 0.11 m at
  0.81 m/s, 90th percentile 0.16 m). This is mostly latency: one 10 Hz scan, the gate cycle, the
  50 Hz PWM frame and the VESC speed loop.
- The brake fires when the body is only 0.13 to 0.20 m of arc from the wall (median swept-body
  TTC 0.19 s on 2026-10-25). Minus 0.11 m of stopping travel, the nose ends 0 to 7 cm off the
  wall: median 4.2 cm on 2026-10-25, 14 of 56 at 2 cm or less (contact with the foam). Returns
  inside the assumed front of the body box while stopped also point at the nose touching.

Clearest cases (bag time in seconds from the first scan):

| Bag, t | Request | Swept-body dist at brake | End clearance | Mechanism |
|---|---|---|---|---|
| 10-25, 108.04 | 0.88 m/s, +0.414 | 0.168 m | 0.000 m (right side) | late TTC trip in a corner, slid into the outer wall |
| 10-25, 132.32 | 0.87 m/s, +0.372 | 0.147 m | 0.000 m (front) | same, nose contact |
| 10-25, 157.10 | 0.94 m/s, +0.407 | 0.183 m | 0.000 m | same, at the fastest approach |
| 10-24, 701.18 | wall_follow 1.0 m/s request after 1.97 m/s, +0.419 | 0.100 m | 0.000 m (left side) | TTC on the request while the car was still faster |
| 10-24, 89.63 to 225 | 0.68 m/s, steering 0.03 | floor latch | stuck 136 s | follower deadlock, see below |

## Why the margin is so thin

`apply_obstacle_gate` uses the arc distance from the LiDAR HEAD (`forward_sector.cpp`
`arc_corridor_distance_m`, reference point = lidar mount). The parts that hit are ahead of
the head: the front bumper is 0.175 m ahead (front_overhang_m 0.13, PROVISIONAL), and at full
lock the outer front corner leads the head by about 0.08 m of arc. So the same 0.3 s TTC means
very different real margins:

- At full lock, 0.3 s on the head distance is about 0.21 s to the body at 0.8 m/s.
- Straight ahead, 0.3 s on the head distance is about 0.08 s to the bumper at 0.8 m/s.
- The 0.25 m floor is 7.5 cm off the bumper straight ahead, about 17 cm of arc at full lock.

The thresholds were lowered on request (0.5 to 0.3 s, warning 1.0 to 0.45 s) to stop the car
braking early, which removed the margin the head offset was silently eating.

## Two follower and gate interactions found on the way

1. 2026-10-24, 89.6 to 225 s: a floor latch at nearly straight steering. gap_follow kept
   requesting 0.68 m/s at steering 0.03 for 136 s. Reverse escape did not fire because some
   steering had a clear 0.5 m arc, but the follower never steered to it. Deadlock until the car
   was moved.
2. 2026-10-25: 206 `/drive_raw` gaps of 63 to 69 ms in 326 s, each just over the 60 ms watchdog
   (3 cycles at 50 Hz). Every one is a one-cycle brake to zero and a rate-limited re-ramp: a
   stutter, mostly right after a brake release. Follower timer jitter, not scan loss (no scan gap
   over 0.11 s).

## Recommendation

"More forgiving" in the sense of braking later would make every one of these contacts harder:
the car could not make the corners at all from where it braked. The two real fixes:

1. Safety node: measure the obstacle distance to the swept BODY, not to the LiDAR head. The
   reference point becomes the body's leading edge at each return's radius (front bumper face
   in the body band, the outer front corner in the outer-corner band), which is what the
   swept-body distance above computes. Then set `ttc_brake_s` 0.20 s and a body floor of
   0.05 m (replacing the 0.25 m head floor), release at TTC 0.30 s / floor x 1.5. Replayed open
   loop on the recorded approaches, this rule trips 0.1 to 0.2 s earlier than the real brakes in
   corners and predicts an end clearance of at least 6 cm on all 51 replayable brakes (median
   9 cm), against 14 contacts as driven. It is no stricter than today in corners, and much safer
   straight ahead. Measure `front_overhang_m` and `rear_overhang_m` first: the rule leans on them.
2. Follower: the car reaches the corner too wide and too fast to turn. The lane_centre apex fix
   (PR 111, profile `lane-2026-10-25`, never driven) is the first thing to try. A corner speed
   cap from the lane trace curvature would also shrink the 0.13 s x v stopping travel.

Smaller fixes worth their own tasks: while refused, gap_follow should steer toward the clear
arc its escape check already finds (or escape after a fixed refusal time); and either fix the
follower timer jitter or raise the watchdog to 4 or 5 missed cycles (80 to 100 ms, 6 to 8 cm
at 0.8 m/s).

## How to validate before the car

1. Add a safety replay CLI next to `gap_follow_replay_cli`: feed the bag's `/scan` and
   `/drive_raw` through `SafetyGateLogic` and `min_path_distance_m`, print every trip with its
   swept-body distance.
2. On both bags, compare old and new rules: list trip times, check the new rule trips no later
   than the old one on every recorded brake, and compute predicted end clearance
   (swept-body distance at trip minus 0.13 s x requested speed). Pass: every predicted end
   clearance at least 0.04 m, and no new trips on stretches where the car drove freely.
3. Re-run the sim battery (`tools/reactive_bench.py`, `l5-reactive-lap`, `l5-park`) for
   regressions, then one floor session in each direction, copying the bag back for the same
   check with real end clearances.

## Outcome (2026-11-03, PR fix/safety-body-distance; nothing here has run on the car)

### What changed

- **racer_safety measures from the swept body.** Every in-path distance is now measured from the
  body's leading edge: the front bumper line straight ahead, and on an arc the body's leading
  point at the return's own radius (the inner flank toward the inside, the bumper line, out to the
  outer front corner). Reverse uses the rear bumper line the same way. Which returns count is
  unchanged. `forward_sector.hpp`, "DISTANCE FROM THE BODY". One thing the review did not say:
  toward the inside of a turn the inner flank trails the head's arc angle, so there the body
  distance is larger than the old head distance (the old rule read such returns closer than any
  part of the car can reach them). On the replayed bags the new rule never tripped later than the
  old one.
- **TTC speed.** The TTC still only judges a request toward the obstacle, but divides by the
  request raised to the larger of a decaying bound of the node's own recent outputs (falling at
  `actuation.stop_deceleration_mps2`) and the fresh `/odom/wheel` speed (optional, 0.1 s
  freshness; absent in sim and in these bags). The judged speed is never below the request, so
  the 2026-10-06 limit cycle cannot come back (`gate_logic.hpp`, "SPEED FOR THE TTC", and a
  286 s re-run of that bench case in `test_ttc_speed.cpp`). Without the bound the replay leaves
  the 10-24 701.18 wall-follow contact at 3.1 cm; with it, 6.3 cm or more.
- **vehicle_params 0.13.0:** `limits.ttc_brake_s` 0.34, `ttc_warning_s` 0.45 (the release line),
  `min_forward_clearance_m` 0.05 m, all on the body distance and PROVISIONAL; new
  `actuation.stop_latency_s` 0.20 s and `stop_deceleration_mps2` 1.0 m/s^2, DERIVED (below).
- **Deadlock.** The reverse escape now also fires when safety_node has refused the follower for
  `escape_refused_after_s` (5 s), whatever the follower's arc probe says. The 10-24 deadlock would
  have backed out at 94.6 s instead of sitting until 225 s.
- **Stutter.** Root cause found and fixed at the source, watchdog untouched (still 60 ms). Every
  one of the 206 `/drive_raw` gaps over 60 ms on 10-25 contained a scan arrival, the header stamps
  show the publisher itself was late (not transport), and the gap length tracks the follower's
  compute time on that very scan: replaying the 10-25 scans through the lane_centre code the car
  ran (commit eea7b8d, before PR 111) gives a correlation of 0.86 between per-scan compute and
  the gap. That code took 3 ms per scan unoptimised on a Mac, so tens of ms on the Jetson, inside
  the scan callback, on the single thread that also runs the 50 Hz timer. PR 111 already made the
  trace about 25 times cheaper; gap_follow_node now also runs perception and the timer in
  separate callback groups on a two-thread executor, so a slow scan cannot delay a command again.
  A launch test slows every scan callback to 50 ms and checks no gap reaches 60 ms (it fails with
  one thread). The runbook now builds with `-DCMAKE_BUILD_TYPE=Release`.

### Stopping travel, measured from the LiDAR

No odometry was recorded, so the car's motion around each brake came from 2D scan matching
between scans (`tools/safety_replay/motion.py`). Two things the review got wrong:

- The car was much slower than requested: about 0.55 of the request in corners (0.45 m/s real at
  0.8 m/s requested; the yaw rate at full lock agrees), 0.75 on straights. The 701.18 car was the
  other way, 1.15 m/s real on a 1.0 m/s request.
- The stopping travel is longer than `0.13 s x speed`: from the scan the gate judged to rest,
  real travel / real speed is 0.30 s median, 0.405 s at the 90th percentile (41 brakes that
  stopped clear, 0.3 to 0.8 m/s real), linear in speed over that range. Speed holds for about
  0.1 s after the trip, then falls at 1.0 to 1.5 m/s^2 (2.8 m/s^2 in the one brake from 1 m/s).
  Against the request it is 0.21 to 0.25 s (90th percentile).

The review's suggested rule (TTC 0.20 s, floor 0.05 m, release 0.30 s, on the body distance)
therefore trips later than the October brakes on 22 of 53 approaches and leaves 40 of them under
4 cm. It was not used.

### Replay, before and after

`tools/replay_safety.py` runs both bags through `safety_replay_cli`, the real gate, with the
October rule ("before") and the committed one ("after"), and for each recorded brake predicts the
end clearance under the new rule as the measured end clearance plus how much earlier on the same
approach it trips (scan-matched car travel between the two trip scans), with contacts corrected
by the 90th-percentile travel. The before-rule replay reproduces 55 of the 56 recorded brakes
within 0.15 s.

| | 2026-10-24 | 2026-10-25 |
|---|---|---|
| Recorded forward obstacle brakes | 19 | 37 |
| Before-rule replay reproduces (within 0.15 s) | 18 | 37 |
| As driven: ended on the foam (2 cm or less) | 3 | 11 |
| As driven: median end clearance | 5.9 cm | 4.2 cm |
| After: approaches predicted | 16 | 37 |
| After: trips later than the recorded brake | 0 | 0 |
| After: median lead over the recorded brake | 0.11 s | 0.39 s |
| After: predicted end clearance, median / min | 8.0 / 6.4 cm | 19.3 / 6.3 cm |
| After: predicted under 4 cm | 0 | 0 |
| After: new brakes where the car drove on | 3 (wall follow, 1.19 to 1.42 m/s) | 6 (1.00 to 1.17 m/s) |

Not predicted on 10-24: two brakes at a standstill (236.6 and 240.0 s, a latch re-tripping on a
wall beside the front corner while steering away; the body rule says that wall is never reached)
and 626.8 s, which the before replay does not reproduce either.

Why 0.34 s (same analysis, both bags, floor 0.05 m, release 0.45 s):

| ttc_brake_s | later than recorded | predicted under 4 cm | min predicted | new brakes where the car drove on |
|---|---|---|---|---|
| 0.20 (the review) | 22 | 40 | -10.4 cm | 0 |
| 0.30 | 5 | 5 | 2.2 cm | 7 |
| 0.33 | 0 | 1 | 3.1 cm | 9 |
| **0.34** | **0** | **0** | **6.3 cm** | **9** |
| 0.35 | 0 | 0 | 6.3 cm | 11 (adds 0.6 m/s full-lock corners the car made) |

So the owner's "more forgiving" holds where it was asked for: no new brake in any corner driven
at the October corner speeds (0.9 m/s request or less), the floor is 0.05 m off the bumper
instead of 0.25 m from the head, and the latch can release by steering away from a wall beside
the front corner. It is stricter on the approach to a wall the car cannot turn away from, which
is where it hit things. The 9 new brakes are all at 1.0 to 1.4 m/s requests on the entry to a
turn with the bumper 0.27 to 0.43 m from the wall; at those speeds the measured stopping travel
is 0.2 to 0.3 m, so the follower turned with little to spare. If they feel like cut-offs on the
floor, the fix is a lower corner speed, not a lower TTC.

### Sim, before and after

Same image (`ros-dev:lane`), same scripts. The lap canaries and e2e are CI's; the bench is
`tools/reactive_bench.py` (before with 3 runs side by side, after with 2; a first after-run that
overlapped another agent's simulator lost real-time factor and was discarded and re-run).

| Test | Before | After |
|---|---|---|
| l5_reactive_lap, CCW 2 laps | 26.05 s, pass | 26.06 s, pass |
| l5_reactive_lap, CW 2 laps | 25.99 s, pass | 25.99 s, pass |
| escape canary, square corner | 26.11 s, 1 escape, pass | 26.04 s, 1 escape, pass |
| escape canary, round corner | 26.19 s, 1 escape, pass | 26.49 s, 1 escape, pass |
| e2e_sim_safety (3 tests) | pass | pass |

Bench, `floor-2026-10-06` (laps, lap time, obstacle brakes, min body clearance, contacts):

| Track | Before | After |
|---|---|---|
| lane_square | 3, 39.8 s, 15, 0.023 m, 0 | 3, 41.2 s, 15, 0.044 m, 0 |
| lane_round | 2, 40.8 s, 12, 0.017 m, 0 | 2, 41.0 s, 14, 0.012 m, 0 |
| lane_open_side | 3, 24.9 s, 0, 0.176 m, 0 | 3, 25.0 s, 0, 0.170 m, 0 |
| chicane | 3, 25.0 s, 0, 0.216 m, 0 | 3, 24.7 s, 0, 0.218 m, 0 |
| open_room | -, 27 brakes, -0.003 m, 2 | -, 22 brakes, -0.012 m, 2 |

Bench, `lane-2026-10-25`:

| Track | Before | After |
|---|---|---|
| lane_square | 3, 39.7 s, 15, 0.022 m, 0 | 3, 39.6 s, 16, 0.056 m, 0 |
| lane_round | 3, 39.6 s, 12, 0.019 m, 0 | 3, 39.8 s, 13, 0.046 m, 0 |
| lane_open_side | 3, 24.9 s, 0, 0.080 m, 0 | 3, 25.0 s, 0, 0.084 m, 0 |
| chicane | 3, 25.8 s, 0, 0.115 m, 0 | 3, 25.7 s, 0, 0.118 m, 0 |
| open_room | -, 80 brakes, -0.010 m, 2 | -, 68 brakes, -0.007 m, 2 |

No follower parameter was changed. The sim's square corners still need an escape per corner (the
car cannot make them at its turning circle, as on the floor); minimum clearance there roughly
doubled with the lane profile. The open_room contacts are unchanged from before (the room's
boxes, a known bench property). The sim has no real stopping latency, so it checks the stack,
not the stopping margin; the replay above is the evidence for that.

### Left for the owner

- **Measure `chassis.front_overhang_m` and `rear_overhang_m` before the floor test.** Both are
  still placeholders (0.13, 0.12) and the brake point now moves one for one with them. If the
  front differs by more than 1 cm, re-run `tools/replay_safety.py --set front_overhang_m=...`.
- Drive with `vesc:=true` so `/odom/wheel` is in the gate and in the bag; the stopping model can
  then be re-derived from real speeds instead of scan matching.
- The runbook section "Safety floor test, body-distance gate" has the order of checks.
- The follower still reaches the corners too fast to turn (the review's second recommendation,
  a corner speed cap from the lane curvature); this PR does not change follower tuning.
