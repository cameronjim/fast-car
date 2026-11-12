# Safety gate and race profile at 1.0 to 1.55 m/s, 2026-11-12 (vehicle_params 0.15.0; nothing here has run on the car)

Owner report from the oval (lanes 1.3 to 1.5 m): `gap_follow` with `race-2026-11-12` at
`max_speed_mps 2.08` drove 189 s, about 148 m (25 laps), mean 0.98 m/s, max 1.55 m/s, "weird
swerving and missed turns", and `safety_node` "fired 66 TTC brakes". Bag
`2026-11-12T03-04-56_car_teleop`, the race run is 1734.4 to 1923.9 s into it. Everything below is
measured on that bag unless it says otherwise.

## What was found, in six lines

1. The 66 are not 66 brakes. They are 18 hard brakes (the output was zeroed), 38 TTC advisories
   ("advisory only, no command change") and 10 steering-hold notes ("steering held while
   obstacle-latched", which share source `ttc` and severity info). One real brake every 10 s, the
   same corner every lap.
2. Of the 18 brakes, 11 are the car arriving at the end of the oval at full lock with the outer
   wall 3 to 5 cm from the swept body (5 would have been contact, 6 a pass under 10 cm). They are
   necessary. 7 are false alarms: 5 on the straight approach to the end wall, 2 elsewhere.
3. The stopping model (0.20 s, 1.0 m/s^2) is about right on a straight (the slowest stop measured,
   0.8 to 1.0 m/s^2) and 1.9 to 2.9 times too long at full lock (true deceleration 2.0 to 2.3). The
   latency is 0.26 s, not 0.20.
4. `safety_node` judged speeds the car was not doing (a request of 1.5 m/s with the wheels at 0.87).
   With a fresh `/odom/wheel` it now judges the measured speed, raised toward the request by what the
   car can gain before a brake bites. Two of the seven false alarms go; every necessary brake is
   kept at the same instant (18 brakes to 16 on the replay). No threshold changed.
5. The other five false alarms have a different cause, in the follower, not the gate: its stopping
   law measured the free path along the arc it was about to steer, the target arc, while the car
   was still on the old one. At those five brakes the target steering was already -0.33 to -0.45
   and the published steering -0.04 to -0.2, the free path 2.0 m, the speed 1.5 to 1.9 m/s, and the
   wall 0.7 m ahead on the arc the car was on. `free_path_on_published_steering` measures the
   shorter of the two arcs.
6. The retune result is in the last section.

**Merge order warning.** This is schema 0.15.0 (a new required field, one value changed). Every
bundle that pins 0.14.0 refuses to load on it, by design (CLAUDE.md invariant 3): the policy
contracts under `training/policies/` and the plug-and-play model manifests under
`training/pnp_models/`. Their fixtures in the tests are updated here; the trained bundles are not
(re-exporting a bundle against a new version is the owner's call, and the PnP work is on other
branches). Nothing about their dynamics changed.

## 1. The classification, and how it was done

**Method, stated before it was run.** The method, the thresholds and the three plants are in the
module docstring of `tools/safety_replay/brake_audit.py`, committed as 77bb4fb with the message
"not yet run on the 2026-11-12 bag". Summary: every `/safety/events` ENGAGE record with source `ttc`
is an engagement (severity brake, or severity info with a time-to-collision detail: an advisory). For
each one the scene is the last scan that had arrived (the scan the gate judged), static for the next
second. The unbraked plan, the follower's own `/drive_raw` steering and speed, is run through the
measured plant (a delay, a first-order wheel-speed lag, an acceleration limit, a steering servo
lag, and the slip tables below) from the true state at the scan, and the smallest distance from the
body rectangle (wheelbase + front overhang, rear overhang, width) to any return over the scan's age
plus one second is the counterfactual clearance. A brake is NECESSARY below 0.04 m (the pass line
the floor reviews already use), MARGINAL from 0.04 to 0.10 m (the unbraked car would have missed by
a hand's breadth; not counted as a false alarm), FALSE ALARM from 0.10 m. A class is flagged when it
changes under either bracketing plant: "hold" (the car keeps its speed for the whole second) and
"agile" (half the delay and lag, 15 per cent less true speed). Corner means a requested steering
above 0.2 rad.

The slip table comes from scan matching (ICP of consecutive scans, 1492 usable pairs of 2601, the car's own parts
removed) against the wheel odometry and the steering command:

| steering below (rad) | pairs | true speed / wheel speed | true curvature / (tan(delta) / L) |
|---|---|---|---|
| 0.1 | 132 | 0.89 | 0.62 (too few turning pairs to mean anything) |
| 0.2 | 176 | 0.88 | 0.91 |
| 0.3 | 227 | 0.84 | 0.90 |
| 0.4 | 306 | 0.75 | 0.95 |
| 0.5 | 651 | 0.64 | 1.04 |

The wheel speed reads 1.12 times the truth on a straight and 1.56 times at full lock (the rear
wheels slip). The speed plant, fitted to `/drive` against the wheel speed over the whole run: delay
0.04 s, time constant 0.30 s, acceleration limit at the grid edge (1.0 m/s^2: not identified, the
car is current limited), command gain 0.90. `/odom` is no help here: its yaw rate is the wheel
speed and the modelled steering (`wheel_twist_node`, no gyro), so the measured lateral
acceleration below is from scan matching.

**Result, the 18 brakes** (t is seconds into the bag; "body distance" is the arc the rear axle can
travel along the requested steering before the body touches a return, "none" = nothing within
2 m on that arc; "real smallest clearance" is over the next second of real scans and includes the
follower's own reverse escape):

| # | t (s) | where | request m/s, rad | wheel m/s | true m/s | body distance on the requested arc (m) | unbraked plan, smallest clearance (m) | class | class changes in a bracket | real smallest clearance in the next 1 s (m) | held (s) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 1741.5 | corner | 0.36, -0.35 | 0.00 | 0.00 | 0.30 | 0.057 | marginal |  | 0.000 | 0.28 |
| 2 | 1751.6 | corner | 1.33, -0.27 | 0.87 | 0.74 | 0.88 | 0.281 | false alarm |  | 0.236 | 0.04 |
| 3 | 1752.1 | corner | 0.36, -0.44 | 0.76 | 0.48 | 0.47 | 0.089 | marginal | yes | 0.236 | 0.62 |
| 4 | 1753.2 | corner | 0.36, -0.45 | 0.24 | 0.15 | 0.27 | 0.029 | necessary | yes | 0.000 | 1.52 |
| 5 | 1755.8 | corner | 0.74, -0.22 | 0.10 | 0.08 | 0.60 | 0.218 | false alarm |  | 0.000 | 0.02 |
| 6 | 1783.3 | corner | 0.36, -0.45 | 0.53 | 0.33 | none | 0.044 | marginal |  | 0.076 | 2.20 |
| 7 | 1793.2 | corner | 0.36, -0.44 | 0.84 | 0.53 | none | 0.027 | necessary |  | 0.000 | 1.98 |
| 8 | 1803.3 | corner | 0.36, -0.44 | 0.75 | 0.48 | none | 0.029 | necessary |  | 0.086 | 2.10 |
| 9 | 1813.2 | corner | 0.36, -0.44 | 0.84 | 0.53 | none | 0.040 | necessary | yes | 0.121 | 1.88 |
| 10 | 1823.1 | corner | 0.36, -0.44 | 0.82 | 0.52 | none | 0.036 | necessary |  | 0.098 | 2.00 |
| 11 | 1839.8 | corner | 0.36, -0.44 | 0.80 | 0.51 | none | 0.046 | marginal |  | 0.000 | 2.10 |
| 12 | 1849.9 | corner | 0.36, -0.45 | 0.77 | 0.49 | none | 0.040 | marginal | yes | 0.137 | 1.54 |
| 13 | 1860.5 | corner | 0.36, -0.45 | 0.44 | 0.28 | 1.84 | 0.047 | marginal |  | 0.051 | 2.40 |
| 14 | 1874.1 | straight | 1.16, -0.00 | 0.91 | 0.81 | 0.71 | 0.160 | false alarm |  | 0.205 | 0.16 |
| 15 | 1883.9 | straight | 1.31, -0.06 | 0.83 | 0.74 | 0.73 | 0.201 | false alarm |  | 0.230 | 0.12 |
| 16 | 1892.1 | straight | 1.16, -0.04 | 0.83 | 0.74 | 0.70 | 0.206 | false alarm |  | 0.130 | 0.02 |
| 17 | 1909.8 | straight | 1.11, -0.01 | 0.84 | 0.75 | 0.65 | 0.170 | false alarm |  | 0.202 | 0.16 |
| 18 | 1920.1 | straight | 1.30, -0.05 | 0.91 | 0.81 | 0.80 | 0.251 | false alarm |  | 0.176 | 0.02 |

| | all | corner | straight |
|---|---|---|---|
| necessary (under 0.04 m unbraked) | 5 | 5 | 0 |
| marginal (0.04 to 0.10 m) | 6 | 6 | 0 |
| false alarm (0.10 m or more) | 7 | 2 | 5 |

Four classes change under a bracketing plant (3, 4, 9, 12), all between necessary and marginal at
0.03 to 0.09 m: the model cannot separate those, and the table should be read as "11 brakes at a
wall within about 10 cm, 7 not". The 38 advisories changed nothing; in the next second the car
really came within 4 cm of something after 15 of them (the corner stops and the reverse escape),
17 of them were followed by a brake within a second.

What the two groups are:

- **The 11 corner brakes** are the lap's one failure. The car comes round the end of the oval at
  full lock (radius 0.69 m), the follower has already dropped its request to the floor (0.36 m/s)
  because the free path along that arc is 0 to 0.08 m, the wheels still read 0.5 to 0.84 m/s (true
  0.3 to 0.53), and the outer wall is 3 to 5 cm from the swept body: the gate holds the throttle
  at zero for about 2 s (the latch), the follower reverses (the escape), and drives on. That is the
  "missed turn", every 10 s. The gate is right; the car arrives at that corner too wide and too
  late for a 0.69 m turning radius in a 1.3 to 1.5 m lane.
- **The 5 straight brakes** (14 to 18) are the approach to the same end wall at a request of 1.1 to
  1.3 m/s with the wheels at 0.83 to 0.91, the steering near zero and the wall 0.65 to 0.8 m of body
  distance ahead. Held for 0.02 to 0.16 s each. The unbraked plan turns in time and clears by 0.16
  to 0.25 m. What the gate cannot know is that the follower is about to steer; what the follower
  could have known is that the car was still on the straight arc (section 5).

## 2. How the car really stopped

Measured with `tools/audit_brakes.py` from the wheel speed (`/odom/wheel`, 50 Hz) and from scan
matching for the true travel. There is no stop in this bag from a TRUE speed above 0.93 m/s on a
straight or 0.79 m/s at full lock: the follower's cruise is 1.0 to 1.3 m/s on the wheels, which is
0.6 to 1.1 m/s true, and the fastest true speed anywhere in the run is 1.35 m/s (p95 1.15). The
1.0 to 1.55 m/s in the task is the wheel speed. What follows is therefore a measurement up to a
true 1.1 m/s and an extrapolation above it, and the extrapolation is a model, not a measurement.

**Latency.** From the scan's stamp to the first sign of deceleration:
- the scan is 0.1005 s old when the gate sees it (log time minus stamp, 2901 scans; p90 0.102, max
  0.15);
- the gate cycle adds up to 0.02 s;
- the wheel speed starts to fall 0.10 to 0.14 s after the command drops (17 hard slow-downs from at
  least 0.9 m/s wheel speed: 14 within 0.10 to 0.14 s, median 0.12 s; one at 0.18 s; two at 0.46 and
  0.50 s where the car was already slowing).
0.10 + 0.02 + 0.14 = 0.26 s with the slowest clean response, against `actuation.stop_latency_s` 0.20.

**Deceleration.** After the command goes to zero the wheel speed falls at 3.0 to 3.6 m/s^2 on
average (peak 5 to 6) at full lock, but the wheel speed is not the car's: the wheels lock and slip.
Corrected by the slip table, the true mean deceleration of the nine clean stops to rest (a command
drop, the car at rest within 0.6 s):

| t (s) | wheel v0 | steering | true v0 | latency (s) | true mean decel (m/s^2) | scan travel (m) |
|---|---|---|---|---|---|---|
| 1783.0 | 1.10 | -0.43 | 0.70 | 0.10 | 2.0 | 0.149 |
| 1793.0 | 1.24 | -0.42 | 0.79 | 0.10 | 2.2 | 0.259 |
| 1803.1 | 1.12 | -0.43 | 0.71 | 0.10 | 2.3 | 0.158 |
| 1813.0 | 1.21 | -0.42 | 0.77 | 0.11 | 1.9 | 0.189 |
| 1822.9 | 1.21 | -0.43 | 0.77 | 0.10 | 2.1 | 0.182 |
| 1839.6 | 1.19 | -0.43 | 0.75 | 0.10 | 2.3 | 0.198 |
| 1870.7 | 0.96 | -0.45 | 0.61 | 0.12 | 1.05 | 0.172 |
| 1884.8 | 1.05 | -0.45 | 0.67 | 0.12 | 1.75 | 0.213 |
| 1924.0 | 0.88 | -0.06 | 0.79 | 0.18 | 0.83 | 0.471 |

(The steering is the command at the drop; the last row is the watchdog stop at the end of the run, a
straight-line stop to rest from 0.79 m/s true in 0.47 m.) Full lock scrubs the front tyres and a
straight stop does not: 2.0 to 2.3 against 0.8 to 1.05. So the model has to cover the slowest.

**Against what `safety_node` assumes** (0.20 s and 1.0 m/s^2, travel = v t + v^2 / 2a): on the
straight stop it is within 10 per cent (model 0.52 m at 0.79 m/s with 0.26 s, measured 0.47 m). At full
lock, with the new latency and the true speed, it overstates the travel 1.9 to 2.9 times (model 0.35
to 0.51 m for a measured 0.15 to 0.26 m).
The model is conservative, correctly, but its latency is 0.06 s short.

**What changed in `vehicle_params`:** `stop_latency_s` 0.20 to 0.26; `stop_deceleration_mps2` kept at
1.0 (the slowest measured, with the range written in the yaml); new `peak_acceleration_mps2` 2.2: the
most the wheel speed gained over 0.2 s windows is 2.71 m/s^2 (one wheelspin burst), 99.9 per cent of
windows are below 2.13 on the race run and 1.78 over the whole 1990 s bag, 99 per cent below 1.67.

## 3. The `safety_node` change

**What it does.** While `/odom/wheel` is fresh, the TTC divides by

    v = max(v_measured, min(v_request, v_measured + peak_acceleration * stop_latency))

instead of the request raised to the decaying output bound and the measurement
(`gate_logic.hpp`, "JUDGED SPEED FROM THE MEASURED WHEEL SPEED"). The bound is not used, because it
exists only to stand in for a measurement. Without a fresh measurement (the simulator before
today, every bag before 2026-11, a stale `/odom/wheel`) or with the model unset or unusable, the gate
is exactly what it was. Thresholds, floor, latch and release are untouched, and the judged speed is
never above the old one, so the change can only remove a trip: a property test over a grid of
requests, wheel speeds, bounds and ranges checks it.

**Why not a stopping-distance law.** The first version replaced the 0.60 s window by the stopping
distance `v t + v^2 / 2a + margin` with the numbers above. On the race bag it also dropped the 2
marginal brakes (1741.5, 1752.1 s) and moved one necessary brake 0.2 s later, and on the two
2026-11-10 bags that have odometry it tripped later than the recorded brake on 13 of 19 and 13 of 26
approaches and left 3 and 17 predictions under 4 cm (the committed rule: 0 and 11). A physical law with
the measured numbers is shorter than 0.60 s at every speed, so it cannot keep the old-bag
behaviour, and a gate whose margin is tuned on those bags should not lose it. It was dropped.

**Proof on the race bag** (`safety_replay_cli`, the real gate, on the recorded `/scan`,
`/drive_raw` and `/odom/wheel`; the rule as driven, the same code with `measured_speed_judging=false`,
reproduces all 18 recorded brakes within 0.15 s):

| | brakes | necessary kept | marginal kept | false alarms kept | later than the recorded brake | new brakes |
|---|---|---|---|---|---|---|
| before (as driven, 0.60 s on request raised to bound) | 18 | 5 of 5 | 6 of 6 | 7 of 7 | 0 | 0 |
| after (measured speed) | 16 | 5 of 5, same instants | 6 of 6, same instants | 5 of 7 | 0 | 0 |

The two removed are brake 2 (1751.6 s: the output bound was 1.5 m/s, the request 1.33, the wheels
0.87, body distance 0.89 m) and brake 5 (1755.8 s: the car at rest, wheels 0.10, a request of
0.74 m/s). Had the car followed the unbraked plan the smallest clearance would have been 0.281 m and
0.218 m. Minimum clearance over all 16 kept brakes' unbraked plans: 0.027 m (the same corner brakes,
which stay).

**Proof on the older bags** (`tools/replay_safety.py --recorded-with 0.13.0`, the four bags of the
2026-11-10 re-tune, the committed 0.14.0 rule against this one, same method as
`calibration-2026-11-10.md`):

| bag | recorded brakes | later than recorded (before, after) | predicted under 4 cm (before, after) | new brakes (before, after) |
|---|---|---|---|---|
| 2026-10-24 | 19 | 0, 0 | 0, 0 | 6, 6 |
| 2026-10-25 | 37 | 0, 0 | 1, 1 | 14, 14 |
| 2026-11-11T03-54-11 (cw) | 19 | 0, 0 | 0, 0 | 0, 0 |
| 2026-11-11T04-20-19 (v3) | 26 | 4, 4 | 11, 11 | 0, 0 |

Identical. The two October bags have no odometry (the gate is untouched there by construction); the
other two have it, and the judged speed there was already the request or the measurement. The bag
`2026-11-12T02-57-50` (418 s, same day, earlier) has no TTC brake at all, before or after.

**Tests.** `test_measured_speed.cpp` (12 cases: the judged speed, the cap, the raise to the
measurement, no bound, the two removed brakes of this bag, reverse, no measurement, every unusable
model, no threshold, latch and release, and the can-only-remove property), the existing 247 gtests
untouched and green, `racer_safety` branch coverage still 100 per cent (387 of 387) on
`gate_logic.cpp` and `forward_sector.cpp`. Sim: `l5_park_test`, `l5_reactive_lap_test` and
`e2e_sim_safety_test` run locally in the ros-dev image and pass; the rest of the L5 canaries run in CI.
The bridge publishes `/odom/wheel`, so the sim runs with the new judged speed.

## 4. The audit tool

`tools/audit_brakes.py BAG --window START END` (needs `safety_replay_cli` for the vehicle numbers):
engagements, the slip table, the plant, the table above, the stopping tables of section 2. The
bag reader takes a time window now (`read_bag(path, window_s=(a, b))`, seeking by the chunk index:
the 6 GB bag opens in a second; an unindexed file is read whole). Tests: `test_brake_audit.py` (26
cases on synthetic walls and known dynamics, including the bug it found in itself: a zero speed time
constant chattered between the limits), `test_replay_follower.py` (window), `test_replay_safety.py`
(`--window`).

## 5. The follower, and the retune

**What the follower saw at the five straight brakes.** The race profile (at `max_speed_mps 2.08`, as
driven) replayed on the bag's real scans, one scan before each brake (the replay is the node's own
code, `gap_follow_replay_cli --speed`):

| brake | gate saw (request m/s, steering) | follower, target steering | follower, published steering | free path it measured (m) | speed it allowed (m/s) |
|---|---|---|---|---|---|
| 1874.1 | 1.16, -0.00 | -0.36 | -0.13 | 2.04 | 1.95 |
| 1883.9 | 1.31, -0.06 | -0.41 | -0.20 | 2.04 | 1.55 |
| 1892.1 | 1.16, -0.04 | -0.42 | -0.21 | 2.04 | 1.54 |
| 1909.8 | 1.11, -0.01 | -0.45 | -0.20 | 2.04 | 1.58 |
| 1920.1 | 1.30, -0.05 | -0.33 | -0.16 | 2.04 | 1.75 |

The free path is the whole horizon (2.04 m) because it was measured along the TARGET arc, already
turning away from the wall; the car, with the steering low-pass (0.15 s) and the servo behind it,
was on a nearly straight arc with the wall 0.7 m of body distance ahead. (The replay's published
steering is its own, not the car's, which was nearer zero; the direction is the same.) The race
speed law says "stop inside the free path"; it was told the path was free.

**The change.** `GapFollowConfig::free_path_on_published_steering` (node parameter and launch
argument of the same name, default false, so every old profile reproduces exactly): the free path
the speed law uses is the shorter of the target arc and the arc of the steering the car is at (the
low-passed command as last published, threaded from the command half to the scan half under the
node's existing mutex). The replay CLI does the same with its own filter output. Tests: a gtest on
the core (flag off ignores the published steering, unknown or equal steering changes nothing, a
hard turn into a side wall shortens the path and not the steering), the node parameter table
comparison (`test_replay_follower.py`), launch tests for the new argument and the old profiles
carrying no race parameter.

**The harness and the rule, committed before the runs** (commits 365e7de and the one after it):
`tools/race_tuning/tracks.py` has a WIDE family (stadiums, lanes 1.3, 1.4, 1.5 m, straights 1.5, 2.0,
3.0 m, island radius 0 to 0.6 m; 8 tuning tracks, held-out seeds disjoint, tested), `selection_gate.py`
has the rule, `race_tune.py evaluate --family wide --rule gate` applies it, and
`tools/race_tuning/retune_2026_11_12.sh` is the exact variant list. The rule: a candidate is
admissible if, over its 16 tuning runs (8 tracks, both directions, 2 laps), every run completed, there
was no wall contact, the minimum body clearance was at least 0.04 m, and `safety_node` braked at most
0.25 times per lap and reversed at most 0.25 times per lap (absolute, not relative to a baseline: the
first rule let a faster car brake as often as the slow one and that is the thing the bag showed). The
best admissible candidate has the lowest lap time relative to the baseline on the same track and
direction; within 0.02 of each other, fewer brakes, then fewer escapes, then the lower speed. The
held-out tracks are then run three times, pooled, with the same clauses.

Candidates (each at 1.0, 1.3, 1.6 m/s, on `race-2026-11-12`): A the published-arc stopping law;
B A with `stopping_margin_m` 0.2 (was 0.1); C B with `lane_lookahead_m` 0.8 (was 0.6); D B with
`a_lat_max_mps2` 1.0 (was 1.5). References: the profile as shipped at 1.0 m/s, and at 1.6 and
2.08 m/s (the owner's run).

**Tuning result** (16 runs per variant, real time factor at least 0.99, no wall contact anywhere):

| variant | min clearance (m) | brakes per lap | lap time (s) | lap ratio vs the lane profile at 0.64 m/s | gate rule |
|---|---|---|---|---|---|
| baseline (lane profile, 0.64 m/s) | 0.200 | 0.00 | 20.95 | 1.000 | reference |
| ref10 (race-2026-11-12, 1.0 m/s) | 0.163 | 0.00 | 11.84 | 0.565 | yes |
| ref16 (the same at 1.6 m/s) | 0.165 | 1.28 [1.00, 1.56] | 10.74 | 0.520 | no, 1.28 brakes per lap |
| ref208 (the owner's 2.08 m/s) | 0.162 | 1.91 [1.56, 2.16] | 11.13 | 0.539 | no, 1.91 |
| A at 1.0 | 0.173 | 0.00 | 12.33 | 0.591 | yes |
| **A at 1.3** | 0.173 | 0.03 [0.00, 0.09] | 11.15 | **0.539** | **yes, the winner** |
| A at 1.6 | 0.169 | 1.06 [0.75, 1.38] | 11.38 | 0.551 | no, 1.06 |
| B at 1.0 / 1.3 / 1.6 | 0.214 / 0.213 / 0.211 | 0.00 / 0.00 / 0.03 | 14.81 / 13.92 / 13.78 | 0.719 / 0.681 / 0.675 | yes |
| C at 1.0 / 1.3 / 1.6 | 0.199 / 0.198 / 0.199 | 0.00 / 0.00 / 0.00 | 12.98 / 11.98 / 11.82 | 0.632 / 0.589 / 0.583 | yes |
| D at 1.0 / 1.3 / 1.6 | 0.215 / 0.211 / 0.213 | 0.00 / 0.00 / 0.16 | 14.95 / 14.44 / 14.35 | 0.726 / 0.705 / 0.701 | yes |

The simulator reproduces the bag's failure: the shipped profile at 1.6 and 2.08 m/s brakes 1.3 and
1.9 times per lap on these lanes (the car braked 0.7 times per lap on the floor), and at its own
1.0 m/s it does not. The published-arc law alone (A) brakes 0.03 times per lap at 1.3 m/s (the
shipped law was not run at 1.3) and 1.06 at 1.6 against the shipped law's 1.28, a sixth fewer. A
larger stopping margin (B) removes the rest at the price of 25 per cent of the
lap time; the longer lookahead (C) removes it without that price: 0 brakes in 16 runs at 1.6 m/s,
8 per cent slower than A at 1.3. The lateral limit (D) bought nothing and cost speed.

By the rule the winner is **A at 1.3 m/s: `race-2026-11-13`** (`gap_follow.launch.py`
`RACE_2026_11_13_PROFILE`: `race-2026-11-12` with `max_speed_mps 1.3` and
`free_path_on_published_steering true`; nothing else moved). C at 1.6 m/s is the way up to 1.6 with no
brakes at all; it was not chosen by the rule (it is slower than A at 1.3) and is run on the held-out
tracks as a disclosed extra, below.

**Held-out result** (8 wide tracks the search never saw, both directions, 3 laps, three repeats
pooled: 48 runs per row, real time factor at least 0.99, completed 48 of 48 and no wall contact in
every row):

| variant | min clearance (m) | brakes per lap (95% CI) | lap time (s) | lap ratio vs the lane profile at 0.64 m/s | gate rule |
|---|---|---|---|---|---|
| baseline (lane profile, 0.64 m/s) | 0.307 | 0.00 | 20.85 | 1.000 | reference |
| ref10 (race-2026-11-12, 1.0 m/s) | 0.188 | 0.00 | 11.62 | 0.557 | yes |
| ref16 (the same at 1.6 m/s) | 0.189 | 0.67 [0.38, 0.97] | 10.44 | 0.502 | no |
| **A at 1.3 = race-2026-11-13** | 0.207 | 0.05 [0.00, 0.12] | 11.10 | 0.534 | **yes** |
| C at 1.6 (disclosed extra) | 0.304 | 0.00 | 11.40 | 0.551 | yes |

The winner generalises: 0.05 brakes per lap against 0.67 for the shipped profile at 1.6, 4 per cent
faster than the shipped profile at its own 1.0 m/s. C at 1.6 m/s is 0.00 brakes in 48 runs and 3 per
cent slower than A at 1.3; it is the setting for the top of the range, not chosen by the rule. The
real floor braked 0.7 times per lap with the old law at 2.08 m/s; the sim says 1.9. The difference
between the sim's brake count and the floor's is not explained (the floor's are mostly the oval's
end corner, which the sim lanes do not reproduce, section 7).

**Lateral acceleration, measured.** From scan matching (true speed times true yaw rate, 1492 usable
pairs of the race run): median 0.53 m/s^2, 90th percentile 0.91, 99th 1.15, maximum 1.34, with no sign
of the limit in the bag (no slide, the curvature ratio 1.04 at full lock). `/odom` cannot give it (its
yaw rate is modelled). The speed law's `a_lat_max_mps2` is in the units of the command, the wheel
speed, which reads 1.12 to 1.56 times the truth: the profile's 1.5 means a TRUE 0.5 m/s^2 at full lock
(R 0.69 m) and about 0.95 at 0.2 rad. So the measured cornering says the law is well inside what the
car did, not at its limit. Raising it was not in the candidate list (the sim's tyre model is not the
floor's and nothing here measured the limit), lowering it (D) bought nothing, so it stays 1.5.

## 6. If l_max_erpm goes above 6000

Nothing in this change assumes the 2.08 m/s ceiling. `actuation.throttle_full_scale_mps` (2.08, from
`l_max_erpm` 6000) is untouched and is the request-to-pulse scale of `racer_drivers`; it must move with
`l_max_erpm` (`tools/tests/test_vehicle_params_calibration.py` fails until both agree). Every speed
tuned here is a true speed with its own explicit ceiling: the gate has none (it judges the measured
speed), `race-2026-11-13` has `max_speed_mps 1.3`, the 1.6 m/s variant is a launch argument, and the
`ref208` row above is only the owner's run reproduced as a reference. What WOULD change with a higher
ERPM cap: `peak_acceleration_mps2` and `stop_latency_s` were measured at 6000 ERPM and 30 A, and the
stopping travel was measured up to a true 1.1 m/s (0.93 on a straight); above that is a model.
Re-run `tools/audit_brakes.py` on the first bag at the new cap before trusting 1.6 m/s and up.

## 7. What was not done, and what to check on the car

- **The corner at the end of the oval is not fixed.** 11 of the 18 brakes are the car at full lock
  3 to 5 cm from the outer wall, one lap in one. The gate is right to hold there. The follower reaches
  that corner too wide and too late for a 0.69 m turning radius in a 1.3 to 1.5 m lane; the wide sim
  tracks (round ends, island radius 0 to 0.6 m) do not reproduce it (0 brakes at 1.0 m/s), so this
  retune could not tune it. The next thing is the real oval: map the bag with `tools/track_session`
  and run the family on that geometry.
- The classification rests on one scan and the follower's recorded request. Four brakes sit between
  necessary and marginal under a bracketing plant. A brake the audit calls a false alarm is one the car
  would have cleared by at least 10 cm IF the follower did what it recorded.
- No stop from a true speed above 0.93 m/s on a straight exists in the bag. The model above that is
  an extrapolation, the deceleration being the slowest measured.
- The sim's wheel speed is the true speed (no slip), its tyres are not the floor's; the sim result
  says the law now holds the brakes down at 1.3 m/s on wide lanes, not what the floor will do.
- The schema bump invalidates every bundle pinned to 0.14.0 (see the top). The trained bundles were
  not touched.
- `test_gap_follow_node_event_launch` failed once under a load average of 50 on this machine (80 Hz
  of `/drive_raw` against a limit of 36, and publishing after scan silence) and passed on the next
  run with the machine quiet; it is timing-based. Not investigated further.

Next floor session (two people, the RC kill in a hand, wheels off the ground first): `ros2 launch
racer_control gap_follow.launch.py profile:=race-2026-11-13 max_speed_mps:=0.64`, then 1.0, then 1.3.
For 1.6 m/s add `stopping_margin_m:=0.2 lane_lookahead_m:=0.8` (the C candidate, held-out result
above). After each run: `uv run --project tools python tools/audit_brakes.py <bag> --window START END
--cli <safety_replay_cli>`; the brake count, the classes and the stop table are the thing to compare.

## Reproduce

```sh
# the audit (about 70 s, needs a safety_replay_cli build for the vehicle numbers)
uv run --project tools python tools/audit_brakes.py /Users/cameronjim/code/car-data/bags/2026-11-12T03-04-56_car_teleop \
  --window 1733 1925 --cli <safety_replay_cli> --json audit.json
# the gate replay, before (as driven) and after, on the race run
uv run --project tools python tools/replay_safety.py --recorded-with current --cli <safety_replay_cli> \
  --before-rule 0.13.0 --before-set ttc_brake_s=0.6 --before-set ttc_warning_s=0.71 \
  --before-set min_forward_clearance_m=0.1 --before-set measured_speed_judging=false \
  --window 1733 1925 /Users/cameronjim/code/car-data/bags/2026-11-12T03-04-56_car_teleop
# the retune (docker, ros-dev image with vesc_msgs)
tools/race_tuning/retune_2026_11_12.sh tuning
tools/race_tuning/retune_2026_11_12.sh heldout A_v1.3
```
