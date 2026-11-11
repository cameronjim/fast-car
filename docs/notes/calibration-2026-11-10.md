# Calibration from real measurements, 2026-11-10 (vehicle_params 0.14.0; nothing here has run on the car)

The first time the physical constants of `config/vehicle_params.yaml` were checked against the
car. Three of them were wrong in ways that mattered: the speed scale (the software believed
speeds 1.40 times the truth), the steering map (symmetric and linear, from the gym's `s_max`) and
the two overhangs (placeholders). The safety node's thresholds had been tuned on the October
bags through those errors, so they are re-tuned here on the same bags in corrected units.

## What changed

| Field | Old | New | Provenance |
|---|---|---|---|
| `chassis.front_overhang_m` | 0.13 | 0.11 | MEASURED, tape, owner: 10 to 11 cm, upper end taken (a larger front overhang brakes earlier) |
| `chassis.rear_overhang_m` | 0.12 | 0.14 | MEASURED, tape, owner |
| `tires.nominal_radius_m` | 0.1095 | 0.055 | MEASURED, tyre diameter 11 cm by tape. The old number was the diameter entered as a radius |
| `drivetrain.gear_ratio` | 11.82 | 8.32 | DERIVED from the push test; tooth counts NOT verified |
| `actuation.throttle_full_scale_mps` | 2.91 | 2.08 | DERIVED: VESC `l_max_erpm` 6000 / pole pairs 2 / gear ratio 8.32 x 2 pi x 0.055 m / 60 = 2.077 m/s |
| `steering.pwm_to_angle_table` | null | 1094 us = +0.504 rad, 1500 us = 0, 1875 us = -0.449 rad | MEASURED, LiDAR scan matching at full lock, effective angle at low speed |
| `steering.min_angle_rad` / `max_angle_rad` | -0.4189 / +0.4189 | -0.449 / +0.449 | MEASURED, the weaker (right) lock, used both ways |
| `limits.ttc_brake_s`, `ttc_warning_s`, `min_forward_clearance_m` | 0.34 / 0.45 / 0.05 | see "Safety re-tune" | replayed on all floor bags in corrected units |
| `meta.schema_version` | 0.13.0 | 0.14.0 | every pin updated (racer_policy tests) |

## Measurements and method

**Overhangs.** Tape on the bench (owner). The front is the upper end of 10 to 11 cm because the
gate brakes earlier with a larger front overhang, so an error on that side is the safe one.

**Speed scale (push test).** The car was pushed by hand along a taped 3.00 m twice with the
throttle killed, and the along-track distance `/odom/wheel` logged was 4.139 m and 4.266 m, a
mean logged/true ratio of 1.401. `/odom/wheel` is `erpm / pole_pairs / gear_ratio x 2 pi x radius`,
so the logged distance scales with `radius / gear_ratio`. With the radius measured at 0.055 m the
ratio that makes the push test read true is 11.82 x (0.055 / 0.1095) x 1.401 = 8.32. That is
likely a larger pinion fitted with the Xerun motor swap; 11.82 was the stock Traxxas Slash 4x4
overall ratio and was never checked against this car. The number is only as good as the push test
(two runs, a 3 per cent spread) and the tyre radius (centimetre tape, about 1 per cent).

The bags that held the push test are not in `car-data/bags` (the earlier ones were deleted), so
it could not be recomputed. An independent check from the driven bags: the along-track distance of
`/odom/wheel` against LiDAR scan matching over 0.2 s pairs on straights gives a logged/true ratio
of 1.35 (28 pairs, 2026-11-11T04-20-19 v3 bag) to 1.36 (20 pairs, 03-54-11 bag), within 4 per
cent of the push test and on the same side as 1.40. At full lock it is 1.5 to 1.7 (the wheels
slip; see "Real-car notes").

**Steering, per side.** Scan matching of consecutive LiDAR scans while the car held full lock at
about 0.27 m/s (about 1500 scan pairs, owner's analysis). Left stop, 1094 us: rear-axle turning
radius median 0.60 m (quartiles 0.56 and 0.64), effective bicycle steer atan(wheelbase / R) = 28.9
deg = 0.504 rad. Right stop, 1875 us: 0.69 m (0.64 to 0.74), 25.7 deg = 0.449 rad. Neutral 1500
us is straight (measured 2026-09-21). The pulse travel is itself asymmetric, 406 us to the left
stop and 375 us to the right. Recomputed here on the 2026-11-10 bags (ICP of the same kind,
`tools/safety_replay/motion.py`, the car's own static returns removed, pairs 0.2 s apart where
/drive held full lock for the second before): left 0.62 m (quartiles 0.57 to 0.66, 760 pairs),
right 0.69 to 0.71 m (quartiles 0.68 to 0.73, 820 pairs). Agreement within 3 per cent.

Both angles are EFFECTIVE bicycle angles at low speed, with tyre slip, not a direct measurement
of the front wheels; the road-wheel angle is probably a little larger. Only the two stops and the
neutral are measured, so the table's three points make each side a straight line from neutral.

## How the steering map works now

The old map put a commanded angle on a straight line from neutral to the stop with `+/-0.4189 rad`
at the ends, symmetric in angle but not in pulse travel, so a commanded angle gave 1.20 times that
angle on the left and 1.07 times on the right. `racer_drivers/pwm_output_node` now inverts the
table (piecewise linear) so a commanded angle is the real angle, left and right
(`pwm_mapping.cpp`, `steering_table`; a table that is malformed, inconsistent with the pulse ends,
neutral or `pwm_left_bound`, or narrower than the angle limits is refused at startup). A null
table keeps the old straight-line behaviour.

**Planning uses a symmetric limit at the weaker side, `max_angle_rad = 0.449`.** Every planner
(gap follow, wall follow, tracker, park, the reverse escape), the safety node's arc corridor, the
teleop keymap, `wheel_twist_node`, the envelope and the sim read `max_angle_rad` alone and assume
a symmetric rack. Per-side limits would touch about fifteen call sites, each a place where a left
turn could be planned with the right turn's radius, and the sim's single `s_max`. With 0.449 a
planned turn is one the car can make to either side, which is the point of the exercise: gap_follow
and safety_node had been predicting a 0.74 m radius where the car does 0.60 m to the left and 0.69 m
to the right. The price is the extra left lock: the left stop (0.504 rad, 0.60 m) is in the table
but no planner asks for it, so the car turns 0.69 m to the left where it could do 0.60 m. If left
turns turn out to matter, the clean next step is a per-side `max_angle_rad` read through one helper
in each planner, not a second number in each.

Why the earlier clockwise laps were worse: with the symmetric map the right lock was
0.449 / 0.4189 = 1.07 times the commanded angle and the left 1.20 times, so the car turned
tighter than planned to both sides, and by 12 per cent less to the right than to the left.

## Old bags in the new units

`tools/safety_replay/calibration.py` converts a bag recorded under 0.13.0 or earlier: speeds
(`/drive_raw`, `/drive`, `/odom/wheel`) divided by 1.401, steering angles through the old linear
pulse map and then the table (`+0.4189` becomes +0.504, `-0.4189` becomes -0.449). `tools/
replay_safety.py` takes `--recorded-with 0.13.0` (required, because a bag does not say what it
was recorded under) and replays the bag twice: the rule it was driven with on the raw bag (that
is what the car's gate saw), and the rule under test on the converted one. Times, scans and
events are untouched, and everything measured from the LiDAR (end clearance, stopping travel,
approach speed) was always in true units.

## Safety re-tune

**Chosen: `ttc_brake_s` 0.60 (was 0.34), `ttc_warning_s` 0.71 (was 0.45, the release line, same
0.11 s gap so the release distance gap at crawl speed is unchanged), `min_forward_clearance_m`
0.10 (was 0.05). All still PROVISIONAL.**

Why the old numbers had to go: they were chosen on the October bags with the software believing
speeds 1.40 times the truth. In true units 0.34 s is 0.14 s of real braking time, and the replay
(below) shows 30 of 101 recorded brakes (4 bags: 2026-10-24, 2026-10-25, and the two 2026-11-10
runs, 1556 s and 1018 s) tripped later than the car's own recorded brake and 49 predicted under 4
cm of clearance.

Method (`tools/replay_safety.py --recorded-with 0.13.0`, same analysis as the 2026-11-02 note):
every bag through the real C++ gate, once with the rule it was driven with on the raw bag (the
October rule for the two October bags and the first 2026-11-10 run, which still had the 0.25 m
head floor; the 0.13.0 rule for the second 2026-11-10 run) and once with the rule under test on
the converted bag; every recorded forward brake gets a measured end clearance and stopping
travel from the LiDAR and a predicted end clearance under the new rule. Two fixes to the
analysis were needed first. (1) Every 2026-11-10 scan has a return inside the body footprint,
0.10 m ahead of the rear axle on the centreline, in 100 per cent of scans (a part of the car; it
is not in the October bags). It read as a permanent contact (end clearance 0 on every brake) and,
being fixed to the car, anchored the scan matching. `motion.static_self_returns` finds such
cells (inside the footprint, returned in at least half the scans) and every measurement leaves
them out. (2) The steering lock the gate is given for old bags is the table's 0.504 rad, not the
0.449 planning limit (`max_steering_rad` in `safety_replay_cli`).

Trade-off, all four bags together (floor 0.10 m, release = brake + 0.11 s; 101 recorded brakes;
"later" = the rule trips after the recorded brake; "under 4 cm" = predicted end clearance, with
the number that were replayable approaches in brackets; "never" = the corrected-radius gate sees
nothing in the path at the recorded brake; "new" = trips on no recorded approach, in brackets
those where the car drove on freely afterwards):

| `ttc_brake_s` | later | under 4 cm | never | min predicted (cm) | new brakes |
|---|---|---|---|---|---|
| 0.34 (committed 0.13.0, floor 0.05) | 30 | 49 (5) | 30 | -55 | 0 (0) |
| 0.45 | 27 | 23 (2) | 30 | -41 | 4 (4) |
| 0.50 | 18 | 17 (5) | 30 | -37 | 8 (7) |
| 0.55 | 6 | 13 (6) | 28 | -33 | 13 (8) |
| **0.60** | **4** | **12 (5)** | **28** | **-29** | **20 (15)** |
| 0.65 | 4 | 12 (5) | 25 | -25 | 25 (19) |
| 0.75 | 0 | 7 (4) | 19 | -18 | 43 (28) |
| 0.90 | 1 | 4 (2) | 22 | -6 | 72 (42) |
| 1.10 | 1 | 4 (3) | 27 | -1 | 134 (109) |

At 0.60 per bag: 2026-10-24, no brake predicted under 4 cm (median 17.8 cm, min 8.9 cm over the
six replayable approaches) and none later; 2026-10-25, one under 4 cm, the rest 15 cm or more;
2026-11-10 first run, none under 4 cm (min 6.0 cm), none later; 2026-11-10 second run (the
clockwise run), 11 under 4 cm. 0.60 is the knee: below it the October bags and the first
2026-11-10 run still have brakes under 4 cm or later than recorded; above it those do not
improve, and the new brakes double by 0.75. The floor does not matter between 0.05 and 0.15 m
(the TTC term, 0.60 x 0.39 m/s = 0.23 m, dominates down to 0.17 m/s); 0.10 m is the stopping
model's distance at that crossover speed (0.17 x 0.2 s + 0.17^2 / 2 = 0.05 m) plus the 4 cm
margin. Floors of 0.20 and 0.25 m only add new brakes at crawl speed (22 and 32 in all at 0.60,
against 20) for no change in the predictions.

The stopping model also says what the number should be: to stop 4 cm clear from speed v the
trip distance is v x 0.2 s + v^2 / 2 + 0.04 m, so a constant TTC has to be 0.50 s at 0.39 m/s,
0.56 s at 0.57 m/s, 0.61 s at 0.71 m/s (the October bags' fastest approaches) and 0.74 s at
1.0 m/s. 0.60 s covers the speeds the car was driven at; faster than about 0.7 m/s true it does
not, and the follower should not be asked to go there until the TTC is made speed-dependent.

**The requirement is NOT fully met, and it cannot be met by this threshold.** The goal was every
replayable brake predicted at least 4 cm clear and none later than the recorded one. With 0.60 s
there remain 12 predictions under 4 cm (11 in the second 2026-11-10 run, the clockwise run, the
car crawling at 0.39 m/s at the right-hand lock, and one in the 2026-10-25 bag) and 4 brakes later
than recorded, all in that second run. They do not go away with a larger TTC (7 remain at 0.75 s, 4 at 0.90 s and
1.10 s, at 20 to 150 new brakes), because they are not stopping-distance failures: the corrected
(tighter) right-hand arc measures the wall 0.28 to 0.38 m away along the path at the moment the
old, wider arc braked at about 0.19 m, and the car then stopped 5 to 9 cm from it. Where the
true path is tighter than the old one the corrected gate brakes later, which is the point of
the correction, but a wall grazed that tangentially is sensitive to a few millimetres of
radius, and the radius is the low-speed full-lock measurement (also see "Real-car notes"). The
2026-10-25 case is brake 94.76 s (0.76 m/s request, steering +0.398 old, +0.478 true): the
corrected gate sees 0.82 m of free path at the moment the old gate braked at 0.3 m, and the car
stopped 7 cm from the wall; no TTC under 1.08 s trips there. If the owner wants those covered
the lever is a gate steering input that includes understeer at speed, which needs a bag with
`/odom/wheel` and a known path (a motion capture or a drawn track), not a bigger TTC. 0.75 s is
the stricter option on the table (4 under 4 cm that are approaches, none later) at twice the new
brakes.

**Clockwise corners of 2026-11-10** (brakes with right steering, 32 recorded in the two
2026-11-10 bags): 11 brakes trip with the speeds corrected and the OLD radius (0.4189 rad as
recorded) and do not trip with the correct one. Nine of them stopped clear (4.8 to 9.0 cm end
clearance after the recorded brake) and the corrected gate sees nothing in the path at all: those
are the brakes that disappear with the correct turning radius and stay safe as far as the data
shows. Two do not stay safe: the corrected gate only trips 0.04 to 0.15 m later than needed and
predicts -5 and -9 cm. (`replay_safety.py --keep-recorded-steering` reproduces the old-radius
side.)

Not predicted at all (`never`, 28 of 101): the recorded brake was at full lock with the wall
outside the corrected, tighter arc. In the October bags all 17 of them stopped 6 to 15 cm clear.
This is where the owner's report that the car is braked in corners it can make shows up: 17 of
the 56 October brakes were that, and under the correct radius the safety node does not make
them.

## Sim, before and after

Same image (`ros-dev:loc`), the CI scripts unchanged, run on a copy of main (before) and this
branch (after). The sim reads `steering.max_angle_rad` as the gym's `s_max` (sim/racer_gym
`params.py`), so the turning circle in every sim test went from R 0.74 m to 0.69 m; the
tracker and localisation canaries cap their speeds at `throttle_full_scale_mps`, which went from
2.91 to 2.08 m/s; the safety thresholds are the new ones. The sim never sees the speed scale or
the pulse map (the bridge takes m/s and radians), so those cannot change it. The first
`l5_reactive_lap` before-run was killed (exit 137, the machine was shared with another agent's
simulators) and re-run alone.

| Test | Before | After |
|---|---|---|
| l5_reactive_lap, CCW 2 laps | 26.14 s, pass | 26.03 s, pass |
| l5_reactive_lap, CW 2 laps | 26.13 s, pass | 25.77 s, pass |
| escape canary, square corner | 26.08 s, 1 escape | 25.88 s, 1 escape |
| escape canary, round corner | 26.41 s, 1 escape, 0.226 m off the centreline | 29.95 s, 1 escape, 0.271 m (limit 0.395 m) |
| l5_tracker_lap, 2 laps | 12.48 s, pass | 12.45 s, pass |
| l5_tracker_lap, Oschersleben 1 lap | 24.71 s | 33.63 s (band 15 to 90 s; the raceline speed cap is 2.08 m/s now) |
| e2e_sim_safety (10 tests) | pass | pass |
| l5_park, min body clearance | pass, 0.204 m | pass, 0.219 m |
| l5_reactive_bench, CI battery (lane_square, lane_open_side, chicane) | brakes 4 / 0 / 0, min clearance 0.047 / 0.187 / 0.220 m, 0 contacts | brakes 12 / 0 / 0 (the TTC 0.60 s), 0.052 / 0.204 / 0.231 m, 0 contacts |
| l5_localization (mapping, oval, lane) | pass | pass; the oval runs at 2.08 m/s instead of 2.91, heading max 0.087 to 0.055 rad; the lane heading max went 0.100 to 0.184 rad, inside its threshold |

`tests/sim_regression` and every Python suite (`run_python_tests.sh`) pass unchanged, apart from
the two goldens below. Not re-run: the five-track `reactive_bench` for the two profiles the
2026-11-02 note used (only the CI subset above); lane_square braking three times as often is
the expected cost of 0.60 s on a track whose corners are tighter than the car's turning circle.

**Parking and the three-point turn** use `/odom/wheel` distances. The sim publishes that from the
gym's longitudinal speed in true metres, so the canary never exercised the speed scale and still
passes. On the real car the old odometry read 1.40 times the truth, so every dead-reckoned
distance and every "reverse 0.4 m" stopped 29 per cent short of what park_node believed; with
the new scale they are true metres. Its arcs are planned at 95 per cent of the planning lock,
R 0.7266 m, where they used to be planned as 0.7855 m and turned about 0.64 m to the left and
0.73 m to the right. Check the first real park run against `docs/notes/parking-2026-10-07.md`
before trusting it.

## Goldens regenerated, and why

- `training/racer_train/tests/fixtures/divergence_expected.json`: nine pure-pursuit commands that
  saturate at the steering limit moved from +/-0.4189 to +/-0.449 (`generate_divergence_fixture.py`
  reads `steering.max_angle_rad`); nothing else changed, and the C++ and Python controllers agree.
- `ros_ws/src/racer_control/test/replay/expected.jsonl`: four rows of the gap_follow replay moved
  the same way (the saturated steering and its low-pass); the lane_centre keys the file
  deliberately predates were left out as before.

## Real-car notes

- The same request is 1.40 times faster. See the runbook, step 0.
- At full lock the odometry reads 1.5 to 1.7 times the true speed (scan matching), not 1.40: the
  rear wheels slip. A converted cornering speed is therefore an upper bound on the true one,
  which errs toward earlier brakes in the replay and in the gate (it judges the request, and
  the measured odometry only ever raises it).
- The steering angles are effective angles at 0.27 m/s. The car probably understeers at speed,
  so the corrected arc is optimistic above that; it is the likeliest reason for the
  clockwise-corner residue above.
- The gear ratio 8.32 is derived, not counted. If the teeth give another ratio, `gear_ratio` and
  `throttle_full_scale_mps` move together
  (`tools/tests/test_vehicle_params_calibration.py` fails until both agree).

## safety_node segfault at shutdown (exit code -11 in CI)

CI on PR 116 (L5 park canary, run 42965728743) showed safety_node dying with exit code -11
right after bridge_node exited -2 at test shutdown. **I could not reproduce it, so the root cause
below is the one race the code allows, not an observed one.** Reproduction rate before the fix:
0 of 30 runs of the real L5 park canary, 0 of 100 SIGINTs with `/scan`, `/drive_raw` and
`/odom/wheel` streaming into the real executable (also 0 of 60 with four busy loops pinning the
cores and two CPUs, 0 of 100 at a 1000 Hz gate rate, 0 of 60 under AddressSanitizer and
UBSan). After the fix: 0 of 30 park canaries, 0 of 100 SIGINTs; so there is no measured
improvement, only no regression.

Cause: `main()` used `rclcpp::spin(node)` with rclcpp's default SIGINT handler. In Humble that
handler calls `rclcpp::shutdown()` from its own thread, which invalidates the rcl context while
the executor thread can be inside a callback on the same context (the 50 Hz timer publishing
`/drive`, a subscription take), and then destroys the node against a context that is already
gone. `on_timer` also had no guard, so a publish failing on the dying context threw into the
catch handler, which published again. The window is microseconds wide per cycle, which fits a
failure seen once in CI and never in many local runs. A callback racing a context teardown
in Fast DDS is a plausible segfault.

Fix, at the source (`racer_safety/src/safety_node.cpp`, `main()`): the process handles SIGINT and
SIGTERM itself (`SignalHandlerOptions::None`); the handler only sets an atomic flag; the single
executor thread polls it every 20 ms between callbacks, removes the node, the node and executor
are destroyed with the context still valid, and only then is the context shut down. `on_timer`
returns at once if the context is no longer ok. Nothing runs concurrently with the teardown.

Regression test: `test/check_safety_node_sigint.py` (ctest `test_safety_node_sigint`) starts
the real executable 20 times, streams `/scan`, `/drive_raw` and `/odom/wheel` faster than
their real rates, sends SIGINT at a different moment each time, KEEPS streaming while the node
shuts down, and fails on any exit status other than 0 or SIGINT, naming the signal (-11 is not
allowed). It passed 100 of 100 here. It would not have failed on the old code in my runs either,
so it is a guard, not a proof; if the crash comes back in CI, the next step is a core dump or an
ASan run of the failing job's binary.

## What the owner should check next

1. Count the pinion and spur teeth (and the transmission ratio) and compare with 8.32.
2. A direct steering-angle check (protractor or a turntable at three or four pulse widths),
   which also tests the straight line from neutral to each stop, and a check of the left stop's
   0.504 rad. If the left lock matters, make the planners per-side.
3. The push test again with the new calibration: `/odom/wheel` must read 3.00 m +/- 0.1 over the
   taped 3.00 m.
4. Re-measure the tyre diameter (it wears), and the front overhang if the bumper changes.
5. Floor session, both directions, vehicle_params 0.14.0 with `vesc:=true`, the first run with a
   speed request about 1.4 times lower than October's if you want the same pace (runbook step
   0), then `replay_safety.py --recorded-with current` on the bag. Watch clockwise corners at
   crawl speed, where this note could not make the replay clean.
6. The follower speeds and the corner speed cap
   (`docs/notes/safety-floor-review-2026-11-02.md`, recommendation 2) are unchanged and matter
   more now.
