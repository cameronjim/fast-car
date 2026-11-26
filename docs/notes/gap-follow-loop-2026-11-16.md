# gap_follow on the indoor loop, 2026-11-16: why it refused, why the sim missed it, the fix

tldr; the car answers a scan about 0.25 s later than the sim's gym car. With race-2026-11-12's short
lookahead (0.6 m) and outside entry that is enough to turn in too late, and safety_node's TTC gate
refuses forward. The sim reproduces it once that lag is in the plant. New profile `race-2026-11-16`
(lookahead 0.8, centre line, stopping margin 0.2, a_lat 1.2, published-arc free path) laps the sim
copy of the loop with zero brakes. Not driven on the floor yet.

## The floor runs

* Data
  * Launch logs + DEVIATIONS: car `~/car/data/pnp/exc-2026-11-16/`, copied to `car-data/pnp/exc-2026-11-16/`
  * Bag: `2026-11-16T22-45-39_car_teleop` (no summary, read with StreamReader to an unreadable last
    record; every gap run is before it)
  * All runs ccw, `gap_follow.launch.py profile:=race-2026-11-12` with launch-argument overrides
* safety_node TTC brake engagements (`/safety/events`, ENGAGE, severity BRAKE, source `ttc`) vs
  reverse escapes (launch log)

| run | settings | length | TTC brakes | escapes | wall contact |
|---|---|---|---|---|---|
| G ccw 1.0 | race-2026-11-12, a_lat 2.0 | 30 s | 5 | 3 | none |
| G ccw 1.3 | race-2026-11-12, a_lat 2.0, max 1.3 | 45 s | 13 | 4 | none |
| F ccw 1.4 | race-2026-11-12, a_lat 2.4, max 1.4 | 20 s | 6 | 2 | none |
| L ccw 0.6 | lane-2026-10-25, target_mode gap_deepest | 40 s | 0 | off | none |

* Every brake = TTC on the requested arc, never the clearance floor. Detail strings: "time-to-collision
  0.33 to 0.60 s <= brake threshold 0.60 s at the requested speed"
* Two kinds
  * Short (0.02 to 0.26 s): request still 1.0 to 1.3 m/s at turn-in, released once the request drops
  * Long (2.0 to 2.5 s, one per escape): car at full lock, crawling at the 0.36 m/s floor, outer wall
    0.4 to 0.5 m ahead and closing. No arc gives 0.5 m -> escape after 1.5 s
* What the car did before each long brake (G 1.0 at +3.45 s, G 1.3 at +21.77, +45.04, F 1.4 at +10.46
  look the same)
  * Exit of one corner -> straighten -> steer RIGHT (-0.2 to -0.4 rad, the 0.25 m outside entry)
    toward the outer wall
  * Request 1.0 to 1.3 m/s with the published steering only +0.1 to +0.2 rad at 0.9 to 1.0 m from
    the wall ahead (head distance). Wheel speed 1.1 to 1.3
  * Request drops only at 0.7 to 0.8 m. Steering reaches full lock at 0.5 m -> full-lock circle (rear
    axle R 0.69 m, outer front corner 0.95 m) no longer clears the outer wall
  * = the owner's "going straight way too much, not initiating the turn early enough"
* Why the request stays high at turn-in: race-2026-11-12 measures the stopping law's free path along
  the TARGET arc (already curving away from the wall), the gate judges the PUBLISHED arc (still
  nearly straight). Same defect as the 2026-11-12 race bag; `free_path_on_published_steering`
  (race-2026-11-13) was the fix, but the floor runs used race-2026-11-12
* Free path horizon (1.26 m at 1.0, 1.68 at 1.3) = NOT a cause. Stopping from 1.0 m/s needs 0.96 m
  with the 0.2 m standoff, inside the horizon

### Measured on the car (LiDAR scan matching, `tools/safety_replay/motion.py` ICP, 1412 scan pairs)

* True speed p50/p90/max: G 1.0 0.76/0.92/1.07, G 1.3 0.69/1.05/1.23, F 1.4 0.65/0.97/1.40
* `/odom/wheel` vs true: 1.13 to 1.19 x (slip, as in safety-race-retune-2026-11-12.md). safety_node
  judges TTC on that high reading -> gate on the car is stricter than on paper
* Lateral acceleration reached (v > 0.4): p50 0.45, p99 1.11, max 1.23 m/s^2. a_lat 2.0 let the law
  ask for 1.17 m/s at full lock
* Curvature vs `/drive` steering: 0.91 x at full lock (gain table says 0.86). During turn-in 0.68 x
  -> lag. Best fit first order 0.15 to 0.2 s, weakly determined (residual hardly changes 0 to 0.2 s).
  yaw-rate-scan-matching-2026-11-13.md measured 0.05 to 0.075 s against the steering model, so call
  the steering lag 0.05 to 0.2 s, not pinned
* Scan age: `/scan` log time - header stamp = 0.100 s median (one sweep). The sim scan has none

### Rear near the wall (L 0.6, gap_deepest)

* Min clearance from the body behind the rear axle = 0.067 m at +17.3 s, steering +0.45 (full left
  lock); no scan under 0.05 m
* Mechanism: full lock beside the outer wall -> the 0.14 m rear overhang swings outward. The swept-path
  clamp only checks forward. gap_deepest sits at full lock most of the time (lane-2026-10-25 note)
* Not targeted here; race-2026-11-16 keeps whole-body clearance >= 0.17 m in every sim run below

## Why the sim check missed it

* Sim = `tests/l5_reactive_bench/track_family_runner.py`, real `safety_node` (same vehicle_params
  limits: TTC brake 0.60 s, release 0.71 s), real gap_follow_node, stock f1tenth_gym plant
* Differences, all making the sim easier
  * Steering: rate limit 3.2 rad/s only, no lag, gain 1.0 (car 0.86 to 0.92, lag 0.05 to 0.2 s)
  * Command: acts on arrival (car: no command delay measured; 0.26 s stop latency)
  * Braking: 9.5 m/s^2, 1.0 -> 0 in ~5 cm (car: 0.26 s then 1.0 m/s^2)
  * Scan: instantaneous, no age, 0.01 m noise (car: 0.1 s sweep)
  * No `/odom/wheel` for gap_follow runs -> gate judges the request + decaying bound, never the
    1.15 x high wheel speed
* Ablation, `rounded_rect` copy of the loop (below), race-2026-11-12 at a_lat 2.0, 1.3 m/s, ccw, 3 laps

| plant | TTC brakes | escapes |
|---|---|---|
| gym | 0 | 0 |
| car speed response only (dyn-v0.1 io.nominal: accel 2.0, decel 0.67, dead band 0.22), 2 seeds | 0, 0 | 0, 0 |
| that + steer tau 0.05 | 0 | 0 |
| that + steer tau 0.05 + command delay 0.1 | 0 | 0 |
| gym + steer tau 0.15 + command delay 0.1 | 11 | 3 |
| car speed response + steer tau 0.15 + delay 0.1 ("car plant"), seeds 1 to 4 | 4, 7, 7, 25 | 2, 3, 3, 4 |
| car plant, cw | 9 | 4 |

* -> the cause in sim = ~0.25 s between scan and wheels, not the deceleration. 0.15 + 0.1 is inside
  what the car has (0.05 to 0.2 lag + up to 0.1 scan age) but at the upper end; the match in counts
  (sim 13 brakes in 3 laps at 1.3 on the map vs car 13 in 45 s) supports it
* Racing line settings: outside entry 0.25 pushes the car toward the outer wall right before turn-in,
  apex 0.10 delays the turn. Not the root cause (removing them alone still braked, V1 below) but they
  spend the margin the lag needs

### Reproduction on the map and the synthetic copy

* Map = 2026-11-13-pnp-lap1 (SLAM, `car-data/maps/`, not committed): irregular octagonal room ~3.5 m,
  island 0.44 x 0.40 m, lane median 1.50 m, loop 5.8 m. Gaps in its outer wall at 12 and 3 o'clock
* Synthetic copy for CI = `rounded_rect`, lane 1.5 m, island 0.44 x 0.40 m, radius 0 (outer wall a
  3.4 m rounded square)
* Car plant = `--plant-bundle dyn-v0.1-2026-11-12 --plant-steer-tau 0.15 --plant-command-delay 0.1`.
  Slow plant = tau 0.25, delay 0.15
* race-2026-11-12, a_lat 2.0, 3 laps, ccw unless stated

| track, plant | max speed | TTC brakes | escapes | lap s |
|---|---|---|---|---|
| map, gym | 1.0 | 0 | 0 | 6.51 |
| map, car | 1.0 | 4 | 2 | 11.21 |
| map, car | 1.3 | 13 | 4 | 13.32 |
| map, car, cw | 1.0 | 0 | 0 | 7.57 |
| map, car, race-2026-11-13 | 1.0 | 3 | 1 | 10.14 |
| map, car, race-2026-11-13 | 1.3 | 1 | 1 | 8.64 |
| copy, gym | 1.3 | 0 | 0 | 5.62 |
| copy, car | 1.0 | 0 | 0 | 7.51 |
| copy, car | 1.3 | 4 | 2 | 9.49 |

## The fix: profile race-2026-11-16

* `race-2026-11-12` untouched (pre-registration). New profile = race-2026-11-13 (published-arc free
  path, max 1.3) +
  * `outside_entry_offset_m` 0.0, `apex_offset_m` 0.0 (centre line)
  * `lane_lookahead_m` 0.8 (turns in ~0.2 m / 0.2 s earlier, eats the lag)
  * `stopping_margin_m` 0.2 (stop 0.30 m short of the wall instead of 0.20)
  * `a_lat_max_mps2` 1.2 (what the car reached; conservative, see ablation)
* No code change. No safety_node change
* Ablation on the map, car plant, ccw, 1.3 m/s, 3 to 5 laps (race-2026-11-13 base unless stated)

| variant | TTC brakes | escapes | lap s | min clearance m |
|---|---|---|---|---|
| V1 centre line | 1 | 1 | 9.58 | 0.079 |
| V2 V1 + a_lat 1.2 | 3 | 1 | 10.73 | 0.043 |
| V3 V2 + stopping margin 0.25 | 0 | 0 | 10.10 | 0.259 |
| V4 V2 + lookahead 0.8 | 0 | 0 | 6.93 | 0.155 |
| V4 with a_lat 2.0 | 0 | 0 | 6.03 | 0.132 |
| V4 with outside entry 0.25 / apex 0.10 kept | 0 | 0 | 7.05 | 0.142 |
| V4 on race-2026-11-12 base (target-arc free path) | 0 | 0 | 7.12 | 0.114 |
| V4, slow plant, cw | 11 | 5 | 13.08 | 0.061 |

* -> lookahead 0.8 = the fix. Stopping margin 0.2 = what survives the slow plant (V4 slow cw braked
  at the map's 12 o'clock wall gap, the lane trace runs at the opening). Rest = margin
* Final matrix on the map, 5 laps each, P1 = V4 (margin 0.1), P2 = race-2026-11-16 (margin 0.2)

| | 1.0 ccw car | 1.0 ccw slow | 1.0 cw car | 1.0 cw slow | 1.3 ccw car | 1.3 ccw slow | 1.3 cw car | 1.3 cw slow |
|---|---|---|---|---|---|---|---|---|
| P1 brakes / escapes | 0/0 | 0/0 | 0/0 | 3/0 | 0/0 | 0/0 | 0/0 | 18/7 |
| P2 brakes / escapes | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 |
| P2 lap s | 7.54 | 8.67 | 6.84 | 7.78 | 7.37 | 8.73 | 6.71 | 8.07 |
| P2 min clearance m | 0.231 | 0.168 | 0.325 | 0.281 | 0.234 | 0.176 | 0.324 | 0.244 |

* P2 on the synthetic copy, 3 laps: car ccw/cw 0/0 brakes, 6.99/6.75 s; slow ccw/cw 0/0, 7.88/7.85 s;
  max 1.0 ccw/cw 0/0; gym ccw/cw 0/0, 6.20/6.18 s; car speed response only ccw/cw 0/0
* No wall contact in any run. Speed cost: on the copy with the gym plant P2 laps 6.2 s vs race-2026-11-12
  5.6 s (both 1.3); on the car plant race-2026-11-12 laps 9.5 to 16.4 s because of its escapes

## Tests

* L1 `ros_ws/src/racer_control/test/test_gap_follow_launch_profile.py`: profile = race-2026-11-13 +
  the five values, types float, race-2026-11-12 values pinned unchanged, overrides win
* L5 `tests/l5_track_family/test_gap_follow_loop_canary.py` (5 runs, ~2 min locally)
  * race-2026-11-12 G 1.3 settings, gym plant -> 0 brakes (why the old check passed)
  * same, car plant -> >= 1 brake (the reproduction, fails before the fix)
  * race-2026-11-16, car plant ccw + cw, slow plant ccw -> 0 brakes, 0 escapes, 0 contacts,
    clearance >= 0.1 m, lap in band
  * Plant fixture: `tests/l5_track_family/fixtures/plant-car-2026-11-16/manifest.yaml` (io.nominal of
    dyn-v0.1-2026-11-12 only)
* racer_app agent: profile list + 1.3 m/s ceiling (`commands.py`, mirrored test)

## Open / not done

* Not driven on the floor. Owner's call. Proposed first run: `profile:=race-2026-11-16
  max_speed_mps:=1.0`, ccw, 30 s, two people, RC kill in hand, wheels off the ground first, then
  1.3, then cw
* Steering lag not pinned (0.05 to 0.2 s). A wheels-off step test of the servo would settle the
  plant assumption
* Sim has no `/odom/wheel` for gap_follow -> safety_node's measured-speed path (1.15 x high on the
  car) is not exercised. Runner flag + bridge wheel-speed gain would close it
* The 11-13 map has wall gaps the 11-16 track may not; brakes in the slow-plant runs cluster there
* Long-brake geometry (full lock, 0.36 m/s floor, wall closing) is still possible if the car ever
  turns in late; the fix makes it not happen in sim, it does not remove the trap
* Rear-overhang swing (L 0.6) is unchecked by the follower

## Reproduce

```sh
# realistic-plant run on the map (data tree mounted at /data), as in tools/track_bench_docker.sh
python3 tests/l5_reactive_bench/track_family_runner.py --track map \
  --track-params-json '{"map_yaml":"/data/maps/2026-11-13-pnp-lap1/2026-11-13-pnp-lap1.yaml"}' \
  --profile race-2026-11-16 \
  --plant-bundle /data/pnp/models/dyn-v0.1-2026-11-12 --plant-steer-tau 0.15 --plant-command-delay 0.1 \
  --stop-after-laps 5 --duration 120 --out /data/runs/loop.json
```

Bag analysis scripts (scan matching, brake windows, scan plots) were one-off; the method is
`tools/safety_replay/motion.py` `car_motion` on consecutive scans with `static_self_returns` masked.
