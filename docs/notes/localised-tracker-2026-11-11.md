# The tracker on the localised pose, 2026-11-11

On the calibrated car (vehicle_params 0.14.0) the raceline tracker following the car's own
localisation (wheel odometry and steering model through the EKF, AMCL on the track's map) braked 4
to 95 times in three laps on five family tracks where it laps cleanly on ground-truth pose. That is
the configuration of the real car, so this note finds why. All runs: the ros-dev image on the dev
machine, the sim car of vehicle_params (0.449 rad each way), 0.64 m/s, the car's LiDAR mount, the
sensor errors of `sim_autopilot.launch.py` on (wheel speed scale 1.02, wheelspin, 0.01 rad steering
trim, 2 cm scan noise). Nothing ran on the car.

## Result

| Track | Dir | Ground truth | Localised, before | Localised, after |
|---|---|---|---|---|
| st_w1.0_s2_r0.2 | ccw | 3 laps 13.7s, 0b 0e 0c, clr 0.112 | 0 laps, 11b 0e 0c, clr 0.074 | 3 laps 13.9s, 5b 0e 0c, clr 0.087 |
| st_w1.0_s2_r0.2 | cw | 3 laps 12.8s, 0b 0e 0c, clr 0.113 | 0 laps, 10b 0e 0c, clr 0.069 | 3 laps 39.7s, 76b 0e 0c, clr 0.066 |
| st_w1.0_s2_r0.4 | ccw | 3 laps 15.0s, 0b 0e 0c, clr 0.225 | 3 laps 15.7s, 4b 0e 0c, clr 0.085 | 3 laps 14.8s, 0b 0e 0c, clr 0.194 |
| st_w1.0_s2_r0.4 | cw | 3 laps 15.0s, 0b 0e 0c, clr 0.236 | 0 laps, 28b 0e 0c, clr 0.065 | 3 laps 15.0s, 0b 0e 0c, clr 0.199 |
| rr_w1.0_2x1.5_r0.0 | ccw | 3 laps 16.0s, 0b 0e 0c, clr 0.119 | 0 laps, 17b 0e 0c, clr 0.075 | 3 laps 15.9s, 0b 0e 0c, clr 0.118 |
| rr_w1.0_2x1.5_r0.0 | cw | 3 laps 15.7s, 0b 0e 0c, clr 0.154 | 1 laps 95.3s, 58b 0e 0c, clr 0.065 | 3 laps 15.7s, 0b 0e 0c, clr 0.113 |
| rr_w1.0_3x2_r0.4 | ccw | 3 laps 19.3s, 0b 0e 0c, clr 0.219 | 3 laps 18.9s, 0b 0e 0c, clr 0.105 | 3 laps 19.1s, 0b 0e 0c, clr 0.198 |
| rr_w1.0_3x2_r0.4 | cw | 3 laps 19.3s, 0b 0e 0c, clr 0.216 | 2 laps 47.2s, 95b 0e 0c, clr 0.068 | 3 laps 19.3s, 0b 0e 0c, clr 0.192 |
| rr_w1.2_2x1.6_r0.8 | ccw | 3 laps 16.2s, 0b 0e 0c, clr 0.225 | 3 laps 16.9s, 0b 0e 0c, clr 0.108 | 3 laps 16.0s, 0b 0e 0c, clr 0.207 |
| rr_w1.2_2x1.6_r0.8 | cw | 3 laps 16.0s, 0b 0e 0c, clr 0.226 | 3 laps 22.1s, 28b 0e 0c, clr 0.072 | 3 laps 16.0s, 0b 0e 0c, clr 0.187 |

b = safety brakes in three laps, e = reverse escapes, c = wall contacts, clr = minimum body
clearance (m). Before: `docs/notes/track-session-2026-11-11.md` (the calibrated rerun). After:
the raceline asks for body clearance 0.25 m first (it used to stop at 0.15 m), and AMCL's
motion noise and update thresholds changed. Eight of the ten cases are clean (0 brakes). The 0.2 m
stadium is not: its centreline radius (0.7 m) is the car's own turning limit (0.69 m), so the
widest clearance its line can have is 0.11 m, and counter-clockwise it still takes 5 brakes and
clockwise 76. That track needs a slower line or a better odometry than there is; the ground-truth
tracker laps it with 0 brakes at the same 0.11 m clearance, so it sits exactly at the margin.

## What was measured, and what it showed

1. **The pose error is small.** Localised, after the fix, the position error is mean 0.03 to 0.04 m,
   p95 0.05 to 0.08 m, heading p95 0.03 to 0.075 rad. Before the fix it was about the same in
   position (mean 0.03 to 0.05 m) with heading p95 0.095 to 0.106 rad in the clockwise runs. The
   runner now records the error against ground truth split into along, across and heading
   (`localisation_error_split` in the result, a `loc` series in `--trace-out`).
2. **The brakes are `safety_node`'s, not the controller's.** At each brake the car is where the line
   says to be to within the pose error; what is wrong is where the line is. The 0.15 m body
   clearance the raceline was asked for becomes 0.09 to 0.12 m of real clearance (the car does not
   hold a line to better than its pose error and the pure pursuit cuts corners), and the safety
   gate's corridor (the body plus 0.05 m each side along the requested steering arc, a floor
   of 0.10 m) then sees a wall with a noisy scan (0.03 m of range noise by itself) and a 5 to 10
   cm pose error. Ground truth passes because it has neither.
3. **Ablation of the sensor errors (clockwise 0.4 m stadium, one run each, brakes in 3 laps)**:
   no errors 0; all 11 to 47 or stalled; only wheelspin 0; only scan noise 7; only wheel
   scale 7; only steering trim 12; all but any one still 9 to 67. Several errors each push a
   0.12 m clearance into the gate; none is a single bug.
4. **Odometry yaw (clockwise first turn)**: the EKF heading drifts 0.09 rad in the first 90 degree right
   turn with no sensor errors (the dynamic gym car understeers against the kinematic `v tan(delta)/L`
   yaw rate), 0.15 to 0.23 rad with steering trim or wheel scale error, against 0.03 counter-clockwise
   (the trim cancels the understeer to the left). AMCL has to take that out; with the old motion noise
   and 0.1 m / 0.1 rad update thresholds it did so slowly (map-frame heading error -0.08 rad at the
   first brake). `wheel_twist_node`'s steering model matches the command exactly (symmetric 0.449 rad,
   what the sim car gets); the drift is the car's dynamics, not the model's limit.
5. **Raceline clearance sweep (0.4 m stadium, localised, brakes in 3 laps, ccw / cw)**: asked
   0.15 m: 3 / 11 to 47; 0.20: 0 / 1; 0.25: 0 / 0 (real clearance 0.19); 0.30: 2 / 27 and 0.35: 2 / 11 (stalled), which
   the 1 m lane cannot give (the fallback line is poor). So 0.25 first,
   then the widest the lane allows.
6. **AMCL sweep (clockwise 0.4 m stadium, with the new clearance not yet applied, brakes)**: stock
   3 (stalled) to 47; update thresholds 0.05 alone 47; with 2000 to 8000 particles 46; sigma 0.1
   39; 480 beams 52; thresholds 0.02 with resampling every 2 scans 44; `alpha1` 0.2 and `alpha2` 0.1
   with the 0.05 thresholds 18. On the 0.2 m stadium (new clearance) counter-clockwise: stock 54, alpha 0.2/0.1
   with thresholds 0.05 -> 4, the same with 2000 to 8000 particles 3, heavier noise (0.3/0.15/0.15) 6, alpha
   0.2/0.1 with the old thresholds 38. Chosen: `alpha1` 0.2, `alpha2` 0.1, `update_min_d` and
   `update_min_a` 0.05 (the two L5 localisation canaries and the mapping canary still pass).
7. **Not found to matter**: the tracker's pose composition (map -> odom stamped by AMCL composed with
   the 100 Hz EKF) and stale-TF handling (no stale-transform stop in any run: the tracker never stopped
   publishing; the stalls are `safety_node` latches); latency between /pose and the tracker (not
   separately measured: the error series shows no lag signature, the error does not grow with speed
   or steering rate). I did not test a faster EKF or different EKF process noise.

## Fixes

* `tools/track_session/track_session.py` and the bench ask the raceline for body clearance 0.25 m first
  (`AUTO_CLEARANCES`), then the widest of 0.20 to 0.05 m the lane allows. The ground-truth tracker is
  not hurt (the wider line costs 2 to 4 percent of lap time: 15.0 against 14.9 s on the 0.4 m stadium).
* `ros_ws/src/racer_state/config/amcl.yaml`: `alpha1` 0.2, `alpha2` 0.1, `update_min_d` 0.05,
  `update_min_a` 0.05, with the reason in the file.
* The runner records the pose error split and series and takes `--noise-set`, `--clearances`,
  `--amcl-params` and `--ekf-params` for this kind of tuning.
* New L5 canary `tests/l5_track_family/test_tracker_localised_canary.py`: the localised tracker on
  the 0.4 m stadium and the 3 x 2 m rounded rectangle, both directions, at most 3 brakes, clearance at
  least 0.12 m, error bounds. With the fix: 4 of 4 pass. With the old 0.15 m ceiling (new AMCL): 4 of 4
  fail. CI job `l5-track-family` runs it.

## Open

The 0.2 m stadium (centreline radius at the car's limit) and every thin-island track stay outside what
the tracker does on the localised pose, for the geometry reasons of the track-session note. The
odometry's 0.09 rad understeer drift in a right turn is a modelling gap that an IMU yaw rate would
close (the EKF has the slot); AMCL carries it today.
