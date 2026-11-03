# LiDAR mapping and localisation, proven in the sim, 2026-11-03

The car had no pose. The raceline tracker and the residual policy both need one, so this adds
odometry (wheel speed plus modelled steering through an EKF), mapping (slam_toolbox) and
localisation (nav2 AMCL) to the stack, wires them into the car and sim launch files, and checks
them against ground truth in the sim with the car's sensor shapes and realistic sensor errors.
Nothing here has run on the Jetson or the car. The first floor session is in
`docs/notes/first-boot-runbook.md`, "First mapping session".

## What was built

| Piece | Where |
|---|---|
| `wheel_twist_node`: `/odom/wheel` speed + `/drive` steering (read only) through a servo model (rack limits, `steering.max_rate_rad_per_s`, `steering.time_constant_s` once measured) into a body twist `vx`, `vy = 0`, `wz = vx tan(delta) / L` with honest variances, on `/odom/wheel_twist` (TwistWithCovarianceStamped, base_link). ROS-free core with gtests (kinematics, left-positive signs, reverse, zero speed, servo lag and rate limit, stale and garbage input), L3 node test | `ros_ws/src/racer_state/` (new package) |
| EKF, slam_toolbox and AMCL parameter files, plus an L1 check that frames, topics and LiDAR ranges agree with the launch files and vehicle_params | `racer_state/config/{ekf,slam_toolbox,amcl}.yaml`, `racer_state/test/test_state_configs.py` |
| `odometry.launch.py` (twist node + `robot_localization` ekf_node, output remapped to `/odom`, TF odom -> base_link), `mapping.launch.py` (+ slam_toolbox online async), `localization.launch.py` (+ map_server, AMCL with `amcl_pose` remapped to `/pose`, lifecycle manager; refuses without `map:=`) | `ros_ws/src/racer_bringup/launch/` |
| `car_teleop.launch.py`: `mapping:=false`, `localization:=false`, `map:=` (both at once refuses); the recorder now also keeps `/odom`, `/odom/wheel_twist`, `/pose`, `/tf`, `/tf_static`, `/map` | `car_teleop.launch.py`, `test/test_localization_launch.py` |
| `sim_autopilot.launch.py`: the same three arguments plus `track_half_width_m` and `sensor_noise`; defaults unchanged | `sim_autopilot.launch.py` |
| `tracker_node`: `odom_topic` (default `/odom`) and `pose_frame` (default empty = unchanged). With `pose_frame:=map` it composes TF map -> odom with the `/odom` pose and stops on a missing or stale map -> odom | `racer_control/src/tracker_node.cpp`, `pose2d.{hpp,cpp}`, `test/test_pose2d.cpp`, `test/test_tracker_node_pose_frame_launch.py` |
| Bridge options, all off by default: `publish_ground_truth_tf`, `base_link_offset_x_m`, `wheel_speed_scale`, `wheel_speed_noise_std_mps`, `wheel_slip_s`, `wheel_odom_rate_hz`, `scan_rate_hz`, `scan_noise_std_m`, `scan_range_max_m`, `steering_offset_rad` | `sim/bridge/racer_gym_bridge/{bridge_node,sensor_noise}.py`, tests |
| L5 canaries: mapping (lane track), localisation on gym_oval at 2.91 m/s and on the lane track at 1.5 m/s; CI job `l5-localization` | `tests/l5_localization/`, `.github/scripts/l5_localization_test.sh` |
| The four apt packages (plus nav2_lifecycle_manager) in both images | `docker/ros-dev/Dockerfile`, `docker/car/Dockerfile` |

New topic: `/odom/wheel_twist`. `/odom` and `/pose` are the ones `claude-docs/04-architecture.md`
already names.

## Decisions (made before this work; recorded here)

- **Odometry is `robot_localization`'s ekf_node in 2D mode**, fed by one small new C++ node. The
  car has no IMU and no steering sensor, so the yaw rate is `v tan(delta) / L` from the VESC
  speed and the steering safety_node let through (`/drive`, which is what the servo is actually
  told), passed through the servo's rate limit (and its first-order lag once
  `steering.time_constant_s` is measured; it is null today, so only the rate limit applies,
  which is exactly what f1tenth_gym's steering does). Every constant comes from the generated
  binding. `ekf.yaml` has a commented IMU slot (`imu0`, yaw rate and ax) for when the ingest
  board exists.
- **Mapping is slam_toolbox online async**, tuned for the C1.
- **Localisation is nav2 AMCL**, with nav2_map_server and nav2_lifecycle_manager, publishing
  map -> odom and `/pose`. This deviates from `04-architecture.md`'s "F1TENTH particle filter":
  that filter needs a hand-built CUDA range_libc that only exists as community forks, while AMCL
  is C++, maintained and in apt for arm64. The F1TENTH particle filter stays the fallback if AMCL
  does not hold at speed on the car.
- **tracker_node composes the pose itself** rather than a new node publishing a map-frame
  Odometry. AMCL's `/pose` only comes out when the filter updates (every 0.1 m) and carries its
  latency; its map -> odom is republished every scan and the EKF's odom -> base_link runs at
  100 Hz, so map -> odom (latest) composed with `/odom` (latest) is the map-frame pose at the
  EKF's rate, the same thing nav2's controllers use, with no extra process or topic. The TF
  lookup runs on its own 25 Hz timer and hands plain doubles to the 50 Hz control callback.

## Frames

`map -> odom` (slam_toolbox or AMCL) `-> base_link` (EKF) `-> laser` (static, vehicle_params
`sensors.lidar`). base_link is the rear axle centre, where the kinematic `vy = 0` holds. The
bridge's ground-truth `map -> base_link` is switched off for all of this, and the bridge then
puts base_link at the rear axle too (`base_link_offset_x_m = -cg_to_rear_axle_m`), because the
gym's pose is the centre of gravity, where the car does slide sideways in a corner.

## Making the sim honest

f1tenth_gym's sensors are nearly perfect. The canaries and `sim_autopilot sensor_noise:=true` use:

| Error | Value | Why |
|---|---|---|
| Wheel speed scale | 1.02 | `tires.nominal_radius_m` is assumed, not measured; the car read 0.78 m/s for 0.80 commanded |
| Wheel speed noise | 0.02 m/s | ERPM quantisation |
| Wheelspin | 0.01 s x acceleration | more speed than the car under throttle |
| Steering trim | 0.01 rad | servo trim nobody told the model about |
| Scan range noise | 0.02 m on top of the gym's 0.01 m | C1 datasheet accuracy, roughly |
| Scan | 720 beams, 360 degrees, 10 Hz, 12 m, cast from the measured mount (0.285 m ahead of the rear axle, yaw pi) | vehicle_params `sensors.lidar` and `lidar_spec` |
| `/odom/wheel` | 50 Hz | the VESC driver's rate |

Above 0.5 m/s the gym runs its dynamic single-track model (tyre slip, load transfer), so the
kinematic yaw rate is wrong in every transient, as it will be on the car. What the sim still
does not have: scan motion distortion (the C1 takes 0.1 s per turn; at 2.9 m/s that is 0.29 m
of travel inside one scan), real reflectivity dropouts, wheels lifting, and a real servo's lag.

## Findings that changed the design

1. **A map with solid walls biases AMCL.** The bridge's corridor maps are solid blocks.
   AMCL's likelihood field does not ray cast, so a beam that ends anywhere inside a block scores
   as a perfect hit, and on gym_oval the filter slid 0.4 to 0.6 m towards the infield (where
   more beams end inside the block) until the car hit the wall, at every speed and with no noise
   at all. The canaries now give AMCL a surfaces-only map (wall cells that touch free floor are
   occupied, the inside is unknown), which is what slam_toolbox saves. On the car: localise on
   a slam_toolbox map, never on a hand-drawn one with filled walls.
2. **nav2's default AMCL settings lose the car at 2.9 m/s.** With every alpha at 0.2,
   `alpha2` alone adds 0.13 rad of heading noise per 0.29 m update, more than the scan pulls
   back. See the tuning table.
3. **The wheel speed scale error shows up along the track.** On gym_oval's constant-width
   corridor the walls say almost nothing about where along the track the car is (a straight
   corridor, and constant-radius arcs), so a 2% speed error leaves the estimate ahead of the car.
   Removing just the scale error took the oval's along-track error from 0.26 m to 0.07 m. The
   fix on the car is calibration (measure the tyre radius, see the runbook), not filter tuning.
4. **The odometry itself is worst in yaw at speed.** Over 1 s windows the EKF's odometry drifts
   (p95) 0.03 m and 0.05 rad at 1 m/s, 0.10 m and 0.08 rad at 2 m/s, 0.24 m and 0.12 rad at
   2.9 m/s, mostly the kinematic yaw rate missing the tyre dynamics. An IMU gyro is the fix
   (the EKF slot is there). EKF process noise changes made no measurable difference with one
   twist source (table).
5. **f1tenth_gym's reset is not the raceline's first point**: it picks a seeded random point
   in the first metre. The canaries take the start pose from ground truth and give it to AMCL on
   `/initialpose`, the way an operator does from Foxglove.
6. **tracker_node's default lookahead (0.4 to 1.5 m) cuts a 0.8 m corner of a 1.1 m lane into
   the inside wall** even on ground truth at 1.5 m/s. The lane canary uses 0.35 to 0.7 m. The
   floor track's corners are near the car's 0.74 m turning radius (`lane-centre-2026-10-24.md`),
   so a raceline for the floor needs that kind of lookahead and corners no tighter than about
   0.8 m.
7. **slam_toolbox lost the lane track at 1.5 m/s** (wall error mean 0.14 m, 36% spurious
   cells) and mapped it cleanly at 0.5 and 1.0 m/s. Map slowly.

## Tuning

All runs: the ros-dev image on the dev machine (Apple silicon, Docker), sensor noise on unless
stated, one run per row unless seeds are listed. Errors are TF map -> base_link at each
ground-truth stamp against ground truth moved to the rear axle, sampled at 20 Hz over the whole
run. "Along" is the signed mean error along the car's heading (positive = estimate ahead).
Runs were affected by another job sharing the machine at times; two runs where the VM clock
jumped are left out (the harness now fails a run on a clock jump).

### AMCL, gym_oval at the 2.91 m/s cap, surfaces-only ground-truth map

| Variant (changes from the row it builds on) | Laps | Pos mean / p95 / max (m) | Along (m) | Heading p95 / max (rad) |
|---|---|---|---|---|
| Solid corridor map, nav2 defaults (alphas 0.2, sigma_hit 0.2, z 0.5/0.5, 60 beams, 500-2000 particles) | crash after 2 s | n/a | estimate 0.4 to 0.6 m inside | n/a |
| Solid map, any alpha set tried (6 sets) | all crash | n/a | n/a | n/a |
| Surfaces map, nav2 defaults | 3 | 0.210 / 0.428 / 0.531 | | 0.176 / 0.229 |
| alphas 0.1/0.02/0.02/0.02, sigma 0.1, z 0.9/0.1 | 3 | 0.300 / 0.484 / 0.567 | | 0.108 / 0.126 |
| same, alphas 0.2/0.05/0.05/0.05 | 3 | 0.273 / 0.583 / 0.679 | | 0.149 / 0.205 |
| same, alphas 0.05/0.01/0.01/0.01 | 3 | 0.409 / 0.796 / 0.846 | | 0.231 / 0.265 |
| alphas 0.1/0.02/0.02/0.02, no noise at all | 3 | 0.089 / 0.146 / 0.178 | 0.019 | 0.056 / 0.080 |
| noise on, only the 2% scale error | 3 | 0.266 / 0.463 / 0.512 | 0.259 | 0.140 / 0.175 |
| noise on except the scale error | 3 | 0.123 / 0.284 / 0.339 | 0.072 | 0.069 / 0.137 |
| beams 120 (alpha3 0.02) | 3 | 0.318 / 0.502 / 0.586 | 0.313 | 0.121 / 0.158 |
| beams 240 | 3 | 0.243 / 0.427 / 0.480 | 0.238 | 0.094 / 0.117 |
| beams 120, alpha3 0.05 | 3 | 0.193 / 0.325 / 0.380 | 0.182 | 0.072 / 0.108 |
| beams 120, alpha3 0.1 | 3 | 0.188 / 0.408 / 0.504 | 0.158 | 0.087 / 0.127 |
| base B = alphas 0.1/0.02/0.05/0.02, sigma 0.1, z 0.9/0.1, beams 240 (3 seeds) | 3 | 0.178 to 0.195 / 0.31 to 0.38 / 0.38 to 0.47 | 0.17 to 0.19 | 0.064 to 0.079 / 0.08 to 0.12 |
| B, beams 360 | 3 | 0.212 / 0.393 / 0.434 | 0.202 | 0.075 / 0.135 |
| B, alpha1 0.2 / alpha1 0.05 / alpha2 0.05 | 3 | 0.202 / 0.219 / 0.303 mean | | |
| B, sigma_hit 0.05 | 3 | 0.167 / 0.311 / 0.345 | 0.158 | 0.057 / 0.091 |
| B, 1000-5000 particles | 3 | 0.143 / 0.278 / 0.327 | 0.131 | 0.060 / 0.076 |
| B, alpha3 0.2 | 3 | 0.143 / 0.283 / 0.402 | 0.094 | 0.076 / 0.118 |
| B, alpha3 0.2, sigma 0.05, 1000-5000 particles, 120 beams (3 seeds) | 3 | 0.118 to 0.155 / 0.22 to 0.32 / 0.25 to 0.43 | 0.03 to 0.08 | 0.070 to 0.076 |
| same, then one of: 60 beams / 500-2000 particles / alpha1, alpha2 0.05, 0.01 / beam model / likelihood max dist 1 m | all 5 lost the car (the cloud jumped up to 6 m along a straight, or to the 180 degree symmetric pose) | | | |
| same, then resample every 2 / alpha4 0.1 | 3 | 0.201 / 0.363 / 0.959 and 0.165 / 0.601 / 0.838 | 0.15, 0.10 | 0.088 / 0.335 and 0.165 / 0.250 |
| **Chosen: alphas 0.1/0.02/0.1/0.02, sigma 0.05, z 0.9/0.1, beams 240, 1000-5000 particles** (seeds 31, 32, 33, rerun of 31) | 3 | 0.151 to 0.182 / 0.27 to 0.34 / 0.33 to 0.46 | 0.13 to 0.17 | 0.058 to 0.067 / 0.09 to 0.11 |
| same at alpha3 0.05 (3 seeds) | 3 | 0.184 to 0.215 / 0.36 to 0.37 / 0.40 to 0.47 | 0.17 to 0.21 | 0.074 to 0.076 |
| same, 360 beams (3 seeds) | 3 | 0.134 to 0.166 / 0.26 to 0.35 / 0.32 to 0.45 | 0.11 to 0.14 | 0.057 to 0.061 |
| same, sigma 0.1, 2000-5000 particles (3 seeds) | 3 | 0.144 to 0.173 / 0.22 to 0.31 / 0.26 to 0.39 | 0.13 to 0.15 | 0.053 to 0.068 |

alpha3 0.2 gave the lowest numbers but was the setting under which the cloud jumped when anything
else was weakened; 0.1 never lost the car in any run. 360 beams and 2000 particles were no
better within the seed spread and cost more CPU. AMCL used about 10% of one dev-machine core at
2.9 m/s (top, 5 s window), so a fraction of one Orin Nano core.

### EKF (chosen AMCL settings, gym_oval 2.91 m/s, seed 31)

| Process noise (vx / vy / vyaw), rate | Pos mean / p95 / max (m) | Odometry drift 1 s p95 (m / rad) |
|---|---|---|
| 0.5 / 0.1 / 1.0, 100 Hz (chosen) | 0.182 / 0.338 / 0.398 | 0.260 / 0.122 |
| 0.05 / 0.01 / 0.1 | 0.214 / 0.352 / 0.404 | 0.235 / 0.108 |
| 2.0 / 0.5 / 5.0 | 0.135 / 0.328 / 0.432 | 0.227 / 0.124 |
| chosen, 50 Hz | 0.152 / 0.278 / 0.384 | 0.263 / 0.138 |

All inside the seed-to-seed spread of the chosen setting (0.151 to 0.182). With one twist source
the process noise only trades smoothing against lag; the error is the kinematic model, not the
filter. Kept at 100 Hz for `04-architecture.md`'s `/odom` >= 100 Hz.

### slam_toolbox, lane track, two laps

| Variant | Wall error mean / p95 (m) | Spurious | Coverage | Pose p95 (m) |
|---|---|---|---|---|
| Chosen (min travel 0.2 m / 0.2 rad / 0.1 s, update 1 s, 12 m), 1.0 m/s, seeds 21, 21, 22 | 0.030 to 0.035 / 0.07 to 0.10 | 0.000 to 0.022 | 1.00 | 0.20 to 0.24 |
| Chosen, no noise | 0.035 / 0.080 | 0.000 | 1.00 | 0.117 |
| Chosen, 0.5 m/s, one lap | 0.036 / 0.102 | 0.003 | 1.00 | 0.314 |
| Chosen, 1.5 m/s | 0.137 / 0.449 | 0.358 | 0.83 | 0.457 |
| slam_toolbox defaults (0.5 m / 0.5 rad / 0.5 s, update 5 s, 20 m), 1.0 m/s | 0.043 / 0.119 | 0.018 | 1.00 | 0.241 |
| Chosen, resolution 0.025 m | 0.030 / 0.068 | 0.000 | 1.00 | 0.212 |
| gym_oval, 1.0 m/s | 0.076 / 0.245 | 0.178 | 0.90 | 0.414 |

0.025 m resolution was slightly better and three times the cells; 0.05 m is kept (AMCL's map,
the C1's own resolution at 5 m is about 6 cm). The oval corridor maps worse for the same reason
it localises worse along track.

### AMCL on a slam_toolbox map (the real workflow)

The saved maps above, AMCL started at the map origin's frame (the mapping start pose), chosen
settings:

| Track, speed | Pos mean / p95 / max (m) | Heading p95 (rad) | Worst distance from centreline (m) |
|---|---|---|---|
| Lane, 1.5 m/s | 0.053 / 0.100 / 0.118 | 0.066 | 0.21 |
| gym_oval, 1.5 m/s | 0.172 / 0.338 / 0.428 | 0.042 | 0.32 |
| gym_oval, 2.91 m/s | 0.257 / 0.549 / 0.633 | 0.089 | 0.39 (lane limit 0.645) |

These include the SLAM map's own distortion against ground truth. On the lane track it is as
good as the ground-truth map.

## Measured accuracy, chosen settings, noise on

| Scenario | Pos mean / p95 / max (m) | Cross-track mean (m) | Heading p95 / max (rad) |
|---|---|---|---|
| Lane track, 1.0 m/s | 0.041 / 0.075 / 0.093 | 0.023 | 0.048 / 0.070 |
| Lane track, 1.5 m/s, seeds 41 to 43 | 0.054 to 0.060 / 0.10 to 0.11 / 0.11 to 0.12 | 0.023 | 0.075 to 0.084 / 0.09 to 0.11 |
| Lane track, 2.0 m/s | 0.106 / 0.181 / 0.204 | 0.031 | 0.138 / 0.168 |
| gym_oval, 1.0 m/s, seeds 31, 32 | 0.057 to 0.064 / 0.13 to 0.16 / 0.19 to 0.24 | 0.023 | 0.047 to 0.060 / 0.08 to 0.10 |
| gym_oval, 1.5 m/s | 0.074 / 0.146 / 0.192 | 0.025 | 0.033 / 0.046 |
| gym_oval, 2.0 m/s | 0.117 / 0.286 / 0.447 | 0.027 | 0.064 / 0.139 |
| gym_oval, 2.5 m/s | 0.111 / 0.224 / 0.279 | 0.035 | 0.045 / 0.063 |
| gym_oval, 2.91 m/s, 4 runs | 0.151 to 0.182 / 0.27 to 0.34 / 0.33 to 0.46 | 0.040 to 0.044 | 0.058 to 0.067 / 0.09 to 0.11 |

Cross-track error, the part that keeps the car in its lane, stays at 2 to 4 cm at every speed;
the rest is along track. Every run with the chosen settings completed its laps on the localised
pose without touching a wall.

## The canaries

`tests/l5_localization`, CI job `l5-localization` (thresholds and their measured values are in
each test file):

- `test_mapping_canary.py`: lane track (5.5 x 4.0 m loop, 1.1 m lane, 0.8 m corners), two laps
  at 1.0 m/s, tracker on ground truth, `mapping.launch.py`, then `/slam_toolbox/save_map` and a
  comparison of the saved pgm with `/sim/map` (wall error, spurious cells, coverage) plus the
  slam_toolbox pose.
- `test_localization_canary_oval.py`: gym_oval, 1.6 m corridor, 2.91 m/s cap, three laps with
  tracker_node driving on the AMCL pose.
- `test_localization_canary_lane.py`: lane track, 1.5 m/s, three laps on the AMCL pose.

All three fail at once on wall contact, leaving the lane, a stall, or a clock jump, before the
error thresholds are looked at. Tuning overrides (`L5_*` environment variables) are documented
at the top of `l5_localization_common.py`; CI sets none.

## Known limitations

- Along-track error on featureless walls: about 0.15 m mean at 2.9 m/s on the oval with an
  uncalibrated tyre radius. Calibrate the radius before chasing it in AMCL.
- No IMU: heading comes from the steering model, which is wrong in transients at speed.
- No steering sensor: `/drive` is what the servo was told, not where it went. Driving with the
  RC transmitter through the mux makes `/drive` meaningless for the EKF (the yaw rate is then
  wrong, and only the LiDAR saves the map). Map with Jetson teleop, not the RC.
- Scan motion distortion is not simulated and not corrected (slam_toolbox and AMCL treat each
  scan as instantaneous).
- AMCL needs an initial pose; there is no automatic global localisation step in the launch.
- The safety node's covariance gate is still not wired to `/pose` (roadmap 2.6).
- The oval canary's 2.91 m/s is the VESC cap, but the car has never been driven at it.

## How to run the first real mapping session

Short version (the full checklist is in `first-boot-runbook.md`, "First mapping session"):
rebuild the car image (new apt packages) and the workspace; start the stack with
`lidar:=true vesc:=true mapping:=true browser_teleop:=true`; drive one slow lap (0.8 to 1.0 m/s)
with the Jetson teleop; save the map with `/slam_toolbox/save_map`; restart with
`localization:=true map:=<yaml>` from the same start spot and check `/pose` in Foxglove while
pushing the car round by hand, then at walking pace.
