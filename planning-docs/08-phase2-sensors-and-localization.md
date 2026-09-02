# Stage 8: Phase 2, sensors, sync, state estimation, localization (roadmap 2.1 to 2.8, gate G2)

Time: 4 to 8 weeks part-time. This is historically the phase that overruns; the plan's answer
is to adopt existing components instead of writing them, and to write the sync numbers down.

Order the deferred cart at the start of this stage: RPLIDAR C1, INA226, Traxxas 6520 RPM
sensor, AS5600 encoder, SparkFun ICM-20948 IMU, IR break-beam pair, pool noodles, cones.

## Steps

1. Ingest board (2.1). The second Pico becomes the sensor collector: IMU over SPI or I2C at
   200 Hz or more, wheel-speed pulses from the RPM sensor, steering angle from the AS5600 on
   the servo output or rack, all timestamped against the Pico's own clock and streamed to the
   Jetson over USB serial as framed packets. Firmware lives in `firmware/ingest/`; it does no
   fusion and no filtering. A `racer_drivers` node parses it into `/ingest/imu`,
   `/ingest/wheel_speeds`, `/ingest/steering` (topic names and types fixed in
   `claude-docs/04-architecture.md`). Mount the IMU at the center of gravity; record its
   extrinsics in `vehicle_params.sensors`.
2. Steering calibration table. With the car on the stand, command a sweep of servo pulse
   widths and measure the road-wheel angle at each (protractor or a phone inclinometer on a
   flat plate against the tire). Record the table in `steering.pwm_to_angle_table`; it is not
   assumed linear.
3. INA226 on the 12 V compute rail, read by the ingest Pico, published as `/power/rail` at
   10 Hz. From here on this is the rail log and the VESC voltage is the cross-check.
4. LiDAR (2.3). Mount the RPLIDAR C1 level, centered, forward of the center of gravity, above
   the wheels' line of sight. Bring up the driver, confirm scan rate and beam count, record the
   mounting pose in `vehicle_params.sensors`.
5. Sync design doc (2.2). Measure offsets between the ingest clock, the LiDAR timeline, and
   the VESC telemetry timeline (a shared physical event visible to two sensors, for example a
   tap that shows in the IMU and a wheel-speed step). Quantify residual jitter. Write
   `docs/sync-design.md` with the numbers and their effect on state estimation at target
   speed. Store the offsets in `vehicle_params.sensors`.
6. EKF (2.4). Adopt `robot_localization` fusing IMU, wheel speeds, and steering angle into
   `/odom` at 100 Hz or more. Configuration only, no hand-written filter. Bench-validate with
   wheels off (yaw rate from a hand-turn matches the IMU) and then on the ground (odometry
   drift over a measured 10 m straight).
7. Venue map and localization (2.5). At the venue, drive the track by keyboard and record a
   bag; build a map with a standard 2D SLAM package; commit the map under
   `config/tracks/<venue>_<layout>/`. Bring up the F1TENTH particle filter against the map,
   fused with the EKF odometry, publishing `/pose`.
8. Covariance gate (2.6). Replace the stub in `racer_safety` with the real gate: pose
   covariance above threshold ramps the speed cap down; emits `/safety/events`. Tests to the
   existing 100 percent branch standard. Safety impact section in the PR.
9. Latency histogram (2.7). Per-hop timestamps through sensor, estimator, safety node, VESC
   command; publish and plot the full-path latency and jitter histogram; write the numbers in
   `docs/notes/`.
10. Replay suite (2.8). From real bags, seed `tests/bags/` with short segments: nominal lap,
    LiDAR dropout (cover the sensor), stale sensor (unplug the ingest USB mid-run on the stand),
    timestamp jump. Golden outputs with stated tolerances; fault-injection tests for the
    estimator and safety node run in CI.
11. Gate G2 sessions: 20 consecutive laps at 70 percent or more of the target speed with
    localization holding (no divergence, no covariance-gate slowdowns). Log every lap.

## Done when

All eight tasks ticked with dated notes, `docs/sync-design.md` committed, and G2 recorded as
passed. If G2 fails after honest attempts, lower the speed target and record the envelope
actually achieved; that is a result, not a failure of the project.
