# Stage 6: VESC configuration (layer 2) and Jetson bring-up (roadmap 1.4)

Time: 3 to 5 hours. Owner: Cameron. The VESC part can happen as early as stage 3.

## Part A: VESC Tool, safety layer 2

1. Install VESC Tool on the Mac. Connect the FSESC 6.7 by USB with the pack connected and
   wheels off the ground. Update the VESC firmware if VESC Tool asks.
2. Run motor setup: FOC, sensored (the hall sensor cable from stage 2). Run motor detection.
   If detection fails on the hall sensors, the sensor cable pinout is wrong; fix at the shop
   and retry. Record the detected parameters (resistance, inductance, flux linkage).
3. Set the layer-2 limits conservatively for bring-up: motor current 30 A, battery current
   25 A, brake/regen current 15 A, max ERPM equivalent to about 3 m/s road speed, battery
   cutoff start 3.4 V per cell and end 3.2 V per cell (3S: 10.2 V and 9.6 V), motor and MOSFET
   temperature limits per the motor's rating. These are widened later in Phase 4 with logged
   justification, never during a session.
4. Set the input mode: PWM (servo-style) input for the RC and mux path now, with the neutral
   and range matching stage 5 step 4. The ROS driver later talks over USB/UART; the PWM path is
   what the mux drives.
5. Export the configuration XML and commit it under `config/vesc/` with the date. It is
   re-exported and re-committed after every change, byte-for-byte diffable, and diffed before
   every driving session (an L6 procedure).
6. Mirror the limits into `config/vehicle_params.yaml`: `drivetrain.current_limit_a`,
   `drivetrain.brake_current_limit_a`, plus `drivetrain.gear_ratio` (spur teeth divided by
   pinion teeth times the truck's transmission ratio, from the Traxxas manual), `pole_pairs`
   (from the motor datasheet, typically 2 for a 4-pole 3665), and
   `motor_kv_rad_per_s_per_v` (4000 RPM/V converted to rad/s per V at this boundary).
   Regenerate bindings.

## Part B: Jetson

7. Flash JetPack 6.x onto the microSD using NVIDIA's Getting Started guide for the Orin Nano
   Super Developer Kit (SD card image written from the Mac with Balena Etcher). Caveat: kits
   with older QSPI firmware need a one-time firmware update that requires NVIDIA SDK Manager
   on an x86 Ubuntu host. A kit shipped in 2026 should already carry current firmware; if the
   SD image does not boot, that is the reason, and the fix is borrowing an Ubuntu x86 machine
   for an hour (the Mac cannot do it, and Docker on the Mac has no USB passthrough).
8. First boot with monitor and keyboard: create the user, connect to Wi-Fi, enable SSH, note
   the IP. From the Mac, `ssh` in. From this point the Mac never touches the car except over
   SSH (`claude-docs/03-environments.md`).
9. Set power mode to the 25 W Super mode (`sudo nvpmodel`) and install Docker on the Jetson.
10. Follow `docker/car/build_on_jetson.md` to build the car image. It is a draft written before
    the hardware existed and will need corrections; fix the doc in place as you go rather than
    working around it. The Jetson torch wheel URL is a required build argument the build
    refuses to guess; take it from NVIDIA's JetPack 6 PyTorch page for the installed JetPack.
11. Run a trivial `ros2 topic list` inside the built image. Set up a heartbeat process on the
    Jetson (a GPIO toggle at 50 Hz on the pin wired to the mux's GPIO 5, per
    `firmware/safety_mux/pico/heartbeat_input.h`); this is what stage 5's watchdog test needs.
12. Power the Jetson from the buck-boost rail (not the wall adapter) and confirm it boots and
    runs the image stably for 15 minutes with the motor idle.

## Done when

VESC detection succeeds with hall sensors, the exported config is committed and mirrored into
`vehicle_params.yaml`, the car image runs `ros2` on the Jetson over SSH, and the Jetson runs
from the pack. Tick roadmap 1.4 with a dated note and correct every "never verified" claim in
`docker/car/README.md`.

## Commit

`config/vesc/<date>.xml`, `vehicle_params.yaml`, fixes to `docker/car/`, build log.
