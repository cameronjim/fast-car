# Stage 5: the safety mux and the kill test (roadmap 1.3, gate G1)

Time: 6 to 10 hours across two or three sessions. This is the most careful stage of the whole
project. Read `claude-docs/05-safety.md` and `firmware/safety_mux/README.md` first. The
firmware exists as a draft with host-tested decision logic; its Pico glue has never compiled
or run. Expect to fix things and expect Claude to be involved at every step.

## What the mux is

An RP2040 sits between everything software and the two actuators. It reads the RC receiver's
kill-switch channel, reads the Jetson's steering and throttle PWM plus a heartbeat toggle, and
either passes the Jetson's PWM through to the servo and ESC or forces both to neutral and cuts
actuator power. Priority every cycle: kill switch, then heartbeat watchdog, then PWM validity,
then passthrough. It never talks to ROS and shares no power rail with the Jetson.

## Steps

### A. Measure the numbers the firmware refuses to run without

1. Bind the Flysky FS-i6X to the FS-iA6B receiver (transmitter manual). Assign a two-position
   switch to a channel, call it the kill channel. Power the receiver from the UBEC's 5 V.
2. With a scope or a Pico running a PWM-reading sketch, measure the kill channel's pulse width
   in both switch positions, plus the receiver's full valid range on a stick channel. Record:
   - `limits.mux_kill_switch_threshold_us` (halfway between the two switch positions)
   - the receiver's valid pulse range (note in `firmware/safety_mux/README.md`'s pinout table)
3. Measure the servo's usable pulse range and neutral by driving it with a servo tester or the
   receiver directly: the pulse at full left, full right, and center. Record into
   `steering.pwm_min_us`, `pwm_max_us`, `pwm_neutral_us`.
4. Decide the ESC PWM range and neutral (the VESC's PWM input mode, stage 6 sets it): record
   `actuation.throttle_pwm_min_us`, `throttle_pwm_max_us`, `throttle_pwm_neutral_us`.
5. Set `limits.mux_watchdog_timeout_s` conservatively (0.3 s) for now; tighten after the tests.
6. Claude edits `config/vehicle_params.yaml` with these values and regenerates bindings.

### B. Build and flash the firmware

7. Install the Pico toolchain on a Mac (arm-none-eabi-gcc via the ARM toolchain download, cmake,
   the Pico SDK fetched by the project's CMakeLists) and build `firmware/safety_mux/`. The
   first build will break on something; Claude fixes it. Commit the fixes.
8. Flash the mux Pico (hold BOOTSEL, plug USB, copy the .uf2). With the params still null it
   must refuse to arm and blink the fault LED fast. Prove that first, then flash the build made
   after step 6 and confirm it arms.

### C. Wire it

9. Wire per the pinout table in `firmware/safety_mux/README.md` (GPIO 2 kill channel in, 3
   steering in, 4 throttle in, 5 heartbeat in, 6 servo out, 7 ESC out, 8 power cutoff). Every
   5 V signal into the Pico goes through a level shifter channel; the Pico's 3.3 V outputs to
   the servo and ESC go through shifter channels the other way. Scope the shifted signals: clean
   edges, correct levels.
10. Power cutoff: the Pico's GPIO 8 drives a relay or a high-side MOSFET in the servo/ESC power
    path so that a dead Pico (pin low) means cut. Teammate designs this small circuit; Claude
    reviews the schematic photo. This is a bench decision the firmware README leaves open.
11. Mux Pico and receiver on the UBEC rail only. Confirm with the multimeter that no wire
    connects that rail to the buck-boost or Jetson.

### D. Bench procedures, wheels off the ground

Copy `tests/bench/procedures/template_wheels_off_actuation.yaml` to a dated procedure file and
fill it in as you go; the `racer_bench` runner records a session record you commit.

12. Kill switch to KILL with the Jetson (or a signal generator standing in for it) sending
    valid PWM: both outputs go to neutral and the cutoff opens.
13. Kill switch to ARMED with valid Jetson PWM: servo and ESC follow the commands, correct
    direction (steering left positive, throttle drive positive).
14. Stop the heartbeat toggle only: the mux cuts within the watchdog timeout (scope it).
15. Feed an out-of-range pulse on steering or throttle: the mux cuts, does not pass it through.
16. Brownout drill: sag the buck-boost input with a bench supply while the UBEC stays fed; the
    mux stays alive and keeps enforcing.

### E. The G1 kill test

17. Full car assembled, wheels off the ground, Jetson booted and actively commanding (stage 7's
    teleop or a simple PWM generator process on the Jetson with the heartbeat running), kill
    switch ARMED, two people present. Physically freeze the Jetson: stop every process
    (`sudo systemctl stop` or `kill -STOP` the heartbeat and command processes) and then, as a
    second test, hard power-cycle the Jetson while the mux stays powered. Observe directly: both
    outputs go neutral, the cutoff opens, the wheels stop. Photograph the scope and write down
    the observed cut latency.
18. Repeat the kill test three times. Then tighten `limits.mux_watchdog_timeout_s` toward the
    measured cut latency plus margin, rebuild, reflash, repeat once more.

## Done when

All five bench procedures pass with committed session records and the kill test is observed
three times. Tick roadmap 1.3 with a dated note describing exactly how the Jetson was frozen
and what was seen. Gate G1 is not fully passed until stage 7 (teleop through the mux) also
passes, but nothing may drive on the ground before this stage is complete.

## Commit

`config/vehicle_params.yaml`, firmware fixes, the filled-in procedure YAML and session records,
the cutoff circuit schematic photo, scope captures, all under a PR titled for task 1.3 with a
safety impact section.
