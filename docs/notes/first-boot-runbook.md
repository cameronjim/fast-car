# First boot on the car: numbered runbook

Written 2026-09-13, **before any of it was done**. The Jetson was powered off; every step
below is a written-ahead procedure derived from the docs and the code, not a record of
something observed. Steps marked **UNVERIFIED** have never been executed anywhere; steps
marked **verified in container** were proven in the `ros-dev` image on the Mac against a fake
sysfs tree, which is not the same as proving anything about a Jetson pin.

Correct this file as you go. A step that turns out wrong is a finding, and it belongs in
`docs/notes/build-log.md` the same day.

## What this runbook gets you

The classical command path, end to end, on real hardware for the first time:

```
keyboard teleop -> /drive_raw -> safety_node -> /drive -> pwm_output_node
   -> 2x 50 Hz PWM -> layer-1 mux board -> steering servo + VESC PPM input
```

It does NOT get you: a kill-tested mux (roadmap 1.3's real test is still pending), measured
PWM calibration, a VESC driver, localization, or the policy. Wheels stay off the ground
throughout.

## Before you start: the standing rules

From `claude-docs/05-safety.md`, not negotiable:

- **Wheels off the ground.** The whole runbook. There is no step here that puts the car on
  the floor.
- **Two people.** One on the RC kill switch, one on the keyboard. The kill-switch person does
  nothing else.
- **The kill switch is armed and tested before power reaches the motor**, every session.
- **Rail voltage and a rosbag on every run** that drives anything.
- The mux is layer 1 and the only guarantee -- but **this mux has never been kill-tested**
  (`firmware/safety_mux/README.md`). Until roadmap 1.3's real test passes, treat the RC kill
  switch and the physical power connector as the actual safety measures and the mux as
  something being tested, not something being relied on.

Also note before touching anything: **the six PWM calibration values in
`config/vehicle_params.yaml` are convention, not measurement** (1000/1500/2000 us on both
channels). The one that bites is throttle neutral: if this ESC's real zero-throttle point is
not 1500 us, "neutral" is a creep. That is why step 11 puts a scope on the pulse before the
ESC is ever powered.

Also note DDS discovery. `docker/car`'s image now pins `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
and `ROS_DOMAIN_ID=42` (added 2026-09-14; see that image's README). This matters on trackside
WiFi specifically because the Mac's `ros-dev` container is very likely to be up on the same
subnet at the same time, and Fast DDS's default multicast discovery on the default domain (0)
would otherwise let the two unrelated ROS graphs see each other's nodes, or fail to discover
each other's, unpredictably. `--network host` (used below) is what actually exposes the
container to the LAN for discovery to happen at all -- confirm `echo $ROS_DOMAIN_ID` inside
the running container prints `42` before trusting `ros2 topic list` output.

Also note the throttle map's scale. `actuation.throttle_full_scale_mps` is 5.0 m/s, and it is
PROVISIONAL and unmeasured like the rest (added 2026-09-13, GitHub issue #40). With the
1000/1500/2000 us ends, **a commanded 1 m/s is 100 us off neutral: 1600 us forward, 1400 us
reverse**, and 5 m/s or anything above it saturates at 2000 us. It used to be scaled by
`limits.global_speed_cap_mps` (20 m/s), which made 1 m/s only 25 us off neutral, probably
inside the VESC's default PPM deadband -- so if you are working from an older printout and
the motor does nothing at low speeds, that is why. `limits.global_speed_cap_mps` still clamps
the command; it is no longer the map's full scale.

---

## 1. Power the Jetson and get a shell (UNVERIFIED on this exact sequence)

1.1 Connect the Jetson's own supply. Do not connect the drive battery yet, and leave the
    motor/ESC unpowered.

1.2 From the Mac:

```sh
ssh racer@racer-car        # 10.0.0.226
```

1.3 Confirm the heartbeat service that the mux's watchdog input depends on is running, which
    GPIO line it is actually configured for, and that nothing has been reconfigured under it:

```sh
systemctl status racer-heartbeat.service    # expect: active (running)
cat /etc/default/racer-heartbeat            # RACER_HB_CHIP / RACER_HB_LINE / RACER_HB_RATE_HZ --
                                             # this is the pin the unit actually asked for, not
                                             # necessarily header pin 7 / line 144
source /etc/default/racer-heartbeat
sudo gpioinfo "$RACER_HB_CHIP" | grep -w "$RACER_HB_LINE"   # expect: "racer-heartbeat" output [used]
```

`gpioinfo` only proves the Jetson's own kernel is driving that line -- it says nothing about
whether the signal reaches the mux, or even the physical header pad
(`tools/jetson_heartbeat/README.md`, "What this does not prove"; the same caveat
`docs/notes/mux-diagnostic-build.md` makes about debugfs-only checks). Do not stop at
`gpioinfo`. Confirm the signal actually gets to the mux with both of the following before
treating the heartbeat as working:

(a) **Meter at the physical pin.** With the mux board's JETSON connector plugged in
    (step 7), a DC multimeter on the header pin named in `/etc/default/racer-heartbeat`'s
    comment table (pin 7 = line 144, pin 32 = line 116), referenced to pin 9 (ground), should
    read a nonzero average consistent with a 3.3 V square wave at the configured
    `RACER_HB_RATE_HZ` -- not 0.0 V and not a steady 3.3 V.

(b) **`read_mux_diag.py` showing `HB OK`.** With the safety_mux DIAGNOSTIC firmware flashed
    (`docs/notes/mux-diagnostic-build.md`) and its USB port attached:

    ```sh
    python3 tools/mux_diag/read_mux_diag.py --device /dev/ttyACM0
    ```

    Expect a line containing `HB OK age <N>ms`, not `HB NO_EDGES_EVER` or `HB TIMED_OUT`.
    `NO_EDGES_EVER` here is the exact failure this step exists to catch: the Jetson's GPIO
    register can be toggling correctly (`gpioinfo` happy) while the mux still sees nothing,
    because the wire, the connector, or the pin choice itself is wrong.

If the service is not running, the line is not claimed, or `read_mux_diag.py` does not show
`HB OK`, stop and fix it before anything else: `tools/jetson_heartbeat/` has the
reproduction steps, and changing which pin is used is an edit to
`/etc/default/racer-heartbeat` plus `sudo systemctl restart racer-heartbeat.service`, not a
rebuild. A silent heartbeat means the mux sees a dead Jetson.

## 2. Clone the repo (UNVERIFIED -- not yet cloned on the device)

```sh
git clone https://github.com/cameronjim/fast-car.git car && cd car
```

## 3. Build the car image without torch (UNVERIFIED -- never built anywhere)

```sh
docker build -t car:local docker/car
```

Expect the loud `NOTICE: BUILDING WITHOUT TORCH` block. `docker/car/build_on_jetson.md` has
the full detail, including what to do when the build fails (it never has succeeded, so
assume it will fail at least once) and why the base image is JetPack 6.1 on a JetPack 6.2.1
host.

Confirm what you built:

```sh
docker run --rm car:local bash -lc 'echo "RACER_TORCH=$RACER_TORCH"'   # expect: absent
```

## 4. Enable the two hardware PWM pins (VERIFIED, confirmed 2026-09-20)

**This has actually been run**, on the Jetson Orin Nano Super Dev Kit, JetPack 6.2 / L4T
R36.4.4. Full steps and the reasoning: `ros_ws/src/racer_drivers/README.md`, "Enabling PWM
pins on the Jetson". In short:

4.1 `sudo python3 /opt/nvidia/jetson-io/config-by-function.py -l all` -- confirms the 40-pin
    header supports exactly `pwm1` (pin 15), `pwm5` (pin 33) and `pwm7` (pin 32); this board
    uses pins 15 and 33. **Confirm pin 7 stays a plain GPIO** (the heartbeat). Out of the box
    none of these functions are enabled, so both pins read 0.0 V even if a PWM controller is
    exported and running in sysfs -- the kernel does not refuse to run a controller whose
    output is not routed to a pad. That 0.0 V reading is the expected failure mode before
    step 4.2, not a wiring fault.

4.2 Enable both in one command and reboot:

    ```sh
    sudo python3 /opt/nvidia/jetson-io/config-by-function.py -o dt 1="pwm1 pwm5"
    ```

    This writes `/boot/jetson-io-hdr40-user-custom.dtbo` and adds it to
    `/boot/extlinux/extlinux.conf`, so it **persists across reboots** -- it is a one-time
    step. **Reboot** for the overlay to take effect.

4.3 After the reboot, confirm both are enabled and find the real chip/channel numbers -- do
    not assume `pwmchip0/pwm0`:

    ```sh
    sudo python3 /opt/nvidia/jetson-io/config-by-function.py -l enabled
    for chip in /sys/class/pwm/pwmchip*; do
      echo "$chip -> $(readlink -f "$chip" | sed 's#/sys/devices/##')  npwm=$(cat "$chip/npwm")"
    done
    ```

    **Measured on this device:** pin 15 = `pwmchip0` (`3280000.pwm`) = steering; pin 33 =
    `pwmchip2` (`32c0000.pwm`) = throttle; both `npwm=1`, so channel index 0 on each. This is
    now `pwm_output_node`'s default (`ros_ws/src/racer_drivers/README.md`). Confirm again on
    any other unit or kernel build -- the numbering depends on which pins were enabled and on
    probe order.

4.4 Already written into `docs/notes/build-log.md` with the date and the raw command output.
    A mapping that lives only in a terminal scrollback did not happen; do the same if you
    re-run this on different hardware.

## 5. Bench-check one PWM channel with nothing connected (VERIFIED, confirmed 2026-09-20)

Before any ROS node touches a pin, with **nothing plugged into the header**:

```sh
echo 0        | sudo tee /sys/class/pwm/pwmchipN/export
echo 20000000 | sudo tee /sys/class/pwm/pwmchipN/pwm0/period
echo 10000000 | sudo tee /sys/class/pwm/pwmchipN/pwm0/duty_cycle   # 50 percent duty
echo 1        | sudo tee /sys/class/pwm/pwmchipN/pwm0/enable
```

**Confirmed with a multimeter on this device:** ~1.64 V DC on both pin 15 (referenced to pin
14) and pin 33 (referenced to pin 34) at 50 Hz / 50 percent duty -- the expected average of a
3.3 V square wave at 50 percent (0.5 x 3.3 V = 1.65 V). Reading 0.0 V here before step 4.2's
pinmux change is the expected failure mode people hit, not a wiring fault. Then
`echo 0 | sudo tee /sys/class/pwm/pwmchipN/pwm0/enable`.

Repeat for the other pin. Both measurements are recorded in `docs/notes/build-log.md`,
2026-09-20.

**What this verifies and what it does not.** Verified: the pads carry a PWM signal at the
right average voltage for a hand-driven sysfs duty cycle. NOT verified: pulse-width accuracy
under load, `pwm_output_node`'s own behaviour driving these channels at 50 Hz with real
duty-cycle values, or anything downstream of the pins (servo, mux board, VESC). Steps 10-14
below are still where those get checked.

## 6. Build the workspace on the device (UNVERIFIED on-device; the same build is green in the ros-dev container on the Mac)

```sh
docker run --rm -it --runtime nvidia -v "$PWD":/workspace -w /workspace car:local bash -lc '
  source /opt/ros/humble/setup.bash
  rosdep update && rosdep install --from-paths ros_ws/src --ignore-src -r -y
  cd ros_ws && colcon build --symlink-install'
```

## 7. Cable the four Jetson wires to the mux board (UNVERIFIED -- the board is not built)

With **everything powered off**, and the pin-1 marks on both the board and the plug checked
before the plug goes in:

| Signal | Jetson header pin | Mux board hole (row 1) |
|---|---|---|
| Steering pulse | 15 | column 8 |
| Throttle pulse | 33 | column 9 |
| Heartbeat | 7 | column 10 |
| Ground | 9 | column 11 |

No 5 V wire: the Jetson powers itself. A reversed plug swaps heartbeat and steering and no
firmware check can see it, so check the mark, twice. Cross-reference
`firmware/safety_mux/README.md`'s board connector map and
`ros_ws/src/racer_drivers/README.md`'s cabling table.

## 8. Start the car launch file with the servo and ESC still unpowered (verified in container, UNVERIFIED on the car)

### Preferred: udev rule granting group write access (UNVERIFIED, try this first)

Added 2026-09-14, written ahead of hardware -- **never applied or tested on the device**.
`pwm_output_node` needs write access to `/sys/class/pwm/pwmchipN/{export,unexport}` and
`/sys/class/pwm/pwmchipN/pwmM/{period,duty_cycle,enable,polarity}`. Those entries are symlinks
into `/sys/devices/...`, so bind-mounting just `/sys/class/pwm` read-write does not help (the
target is still the container's own read-only `/sys` copy) -- the fix is host-side
permissions plus a narrow bind-mount of the real device path, not a container-side workaround.

8.1 On the Jetson, once (after step 4's pinmux reboot, so the `pwmchip` nodes actually exist):

```sh
sudo groupadd -f racer-pwm
sudo usermod -aG racer-pwm racer
```

8.2 Create `/etc/udev/rules.d/99-racer-pwm.rules`:

```
# racer-pwm: group write access to exported PWM channel attributes so pwm_output_node's
# container never needs --privileged. UNVERIFIED (docs/notes/first-boot-runbook.md, step 8).
SUBSYSTEM=="pwm", KERNEL=="pwmchip*", ACTION=="add", \
  RUN+="/bin/chgrp -R racer-pwm /sys/class/pwm/%k", \
  RUN+="/bin/chmod -R g+rwX /sys/class/pwm/%k"
SUBSYSTEM=="pwm", KERNEL=="pwm[0-9]*", ACTION=="add", \
  RUN+="/bin/sh -c 'chgrp racer-pwm /sys%p/period /sys%p/duty_cycle /sys%p/enable /sys%p/polarity 2>/dev/null; chmod g+rw /sys%p/period /sys%p/duty_cycle /sys%p/enable /sys%p/polarity 2>/dev/null'"
```

The first rule covers `pwmchipN/export` and `unexport` (present as soon as the chip appears);
the second re-applies group ownership to each channel's attribute files the moment `export`
creates them (they don't exist until then, so the first rule's `-R` cannot reach them).

8.3 Reload and re-trigger: `sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=pwm`.
Log out/in (or `newgrp racer-pwm`) so the `racer` shell picks up the new group.

8.4 Find the real device-tree path udev is granting access to (needed for the narrow
bind-mount below), and confirm the permissions actually landed:

```sh
readlink -f /sys/class/pwm/pwmchipN                 # e.g. /sys/devices/<soc-path>/pwmchipN
ls -l /sys/class/pwm/pwmchipN/export                # expect group racer-pwm, g+w
```

8.5 Launch WITHOUT `--privileged`, bind-mounting only the real device path from 8.4 (read-write)
instead of all of `/sys`, and joining the host's `racer-pwm` group:

```sh
docker run --rm -it --runtime nvidia --network host \
  --group-add "$(getent group racer-pwm | cut -d: -f3)" \
  -v /sys/devices/<soc-path-from-8.4>:/sys/devices/<soc-path-from-8.4> \
  -v "$PWD":/workspace -w /workspace/ros_ws car:local bash -lc '
    source /opt/ros/humble/setup.bash && source install/setup.bash
    ros2 launch racer_bringup car_teleop.launch.py \
      steering_pwmchip:=N steering_pwm_channel:=M \
      throttle_pwmchip:=P throttle_pwm_channel:=Q'
```

If `pwm_output_node` still fails to open/export the channel (permission denied), the group
membership or the udev rule's `RUN+=` ordering is the likely culprit -- check `journalctl -u
systemd-udevd` for the rule actually firing, and fall back to 8-fallback below rather than
guessing at more udev rules under time pressure during a bench session.

### Fallback: `--privileged` (the previously-documented approach, kept as the known-working blunt instrument)

```sh
docker run --rm -it --runtime nvidia --network host --privileged -v /sys:/sys \
  -v "$PWD":/workspace -w /workspace/ros_ws car:local bash -lc '
    source /opt/ros/humble/setup.bash && source install/setup.bash
    ros2 launch racer_bringup car_teleop.launch.py \
      steering_pwmchip:=N steering_pwm_channel:=M \
      throttle_pwmchip:=P throttle_pwm_channel:=Q'
```

`--privileged` with a writable `/sys` certainly works but grants far more than PWM write
access (every device, full capability set, no seccomp/apparmor confinement) to the container
that drives the actuators, which is not a resting place -- use it only if 8.1-8.5 above do not
pan out on the actual hardware, and note in `docs/notes/build-log.md` why the udev approach
did not work if you fall back to this.

Either way, expect two nodes and a startup line from `pwm_output_node` naming the calibration
it read out of `vehicle_params`. If it refuses to start, read the message: it names the field
it is missing, and the answer is to measure that field, never to edit the check.

## 9. Verify /drive is neutral with no input (verified in container, UNVERIFIED on the car)

In a second shell on the Jetson:

```sh
docker exec -it <container> bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/ros_ws/install/setup.bash && ros2 topic echo /drive --once'
```

Expect `steering_angle: 0.0` and `speed: 0.0`, at 50 Hz, forever, with nothing publishing
`/drive_raw`. This is exactly what the L3 launch test asserts in CI, so a difference here is
a hardware/environment finding worth writing down.

## 10. Verify the pulses electrically (UNVERIFIED)

```sh
cat /sys/class/pwm/pwmchipN/pwm0/duty_cycle    # expect 1500000 (ns) = 1500 us
cat /sys/class/pwm/pwmchipP/pwmQ/duty_cycle    # expect 1500000
```

Then the measurement that actually counts, on the pins themselves, with a scope or a
duty-reading meter: **50 Hz, 1.5 ms high on both**. The sysfs file says what was asked for;
the scope says what came out.

## 11. Power the servo ONLY, wheels off the ground, kill switch armed (UNVERIFIED)

The ESC and the drive battery stay out of this step. The steering polarity (step 12) is the
first bench calibration and there is no reason for the motor to be live while it happens.

11.1 Second person on the RC transmitter, kill switch in the CUT position, hand on it.

11.2 Power the mux board's 5 V rail (UBEC), then the receiver. Confirm the mux is not in its
     fault blink (`firmware/safety_mux/README.md`: fast blink = it refused to arm).

11.3 Connect the servo. Nothing should move. If the wheels twitch or crawl to a lock, cut
     power: the neutral value or the polarity is wrong, and step 12 is where that gets sorted
     out -- with the ESC still in the box.

## 12. Calibrate the steering polarity -- THE FIRST BENCH CALIBRATION (MEASURED 2026-09-21)

This is the one the code was explicitly waiting for, and it came before the ESC was powered
because it needed nothing but the servo and because getting it wrong is how the car steers
into the thing it was avoiding.

**MEASURED 2026-09-21, on the car, wheels off the ground, mux armed:** a SHORTER pulse turns
the wheels LEFT. `steering.pwm_left_bound` in `config/vehicle_params.yaml` is
`"pwm_min_us"` -- this is no longer a code/launch default (the old
`steering_left_is_pwm_max`, `true`, was a guess; see `docs/notes/build-log.md`'s 2026-09-21
evening entry). The whole left-positive chain -- `a`/LEFT key increases `steering_angle_rad`,
`angular.z > 0` with forward speed gives a positive angle, `safety_node` passes the sign
through unchanged, and `pwm_output_node` sends a positive angle to whichever pulse end
`steering.pwm_left_bound` names -- is pinned by unit tests at every hop
(`ros_ws/src/racer_drivers/test/test_pwm_mapping.cpp`'s `CommittedCalibration*` group and
`SteeringSign` group, `racer_safety/test/test_gate_logic.cpp`'s `SteeringSign` group,
`racer_tools/test/test_keymap.py`, `test_twist_teleop.py`).

What was done (for the historical record; do not repeat this to re-verify the sign, only to
re-verify a specific unit's servo wiring): with the wheels off the ground, the ESC unpowered,
and the kill switch held, a small LEFT command was published by hand (positive angle, zero
speed):

```sh
ros2 topic pub --once /drive_raw ackermann_msgs/msg/AckermannDriveStamped \
  '{drive: {steering_angle: 0.2, speed: 0.0}}'
```

The front wheels turned left, confirming the shorter-pulse-is-left convention above. The two
mechanical stops were also swept and are recorded in `config/vehicle_params.yaml`'s
`steering.pwm_min_us` (1094 us, left) / `pwm_max_us` (1875 us, right); see
`docs/notes/bench-session-2026-09-20.md`'s steering endpoint follow-up for the sweep method
and the quantisation caveat (the Jetson PWM's 78.125 us grid).

## 13. Configure the VESC in VESC Tool over USB, before it is ever fed a pulse (UNVERIFIED)

The VESC's USB link is configuration and telemetry only; the command path is the PWM pulse
through the mux into the PPM input (`docs/notes/build-log.md`, 2026-09-12). This step is the
layer-2 configuration (`claude-docs/05-safety.md`) and it decides what the pulses this repo
sends actually mean.

**Set the PPM app's Control Type to `Current No Reverse With Brake` for first boot.** Reasons,
in order:

- **`safety_node`'s "brake" is not a brake.** Layer 3's only lever is the `/drive` `speed`
  field, and every gate that fires writes 0.0 into it. `pwm_output_node` maps speed 0 to
  `actuation.throttle_pwm_neutral_us`, and a neutral PPM pulse is ZERO CURRENT -- a coast.
  "Watchdog fired, braking" produces a car that keeps rolling. Nothing in software can change
  that; the deceleration, if there is to be any, has to come from this setting or from a
  future closed-loop `vesc_node`.
- **It makes a below-neutral pulse the safe thing rather than the dangerous thing.** In
  `Current`, below neutral is reverse drive current: a sign error or a stuck negative command
  spins the motor backwards. In `Current No Reverse With Brake` the same pulse is proportional
  braking. On a car nobody has driven, the direction a mistake should fail in is obvious.
- **Reverse is not wanted for the first drives anyway** (see step 15 and the `allow_reverse`
  parameter), so giving up reverse costs nothing right now.

What this step does NOT do: pick numbers. Deadband width, current limits, ramping -- none of
those have been measured on this car and none is written down anywhere in this repo, so none
is prescribed here. Set the control type, leave the rest at whatever VESC Tool gives you,
**export the configuration and commit it** (12-testing L6 has a "VESC config diff: exported
config matches the committed layer-2 config" bench check that needs a committed baseline to
diff against), and write what you chose into `docs/notes/build-log.md`.

If you choose a different control type, write down that you did and why, because the words
"brake" and "reverse" in this repo's code and docs are written against this one.

## 14. Power the ESC, wheels off the ground (UNVERIFIED)

14.1 Only now connect the drive battery / ESC. Expect silence and no motor motion. **A
     creeping motor here means this ESC's real zero-throttle point is not 1500 us** -- cut
     power, measure it, and put the measured value in `config/vehicle_params.yaml` before
     going further. The mux firmware bakes these in at compile time, so that means rebuild
     and reflash too.

## 15. Keyboard teleop, wheels off the ground, second person on the kill switch (UNVERIFIED)

**Reverse is OFF by default and that is deliberate.** Both teleop sources take an
`allow_reverse` parameter, default `false`, which clamps the commanded speed floor at 0.0 m/s
instead of `limits.min_velocity_mps` (-5.0). So the throttle-down key decelerates to a stop
and stops there, and no below-neutral pulse is ever produced. Leave it off for these first
drives. Turn it on -- `ros2 run racer_tools keyboard_teleop_node --ros-args -p
allow_reverse:=true`, or `allow_reverse:=true` on `car_teleop.launch.py` -- only once step 13
is done and recorded, because until then nobody knows whether a below-neutral pulse is reverse
or braking on this ESC.

15.1 Confirm the rosbag is recording and rail voltage is being logged. A run without a bag is
     a bug (`CLAUDE.md` invariant 5). Since 2026-09-21 the launch does this for you: look for
     the `[invariant 5] recording <format> bag to <path>` line at startup, and for
     `rail_voltage_node` reporting its INA3221 channels rather than warning that none were
     found. See "Every run is recorded" below.

15.2 Kill-switch person: cut, confirm the wheels and motor stop, restore. Do this before
     driving, not after.

15.3 In a second terminal (keyboard teleop needs a real TTY, which is why it is not started
     by the launch file):

```sh
docker exec -it <container> bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/ros_ws/install/setup.bash && ros2 run racer_tools keyboard_teleop_node'
```

15.4 Smallest possible speed command first. Confirm: the motor spins the correct direction,
     releasing the key returns to neutral within the watchdog timeout, and the kill switch
     stops it instantly at any point. A 1 m/s command should be 1600 us on the throttle
     channel; if the motor does not move at 1600 us, the VESC's PPM deadband is wider than
     100 us and wants narrowing in VESC Tool (record the change in the committed VESC config),
     not a bigger number in vehicle_params.

15.5 **Release the key and watch what actually happens.** Releasing does not brake: the
     command goes to zero, the pulse goes to neutral, and on this ESC that is zero current.
     With the wheels off the ground the motor will spin down slowly. That is the layer-3
     "brake" behaving exactly as documented, not a fault. Time the spin-down and write it
     down -- it is the first real evidence of what a watchdog trip does on this car.

15.6 **This is where `actuation.throttle_full_scale_mps` gets measured.** With the wheels
     still off the ground, sweep the commanded speed up to full scale, record wheel speed
     against pulse width, and set that field to the speed observed at
     `actuation.throttle_pwm_max_us`. Until that is done the 5.0 in the committed file is a
     guess, and the open-loop map does not claim that commanding X m/s produces X m/s.

15.7 Stop, power down in reverse order (drive battery, then servo/receiver rail, then the
     Jetson), and write the session up in `docs/notes/build-log.md` the same day.

## Launch and drive (VERIFIED end to end on the Jetson, 2026-09-21)

**This section was executed, not written ahead.** Everything below ran on the real Jetson
(racer@10.0.0.226, JetPack 6.2.1 / L4T R36.4.4) on 2026-09-21, with the LiPo OUT of the car,
the VESC and servo unpowered and the servo lead unplugged from the mux board, so nothing could
move. What that session proved and what it did not is in `docs/notes/build-log.md`'s
2026-09-21 evening entry. Steps 1-15 above are the first-time procedure; this is the short
version for every session after the pins and the image already exist.

### Pre-drive checklist (do these in order, every time)

Nothing in this checklist is optional, and the order matters: the car gets power only after
the software is up and holding neutral.

**Steering endpoints and sign are now measured (2026-09-21 evening):** 1094/1500/1875 us,
shorter pulse is LEFT (`config/vehicle_params.yaml`'s `steering.pwm_min_us` / `pwm_max_us` /
`pwm_left_bound`, see "Reading the mux numbers" below). A full-lock steering command no
longer reaches the mux's 1000-2000 us window edge, so the earlier caution against commanding
full lock while armed no longer applies to steering specifically -- the general first-drive
caution against anything untested still does.

1. **Wheels off the ground.** A stand, a box, anything. The whole of Phase 1 is off-ground.
2. **Second person on the kill switch**, before anything is powered (`claude-docs/05-safety.md`
   two-person rule).
3. **Heartbeat alive**: `systemctl is-active racer-heartbeat` must print `active` BEFORE the
   stack starts. If it is not, stop -- the mux watchdog is the Jetson's liveness signal and
   the mux will cut anyway (`DECISION=CUT reason=2:WATCHDOG_TIMEOUT`).
4. **Transmitter on, kill knob (VrA / CH5) fully COUNTER-CLOCKWISE** (= killed). Turn the
   transmitter on before the car has power, and off after, always.
5. **Plug the servo lead back into the mux board**, and put the battery in. Check the pin-1
   mark on the plug.
6. **Start the stack** (below) and confirm both channels sit at neutral BEFORE arming.
7. **Confirm it is recording.** The startup log must show `[invariant 5] recording <format>
   bag to <path>` and `rail_voltage_node` must list INA3221 channels, not warn that it found
   none. No bag, no drive -- see "Every run is recorded" below.
8. **Only then turn the kill knob clockwise to arm.** The mux diagnostic build should show
   `DECISION=PASS` only at this point, and `CUT` at every earlier step. If it shows `PASS`
   before you armed, stop and find out why.

### Start the stack

One command, from `~/car` on the Jetson. No `--privileged`, no `-v /sys:/sys`, no
`--runtime nvidia` (nothing in the control stack uses CUDA):

```sh
cd ~/car
docker run --rm -it --name car-stack --network host \
  --user "$(id -u):$(id -g)" \
  --group-add "$(getent group gpio | cut -d: -f3)" \
  -e HOME=/tmp \
  -v /sys/devices/platform/bus@0/3280000.pwm:/sys/devices/platform/bus@0/3280000.pwm \
  -v /sys/devices/platform/bus@0/32c0000.pwm:/sys/devices/platform/bus@0/32c0000.pwm \
  -v "$PWD":/workspace -w /workspace/ros_ws \
  car:local bash -lc '
    source /opt/ros/humble/setup.bash && source install/setup.bash
    exec ros2 launch racer_bringup car_teleop.launch.py browser_teleop:=true'
```

Drop `browser_teleop:=true` to come up with no command source at all (the launch file's
default), which is what you want for a pure neutral check.

**Why this is not `--privileged`** (this replaces step 8's "UNVERIFIED, try this first"
sketch, and the custom `racer-pwm` group and udev rule it proposed are NOT needed):

- The Jetson already ships `/lib/udev/rules.d/60-jetson-gpio-common.rules`, which chgrps
  `pwmchipN/{export,unexport}` and each exported channel's `period`/`duty_cycle`/`enable` to
  the **`gpio`** group. `racer` is already in `gpio`. So `--group-add` that group's GID (999
  on this device) is the whole permission story -- verified 2026-09-21, both channels driven
  for real from an unprivileged, non-root container.
- The two `-v` lines bind-mount only the real device-tree paths the `/sys/class/pwm/pwmchipN`
  symlinks resolve to, read-write. Docker mounts `/sys` read-only, and bind-mounting
  `/sys/class/pwm` does not help because those entries are symlinks into `/sys/devices` --
  mounting the two `*.pwm` platform directories is the narrow fix. Find them again with
  `readlink -f /sys/class/pwm/pwmchip0` if the device ever changes.
- `--user "$(id -u):$(id -g)"` keeps the container off root and stops it writing root-owned
  build artifacts into your repo. `-e HOME=/tmp` is needed because that UID has no passwd
  entry inside the image and ROS wants a writable HOME for its logs.
- `--network host` is for DDS discovery and for the Foxglove bridge's port 8765.
- `exec` before `ros2 launch` matters -- see "Stopping" below.
- Nothing extra is needed for logging: the `-v "$PWD":/workspace` mount above is also where
  the bags go (`~/car/data/bags` on the host). Add `record:=false` ONLY for the pin-level
  checks where the battery is out -- see "Every run is recorded" below.

The workspace must already be built (step 6). If `install/` is missing, build it first, and
note the `apt-get update` that step 6's original command was missing:

```sh
docker run --rm -v "$PWD":/workspace -w /workspace car:local bash -lc '
  source /opt/ros/humble/setup.bash
  apt-get update && rosdep install --from-paths ros_ws/src --ignore-src -r -y
  cd ros_ws && colcon build --symlink-install
  chown -R 1000:1000 build install log /workspace/tools/.venv'
```

`apt-get update` is required because the image deletes `/var/lib/apt/lists/*`, so `rosdep`'s
`apt-get install` cannot find `python3-jsonschema` without it. The `chown` is because that
build has to run as root (rosdep installs packages) and would otherwise leave root-owned
directories in the repo.

### Confirm neutral before arming

```sh
# Both channels: 20 ms period, 1500 us pulse, enabled.
for c in 0 2; do cat /sys/class/pwm/pwmchip$c/pwm0/{period,duty_cycle,enable}; done

# What the mux actually SEES (this is the evidence that matters):
sudo stty -F /dev/ttyACM0 115200 raw -echo; sudo timeout 5 cat /dev/ttyACM0
```

Expect `STEER gp10=1484us FRESH(...)` and `THR gp7=1484us FRESH(...)`. **1484, not 1500, is
correct** -- see "Reading the mux numbers" below. With the transmitter still off you will also
see `KILL ... NO_EDGES` and `DECISION=CUT reason=1:RC_SIGNAL_INVALID`; that is expected and is
exactly what you want before arming.

Only one process may read `/dev/ttyACM0` at a time -- a second reader steals the bytes.

### Drive it from a browser on the Mac

1. Same WiFi as the car. On the Mac, open <https://app.foxglove.dev> (or the desktop app).
2. **Open connection** -> **Foxglove WebSocket** -> `ws://10.0.0.226:8765`. (Substitute the
   car's address; `racer-car` also resolves on this network. There is no TLS and no auth on
   this port -- it is a LAN-only debug interface.)
3. Add a **Teleop** panel and point it at `/teleop/cmd_vel`. The committed layout
   `ros_ws/src/racer_bringup/config/foxglove_sim_viz.layout.json` already contains one wired
   to that topic and can be imported with **Import layout from file**, but it is the SIM
   layout: its 3D panel expects `/sim/...` topics that do not exist on the car and will sit
   empty. There is no car-specific layout yet.
4. The launch must have been started with `browser_teleop:=true`, otherwise
   `twist_teleop_adapter_node` is not running and the panel publishes into nothing.
5. Click and hold a direction button. Path:
   `Teleop panel -> /teleop/cmd_vel (geometry_msgs/Twist) -> twist_teleop_adapter_node ->
   /drive_raw -> safety_node -> /drive -> pwm_output_node -> PWM -> mux`. Releasing all
   buttons commands zero after `twist_timeout_s` (0.5 s).

Verified 2026-09-21: the websocket handshake returns `101 Switching Protocols` with
`sec-websocket-protocol: foxglove.sdk.v1` both from the Jetson itself and from the Mac over
the LAN, and a `Twist` published on `/teleop/cmd_vel` moved both PWM channels through the full
gated path. **A human clicking the panel in a real browser has still not been done** -- the
panel config has never been visually confirmed (the same gap `docs/notes/milestone-5-browser-
teleop.md` already records).

The default Teleop panel binds the up button to `linear.x = 2.0 m/s`, which is just above the
deadzone below -- deliberately, so the first click actually moves the car rather than clicking.

### The throttle start deadzone (expect this, it is not a fault)

This drivetrain is **sensorless**, and it needs roughly **1700 us** (about 40 percent of the
throttle range) to start turning from rest. Measured on the bench 2026-09-21: 1560 us did
nothing, 1600 and 1650 us made the rear tyres click for a few seconds without turning, 1700
and 1750 us spun them up.

On the provisional open-loop map (`actuation.throttle_full_scale_mps: 5.0`, so 1 m/s = 100 us
off neutral) that means:

| Commanded speed | Pulse | What happens from rest |
|---|---|---|
| 0.5 m/s | 1550 us | nothing |
| 1.0 m/s | 1600 us | clicks, does not turn |
| 1.5 m/s | 1650 us | clicks, does not turn |
| 2.0 m/s | 1700 us | starts |

So **speed commands below roughly 2 m/s will click and not move**. That is expected until the
sensored hall adapter is fitted, which is the proper fix; it is not a reason to raise
`throttle_full_scale_mps`, and the map does not model the deadzone today. Do not sit on a
clicking command -- it is a stalled motor drawing current.

### Reading the mux numbers (why 1500 reads as 1484)

The DIAG_BUILD firmware measures pulse width on a **15.625 us grid** and reports the nearest
grid point. Every reading below is an exact multiple of 15.625 us. Measured 2026-09-21,
commanded value from sysfs against what the Pico reported:

| Commanded | `duty_cycle` (ns) | Mux reports |
|---|---|---|
| neutral | 1500000 | 1484 us (`1485` on the other channel) |
| +0.1 rad | 1619360 | 1640 us |
| +0.2 rad | 1738720 | 1718 us |
| +0.3 rad | 1858081 | 1875 us |
| +0.4189 rad (full left) | 2000000 | **2031 us -- `OUT_OF_RANGE`** |
| -0.4189 rad (full right) | 1000000 | 1016 us |

**Consequence, and it is a real one:** a legitimate full-left steering command produces a
2000 us pulse that the mux rounds UP to 2031 us, outside its own inclusive 1000-2000 us
validity window, so the mux flags `STEER ... OUT_OF_RANGE` and would **CUT on steering
(reason 3) at full lock while armed**. Full right (1016 us) is fine. This is a known open
item, not something to work around in software: the steering endpoints in
`config/vehicle_params.yaml` are still the provisional 1000/1500/2000 us and have to be
measured anyway (the servo already buzzes against its mechanical stop at 1200 us), and
narrowing them away from the channel ends removes this as a side effect. Until then, do not
command full lock with the mux armed.

**RESOLVED 2026-09-21 evening.** The steering endpoints were measured and narrowed to
1094/1500/1875 us (`docs/notes/bench-session-2026-09-20.md`'s steering endpoint follow-up),
comfortably inside the mux's 1000-2000 us validity window on both ends, so full lock no
longer produces an `OUT_OF_RANGE` reading or a steering cut. The same measurement also found
the sign above was backwards: a positive (LEFT) `steering_angle` had been going to the
LONGER pulse, and the table above -- from before that fix -- reads that way (+0.2 rad above
neutral). It is now the opposite: full LEFT is 1094 us, full RIGHT is 1875 us
(`config/vehicle_params.yaml`'s `steering.pwm_left_bound: "pwm_min_us"`). Commanding full
lock with the mux armed is no longer a special case; the general pre-drive caution against
untested new calibration still applies.

### Stopping

**Stop with Ctrl-C** (or `docker kill -s INT car-stack` from another shell). Not `docker stop`.
The two behave differently, and it is worth knowing which you get:

| How you stop it | What the channels do | What the mux sees |
|---|---|---|
| **Ctrl-C / SIGINT** (correct) | neutral written, then both channels **disabled** -- pulses stop | `STEER`/`THR ... STALE (stuck or stopped)`, mux cuts |
| `docker stop` (SIGTERM) | channels stay **enabled at neutral 1500 us**, pulses keep running | `STEER`/`THR 1484us FRESH`, indefinitely |

Both are safe -- neither leaves a driving pulse behind -- but only the first actually stops
the pulse train. The SIGTERM case was observed even with `exec ros2 launch` as PID 1
(2026-09-21), so do not assume `docker stop` gives you a clean shutdown.

After Ctrl-C the channels are left exported and disabled. To return the pins to a fully idle
state:

```sh
for c in 0 2; do echo 0 > /sys/class/pwm/pwmchip$c/unexport; done
```

Leave `racer-heartbeat` running. Nothing in this procedure should ever stop it.

### Every run is recorded. A drive with `record:=false` is a bug.

`car_teleop.launch.py` starts a rosbag2 recorder and `rail_voltage_node` **by default**
(GitHub issue #64, roadmap 1.6). `CLAUDE.md` invariant 5 -- "every run is logged (rosbag +
rail voltage); code paths that drive the car without logging are bugs" -- is now enforced by
the launch file, not by remembering to open a second shell. The manual `ros2 bag record`
workaround that used to live here is gone; do not reintroduce it.

**Where bags land.** One directory per run, under the `bag_dir` launch argument:

```
~/car/data/bags/2026-09-21T19-42-20_car_teleop/     # on the Jetson
/workspace/data/bags/2026-09-21T19-42-20_car_teleop # the same directory inside the container
```

`bag_dir` defaults to `/workspace/data/bags`, which is the repo's own `data/` (gitignored,
`claude-docs/02-repo-layout.md`) seen through the `-v "$PWD":/workspace` mount the start
command already has -- so bags persist on the Jetson's disk with no extra mount and no root.
The directory name is the local-time ISO timestamp plus the launch name. Pass
`bag_dir:=/somewhere/else` to record onto a USB disk; the root is created if missing, and
each run's own directory must not already exist (rosbag2 refuses, which is what keeps bags
immutable).

**What is recorded**, by regex, so a topic that does not exist yet is skipped rather than
waited for: `/drive_raw`, `/drive`, `/safety/events`, `/teleop/cmd_vel`, everything under
`/telemetry/` (the rail volts and amps), `/scan` when a LiDAR is finally fitted, plus
`/rosout` and `/parameter_events`.

**Format: mcap on the car, sqlite3 elsewhere.** `bag_storage:=auto` (the default) picks mcap
when `ros-humble-rosbag2-storage-mcap` is installed, which `docker/car/Dockerfile` does, and
sqlite3 otherwise (the `ros-dev` container, where the L3 launch test runs). The choice is
printed at startup -- the first `[invariant 5] recording ... bag to ...` line -- so it is
never a guess. Override with `bag_storage:=sqlite3` if you need to open a bag with a tool
that cannot read mcap.

**Rail voltage.** `rail_voltage_node` reads the Jetson carrier board's own INA3221 through
`/sys/bus/i2c/drivers/ina3221/*/hwmon/hwmon*/` and publishes, at 5 Hz, volts and amps in SI:
`/telemetry/rail_voltage_v` and `/telemetry/rail_current_a` for VDD_IN, plus
`/telemetry/rail/<label>/voltage_v|current_a` for every channel the chip reports. The INA226
in `claude-docs/11-hardware.md` is still not fitted; this covers the same failure mode in the
meantime. If the sysfs tree is missing the node warns once and publishes nothing rather than
failing -- **if you see that warning on the car, the bag only half satisfies invariant 5 and
the run should be treated as suspect.**

**If the recorder dies, the launch dies.** A recorder that exits for any reason (disk full,
`bag_dir` not writable, an unknown storage plugin) logs

```
[ERROR] [car_teleop]: FATAL [CLAUDE.md invariant 5]: the rosbag recorder exited ...
```

and shuts the whole launch down within about a second. `pwm_output_node` takes its normal
shutdown path on the way out -- neutral written, both channels disabled -- so the car stops
being commanded and the mux sees a stale pulse train and cuts. Nothing in the command path
consults the recorder: `/drive` is never gated on logging (that would put a logging
dependency inside safety layer 3). Fix the cause and relaunch.

**`record:=false` is for bench work only.** It exists for the pin-level checks earlier in this
runbook, where the battery is out and nothing can move. **A drive with `record:=false` is a
bug**, the same bug this section used to describe. If you find yourself reaching for it to get
past a recorder error, the recorder error is the thing to fix.

**Copying a bag to the Mac**, from the Mac:

```sh
rsync -av racer@10.0.0.226:~/car/data/bags/2026-09-21T19-42-20_car_teleop/ \
  ~/code/car/data/bags/2026-09-21T19-42-20_car_teleop/
```

`data/` is gitignored on both ends, so a copied bag never lands in a commit. Inspect it with
`ros2 bag info <dir>` inside the `ros-dev` container (an mcap bag needs the mcap plugin, which
`ros-dev` does not have -- `pip install mcap` and the `mcap` CLI, or re-record with
`bag_storage:=sqlite3`, are the two ways round that until `ros-dev` gains the plugin).

## Afterwards

- Everything measured in steps 5, 10, 11, 12, 13 and 15 goes into `config/vehicle_params.yaml` or a
  dated build-log entry. A measurement that lives only in a scrollback did not happen
  (`claude-docs/10-conventions.md`).
- Roadmap 1.3's real kill test (Jetson genuinely frozen, cut observed) is still open and is
  the next thing, not the floor.
- The throttle map stays open loop and provisional until `vesc_node` exists.
