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

Also note the throttle map's scale. `actuation.throttle_full_scale_mps` was 5.0 m/s until
2026-10-06, then 2.91 m/s, and is 2.08 m/s since the 2026-11-10 calibration (the tyre radius
had been entered as a diameter and the gear ratio was re-derived from a push test, see
docs/notes/calibration-2026-11-10.md). It is derived from the VESC's 6000 ERPM cap and the
drivetrain fields, and not yet measured against pulse width on the bench. The numbers in
the rest of this paragraph are the ORIGINAL 5.0 m/s map, kept as the record of the first
drives: with the 1000/1500/2000 us ends, **a commanded 1 m/s was 100 us off neutral: 1600 us forward, 1400 us
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

**Sensored detection, once the hall adapter is built (2026-09-29).** The motor/VESC sensor
pinouts and the straight-through adapter build are in
`docs/notes/hardware-arrival-checklist.md`'s sensor cable item and
`docs/notes/build-log.md`'s 2026-09-29 entry -- read those before wiring the SENSE port. Once
the adapter is spliced and its continuity checked (motor GND to the PH plug's "-" position,
motor +5V to the PH plug's 5V position), and with wheels off the ground and the mux knob
killed: in VESC Tool, Setup Motors FOC, select sensored, Run Detection. Expect a hall table.
"Hall detection failed" means swap any two hall wires, not re-check the pinout -- hall ORDER
does not matter, only that GND/+5V/TEMP land correctly, which the continuity check already
confirmed. At the same time, **enable motor temperature sensing in VESC Tool** (the thermistor
is on sensor pin 5; the 85 C limit is already set in the committed config). After detection
succeeds, re-export the VESC XMLs into `config/vesc/` with a new date and commit them -- not
done as of 2026-09-29, named here as the next step rather than fabricated.

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
docker exec -it <container> bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/ros_ws/install/setup.bash && ros2 run racer_tools keyboard_teleop_node --ros-args -p min_speed_mps:=0.57 -p speed_step_mps:=0.18'
```

     `speed_step_mps:=0.18` is the first-drive tap size (0.25 until 2026-11-10; every speed was 1.401 times the true one before the calibration, so the numbers were divided by 1.401 and the car does what it did) (GitHub issue #72; the rationale is
     `FIRST_DRIVE_SPEED_STEP_MPS` in `car_teleop.launch.py`). Without it the node uses the
     derived 0.19 m/s. `min_speed_mps:=0.57` (0.8 before the calibration) is the minimum commanded speed (owner-tested
     2026-09-30, `FIRST_DRIVE_MIN_SPEED_MPS`): the first tap from rest commands 0.57 m/s, and a
     throttle-down that would land below 0.57 stops the car. Without it (default 0.0) the
     minimum is disabled. Both values must be floats. The startup line prints the step and
     the minimum it is actually using.

15.4 Smallest possible speed command first: ONE tap. Confirm: the motor spins the correct
     direction, releasing the key returns to neutral within the watchdog timeout, and the
     kill switch stops it instantly at any point. With the first-drive profile ONE tap now
     commands 0.8 m/s (the minimum speed), not 0.25 m/s; see "What a keyboard tap does now"
     below for the pulse. (The smaller 0.25 m/s first tap only exists with `min_speed_mps`
     left at 0.) If the wheels do not turn on the first tap, compare the VESC
     PPM centre and deadband against `actuation.throttle_deadband_us` before touching
     anything else: the two must agree, and the fix goes in whichever one is wrong, recorded
     in the committed VESC config or in `config/vehicle_params.yaml`.

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

> **THE CAR CAN NOW REVERSE ON ITS OWN (2026-10-06 night).** gap_follow_node's reverse escape
> (`reverse_escape`, ON in the `floor-2026-10-06` profile) commands the car BACKWARDS, with the
> wheels at full lock, whenever safety_node has refused its forward request for 1.5 s and it
> sees no way forward: up to 0.4 m (commanded) or 2 s at a time, up to 3 times in a row. Nobody
> touches a key for it. Before any gap follow run, clear the space BEHIND and BESIDE the car as
> well as in front, keep feet and hands out of it, and keep the kill switch in a second
> person's hand. safety_node now judges reverse requests too (a rear corridor from the rear
> bumper, `ttc_reverse` on `/safety/events`), but `chassis.rear_overhang_m` is still a
> PROVISIONAL 0.12 m and the rear check has not been on the car yet. To run without the
> escape: `reverse_escape:=false`.

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
# Both channels: 4 ms period (4000000 ns), duty 1528662 ns (see below), enabled.
for c in 0 2; do cat /sys/class/pwm/pwmchip$c/pwm0/{period,duty_cycle,enable}; done

# What the mux actually SEES (this is the evidence that matters):
sudo stty -F /dev/ttyACM0 115200 raw -echo; sudo timeout 5 cat /dev/ttyACM0
```

**The period changed on 2026-09-21: 4000000 ns, not 20000000.** The Jetson frame is now 4 ms
(250 Hz), from `config/vehicle_params.yaml`'s `actuation.steering_pwm_period_us` /
`throttle_pwm_period_us` (GitHub issue #66). A `period` of 20000000 here means you are running
an older build; check it before reading anything else, because it changes every number below.
The servo and the ESC are unaffected either way -- the mux regenerates its own 50 Hz outputs.

**The duty is 1528662 ns, not 1500000, and that is correct (since 2026-09-29, GitHub issue
#77).** The Jetson does not emit the 4000 us frame it is asked for: it emits about 3925 us,
which scaled every pulse about 1.9 percent short (1500 read 1475). pwm_output_node now
pre-scales each duty by requested / achieved (`actuation.*_pwm_achieved_period_us` in
`config/vehicle_params.yaml`), so it writes 1500 x 4000 / 3925 = 1528.662 us of duty to get
1500 us on the wire. The startup log prints the same thing in its
`pwm frame compensation (issue #77)` line, including the neutral reading the mux should
report (1502.5 us). A duty of exactly 1500000 means an older build.

Expect `STEER gp10=1500us FRESH(...)` and `THR gp7=1500us FRESH(...)` within 8 us (the model
predicts 1502 to 1503). **1475 means the compensation is not running** (older build, or the
achieved period in vehicle_params equals the requested one); 1484 means a 20 ms frame. With the transmitter still off you will also see
`KILL ... NO_EDGES` and `DECISION=CUT reason=1:RC_SIGNAL_INVALID`; that is expected and is
exactly what you want before arming.

Only one process may read `/dev/ttyACM0` at a time -- a second reader steals the bytes.

### Bench verification of the 4 ms frame (do this once, before the next drive)

**RUN 2026-09-29, and it FAILED step 3 in an instructive way:** the grid did get finer, but
every reading was about 1.7 percent short (1500 read 1475, 1652 read 1620 to 1624) while
`period` read back 4000000. That is GitHub issue #77, fixed by the frame compensation; verify
that fix with the next section, which supersedes step 3's table below. Steps 1, 2, 4 and 5
still apply as written.

UNVERIFIED AS OF 2026-09-21: the arithmetic below is exact, but nobody has yet put the new
frame in front of the Pico. Run this with the wheels off the ground, the battery OUT, the
servo lead unplugged and the VESC unpowered -- it is a pin-and-diagnostic check, nothing
needs to move. (Battery out means no bag is required for this one: start the stack with
`record:=false`, per "Every run is recorded" below.)

1. Start the stack as above, with no teleop source, and confirm the startup log's frame line:
   `pwm frame: steering 4000 us (250.0 Hz, 15.625 us pulse grid), throttle 4000 us ...`.
2. `for c in 0 2; do cat /sys/class/pwm/pwmchip$c/pwm0/period; done` must print `4000000`
   twice. If it prints `20000000`, stop: the node did not take the configured period.
3. Read the Pico diagnostic (`sudo timeout 5 cat /dev/ttyACM0`) and check three commanded
   pulses. Command them through the normal gated path, not by writing sysfs by hand:

   | Commanded pulse | Nearest 15.625 us grid point | Acceptable mux reading |
   |---|---|---|
   | 1500 us | 1500.000 (96 steps, exact) | 1500 +- 8 us |
   | 1600 us | 1593.75 / 1609.375 | 1594 or 1609, and nothing further than 8 us from 1600 |
   | 1700 us | 1703.125 (109 steps) | 1703 +- 8 us |

   The old 78.125 us grid could only produce 1484, 1562 and 1719 for those three. Seeing any
   of those three numbers means the frame did not change.
4. Confirm the mux still REGENERATES a **50 Hz pulse** to the servo and the ESC. Two places
   on the diagnostic, and both must hold:
   - the first line's `OUT servo gp1=1500us esc gp3=1500us` -- what the mux decided to emit;
   - the `PWMREG` line under it -- what the RP2040's registers are actually emitting:
     `servo gp1 ... frame=20000.0us 50.00Hz` and the same for `esc gp3`.

   This is the whole safety argument for the change: the Jetson frame moved, the mux's output
   frame did not. If that `frame=` reads anything but about 20000.0 us / 50 Hz, stop and do
   not arm.
5. Confirm the decision is still `DECISION=CUT reason=1:RC_SIGNAL_INVALID` with the
   transmitter off, and that both channels read `FRESH` rather than `STALE`: a 4 ms frame
   refreshes the capture five times as often, so staleness must be further away than before,
   never closer.

Record the readings in `docs/notes/build-log.md` when this is run.

### Bench verification of the frame compensation (GitHub issue #77; do this once, before the next drive)

UNVERIFIED: written 2026-09-29 from the bench readings that found the problem, not yet run
against the fix. Same conditions as the section above: wheels off the ground, battery OUT,
servo lead unplugged, VESC unpowered, `record:=false`, no teleop source. Nothing needs to move;
the acceptance is read off the mux diagnostic.

1. Start the stack and confirm the startup log has the line
   `pwm frame compensation (issue #77): steering achieved 3925.0 us (measured), neutral 1500 us
   written as duty 1528662 ns, predicted at the mux 1502.5 us; throttle achieved 3925.0 us ...`.
   No such line means an older build: stop.
2. `for c in 0 2; do cat /sys/class/pwm/pwmchip$c/pwm0/{period,duty_cycle}; done` must print
   `4000000` and `1528662` for each channel. The period is still the REQUESTED 4000000; only
   the duty is compensated.
3. Command these pulses through the gated path (not by writing sysfs), and read the mux
   (`sudo timeout 5 cat /dev/ttyACM0`). The throttle channel is the easiest to drive to an exact
   pulse: publish `/drive_raw` with `ros2 topic pub -r 50 /drive_raw
   ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: S}}"` for the speed in the table
   (the deadband offset is included in the speeds below), Ctrl-C after each reading:

   | Commanded pulse (throttle) | Speed `S` to publish | duty_cycle written | Model predicts at the mux | ACCEPT if the mux reads |
   |---|---|---|---|---|
   | 1500 us (neutral) | 0.0 | 1528662 | 1502.5 | 1492 to 1508 |
   | 1600 us | 0.5556 (= (1600 - 1550) x 5 / 450) | about 1630577 | 1594.5 | 1592 to 1608 |
   | 2000 us | 5.0 | 2038217 | 1993.2 | 1992 to 2000, and `THR` NOT `OUT_OF_RANGE`; the mux forwards it as read |

   And on steering, with speed 0: `steering_angle` 0.0 must read 1500 +- 8 us. Since schema
   0.14.0 the angle goes through the per-side table (`steering.pwm_to_angle_table`), so full
   left (+0.449, the planning limit, which stops short of the 1094 us left stop) predicts 1134.5
   us and full right (-0.449) predicts 1870.5 us, each within 8 us. (Before 0.14.0: +0.4189 read
   1088.6 us and -0.4189 read 1870.5 us, against the 1094 / 1875 stops.)
4. **If a reading is outside its band, re-derive the achieved period from it; do not tune
   around it.** For a reading of R us with the written duty D ns and requested period P ns
   (4000000): the controller's count is `c = round(256 x D / P)` and the achieved period is
   `R x 256 / c` us. Example: D = 1528662 gives c = 98, so a reading of 1495 us means
   1495 x 256 / 98 = 3905.3 us. Take it from at least two pulses far apart (1500 and 2000),
   average, write it into `actuation.steering_pwm_achieved_period_us` /
   `throttle_pwm_achieved_period_us` with the date and readings in the comment, bump
   `meta.schema_version`'s patch number, rebuild, and repeat this section.
5. **Only once this passes on both channels:** the VESC centre goes back to 1.500 ms and its
   PPM deadband shrinks to about 4 percent (20 us), and `actuation.throttle_deadband_us` must be
   changed to match in the same session (see "What a keyboard tap does now" below and
   `docs/notes/build-log.md` 2026-09-29). Until then leave the VESC at centre 1.4875 ms /
   deadband 10 percent: it is what makes the two neutrals the ESC can see (the Jetson's and
   the mux's CUT output of exactly 1500 us) both fall inside the deadband.

Record the readings in `docs/notes/build-log.md`.

### What a keyboard tap does now (2026-09-30)

With the first-drive profile (`min_speed_mps:=0.57 -p speed_step_mps:=0.18`, see step 15.3 or
`car_teleop.launch.py`'s `FIRST_DRIVE_MIN_SPEED_MPS` / `FIRST_DRIVE_SPEED_STEP_MPS`), the
rule is: from rest, W commands max(0.57, 0.18) = 0.57 m/s; above that W adds 0.18; S subtracts
0.18, and if the result would be below 0.57 it goes to exactly 0 (a clean stop, no crawl
below the minimum). SPACE and `q` are unchanged (zero). Speeds between 0 and 0.57 are never
commanded. Pulse arithmetic uses the deadband offset from `actuation.throttle_deadband_us`
(see `config/vehicle_params.yaml` for the current value) and is checked in racer_drivers'
gtests; speed 0 is always exactly 1500 us.

| Key presses (from rest) | Commanded speed |
|---|---|
| none | 0.0 m/s (1500 us exactly) |
| W | 0.80 m/s |
| W W | 1.05 m/s |
| W W W | 1.30 m/s |
| W W W, then S | 1.05 m/s |
| W, then S | 0.0 m/s (0.55 would be below the minimum, so it stops) |
| W W, then S S | 0.0 m/s (1.05, 0.80, then 0) |
| S from rest | 0.0 m/s (reverse is off by default) |

Holding W still ramps, one step per key-repeat event. With `allow_reverse:=true` the same
shape holds on the negative side for |speed|: S from rest commands -0.8, and crossing from
forward to reverse always passes through 0 (W at -0.8 stops; it does not jump to +0.x).
`min_speed_mps` must be >= 0 and <= the maximum speed (and <= the reverse limit when reverse
is allowed) or the node refuses to start. The older 0.25 m/s-per-tap ladder of 2026-09-29
(first tap 0.25 m/s) applies only when `min_speed_mps` is left at its default 0.0.

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

### Gap follow from the launch file (LiDAR; checkpoint profile of 2026-10-06)

gap_follow_node drove its first working floor laps on 2026-10-06; that parameter set is the
`floor-2026-10-06` profile of `racer_control`'s `gap_follow.launch.py` (table and known limits
in `docs/notes/reactive-control-port-2026-10-05.md`, "Checkpoint 2026-10-06"). Same pre-drive
checklist as above, kill switch in a second person's hand.

**WITH THIS PROFILE THE CAR REVERSES ON ITS OWN** (the reverse escape, see the box at the top of
"Launch and drive" and `docs/notes/reactive-control-port-2026-10-05.md`, "Reverse escape").
Clear the space behind the car as well as ahead. The profile also turns on lane centring.
Neither has been on the floor yet; `reverse_escape:=false centering_gain:=0` (plus
`speed_time_constant_s:=0 target_range_median_scans:=1`) gives exactly the parameters of the
first laps.

1. Start the stack with the LiDAR and NO teleop source: in "Start the stack" above, end the
   command with `exec ros2 launch racer_bringup car_teleop.launch.py lidar:=true` (no
   `browser_teleop:=true`). gap_follow_node publishes `/drive_raw`, and nothing arbitrates
   between two publishers of it, so no keyboard or browser teleop while it runs. Recording
   stays on (the default); the bag picks up `/scan`, `/drive_raw` and `/drive`.
2. In a second terminal, into the same container:

   ```sh
   docker exec -it car-stack bash -lc '
     source /opt/ros/humble/setup.bash && source /workspace/ros_ws/install/setup.bash
     exec ros2 launch racer_control gap_follow.launch.py profile:=floor-2026-10-06'
   ```

   The startup line `gap_follow_node up: ...` must show `lane centring gain 0.600`, `max speed
   0.90 m/s, speed time constant 0.500 s, target range median over 5 scans`, and the next line
   must be the WARN `REVERSE ESCAPE ON: this node will command the car to REVERSE on its own
   ...` (or `reverse escape off` with `reverse_escape:=false`). If they do not, the workspace
   was not rebuilt after pulling (step 6). Every escape logs `reverse escape N/3: ...` when it
   starts and `complete`, `timed out` or `aborted` when it ends.
3. Any launch argument given explicitly overrides the profile, for example
   `profile:=floor-2026-10-06 max_speed_mps:=0.5`. Leave `laser_yaw_from_vehicle_params` alone:
   on the car the LiDAR yaw comes from `config/vehicle_params.yaml`.
4. Stop with Ctrl-C in the second terminal. gap_follow_node then stops publishing and
   safety_node brakes on the `/drive_raw` silence; the kill switch is still the first thing to
   reach for if the car does anything unexpected, including backing up when you did not expect
   it.

### Safety floor test, body-distance gate (vehicle_params 0.14.0; NOT yet run on the car)

Since 2026-11-02 safety_node measures every obstacle distance from the swept body (front bumper,
or the leading corner on an arc) instead of the LiDAR head, and raises the TTC speed to
`/odom/wheel` when the car is faster than its request. Since 2026-11-10 (schema 0.14.0, the
calibration in `docs/notes/calibration-2026-11-10.md`) it brakes at TTC 0.60 s on that distance
(release 0.71 s) with a 0.10 m floor, in TRUE speeds: the tyre radius and gear ratio were wrong,
so every speed in the software was 1.40 times the truth until then. gap_follow_node escapes after
5 s of refusal even when it thinks it sees a way through, and its timer no longer waits for the
scan callback. Why, and the bag replay behind the numbers:
`docs/notes/safety-floor-review-2026-11-02.md` and the 2026-11-10 note. Do these in order before
the first drive with it.

0. **Same request, 1.40 times the speed.** A follower speed request is now the true straight-line
   wheel speed. The October floor runs at "0.8 m/s" were 0.57 m/s on the floor; the same
   `max_speed_mps` now makes the car 1.40 times faster than it was then (through corners it is
   slower than the request, the wheels slip). Lower the follower's `max_speed_mps` by that
   factor first if you want the October pace, and confirm the calibration once on the bench or the
   floor before any run: push the car along a taped 3.00 m with the throttle killed and read the
   along-track distance of `/odom/wheel` (`pose.pose.position.x`, the change over the push): it
   must be 3.00 m, give or take 0.1 m (it read 4.14 and 4.27 m with the 0.13.0 values).
1. **The overhangs are measured (0.11 and 0.14 m, 2026-11-10).** The brake point moves one for one
   with them. Re-measure only if the bumper changes. If the front one differs from 0.11 m by more
   than 1 cm, re-run the replay with the new value before driving: `uv run --project tools python
   tools/replay_safety.py --recorded-with 0.13.0 --before-rule october <the October bags> --set
   front_overhang_m=<value>` and compare with the table in the 2026-11-10 note.
2. **Build optimised.** The 2026-10-25 stutter came from a scan callback that took 20 to 50 ms on
   the Jetson; the code is now much cheaper and threaded, but build the workspace with
   `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release` (step 6 builds without
   a build type, which compiles without optimisation).
3. **Start the stack with the VESC telemetry**, so `/odom/wheel` is used by the gate and recorded:
   `ros2 launch racer_bringup car_teleop.launch.py lidar:=true vesc:=true`. The `safety_node up:`
   line must say `distances from the swept body, TTC speed bound decaying at 1.00 m/s^2, wheel
   speed from /odom/wheel while fresh (0.10 s)` and `ttc_brake_s=0.600000`. Without the VESC
   UART the gate still works on the request alone.
4. **Wheels off the ground, a box 0.30 m ahead of the bumper:** with the gap follower running at
   its crawl the wheels must stop (one `ttc` brake engage), and must restart when the box is
   moved to 0.6 m. With the box beside the front corner (inside the 5 cm side margin) and the
   steering pointing away from it, the wheels must turn.
5. **Floor, both directions,** profile `lane-2026-10-25` (PR 111, the lane_centre apex fix, never
   driven): `ros2 launch racer_control gap_follow.launch.py profile:=lane-2026-10-25`. The startup
   WARN must now say `... or for 5.00 s whatever the arc`. Kill switch in a second person's hand.
6. **What to look for, then check with the bag.** Brakes should come earlier on the approach to a
   wall than in October and end clear of the foam. A brake on the entry to a corner at 0.7 m/s or
   more is expected (the 2026-11-10 note counts the new brakes the replay found on approaches the
   car drove freely, and they are all at those speeds): if those feel like cut-offs, lower
   `max_speed_mps`, do not lower the TTC. A car refused for 5 s backs out and
   logs `reverse escape N/3: ... although the probe sees a clear forward arc`. Copy the bag off and
   run `tools/replay_safety.py --recorded-with current` on it (`0.13.0` is for the bags up to
   2026-11-10 only): the `end (m)` column is the real end clearance measured from
   the scans (want 0.04 m or more on every brake), `watchdog engages: recorded` should be near
   zero (it was 186 on 2026-10-25), and with `/odom/wheel` in the bag the stopping model
   (`actuation.stop_latency_s`, `stop_deceleration_mps2`) can be re-derived from real speeds.

### Parallel park and three-point turn (park_node; NOT yet run on the car)

`racer_control`'s park_node (roadmap 2.9, 2026-10-07) finds a slot along a row of obstacles
with the LiDAR and parallel parks in it, or turns the car round in its lane, from the VESC's
wheel odometry. **IT REVERSES ON ITS OWN.** Everything, including the prerequisites (VESC UART
wired and `/odom/wheel` checked in "VESC telemetry" below, both overhangs measured, the turning
radius checked, an L6 sweep), the commands and what to expect in the log, is in
`docs/notes/parking-2026-10-07.md`, "Running it on the car". In short: the stack with
`lidar:=true vesc:=true` and no teleop source, then
`ros2 launch racer_control park.launch.py profile:=floor-2026-10-07`, then
`ros2 service call /park_node/start std_srvs/srv/Trigger` (or `/park_node/three_point_turn`;
`/park_node/abort` stops it). Kill switch in a second person's hand.

### The throttle start deadzone (expect this, it is not a fault)

**HISTORICAL, 2026-09-21 (sensorless, Current No Reverse With Brake).** Since 2026-09-29 the
motor is sensored (hall adapter fitted, detection done) and the VESC runs centred PID Speed
Control, so a start no longer needs 1700 us: see "What a keyboard tap does now" above. The
record below is kept because it is the measurement the change was made against.

This drivetrain is **sensorless**, and it needs roughly **1700 us** (about 40 percent of the
throttle range) to start turning from rest. Measured on the bench 2026-09-21: 1560 us did
nothing, 1600 and 1650 us made the rear tyres click for a few seconds without turning, 1700
and 1750 us spun them up.

On the open-loop map as it was at the time (`actuation.throttle_full_scale_mps: 5.0`, so 1 m/s
= 100 us off neutral; 2.91 since 2026-10-06, so 1 m/s = 165 us) that meant:

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

### Reading the mux numbers (why 1500 used to read as 1484)

**SUPERSEDED 2026-09-21 by the frame-period change (GitHub issue #66); the table below is the
20 ms record, kept because it is the measurement the change was made from.** The coarse grid
in it was never the Pico's: it was the JETSON's. The Tegra PWM controller has 8-bit duty
resolution (`pwm-tegra.c`, `PWM_DUTY_WIDTH 8`), so a commanded pulse was quantised to
period/256 = 78.125 us at the 20 ms frame -- about 13 positions across the whole 1000-2000 us
range. With the frame now 4000 us the step is 15.625 us and a commanded 1500 us is an exact
grid point, so it should read 1500, not 1484. See "Bench verification of the 4 ms frame"
above, and `ros_ws/src/racer_drivers/README.md`'s "Actuator resolution" for the arithmetic.

The DIAG_BUILD firmware measures pulse width on a **15.625 us grid** and reports the nearest
grid point. Every reading below is an exact multiple of 15.625 us. Measured 2026-09-21 at the
OLD 20 ms frame, commanded value from sysfs against what the Pico reported:

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
`/telemetry/` (the rail volts and amps), `/scan` when the launch runs with `lidar:=true`
(see "LiDAR first power-up" below), `/odom/wheel`, `/telemetry/vesc/*` and `/vesc/sensors/core`
when it runs with `vesc:=true` (see "VESC telemetry" below), `/camera/<name>/record/compressed` (the
small drive-only stream, never the full-rate or raw images) when cameras run (see "Cameras first power-up" below), plus `/rosout` and
`/parameter_events`.

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

**Replaying a bag through the gap follower** (what would it have steered on these scans, how
often did the swept-path clamp and corner override fire, how many brakes and how long the
latches were): `tools/replay_follower.py`, which reads mcap directly, including a bag whose
recorder was killed and has no footer. Run it on the Mac against the copied bag, never on the
Jetson: `docs/notes/replay-follower.md`.

## LiDAR first power-up (roadmap 2.3; UNVERIFIED, written 2026-10-05 before the C1 arrived)

**Nothing in this section has been run.** It is the procedure for the RPLIDAR C1's first
session on the Jetson, written from the driver source and the datasheet. The car does not move
in any step: no battery is needed, the VESC and servo can stay unpowered, and the LiDAR is
started on its own, not with the teleop stack.

What it proves when it passes: the C1 enumerates, the driver in the car image talks to it,
`/scan` arrives at the datasheet rate with the expected shape, and the `base_link -> laser`
transform points the right way. What it does not prove: the mount values (still PROVISIONAL
in `config/vehicle_params.yaml` until measured), LiDAR timing against the other sensors
(roadmap 2.2), or anything about driving with the LiDAR.

### L1. Before you plug it in: rebuild the image and the workspace

The `sllidar_ros2` driver is a new layer in `docker/car/Dockerfile`, and `lidar.launch.py` /
`lidar_check` are new in `ros_ws`. On the Jetson, from `~/car`, after pulling this branch:

```sh
docker build -t car:local docker/car
docker run --rm -v "$PWD":/workspace -w /workspace car:local bash -lc '
  source /opt/ros/humble/setup.bash
  apt-get update && rosdep install --from-paths ros_ws/src --ignore-src -r -y
  cd ros_ws && colcon build --symlink-install
  chown -R 1000:1000 build install log /workspace/tools/.venv'
docker run --rm car:local bash -lc 'source /opt/ros/humble/setup.bash && ros2 pkg prefix sllidar_ros2'
# expect: /opt/racer_thirdparty
```

### L2. Plug it in and confirm the serial device

The C1 takes 5 V at about 260 mA from one Jetson USB-A port through its CP210x adapter (check
the adapter is in the box; some listings ship the bare head). Plug the adapter into the head
first, then into the Jetson. On the Jetson host:

```sh
lsusb | grep -i 10c4:ea60            # expect: Silicon Labs CP210x UART Bridge
sudo dmesg | tail -n 5               # expect: cp210x converter now attached to ttyUSB0
ls -l /dev/ttyUSB*                   # expect: crw-rw---- 1 root dialout ... /dev/ttyUSB0
getent group dialout                 # note the GID (20 on stock Ubuntu)
```

The head's motor should spin up as soon as it has power. If there is no `ttyUSB0`: `lsmod |
grep cp210x` (and `sudo modprobe cp210x` if the module is not loaded; if the module does not
exist in this JetPack kernel, stop and record that, it is a finding). If `ttyUSB0` appears and
then vanishes a second later in `dmesg`, something else grabbed it (on Ubuntu desktop images
`brltty` is the usual culprit).

Optional but recommended once another USB serial device (the ingest board) is around: install
the udev rule so the LiDAR keeps a stable name, and use `/dev/lidar` everywhere below instead
of `/dev/ttyUSB0`:

```sh
sudo cp tools/udev/99-racer-lidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/lidar                     # expect: /dev/lidar -> ttyUSB0
```

### L3. Start the driver on its own

From `~/car`. Same unprivileged shape as "Start the stack" above, minus the PWM mounts, plus
the serial device and the `dialout` group:

```sh
cd ~/car
docker run --rm -it --name car-lidar --network host \
  --user "$(id -u):$(id -g)" \
  --device /dev/ttyUSB0 \
  --group-add "$(getent group dialout | cut -d: -f3)" \
  -e HOME=/tmp \
  -v "$PWD":/workspace -w /workspace/ros_ws \
  car:local bash -lc '
    source /opt/ros/humble/setup.bash && source install/setup.bash
    exec ros2 launch racer_bringup lidar.launch.py'
```

With the udev rule: `--device /dev/lidar` and `... lidar.launch.py serial_port:=/dev/lidar`.
The other launch arguments (`serial_baudrate` 460800, `frame_id` laser, `angle_compensate`
true, `scan_mode` Standard) default to the C1 values from Slamtec's own C1 launch file.

Expect, from `sllidar_node`, the device's model, firmware and serial number, a health line,
then:

```
current scan mode: Standard, sample rate: 5 Khz, max_distance: <about 12> m, scan frequency:10.0 Hz
```

`scan frequency` there is not measured: it is `sensors.lidar_spec.nominal_scan_rate_hz` handed
to the driver, which uses it to size its 720-beam array. `max_distance` is what the head
reports for the mode; record it. If the launch dies with `lidar.launch.py: ... sensors.lidar
has null ...`, the mount fields in `config/vehicle_params.yaml` were blanked: fill them in, do
not work around it. `Error, cannot bind to the specified serial port` means the device or the
group did not reach the container: re-check `--device` and `--group-add`.

### L4. Run the scan-rate check

In a second shell on the Jetson, inside the same container:

```sh
docker exec -it car-lidar bash -lc '
  source /opt/ros/humble/setup.bash && source install/setup.bash
  ros2 run racer_tools lidar_check'
```

It listens to `/scan` for 10 s (`--ros-args -p duration_s:=30.0` for longer), prints a report
and exits 0 on PASS, 1 on FAIL, 2 if fewer than two scans arrived. Expected for the C1 with
the defaults (thresholds come from `sensors.lidar_spec`, not from this text):

| Line | Expect | Fails when |
|---|---|---|
| scan rate | 8 to 12 Hz, about 10 Hz typical | below 8.000 Hz (`min_scan_rate_hz`) |
| largest stamp gap | about 0.1 s | (reported only; a long gap is a dropped scan) |
| beam count | exactly 720 every scan | anything else (`angle_compensate` true) |
| field of view | 6.2832 rad (2 pi) | outside 6.2832 +- 0.0126 rad |
| declared range band | 0.050 .. `max_distance` from L3 | (reported only) |
| observed valid ranges | nearest and farthest real returns in the room | no valid return at all |
| invalid fraction | low indoors (glass, black surfaces and open space past range add to it) | above 0.5 (`max_invalid_fraction`) |

A cross-check that does not use this repo's code: `ros2 topic hz /scan` in the same container
should agree with the rate line. Paste the whole report into the build log entry for this
session.

### L5. Look at it in Foxglove, and check the transform points the right way

`lidar.launch.py` does not start `foxglove_bridge`. Start one in the same container:

```sh
docker exec -it car-lidar bash -lc '
  source /opt/ros/humble/setup.bash && source install/setup.bash
  exec ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765'
```

In Foxglove on the Mac: open a connection to `ws://10.0.0.226:8765`, add a 3D panel, set its
display frame to `base_link`, and turn on `/scan` (and `/tf_static`, which carries
`base_link -> laser`). The room's walls should draw as a ring of points around the car.

Then the orientation check, which is the point of this step: put a box on the floor about
0.5 m **straight ahead** of the car's nose.

- The box shows up on the **+x** side of `base_link` (the red axis): the yaw is right.
- It shows up **behind** the car: the head is mounted rotated by half a turn relative to what
  `mount_yaw_rad` says. Set `sensors.lidar.mount_yaw_rad` to `3.141593` and repeat. The
  driver itself already rotates its scan by pi from the head's 0 degree mark
  (`sllidar_node.cpp`, `publish_scan`), so either answer is plausible before this check.
- It shows up **to the side**, or a box placed to the car's LEFT shows up on its right: stop.
  That is a mirrored scan (head upside down, or the wrong rotation sense), not a yaw error, and
  must not be "fixed" with the yaw. Record it in the build log.

While the 3D panel is up, look at the closest returns: anything that is part of the car
(mount posts, the Jetson, cables, antenna) shows as points within a few tens of centimetres.
Note how close the nearest self-return is. **That number matters before any drive with
`lidar:=true`**. Since 2026-10-06 (`docs/notes/ttc-limit-cycle-2026-10-06.md`) `safety_node`'s
obstacle gate works like this:

- It looks only at returns in the car's path: a corridor `chassis.width_m` / 2 +
  `limits.obstacle_corridor_margin_m` (0.155 + PROVISIONAL 0.05 = 0.205 m) each side of the
  path, since vehicle_params 0.9.0 ("corridor, not wedge": on the first floor test a bag 0.3 m
  beside the path braked the car like one straight ahead). Since vehicle_params 0.9.2 ("arc
  corridor", late floor test) the path BENDS with the REQUESTED steering: straight ahead for a
  straight request, otherwise the arc the car would drive at that steering (from
  `chassis.wheelbase_m`, `steering.max_angle_rad` and the LiDAR mount), up to a quarter turn.
  It is re-judged on every gate cycle with the current request, so a car latched on a wall
  releases as soon as the planner (or the operator) steers away from the wall and that arc is
  clear, even while the steering hold below has frozen `/drive`'s steering. Before 0.9.2 the
  corridor stayed straight whatever the steering, and on the floor a car at full lock away from
  a wall stayed latched for 20 to 37 s. Each laser bearing is first turned into a vehicle
  bearing with `sensors.lidar.mount_yaw_rad` (pi on this car). The forward sector of +/-
  `limits.ttc_forward_sector_half_angle_rad` (PROVISIONAL 1.2 rad since 0.9.1, about 69 deg
  each side) is kept as an outer bound only, so nothing behind or far beside the car is ever
  considered. Returns that are NaN, inf, zero, below `range_min` or above `range_max` are
  ignored. The person holding the kill switch behind the car does not count.
- The distance used below is the distance along the path from the LiDAR head to the return
  (along-track for a straight request, arc length on a turn), not its straight-line range.
- TTC is the nearest in-path distance divided by the REQUESTED speed (not the speed the gate
  last output). At or below `limits.ttc_brake_s` (PROVISIONAL 0.35 s) it zeroes the forward
  throttle; at the 0.8 m/s first-tap speed that is anything within 0.28 m ahead.
- A distance floor, `limits.min_forward_clearance_m` (PROVISIONAL 0.20 m from the LiDAR head),
  zeroes the forward throttle for any forward request, however slow.
- Both LATCH. Once braked, forward throttle stays at zero until the request's TTC is above
  `limits.ttc_warning_s` (PROVISIONAL 0.36 s since 0.9.2, just above the brake, 0.29 m at
  0.8 m/s) AND the nearest return in the path is beyond 1.5 times the floor (0.30 m). Letting go of the throttle can clear the TTC half
  (a zero request has no TTC), but pressing it again re-trips on the same cycle, so no forward
  throttle reaches the motor while the obstacle is still there. Reverse still works while
  latched.
  `/safety/events` shows one `ttc` BRAKE engage record when it trips and one release record
  starting "ttc brake released" when it clears.
- Steering stays live while the car brakes, so it can still be steered while it coasts. Once
  the latch has held `/drive` speed at zero for `limits.obstacle_steering_hold_after_s`
  (PROVISIONAL 0.5 s, since vehicle_params 0.8.0), the steering is FROZEN at the angle it had
  then, until the latch releases: a car parked against an obstacle stops hunting its servo
  however the planner's steering request wanders. Reversing while latched unfreezes it. Expect
  one `ttc` INFO engage record "steering held while obstacle-latched" (with the held angle) and
  one "steering hold released" record when the latch clears. On the stand: about half a second
  after `/drive` goes to zero the servo should stop moving, and it should follow the request
  again as soon as the obstacle is moved away.

So a self-return inside the path and closer than about 0.30 m would hold the car at
zero throttle indefinitely. That is why `car_teleop.launch.py`'s `lidar` argument defaults to
false.

### L6. Measure the mount and write everything down

With the mount on the car: measure the head's optical centre from the `base_link` origin
(x forward, y left, z up, metres) and its yaw from L5, put them in `config/vehicle_params.yaml`
`sensors.lidar` replacing the PROVISIONAL placeholders (bump `meta.schema_version`'s patch
number), and record in a dated `docs/notes/build-log.md` entry: the `lidar_check` report, the
driver's startup lines (model, firmware, `max_distance`), the nearest self-return, and which
way the yaw came out. Then roadmap 2.3 can be ticked.

To record a bag of the LiDAR with the rest of the stack, start "Start the stack" above with
the two extra flags (`--device ...` and `--group-add` for `dialout`) and `lidar:=true` (plus
`lidar_serial_port:=/dev/lidar` with the udev rule). The recorder's regex already includes
`/scan`.

## VESC telemetry (wheel odometry over the VESC UART; UNVERIFIED, written 2026-10-07)

**Nothing in this section has been run on the car.** The software side (the f1tenth
`vesc_driver` read-only patched in the car image, `racer_drivers/vesc_odometry_node`,
`vesc_telemetry.launch.py`) was built and exercised in the `ros-dev` container on 2026-10-07
against a fake VESC on a pseudo-terminal; no real VESC, no Jetson UART, no wiring.

What it gives you: `/odom/wheel` (`nav_msgs/Odometry`: `twist.twist.linear.x` = wheel speed in
m/s, positive forward; `pose.pose.position.x` = SIGNED along-track distance in metres since the
node started, forward adds and reverse subtracts, NOT an x coordinate; pose covariance 1e6 so
nothing fuses it as a pose; frames `odom` / `base_link`, no TF) and `/telemetry/vesc/`
`voltage_v`, `current_motor_a`, `current_input_a`, `temp_fet_degc`, `temp_motor_degc`, `erpm`
(raw, the non-SI driver-boundary value) and `fault` (a name such as `NONE` or `OVER_TEMP_FET`,
or `STALE_NO_VESC_DATA` while nothing arrives). Speed is ERPM / `drivetrain.pole_pairs` /
`drivetrain.gear_ratio` x 2 pi x `tires.nominal_radius_m`, from the generated binding only; with
today's gear ratio (8.32, derived from a push test) and measured tyre radius (0.055 m) that is
0.000346 m/s per ERPM, so the VESC's 6000 ERPM cap reads 2.08 m/s, the same number as
`actuation.throttle_full_scale_mps`. (With the 0.13.0 values it read 2.91 m/s, 1.40 times the
truth.)

**It is read-only, and it must stay that way.** The motor is commanded ONLY by the PPM pulse
through the layer-1 mux (`CLAUDE.md` invariant 1). Upstream `vesc_driver` subscribes six
command topics and forwards them to the VESC over this same serial line; the car image builds
it with `docker/car/patches/vesc_driver-readonly.patch`, which removes those subscriptions and
makes every set-command a no-op, and the image build refuses if the patched source can still
subscribe or send one. The driver only ever asks the VESC for its firmware version and its
state (`COMM_FW_VERSION`, `COMM_GET_VALUES`, 50 Hz). `vesc_ackermann` is not built. Do not
"fix" a telemetry problem by installing the unpatched driver or the apt/ROS 1 one.

**Roadmap note (2.1 / 2.4 / 2.9).** Roadmap 2.1 plans wheel-speed sensors on the ingest board.
The VESC's sensored motor already measures driven-wheel speed (through the drivetrain, all four
wheels are driven on the Slash 4x4), so `/odom/wheel` can be the wheel-speed input of 2.1 and
2.4 instead of a separate sensor, at least until slip measurement needs per-wheel speeds. It is
also the "how far have I moved" signal for 2.9's parking and three-point turn. The VESC keeps
its own timeline (`claude-docs/11-hardware.md` "Sensor sync"): the stamp on `/odom/wheel` is the
Jetson's receive time in the driver, so 2.2 still has to measure that offset.

### V1. VESC Tool: turn on the UART app without touching PPM

Over USB, as in step 13, wheels off the ground, mux knob killed:

1. **App Settings > General > App to Use: `PPM and UART`** (it is `PPM` today:
   `app_to_use` 1 in `config/vesc/2026-09-29d-fsesc67-app.xml`; `PPM and UART` is 4). This
   keeps the PPM input exactly as configured (control type, deadband, ranges are unchanged) and
   additionally listens on the UART. Do NOT pick `UART` alone: that turns PPM off and the car
   can no longer be driven.
2. **App Settings > UART > Baudrate: 115200** (`app_uart_baudrate`, already 115200 in the
   committed export). It must equal the launch argument `baud` (default 115200).
3. Write the app configuration, then re-export it into `config/vesc/` with a new date
   (`2026-10-07-fsesc67-app.xml` or the date you do it) and commit it: the committed export is
   safety layer 2 and the L6 config-diff check diffs against it. Change nothing in the motor
   configuration.
4. Check PPM still works before anything else: arm the mux, one keyboard tap, wheels turn,
   release, they stop. If they do not, put App to Use back to `PPM` and stop.

The same two settings apply unchanged to the FSESC 4.12 when it replaces the 6.7.

### V2. Wiring: three wires, 3.3 V logic, never the VESC's 5 V

| Jetson Orin Nano 40-pin header | Wire | VESC COMM (UART) port |
|---|---|---|
| pin 8, UART1_TXD (3.3 V) | crosses to | RX |
| pin 10, UART1_RXD (3.3 V) | crosses to | TX |
| pin 6 (or 9, 14, 20, 25, 30, 34, 39), GND | straight | GND |
| (nothing) | | 5 V: **leave unconnected** |
| (nothing) | | 3.3 V: leave unconnected |

TX goes to RX and RX to TX. On JetPack 6 header pins 8/10 are `/dev/ttyTHS1` (they were
`ttyTHS0` on JetPack 5; NVIDIA developer forum, "How to access UART1 on expansion pins 8/10").

**FSESC 6.7 COMM pinout: UNVERIFIED.** On VESC 6-derived boards the COMM port is a small
JST-PH socket that carries the UART's TX and RX (3.3 V logic, per VESC forum reports) next to
supply pins. No source found on 2026-10-07 (Flipsky's manual pages were not readable, forum
threads give no pin order) states the FSESC 6.7's pin order, so none is written here as fact.
Before connecting anything to the Jetson:

- Read the pin names off the board's silkscreen next to the COMM socket (and the pigtail
  Flipsky ships with it). Photograph it and commit the photo to `docs/notes/`.
- With the VESC powered from the pack and the Jetson NOT connected, measure each candidate pin
  to GND with a meter. GND: continuity to battery negative. 5 V and 3.3 V: steady 5 V / 3.3 V
  (these are the two you must not connect). VESC TX: idles high at about 3.3 V once the UART
  app is enabled. **If the pin you take for TX reads above 3.6 V, stop**: the Orin's header
  pins are 3.3 V and not 5 V tolerant.
- Only then make the three-wire lead. Keep it short and away from the motor phase wires.

The Jetson and the VESC already share a ground through the power wiring; the GND wire is still
required as the signal reference. Never connect the VESC's 5 V to the Jetson's 5 V or to the
mux board's 5 V bus (same rule as the PPM BEC pin, `claude-docs/11-hardware.md`).

### V3. Host: free the UART and check permissions

On the Jetson host (not in the container):

```sh
ls -l /dev/ttyTHS1               # expect: crw-rw---- 1 root dialout ... /dev/ttyTHS1
getent group dialout             # note the GID; racer must be a member (id racer)
sudo lsof /dev/ttyTHS1           # expect: nothing holds it
systemctl list-units --all | grep -i -e getty -e nvgetty
```

If `nvgetty` (NVIDIA's serial console service) or a `serial-getty@ttyTHS1` unit is present and
active, a login prompt is fighting the driver for the port. Disable it:

```sh
sudo systemctl disable --now nvgetty
sudo systemctl disable --now serial-getty@ttyTHS1.service   # only if it exists
sudo usermod -aG dialout racer                              # only if racer is not in dialout
```

Log out and back in after a `usermod`. A udev rule is not needed for `ttyTHS1`: it is created
`root:dialout 0660` by the stock rules, so the `dialout` group is the whole permission story,
the same as the LiDAR's `ttyUSB0`. Optional loopback check of the Jetson side alone, with the
VESC unplugged and a jumper between pins 8 and 10:

```sh
stty -F /dev/ttyTHS1 115200 raw -echo
cat /dev/ttyTHS1 & sleep 0.5; echo uart-ok > /dev/ttyTHS1; sleep 0.5; kill %1   # expect: uart-ok
```

Remove the jumper afterwards.

### V4. Rebuild, then run the telemetry on its own

The driver is a new layer in `docker/car/Dockerfile` and the node is new in `ros_ws`. On the
Jetson, from `~/car`, after pulling this branch: rebuild the image and the workspace exactly as
in "LiDAR first power-up" L1, then check the lookup:

```sh
docker run --rm car:local bash -lc 'source /opt/ros/humble/setup.bash && ros2 pkg prefix vesc_driver'
# expect: /opt/racer_thirdparty
```

Start the driver and the odometry node alone, VESC powered (wheels off the ground, mux knob
killed: nothing here can move the car, but the VESC is live):

```sh
cd ~/car
docker run --rm -it --name car-vesc --network host \
  --user "$(id -u):$(id -g)" \
  --device /dev/ttyTHS1 \
  --group-add "$(getent group dialout | cut -d: -f3)" \
  -e HOME=/tmp \
  -v "$PWD":/workspace -w /workspace/ros_ws \
  car:local bash -lc '
    source /opt/ros/humble/setup.bash && source install/setup.bash
    exec ros2 launch racer_bringup vesc_telemetry.launch.py'
```

(`serial_port:=...` and `baud:=...` override the defaults `/dev/ttyTHS1` and 115200.) Expect
`Connected to VESC with firmware version <major>.<minor>` from `vesc_driver`, then
vesc_odometry_node's startup line with `0.000485 m/s per ERPM`. Without a VESC answering, the
driver never prints the firmware line and vesc_odometry_node warns `no VescStateStamped on
/vesc/sensors/core`; check, in order: App to Use (V1), TX/RX crossed (V2), the port is free
(V3), the baud on both ends. `Failed to connect to the VESC` means the device or the group did
not reach the container: re-check `--device` and `--group-add`.

### V5. Bench checks (record the results in the build log)

In a second shell: `docker exec -it car-vesc bash -lc 'source /opt/ros/humble/setup.bash &&
source install/setup.bash && ...'` with:

- `ros2 node info /vesc/vesc_driver`: **Subscribers lists only `/parameter_events`.** Any
  `commands/...` subscriber means the unpatched driver is running: stop and rebuild the image.
- `ros2 topic hz /odom/wheel`: about 50 Hz.
- `ros2 topic echo /telemetry/vesc/voltage_v`: the pack voltage, within a few tenths of a meter
  reading. `fault`: `NONE`. `temp_fet_degc`: room temperature-ish. `temp_motor_degc`: plausible
  only once motor temperature sensing is enabled in VESC Tool (step 13); until then it is not a
  measurement.
- **Sign check, wheels off the ground:** turn a wheel BY HAND in the forward direction. On a
  sensored motor the halls see this: `erpm` and `/odom/wheel` `twist.twist.linear.x` must go
  POSITIVE and `pose.pose.position.x` must grow. Negative means the VESC's positive rotation is
  backwards relative to `claude-docs/06-vehicle-params.md`'s "positive = drive torque forward";
  that is a VESC motor-direction setting or a wiring question to resolve and write down, not a
  sign to flip in software.
- **Distance check:** mark a tyre, roll the car by hand along the floor for exactly 10 wheel
  turns (or push it a taped 2.000 m), and compare `pose.pose.position.x` with 10 x 2 pi x
  `tires.nominal_radius_m` (or 2.000 m). The ratio is the first measurement of
  `drivetrain.gear_ratio` x tyre radius together; write it into the build log, and only then
  into `config/vehicle_params.yaml` (with its schema bump) when the gear ratio is counted.

To record it with everything else, start "Start the stack" with `--device /dev/ttyTHS1`, the
`dialout` `--group-add`, and `vesc:=true` (`vesc_serial_port:=` to override the port). The
recorder regex keeps `/odom/wheel`, `/telemetry/vesc/.*` and the raw `/vesc/sensors/core`.
If the VESC is absent the driver exits and the rest of the launch keeps running; the bag then
shows `STALE_NO_VESC_DATA` on `/telemetry/vesc/fault`.

## Cameras first power-up (OPTIONAL, outside the thesis; UNVERIFIED, written 2026-10-07 before the cameras were fitted)

**Optional, and not part of the thesis.** The cameras are for detection experiments and
training data. Nothing in the command path reads them, and every other section of this
runbook works the same with them unplugged. **Nothing in this section has been run on the
Jetson.** The ROS side (gscam with a test pattern in place of the camera, `camera_check`, the
recorder regex, the two-container DDS fix) was exercised in the `ros-dev` image on the Mac;
Argus, the overlay and both real cameras have not (`docs/notes/build-log.md`, 2026-10-07).

Hardware: one Waveshare IMX219-160 on the Orin Nano dev kit's CAM0 socket (the second IMX219
arrives in about three weeks) and one ELP AR0234 global-shutter UVC camera on USB 3. The car
does not move in any step; no battery is needed.

What it proves when it passes: the IMX219 overlay is applied without losing the 40-pin
pinmux, both cameras stream into ROS at the requested mode from an unprivileged container,
car-stack receives and records the compressed streams, and the bag growth rate is measured.
What it does not prove: any calibration, camera mount poses (not part of this work; no camera
transform is published), or anything about detection.

### C1. Fit the CSI camera, Jetson powered off

The Waveshare module has a 15-pin connector and the dev kit's camera sockets are 22-pin, so it
needs one of the 15-to-22-pin ribbons from the pack. Lift the CAM0 socket's latch, insert the
22-pin end with its exposed contacts facing the socket's contacts, close the latch, and do the
same at the camera end. Which way the contacts face on this carrier is UNVERIFIED: look at the
socket before inserting, and never plug or unplug a ribbon with the Jetson on.

### C2. Enable the IMX219 overlay (host, one time), WITHOUT losing the pinmux overlay

The 40-pin header currently depends on `/boot/racer-hdr40-gpio.dtbo` on the `OVERLAYS` line
of `/boot/extlinux/extlinux.conf` (`tools/jetson_pinmux/README.md`). `jetson-io` rewrites that
file. If it drops the racer overlay, header pin 7 goes back to tristated, the heartbeat never
reaches the mux, and the mux cuts (safe, but the car will not drive). If it re-adds
`/boot/jetson-io-hdr40-user-custom.dtbo`, the two overlays fight over the same pinmux node.
So back up first and check before rebooting:

```sh
sudo cp /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.pre-camera
grep -n OVERLAYS /boot/extlinux/extlinux.conf     # expect /boot/racer-hdr40-gpio.dtbo
sudo /opt/nvidia/jetson-io/jetson-io.py
```

In the menu (labels UNVERIFIED on this JetPack): "Configure Jetson 24pin CSI Connector" ->
"Configure for compatible hardware" -> **"Camera IMX219-A"** (single camera; A is expected to
be CAM0, UNVERIFIED on this carrier) -> "Save pin changes" -> **"Save and exit without
rebooting"**. Then, before rebooting:

```sh
ls /boot/*imx219*.dtbo                            # the camera overlay jetson-io picked
grep -n OVERLAYS /boot/extlinux/extlinux.conf
cd ~/car/tools/jetson_pinmux && sudo ./install.sh # idempotent: puts the racer overlay back on
                                                  # every OVERLAYS line, removes the conflicting
                                                  # jetson-io hdr40 entry, keeps the camera one
grep -n OVERLAYS /boot/extlinux/extlinux.conf     # must list BOTH racer-hdr40-gpio.dtbo and the
                                                  # imx219 .dtbo, and NOT jetson-io-hdr40-user-custom
sudo reboot
```

If `jetson-io` added a new boot entry (a second `LABEL` block) and made it the `DEFAULT`, the
check applies to that entry's `OVERLAYS` line; `install.sh` edits every one. To undo all of
it: `sudo cp /boot/extlinux/extlinux.conf.pre-camera /boot/extlinux/extlinux.conf` and reboot.

After the reboot, first the pinmux, then the camera:

```sh
ls /sys/class/pwm                                 # pwmchip0 and pwmchip2 still there
sudo systemctl stop racer-heartbeat
cd ~/car/tools/jetson_pinmux && sudo ./verify.sh 144 5   # pin 7: tristate 0, about 3.3 V
sudo systemctl start racer-heartbeat

sudo dmesg | grep -i imx219        # expect the sensor bound on an i2c bus, no "probe failed" / -121
ls -l /dev/video*                  # expect /dev/video0 = the CSI sensor (group video)
systemctl is-active nvargus-daemon # expect active
```

Then the first picture. With a monitor on the Jetson, `nvgstcapture-1.0 --sensor-id=0` shows
a preview (`j` then Enter saves a JPEG, `q` quits). Over SSH, without a display:

```sh
gst-launch-1.0 -e nvarguscamerasrc sensor-id=0 num-buffers=120 \
  ! 'video/x-raw(memory:NVMM),width=1280,height=720,framerate=60/1,format=NV12' ! fakesink
gst-launch-1.0 -e nvarguscamerasrc sensor-id=0 num-buffers=60 \
  ! 'video/x-raw(memory:NVMM),width=1280,height=720,framerate=60/1' ! nvvidconv \
  ! 'video/x-raw,format=I420' ! jpegenc ! multifilesink location=/tmp/csi0_%02d.jpg
```

The first command must end with no error. Its `GST_ARGUS:` lines list the sensor modes
(expect a 1280 x 720 mode at about 60 fps among them): **copy that list into the build log**.
If 1280x720 at 60 is not listed, use a mode that is, in every command below
(`csi_width`/`csi_height`/`csi_fps`). Copy `/tmp/csi0_59.jpg` (a late frame, after auto
exposure settled) to the Mac to look at it. "No cameras available" means the overlay or the
ribbon: re-check `dmesg`, then the ribbon orientation, then try "Camera IMX219-C".

### C3. Plug in the USB camera and find its device node

Any of the dev kit's USB-A ports. On the host:

```sh
lsusb                                             # the ELP camera; note VID:PID for the build log
lsusb -t                                          # its line should show 5000M (USB 3), not 480M
v4l2-ctl --list-devices                           # the ELP lists two nodes; the first captures
USBCAM=$(readlink -f /dev/v4l/by-id/usb-*-video-index0); echo "$USBCAM"   # e.g. /dev/video1
v4l2-ctl -d "$USBCAM" --list-formats-ext          # expect MJPG with 1280x720 at 60 fps
ls -l "$USBCAM"                                   # group video
```

Use that plain `/dev/videoN` everywhere: usb_cam 0.8.1 rejects `/dev/v4l/by-id/` paths
(`camera_usb.launch.py` docstring), and the container must see the device at the same path
as the host. N can change when cameras are re-plugged or the CSI overlay changes, so re-run
the `readlink` line before each session.

### C4. Rebuild the image and the workspace

The camera packages are a new layer in `docker/car/Dockerfile`, and the camera launch files,
`camera_check` and the DDS profile are new in `ros_ws`. On the Jetson, from `~/car`, after
pulling:

```sh
docker build -t car:local docker/car
docker run --rm -v "$PWD":/workspace -w /workspace car:local bash -lc '
  source /opt/ros/humble/setup.bash
  apt-get update && rosdep install --from-paths ros_ws/src --ignore-src -r -y
  cd ros_ws && colcon build --symlink-install
  chown -R 1000:1000 build install log /workspace/tools/.venv'
```

Then check that the NVIDIA runtime gives an unprivileged container the Argus plugin (this is
the UNVERIFIED part of the whole design):

```sh
docker info | grep -i runtimes                    # expect nvidia among them
docker run --rm --runtime nvidia --user "$(id -u):$(id -g)" -e HOME=/tmp \
  --group-add "$(getent group video | cut -d: -f3)" \
  -v /tmp/argus_socket:/tmp/argus_socket car:local bash -lc '
    gst-inspect-1.0 nvarguscamerasrc | head -n 5
    gst-launch-1.0 -e nvarguscamerasrc sensor-id=0 num-buffers=60 \
      ! "video/x-raw(memory:NVMM),width=1280,height=720,framerate=60/1,format=NV12" ! fakesink'
```

"No such element or plugin" means the runtime did not mount NVIDIA's GStreamer plugins: check
`grep -ri argus /etc/nvidia-container-runtime/host-files-for-container.d/` on the host and
record what is there. An nvmap / nvhost / "Failed to create CaptureSession" error means a
device node or the socket did not get through: `ls -l /dev/nvhost* /dev/nvmap /dev/host1x*
2>/dev/null` on the host and add each with `--device`, and check `ls -l /tmp/argus_socket`.
Do not reach for `--privileged`; record what was needed in the build log and in
`docker/car/README.md`'s flag table.

### C5. Start the cameras in their own container, car-camera

car-stack is unprivileged and has no NVIDIA runtime by design, so the cameras get their own
container on the host network (`docker/car/README.md` "Cameras" explains every flag). From
`~/car`, in its own terminal:

```sh
cd ~/car
USBCAM=$(readlink -f /dev/v4l/by-id/usb-*-video-index0)
docker run --rm -it --name car-camera --network host \
  --runtime nvidia \
  --user "$(id -u):$(id -g)" \
  --group-add "$(getent group video | cut -d: -f3)" \
  -e HOME=/tmp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/ros_ws/src/racer_bringup/config/fastdds_udp_only.xml \
  -v /tmp/argus_socket:/tmp/argus_socket \
  --device "$USBCAM" \
  -v "$PWD":/workspace -w /workspace/ros_ws \
  car:local bash -lc "
    source /opt/ros/humble/setup.bash && source install/setup.bash
    exec ros2 launch racer_bringup cameras.launch.py usb_device:=$USBCAM"
```

Leave the `FASTRTPS_DEFAULT_PROFILES_FILE` line in: without it car-stack sees the camera
topics but receives no frames (reproduced on the Mac, see the build log). Other arguments:
`usb:=false` or `csi:=false` to start one camera, `usb_width`/`usb_height`/`usb_fps`,
`csi_width`/`csi_height`/`csi_fps`, `csi_flip_method:=2` for an upside-down CSI mount, and
`jpeg_quality:=60` (reaches both cameras) to shrink the bag.

Expect, from the two nodes:

```
[camera_usb] Starting 'usb' (/dev/video1) at 1280x720 via mmap (mjpeg2rgb) at 60 FPS
[camera_csi0] Using gstreamer config from rosparam: "nvarguscamerasrc sensor-id=0 ! ..."
[camera_csi0] Publishing stream...
[camera_csi0] Started stream.
```

Both warn that no calibration file exists: expected, nothing is calibrated. `Device specified
is not available or is not a vaild V4L2 device` means `usb_device` and `--device` disagree or
the camera moved to another N. `Failed to PAUSE stream, check your gstreamer configuration`
means the pipeline did not start: go back to the C4 container check. If the USB picture has
wrong colours or stripes, its JPEGs are 4:2:0: start that camera on its own with
`ros2 launch racer_bringup camera_usb.launch.py device:=$USBCAM av_device_format:=YUV420P`
and record it.

### C6. Measure the frame rate

In a second terminal, inside car-camera:

```sh
docker exec -it car-camera bash -lc '
  source /opt/ros/humble/setup.bash && source install/setup.bash
  ros2 run racer_tools camera_check --ros-args -p topic:=/camera/usb/image_raw/compressed \
    -p expected_width:=1280 -p expected_height:=720
  ros2 run racer_tools camera_check --ros-args -p topic:=/camera/csi0/image_raw/compressed \
    -p expected_width:=1280 -p expected_height:=720'
```

Each listens for 10 s (`-p duration_s:=30.0` for longer) and exits 0 on PASS, 1 on FAIL, 2 if
fewer than two frames arrived. With the defaults:

| Line | Expect | Fails when |
|---|---|---|
| frame rate (stamps) | about 60 Hz | below 54 Hz (`min_rate_hz`; pass a lower value if the camera was launched slower) |
| frame rate (arrival) | about the same | (reported only; much lower than the stamp rate means a slow subscriber) |
| resolution | 1280x720 | anything else, or a change during the window |
| undecodable frames | 0 | any |
| mean frame size | tens to a couple of hundred kB, scene dependent | (reported only) |
| payload rate | about 5 to 10 MB/s per camera (estimate) | (reported only; this IS the bag growth, record it) |

A UVC camera in a dim room may stretch its exposure past 1/60 s and drop below 60 fps: test
in normal room light, and note it if the rate falls with the lights down. While both cameras
run, note the CPU load with `top` on the host: the USB stream is decoded and re-encoded on the
CPU, and the CSI stream converted and encoded there too (UNVERIFIED cost at 720p60). If the
control stack's nodes slow down, drop to `usb_fps:=30 csi_fps:=30`.

Then the same check from car-stack's side, which is what the recorder sees (start car-stack
as in "Start the stack" if it is not running):

```sh
docker exec -it car-stack bash -lc '
  source /opt/ros/humble/setup.bash && source install/setup.bash
  ros2 run racer_tools camera_check --ros-args -p topic:=/camera/usb/image_raw/compressed'
```

PASS here too. Exit 2 with the topics listed is the shared-memory problem: car-camera was
started without the profile line.

### C7. Look at it in Foxglove

car-stack's `foxglove_bridge` (on by default, `viz`) serves the camera topics too: in Foxglove
on the Mac, connect to `ws://10.0.0.226:8765`, add an **Image** panel and pick
`/camera/usb/image_raw/compressed`, then a second for `/camera/csi0/image_raw/compressed`.
Always the `compressed` topics: a raw `image_raw` is about 166 MB/s per camera and will choke
the WiFi. Expect a live picture at whatever rate the WiFi carries (below the camera's 60 fps
is normal over WiFi; `camera_check` is the rate measurement, not Foxglove). Without car-stack
running, start a bridge in car-camera the way LiDAR step L5 does
(`docker exec -it car-camera bash -lc '... exec ros2 run foxglove_bridge foxglove_bridge
--ros-args -p port:=8765'`). If the CSI picture is upside down, restart car-camera with
`csi_flip_method:=2`.

### C8. Recording and bag growth

**Lean bags (2026-11-11).** The bag no longer takes the full-rate camera topics. Each camera has a
`camera_record_node` (started by `cameras.launch.py`, `record_streams:=true`) that publishes
`/camera/<name>/record/compressed`: at most 5 Hz, fitted inside 640 x 360, JPEG 60, and published
only while `/drive_raw` is being published (a driving launch is up, plus 5 s). `car_teleop.launch.py`'s
recorder regex takes that topic and NOT `image_raw/compressed`. Cameras on with the car idle record no
camera data at all; driving records a few hundred kB/s per camera. The full-rate topics stay live for
Foxglove, the racer app and the lap counter. Tune with `record_rate_hz`, `record_max_width_px`,
`record_max_height_px`, `record_jpeg_quality`, `record_gate_hold_s`; `record_gate:=false` publishes
the stream all the time (bench only). The numbers below are the FULL-RATE topics, which you only get
now by recording them by hand (`ros2 bag record -s mcap --regex "^/camera/.*/image_raw/compressed$"`).

Full-rate growth at 720p60, **an estimate until measured**: a 1280x720 JPEG at `jpeg_quality` 80 is roughly 80
to 170 kB for an indoor scene, so 60 fps is about **5 to 10 MB/s per camera (17 to 37 GB per
hour)**, about 10 to 20 MB/s (35 to 75 GB per hour) with both. The raw images would be
1280 x 720 x 3 bytes x 60 = 166 MB/s per camera, which is why they are left out. Replace the
estimate with `camera_check`'s measured payload rate, and check the space first:

```sh
df -h ~/car/data
```

Lower `jpeg_quality` or the fps if a session would not fit. After a recorded run,
`ros2 bag info` on the bag should list `/camera/usb/image_raw/compressed` and
`/camera/csi0/image_raw/compressed`, each with about 60 messages per second of duration, and no
`image_raw`.

To collect camera data without the drive stack (no car-stack, so no drive and no invariant-5
recorder), record inside car-camera:

```sh
docker exec -it car-camera bash -lc '
  source /opt/ros/humble/setup.bash
  mkdir -p /workspace/data/bags
  exec ros2 bag record -s mcap --regex "^/camera/.*/compressed$" \
    -o /workspace/data/bags/$(date +%Y-%m-%dT%H-%M-%S)_cameras'
```

Ctrl-C closes the bag cleanly.

### C9. Write it down

Dated `docs/notes/build-log.md` entry: which overlay `jetson-io` applied and the final
`OVERLAYS` line, the `GST_ARGUS` sensor-mode list, the USB camera's VID:PID, `/dev/videoN` and
`--list-formats-ext` MJPG modes, anything the container needed beyond the documented flags,
both `camera_check` reports from car-camera and from car-stack, the measured MB/s, and the CPU
load. When the second IMX219 arrives: `jetson-io` "Camera IMX219 Dual", the same overlay
checks as C2, and `camera_csi.launch.py sensor_id:=1` publishes under `/camera/csi1/*`
(`cameras.launch.py` starts sensor 0 only until then).

## First mapping session (UNVERIFIED, written 2026-11-03; proven only in the sim)

Builds a slam_toolbox map of the floor track, then localises on it with AMCL. Background,
tuning and the sim numbers: `docs/notes/localization-2026-11-03.md`. Needs the LiDAR bench
("LiDAR first power-up") and the VESC telemetry bench ("VESC telemetry") done first: the EKF's
speed is `/odom/wheel` and both map builders read `/scan`. Nothing here changes the command path;
the kill switch rules are the usual ones.

### M1. Rebuild

The car image gains five apt packages (robot_localization, slam_toolbox, nav2_amcl,
nav2_map_server, nav2_lifecycle_manager) and the workspace gains `racer_state`. Rebuild both
exactly as in "LiDAR first power-up" L1, then:

```sh
docker run --rm car:local bash -lc 'source /opt/ros/humble/setup.bash && ros2 pkg prefix slam_toolbox nav2_amcl robot_localization'
```

### M2. Calibrate the wheel speed first (10 minutes, saves the most error)

The sim's biggest error source was a 2% wheel speed scale error (`tires.nominal_radius_m` is
assumed). Do V5's distance check properly: push the car along a taped 5.000 m straight, three
times, and compare `/odom/wheel` `pose.pose.position.x`. Write the ratio into the build log. If it
is more than 1% off, correct `tires.nominal_radius_m` (schema patch bump) before mapping.

### M3. Map: one slow lap, Jetson teleop

Start the stack as in "Start the stack", with the LiDAR and VESC flags from those sections and
the end of the command changed to:

```sh
exec ros2 launch racer_bringup car_teleop.launch.py browser_teleop:=true lidar:=true vesc:=true mapping:=true
```

- Put the car on the start line and LEAVE IT STILL for 10 s while the stack comes up: the map's
  origin is where the car stands when slam_toolbox starts, facing +x. Mark that spot and heading
  on the floor with tape; localisation starts from it.
- In Foxglove (fixed frame `map`): `/map`, `/scan`, the TF tree `map -> odom -> base_link ->
  laser`. Expect `wheel_twist_node up: ...`, the EKF, and `slam_toolbox` registering scans.
- Drive ONE lap at 0.8 to 1.0 m/s with the browser teleop (Jetson teleop, not the RC
  transmitter: the EKF models the steering from `/drive`, which the RC override bypasses). The
  sim mapped cleanly at 0.5 to 1.5 m/s; slower gives the odometry less to get wrong between
  scan matches. Smooth steering, no reversing. Finish the lap past the start so the loop
  closes, then a few metres more.
- If the loop is plain foam board all the way round, put two or three boxes or bags OUTSIDE the
  walls (where the LiDAR sees them over the boards, or through gaps) before mapping, and leave
  them there for every later run: walls that look the same everywhere are what fooled the sim's
  loop closure and AMCL's along-track estimate.
- Watch `/map` grow. Walls should be single thin lines. Doubled or smeared walls mean the scan
  matcher lost track or the loop closed in the wrong place: stop, note where, and redo the lap.

### M4. Save the map (stack still running)

In a second shell into the container:

```sh
docker exec -it car-stack bash -lc '
  source /opt/ros/humble/setup.bash && source /workspace/ros_ws/install/setup.bash
  mkdir -p /workspace/data/maps
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: /workspace/data/maps/floor-2026-11-xx}}"'
```

Expect `result: 0` and `floor-2026-11-xx.pgm` + `.yaml` in `~/car/data/maps/` (not in git,
`data/` is ignored; copy it off the car). Open the pgm: thin walls, grey (unknown) outside. Do
NOT paint walls in by hand: AMCL's laser model is biased by filled walls (the note, finding 1).
Stop the stack with Ctrl-C; the bag holds `/scan`, `/odom`, `/tf` and `/map` for replay.

### M5. Localise on it

Put the car back on the taped start spot, facing the same way, and start with the end of the
command changed to:

```sh
exec ros2 launch racer_bringup car_teleop.launch.py browser_teleop:=true lidar:=true vesc:=true localization:=true map:=/workspace/data/maps/floor-2026-11-xx.yaml
```

The default initial pose is the map origin, which is that spot. If the car is elsewhere, use
Foxglove's "Publish pose estimate" on `/initialpose` (map frame).

1. Car still: the scan in Foxglove must sit on the map's walls. If it is rotated or offset, set
   the pose estimate again.
2. Push the car round the lap BY HAND (motor off, mux killed): `/pose` and TF `map ->
   base_link` must follow it round the whole loop without jumping. `ros2 topic hz /pose` is
   scan rate only while moving (AMCL updates every 0.1 m).
3. Then one lap at walking pace with the teleop, then 1.0 m/s. Watch for the scan sliding off
   the walls on the straights (along-track drift, expected to be the weak direction).
4. Write in the build log: map name and start spot, how well the scan sat on the walls at each
   speed, any jump of `/pose` and where, CPU (`top`: amcl, ekf_node).

Driving the raceline tracker on this pose (tracker_node `pose_frame:=map`) needs a raceline
written in this map's frame, which is a later step.

## Using the app (racer-agent web UI; UNVERIFIED on the car, written 2026-11-11)

The app replaces the ssh-and-docker-commands part of a session: it starts the stack, the cameras
and the driving launches with the same commands this runbook uses, shows the pre-drive checks, and
has a STOP ALL button. Design, thresholds and what is untested: `docs/notes/racer-app-2026-11-11.md`.
**It does not replace anything in the safety section above.** Wheels off the ground for the first
runs, a second person on the RC kill knob, and the knob armed and tested before power reaches the
motor, every session. The app never publishes to the command path and its STOP is a software stop:
Wi-Fi can drop.

### A1. Install (once, then again after every `git pull`)

```sh
ssh racer@racer-car
cd ~/car && tools/racer_app/deploy/install.sh
```

It prints an access token once: save it. Re-running is safe and keeps the token and your autostart
choice. It needs the docker group, `python3` 3.10 or newer, and installs `uv` (pinned) if missing.
Check: `systemctl status racer-agent` (active), `curl -s localhost:8080/health`.

### A2. Open it

From the Mac: `python3 tools/racer_app/find_car.py` (mDNS, then a scan of the local network). Or
`http://racer-car.local:8080/` on a phone or laptop on the same Wi-Fi. Paste the token once; the
browser remembers it. The page is only reachable from private addresses.

**The desktop app (Racer).** Same UI, as an app that finds the car by itself. Get the installer
from the "Racer desktop" workflow run on GitHub (Actions, the run for the branch, Artifacts) and
unzip it. The builds are unsigned, so the first launch shows a warning:

- **Mac:** open the `.dmg`, drag Racer to Applications. First time, right-click Racer, choose
  Open, then Open again in the dialog (a double click only offers Move to Bin). If macOS says the
  app is damaged, run `xattr -dr com.apple.quarantine /Applications/Racer.app` once.
- **Windows:** run the `.msi` (or the `-setup.exe`). SmartScreen says "Windows protected your
  PC": click More info, then Run anyway.
- **Linux:** `chmod +x Racer*.AppImage && ./Racer*.AppImage`, or install the `.deb`.

On launch it looks for the car on the Wi-Fi (mDNS first, then it scans the local subnet) and
connects when it finds one; with several cars it lists them. It remembers the last car, goes
looking again by itself when the Jetson reboots, and follows it if it comes back on a different
address. If it finds nothing, type the address (for example `192.168.1.50`) in the box. The first
time it asks for the access token that install printed: paste it once and it is kept in the Mac
Keychain, Windows Credential Manager or the Linux Secret Service. Like the web page it never drives
the car, STOP ALL is still a software stop, and the knob is still the stop that works.

### A3. A session

1. **Power the car.** With autostart on (the default after install) the stack comes up by itself
   about a minute after the Jetson boots: LiDAR and VESC, no cameras, recorder running. In the
   UI the Stack card says running and the checks list fills in. If it does not, look at
   `journalctl -u racer-stack` (a full disk makes it refuse, on purpose).
2. **Read the checks.** Required checks must be green to drive: stack, recorder, battery (3S, 3.70
   V per cell or better), disk (5 GiB is the hard floor, 15 is green), safety node, LiDAR `/scan`,
   VESC telemetry and fault, heartbeat service, temperatures. Wi-Fi, cameras, containers and
   uptime are shown but never gate. Amber or red on a required check means the Driving card asks
   for an override and a typed reason, which goes in the action log. A missing stack, a missing
   recorder or a disk under 5 GiB cannot be overridden.
3. **Cameras (optional).** Start cameras. They add a few hundred kB/s each to the bag while a drive runs and nothing while idle (the bag stream is 5 Hz, 640 x 360), need 10 GiB free
   to start, and the disk guard stops them below 7 GiB. The thumbnails refresh about once a second.
4. **Drive.** Pick gap follow (profile `lane-2026-10-25`), wall follow, tracker (needs the stack
   started with localisation, a live pose, and a raceline in that map's frame) or park. The
   car does not move until you arm it with the knob and the launch starts publishing; the launch
   is the same process as in "Gap follow from the launch file" above, including the reverse escape.
   Park has Park, Three-point turn and Abort buttons once its launch is running.
5. **Stop.** STOP ALL (bottom of the screen) sends SIGINT to the driving launch, the same as Ctrl-C
   in the second terminal; `safety_node` then brakes on the `/drive_raw` silence and the stack keeps
   recording. If anything does something you did not expect, the knob is the first thing to reach
   for, not the button.
6. **Mapping.** Stack card: tick mapping, Start stack (or Restart), drive the lap slowly with the
   teleop, type a map name and Save map (writes `data/maps/NAME.yaml`, refuses to overwrite). Then
   restart with localisation and choose that map.
7. **Finish.** Stop stack closes the bag cleanly. Reboot and Shut down stop everything cleanly
   first. The Bags card shows folder sizes and can delete old ones (not the one being recorded).

### A4. What to check on the first run on the bench

- `systemctl status racer-agent racer-stack` and `journalctl -u racer-agent` after a reboot: the
  stack container is up and `docker top car-stack -eo args` shows `ros2 bag record`.
- The dashboard shows speed from `/odom/wheel`, battery volts from the VESC, the LiDAR scan with
  the car outline from `vehicle_params`, and the front of the car at the top. Put a hand in front of
  the LiDAR: the dots must appear in front of the car outline, not behind it.
- Start gap follow with the motor unpowered or wheels off the ground, press STOP ALL, and confirm
  in the action log and `/safety/events` that the launch ended and `safety_node` braked.
- Pull the Wi-Fi mid-run (bench, wheels off): the launch keeps running, which is exactly why the
  knob matters. Write down what happened in `docs/notes/build-log.md`.
- Power-cycle the Jetson with the stack running: the next agent start logs an unclean shutdown.

## New track session (UNVERIFIED on the car, written 2026-11-11; proven only in the sim)

The owner rebuilds the track every session, so nothing here is tuned to one layout: a mapping
run becomes a map, the map a raceline for the car's real turning limits, and the raceline a
tracker launch. One command per step, all in `tools/track_session/track_session.py` (run the
offline steps with `uv run --project tools python tools/track_session/track_session.py ...`).
Results and the reasoning are in `docs/notes/track-session-2026-11-11.md`. Read its "What to
expect" first: on a lane around 0.8 m wide or an island end this tight the tracker can stall
against safety_node's corridor, and `gap_follow` with reverse escape (optionally with outside
entry) is the driver that gets round.

The steering limits (0.449 rad each way, vehicle_params 0.14.0), the speed scale and the overhangs
are read from `config/vehicle_params.yaml`; commands are true angles and true speeds, so nothing is
scaled. Bags recorded before the calibration (2026-11-10) are mapped with `--speed-divisor 1.401
--steer-scale-left 1.203 --steer-scale-right 1.072`.

### T1. Map (one slow lap, Jetson teleop, as in M3) and save

Live on the car, with `mapping:=true` as in "First mapping session" M3, then

    uv run --project tools python tools/track_session/track_session.py save-map \
        --out /data/maps/<track>/<track>

or offline from the teleop bag (in the ros-dev image on the laptop; it replays the bag through the
car's own EKF and slam_toolbox with the scan motion corrected, which matters on a lane this narrow,
see the note):

    tools/track_session/track_session.py map /data/bags/<bag> --name <track>

Look at `<track>.pgm`: crisp single walls, the island a closed outline. Drive the first lap
counter-clockwise and slowly; a map from clockwise driving alone came out doubled in the sim of
the real bags.

### T2. Look at the lane

    uv run --project tools python tools/track_session/track_session.py analyze /data/maps/<track>/<track>.yaml

Prints the lane width along the centreline, the island's size, the radius of each end's outer
wall and the centreline radius at each end. A centreline end radius under the car's 0.60 m (left)
or 0.69 m (right) turning radius means the middle of the lane is not drivable at full lock and the
line has to run wide.

### T3. Raceline

    uv run --project tools python tools/track_session/track_session.py raceline \
        /data/maps/<track>/<track>.yaml --v-max 0.9

Writes `<track>_ccw.csv` and `<track>_cw.csv` (tracker_node's format), a JSON report and a PNG
each. The line is the minimum-curvature path inside the lane with the whole body outline kept off
the walls (the widest of 0.25 to 0.05 m the lane allows: 0.25 first because the localised car brakes on lines that pass closer than about 0.2 m, see the note) and the curvature held under the car's
steering on each side (85 percent of tan(lock) / wheelbase). It exits 2 and says NOT FEASIBLE when
the lane does not allow it: believe it, and use `gap_follow` there. `--v-max` is the top TRUE
speed; start at 0.5.

### T4. Run it

    uv run --project tools python tools/track_session/track_session.py tracker \
        /data/maps/<track>/<track>_ccw.csv /data/maps/<track>/<track>.yaml

prints the two commands and the checklist: `car_teleop.launch.py ... localization:=true map:=...`
in one terminal and `ros2 launch racer_control track_follow.launch.py raceline:=...` in another.
Kill switch held, lane clear, car on the mapping start spot facing the same way. Every run is in
the bag.

## Afterwards

- Everything measured in steps 5, 10, 11, 12, 13 and 15 goes into `config/vehicle_params.yaml` or a
  dated build-log entry. A measurement that lives only in a scrollback did not happen
  (`claude-docs/10-conventions.md`).
- Roadmap 1.3's real kill test (Jetson genuinely frozen, cut observed) is still open and is
  the next thing, not the floor.
- The throttle map stays open loop and provisional until `vesc_node` exists.
