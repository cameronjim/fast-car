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

1.3 Confirm the heartbeat service that the mux's watchdog input depends on is running, and
    that nothing has been reconfigured under it:

```sh
systemctl status racer-heartbeat.service    # expect: active (running)
sudo gpioinfo gpiochip0 | grep -w 144       # expect: "racer-heartbeat" output [used]
```

If that line is not claimed, stop and fix it before anything else: `tools/jetson_heartbeat/`
has the reproduction steps. A silent heartbeat means the mux sees a dead Jetson.

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

## 4. Enable the two hardware PWM pins (UNVERIFIED -- the whole procedure)

Full steps and the reasoning: `ros_ws/src/racer_drivers/README.md`, "Enabling PWM pins on the
Jetson". In short:

4.1 `sudo /opt/nvidia/jetson-io/config-by-pin.py` -- record what pins 15 and 33 currently
    are. **Confirm pin 7 stays a plain GPIO** (the heartbeat).

4.2 `sudo /opt/nvidia/jetson-io/config-by-function.py` -- select `pwm` for the entries
    covering pins 15 and 33, save as a new overlay, **reboot**.

4.3 After the reboot, find the real chip/channel numbers -- do not assume `pwmchip0/pwm0`:

```sh
for chip in /sys/class/pwm/pwmchip*; do
  echo "$chip -> $(readlink -f "$chip" | sed 's#/sys/devices/##')  npwm=$(cat "$chip/npwm")"
done
```

4.4 Write the raw output of 4.3 into `docs/notes/build-log.md` with the date. A mapping that
    lives only in a terminal scrollback did not happen.

## 5. Bench-check one PWM channel with nothing connected (UNVERIFIED)

Before any ROS node touches a pin, with **nothing plugged into the header**:

```sh
echo 0        | sudo tee /sys/class/pwm/pwmchipN/export
echo 20000000 | sudo tee /sys/class/pwm/pwmchipN/pwm0/period
echo 1500000  | sudo tee /sys/class/pwm/pwmchipN/pwm0/duty_cycle
echo 1        | sudo tee /sys/class/pwm/pwmchipN/pwm0/enable
```

Scope or duty-reading meter on the pin: **50 Hz, 1.5 ms high**. Anything else and the pinmux
change did not take. Then `echo 0 | sudo tee /sys/class/pwm/pwmchipN/pwm0/enable`.

Repeat for the other pin. Record both measurements.

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

```sh
docker run --rm -it --runtime nvidia --network host --privileged -v /sys:/sys \
  -v "$PWD":/workspace -w /workspace/ros_ws car:local bash -lc '
    source /opt/ros/humble/setup.bash && source install/setup.bash
    ros2 launch racer_bringup car_teleop.launch.py \
      steering_pwmchip:=N steering_pwm_channel:=M \
      throttle_pwmchip:=P throttle_pwm_channel:=Q'
```

Expect two nodes and a startup line from `pwm_output_node` naming the calibration it read out
of `vehicle_params`. If it refuses to start, read the message: it names the field it is
missing, and the answer is to measure that field, never to edit the check.

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

## 12. Calibrate the steering polarity -- THE FIRST BENCH CALIBRATION (UNVERIFIED)

This is the one the code is explicitly waiting for, and it comes before the ESC is powered
because it needs nothing but the servo and because getting it wrong is how the car steers into
the thing it was avoiding.

`steering_left_is_pwm_max` defaults to `true` **and that default is a GUESS**. No project doc
defines which pulse end is full left; nobody has put a scope on this servo. The whole
left-positive chain -- `a`/LEFT key increases `steering_angle_rad`, `angular.z > 0` with
forward speed gives a positive angle, `safety_node` passes the sign through unchanged, and
`pwm_output_node` sends a positive angle to `steering.pwm_max_us` when this flag is true -- is
pinned by unit tests at every hop, so the ONLY unknown left in it is this flag. That makes
this one measurement the difference between a correct chain and a mirrored one.

With the wheels off the ground, the ESC unpowered, and the kill switch held:

12.1 Publish a small LEFT command by hand (positive angle, zero speed):

```sh
ros2 topic pub --once /drive_raw ackermann_msgs/msg/AckermannDriveStamped \
  '{drive: {steering_angle: 0.2, speed: 0.0}}'
```

12.2 Watch the front wheels. They must turn **left**. If they turn right, restart the launch
     with `steering_left_is_pwm_max:=false` and repeat until left means left.

12.3 Write the answer into `docs/notes/build-log.md` AND change the launch file's default so
     nobody has to remember the flag.

Also check the ends: command `steering_angle: 0.4189` and `-0.4189` and confirm the rack does
not bind or buzz at either end. If it does, the pulse ends are past this servo's real travel
and `steering.pwm_min_us` / `pwm_max_us` need measuring, not guessing.

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
     a bug (`CLAUDE.md` invariant 5).

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

## Afterwards

- Everything measured in steps 5, 10, 11, 12, 13 and 15 goes into `config/vehicle_params.yaml` or a
  dated build-log entry. A measurement that lives only in a scrollback did not happen
  (`claude-docs/10-conventions.md`).
- Roadmap 1.3's real kill test (Jetson genuinely frozen, cut observed) is still open and is
  the next thing, not the floor.
- The throttle map stays open loop and provisional until `vesc_node` exists.
