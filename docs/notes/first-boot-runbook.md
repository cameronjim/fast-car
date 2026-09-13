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

## 11. Power the servo and the ESC, wheels off the ground, kill switch armed (UNVERIFIED)

11.1 Second person on the RC transmitter, kill switch in the CUT position, hand on it.

11.2 Power the mux board's 5 V rail (UBEC), then the receiver. Confirm the mux is not in its
     fault blink (`firmware/safety_mux/README.md`: fast blink = it refused to arm).

11.3 Connect the servo. Nothing should move. If the wheels twitch or crawl to a lock, cut
     power: the neutral value or the polarity is wrong, and step 12 is where that gets sorted
     out, not with the ESC live.

11.4 Only then connect the drive battery / ESC. Expect silence and no motor motion. **A
     creeping motor here means this ESC's real zero-throttle point is not 1500 us** -- cut
     power, measure it, and put the measured value in `config/vehicle_params.yaml` before
     going further. The mux firmware bakes these in at compile time, so that means rebuild
     and reflash too.

## 12. Calibrate the steering polarity (UNVERIFIED -- this is the bench calibration the code is waiting for)

`steering_left_is_pwm_max` defaults to `true` and is a guess. With the wheels off the ground
and the kill switch held:

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

## 13. Keyboard teleop, wheels off the ground, second person on the kill switch (UNVERIFIED)

13.1 Confirm the rosbag is recording and rail voltage is being logged. A run without a bag is
     a bug (`CLAUDE.md` invariant 5).

13.2 Kill-switch person: cut, confirm the wheels and motor stop, restore. Do this before
     driving, not after.

13.3 In a second terminal (keyboard teleop needs a real TTY, which is why it is not started
     by the launch file):

```sh
docker exec -it <container> bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/ros_ws/install/setup.bash && ros2 run racer_tools keyboard_teleop_node'
```

13.4 Smallest possible speed command first. Confirm: the motor spins the correct direction,
     releasing the key returns to neutral within the watchdog timeout, and the kill switch
     stops it instantly at any point.

13.5 Stop, power down in reverse order (drive battery, then servo/receiver rail, then the
     Jetson), and write the session up in `docs/notes/build-log.md` the same day.

## Afterwards

- Everything measured in steps 5, 10, 11 and 12 goes into `config/vehicle_params.yaml` or a
  dated build-log entry. A measurement that lives only in a scrollback did not happen
  (`claude-docs/10-conventions.md`).
- Roadmap 1.3's real kill test (Jetson genuinely frozen, cut observed) is still open and is
  the next thing, not the floor.
- The throttle map stays open loop and provisional until `vesc_node` exists.
