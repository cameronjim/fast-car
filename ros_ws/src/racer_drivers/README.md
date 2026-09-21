# racer_drivers

On-vehicle drivers (`claude-docs/02-repo-layout.md`). One node so far.

## `pwm_output_node` -- the tail of the command path

```
safety_node --/drive--> pwm_output_node --2x 50 Hz PWM--> layer-1 mux board --> servo + VESC PPM
```

It subscribes to `/drive` (reliable, depth 10) and to nothing else on the command path, and
converts `steering_angle` (rad, LEFT positive per `claude-docs/06-vehicle-params.md`) and
`speed` (m/s) into two 50 Hz servo pulses driven by the Jetson's hardware PWM through the
Linux sysfs PWM interface.

**It never subscribes to `/drive_raw`.** Doing so would put an ungated command on the wire
and route around `safety_node`, which is the bypass `CLAUDE.md` invariant 1 forbids. The L3
launch tests assert this against the live node graph, not by reading the source.

### Status

**UNVERIFIED ON HARDWARE.** As of 2026-09-13 this node has been built and tested only in the
`ros-dev` container against a FAKE sysfs tree (ordinary files in a temp directory). No pin
has been scoped, no servo has moved, no ESC has been armed by it. Everything under "Enabling
PWM pins on the Jetson" below is a written-ahead procedure, not a record of something done.

### Fail-closed behaviour

Both channels are written the calibrated neutral, in this order, in every one of these cases:

| When | What happens |
|---|---|
| Startup, before the `/drive` subscription is even created | period set, neutral duty written, channel enabled |
| No `/drive` yet | neutral every cycle |
| `/drive` silent for `drive_timeout_s` (default 0.1 s) or longer | neutral every cycle |
| A non-finite `steering_angle` or `speed` in a fresh message | that channel goes neutral |
| Any exception in the 50 Hz cycle | both channels forced to neutral, throttled error log, node keeps running |
| SIGINT / SIGTERM / shutdown | neutral written, THEN the channel disabled |

Disabling a channel stops the pulses entirely, which the mux firmware's PWM validity check
reads as a cut -- that is deliberate, and it is why neutral is written before the disable
rather than instead of it.

This is layer-3-adjacent risk reduction, not a guarantee. `claude-docs/05-safety.md`: only
layer 1, the hardware mux, is a guarantee.

### Where the numbers come from

Every physical value comes from `config/vehicle_params.yaml` through the generated C++
binding (`CLAUDE.md` invariant 2). The node refuses to start, naming the field, if any of
these is `null` -- the same refuse-to-arm discipline as `firmware/safety_mux`'s
`logic/mux_params.c`:

| vehicle_params field | Used for | Value today |
|---|---|---|
| `steering.pwm_min_us` / `pwm_neutral_us` / `pwm_max_us` | steering pulse ends and neutral | 1000 / 1500 / 2000 (PROVISIONAL, unmeasured) |
| `steering.min_angle_rad` / `max_angle_rad` | angle range the pulse ends correspond to | -0.4189 / +0.4189 (gym defaults) |
| `actuation.throttle_pwm_min_us` / `throttle_pwm_neutral_us` / `throttle_pwm_max_us` | throttle pulse ends and neutral | 1000 / 1500 / 2000 (PROVISIONAL, unmeasured) |
| `actuation.throttle_full_scale_mps` | full-scale reference for the open-loop speed map | 5.0 (PROVISIONAL, unmeasured) |
| `limits.global_speed_cap_mps` | clamp applied to the commanded speed before the map | 20.0 (a model-validity bound, NOT a safety cap) |

None of these is null today, so the node starts. **Every one of the six PWM values is a
standard-RC-convention placeholder, not a measurement** (see that file's header block and
`docs/notes/hardware-arrival-checklist.md` section 3).

The full scale and the cap are two different numbers and were split apart on 2026-09-13
(GitHub issue #40). Before that the cap was also the full scale, and at 20 m/s that put a
commanded 1 m/s only 25 us off neutral -- plausibly inside the VESC's default PPM deadband,
so gentle keyboard teleop would have produced no motion at all and looked like a wiring
fault. **With `throttle_full_scale_mps` at 5.0 and the 1000/1500/2000 us ends, a commanded
1 m/s is now 100 us off neutral (1600 us forward, 1400 us reverse).** 5.0 is itself
unmeasured; it is replaced by the wheels-off-the-ground throttle sweep in step 13 of
`docs/notes/first-boot-runbook.md`.

`limits.global_speed_cap_mps` is unchanged in role: a commanded speed is clamped to it before
the map runs, exactly as before. A command above the full scale but below the cap is not
rejected -- it saturates the pulse at the channel end. Lower the cap before driving.

### The throttle map is open loop and provisional

The VESC is in PPM mode, where a pulse commands duty or current, **not speed**. There is no
feedback in this node and no claim that commanding X m/s produces X m/s. The linear
speed-to-pulse map scaled by `actuation.throttle_full_scale_mps` exists so the car can be driven at
all at first boot; it is replaced by the real closed-loop VESC driver
(`claude-docs/04-architecture.md`'s `vesc_node`) when that exists. USB to the VESC stays
telemetry and configuration only (`docs/notes/build-log.md`, 2026-09-12 command-path
decision).

### Steering polarity is bench-calibrated, not documented

No project doc says which pulse end is full LEFT. The node therefore takes a declared
parameter, `steering_left_is_pwm_max` (default `true` = `steering.pwm_max_us` is full left),
and it must be confirmed with the wheels off the ground before the car drives
(`docs/notes/first-boot-runbook.md`). Backwards, the car steers into whatever it was
avoiding.

### Parameters

All declared with descriptors and ranges (`claude-docs/10-conventions.md`).

| Parameter | Default | Notes |
|---|---|---|
| `output_rate_hz` | 50.0 | Servo frame rate AND the PWM carrier period. Not a test knob. |
| `drive_timeout_s` | 0.1 | `/drive` staleness -> neutral. Node tuning, deliberately NOT `limits.mux_watchdog_timeout_s` (that is the layer-1 MCU's own heartbeat window, a different mechanism on a different device). |
| `sysfs_root` | `/sys/class/pwm` | Only tests change this. |
| `steering_pwmchip` / `steering_pwm_channel` | 0 / 0 | UNVERIFIED, read off the device. |
| `throttle_pwmchip` / `throttle_pwm_channel` | 0 / 0 | UNVERIFIED, read off the device. |
| `steering_left_is_pwm_max` | true | Bench-calibrated, see above. |

## Enabling PWM pins on the Jetson (UNVERIFIED PROCEDURE)

**None of this has been run.** The Jetson was powered off when this was written; every
command below comes from NVIDIA's Jetson-IO documentation and the pattern
`tools/jetson_heartbeat/README.md` established for determining pin mappings on THIS board.
Treat the output of each step as the authority and correct this section the first time it is
actually done.

On the Jetson Orin Nano 40-pin header, **physical pins 15 and 33** are the two
hardware-PWM-capable pins. They do not come up as PWM: the pinmux has to be switched, which
`/opt/nvidia/jetson-io` does by editing the device tree overlay, and that needs a **reboot**.

1. Confirm what the header is currently configured as, the same way the heartbeat pin was
   determined (this reads the LIVE device tree, it does not guess from a pinout diagram):

   ```sh
   sudo /opt/nvidia/jetson-io/config-by-pin.py
   ```

   Note which functions pins 15 and 33 currently carry, and confirm nothing else needs them.
   Physical pin 7 must stay a plain GPIO: that is the heartbeat
   (`tools/jetson_heartbeat/`, `gpiochip0` line 144 / `PAC.06`), and repurposing it would
   silently disarm the mux's watchdog input.

2. Switch both pins to their PWM function:

   ```sh
   sudo /opt/nvidia/jetson-io/config-by-function.py
   ```

   Select `pwm` for the entries covering pins 15 and 33, save the configuration as a new
   device tree overlay when prompted, and reboot.

3. After the reboot, find out what the kernel actually called them. **Do not assume
   pwmchip0/pwm0** -- the numbering depends on which pins were enabled and on probe order:

   ```sh
   ls -l /sys/class/pwm/
   for chip in /sys/class/pwm/pwmchip*; do
     echo "$chip -> $(readlink -f "$chip" | sed 's#/sys/devices/##')  npwm=$(cat "$chip/npwm")"
   done
   ```

   The symlink target contains the SoC PWM controller address, which is what ties a chip
   number to a header pin. Record the mapping in `docs/notes/build-log.md` with the raw
   command output, the way the heartbeat pin mapping was recorded.

4. Verify one channel by hand before any ROS node touches it, with **nothing connected**:

   ```sh
   echo 0        | sudo tee /sys/class/pwm/pwmchipN/export
   echo 20000000 | sudo tee /sys/class/pwm/pwmchipN/pwm0/period      # 20 ms = 50 Hz
   echo 1500000  | sudo tee /sys/class/pwm/pwmchipN/pwm0/duty_cycle  # 1500 us neutral
   echo 1        | sudo tee /sys/class/pwm/pwmchipN/pwm0/enable
   ```

   Put a scope (or a meter that reads duty cycle) on the pin and confirm 50 Hz and a 1.5 ms
   high time before believing any of it. Then `echo 0 > .../enable`.

5. Give the container access. The node writes to `/sys/class/pwm`, which a container does not
   get by default; see `docker/car/build_on_jetson.md` for the exact `docker run` flags.

6. Pass the numbers you found as launch arguments, do not edit code:

   ```sh
   ros2 launch racer_bringup car_teleop.launch.py \
     steering_pwmchip:=N steering_pwm_channel:=M \
     throttle_pwmchip:=P throttle_pwm_channel:=Q
   ```

### Software PWM is NOT acceptable for steering

If the pinmux change turns out to be awkward, the tempting shortcut is to bit-bang the pulse
from userspace on an ordinary GPIO. Do not.

A hobby servo encodes its entire commanded range in the 1000 us window between a 1000 us and
a 2000 us pulse, so on this car's +/-0.4189 rad rack, **one microsecond of pulse error is
about 0.0008 rad of road-wheel angle**. A hardware PWM peripheral clocks the edge out of a
timer and its jitter is measured in nanoseconds. A userspace loop on a non-realtime Linux
kernel is at the mercy of scheduler preemption, CPU frequency changes, interrupt storms and
container CPU shares: tens to hundreds of microseconds of jitter is normal, and a single
missed deadline is a full-scale glitch on the pulse. That is jitter and glitching applied
directly to the steering rack of a moving car, and the mux cannot filter it out -- a
plausible-looking pulse of the wrong width is exactly what its validity check passes through.

Throttle is no better in principle; steering is called out because a glitched steering pulse
is a swerve, not a stumble. Hardware PWM on pins 15 and 33, or the node does not run.

## Cabling: Jetson 40-pin header -> mux board JETSON connector

Four wires, one connector. The mux board's JETSON header is a 4-pin male header with pin 1
marked (`firmware/safety_mux/README.md`, board connector map); the column/row coordinates
below are perfboard holes on **row 1**, in the planned layout drawn in
`docs/notes/build-log.md` (2026-09-12). **Planned, unverified: nothing is soldered.**

| Signal | Jetson 40-pin header | Mux board hole | Mux side |
|---|---|---|---|
| Steering pulse | **physical pin 15** (hardware PWM) | column 8, row 1 | level shifter -> RP2040 GPIO 10 |
| Throttle pulse | **physical pin 33** (hardware PWM) | column 9, row 1 | level shifter -> RP2040 GPIO 7 |
| Heartbeat | **physical pin 7** (`gpiochip0` line 144, `PAC.06`) -- existing `racer-heartbeat.service`, `tools/jetson_heartbeat/` | column 10, row 1 | direct (already 3.3 V) -> RP2040 GPIO 5 |
| Ground | **physical pin 9** | column 11, row 1 | the one shared ground net |

Notes that matter more than the table:

- **There is no 5 V wire.** The Jetson powers itself; the mux board must never back-feed it.
- **The connector can be plugged in reversed**, which swaps heartbeat and steering, and no
  firmware check can see that. Pin 1 is marked on the board, the plug is keyed, and the mark
  gets checked before every plug-in.
- The ground wire is a shared reference, not a second supply: all three signals are voltages
  measured against the Jetson's zero.
- All three Jetson inputs are pulled down on the Pico, so an unplugged or broken cable reads
  as a steady low -- no pulses, no heartbeat toggle -- which the watchdog and the PWM
  validity check both treat as a cut. That is the intended failure mode and it is a
  prediction until the "unplug the Jetson cable mid-run" bench test is actually run.

Cross-reference: `firmware/safety_mux/README.md`'s "Board connector map" is the authority on
the board side of this connector; this table is the authority on which Jetson pin each wire
leaves from.

## Layout

```
racer_drivers/
├── README.md                      this file
├── include/racer_drivers/         ROS-free, sysfs-free headers
│   ├── pwm_mapping.hpp            angle/speed -> pulse, clamping, staleness, refuse-on-null
│   ├── pwm_sink.hpp               the PwmChannelSink interface + in-memory fake + sysfs impl
│   └── pwm_output_driver.hpp      fail-closed sequencing over two sinks
├── src/                           those three, plus pwm_output_node.cpp (all the ROS)
└── test/
    ├── test_pwm_mapping.cpp       L1 gtest: table-driven, bounds/epsilon/NaN/inf
    └── test_pwm_output_node_launch.py  L3 launch_testing against a fake sysfs tree
```
