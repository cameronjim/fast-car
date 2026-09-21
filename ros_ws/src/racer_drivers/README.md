# racer_drivers

On-vehicle drivers (`claude-docs/02-repo-layout.md`). One node so far.

## `pwm_output_node` -- the tail of the command path

```
safety_node --/drive--> pwm_output_node --2x 250 Hz PWM--> layer-1 mux board --2x 50 Hz--> servo + VESC PPM
```

It subscribes to `/drive` (reliable, depth 10) and to nothing else on the command path, and
converts `steering_angle` (rad, LEFT positive per `claude-docs/06-vehicle-params.md`) and
`speed` (m/s) into two servo pulses driven by the Jetson's hardware PWM through the Linux
sysfs PWM interface, on a 4 ms (250 Hz) frame. The frame rate is a resolution decision on the
Jetson-to-Pico link and nothing more -- the mux regenerates its own 50 Hz pulses to the servo
and the ESC, so that is still what the actuators see. See "Actuator resolution" below
(GitHub issue #66).

**It never subscribes to `/drive_raw`.** Doing so would put an ungated command on the wire
and route around `safety_node`, which is the bypass `CLAUDE.md` invariant 1 forbids. The L3
launch tests assert this against the live node graph, not by reading the source.

### Status

**NODE VERIFIED ON HARDWARE AS FAR AS THE MUX, 2026-09-21.** This node ran on the real Jetson
out of the `car` image and drove both channels through the full `/drive_raw -> safety_node ->
/drive` path, with the safety-mux Pico (DIAG_BUILD) confirming the pulses arrive: neutral
reads `STEER gp10=1484us FRESH` / `THR gp7=1485us FRESH`, a 0.2 rad steering command produces
exactly the 1738720 ns `pwm_mapping` predicts, and 0.5 m/s produces exactly 1550000 ns. Two
bugs in `pwm_sink.cpp` were found doing it (a udev permissions race on a freshly exported
channel, and an `enable` write rejected while `period` was still 0 on pwmchip2) and are fixed;
see `docs/notes/build-log.md`'s 2026-09-21 evening entry and the comments in `start()`.

What that still does NOT cover: the servo and ESC were unpowered and the servo lead unplugged
throughout, so **no wheel has moved under this node's command** and, AT THE TIME, the steering
polarity was confirmed only at the pin, not at the wheels. Note also that at the OLD,
provisional 1000/1500/2000 us endpoints a full-lock 2000 us command read as 2031 us at the mux
and tripped its `OUT_OF_RANGE` check -- see the runbook's "Reading the mux numbers".

**STEERING ENDPOINTS AND SIGN MEASURED AT THE WHEELS, 2026-09-21 evening.** With the wheels
off the ground and the mux armed, the steering channel's mechanical limits and its sign were
both measured for the first time (`docs/notes/bench-session-2026-09-20.md`): left mechanical
stop 1094 us, right mechanical stop 1875 us, neutral 1500 us (actual 1484 us at the mux,
quantisation), and **a SHORTER pulse turns the wheels LEFT** -- the opposite of what the
mapping had been assuming (a positive, LEFT, `steering_angle` had been going to a LONGER
pulse; 0.2 rad measured 1719 us, above neutral, before this fix). Both the endpoints and the
sign now live in `config/vehicle_params.yaml` (`steering.pwm_min_us` / `pwm_max_us` /
`pwm_left_bound`), not in this node. The new, narrower endpoints also resolve the
`OUT_OF_RANGE` issue above: full lock now produces 1094 or 1875 us, both comfortably inside
the mux's 1000-2000 us validity window.

The earlier pin-identity note, still accurate: as of 2026-09-20 the
pinmux procedure below has actually been run on the real Jetson Orin Nano Super Dev Kit
(JetPack 6.2 / L4T R36.4.4): header pins 15 and 33 are confirmed enabled and driven, and the
`pwmchip0` = pin 15 / `pwmchip2` = pin 33 mapping this node now defaults to is a measurement,
not a guess (`docs/notes/build-log.md`, 2026-09-20). What that measurement covers: the pads
carry a PWM signal at the expected average voltage for 50 Hz / 50 percent duty, driven by
hand through sysfs. What it does NOT cover: pulse-width accuracy under load, this node's own
behaviour driving those channels, or anything downstream of the pins (servo, mux board,
VESC). No pin has been scoped with this node running, no servo has moved, no ESC has been
armed by it.

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
| `steering.pwm_min_us` / `pwm_neutral_us` / `pwm_max_us` | steering pulse ends and neutral | 1094 / 1500 / 1875 (MEASURED 2026-09-21) |
| `steering.pwm_left_bound` | which pulse end is full LEFT | `pwm_min_us` (MEASURED 2026-09-21: shorter pulse is left) |
| `steering.min_angle_rad` / `max_angle_rad` | angle range the pulse ends correspond to | -0.4189 / +0.4189 (gym defaults) |
| `actuation.throttle_pwm_min_us` / `throttle_pwm_neutral_us` / `throttle_pwm_max_us` | throttle pulse ends and neutral | 1000 / 1500 / 2000 (PROVISIONAL, unmeasured) |
| `actuation.throttle_full_scale_mps` | full-scale reference for the open-loop speed map | 5.0 (PROVISIONAL, unmeasured) |
| `limits.global_speed_cap_mps` | clamp applied to the commanded speed before the map | 20.0 (a model-validity bound, NOT a safety cap) |
| `actuation.steering_pwm_period_us` / `throttle_pwm_period_us` | PWM frame period written to each channel's sysfs `period` | 4000 / 4000 (250 Hz; sets the pulse grid, see "Actuator resolution") |

None of these is null today, so the node starts. The four steering PWM fields (endpoints,
neutral, and the sign) are now MEASURED, 2026-09-21 (`docs/notes/bench-session-2026-09-20.md`).
**The three throttle PWM values are still a standard-RC-convention placeholder, not a
measurement** (see that file's header block and `docs/notes/hardware-arrival-checklist.md`
section 3).

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

### Steering polarity is measured, not a code default

**MEASURED 2026-09-21** (`docs/notes/bench-session-2026-09-20.md`, wheels off the ground, mux
armed): a SHORTER pulse turns the wheels LEFT, so `steering.pwm_left_bound` is `"pwm_min_us"`.
This is no longer a node parameter -- it used to be a declared parameter,
`steering_left_is_pwm_max` (default `true`, an unmeasured guess), and it moved into
`config/vehicle_params.yaml` once it was actually measured, per `CLAUDE.md` invariant 2 (a
sign convention is a physical constant, not a code default). The node reads it from the
generated binding and refuses to start if the field is not exactly `"pwm_min_us"` or
`"pwm_max_us"`. Backwards, the car steers into whatever it was avoiding, which is why this was
verified with the wheels off the ground before anything else.

### Actuator resolution: the frame period sets the pulse grid (GitHub issue #66)

The Jetson's Tegra PWM controller expresses duty as an **8-bit fraction of the period**
(Linux `pwm-tegra.c`, `PWM_DUTY_WIDTH 8`). A commanded pulse width therefore lands on the
nearest multiple of `period / 256`, and nothing above the driver can recover the difference.
At the 20 ms frame this node used until 2026-09-21 that step is 78.125 us, which is about
**13 usable positions across 1000-2000 us**: 2.3 deg of steering per step on a +-30 deg rack,
and a throttle that behaves like a switch, because the first steps off neutral fall inside
the VESC's 15 percent current deadband and the sensorless motor does not catch until roughly
1700-1800 us.

Measured at the mux on 2026-09-21, at the 20 ms frame:

| commanded | measured at the Pico | grid steps |
|---|---|---|
| 1000 us | 1016 us | 13 x 78.125 |
| 1500 us | 1484 us | 19 x 78.125 |
| 2000 us | 2031 us | 26 x 78.125 |

**The fix is a shorter frame on the Jetson side, and it costs nothing downstream.** The
safety mux captures the INPUT pulse width from edge timestamps
(`firmware/safety_mux/pico/pwm_capture.c`, `time_us_64`, 1 us resolution) and **regenerates**
its own outputs to the servo and the ESC at a fixed 50 Hz / 20 ms frame
(`firmware/safety_mux/pico/pwm_output.c`, `top=39999` at 0.5 us per count). The servo and the
VESC never see the Jetson's frame rate at all -- only the Jetson-to-Pico link does. So the
frame period is free to be whatever gives the best resolution, within the limits below.

The achievable grid for a given frame, which `pulse_grid_step_us()` in `pwm_mapping.hpp`
computes and the node logs at startup:

| frame period | frame rate | pulse grid step | positions across 1000-2000 us |
|---|---|---|---|
| 20000 us | 50 Hz | 78.125 us | about 13 |
| 8000 us | 125 Hz | 31.25 us | about 32 |
| **4000 us** | **250 Hz** | **15.625 us** | **about 64** |
| 2000 us | 500 Hz | 7.8125 us | about 128 (but see below) |

Both channels are configured at **4000 us** in `config/vehicle_params.yaml`
(`actuation.steering_pwm_period_us`, `actuation.throttle_pwm_period_us`). Steering gets the
short frame too, for the same reason the throttle does: the analog Traxxas 2075 servo is
behind the mux's regenerated output and never sees this frame either, so leaving steering at
20 ms would keep 2.3 deg per step in exchange for nothing.

**Why not shorter than 4000 us.** A 2000 us pulse has to fit inside its frame with a clear
low gap after it, because the mux measures the gap's edges. At 4000 us, full throttle is 50
percent duty with a 2 ms gap. Shorter frames push that duty towards 100 percent, and a merged
pair of frames (two consecutive missed edge interrupts on the Pico) stops being rejected by
`pwm_capture.c`'s 200-5000 us plausibility band and starts looking like a plausible pulse.
`validate_config()` enforces the same rule from this side: a frame period shorter than twice
its channel's own `pwm_max_us` is refused at startup, named, and not repaired.

**What this does NOT change.** The mapping is period-independent -- a pulse width in
microseconds is the same pulse width at any frame rate, and the L1 tests pin that. The duty
values this node writes are identical before and after; only what the peripheral can round to
changes. The command update cadence is still 50 Hz (`output_rate_hz`), and the mux's outputs
are still 50 Hz.

Verify it on the bench before driving: `docs/notes/first-boot-runbook.md`, "Launch and
drive".

### Parameters

All declared with descriptors and ranges (`claude-docs/10-conventions.md`).

| Parameter | Default | Notes |
|---|---|---|
| `output_rate_hz` | 50.0 | How often the two duty cycles are REWRITTEN (the command update cadence). No longer the PWM carrier period: that is `actuation.steering_pwm_period_us` / `throttle_pwm_period_us` in `config/vehicle_params.yaml` (issue #66). Not a test knob. |
| `drive_timeout_s` | 0.1 | `/drive` staleness -> neutral. Node tuning, deliberately NOT `limits.mux_watchdog_timeout_s` (that is the layer-1 MCU's own heartbeat window, a different mechanism on a different device). |
| `sysfs_root` | `/sys/class/pwm` | Only tests change this. |
| `steering_pwmchip` / `steering_pwm_channel` | 0 / 0 | VERIFIED on the Jetson Orin Nano Super Dev Kit, 2026-09-20: header pin 15. |
| `throttle_pwmchip` / `throttle_pwm_channel` | 2 / 0 | VERIFIED, same device and date: header pin 33. |

Steering polarity is no longer a node parameter -- see "Steering polarity is measured, not a
code default" above.

## Enabling PWM pins on the Jetson (VERIFIED PROCEDURE, confirmed 2026-09-20)

**This has been run**, on the actual Jetson Orin Nano Super Dev Kit, JetPack 6.2 / L4T
R36.4.4. The commands and the mapping below are a record of what was done and measured, not
a written-ahead guess; the raw output is also logged in `docs/notes/build-log.md`,
2026-09-20 entry.

On the Jetson Orin Nano 40-pin header, `sudo python3 /opt/nvidia/jetson-io/config-by-
function.py -l all` reports exactly three PWM-capable functions: `pwm1` on physical pin 15,
`pwm5` on physical pin 33, `pwm7` on physical pin 32. This board uses pins 15 and 33 (see
"Cabling" below); pin 32 / `pwm7` is unused. **Out of the box none of these functions are
enabled on the 40-pin header**, and this is the trap: a PWM controller can be exported and
enabled in sysfs and still read 0.0 V, because the kernel happily runs a controller whose
output the pinmux has not routed to a pad. The pinmux has to be switched, which
`/opt/nvidia/jetson-io` does by editing the device tree overlay, and that needs a **reboot**.

1. List what the header currently supports and what is enabled (this reads the LIVE device
   tree, it does not guess from a pinout diagram):

   ```sh
   sudo python3 /opt/nvidia/jetson-io/config-by-function.py -l all
   ```

   Confirm pins 15 and 33 are the two entries you expect (`pwm1`, `pwm5`) and that nothing
   else needs them. Physical pin 7 must stay a plain GPIO: that is the heartbeat
   (`tools/jetson_heartbeat/`, `gpiochip0` line 144 / `PAC.06`), and repurposing it would
   silently disarm the mux's watchdog input.

2. Enable both PWM functions in one shot and reboot:

   ```sh
   sudo python3 /opt/nvidia/jetson-io/config-by-function.py -o dt 1="pwm1 pwm5"
   ```

   This writes `/boot/jetson-io-hdr40-user-custom.dtbo` and adds it to
   `/boot/extlinux/extlinux.conf`, so it **survives reboots** -- it is a one-time step, not
   something to redo every boot. Reboot for the overlay to take effect.

3. After the reboot, confirm both functions are enabled and find the kernel's own numbering.
   **Do not assume pwmchip0/pwm0** -- the numbering depends on which pins were enabled and on
   probe order, and it is confirmed, not assumed, on this board:

   ```sh
   sudo python3 /opt/nvidia/jetson-io/config-by-function.py -l enabled
   for chip in /sys/class/pwm/pwmchip*; do
     echo "$chip -> $(readlink -f "$chip" | sed 's#/sys/devices/##')  npwm=$(cat "$chip/npwm")"
   done
   ```

   **Measured mapping on this device (2026-09-20):** `pwmchip0` = `3280000.pwm` = header pin
   15 = **steering**; `pwmchip2` = `32c0000.pwm` = header pin 33 = **throttle**
   (`pwmchip1` = `32a0000.pwm`, `pwmchip3` = `32e0000.pwm`, `pwmchip4` = `39c0000.tachometer`
   are present but unused). Each chip has `npwm=1`, so the channel index within each chip is
   always 0. This is the mapping `pwm_output_node`'s defaults now encode
   (`steering_pwmchip=0`/`steering_pwm_channel=0`, `throttle_pwmchip=2`/
   `throttle_pwm_channel=0`). Confirm it again on any other unit or kernel build before
   trusting the defaults there -- the symlink target contains the SoC PWM controller address,
   which is what actually ties a chip number to a header pin.

4. Verify each channel by hand before any ROS node touches it, with **nothing connected**:

   ```sh
   echo 0        | sudo tee /sys/class/pwm/pwmchipN/export
   echo 4000000  | sudo tee /sys/class/pwm/pwmchipN/pwm0/period      # 4 ms = 250 Hz, the
                                                                     # configured frame
   echo 2000000  | sudo tee /sys/class/pwm/pwmchipN/pwm0/duty_cycle  # 50 percent duty
   echo 1        | sudo tee /sys/class/pwm/pwmchipN/pwm0/enable
   ```

   **Confirmed on this device:** a DC meter across the pin and its adjacent ground pin reads
   ~1.64 V on both channels -- pin 15 referenced to pin 14, pin 33 referenced to pin 34 --
   which is the expected average of a 3.3 V square wave at 50 percent duty (0.5 x 3.3 V =
   1.65 V). Reading 0.0 V here before the pinmux change in step 2 is the expected failure
   mode, not a wiring fault: it means the controller is toggling in sysfs but the pad is not
   yet routed. Then `echo 0 | sudo tee /sys/class/pwm/pwmchipN/pwm0/enable`.

   What this step verifies: the pads carry a PWM signal at the right average voltage. What
   it does NOT verify: pulse-width accuracy under load, `pwm_output_node`'s own behaviour
   driving these channels, or anything downstream of the pins.

5. Give the container access. The node writes to `/sys/class/pwm`, which a container does not
   get by default; see `docker/car/build_on_jetson.md` for the exact `docker run` flags.

6. The numbers above are now the node's defaults, so a plain
   `ros2 launch racer_bringup car_teleop.launch.py` uses them. Override only if a different
   unit measures a different mapping:

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
