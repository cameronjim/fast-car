# firmware/safety_mux -- layer-1 safety mux (DRAFT, roadmap task 1.3)

## Status: UNVERIFIED ON HARDWARE

**This firmware has never run on a real RP2040, has never been connected to a real RC
receiver/servo/ESC, and has never been through the roadmap 1.3 kill test.** Nothing here may
be treated as a working safety system. Per `claude-docs/05-safety.md`:

> Only layer 1 is a guarantee. ... Kill is tested by ACTUALLY freezing the Jetson (roadmap
> 1.3), not by reasoning about it.

The only thing that makes this firmware "real" is that test: physically freezing the Jetson
(e.g. `sudo systemctl stop`-ing everything, or pulling its network/killing its process tree
outright) and confirming the mux cuts drive and steering PWM to the ESC/servo, with a human
present and the wheels off the ground. That has not happened. Until it has, roadmap task 1.3
stays `[~]` in `claude-docs/01-roadmap.md`, not `[x]`, no matter how much of this directory
exists.

This directory is authored now, ahead of the hardware, for the same reason
`docker/train-cuda/` was (see that image's own `[~]` roadmap note): so that when the BOM
parts in `claude-docs/11-hardware.md` arrive, there is something to assemble, flash, and
bench-test against instead of a blank firmware project.

## What is and isn't tested

| Piece | Where | Tested how |
|---|---|---|
| Mux state machine, watchdog timing, PWM validity checks, RC switch interpretation (including the kill-switch hysteresis band), param null/finiteness/range checking | `logic/` | Host-compiled with plain `gcc` (no Pico SDK, no cross-compiler), table-driven, every branch exercised -- see `tests/`. Runs in CI on every push (`.github/scripts/safety_mux_host_tests.sh`). |
| GPIO/PWM capture (including the staleness window and the plausibility band), PWM output, heartbeat input, power-cutoff GPIO, startup ordering, main loop | `pico/` | **Compiles, otherwise untested.** Cross-compiled clean (no warnings) against pico-sdk 2.1.0 and flashed to nothing -- see "Building" below and `docs/notes/safety-mux-first-build.md`. Compiling is not testing: this is real hardware/interrupt access with no host equivalent, it has never run on a chip, and that note records a known IRQ-handler collision between `pwm_capture.c` and `heartbeat_input.c` that a compiler cannot see (since fixed), and `docs/notes/firmware-review-2026-09-14.md` records eight more defects of the same kind -- floating output pins at boot, a captured pulse that never aged out, torn multi-word reads from interrupt context -- all fixed, none of them yet observed on a chip. |
| The whole thing, on a Jetson, RC receiver, servo, and ESC | (nothing yet) | Roadmap 1.3's kill test, `claude-docs/12-testing.md` L6/L7. Pending hardware. |

## Building

The Pico target cross-compiles inside a container so the toolchain is reproducible and does
not depend on what happens to be installed on the build machine. From the repo root:

```sh
docker run --rm -v "$PWD":/repo -w /repo debian:bookworm bash -c '
  apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
    gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib \
    cmake build-essential git python3 python3-yaml python3-jsonschema &&
  cmake -S firmware/safety_mux -B firmware/safety_mux/build &&
  cmake --build firmware/safety_mux/build -j"$(nproc)"'
```

Output: `firmware/safety_mux/build/safety_mux_firmware.uf2`. The build tree and
`build-artifacts/` are gitignored; the `.uf2` is build output and is never committed.
`CMakeLists.txt` pins pico-sdk to release tag 2.1.0 via `FetchContent` and runs
`tools/gen_params.py` itself, so there is nothing to install or generate first.

**As of 2026-09-14 this firmware ARMS: no fault blink.** The nine `vehicle_params` fields the
mux needs were filled in with PROVISIONAL, UNMEASURED standard-RC values (1000/1500/2000 us,
100 ms watchdog, 1500 us kill threshold) so a first bench test is possible. With nothing
connected it sits in the CUT state driving 50 Hz / 1500 us neutral on GPIO 1 and 3 with the
GPIO 0 cutoff low. **Wheels off the ground**: those numbers are convention, not measurement,
and must be replaced per `docs/notes/hardware-arrival-checklist.md` section 3 before the car
drives. The refuse-to-arm guard is untouched -- revert any of those fields to `null` and it
fast-blinks again. See `docs/notes/safety-mux-first-build.md` for exactly what to expect on
flashing and what it does and does not prove.

CI (`safety-mux-host-tests` job in `.github/workflows/ci.yml`) builds and runs `logic/` +
`tests/` with `gcc -Wall -Wextra -Werror -Wpedantic` on `ubuntu-latest`. It does **not**
attempt a Pico SDK cross-compile: standing up `arm-none-eabi-gcc` plus a `FetchContent`'d
`pico-sdk` in CI is real infrastructure, and the actual safety claim rests entirely on
`logic/`'s decision logic being correct, not on whether `pico/` happens to compile. Promoting
this to a real cross-compile CI job (`CMakeLists.txt` is already written for it) is a
reasonable next step once this firmware is closer to bench-tested than drafted.

## Design (per `claude-docs/05-safety.md`)

- This MCU shares no code, power rail, or failure mode with the Jetson. It never subscribes
  to, parses, or trusts anything from ROS -- the only Jetson-originated signals it reads are
  raw PWM (steering, throttle) and a raw digital heartbeat toggle, exactly as electrical
  signals, never as messages.
- No software task may reconfigure, reflash, or route around this MCU. Nothing in this repo
  writes to it except a human with a debug probe/USB, by hand, on the bench.
- Decision priority, checked in this order every cycle (`logic/src/mux_decision.c`):
  1. RC kill switch (or an unreadable kill-switch channel, treated identically) -- the only
     input a human directly holds.
  2. Jetson heartbeat watchdog -- catches a frozen/hung/crashed Jetson.
  3. Per-channel Jetson PWM validity -- catches a glitched-but-alive command signal. A
     channel is invalid if its pulse is out of range OR if no plausible pulse has arrived
     recently: a stuck-high line produces no edges, and the last width captured before it
     stuck is not a live command.
  4. Otherwise: passthrough.

  The kill switch has a dead band around its threshold (`logic/rc_switch.c`,
  `RC_SWITCH_DEFAULT_HYSTERESIS_US`, currently a compile-time 100 us): ARMED at or above
  threshold + band, KILL below threshold - band, hold in between. Holding never holds ARMED
  out of an unknown state, so a switch parked in the band at power-on reads KILL.
- A cut is a cut: both PWM outputs go to a configured neutral value (not merely "no signal",
  which would depend on the servo/ESC's own undocumented failsafe behavior) and the power
  cutoff GPIO is asserted, together, every time. There is no partial-cut state.
- Every output pin is driven to a defined safe state in the first statements of `main()`,
  before `stdio_init_all()` and before any param is read: an RP2040's GPIOs come out of reset
  as high-impedance inputs, a floating servo input can twitch the servo, and a floating ESC
  input is what some ESCs arm on. A refusal to arm therefore holds the outputs inert rather
  than floating.
- Physical constants (PWM ranges, neutral values, the watchdog timeout, the kill-switch
  threshold) are never hand-typed here (`CLAUDE.md` invariant 2). They come from
  `config/vehicle_params.yaml` via `tools/gen_params.py`'s generated C binding
  (`pico/main.c`'s `raw_fields_from_generated_params()`), and `logic/mux_params.c` refuses to
  produce a usable `MuxParams` if any required field is still `null`, is NaN/infinite, or is
  structurally unusable (a neutral outside its own channel range, a non-positive watchdog
  timeout, a kill threshold outside the receiver's range) -- `pico/main.c` halts forever
  (blinking a fault LED, printing which field and which of those three problems it has, over
  USB serial) rather than silently assuming a default. All of these fields are `null` in the committed `config/vehicle_params.yaml` right
  now (see that file's comments): this firmware **cannot arm** until they are bench-measured
  and filled in.

## Pinout (confirmed against the as-built perfboard, 2026-09-20)

Pin numbers are RP2040 GPIO numbers, matched exactly in `pico/main.c`'s `#define`s -- change
one, change both. As of 2026-09-20 the perfboard is soldered and this mapping has been
confirmed by the owner against the physical board; this table and `pico/main.c`'s `#define`s
were updated together for that confirmation.

`pico/main.c`'s seven `#define`s are the ONLY place a GPIO number reaches the firmware: nothing
in `logic/`, in `tests/`, or in the build depends on one, so a remap is a one-file code change.
It is not a one-file CHANGE, though, because the numbers are also written down in prose in
`planning-docs/05-safety-mux-and-kill-test.md`, `planning-docs/06-vesc-config-and-jetson-bringup.md`,
`ros_ws/src/racer_drivers/README.md`'s cabling table, and this file's own connector map below --
those are the live references and are updated in the same change. `docs/notes/safety-mux-first-build.md`
and `docs/notes/build-log.md`'s dated entries are historical records of what the numbers were on the
date they were written and are left as recorded; a remap gets its own new dated `build-log.md` entry
instead (see the 2026-09-20 entry).

| Signal | Direction | RP2040 GPIO | Notes |
|---|---|---|---|
| RC receiver kill-switch channel | in | GPIO 12 | PWM capture (interrupt-timed). Receiver's own valid PWM range and this channel's ARMED/KILL threshold are bench-measured, not assumed (`config/vehicle_params.yaml`'s `limits.mux_kill_switch_threshold_us`). |
| Jetson steering PWM | in | GPIO 10 | PWM capture. Range = `vehicle_params.steering.pwm_{min,max}_us`. |
| Jetson throttle PWM | in | GPIO 7 | PWM capture. Range = `vehicle_params.actuation.throttle_pwm_{min,max}_us`. |
| Jetson heartbeat | in | GPIO 5 | Raw digital toggle from a lightweight Jetson-side process (see `pico/heartbeat_input.h`'s comment) -- NOT a UART message, NOT ROS. Timeout = `vehicle_params.limits.mux_watchdog_timeout_s`. Jetson-side source: `tools/jetson_heartbeat/`, driven off Jetson 40-pin header **physical pin 7** (`gpiochip0` line 144, kernel name `PAC.06`; pin 9 is the shared ground) -- see that tool's README for how the pin mapping was determined and verified. |
| Servo PWM out | out | GPIO 1 | 50 Hz hardware PWM to the steering servo. |
| ESC PWM out | out | GPIO 3 | 50 Hz hardware PWM to the ESC. |
| Power cutoff | out | GPIO 0 | Active-HIGH = power enabled (fail-safe: a dead/reset RP2040 or a browned-out driver circuit defaults this LOW = cut). Drives a relay or high-side MOSFET gate in the motor/servo power path -- exact drive circuit is a bench decision, not fixed here. |
| Fault LED | out | Pico's onboard LED (`PICO_DEFAULT_LED_PIN`) | Fast blink = refused to arm (missing `vehicle_params` field), see `pico/main.c`'s `fault_halt_missing_param()`. |

### Board connector map (confirmed against the as-built perfboard, 2026-09-20)

The pinout above is GPIO numbers. This is how those GPIOs reach the outside world on the
perfboard, which is now soldered. This mapping has been confirmed by the owner against the
physical board (2026-09-20); it has not yet been powered or bench-checked.

| Header | Pins | Carries | Notes |
|---|---|---|---|
| KILL | 3-pin (SIG, +5V, GND) | RC receiver's kill-switch channel | The board **powers the receiver** through this lead off the UBEC 5 V rail. SIG goes through the level shifter into GPIO 2. |
| JETSON | 4-pin (STEER SIG, THROTTLE SIG, HEARTBEAT, GND) | all three Jetson-originated signals plus their shared return | Pin 1 (STEER SIG) is marked on the board and the plug is keyed, because a reversed plug swaps heartbeat and steering and nothing in firmware can see that. No 5 V pin: **the Jetson powers itself.** Steering and throttle go through the shifter into GPIO 3 and 4; the heartbeat is already 3.3 V and goes straight to GPIO 5. On the Jetson side this connector is fed by four 40-pin-header pins: STEER SIG from **physical pin 15**, THROTTLE SIG from **physical pin 33** (the two hardware-PWM-capable pins, driven by `ros_ws/src/racer_drivers/pwm_output_node` -- see that package's README for the pinmux procedure and the hole-by-hole cabling table), HEARTBEAT from **physical pin 7** and GND from **physical pin 9** -- see `tools/jetson_heartbeat/README.md` for how the pin 7 mapping was determined and verified. The two PWM pins were confirmed on the Jetson on 2026-09-20: the pinmux change is applied and persists across reboots, and pin 15 (`pwmchip0`, steering) and pin 33 (`pwmchip2`, throttle) both measured the expected ~1.64 V at 50 Hz / 50 percent duty (`docs/notes/build-log.md`, 2026-09-20). That confirms the pads are driven at the right average voltage from the Jetson side; it does not confirm anything about this connector, the level shifter, or this board -- the Jetson-to-mux cable in this table is still unsoldered and unverified. |
| SERVO | 3-pin (SIG, +5V, GND) | steering servo | The board **powers the servo** from the same 5 V rail. SIG is GPIO 6's output. |
| VESC | 3-pin, **+5V position left EMPTY** | ESC PPM input | The VESC has its own BEC; connecting its middle pin to the board's 5 V would tie two supplies together. Only SIG (GPIO 7) and GND are populated. |
| CUTOFF | 2-pin (SIG, GND) | power-cutoff drive circuit (GPIO 8) | Reserved. The relay/MOSFET stage does not exist yet; the header is there so it does not need re-soldering later. |
| KILL | 3-pin (SIG, +5V, GND) | RC receiver's kill-switch channel | The board **powers the receiver** through this lead off the UBEC 5 V rail. SIG goes through the level shifter into GPIO 12. |
| JETSON | 4-pin (STEER SIG, THROTTLE SIG, HEARTBEAT, GND) | all three Jetson-originated signals plus their shared return | Pin 1 (STEER SIG) is marked on the board and the plug is keyed, because a reversed plug swaps heartbeat and steering and nothing in firmware can see that. No 5 V pin: **the Jetson powers itself.** Steering and throttle go through the shifter into GPIO 10 and 7; the heartbeat is already 3.3 V and goes straight to GPIO 5. On the Jetson side this connector is fed by four 40-pin-header pins: STEER SIG from **physical pin 15**, THROTTLE SIG from **physical pin 33** (the two hardware-PWM-capable pins, driven by `ros_ws/src/racer_drivers/pwm_output_node` -- see that package's README for the pinmux procedure and the hole-by-hole cabling table), HEARTBEAT from **physical pin 7** and GND from **physical pin 9** -- see `tools/jetson_heartbeat/README.md` for how the pin 7 mapping was determined and verified. The two PWM pins are UNVERIFIED: the pinmux change and the resulting chip numbering have not been done on the board. |
| SERVO | 3-pin (SIG, +5V, GND) | steering servo | The board **powers the servo** from the same 5 V rail. SIG is GPIO 1's output. |
| VESC | 3-pin, **+5V position left EMPTY** | ESC PPM input | The VESC has its own BEC; connecting its middle pin to the board's 5 V would tie two supplies together. Only SIG (GPIO 3) and GND are populated. |
| CUTOFF | 2-pin (SIG, GND) | power-cutoff drive circuit (GPIO 0) | Reserved. The relay/MOSFET stage does not exist yet; the header is there so it does not need re-soldering later. |
| 5V / GND | 2-pin screw terminal | UBEC 5 V in | The whole board's supply. This is the rail a Jetson or compute-rail failure cannot take down. |

**Pico orientation on the board (planned, unverified on hardware).** The Pico sits with its
USB socket facing the board's **left edge**. Seen from the top, component side up, that puts
**pins 1 to 20 along the lower row** (pin 1 at the bottom-left) and **pins 40 down to 21 along
the upper row**. Every GPIO this board uses (GPIO 0, 1, 3, 5, 7, 10, and 12) is on the
**lower row**; the upper
row is the power side (VBUS, VSYS, 3V3 OUT, RUN). Get this backwards and the whole signal
harness lands on the power pins.

Wires run on the **underside**, the copper face: components sit on top, so a wire may pass
straight under the Pico's body, which is how the level-shifted signals and the heartbeat reach
the lower row. Related, and the easier mistake to make: **flipping the board to solder mirrors
the columns left-to-right**, so column 1 ends up on the right. Mark column 1 and row 1 on both
faces with a paint pen before soldering and count from the marked corner every time.

One ground net ties the screw terminal, both shifter ground pins, the Pico's grounds, and
every header's ground together. The Jetson's fourth wire is that shared reference, not a
second power wire: a voltage is a difference against a ground, so all three of its signals
need the Jetson and the Pico to agree on where zero is.

The three Jetson inputs are **pull-down** on the Pico side. An unplugged (or broken) Jetson
cable therefore reads as a steady low: no PWM pulses and no heartbeat toggle, which the
watchdog and the PWM validity check both treat as a cut. That is the intended failure mode,
and it is the thing the bench test "unplug the Jetson cable mid-run" exists to prove, not
assume.

Per `claude-docs/11-hardware.md`'s wiring rules:

- This MCU and the RC receiver are powered from a rail that a Jetson or compute-rail failure
  cannot take down (e.g. the receiver's own BEC off the drive battery, NOT the Jetson's 12V/5V
  buck). This is a wiring decision for the bench, not something firmware enforces or can
  enforce.
- Every connector polarized/keyed; the actual harness gets photographed and committed to
  `docs/notes/` once it exists (not yet -- no hardware).
- The power-cutoff drive circuit (relay vs. MOSFET, gate drive voltage, flyback protection if
  a relay) is not specified here -- it is a bench/schematic decision for
  `claude-docs/11-hardware.md`'s "Bench discipline (Desktop B)" work, out of scope for a
  firmware draft.

## Layout

```
firmware/safety_mux/
├── README.md              this file
├── CMakeLists.txt         Pico SDK build (cross-compiles clean, see "Building")
├── logic/                 pure C, zero Pico SDK dependency, host-buildable and host-tested
│   ├── include/safety_mux/*.h
│   └── src/*.c
├── tests/                 host test runner (plain C, no external framework) + table-driven
│   │                      suites, one per logic/ unit
│   ├── framework.h
│   ├── main.c
│   └── test_*.c
└── pico/                  Pico SDK glue: GPIO/PWM capture+output, heartbeat input, power
                           cutoff, main loop. UNVERIFIED ON HARDWARE.
```

Run the host tests locally: `.github/scripts/safety_mux_host_tests.sh` (needs only `gcc`, no
Pico SDK, no `cmake`).

## What still has to happen before this is real (roadmap 1.3)

1. Assemble the chassis/ESC/servo/RC receiver (roadmap 1.1).
2. Bench-measure the PWM ranges, neutral values, kill-switch threshold, and pick a watchdog
   timeout; write them into `config/vehicle_params.yaml` (replacing the `null`s).
3. ~~Build `pico/` for real with a Pico SDK toolchain~~ (done, see "Building" and
   `docs/notes/safety-mux-first-build.md`) and flash an actual RP2040 (not done).
4. Bench-test each I/O path individually (PWM capture reads sane values, PWM output drives
   the servo/ESC correctly, power cutoff actually cuts) -- `claude-docs/12-testing.md` L6.
   `docs/notes/firmware-review-2026-09-14.md` lists what the 2026-09-14 review changed and
   what each change still needs a scope to confirm (the startup timeline, the capture
   staleness window, the kill-switch dead band).
5. The kill test itself: freeze the Jetson for real, prove the cut, with a human present and
   wheels off the ground (`claude-docs/12-testing.md` L7, roadmap Gate G1).

Only after step 5 does roadmap task 1.3 become `[x]`.
