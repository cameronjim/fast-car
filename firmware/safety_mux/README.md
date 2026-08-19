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
| Mux state machine, watchdog timing, PWM validity checks, RC switch interpretation, param null-checking | `logic/` | Host-compiled with plain `gcc` (no Pico SDK, no cross-compiler), table-driven, every branch exercised -- see `tests/`. Runs in CI on every push (`.github/scripts/safety_mux_host_tests.sh`). |
| GPIO/PWM capture, PWM output, heartbeat input, power-cutoff GPIO, main loop | `pico/` | **Not tested anywhere.** Real hardware/interrupt access with no host equivalent. Written from the Pico SDK's documented API, never compiled with the actual Pico SDK toolchain (not available in this repo's containers or in CI -- see `CMakeLists.txt`'s header comment), never run on a chip. |
| The whole thing, on a Jetson, RC receiver, servo, and ESC | (nothing yet) | Roadmap 1.3's kill test, `claude-docs/12-testing.md` L6/L7. Pending hardware. |

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
  3. Per-channel Jetson PWM validity -- catches a glitched-but-alive command signal.
  4. Otherwise: passthrough.
- A cut is a cut: both PWM outputs go to a configured neutral value (not merely "no signal",
  which would depend on the servo/ESC's own undocumented failsafe behavior) and the power
  cutoff GPIO is asserted, together, every time. There is no partial-cut state.
- Physical constants (PWM ranges, neutral values, the watchdog timeout, the kill-switch
  threshold) are never hand-typed here (`CLAUDE.md` invariant 2). They come from
  `config/vehicle_params.yaml` via `tools/gen_params.py`'s generated C binding
  (`pico/main.c`'s `raw_fields_from_generated_params()`), and `logic/mux_params.c` refuses to
  produce a usable `MuxParams` if any required field is still `null` -- `pico/main.c` halts
  forever (blinking a fault LED, printing over USB serial) rather than silently assuming a
  default. All of these fields are `null` in the committed `config/vehicle_params.yaml` right
  now (see that file's comments): this firmware **cannot arm** until they are bench-measured
  and filled in.

## Proposed pinout (UNVERIFIED, pending bench wiring)

Pin numbers are RP2040 GPIO numbers, chosen for this draft and matched exactly in
`pico/main.c`'s `#define`s -- change one, change both.

| Signal | Direction | RP2040 GPIO | Notes |
|---|---|---|---|
| RC receiver kill-switch channel | in | GPIO 2 | PWM capture (interrupt-timed). Receiver's own valid PWM range and this channel's ARMED/KILL threshold are bench-measured, not assumed (`config/vehicle_params.yaml`'s `limits.mux_kill_switch_threshold_us`). |
| Jetson steering PWM | in | GPIO 3 | PWM capture. Range = `vehicle_params.steering.pwm_{min,max}_us`. |
| Jetson throttle PWM | in | GPIO 4 | PWM capture. Range = `vehicle_params.actuation.throttle_pwm_{min,max}_us`. |
| Jetson heartbeat | in | GPIO 5 | Raw digital toggle from a lightweight Jetson-side process (see `pico/heartbeat_input.h`'s comment) -- NOT a UART message, NOT ROS. Timeout = `vehicle_params.limits.mux_watchdog_timeout_s`. |
| Servo PWM out | out | GPIO 6 | 50 Hz hardware PWM to the steering servo. |
| ESC PWM out | out | GPIO 7 | 50 Hz hardware PWM to the ESC. |
| Power cutoff | out | GPIO 8 | Active-HIGH = power enabled (fail-safe: a dead/reset RP2040 or a browned-out driver circuit defaults this LOW = cut). Drives a relay or high-side MOSFET gate in the motor/servo power path -- exact drive circuit is a bench decision, not fixed here. |
| Fault LED | out | Pico's onboard LED (`PICO_DEFAULT_LED_PIN`) | Fast blink = refused to arm (missing `vehicle_params` field), see `pico/main.c`'s `fault_halt_missing_param()`. |

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
├── CMakeLists.txt         Pico SDK build (untested here, see "What is and isn't tested")
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
3. Build `pico/` for real with a Pico SDK toolchain on Desktop B and flash an actual RP2040.
4. Bench-test each I/O path individually (PWM capture reads sane values, PWM output drives
   the servo/ESC correctly, power cutoff actually cuts) -- `claude-docs/12-testing.md` L6.
5. The kill test itself: freeze the Jetson for real, prove the cut, with a human present and
   wheels off the ground (`claude-docs/12-testing.md` L7, roadmap Gate G1).

Only after step 5 does roadmap task 1.3 become `[x]`.
