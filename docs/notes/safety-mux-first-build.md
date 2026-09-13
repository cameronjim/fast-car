# safety_mux: first real compile of the Pico target

Date: 2026-09-12. Roadmap task 1.3 (still `[~]`, see "Honest status" below).

Until this session, `firmware/safety_mux/pico/` and `firmware/safety_mux/CMakeLists.txt` had
never been through a Pico SDK toolchain by anyone. They were written from the SDK's
documented API, and CI only ever built `logic/` + `tests/` with host `gcc`. This note records
the first actual cross-compile: what it took, what was broken, and what it does and does not
prove.

## Toolchain

The build runs inside a Linux container so it is reproducible off this machine.

| Piece | Version |
|---|---|
| Container base | `debian:bookworm` (Debian 12.15), arm64 |
| Cross-compiler | `gcc-arm-none-eabi` 15:12.2.rel1-1 (arm-none-eabi-gcc 12.2.1 20221205) |
| C library | `libnewlib-arm-none-eabi`, `libstdc++-arm-none-eabi-newlib` |
| CMake | 3.25.1 |
| Python | 3.11.2 with `python3-yaml`, `python3-jsonschema` (for `tools/gen_params.py`) |
| pico-sdk | release tag **2.1.0**, commit `95ea6acad131124694cda1c162c52cd30e0aece0`, pulled by the `FetchContent` pin already in `CMakeLists.txt` (unchanged) |
| picotool | 2.3.2-develop, built from source by the SDK, no libusb (not needed to emit a `.uf2`) |

Exact commands: see `firmware/safety_mux/README.md`'s "Building" section.

## Errors encountered, and the fix for each

### 1. `unknown type name 'uint'` in all four `pico/*.h` headers

Seven errors across `pwm_capture.h`, `pwm_output.h`, `heartbeat_input.h`, `power_cutoff.h`.
Every one of those headers declares functions taking `uint gpio` but included only
`<stdint.h>`. `uint` is not a C standard type: it is a Pico SDK typedef from
`pico/types.h`. The `.c` files include their own header first, before any SDK header, so
nothing had defined `uint` at that point.

**Fix:** added `#include "pico/types.h"` to each of the four headers, next to the existing
`<stdint.h>`. No signature changed; the headers now just declare what they already used.

### 2. `filename '/build/sm/safety_mux_firmware' does not have a recognized file type`

Everything compiled and linked; the build then died in `pico_add_extra_outputs()`'s picotool
post-processing step, because the linked ELF had no extension. Configure had also printed a
warning that turned out to be the same root cause:

> System is unknown to cmake, create: Platform/PICO to use this system

`CMakeLists.txt` called `FetchContent_MakeAvailable(pico_sdk)`. That `add_subdirectory()`s the
SDK, which runs the SDK's own `project()` and, critically, does its
`list(APPEND CMAKE_MODULE_PATH ${PICO_SDK_PATH}/cmake)` **in the SDK's directory scope, not
ours**. Our `project(safety_mux C CXX ASM)` therefore could not find the SDK's
`cmake/Platform/PICO.cmake`, which is the file that sets `CMAKE_EXECUTABLE_SUFFIX .elf`. So
the executable was named `safety_mux_firmware` with no suffix, and picotool refuses a file it
cannot identify by extension.

**Fix:** switched to the SDK's own documented import pattern (what
`external/pico_sdk_import.cmake` in the SDK tree does): populate the dependency **without**
`add_subdirectory`, then `include(pico_sdk_init.cmake)`, then `project()`, then
`pico_sdk_init()`. Concretely, `FetchContent_MakeAvailable` became
`FetchContent_GetProperties` + `FetchContent_Populate`, with a `CMP0169` policy guard because
single-argument `FetchContent_Populate()` is deprecated in CMake 3.30+ (it is still what the
SDK's own import script uses). The pinned SDK tag was not touched.

That was all of it: two fixes, and the build is clean. Nothing in `firmware/safety_mux/logic/`
was modified, and no safety logic was stubbed, weakened, or bypassed to get here.

## Generated vehicle params

`tools/gen_params.py` runs as a build step (`add_custom_command` in `CMakeLists.txt`) and
emitted `vehicle_params_generated.h` into the build tree with no changes needed. It is never
committed and was never hand-written, per `claude-docs/06-vehicle-params.md` rule 3.

Every field the mux needs is confirmed `null` in `config/vehicle_params.yaml` and comes
through the binding as `..._is_set = false`. That is correct and expected; see "What to expect
when you flash it".

## Warnings

Zero. The clean build produces no compiler warnings in `pico/` or `logic/`.

## Host logic tests

Still pass, unchanged:

```
$ .github/scripts/safety_mux_host_tests.sh
== test_pwm_validity_suite ==
== test_watchdog_suite ==
== test_rc_switch_suite ==
== test_mux_params_suite ==
== test_mux_decision_suite ==

All assertions passed.
```

Expected, since `logic/` was not touched, but it was run rather than assumed.

## Output

`firmware/safety_mux/build-artifacts/safety_mux.uf2`, 73,728 bytes,
sha256 `2cfc8f9cdc90ad958686fa57fae7a2609e2f14d4d62de3cbb186310f59f09b45`.
Gitignored, not committed: it is build output, regenerate it rather than trusting a copy.

## What to expect when you flash it

**A refusal to arm is the PASS condition for this first flash.** Do not read it as a failure
and do not "fix" it by inventing parameter values.

Hold BOOTSEL, plug the Pico in, drop `safety_mux.uf2` on the `RPI-RP2` drive. It will reboot
and:

- **The onboard LED (GPIO 25) fast-blinks, on 100 ms / off 100 ms, forever.** That is
  `fault_halt_missing_param()` in `pico/main.c`.
- Over USB serial it repeats:
  `FATAL: config/vehicle_params.yaml is missing a required safety_mux field: steering_pwm_min_us`
  (`steering_pwm_min_us` is simply the first unset field `mux_params_from_raw()` checks; there
  are eight more behind it).
- **No PWM is generated on GPIO 6 or 7, and the power-cutoff GPIO 8 is never driven high.**
  `main()` halts before it initializes any I/O, so the servo and ESC get nothing and the power
  path stays cut. That is the intended fail-closed behaviour.

This proves the binary boots, runs, reads the generated params binding, and correctly refuses.
It proves nothing about PWM capture, PWM output, the watchdog, or the kill switch, none of
which execute on this path.

To get past the fault blink you must bench-measure the real values and fill in
`config/vehicle_params.yaml`: `steering.pwm_{min,max,neutral}_us`,
`actuation.throttle_pwm_{min,max,neutral}_us`, `limits.mux_watchdog_timeout_s`,
`limits.mux_kill_switch_threshold_us`. Then rebuild, because the params are baked into the
binary at compile time.

## Finding NOT fixed here: the two GPIO IRQ handlers collide

Found while reading the SDK sources; reporting rather than changing it, because it is a
behaviour change that cannot be validated without hardware and this task was scoped to
compiling.

`pwm_capture.c` installs its handler with `gpio_set_irq_callback(pwm_capture_irq_handler)`.
`heartbeat_input.c` installs its own with
`gpio_set_irq_enabled_with_callback(..., &heartbeat_irq_handler)`. In SDK 2.1.0,
`gpio_set_irq_enabled_with_callback()` calls `gpio_set_irq_callback()` internally, and that
function stores **one callback per core** (`callbacks[core] = callback` in
`hardware_gpio/gpio.c`), overwriting whatever was there.

`main()` calls the three `pwm_capture_init_channel()` calls first and `heartbeat_input_init()`
after, so the heartbeat handler replaces the PWM capture handler for the whole GPIO bank.
On hardware that would mean `pwm_capture_read_us()` returns `-1.0` forever, for all three
channels, and the mux would cut permanently.

The failure direction is safe (a permanently cut mux, not a permanently passed-through one),
but the mux would be inert rather than working. This needs one shared dispatching callback, or
`gpio_add_raw_irq_handler_masked()` per channel, before any bench test of the capture path can
mean anything. It is invisible to the host logic tests, which never touch SDK interrupt
plumbing. Worth doing before roadmap 1.3 step 4.

## Honest status

- **Compiled:** yes, cleanly, from scratch, with the pinned SDK.
- **Flashed to hardware:** no.
- **Run on an RP2040:** no.
- **Connected to a receiver, servo, or ESC:** no.
- **Roadmap 1.3 kill test:** not started.

Roadmap task 1.3 stays `[~]`. This moves exactly one item on
`firmware/safety_mux/README.md`'s "What still has to happen" list (step 3's build half) and
nothing else. The IRQ collision above means step 4's bench tests have known work waiting
before they can pass.
