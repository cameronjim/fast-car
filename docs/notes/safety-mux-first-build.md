# safety_mux: first real compile of the Pico target

Date: 2026-09-12. Roadmap task 1.3 (still `[~]`, see "Honest status" below).

**Updated later the same day (2026-09-12)** with two follow-up fixes, both on this same
branch: the GPIO interrupt collision found below is now FIXED, and the nine `null`
safety-mux fields in `config/vehicle_params.yaml` have been filled in with PROVISIONAL,
unmeasured values so the firmware arms for a first bench test. That changes what you see when
you flash it -- read "What to expect when you flash it" below, which has been rewritten, not
the old version of it.

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

At first compile, every field the mux needs was `null` in `config/vehicle_params.yaml` and
came through the binding as `..._is_set = false`. As of the 2026-09-12 provisional-params fix
below, all nine are set (`..._is_set = true`) with UNMEASURED placeholder values, and
`meta.schema_version` is `0.2.1`. See "What to expect when you flash it".

## Warnings

Zero. The clean build produces no compiler warnings in `pico/` or `logic/`.

## Host logic tests

Still pass, unchanged, on both the first compile and after the 2026-09-12 fixes below (`logic/`
was never touched by either):

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

First compile (params all `null`, no IRQ fix): `firmware/safety_mux/build-artifacts/safety_mux.uf2`,
73,728 bytes, sha256 `2cfc8f9cdc90ad958686fa57fae7a2609e2f14d4d62de3cbb186310f59f09b45`.

**Current build (2026-09-12, both fixes below in, this is the one to flash):**
`firmware/safety_mux/build-artifacts/safety_mux.uf2`, **74,752 bytes**,
sha256 `92aabfa925a0f853b6e26d317ad050470733239a80c46bcedf892181e3165551`.
Same container, same pinned SDK 2.1.0, clean tree (`rm -rf firmware/safety_mux/build` first),
zero compiler warnings. (`gmake` prints "Clock skew detected" throughout, which is the
container clock against the bind-mounted host filesystem's mtimes, not a compiler diagnostic;
the build was from scratch, so nothing was skipped.)

Gitignored, not committed: it is build output, regenerate it rather than trusting a copy.

## What to expect when you flash it

**Amended 2026-09-14** by the review in `docs/notes/firmware-review-2026-09-14.md`, which
changed three things this section describes. Read this section with those in mind, and that
note for why:

1. GPIO 6, 7 and 8 are now driven LOW from the first statements of `main()` and stay that way
   if the firmware refuses to arm. They used to float from reset, and float forever on a
   refusal, which is what the fast-blink state actually looked like on the pins.
2. The kill-switch channel now has a 100 us dead band around its threshold, so the arm point
   in the bench sequence below is 1600 us, not 1500 us, and the kill point is below 1400 us.
   A switch parked between the two holds its previous position and reads KILL at power-on.
3. A capture channel with no pulse in the last 60 ms reads invalid regardless of the last
   width it saw, so a stuck-high input cuts instead of repeating its last command.

This section was rewritten on 2026-09-12 after the provisional-params fix below. It now
describes a firmware that **arms**. The old version of this section said a fault blink was
the pass condition; that was true only while the params were `null`.

Hold BOOTSEL, plug the Pico in, drop `safety_mux.uf2` on the `RPI-RP2` drive. It reboots and:

### The LED

**The onboard LED (GPIO 25) stays OFF and never blinks.** That is the arm indication.

Be honest about what that means: `pico/main.c` only ever drives the LED in
`fault_halt_missing_param()`. On the arming path the LED is never even initialized, so "off"
is indistinguishable from "the board is dead, unpowered, or did not boot". A dark LED is a
*necessary* sign of arming, not a sufficient one -- confirm with the PWM outputs below.

- **No LED at all** = armed (or dead; check the outputs).
- **Fast blink, 100 ms on / 100 ms off, forever** = the refuse-to-arm path. It should NOT
  happen with this build. If it does, something reverted a `vehicle_params` field to `null`,
  or the binary on the board is an older one. Do not "fix" it by inventing values; read the
  serial line, which names the exact missing field.

Adding a real heartbeat/status blink on the arming path is worth doing and is deliberately
not in this change: it would be new untested behaviour on a board about to be flashed.

### The serial line

**Over USB the firmware prints nothing at all once it arms.** The only `printf` in
`pico/main.c` is in the fault-halt loop. So:

- USB enumerating as a CDC serial device, with silence on it, is the expected armed state.
- Any repeating `FATAL: config/vehicle_params.yaml is missing a required safety_mux field:
  <name>` line means it refused to arm, and names which field.

### The outputs -- this is the real check

With the board powered and **nothing else connected** (no receiver, no Jetson):

- **GPIO 6 (servo) and GPIO 7 (ESC/VESC PPM) each carry a 50 Hz PWM frame with a 1500 us
  pulse.** That is the CUT state's neutral output, driven continuously, not an absence of
  signal. Scope or servo-tester those two pins: seeing 50 Hz / 1500 us is the actual proof
  the firmware booted, read its params, and is running the mux loop.
- **GPIO 8 (power cutoff) is driven LOW**, i.e. cut. It is initialized low and only goes high
  when `mux_decide()` returns `cut == false`.
- **Wheels off the ground, always.** 1500 us is this ESC's *assumed* zero-throttle point, not
  a measured one (see the provisional-params section). If the real neutral is elsewhere, a
  cut state commands a creep.

The cut reason with nothing connected is `MUX_REASON_RC_SIGNAL_INVALID`: GPIO 2 is pulled
down, no pulses arrive, `pwm_capture_read_us()` returns -1.0, and `rc_switch_read()` treats an
uninterpretable channel exactly like KILL. Nothing on the board reports that reason out loud
yet -- it is a value inside `mux_decide()`, not a printed line.

### Getting it to pass drive through (the bench sequence)

`mux_decide()` cuts unless ALL of these hold, checked in this order:

1. RC kill-switch channel on GPIO 2 has a valid pulse inside 1000-2000 us, arriving at least
   every 60 ms, **and** reads at or above 1600 us (ARMED; 1500 us threshold plus the 100 us
   dead band added 2026-09-14). Below 1400 us it is KILL, in between it holds its previous
   position, and unreadable or stale cuts.
2. The Jetson heartbeat on GPIO 5 has toggled within the last 100 ms
   (`mux_watchdog_timeout_s`). Nothing toggling it means a permanent watchdog cut, which is
   correct.
3. The steering pulse on GPIO 3 is inside 1000-2000 us and arrived in the last 60 ms.
4. The throttle pulse on GPIO 4 is inside 1000-2000 us and arrived in the last 60 ms.

Only then do GPIO 6/7 mirror GPIO 3/4 and GPIO 8 go high. Which end of the kill-switch
channel is ARMED has not been measured on this transmitter: if flipping the switch arms it
backwards, that is the threshold/polarity assumption, not a firmware bug. Measure the channel
before trusting it, and keep the wheels off the ground while you do.

## Fix applied 2026-09-12 (1): one shared GPIO IRQ dispatcher

The collision described below under "Finding NOT fixed here" is fixed.

**What was wrong.** In SDK 2.1.0, `gpio_set_irq_callback()` (and
`gpio_set_irq_enabled_with_callback()`, which calls it) stores ONE callback per core.
`pwm_capture.c` installed one, `heartbeat_input.c` installed another, and since `main()` inits
capture first and heartbeat second, heartbeat's won for the whole GPIO bank. All three PWM
capture channels would have read -1.0 forever and the mux would have cut permanently: safe
direction, but inert.

**The approach.** A new module, `pico/gpio_irq_dispatch.{h,c}`, is the single owner of that
per-core callback. It keeps a small fixed-size table of `{gpio, handler}` (8 slots, no
allocation), installs `gpio_set_irq_callback()` exactly once on first registration, and its
callback routes each edge to the handler registered for that GPIO number. `pwm_capture.c` and
`heartbeat_input.c` now call `gpio_irq_dispatch_register(gpio, mask, handler)` instead of
touching the SDK callback themselves.

The dispatch callback is a linear scan over at most 8 slots and then a direct call: short,
allocation-free, no printf, no blocking, same as a raw SDK callback.

**What did NOT change.** Both public headers (`pwm_capture.h`, `heartbeat_input.h`) keep their
exact APIs, so `pico/main.c` is untouched, and the two modules remain separately usable: each
one works alone or alongside the other. Nothing in `firmware/safety_mux/logic/` was modified.

**Fail-safe semantics are preserved.** A channel with no edges still reads -1.0 from
`pwm_capture_read_us()` and is rejected by `pwm_is_valid_us()` / `rc_switch_read()`. A
heartbeat with no edges still returns `+Inf` from `heartbeat_input_age_s()` and still trips
`watchdog_timed_out()`. If registration ever fails (table full), the module simply never
records an edge, which lands in the same stale/invalid state rather than a false "valid"
reading -- that is why the failure path is a silent no-op rather than a fault.

`gpio_irq_dispatch.h` carries a long comment naming the single-callback-per-core constraint
and the bug it caused, so the next person does not reinstall a private callback.

**Still unverified on hardware.** This is the correct SDK usage, but no RP2040 has run it. The
bench test that actually proves it is the first one where a PWM capture channel reads a real
pulse while the heartbeat is also toggling.

## Fix applied 2026-09-12 (2): provisional PWM params so it can arm

`config/vehicle_params.yaml`'s nine safety-mux fields were `null`, so the firmware refused to
arm. They are now filled in:

| Field | Value | Basis |
|---|---|---|
| `steering.pwm_min_us` | 1000 | standard hobby-RC servo convention |
| `steering.pwm_neutral_us` | 1500 | standard hobby-RC servo convention |
| `steering.pwm_max_us` | 2000 | standard hobby-RC servo convention |
| `actuation.throttle_pwm_min_us` | 1000 | standard hobby-RC ESC convention |
| `actuation.throttle_pwm_neutral_us` | 1500 | assumed ESC zero-throttle point |
| `actuation.throttle_pwm_max_us` | 2000 | standard hobby-RC ESC convention |
| `limits.mux_watchdog_timeout_s` | 0.1 | about 5 missed frames of a 50 Hz heartbeat; conservative starting point |
| `limits.mux_kill_switch_threshold_us` | 1500 | midpoint of the 1000-2000 us range |

**Every one of these is PROVISIONAL and NOT MEASURED.** Nobody has put a scope on this car's
receiver, servo, or ESC. They are convention and conservative guesses, written down so the
firmware arms for a bench test, and each is marked PROVISIONAL inline in the YAML with a
pointer to the step that replaces it: `docs/notes/hardware-arrival-checklist.md` section 3
(roadmap task 1.3). **They must be replaced by real measurement before the car drives on the
floor.** The params are baked into the binary at compile time, so that means rebuild and
reflash, not an edit on the car.

`meta.schema_version` went 0.2.0 -> 0.2.1 (values changed, no schema field added or removed;
`claude-docs/06-vehicle-params.md` rule 5 bumps on any change). `meta.sysid_session_id` stays
`none-preliminary` deliberately: there is still no on-vehicle fit, and the sim-regression
golden references and the policy-contract fixtures pin that string. Since `meta` is
`additionalProperties: false` in the schema, the "these are provisional" record lives as a
comment block in `meta` and in the file header rather than as a tenth `meta` key. The two
`racer_policy` test literals that assert the committed schema version were updated to 0.2.1.

The bindings were regenerated with `tools/gen_params.py` (never hand-written,
`claude-docs/06-vehicle-params.md` rule 3) and the round-trip tests still pass.

**The refuse-to-arm guard was not weakened.** `mux_params_from_raw()` still returns the first
missing field and `main()` still halts and fast-blinks on any `null`. We supplied values; we
did not disable the check. Its host test suite (`test_mux_params_suite`) is unchanged and
passing.

## The original finding, as first reported (now fixed, see above)

Kept verbatim for the record. It was found while reading the SDK sources and reported rather
than fixed at the time, because it was a behaviour change that could not be validated without
hardware and that task was scoped to compiling. It was fixed later the same day; see "Fix
applied 2026-09-12 (1)" above.

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

Unchanged by the 2026-09-12 fixes below. The IRQ fix is correct SDK usage but has not run on
an RP2040, and the nine params it now arms with are unmeasured placeholders. Roadmap task 1.3
stays `[~]`.

Roadmap task 1.3 stays `[~]`. This moves exactly one item on
`firmware/safety_mux/README.md`'s "What still has to happen" list (step 3's build half) and
nothing else. The IRQ collision above means step 4's bench tests have known work waiting
before they can pass.
