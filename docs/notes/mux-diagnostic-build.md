# safety_mux: the opt-in diagnostic build

Date: 2026-09-20. A bench debugging aid, not a change to the shipping safety behaviour.

## What it is

The same firmware, built with one CMake option turned on, that prints one line over USB
serial about twice a second saying exactly what the mux is seeing and why it is cutting.

It exists because the mux is a black box on the bench: the fault LED only ever indicates
"refused to arm", and everything else (which input is stale, which pulse is out of range,
which of the four cut conditions won) is a value inside `mux_decide()` that nothing reports.
With a multimeter you can see that a pin has roughly the right average voltage; you cannot
see whether the mux thinks that pin is valid.

## What it is NOT

It does not change any safety behaviour. Specifically:

- Nothing in `firmware/safety_mux/logic/` was modified. Same decision function, same priority
  order (RC kill, then heartbeat watchdog, then steering PWM, then throttle PWM), same
  fail-safe defaults, same cut semantics (both channels to neutral plus the cutoff GPIO low,
  together, every time).
- The diagnostic code reads state that the cycle has already produced and prints it. It takes
  no decision, writes no output pin, and holds no state that any decision reads.
- The one hook it needed, `pwm_capture_diag()` in `pico/pwm_capture.c`, is a read-only
  accessor: it reads the same fields `pwm_capture_read_us()` reads, under the same
  interrupt-disabled section, and writes nothing. It is compiled only in the diagnostic build.
- The shipping build is unchanged byte for byte by the diagnostic sources. Proof, not
  assertion: when this was written, with `DIAG_BUILD=OFF`, the branch produced
  `safety_mux.uf2` at 79360 bytes, sha256
  `ccddd7452d1c6dc114805d4f265a96be6565ee238b911141bc26dd2f2b132683`, the same size and hash
  as the shipping artifact recorded in `docs/notes/build-log.md`'s 2026-09-20 entry, built
  before any of this existed. Later changes to `logic/` change BOTH builds together and move
  both hashes; the 2026-09-21 window/clamp change (GitHub issue #63) is the first of those,
  and that entry in `build-log.md` carries the current pair. What stays true is the property
  being claimed here: the diagnostic sources add printing, never a decision.

Because the diagnostic binary is a different binary, the shipping build stays the one that
drives the car. Use the diagnostic build to find out what is wrong, then flash the shipping
build back.

## Building

Both builds use the container in `docs/notes/safety-mux-first-build.md` (debian:bookworm
arm64, `arm-none-eabi-gcc` 12.2, pico-sdk 2.1.0 via `FetchContent`, `tools/gen_params.py` run
as a build step). From the repo root:

```sh
# Shipping firmware (DIAG_BUILD defaults to OFF; this is the one that drives the car)
docker run --rm -v "$PWD":/repo -w /repo debian:bookworm bash -c '
  apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
    gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib \
    cmake build-essential git python3 python3-yaml python3-jsonschema &&
  cmake -S firmware/safety_mux -B firmware/safety_mux/build &&
  cmake --build firmware/safety_mux/build -j"$(nproc)" &&
  mkdir -p firmware/safety_mux/build-artifacts &&
  cp firmware/safety_mux/build/safety_mux_firmware.uf2 \
     firmware/safety_mux/build-artifacts/safety_mux.uf2'

# Diagnostic firmware (separate build tree, separate artifact name)
docker run --rm -v "$PWD":/repo -w /repo debian:bookworm bash -c '
  apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
    gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib \
    cmake build-essential git python3 python3-yaml python3-jsonschema &&
  cmake -S firmware/safety_mux -B firmware/safety_mux/build-diag -DDIAG_BUILD=ON &&
  cmake --build firmware/safety_mux/build-diag -j"$(nproc)" &&
  mkdir -p firmware/safety_mux/build-artifacts &&
  cp firmware/safety_mux/build-diag/safety_mux_firmware.uf2 \
     firmware/safety_mux/build-artifacts/safety_mux_diag.uf2'
```

`build/`, `build-diag/` and `build-artifacts/` are all gitignored. The `.uf2` files are build
output and are never committed.

Flash the usual way: hold BOOTSEL, plug the Pico in, drag the chosen `.uf2` onto the `RPI-RP2`
drive.

## Watching it from a Mac

Find the device (the Pico's CDC port appears once the firmware is running, not in BOOTSEL
mode):

```sh
ls /dev/tty.usbmodem*
```

Then attach:

```sh
screen /dev/tty.usbmodem* 115200
```

If more than one `usbmodem` device is listed, `screen` will not accept the glob; paste the
exact name instead, for example `screen /dev/tty.usbmodem14201 115200`. The baud rate is
ignored by a USB CDC device (there is no real UART behind it) but `screen` wants one.

Leave `screen` with `Ctrl-a` then `k` then `y`. If a later session says the device is busy,
run `screen -ls` and `screen -X -S <id> quit` to clear the dead one.

Nothing is printed until a terminal actually opens the port, by design (see "Not blocking on
USB"). When one attaches, the firmware prints a four line banner and then the per-cycle line.

## Reading the output

On attaching:

```
=== safety_mux DIAGNOSTIC build (printing only; decision logic identical to the shipping build) ===
pins: kill=gp12 steer=gp10 throttle=gp7 heartbeat=gp5 | out servo=gp1 esc=gp3 cutoff=gp0
params: steer 1000-2000 (neutral 1500) | throttle 1000-2000 (neutral 1500) | watchdog 100ms | kill thr 1500us +-100us | rc range 1000-2000
cut priority: 1 rc kill/unreadable, 2 heartbeat watchdog, 3 steering, 4 throttle
pwm window: the two Jetson command channels are accepted up to 62.5us (4 x 15.6us capture grid) OUTSIDE their configured range, and an accepted pulse is CLAMPED back into that range before it is forwarded. The rc kill channel is NOT widened. Stale/no-edge/out-of-range semantics are otherwise unchanged.
```

Then, twice a second, one line. A nominal passthrough looks like this:

```
[0 t=12.945s] KILL gp12=1872us FRESH(12ms, in 1000-2000 +-0.0us) -> ARMED (arm>=1600us kill<1400us) | HB gp5 age=11ms (timeout 100ms) OK | STEER gp10=1500us FRESH(8ms, in 1000-2000 +-62.5us) | THR gp7=1500us FRESH(8ms, in 1000-2000 +-62.5us) | DECISION=PASS reason=NORMAL | OUT servo gp1=1500us esc gp3=1500us cutoff gp0=HIGH(power enabled)
```

Field by field:

| Field | Meaning |
|---|---|
| `[0 t=12.945s]` | Line sequence number, then seconds since the Pico booted. A jump in `t` or a reset of the sequence number means the board rebooted, which is itself a finding (brownout). |
| `KILL gp12=1872us ...` | The kill-switch channel's captured pulse width and its state. `FRESH(Nms, in A-B)` = a plausible pulse arrived N ms ago and is inside the receiver's valid range. The other states are below. |
| `-> ARMED` | The decoded kill-switch position the decision actually used this cycle: `ARMED`, `KILLED`, or `UNREADABLE`. This is `MuxOutput.switch_position`, not a recomputation, so it can never disagree with the decision. |
| `(arm>=1600us kill<1400us)` | The thresholds it is being compared against: `limits.mux_kill_switch_threshold_us` plus and minus the dead band (`RC_SWITCH_DEFAULT_HYSTERESIS_US`, 100 us). Between the two edges the position holds, and a hold out of `UNREADABLE` resolves to KILL, never ARMED. |
| `HB gp5 age=11ms (timeout 100ms) OK` | Milliseconds since the last heartbeat edge, the configured `limits.mux_watchdog_timeout_s`, and the watchdog's verdict. `OK` or `TIMED_OUT`, decided by the same `watchdog_timed_out()` the mux uses. `NO_EDGES_EVER` means no edge has ever been seen since boot. |
| `STEER gp10=...`, `THR gp7=...` | The two Jetson command channels, same format and same states as the kill channel, checked against their own `vehicle_params` ranges widened by the 62.5 us capture-quantisation slack. Unlike the kill channel, an accepted pulse on these is clamped back into the unwidened range before it is forwarded. |
| `DECISION=PASS` / `DECISION=CUT` | Whether this cycle passed the Jetson's commands through or cut. |
| `reason=NORMAL` / `reason=2:WATCHDOG_TIMEOUT` | The winning cut condition, prefixed with its position in the priority order. Several conditions can be true at once; the number tells you which one was checked first and therefore reported. Fix them in number order. |
| `OUT servo gp1=1500us esc gp3=1500us` | The pulse widths being commanded on the two outputs right now. On a cut these are the configured neutrals; on a pass they mirror the Jetson inputs, clamped to the configured range (so a 2031 us input shows here as 2000 us). |
| `cutoff gp0=LOW(power cut)` | The power-cutoff GPIO's actual output level, read back from the pin. |

Per-channel states:

| State | Means |
|---|---|
| `FRESH(Nms, in A-B +-Tus)` | A plausible pulse arrived N ms ago and is inside the configured range. `T` is the capture-quantisation slack this channel allows (62.5 us on the two Jetson command channels, 0 on the kill channel -- see `firmware/safety_mux/README.md` and GitHub issue #63). One of the two states that does not cut. |
| `FRESH(Nms, Dus outside A-B, within Tus grid slack; CLAMPED->Vus)` | The pulse is D us outside the configured range but inside the widened window, so it is accepted and **forwarded as V us**, the range edge, not as the measured width. This is what a commanded 2000 us measuring 2031 us looks like. Does not cut. |
| `NO_EDGES(...)` | No plausible complete pulse has ever been captured on this pin since boot. The line is dead, unplugged, stuck at a level, or carrying only noise. |
| `STALE(last edge Nms ago, window 60ms; stuck or stopped)` | A pulse was captured once, but nothing has arrived within the capture staleness window, so the last width is not a live command. A stuck-high line looks exactly like this. |
| `OUT_OF_RANGE(allowed A-B +-Tus grid slack)` | Pulses are arriving and are fresh, but the width is outside the configured range **and** outside the quantisation slack around it. Genuinely bad, not a rounding artefact. |
| `NOT_REGISTERED` | `pwm_capture_init_channel()` was never called for that GPIO. Should be impossible; it would be a firmware bug, not a wiring fault. |

Real examples of each cut, generated from the actual format strings:

```
[1 t=13.545s] KILL gp12=1102us FRESH(9ms, in 1000-2000 +-0.0us) -> KILLED (arm>=1600us kill<1400us) | HB gp5 age=11ms (timeout 100ms) OK | STEER gp10=1500us FRESH(8ms, in 1000-2000 +-62.5us) | THR gp7=1500us FRESH(8ms, in 1000-2000 +-62.5us) | DECISION=CUT reason=1:RC_KILL_SWITCH | OUT servo gp1=1500us esc gp3=1500us cutoff gp0=LOW(power cut)
[2 t=14.145s] KILL gp12=--- NO_EDGES(no plausible pulse since boot; line dead/unplugged/noise) -> UNREADABLE (arm>=1600us kill<1400us) | ... | DECISION=CUT reason=1:RC_SIGNAL_INVALID | ...
[3 t=14.745s] KILL gp12=1872us FRESH(10ms, in 1000-2000 +-0.0us) -> ARMED (arm>=1600us kill<1400us) | HB gp5=NO_EDGES_EVER (timeout 100ms) TIMED_OUT | ... | DECISION=CUT reason=2:WATCHDOG_TIMEOUT | ...
[4 t=15.345s] ... | STEER gp10=1500us STALE(last edge 430ms ago, window 60ms; stuck or stopped) | ... | DECISION=CUT reason=3:STEERING_PWM_INVALID | ...
[5 t=15.945s] ... | THR gp7=2450us OUT_OF_RANGE(allowed 1000-2000 +-62.5us grid slack) | DECISION=CUT reason=4:THROTTLE_PWM_INVALID | ...
```

And the case that is NOT a cut any more (GitHub issue #63): a commanded 2000 us landing on the
capture grid at 2031 us is accepted and forwarded at the range edge.

```
[6 t=16.545s] ... | STEER gp10=2031us FRESH(8ms, 31us outside 1000-2000, within 62.5us grid slack; CLAMPED->2000us) | ... | DECISION=PASS reason=NORMAL | OUT servo gp1=2000us esc gp3=1500us cutoff gp0=HIGH(power enabled)
```

## PWM output audit, 2026-09-20 (prompted by a 0.36 V reading on GP1)

A DC multimeter on GP1 (direct 3.3 V logic, not through the level shifter) read a constant
0.36 V with the mux running, not tracking the Jetson steering input swept 1000 to 2000 us.
0.36 V at 3.3 V logic is about 10.9 percent duty; a 1500 us pulse in a 20 ms frame is
7.5 percent, about 0.25 V. `pico/pwm_output.c`'s slice, divider and wrap arithmetic was
audited against the RP2040 PWM peripheral and pico-sdk 2.1.0's `hardware_pwm/include/hardware/pwm.h`.

**No arithmetic defect was found.** Point by point:

| Checked | Finding |
|---|---|
| Divider and wrap for a 20 ms period | `clk_sys` 125 MHz / 62.5 = 2 MHz counter, 0.5 us per count. The counter counts 0 to TOP and wraps, so the frame is TOP+1 = 40000 counts = 20.000 ms = 50.000 Hz exactly. Correct. |
| Integer truncation in that arithmetic | None. 62.5 is exactly representable in the RP2040's 8.4 fixed-point divider (integer 62, fraction 8/16), and the SDK's `pwm_set_clkdiv()` computes `i = 62`, `f = 8` with no rounding. 40000 and 3000 are exact integers. |
| Are both slices configured independently | Yes. GP1 is slice 0 channel B, GP3 is slice 1 channel B (slice = gpio >> 1, channel = gpio & 1). `pwm_output_init_channel()` is called once per output and does its own `pwm_set_clkdiv`, `pwm_set_wrap` and `pwm_set_enabled` on that GPIO's own slice. Neither can clobber the other's period register. |
| Does `pwm_output_set_us` use the same wrap the init used | Yes. `level = us * 2` is the inverse of 0.5 us per count, consistent with TOP+1 = 40000. The clamp at TOP/2 = 19999.5 us maps to level 39999 = TOP, the longest expressible pulse, rather than overflowing into a permanently high output. |
| Does GP0 on slice 0 channel A disturb slice 0 for GP1 | No. `power_cutoff_init()` configures GP0 through SIO (`gpio_init`, `gpio_set_dir`, `gpio_put`) and never calls `gpio_set_function(GPIO_FUNC_PWM)`, so slice 0 channel A's output is not routed to any pin, and none of the SIO calls write a PWM register. Slice 0's TOP, DIV, CSR are written only by the GP1 init; channel A's compare value is left at reset and affects nothing. |
| Per-cycle `pwm_set_chan_level()` at 200 Hz | Safe. The RP2040's counter-compare register is double-buffered: a write takes effect at the next wrap, so it cannot produce a runt or stretched pulse mid-frame. |

**The important correction: a divider or clock error cannot produce a high DC average.** Duty
cycle is `level / (TOP + 1)`. That expression contains no clock term at all. Changing the
divider, or running on a different `clk_sys`, changes the frame RATE and the real-world pulse
width in microseconds, but the fraction of time the pin spends high stays 3000/40000 = 7.50
percent, and a DC average is exactly that fraction of the logic level. So of the two
hypotheses raised from the bench:

- "the period is about 13.75 ms rather than 20 ms" would NOT raise the meter reading at all.
  It is ruled out as an explanation of 0.36 V.
- "the pulse width is about 2180 us" WOULD, and it corresponds to a compare level of 4360
  rather than 3000. No configured value in `config/vehicle_params.yaml` is 2180 us (neutral is
  1500, the range maximum is 2000), so nothing in the decision path can command it.

A frame-rate error still matters for a different reason, because a servo fed the wrong frame
rate can ignore the signal and go limp, which is the observed symptom. It just is not what a
DC voltage measurement can detect.

**One measurement inconsistency worth naming.** Earlier the same evening the servo header's
signal pin read about 0.24 V, and GP1 now reads 0.36 V, with continuity between them
confirmed. Those are the same net, so they cannot both be the same waveform correctly
measured. 0.24 V is 7.3 percent, which is the expected 7.5 percent. A DC multimeter averaging
a 50 Hz square wave with a 7.5 percent duty cycle is close to the worst case for a cheap
meter's input filter and sampling, and readings that drift with probe placement are common. I
cannot resolve this without the hardware; it is a real possibility that the firmware is
emitting exactly the right waveform and the meter is the thing that is wrong.

**What was changed.** Nothing in the emitted code. Two things were added:

1. Build-time assertions in `pico/pwm_output.c` pinning the frame arithmetic: that `clk_sys`
   is 125 MHz (the 62.5 divider's silent assumption, now a compile error rather than a
   comment), that TOP+1 is 40000, that 1500 us is 3000 counts, and that this is 7.5 percent
   duty. `_Static_assert` emits no code, and the shipping `.uf2` is byte-for-byte identical
   with them in place (same 79360 bytes, same sha256), which was verified by rebuilding.
2. A `PWMREG` line in the diagnostic output, below, which reads the registers back.

**The `PWMREG` line.** Printed with every report, from the actual peripheral registers rather
than from the commanded value:

```
    PWMREG clk_sys=125000000Hz | servo gp1 slice0.B top=39999 div=62.500 level=3000 en=1 pinfn=PWM -> pulse=1500.0us frame=20000.0us 50.00Hz duty=7.50% | esc gp3 slice1.B top=39999 div=62.500 level=3000 en=1 pinfn=PWM -> pulse=1500.0us frame=20000.0us 50.00Hz duty=7.50%
```

`duty` is the number to compare against the multimeter: multiply it by the logic level.
`clk_sys` is read with `clock_get_hz(clk_sys)`, not assumed. `pinfn` says whether the pin is
actually switched to the PWM function, which distinguishes "the peripheral is right but the
pin is not routed" from everything else. Two reference cases, both generated from the real
formatting code:

```
# a wrong clk_sys (150 MHz) -- note the duty does NOT move, only the rate and the pulse
    PWMREG clk_sys=150000000Hz | servo gp1 slice0.B top=39999 div=62.500 level=3000 en=1 pinfn=PWM -> pulse=1250.0us frame=16666.6us 60.00Hz duty=7.50% | ...
# what the 0.36 V reading would require
    PWMREG clk_sys=125000000Hz | servo gp1 slice0.B top=39999 div=62.500 level=4360 en=1 pinfn=PWM -> pulse=2180.0us frame=20000.0us 50.00Hz duty=10.90% | ...
```

So on the bench tonight, flash the diagnostic build and read one line:

- `duty=7.50%` and `50.00Hz` and `pinfn=PWM`: the firmware is emitting the correct waveform.
  The fault is downstream (wiring, the servo's logic-level threshold) or the meter. Put a
  scope on GP1 next, not a meter.
- `duty` near 10.90 percent, or `level` near 4360: something really is commanding about
  2180 us, and the `OUT servo` field on the line above will say what it thinks it commanded.
  A disagreement between the two fields is a firmware bug and the registers are the evidence.
- `frame` or `Hz` off, with `duty=7.50%`: a clock problem, which the build-time assertion is
  supposed to make impossible, so treat it as a surprise worth stopping on.
- `pinfn=NOT_PWM` or `en=0`: the peripheral is not driving the pin at all.

## Symptom table

Tonight's actual symptom is the last row.

| Symptom | What the diagnostic line shows | Likely cause |
|---|---|---|
| Mux never passes, kill knob appears to be in the armed position | `KILL gp12=1102us FRESH ... -> KILLED`, `reason=1:RC_KILL_SWITCH` | The knob's armed end is the low end on this channel, or the threshold is wrong for this receiver. Turn VrA the other way and watch the number move. Whichever end reads above 1600 us is the armed end. If the number never crosses 1600 us in either direction, the threshold in `config/vehicle_params.yaml` does not match the measured channel, which is exactly the unmeasured provisional value that has to be replaced. |
| Mux never passes, receiver is connected | `KILL gp12=--- NO_EDGES ... -> UNREADABLE`, `reason=1:RC_SIGNAL_INVALID` | Nothing usable is reaching GP12. Receiver not powered from the board's 5 V lead, CH5 not the channel that is wired, the signal not passing the level shifter, or no common ground. Note this is the same line you get with the transmitter switched off, because failsafe is set to the kill end. |
| Mux cuts even with the kill switch clearly armed and the Jetson running | `KILL ... -> ARMED`, then `HB gp5=NO_EDGES_EVER ... TIMED_OUT` or `HB gp5 age=340ms ... TIMED_OUT`, `reason=2:WATCHDOG_TIMEOUT` | The heartbeat is not reaching GP5. `NO_EDGES_EVER` means the wire or the Jetson service is dead at the mux end even though pin 7 toggles at kernel level, so suspect the JETSON connector, a reversed keyed plug, or the shared ground. A finite but too large `age` means the toggle is arriving slower than the 100 ms timeout, which is a rate problem, not a wiring problem. |
| Mux cuts, kill armed, heartbeat OK | `STEER gp10=... STALE(...)` or `OUT_OF_RANGE(...)`, `reason=3:STEERING_PWM_INVALID` (or the `THR gp7` equivalent, `reason=4`) | `STALE` with a sensible last width means the Jetson stopped producing edges (pwmchip disabled, duty left at 0 or 100 percent), or the line is stuck. `NO_EDGES` means that Jetson PWM pad is not reaching the mux at all, which the pin 15 and pin 33 voltage measurements do not rule out, because they were taken at the Jetson header and not at GP10 and GP7. `OUT_OF_RANGE` means pulses are arriving fine and the commanded width is outside 1000 to 2000 us by more than the 62.5 us quantisation slack (GitHub issue #63); a width just past an edge is accepted and clamped instead, and the line says `CLAMPED->`. |
| **Mux says PASS but the servo still does not respond and goes limp** | `DECISION=PASS reason=NORMAL`, `OUT servo gp1=1500us`, `cutoff gp0=HIGH(power enabled)`, and on the `PWMREG` line `duty=7.50% 50.00Hz pinfn=PWM` | Not a logic problem. The decision is passing and the firmware is commanding a 1500 us pulse. Look downstream of GP1: the servo lead, the SERVO header's ground (a servo with 5 V and signal but no common ground with the Pico goes limp exactly like this), the GP1 to header trace or solder joint, or the servo not accepting 3.3 V logic. A limp servo specifically means it is seeing no valid pulse train at all, since a servo that receives 1500 us holds centre stiffly. Scope GP1 directly at the Pico pin, then at the header pin: if the pin is clean and the header is not, the fault is on the board. |
| Mux says CUT but the servo is limp rather than centred | any `DECISION=CUT` line with `OUT servo gp1=1500us` | Same downstream conclusion as the row above. A cut is not an absence of signal in this design: both outputs are driven to the configured neutral continuously. A limp servo during a cut therefore also points at wiring or waveform, not at the decision. |
| Output line never appears, board seems alive | nothing on the serial port | Either the shipping build is flashed rather than the diagnostic one, or the terminal is not actually open on the CDC port. The firmware deliberately prints nothing until a host opens the port. |
| Fast LED blink, no mux line | banner never appears | The refuse-to-arm path, unrelated to this build. A `vehicle_params` field is null. The fault message names the field and is printed by the shipping build too. |

One honest caveat on the two rows above: 0.24 V average on the servo signal pin is what a
3.3 V, 1500 us, 50 Hz pulse train averages to (1500/20000 of 3.3 V is 0.2475 V), so the
measurement is consistent with the firmware doing the right thing. It is equally consistent
with a 7.5 percent duty cycle at any frequency, which is why the diagnostic line, plus a
scope on GP1, is the thing that separates the two.

## Not blocking on USB

The board will sometimes run with nothing plugged into USB, so this was checked in the SDK
source rather than assumed. In pico-sdk 2.1.0,
`src/rp2_common/pico_stdio_usb/stdio_usb.c`'s `stdio_usb_out_chars()`:

- returns immediately when no host has the CDC port open (`stdio_usb_connected()` is false),
  so an unattached board is not slowed down at all; but
- when a host IS attached and stops reading, it spins in `tud_task()` until
  `PICO_STDIO_USB_STDOUT_TIMEOUT_US` has elapsed, which defaults to 500000, half a second.
  That would be five hundred missed control cycles.

Both are handled. The diagnostic target compiles with `PICO_STDIO_USB_STDOUT_TIMEOUT_US=0`, so
a full buffer breaks the write loop on the first attempt and characters are dropped rather
than waited on, and `diag_report_tick()` additionally checks `stdio_usb_connected()` before
doing any formatting work. A dropped diagnostic line is cosmetic; a stalled mux loop is not.

The print itself happens on roughly one control cycle in a hundred (the loop runs at 200 Hz,
the report at 2 Hz), after the outputs have already been written to the pins, so the line can
never describe something ahead of what the car is doing.

## What this does not prove

Nothing here has run on an RP2040. The diagnostic build cross-compiles clean with `-Wall
-Wextra` (zero warnings on the three files this change touches) and the example lines above
were produced by compiling the real `diag_report.c` formatting code on the host against
scripted inputs, so the format strings and the field text are exact. Whether the USB CDC port
enumerates, whether the 2 Hz print actually leaves the 200 Hz loop undisturbed on the chip,
and whether any of the captured numbers are correct are all bench questions that this build
exists to answer, not ones it answers in advance.

Roadmap task 1.3 is unaffected and stays `[~]`.
