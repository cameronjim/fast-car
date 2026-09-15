# safety_mux + jetson_heartbeat review, 2026-09-14

A read-and-fix pass over `firmware/safety_mux/` (the RP2040 layer-1 mux) and
`tools/jetson_heartbeat/` before the first bench test, ahead of the perfboard being finished
and the Pico being flashed for the first time. Roadmap task 1.3, `claude-docs/05-safety.md`
layer 1.

Nothing here has run on an RP2040. This is a code review plus the fixes it justified, plus a
containerized cross-compile. **Compiling is still not testing**, and the directory's own
README says so; this note does not move roadmap 1.3 off `[~]`.

Scope note: `config/vehicle_params.yaml` and its schema were NOT touched (another branch is
editing them). Two constants this pass wanted as fields are compile-time constants instead,
each flagged below and in the code.

## Summary of what was found

Eight defects, all fixed. Four of them are fail-OPEN paths, which is the only direction
layer 1 may never fail in:

| # | Where | Defect | Direction |
|---|---|---|---|
| 1 | `pico/main.c` | Both PWM output pins and the power-cutoff pin float from reset until their init calls; on a param refusal they float **forever** | fail-open |
| 2 | `pico/pwm_capture.c` | A captured pulse width never ages out: a stuck-high or stuck-low input reports the last pre-fault width as a live command indefinitely | fail-open |
| 3 | `logic/pwm_validity.c` | A NaN `min_us`/`max_us` made every comparison false and every pulse read VALID | fail-open |
| 4 | `logic/watchdog.c` | A NaN or +Inf `timeout_s` made `age >= timeout` false forever: a watchdog that never trips | fail-open |
| 5 | `pico/pwm_capture.c` | The width and its rise timestamp are multi-word values written in an ISR and read unsynchronized: the main loop can read a torn value | either |
| 6 | `pico/heartbeat_input.c` | Same torn-read on the 64-bit edge timestamp: can synthesize an age ~71 minutes in the past | spurious cut |
| 7 | `logic/rc_switch.c` | No hysteresis: a channel resting near the threshold flaps ARM/KILL at the 200 Hz loop rate | pulsing |
| 8 | `tools/jetson_heartbeat` systemd unit | systemd's default start rate limit (5 starts / 10 s) turns a repeated failure into a permanent stop | permanent cut |

Plus smaller items: a `strtoul` sign hole in `--line` parsing, no upper bound on `--rate-hz`,
a missing shutdown ordering on a `DefaultDependencies=no` unit, and one dead `return`.

## The eight questions, answered

### 1. Startup and arming race

**Before this change, the first second looked like this:**

| Time | GPIO 6 (servo) | GPIO 7 (ESC) | GPIO 8 (cutoff) |
|---|---|---|---|
| reset -> `main()` | floating (SIO input, no pull) | floating | floating |
| `stdio_init_all()` (USB enumeration, tens of ms) | floating | floating | floating |
| param refusal -> `fault_halt_missing_param()` | **floating forever** | **floating forever** | **floating forever** |
| `pwm_output_init_channel()` | PWM function, compare level 0 (0% duty) | same | floating |
| `power_cutoff_init()` | 0% duty | 0% duty | LOW (cut) |
| first loop iteration | neutral | neutral | LOW |

Two real problems. The first is the float window: the RP2040 comes out of reset with GPIOs as
high-impedance inputs, and the mux board has no external pulldowns on the OUTPUT side (the
README's pull-down note covers the three Jetson INPUTS). A floating servo signal line can
twitch the servo and a floating ESC signal line is exactly the condition some ESCs arm on.
The second is worse: the refuse-to-arm path ran **before** any output was initialized, so the
one state that is supposed to be maximally inert -- the firmware declaring it cannot safely
run -- left all three output pins floating for as long as the board stayed powered.

**Now:** `main()`'s first three statements, before `stdio_init_all()` and before any param is
read, are `power_cutoff_init(GPIO 8)` (LOW = cut) and `pwm_output_init_safe()` on GPIO 6 and
7, which drive them LOW as plain GPIO outputs. Driven-low is "no pulses": not the mux's
neutral, and deliberately not called that, because the configured neutral is not knowable
before the params are validated and inventing one is the silent default CLAUDE.md invariant 2
exists to prevent. It is the right state for that gap, and it is the state the pins hold
forever on a refusal. The neutral is applied the moment it IS known, inside
`pwm_output_init_channel()`, which programs the compare level and enables the slice BEFORE
switching the pin to the PWM function -- so there is no 0%-duty frame between "this pin is a
PWM output" and "this PWM output says neutral" either.

Input init (capture, heartbeat) now comes last, after the outputs are at neutral.

**KILL -> ARMED glitch:** there is none, and `test_mux_decision.c` now pins it. `mux_decide()`
decides the arming cycle from that cycle's own captured values; a stale or never-captured
steering/throttle channel still cuts on the cycle the switch arms, and the watchdog is still
checked ahead of both. There is no "arm now, validate next cycle" path for a stale pulse to
pass through. Defect 2 was the real version of this concern (a stale pulse that never
expires), and it is fixed at the capture layer.

### 2. PWM capture robustness

**(a) Stuck-high input.** This was the worst finding. `last_pulse_us` was written once per
falling edge and read forever after, with no timestamp. A line that goes high and stays high
(shorted level shifter, latched receiver, a Jetson PWM peripheral left at 100% duty) produces
no further falling edges, so the last width captured *before* the fault kept being reported
as a live, valid command. The mux would have passed through a frozen steering or throttle
value indefinitely. Fixed: every accepted pulse records `last_pulse_end_us`, and
`pwm_capture_read_us()` returns the -1.0 "no believable pulse" sentinel if that is older than
`PWM_CAPTURE_MAX_AGE_US` (60 ms, three missed 50 Hz frames).

**(b) Glitch filtering.** Two layers, and the answer to "is there a minimum plausible width
check in `logic/pwm_validity`" is: yes, implicitly, via `min_us` from `vehicle_params` -- a
1 us spike reads as 1.0 and is rejected as out of range, fail-safe. But it was rejected by
*cutting*: the spike overwrote a good reading and dropped the mux into a cut for a frame
(20 ms of neutral throttle from a single noise edge). Now `pwm_capture.c` discards widths
outside a deliberately wide plausibility band (200 to 5000 us) at the ISR, using the same
host-tested `pwm_is_valid_us()` rather than a second hand-rolled comparison. That band is not
a second copy of the command check: `logic/` and `vehicle_params` remain the only thing that
decides whether a pulse is a legal command. And discarding is safe precisely because of (a):
an implausible pulse does not refresh the freshness timestamp, so a channel receiving nothing
but noise still ages out and cuts.

**(c) Torn reads.** Real, on both the capture and heartbeat paths. `volatile double` and
`volatile uint64_t` are each two 32-bit accesses on a Cortex-M0+; `volatile` orders them but
does not make the pair atomic, so the main loop could read the low word of a new pulse with
the high word of an old one. Fixed by reading the state under
`save_and_disable_interrupts()` / `restore_interrupts()` -- a handful of instructions at
200 Hz, and the handler is the only writer and runs on this core. Also fixed in passing:
`rise_us != 0` was being used as a "have we seen a rise" sentinel (0 is a legitimate
timestamp in the first microsecond after boot) and was not cleared on the falling edge, so a
second falling edge with no rise between would have measured a width spanning frames. It is
now an explicit `have_rise` flag, cleared on use.

**(d) 50 Hz frame timing.** This is (a): the staleness window IS the arrival-rate check, and
it is independent of the last width being in range. A channel is stale if no plausible pulse
has arrived in 60 ms, whatever the last width was.

### 3. Kill switch semantics

Verified, then changed. At exactly 1500 us the old code read ARMED (`>=`); at -1 (the
never-captured sentinel), at 0 (a common receiver failsafe idle), and at NaN it read
`RC_SWITCH_SIGNAL_INVALID`, which `mux_decide()` treats exactly like KILL. That half was
correct and stays correct.

The gap was flapping. With no dead band, a three-position switch, a trimmed transmitter, or
plain receiver jitter parking the channel within a microsecond or two of 1500 would toggle
ARM/KILL at the 200 Hz loop rate, pulsing the ESC and the power cutoff. Added: a symmetric
dead band around the threshold, `RC_SWITCH_DEFAULT_HYSTERESIS_US` = 100 us, so ARM at or
above 1600, KILL below 1400, hold in between. Holding never holds ARMED out of an unknown
state: a previous position of `RC_SWITCH_SIGNAL_INVALID` (which is also what `main()` seeds
at power-on) resolves to KILL, so a switch already parked in the dead band at boot, or a
channel that dropped out and came back, cannot arm without a genuine move past the arm edge.

`rc_switch_read()` gained the band and a `previous` argument, and `mux_decide()` carries the
resolved position out through `MuxOutput.switch_position` for `main()` to feed back. The
function stays pure, which is what keeps the table-driven tests meaningful. With
`hysteresis_us` = 0 the behaviour is byte-for-byte the old `>= threshold` comparison, and the
original case table is kept verbatim as the zero-band block to prove that.

**Flagged, not done:** 100 us is a compile-time constant in `rc_switch.h`. It belongs in
`config/vehicle_params.yaml` as `limits.mux_kill_switch_hysteresis_us`, next to the threshold.
It was not added because that file is being edited on another branch and because the right
value is a measurement (how far this channel actually wanders), not a convention. It flows
through `RawMuxParamFields` like every other param, so the day it becomes a field it is a
two-line change in `pico/main.c`.

### 4. Watchdog and heartbeat

**MCU side.** The +Inf-when-never-seen path is correct and `watchdog_timed_out()` treats
non-finite as timed out. Wraparound is handled (`now < last` returns +Inf; `time_us_64()`
wraps after roughly 584,000 years, so this is belt and braces). Stuck HIGH and stuck LOW both
time out correctly, because the handler is registered for both edges and records only that an
edge happened, never a level -- a level-triggered design would have failed here and this one
does not. Two fixes: the torn 64-bit read described above, and the fail-open NaN/+Inf
`timeout_s` (defect 4).

**Jetson side.** Deadline scheduling is correct: `clock_gettime` once, then
`clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)` against a deadline advanced by one
half-period per iteration, so scheduling latency does not accumulate as drift, and
`clock_nanosleep`'s return value is used directly rather than through `errno` (it does not set
it). EINTR is retried unless stopping.

- **Line busy:** `gpiod_line_request_output` fails, the program exits 1 with a specific
  message naming the line and "already claimed by another process?". Correct behaviour on its
  own -- but see the systemd finding below, which turned it into a permanent stop.
- **Chip renamed by a JetPack update:** `gpiod_chip_open_by_name("gpiochip0")` fails, exit 1
  with the chip name and `strerror`. Same systemd interaction. This is a real risk on Orin
  (the chip enumeration order between `tegra234-gpio` and `tegra234-gpio-aon` is not a
  contract), and the robust fix is to resolve the line by its NAME (`PAC.06`) with
  `gpiod_line_find()` and fall back to chip+offset. **Not done in this pass**: it is an
  enhancement to code that is installed and verified working on the bench Jetson, not a
  defect, and it deserves its own change with the loopback test behind it. Listed as a
  follow-up below. The `ExecStart` arguments are now covered by a unit test so the unit and
  the documented pin mapping cannot drift apart silently.
- **systemd hardening:** `Restart=always` and `RestartSec=1` were already right, and
  `RestartSec=1` being ten times the 0.1 s watchdog window is correct, not a bug: a crash of
  this process IS a cut, and should be. What was missing is `StartLimitIntervalSec=0`.
  systemd's default start rate limit (5 starts in 10 s) would have put the unit in a failed
  state and stopped restarting it after about five seconds of any persistent failure -- the
  two failures above included. The mux fails safe either way, but a watchdog source that has
  permanently given up retrying is not something to find out about at the bench. Also added
  `Conflicts=shutdown.target` and `Before=shutdown.target`: `DefaultDependencies=no` drops the
  implicit shutdown ordering, so without them the heartbeat is killed at an arbitrary point
  in the shutdown rather than stopped cleanly.
- **`WantedBy=multi-user.target`:** correct as-is. With `DefaultDependencies=no` +
  `After=sysinit.target` the unit starts as soon as the transition reaches it, and
  `multi-user.target` is the right install target for something that should be running
  whenever the machine is up. Pulling it into `sysinit.target` instead would buy a fraction of
  a second at the cost of ordering it against units it has no business preceding.
- **Should it refuse to start when ROS is not running?** No, and confirmed. The heartbeat
  proves the OS is alive and servicing low-level I/O; that is the entire claim
  (`claude-docs/05-safety.md` layer 1: the mux catches a "Jetson freeze, Linux hang, software
  crash, ROS deadlock", and a heartbeat that stopped because ROS stopped would conflate two
  different failures). The README already said this in three places (Requirements, the unit's
  own comment, `heartbeat_input.h`'s design note). It said it as a dependency statement, so a
  short explicit "what this does NOT prove" paragraph has been added: a running heartbeat is
  not evidence that ROS, the control stack, or anything above the kernel is healthy, and
  nothing may be built on top of it that assumes otherwise.

### 5. Fail-safe defaults

Now yes to all three, and it was no to all three before. Every output GPIO is initialised to
its safe state in the first three statements of `main()`, before `stdio_init_all()` and before
any param is read (question 1 has the timeline). GPIO 8 goes LOW in the first statement and is
only ever raised by `power_cutoff_set_enabled(gpio, !output.cut)` from a `mux_decide()` that
returned `cut == false`, which cannot happen before the loop starts. The param-refusal halt
now runs strictly after that safe state is established, so a fast-blinking board holds GPIO 6
and 7 driven LOW (no pulses for a servo or an ESC to act on) and GPIO 8 LOW (cutoff open) for
as long as it is powered. `fault_halt_missing_param()` carries that precondition as a comment
so it does not get moved back up.

The fault message also now distinguishes the three ways a param can be unusable: null,
non-finite, or out of range.

### 6. Simplicity

Removed rather than added, where equivalence was provable:

- `pwm_capture_init_channel()`'s trailing `if (!gpio_irq_dispatch_register(...)) { return; }`
  was a `return` at the end of a void function. It is now `(void)`-cast, matching
  `heartbeat_input.c`'s identical call.
- `pwm_output_init_channel()` computed a `channel` it never used.
- `mux_params.c` was ten near-identical `if` blocks. The original comment defended that
  against a table, and it was defensible at ten; it is not at twenty-two, which is what adding
  a finiteness check per field would have made it. It is now one `{field, name}` table and one
  loop -- the failure mode of twenty-two near-identical blocks is one of them naming the wrong
  field, and the existing per-field tests prove the reporting order is unchanged.

Deliberately NOT changed: the decision-priority `if` chain in `mux_decision.c` (the ordering
IS the safety argument and reads better spelled out), the `gpio_irq_dispatch` linear scan, and
the plain-`gcc` test framework. No duplicated checks were found between `logic/` and `pico/`:
the new capture-side plausibility band uses `logic/`'s own `pwm_is_valid_us()` rather than
re-implementing a comparison, and is a different question (is this electrically a pulse) from
the one `logic/` answers (is this a legal command).

### 7. Pin remap readiness

**Two-file claim: almost true, and the exceptions are all documentation.** Every GPIO number
that the *firmware* uses lives only in `pico/main.c`'s seven `#define`s. `logic/` contains no
pin numbers, the tests contain none, and there is no board page generator in this repo (the
perfboard layout lives in prose in `docs/notes/build-log.md`). So a remap is a code change in
exactly one file.

It is not a two-file change overall, because the pin numbers are also written down in five
other places that will be wrong the moment the holes are confirmed:

| File | What it says |
|---|---|
| `firmware/safety_mux/README.md` | pinout table, connector map, "GPIO 2 to 8 is on the lower row", the flashing note's GPIO 6/7/8 |
| `docs/notes/safety-mux-first-build.md` | "What to expect when you flash it" (GPIO 2/3/4/5/6/7/8 by number, and GPIO 25 for the LED) |
| `docs/notes/build-log.md` | 2026-09-11 wiring plan (GP2 to GP8 with perfboard columns), 2026-09-12 entries |
| `planning-docs/05-safety-mux-and-kill-test.md` | step 2's GPIO list, step 10's GPIO 8 cutoff |
| `planning-docs/06-vesc-config-and-jetson-bringup.md` | "the pin wired to the mux's GPIO 5" |
| `ros_ws/src/racer_drivers/README.md` | cabling table rows naming RP2040 GPIO 3 and GPIO 4 |

The README now says so explicitly, so whoever does the remap has the list instead of
rediscovering it with `grep`.

### 8. Mux accepted range vs. what `pwm_mapping` can emit

They agree, exactly, and by construction: both sides read the SAME `vehicle_params` fields
through generated bindings and neither hand-writes a bound.

- `racer_drivers::speed_to_pulse_us` / `steering_angle_to_pulse_us` finish with
  `std::clamp(pulse_us, channel.min_us, channel.max_us)`, and non-finite or stale commands map
  to `neutral_us`. The emitted set is exactly `[min_us, max_us]`, endpoints included.
- The mux's `pwm_is_valid_us()` accepts `[min_us, max_us]`, endpoints included (there are
  boundary cases for both endpoints in `test_pwm_validity.c`).
- Neutral is the same field on both sides (`steering.pwm_neutral_us`,
  `actuation.throttle_pwm_neutral_us`), currently 1500 us, and 1500 is inside the accepted
  range, so the driver's neutral never reads as a cut-worthy command.

So a legal command cannot be cut on WIDTH. One genuine mismatch exists, on RATE, and it is
new with this change: the mux now requires a pulse every 60 ms, while `pwm_output_node`
declares `output_rate_hz` with an accepted range of **1.0 to 400.0 Hz**. Anything below about
17 Hz would make the mux cut a perfectly legal command stream. 50 Hz (the default, and what
the node's own documentation and the hardware both assume) has a 3x margin. This is recorded
in the subject-to-change list below rather than fixed here, because the fix is either a
narrower bound on that ROS parameter or a `vehicle_params` field both sides read, and
`ros_ws/` and `vehicle_params.yaml` are out of scope for this branch.

## Fixes made

`logic/` (all host-tested):

1. `pwm_validity.c`: non-finite bounds are invalid (was fail-open).
2. `watchdog.c`: non-finite or non-positive `timeout_s` is timed out (was fail-open).
3. `rc_switch.c`: hysteresis band with a held position; non-finite threshold cuts rather than
   holding; signature gained `hysteresis_us` and `previous`.
4. `mux_decision.c`: carries the switch position in and out; no change to the priority order.
5. `mux_params.c`: rejects non-finite values and structurally unusable sets (neutral outside
   its own min/max, non-positive watchdog timeout, negative hysteresis, inverted receiver
   range, threshold outside that range), each naming its field and a problem kind. Table-driven
   rather than 22 blocks.

`pico/` (compiles clean, unverified on hardware):

6. `main.c`: outputs to their safe state first, neutral applied as soon as it is known, inputs
   last, switch position carried across cycles, fault message names the problem kind.
7. `pwm_output.c/.h`: new `pwm_output_init_safe()`; `pwm_output_init_channel()` takes the
   initial pulse and programs it before enabling the pin; `pwm_output_set_us()` clamps
   non-finite, negative, and over-frame values instead of casting them into a `uint16_t`.
8. `pwm_capture.c/.h`: staleness window, plausibility band, atomic read, `have_rise` flag,
   dead `return` removed.
9. `heartbeat_input.c`: atomic read of the edge timestamp.

`tools/jetson_heartbeat/`:

10. `config.c`: `--line` requires a leading digit (`strtoul` accepted signs and whitespace;
    `-18446744073709551615` parsed as line 1); `--chip` rejects the empty string; `--rate-hz`
    gained a 10 kHz ceiling so a typo cannot turn the toggle loop into a spin.
11. `systemd/racer-heartbeat.service`: `StartLimitIntervalSec=0`, `Conflicts=shutdown.target`,
    `Before=shutdown.target`.

Both test frameworks now count and print the assertions they ran, so a suite that silently
stopped running is distinguishable from a suite that passed.

**Tests:** `firmware/safety_mux` 91 -> 320 assertions, `tools/jetson_heartbeat` 34 -> 58. Both
green, both under `-Wall -Wextra -Werror -Wpedantic`. No tolerance, golden file, or gate was
weakened.

**Build:** same container and pinned SDK as `docs/notes/safety-mux-first-build.md`
(`debian:bookworm` arm64, arm-none-eabi-gcc 12.2, pico-sdk 2.1.0 via FetchContent), from a
clean tree, zero compiler warnings. `firmware/safety_mux/build-artifacts/safety_mux.uf2`,
**79,360 bytes**, sha256
`696703811737e8acdcf3431f94ff1d0825d97829d6d0c9e59a5747485e12b41b`. Gitignored; regenerate it
rather than trusting a copy.

## Subject to change when the car is assembled

Everything below is a number or a decision this firmware currently carries that nobody has
measured. This list is the firmware half of
`docs/notes/hardware-arrival-checklist.md` section 3.

1. **Pin assignments.** All seven `#define`s in `pico/main.c`, blocked on the final perfboard
   hole confirmation. Question 7 above lists every other file that repeats them.
2. **PWM ranges and neutrals.** `steering.pwm_{min,neutral,max}_us` and
   `actuation.throttle_pwm_{min,neutral,max}_us`, all six PROVISIONAL at 1000/1500/2000.
   The throttle neutral is the dangerous one: if this ESC's real zero-throttle point is not
   1500 us, the CUT state commands a creep.
3. **Watchdog timeout.** `limits.mux_watchdog_timeout_s` = 0.1, a conservative guess, not
   tuned against measured heartbeat jitter under load.
4. **Kill-switch threshold.** `limits.mux_kill_switch_threshold_us` = 1500, the midpoint of an
   assumed range.
5. **Kill-switch hysteresis.** 100 us, compile-time in `rc_switch.h`, wants to become
   `limits.mux_kill_switch_hysteresis_us`. Measure how far the channel actually wanders first.
6. **Kill polarity.** Which end of the channel is ARMED is UNMEASURED. If flipping the switch
   arms the car backwards, that is this assumption, not a bug. Measure the channel before
   trusting it, wheels off the ground.
7. **RC receiver signal range.** 1000/2000 us, hardcoded in `pico/main.c`'s
   `raw_fields_from_generated_params()` because it describes the receiver, not the vehicle.
   Replace with the datasheet or measured range.
8. **Capture staleness window.** `PWM_CAPTURE_MAX_AGE_US` = 60 ms, i.e. three missed 50 Hz
   frames. Re-derive if the frame rate changes.
9. **Capture plausibility band.** 200 to 5000 us. Widen or narrow once the level shifter's
   real edge behaviour has been seen on a scope.
10. **The 50 Hz agreement itself.** The mux, `pwm_output_node`'s `output_rate_hz` default, and
    the servo/ESC all assume 50 Hz, but that ROS parameter accepts 1 to 400 Hz and nothing
    enforces the agreement (question 8).
11. **PWM output clock assumption.** `pwm_output.c`'s `clkdiv = 62.5` assumes a 125 MHz
    `clk_sys`. Confirm on the actual board, or compute it from `clock_get_hz(clk_sys)`.
12. **Power cutoff circuit.** GPIO 8 drives nothing yet. Relay vs. MOSFET, gate drive, flyback
    protection, and whether active-HIGH-equals-enabled survives contact with the real circuit
    are all open. Until it exists, a "cut" is a command cut only, not a power cut.
13. **Heartbeat rate and line.** 50 Hz on `gpiochip0` line 144. The rate is an operator choice
    against the watchdog timeout, and the chip/line resolution is by number rather than by
    name (question 4).
14. **Fault LED.** `PICO_DEFAULT_LED_PIN`, and the arming path never touches it, so "off"
    means armed OR dead. A real arm/heartbeat blink is worth adding once the board is proven.

## Follow-ups not done here

- Add `limits.mux_kill_switch_hysteresis_us` to `config/vehicle_params.yaml` and its schema,
  and read it in `raw_fields_from_generated_params()` instead of the compile-time constant.
  Same for the capture staleness window if a `vehicle_params` home is wanted for it.
- Resolve the heartbeat's GPIO by line NAME (`PAC.06`) with `gpiod_line_find()`, falling back
  to chip+offset, so a JetPack update that renumbers the chips does not stop the heartbeat.
- Narrow `pwm_output_node`'s `output_rate_hz` bounds, or derive both sides' frame-rate
  assumption from one place.
- Promote the Pico cross-compile to a CI job (`CMakeLists.txt` is already written for it), and
  the host tests to an enforced coverage gate, once this is closer to bench-tested than
  drafted.
- An arm/alive blink on the fault LED, so a dark board is not ambiguous.
