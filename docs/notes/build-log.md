# Build log

Dated entries for physical build decisions: what changed on the car or on a board, why, and
what still has to be proven on the bench before the decision counts as correct. Design docs
say how things are meant to be; this file says when a choice was made and on what grounds.
Nothing here is a test result unless it says it was observed.

## 2026-09-14 -- safety mux firmware and heartbeat reviewed before the first flash

Read-and-fix pass over `firmware/safety_mux/` and `tools/jetson_heartbeat/` ahead of flashing
the Pico for the first time. Full findings, the traced startup timeline, and the
subject-to-change list: `docs/notes/firmware-review-2026-09-14.md`. No hardware involved;
nothing here has run on a chip.

Eight defects found and fixed, four of them fail-OPEN:

- **Output pins floated at boot, and forever on a refusal.** `main()` initialized the servo,
  ESC and cutoff pins after reading params, so they sat high-impedance from reset, and the
  refuse-to-arm halt ran before any of them. A floating ESC signal line is what some ESCs arm
  on. All three are now driven to their safe state in the first three statements of `main()`,
  before anything else, and the configured neutral is applied the moment it is known.
- **A captured PWM pulse never aged out.** A stuck-high input produces no further falling
  edges, so the last width captured before the fault was reported as a live command forever.
  There is now a 60 ms staleness window (three missed 50 Hz frames) on every capture channel.
- **NaN PWM bounds made every pulse valid, and a NaN watchdog timeout never tripped.** Both
  are broken-config paths rather than runtime events, but both were fail-open. Both now cut,
  and `mux_params.c` additionally refuses to arm on a non-finite or structurally unusable
  param set (a neutral outside its own channel range, for instance).
- **Torn reads from interrupt context.** A `double` and a `uint64_t` are each two 32-bit
  accesses on a Cortex-M0+, so the main loop could read half of one pulse and half of the
  next, or a heartbeat timestamp 71 minutes in the past. Both reads are now taken with
  interrupts briefly disabled.
- **No kill-switch hysteresis.** A channel resting near the 1500 us threshold would flap
  ARM/KILL at the 200 Hz loop rate. There is now a 100 us dead band (arm at 1600, kill below
  1400, hold between), compile-time for now, flagged to become a `vehicle_params` field once
  the channel has been scoped. Holding never holds ARMED out of an unknown state, so a switch
  parked in the band at power-on reads KILL.
- **The heartbeat service would have permanently given up.** systemd's default start rate
  limit stops restarting a unit after 5 starts in 10 s, so a busy GPIO line or a renamed
  gpiochip would have killed the heartbeat for good after about five seconds. Fixed with
  `StartLimitIntervalSec=0`, plus the shutdown ordering that `DefaultDependencies=no` drops.

Host tests: 91 to 320 assertions on the mux, 34 to 58 on the heartbeat, all table-driven,
covering the NaN/inf and boundary cases each fix is about. The firmware cross-compiles clean
in the same pinned container as the 2026-09-12 first build: 79,360 byte `.uf2`, sha256
`696703811737e8acdcf3431f94ff1d0825d97829d6d0c9e59a5747485e12b41b`.

None of this is bench-verified and none of it moves roadmap 1.3. It does mean the first flash
exercises firmware whose startup sequence and fail-open paths have been read properly, rather
than finding them with a servo attached. `config/vehicle_params.yaml` was deliberately not
touched (another branch is editing it), so the two constants that want to live there stay
compile-time and are listed as follow-ups.

## 2026-09-14 -- motor sensor cable: adapter required, not a repin

Researched the Hobbywing-to-VESC sensor connection properly after it came up as a surprise
item. Earlier notes called it a "repin", which was misleading. Findings:

- The connectors are physically incompatible: Hobbywing motor side is JST ZH at 1.5 mm pitch,
  the VESC SENSE port is JST PH at 2.0 mm pitch. They cannot mate, so there is no accidental
  wrong-plug failure mode. Hobbywing sells a sensor adapter cable as a product, which
  corroborates this. An adapter must be bought or built either way.
- Hall signal ORDER is a non-issue. VESC Tool's hall detection rotates the rotor under FOC
  current control and records the electrical angle for each hall state, learning the table.
  Any permutation of the three hall lines detects correctly (vesc-project.com/node/1815,
  /node/451).
- Power, ground and temperature placement are the only dangerous part. Driving +5 V into a
  hall output can destroy that hall IC's output transistor; shorting +5 V to ground stresses
  the VESC's onboard sensor-supply regulator.
- No manufacturer pin table was found for the Xerun 3652SD G3 (HW30401064) specifically. The
  EFRA Members Handbook 2023 App.4 s4.2 governs this motor class and specifies black=GND,
  orange=hall C, white=hall B, green=hall A, blue=10k NTC thermistor, red=+5 V. One pev.dev
  forum post conflicts, possibly describing a Hobbywing ESC header rather than the motor
  cable. Treat both as hypotheses: measure before wiring.
- Sensorless operation is safe for the first bench spin and needs no sensor cable at all.
  What is lost is low-speed smoothness and startup torque below roughly 2000 ERPM; above that
  it is indistinguishable. No electrical risk to the motor, since the hall sensors are simply
  unpowered. Use conservative current limits for the first run regardless.

Decision: first spin runs SENSORLESS. The adapter is a separate, later session with a
multimeter, using the measurement procedure now recorded in the arrival checklist.

## 2026-09-14 -- capacitors dropped, proceeding on the FSESC 6.7 knowingly, perfboard and motor soldering underway

Reported by Cameron, no code touched. Several small physical decisions and a status snapshot
of the shop work in progress.

**Capacitors: decided not to fit.** The separate 1000 uF caps called out in the BOM and in
`planning-docs/02-bench-prep-and-soldering.md` stay in the parts bin. The FSESC has three
onboard bulk capacitors and the battery leads to it are short, so the extra capacitance buys
nothing here. This closes the open item noted in `claude-docs/11-hardware.md`'s BOM audit
("optional insurance, not a requirement, if battery leads are kept short") -- the leads are
short, so they are skipped. `planning-docs/02-bench-prep-and-soldering.md` and
`docs/notes/hardware-arrival-checklist.md` are corrected to stop instructing a fit.

**ESC: proceeding on the FSESC 6.7 for the test build, eyes open.** The Flipsky FSESC 6.7 is
going into the car now even though Flipsky specs it 14-60 V, 4S minimum, and the pack is 3S
(9.6-12.6 V) -- see the 2026-09-11 entry below for how that mismatch was found and why the
3S-rated FSESC 4.12 was ordered as the real fix. The 4.12 is still in transit, roughly
Sep 22-29. This is a deliberate, informed decision, not a repeat of the same mistake: the
known risk is that an ESC run below its rated minimum most likely boots and drives fine at
light load, and the untested case is sustained load, where the ESC's own low-voltage
assumptions (gate drive, BEC regulation headroom) may not hold. Nothing about the swap gets
harder for having done this first: the connectors are the only shared work between the two
ESCs, so replacing the 6.7 with the 4.12 when it arrives is still a small job. Wheels stay off
the ground for every test run on the 6.7, same as everything else at this stage.

**Perfboard / kill-switch board: soldering, not finished.** All terminal pins are soldered,
and the Pico socket and both level-shifter sockets are soldered in. Currently soldering the
wires between them. Not yet finished, not yet flashed, not yet bench tested. The pin mapping
between the perfboard's holes and the Pico's GPIO row (`firmware/safety_mux/README.md`'s
board connector map) is still not finalized -- Cameron has not yet confirmed the final holes.
That confirmation is the one open blocker before flashing: once it lands, the firmware
`#define`s and the README's pinout table get updated together, in the same change.

**Motor: mounted, phase wires being soldered direct.** The Xerun 3652SD G3 is mounted with
the stock pinion (see stage 3's shaft/pinion notes). The three VESC phase wires are being
soldered directly to the motor rather than through bullet connectors: only one female bullet
connector was on hand, so direct solder with individual heat shrink on each joint was chosen
instead of a mismatched connector set. These joints are expected to be cut and redone when
the FSESC 4.12 arrives and the ESC gets swapped.

**Car: disassembled, laid out for the electronics.** Stock ESC and stock receiver are both
out (receiver bagged, per stage 3). Parts are being laid out on the chassis ahead of the deck
layout and mounting pass.

**Jetson: set up and idle.** JetPack 6.2 flashed, on WiFi, SSH key auth plus passwordless
sudo confirmed, and `racer-heartbeat.service` re-verified toggling header pin 7 at 50 Hz (see
the 2026-09-12 entry below for how that pin mapping was determined). The Jetson itself is
currently powered off; none of this needs the rest of the car assembled, so it can resume any
time it is powered.

**Forward plan, in order.** Finish soldering the perfboard wires; solder the VESC phase wires
to the motor (direct, heat shrink each joint -- already started, above); lay out and mount the
electronics on the car (deck layout, standoffs, routing). Then, before anything drives: EC5
onto the VESC's battery leads; first LiPo charge once the charger arrives; finalize the Pico
pin mapping and flash the firmware (blocked on Cameron confirming the holes, above);
continuity checks before any battery is connected; bench power-up of both rails with a
multimeter; the Jetson's PWM pinmux enable and `pwmchip` numbering confirmation
(`docs/notes/first-boot-runbook.md` steps 4-5, needs only the Jetson powered); wiring the four
Jetson lines to the mux board; the wheels-off bench test sequence; then the G1 kill test. A
fuller breakdown with what blocks what is in
`docs/notes/build-status-2026-09-14.md`.

## 2026-09-13 -- two provisional numbers written down: throttle full scale, and the TTC thresholds

Two gaps found in this morning's first-boot sweep, both closed the same way the nine
safety-mux PWM values were closed on 2026-09-12: supply a value, mark it PROVISIONAL, name
the bench step that replaces it, and touch none of the refusal logic. Nothing here has been
on hardware. GitHub issues #40 and #36.

**`actuation.throttle_full_scale_mps`, new field, PROVISIONAL 5.0 m/s (issue #40).**
`pwm_output_node`'s open-loop throttle map had no full-scale reference of its own, so it used
`limits.global_speed_cap_mps` -- 20 m/s, the f1tenth_gym dynamic-model validity bound, which
is not an actuation scale at all. At that scale a commanded 1 m/s is a pulse **25 us** off
neutral, comfortably inside VESC Tool's default PPM deadband, so the first gentle keyboard
teleop command would very likely have produced no motion and looked exactly like a wiring
fault on a car nobody has driven yet. With 5.0 m/s and the 1000/1500/2000 us ends, **1 m/s is
now 100 us off neutral: 1600 us forward, 1400 us reverse**. The 5.0 is a guess about a car
that does not exist; the runbook's wheels-off-the-ground throttle step now says to sweep the
command, record wheel speed against pulse width, and write down the speed observed at
`throttle_pwm_max_us`. The field disappears when `vesc_node` closes the loop.

`limits.global_speed_cap_mps` was NOT lowered and NOT repurposed -- it is still the clamp it
was, applied to the command before the map runs, and lowering it still tightens both this
node and `safety_node`. What changed is that the cap is no longer doing a second job it was
never suited for. A command above the full scale but below the cap is not rejected: it
saturates the pulse at the channel end, and there are now gtests for both that and the
cap clamp, plus one pinning the 1 m/s -> 100 us slope against the committed numbers.

**`limits.ttc_warning_s` 1.0 and `limits.ttc_brake_s` 0.5, PROVISIONAL (issue #36).** Both
were `null`, which `safety_node` correctly maps to "gate disabled". That was honest but it
meant the layer-3 TTC brake would have stayed inert the moment a LiDAR was fitted -- the
safety gate most likely to be assumed present on a first drive. These are the conventional
F1TENTH-class starting points, and they are literally the pair the L3 TTC test had been
passing as a launch override since the gate was written; they are **not** tuned against this
car's braking distance, which nobody has measured. Phase 1/2 replaces them.

Two things stay true and are written into the config and the node comment. First, with no
`/scan` publisher the gate is still a clean no-op regardless of these numbers, and there is
now a launch test that asserts exactly that (a 5 m/s command passes through untouched, no
`ttc` event) rather than leaving it inferred. Second, `safety_node` has no `/odom`, so the
forward speed it divides range by is **its own commanded speed**. Slower than commanded means
TTC is under-estimated and it brakes early, which is the safe direction; coasting faster than
commanded means it is over-estimated. That is a documented limitation until `/odom` exists,
not something these thresholds fix.

**No gate was weakened.** The refuse-to-start-if-null path in `pwm_output_node` is untouched
and now also covers the new field (it is nullable in the schema, so the generated binding
types it as an optional and the refusal is live). `null` still disables the TTC gate. No
tolerance, golden file or coverage gate moved.

**Schema.** `meta.schema_version` 0.2.1 -> 0.2.2: one field added to `actuation` (nullable,
with units and description, like the other unmeasured fields) and two values filled in. Rule
5 bumps on any change. Bindings regenerated with `tools/gen_params.py`, never hand-edited.
The three `racer_policy` tests that pin the version literal were updated, and
`tools/tests/fixtures/full_fixture.yaml` gained the new required key.

**Tests.** `.github/scripts/ros_build_test.sh` in the `ros-dev` container: **265 tests before,
270 after, 0 failures** both times. `.github/scripts/run_python_tests.sh` green (all gated
packages, `tools` at 92 including the gen_params round-trips). ruff and clang-format clean.
Observed in the container: `safety_node up: ... ttc_brake_s=0.500000, ttc_warning_s=1.000000`
with no launch override in sight, and `pwm_output_node up ... [1000/1500/2000 us, full scale
5.00 m/s, cap 20.00 m/s, OPEN LOOP PROVISIONAL]`.
## 2026-09-13 -- /safety/events now records gate transitions, not every cycle

Software only; the car was not powered. Fixes GitHub issue #37, which came out of the
first-boot audit (`docs/notes/first-boot-audit-2026-09-13.md`, finding #5): `safety_node`
published one `/safety/events` record per engaged gate **per control cycle**, so a gate that
stayed engaged logged itself at 50 Hz. Measured on the bench-equivalent reproducer: 248
`watchdog` records in 14 s from a node that had simply not been given a `/drive_raw`
publisher yet. `claude-docs/09-evaluation.md` reports the intervention count as a metric, so
as it stood that metric measured how long the operator took to bring the stack up, not how
the car behaved. Comparing a learned policy against the baseline on that number would have
been comparing start-up latency.

**What changed.** One intervention is now one ENGAGEMENT of a gate: exactly one record when
it engages, exactly one when it releases, nothing in between. No periodic "still engaged"
record was added -- the interval between the two records is the sustained state, and one
fewer record class is one fewer thing a counter has to know to ignore. The engagement's
identity is the (source, severity) pair rather than source alone, so a TTC advisory
escalating to a TTC brake is a release plus a new engage rather than a silent change of
character inside one record; an evaluation counting BRAKE-severity interventions still sees
the brake. Gates are independent: several can be engaged at once, each with its own
lifecycle.

**Gating behaviour is untouched.** What gets clamped, braked, or passed through is
byte-for-byte the same decision it was: `SafetyGateLogic::evaluate` still reports every
engaged gate every cycle, and the new `GateEventTracker` sits between that and the
publisher. Fail-closed and the watchdog are unchanged, and the fault path publishes through
the same tracker, so a sustained internal fault is one record and any gate engaged before
the fault gets its release.

**Interface change.** `racer_msgs/SafetyEvent.msg` gained `phase` (`PHASE_ENGAGE` /
`PHASE_RELEASE`) and `duration_s`. `PHASE_ENGAGE` is the zero default deliberately, so an
older bag with no `phase` field reads back as engagements, which is what those records meant.
Bags recorded before today still deserialize; their counts still mean "cycles", not
"interventions", and nothing in the repo has evaluation numbers derived from them yet.

**Counting rule, written down in three places** (the message file, `gate_logic.hpp`, and the
L3 tests): an intervention count is a count of `PHASE_ENGAGE` records. The two sim end-to-end
tests that count `/safety/events` were updated to filter on that; their tolerance ceiling was
left where it is rather than re-tightened against a single post-fix sample.

**Tests.** The edge detection is pure and ROS-free, so it is table-driven gtest: every gate
source x every severity through engage/sustain/release, 100 sustained cycles emitting
nothing, simultaneous gates with independent durations, flapping (engage, release, engage
within three cycles = three records), severity escalation, a duplicate activation in one
cycle, and garbage clock input (NaN, +/-Inf, a clock that goes backwards -- all report a 0.0
duration rather than a negative or non-finite one). The L3 launch tests now assert exactly
one engage per intervention and one on release with a plausible duration, and a new
regression test asserts ZERO records over a 3 s window in which a gate is engaged the whole
time -- the direct inverse of the 248-records-in-14-s reproducer. 265 tests before, 277
after, all green in `ros-dev:local`; the gate-logic branch-coverage gate stays at 100%
(96 branches).

**Not done.** A gate still engaged when the node exits never gets its release record. That
is accepted: the node is gone and there is nobody to publish it, so a bag reader should treat
a trailing engage as "engaged until end of bag".
## 2026-09-13 -- first-boot audit follow-ups: teleop cleanup, ros-dev deps, audit status

Three small fixes out of `docs/notes/first-boot-audit-2026-09-13.md`, on
`chore/first-boot-followups`. No hardware touched.

- **`racer_tools/keyboard_teleop_node.py` finding #6 fixed.** `main`'s `finally` block used
  to reference `node` even when `KeyboardTeleopNode()` itself raised before `node` was ever
  assigned, so a real constructor failure (e.g. `vehicle_params_loader` not finding the repo
  root) surfaced as a masking `NameError` instead. `node` now starts `None` and `finally`
  only calls `destroy_node()` when construction actually succeeded. New unit test
  (`test/test_keyboard_teleop_node_main.py`) simulates a constructor failure and asserts the
  original exception type propagates.
- **`docker/ros-dev/Dockerfile` apt gap closed.** The audit's test-results note flagged that
  a fresh `ros-dev` container could not `colcon build` `ros_ws` until `rosdep install` had
  run, because the image did not ship `ros-humble-ackermann-msgs`. Ran
  `rosdep install --simulate --from-paths ros_ws/src --ignore-src` inside the image to find
  the full gap: `ros-humble-ackermann-msgs` and `python3-jsonschema`, both now added to the
  apt install list with a comment. Verified: built `ros-dev:test` from the changed
  Dockerfile, then ran a clean `colcon build --symlink-install` of `ros_ws` in a fresh
  container from it with no `rosdep install` step -- succeeds. Base image digest unchanged.
- **`docs/notes/first-boot-audit-2026-09-13.md` status note added.** Findings #7 and #8 were
  gaps as of the audit; PR #39 (merged to `main`) since added a car launch file and
  `racer_drivers/pwm_output_node`. Added a dated "Status after PR #39" note under the
  findings table: #7 is closed, #8 (`racer_state` still empty, covariance gate still a stub)
  remains open. The audit text itself was not rewritten.

## 2026-09-13 -- the Jetson can now command the car: pwm_output_node, car_teleop launch, torch-optional car image

The missing piece between `/drive` and the wires. Nothing in this entry has touched hardware:
the Jetson was powered off all day, so everything on-device below is a written-ahead
procedure, and everything proven was proven in the `ros-dev` container on the Mac against a
FAKE sysfs tree.

**`ros_ws/src/racer_drivers/pwm_output_node` (new, C++).** Subscribes `/drive` (reliable,
depth 10) and nothing else on the command path -- never `/drive_raw`, which would route
around `safety_node` (`CLAUDE.md` invariant 1); both L3 launch tests assert that against the
live node graph rather than by reading the source. It maps `steering_angle` (rad, left
positive) and `speed` (m/s) onto two 50 Hz servo pulses through the Linux sysfs PWM interface
(`/sys/class/pwm/pwmchipN/pwmM`). This is the command path decided on 2026-09-12: PWM through
the mux into the VESC's PPM input, USB to the VESC for telemetry and configuration only.

Fail-closed in five places, all tested: neutral on both channels at startup before the
subscription exists; neutral while no `/drive` has arrived; neutral when `/drive` goes silent
for `drive_timeout_s` (default 0.1 s, a declared parameter -- deliberately NOT
`limits.mux_watchdog_timeout_s`, which is the layer-1 MCU's own heartbeat window on a
different device); neutral on any exception in the cycle; and neutral-then-disable on
SIGINT/SIGTERM. There is no path that leaves a stale non-neutral pulse.

Every pulse bound, angle limit and the speed reference comes from `config/vehicle_params.yaml`
through the generated C++ binding, and the node refuses to start naming the field if one is
null -- the same discipline as `firmware/safety_mux`'s `logic/mux_params.c`. None of the
fields it needs is null today, so it starts. **That is not good news about the numbers**: the
six PWM values are the same unmeasured 1000/1500/2000 convention placeholders filled in on
2026-09-12, and `limits.global_speed_cap_mps` (20 m/s, a dynamics-model validity bound) is
the throttle map's full-scale reference, which means a commanded 1 m/s is a pulse 25 us off
neutral. Both must be measured and lowered before the car is on the floor.

**The throttle map is open loop and provisional and says so everywhere.** PPM commands duty
or current, not speed; there is no feedback in this node. It exists so first boot can happen
and is replaced when `vesc_node` does.

**Steering polarity is an unknown, not an assumption.** No project doc says which pulse end
is full LEFT, so it is a declared parameter (`steering_left_is_pwm_max`, default true) marked
bench-calibrated in the code, the README and the runbook. Step 12 of the runbook is where it
gets settled.

**`racer_bringup/launch/car_teleop.launch.py` (new).** `safety_node` + `pwm_output_node` +
`foxglove_bridge` (8765), no sim bridge. Both teleop sources default OFF, unlike the sim
launch: a launch file that moves a physical car comes up with no command source until an
operator picks one.

**Tests.** L1 gtest over the ROS-free mapping and the output sequencing (table-driven: at
bound, epsilon over, NaN, inf, both steering polarities, in-memory fake sinks); L3
launch_testing for the node alone and for the car launch, both against a temp-directory
"sysfs" of ordinary files, asserting neutral-on-start, correct mapped pulses, neutral on
silence, reliable-QoS incompatibility with a best_effort publisher, and neutral-plus-disabled
after shutdown. `.github/scripts/ros_build_test.sh` (what the `l3-and-cpp` CI job runs) was
executed verbatim in the container: **251 tests, 0 failures**. The new tests need no CI
wiring because that job already builds and tests all of `ros_ws`.

Also observed, headless in the container: `car_teleop.launch.py` up with a fake sysfs root,
`/drive` steady at `steering_angle: 0.0, speed: 0.0`, both channels `period=20000000
duty_cycle=1500000 enable=1`, and after SIGINT `duty_cycle=1500000 enable=0`.

**`docker/car`: torch is now optional.** `INSTALL_TORCH=skip` (the new default) builds without
torch and shouts about it -- a NOTICE block in the build log, `/etc/racer/torch-status`,
an `/etc/racer/torch-absent` marker and `RACER_TORCH=absent` exported into every login shell
-- so nothing can quietly assume torch is present. `INSTALL_TORCH=required` keeps the old
refuse-if-`TORCH_WHEEL_URL`-unset behaviour exactly as it was. The reason is scope: torch is
there for `racer_policy` inference in roadmap phase 5.x, the phase in front of us imports
none of it, and the old Dockerfile blocked a first boot on a hand-resolved wheel URL that
first boot does not use. The three branches were exercised as a shell script in a plain
`ubuntu:22.04` container; the image itself still has never been built.

**Base image stays r36.4.0, and that is a decision, not an oversight.** The device reports
L4T R36.4.4 (JetPack 6.2.1) and the Dockerfile pins r36.4.0 (JetPack 6.1), so the obvious fix
is a matching tag. There is not one: `nvcr.io/nvidia/l4t-jetpack`'s full tag list, queried
against the registry API today, ends at `r36.4.0`, and `l4t-base` ends at `r36.2.0`. NVIDIA
has not published a JetPack 6.2 image. An older L4T container on a newer host of the same
major is the supported direction (the driver comes from the host), both are jammy, and the
pinned digest re-resolved unchanged. Whether the 6.1 CUDA stack behaves on a 6.2.1 host is an
on-device check; the control stack needs no CUDA, so it gates phase 5, not first boot.

**Docs.** `docs/notes/first-boot-runbook.md` is the numbered procedure: power up, clone,
build without torch, enable the PWM pinmux via `/opt/nvidia/jetson-io` and reboot, read the
real `pwmchip` numbers off the device, scope one channel bare, cable four wires to the mux
board, launch, verify `/drive` neutral, verify the pulses with a meter, then servo and ESC
with wheels off the ground, then keyboard teleop with the kill switch armed and a second
person present. `docker/car/build_on_jetson.md` was corrected against the device that now
exists (Docker installed, NVIDIA runtime present, `racer` in the docker group, passwordless
sudo, repo not yet cloned) -- the old draft was wrong on three of those.

**Cabling, planned and unsoldered.** Jetson pin 15 -> steering, pin 33 -> throttle (the two
hardware-PWM-capable pins on the 40-pin header), pin 7 -> heartbeat (already running), pin 9
-> ground; into the mux board's JETSON header at row 1, columns 8, 9, 10 and 11 respectively.
`firmware/safety_mux/README.md`'s connector map now names the two PWM pins alongside the
heartbeat pin it already named.

**What is unverified.** All of it, on-device: the pinmux change, the reboot, the resulting
`pwmchip` numbering, whether a pulse leaves the pad at all, the container's write access to
`/sys/class/pwm` (the runbook uses `--privileged`, which is a blunt instrument and should be
narrowed once something works), the image build, every calibration value, the steering
polarity, and the mux's response to any of these signals. The mux still has not been
kill-tested. Nothing here changes roadmap 1.3's status.

## 2026-09-12 -- Jetson-side heartbeat installed and running on the bench Jetson

Roadmap task 1.3 (`claude-docs/05-safety.md` layer 1). Built, installed, and verified the
Jetson-side half of the mux's heartbeat watchdog input on the bench Jetson (racer-car,
10.0.0.226, JetPack 6.2 / L4T R36.4.4). Source, systemd unit, and full detail:
`tools/jetson_heartbeat/` (its README has the reproduction steps for everything below).

**What was installed.** `tools/jetson_heartbeat/racer-heartbeat`, a small C program (built
against `libgpiod-dev` 1.6.3, installed via apt for this) that toggles one GPIO line at a
fixed rate using `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)` against an absolute
deadline (no drift accumulation), independent of ROS and the network stack. Installed to
`/opt/racer/jetson_heartbeat/racer-heartbeat` and run via a new systemd unit,
`racer-heartbeat.service` (`DefaultDependencies=no`, `After=sysinit.target`, `Restart=always`),
enabled and started. `systemctl status` shows `active (running)`; killing the process with
`SIGKILL` and re-checking status shows a fresh PID within about a second, confirming the
restart policy actually works and isn't just configured.

**Pin mapping.** Jetson 40-pin header **physical pin 7 (signal) == `gpiochip0` line 144**
(kernel name `PAC.06`, global gpio 492), **physical pin 9 == ground**. This was read off the
live pinmux device tree of this specific board using NVIDIA's own `/opt/nvidia/jetson-io`
tooling (`Jetson.board.Board` / `Jetson.header.Header`, which parse the running kernel's DT),
not assumed from a generic pinout diagram -- the pinmux-node name it returned for pin 7
(`soc_gpio59_pac6`) independently matches the line name `gpioinfo gpiochip0` prints for line
144 (`PAC.06`), which is the cross-check that makes this a determination rather than a guess.
`firmware/safety_mux/README.md`'s pinout table and connector map now name this concrete pin.

**Verification method and result.** Software-only, no jumper wire yet (see "what remains
unproven"):

- `gpioinfo gpiochip0` shows line 144 move from `unused input` to `"racer-heartbeat" output
  [used]` while the service runs, and back to `unused` when stopped -- proves the process
  actually claims and drives the line, not just that it starts without erroring.
- `/sys/kernel/debug/gpio`'s `gpio-492` entry (a kernel-internal readback path independent of
  the character-device API the program itself calls) shows `out hi` / `out lo` tracking the
  commanded value.
- Toggle rate: a throwaway diagnostic (not committed) polled `/sys/kernel/debug/gpio` in a
  tight loop for 2 seconds, timestamping every value transition on `gpio-492` with
  `clock_gettime(CLOCK_MONOTONIC)`. Result: **200 transitions in 1.9901 s, average edge
  interval 0.0100 s** -- 100 edges/second, exactly the 50 Hz square wave (edge every 10 ms)
  the default `--rate-hz 50` is supposed to produce, against the current (PROVISIONAL) 0.1 s
  `mux_watchdog_timeout_s`, i.e. about 10 edges per watchdog window.
- Error paths: an out-of-range `--line` and a second instance racing for an already-claimed
  line both exit 1 with a specific stderr message rather than hanging or exiting silently.
  `SIGTERM` (what `systemctl stop` sends) produces a clean shutdown log line and exit 0.

**What remains unproven.** That the signal measured above (the SoC's internal GPIO register)
actually reaches the physical pin 7 pad on this board's connector -- the pinmux mapping comes
from NVIDIA's own live device tree for this exact carrier, which is about as authoritative as
software gets, but no multimeter or oscilloscope has touched pin 7 itself. Actual voltage
levels and edge timing under load are also unmeasured. A loopback jumper from physical pin 7
to physical pin 29 (`gpiochip0` line 105, `PQ.05`, confirmed free the same way as pin 7) plus
`gpiomon` on the second line would close this gap with an interrupt-timed count on a
genuinely separate, physically-wired pin -- that jumper has not been placed yet, since it
needs a human at the bench. The heartbeat also has not been connected to the mux board at
all: that board's JETSON connector doesn't exist yet (`firmware/safety_mux/README.md`'s own
status), so nothing here demonstrates the mux MCU actually sees or reacts to this signal.
Roadmap 1.3's real kill test (Jetson frozen for real, cut proven, human present, wheels off
the ground) is still pending, unaffected by this entry.
## 2026-09-12 -- safety mux firmware: GPIO IRQ collision fixed, provisional PWM params filled in

Two changes to `firmware/safety_mux/`, both aimed at the same thing: making tonight's first
flash of the soldered kill-switch board actually exercise the mux instead of sitting in the
fault blink. Neither has been run on an RP2040 yet. See
`docs/notes/safety-mux-first-build.md` for the detail.

**1. The two GPIO interrupt handlers were evicting each other (a real hardware bug).**
`pico/pwm_capture.c` and `pico/heartbeat_input.c` each installed their own handler, and the
Pico SDK keeps exactly ONE GPIO callback per core. `main()` inits capture first and heartbeat
second, so the heartbeat handler silently won and all three PWM capture channels would have
read -1.0 forever: the mux would have cut permanently. Fail-safe, but inert. Fixed by adding
`pico/gpio_irq_dispatch.{h,c}`, the one owner of that shared callback; both modules now
register per-GPIO handlers through it and it routes each edge by GPIO number. Public headers,
`main.c`, and everything in `logic/` are unchanged. The header carries a long comment
explaining the single-callback constraint so this does not get reintroduced.

**2. The nine safety-mux fields in `config/vehicle_params.yaml` were filled in as
PROVISIONAL.** They were all `null`, so the firmware refused to arm (correctly). They now
carry the standard hobby-RC convention: 1000 / 1500 / 2000 us for both the steering and the
throttle channels, `mux_watchdog_timeout_s` 0.1 (about five missed frames of a 50 Hz
heartbeat), `mux_kill_switch_threshold_us` 1500 (the midpoint of that range). `schema_version`
0.2.0 -> 0.2.1.

**None of those nine numbers is measured.** They are convention and conservative guesses,
written down so the firmware can arm on a bench with a scope on it. They must be replaced by
real bench measurement -- `docs/notes/hardware-arrival-checklist.md` section 3, roadmap task
1.3 -- **before the car drives on the floor**. The throttle neutral is the one that bites: if
this ESC's real zero-throttle point is not 1500 us, the mux commands a creep in the CUT
state, which is the opposite of what the cut state is for. Wheels stay off the ground until
that is measured. The values are baked into the binary at compile time, so measuring means
rebuild and reflash, not an edit on the car.

The refuse-to-arm guard itself was not touched: a `null` in any of those fields still halts
the firmware. We supplied values, we did not disable the check.

## 2026-09-12 -- kill-switch board: the Pico was drawn mirrored, corrected

**Change.** The first board drawing had the Pico mirrored: the GPIO row was drawn on the
wrong side of the chip. Viewed from the top with the USB socket facing the board's left edge,
pins 1 to 20 run along the lower row (pin 1 bottom-left) and pins 40 down to 21 along the
upper row. The drawing had those two rows swapped, so every GPIO in the wiring plan was on
the power row and VSYS, 3V3 OUT and the pin-38 ground were on the GPIO row.

**How it was caught.** Cameron spotted it while reading the layout, before any soldering.
Nothing physical was affected, because nothing physical exists yet.

**What changed.** Only the row numbers. No hole column moved: VSYS is still column 6, 3V3 OUT
still column 9, GP2 to GP8 still columns 8 to 15. The plan now reads VSYS (6,12), 3V3 OUT
(9,12), GND pin 38 (7,12) on the upper row, and GP2 (8,19), GP3 (9,19), GP4 (10,19), GP5
(11,19), GND pin 8 (12,19), GP6 (13,19), GP7 (14,19), GP8 (15,19) on the lower row. The
continuity check before first power-up now probes the row-19 holes.

**Consequence for the wiring.** The three level-shifted signals and the heartbeat now have to
reach the far side of the Pico. They are routed on the underside and pass beneath the Pico's
body. That is safe because components sit on the top face and every wire is soldered on the
copper face, so there is nothing but bare board under the Pico. Noted on the build page.

**Flip-mirror warning added.** Both figures are drawn from the top, and the columns mirror
left-to-right the moment the board is turned over to solder: column 1 ends up on the right.
Column 1 and row 1 get marked with a paint pen on both faces before any soldering, and every
count starts from the marked corner.

**Status.** Planned layout only. Still nothing soldered, nothing powered, nothing measured.
The orientation above is from the Pico datasheet pinout, not from a board in hand.

## 2026-09-12 -- kill-switch board: one 4-pin Jetson header, and the command path

**Change.** The three Jetson inputs on the layer-1 mux perfboard are consolidated into a
single 4-pin male header: STEER SIG, THROTTLE SIG, HEARTBEAT, GND, pin 1 at the left and
marked with a paint pen. It replaces the earlier plan of three separate connectors (a 3-pin
steering, a 3-pin throttle, a 2-pin heartbeat). The KILL input and the SERVO output stay
3-pin, and the VESC output stays 3-pin with the +5V position left empty. See
`firmware/safety_mux/README.md`'s board connector map for the full layout.

**Why.** A voltage is a difference measured against a ground, so all three Jetson signals
have to be referenced to the same ground the Jetson sent them from. That is one wire, not
three. At 50 Hz, with microamp currents and microsecond edges, one shared return carries all
three signals without any crosstalk worth measuring, so separate returns per signal buy
nothing and cost two more connectors to get backwards. The receiver and servo headers stay
3-pin for a different reason: their middle pin is power, not reference. The board feeds 5 V
to the receiver and the servo. The Jetson powers itself, so its header has no 5 V pin at all,
which also removes any chance of back-feeding the Jetson from the board's rail.

**Cost of the change.** One connector that can be plugged in reversed. Reversed, it puts the
heartbeat where steering belongs, and no firmware check can see that. Mitigation is physical:
pin 1 marked on the board, a matching mark on the plug, and a keyed cable.

**Failure mode, claimed but not yet proven.** All three Jetson inputs are pull-down on the
Pico (`pico/pwm_capture.c` and `pico/heartbeat_input.c` both call `gpio_pull_down`), so an
unplugged or broken cable reads as a steady low: no pulses and no heartbeat toggle, which the
watchdog and the PWM validity check both treat as a cut. That is the intended behavior and it
is written down here as a claim, not a result. The bench test that settles it ("unplug the
Jetson cable mid-run, confirm both outputs go neutral within the watchdog timeout and the
cutoff opens") is now in `planning-docs/05-safety-mux-and-kill-test.md` and in
`docs/notes/hardware-arrival-checklist.md`. Until it has been observed on a scope, this
paragraph is a prediction.

**Command path decision.** The Jetson commands the motor by PWM, through the mux board, into
the VESC's PPM input. The VESC's USB link is for telemetry and configuration only, and
nothing in the drive path goes through it. The reason is that the mux only has authority over
what passes through it: a USB current-control path would run around the mux entirely and put
the motor under the Jetson's direct control, which is exactly what layer 1 exists to prevent.
This costs some fidelity compared with current control, and it is accepted for now. USB
current control can be reconsidered only after the GPIO 8 power-cutoff circuit physically
exists and has been kill-tested, because at that point the mux can cut motor power outright
rather than only cutting the command. That circuit does not exist yet.

**Status.** Planned layout only. Nothing soldered, nothing powered, nothing measured.
Running record of the physical build: decisions, deliveries, measurements, and mistakes, in
the order they happened. Per `claude-docs/10-conventions.md`, writeup is continuous. Newest
entries at the bottom.

## 2026-09-10

- Jetson Orin Nano Super Dev Kit arrived (Arrow, the last unit). JetPack 6.2.1 SD-card image
  (`jp62-r1-orin-nano-sd-card-image.zip`, Jetson Linux 36.4.4) downloaded; balenaEtcher
  downloaded. Flash pending. Firmware-version gate noted: a 2026 unit should already carry
  36.x UEFI; if it does not boot the JP6 card, the JetPack 5.1.3 bridge path applies.
- Both Picos confirmed to have male headers pre-soldered, so the kill-switch board becomes a
  socketed perfboard (female headers for the Pico and level shifter, male headers for the
  off-board plugs, screw terminals for 5 V in). No breadboard needed.
- Miuzei 151-piece perfboard + header kit ordered (amazon.ca B0H11M27WN).

## 2026-09-11

- Truck (Slash 4x4 HD VX3, green), Zeee 3S 5200 packs, and the rest of the Amazon cart
  arrived. Motor (Xerun 3652SD G3 4500KV) and the VESC still in transit.
- No stock battery and no charger on hand. **The SkyRC S65 was never actually ordered**: the
  order plan listed it on the xtremerc order but it was not on the receipt, and the shop's
  own email ("as long as you already have batteries and a charger") was the missed warning.
  Replacement ordered: 80 W / 6 A B6-class balance charger, amazon.ca B0G9MDKCW3, chosen
  because it includes an XT60-to-EC5 lead (plug-and-charge on the Zeee packs). Charge plan:
  LiPo, 3S, balance mode, 5.0 A (about 1C).
- **VESC minimum-voltage mismatch found.** Flipsky specs the FSESC 6.7 at 14-60 V, 4S
  minimum (their product page and their own 4.20-vs-6.7 comparison post). The traction pack
  is 3S: 12.6 V full, ~9.6 V empty, below spec across the whole range. The 2026-08
  compatibility audit checked current, sensors, and connectors but not minimum voltage.
  Options considered: bench-test the 6.7 at 3S (out-of-spec on the drive path, rejected once
  an in-spec part was available), two packs in series (6S would over-speed and overheat the
  2-3S motor, rejected), 4S packs (same motor problem, rejected). Decision: cancel the 6.7
  (delayed anyway) and order the **Flipsky FSESC 4.12, 50 A, 8-60 V, 3S-rated**, amazon.ca
  B0CSDYF7RF, delivery Sep 22-29. Flipsky-direct Mini FSESC 4.20 ($56 USD) was the faster
  alternative if express shipping had been chosen.
- FSESC 4.12 compatibility check against every interface: 3S voltage in spec; EC5 soldered
  onto its bare 12 AWG leads; 50 A continuous vs ~30-40 A motor draw with layer-2 limits at
  ~40 A; 60,000 ERPM cap = ~30,000 motor RPM = ~15 m/s at the wheels (double what the car
  needs); hall-sensor port for the Xerun's cable (repin still required); PPM input driven
  directly by the mux board's 3.3 V GPIO 7 pulses; its 5 V BEC pin left unconnected on the
  board; USB telemetry to the Jetson unchanged; ~60x40x20 mm, smaller than the stock ESC.
- Stock Traxxas ESC ruled out for the project: sensorless-only (no hall socket), no computer
  interface or telemetry, no programmable or exportable limits (layer 2 lives in the VESC
  config). It goes in the spares bag.
- Kill-switch board wiring plan written against `firmware/safety_mux/pico/main.c`: GPIO 2/3/4
  inputs (kill, steering, throttle) through the level shifter; GPIO 5 heartbeat direct;
  GPIO 6/7 servo/VESC outputs direct; GPIO 8 power cutoff reserved; Pico powered via VSYS
  from the UBEC's 5 V; shifter HV = 5 V, LV = Pico 3V3 OUT; single shared ground.
- Today's work order: mechanical check of the truck, photograph then remove stock ESC and
  receiver (servo and motor stay), then session 1 at the shop (sockets and headers only, no
  wiring until the layout photo is checked).
