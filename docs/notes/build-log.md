# Build log

Dated entries for physical build decisions: what changed on the car or on a board, why, and
what still has to be proven on the bench before the decision counts as correct. Design docs
say how things are meant to be; this file says when a choice was made and on what grounds.
Nothing here is a test result unless it says it was observed.

## 2026-09-21 late -- Jetson PWM frame shortened to 4 ms: 15.6 us actuator resolution (issue #66)

GitHub issue #66 recorded the measurement that started this: the Jetson commands a pulse and
the mux reads a different one. 1500 us commanded read 1484, 2000 read 2031, 1000 read 1016 --
every reading an exact multiple of 78.125 us. The cause is not the mux and not the cable. The
Tegra PWM controller expresses duty as an 8-bit fraction of the period (`pwm-tegra.c`,
`PWM_DUTY_WIDTH 8`), so at the 20 ms frame `pwm_output_node` was writing, the smallest
expressible step is 20000/256 = 78.125 us. That is about **13 distinct positions across
1000-2000 us**: 2.3 deg of steering per step, and a throttle that behaves like a switch,
because the first steps off neutral sit inside the VESC's 15 percent current deadband and the
sensorless motor does not catch until roughly 1700-1800 us (see the runbook's throttle
deadzone table, measured the same day).

**The decision: option 1 from the issue, at 4 ms rather than 5.** The frame period is now a
configured value, `actuation.steering_pwm_period_us` and `actuation.throttle_pwm_period_us`,
both 4000 us (250 Hz), which puts the grid at 15.625 us -- five times finer, about 64
positions across the same span.

**Why this is not a servo-tolerance question at all, which is what the issue assumed.** The
issue listed "the servo is the risk" for a shorter frame. It is not, because the servo never
sees this frame. `firmware/safety_mux` captures the INPUT pulse width on GP10/GP7 from edge
timestamps (`pico/pwm_capture.c`, `time_us_64`, 1 us) and **regenerates** its own outputs on
GP1/GP3 at a fixed 50 Hz / 20 ms frame (`pico/pwm_output.c`, `top = 39999` at 0.5 us per
count, set from `clk_sys`). The Jetson frame rate exists only on the Jetson-to-Pico link. The
analog Traxxas 2075 and the VESC PPM input keep getting exactly the 50 Hz they got before, so
the steering channel gets the short frame too: leaving it at 20 ms would keep 2.3 deg per step
in exchange for nothing.

**Firmware verdict: nothing to change, and nothing was changed.** Checked line by line before
touching anything:

- Capture measures high time from edges and assumes no frame at all.
  `pico/pwm_capture.c:72-93`: rising edge stores `time_us_64()`, falling edge subtracts. The
  only frame-shaped assumption is the plausibility band at lines 23-24 (200-5000 us), which
  exists to reject noise spikes, not to police a frame rate.
- The staleness window still behaves. `PWM_CAPTURE_MAX_AGE_US` is 60000 us
  (`pico/pwm_capture.c:39`) and it is a MAXIMUM AGE, not a frame multiple: at 4 ms a pulse
  arrives five times as often, so every reading is fresher than before and the window is
  further away, never closer. Its comment at lines 26-29 describes it as "three missed frames"
  at 50 Hz, which at 250 Hz is now fifteen -- less sensitive to a stalled link, not more, and
  changing it is a separate decision (it is already on the subject-to-change list in
  `docs/notes/firmware-review-2026-09-14.md`) rather than something this change should
  silently retune.
- Validity is unchanged. `logic/src/pwm_validity.c` and `logic/src/pwm_window.c` are pure
  range checks on a measured width, with no period term anywhere. The 62.5 us tolerance
  (`logic/include/safety_mux/pwm_window.h:56`) was derived from the worst case of rounding
  onto the emitter's 78.125 us step; at 15.625 us that worst case shrinks to 7.8 us, so the
  existing tolerance is now more conservative than it was, not less. The header's rationale
  text still describes the 20 ms emitter, which is now a historical note; it is a comment,
  not a behaviour, and it is not worth a reflash.
- The output stage is untouched. `pico/pwm_output.c` computes its own `top` and `clkdiv` from
  `clk_sys` and never reads an input frame.
- IRQ load is not a concern. Two captured Jetson channels at 250 Hz is 1000 edges per second,
  plus about 100 for the 50 Hz RC kill channel and about 100 for the heartbeat: roughly 1200
  interrupts per second on a 125 MHz RP2040, where the handler is a lookup, a subtraction and
  a range check. That is order 0.1 percent of the core, and the 200 Hz main loop
  (`pico/main.c:44`) is unchanged.
- One genuine edge case, and it fails closed. Two consecutive missed edge interrupts could
  merge frames into a measured width of `frame + high` = up to 5000 us, which at 20 ms was far
  outside the 200-5000 us plausibility band and at 4 ms sits right at its ceiling. If such a
  reading is ever believed by the capture, it still has to pass
  `pwm_window_accept_us()` against the configured 1000-2000 us range widened by 62.5 us, which
  rejects it, and the mux cuts. Worse readings cut; they do not pass.

So the `.uf2` files are unchanged and no reflash is needed. The firmware's compiled-in
vehicle_params values are also untouched by this bump -- the two new fields are consumed by
`racer_drivers`, not by the mux.

**Schema.** `schema_version` 0.3.0 -> 0.4.0 (two new required fields, so a minor bump per
`claude-docs/06-vehicle-params.md` rule 5). `pwm_output_node` no longer derives the carrier
period from `output_rate_hz`; that parameter is now only the duty-rewrite cadence, still
50 Hz, and the period comes from the generated binding per channel. `validate_config()`
refuses a frame period shorter than twice its channel's own `pwm_max_us`, which is the rule
that keeps a 2000 us pulse from filling its frame and leaving the mux's edge capture no low
gap. At 4000 us and a 2000 us maximum that is exactly the boundary, which is also why the
frame is 4 ms and not the 2.5 ms the schema floor allows.

**NOT YET VERIFIED ON HARDWARE.** The grid arithmetic is exact and the tests pin it, but the
new frame has not been in front of the Pico. The bench procedure is in
`docs/notes/first-boot-runbook.md`, "Bench verification of the 4 ms frame": commanded 1500 /
1600 / 1700 us should read within one 15.6 us step, and the mux `OUT` fields must still show a
50 Hz regenerated pulse. Do not drive on the floor before that has been read off the
diagnostic.

## 2026-09-21 evening -- steering endpoints and sign measured, mapping sign corrected

The steering channel's two remaining unmeasured provisional numbers -- the PWM endpoints and
which end is LEFT -- were measured on the car, wheels off the ground, mux armed, commanded
through the Jetson PWM (`docs/notes/bench-session-2026-09-20.md`'s steering endpoint
follow-up).

**Endpoints.** Left mechanical stop: 1093.75 us commanded was clean; 1015.6 us made the servo
hum against the stop. Right mechanical stop: 1875 us commanded was clean; 1953 us hummed.
Neutral (1500 us commanded) puts the wheels straight ahead. Committed to
`config/vehicle_params.yaml` as `steering.pwm_min_us: 1094`, `pwm_max_us: 1875`,
`pwm_neutral_us: 1500` (rounded to the nearest whole microsecond; the Jetson PWM's 78.125 us
grid means these are the nearest achievable steps, not continuous readings). This also
resolves the `OUT_OF_RANGE` finding in `docs/notes/first-boot-runbook.md`'s "Reading the mux
numbers": both new endpoints sit well inside the mux's 1000-2000 us validity window, where the
old provisional 1000/2000 us pushed a full-left command to a mux-reported 2031 us.

**Sign.** A SHORTER pulse turns the wheels LEFT. The mapping had been assuming the opposite --
verified backwards at the mux earlier this session (steering_angle 0.2 rad measured 1719 us,
above neutral). This is the measurement `docs/notes/first-boot-runbook.md` step 12 and
`ros_ws/src/racer_bringup/launch/car_teleop.launch.py`'s `steering_left_is_pwm_max` argument
had both been waiting on since it was flagged an unmeasured guess
(`docs/notes/command-path-review-2026-09-14.md`).

**Where the sign now lives.** Previously `steering_left_is_pwm_max` was a declared ROS
parameter on `pwm_output_node` with an unmeasured default (`true`), passed down from a launch
argument of the same name. Now that it is measured, `CLAUDE.md` invariant 2 applies -- a sign
convention is a physical constant, not a code or launch default -- so it moved into
`config/vehicle_params.yaml` as a new required field, `steering.pwm_left_bound`
(`"pwm_min_us"` or `"pwm_max_us"`, schema_version bumped 0.2.2 -> 0.3.0). `pwm_output_node`
reads it from the generated binding and refuses to start if it is anything else; the launch
argument and the node parameter are both gone. `pwm_mapping.cpp`'s `left_is_pwm_max` bool is
unchanged as an internal detail -- it is now derived from `pwm_left_bound` inside the node
instead of being declared there.

**Tests.** `ros_ws/src/racer_drivers/test/test_pwm_mapping.cpp` gained a
`committed_steering_config()` golden fixture and a `CommittedCalibration*` test group pinning
the real committed numbers (full left 0.4189 rad -> 1094 us, full right -0.4189 rad -> 1875
us, zero -> 1500 us, clamping at and past both bounds, and an exhaustive sweep asserting the
mapped pulse never leaves `[1094, 1875]`). The L3 launch tests
(`test_pwm_output_node_launch.py`, `test_pwm_output_node_clock_launch.py`) no longer pass
`steering_left_is_pwm_max` and instead derive the expected LEFT/RIGHT pulse from
`config/vehicle_params.yaml`'s `steering.pwm_left_bound` at test time, so they stay correct if
the sign or the fixture values change again.

**Not yet done, on purpose (later task):** `steering.pwm_to_angle_table` is still `null` -- the
angle-to-pulse map between these two measured endpoints is still ASSUMED LINEAR, not measured
as a table.

## 2026-09-21 late -- rosbag recording and rail voltage wired into car_teleop.launch.py

Closes the logging gap the entry below recorded and left open (GitHub issue #64, roadmap 1.6).
`CLAUDE.md` invariant 5 says every run is logged, rosbag plus rail voltage, and that a code
path which drives the car without logging is a bug. Until today nothing in `racer_bringup`
started a recorder, and nothing published rail voltage at all, so the invariant was satisfied
only by an operator remembering a second shell. It is now the launch file's job.

**What gets recorded.** `car_teleop.launch.py` starts `ros2 bag record` by default, into
`<bag_dir>/<local ISO timestamp>_car_teleop`, `bag_dir` defaulting to `/workspace/data/bags`
(= `~/car/data/bags` on the Jetson, through the workspace mount the start command already
has). Topic selection is a regex rather than a list, so `/scan` is picked up the day a LiDAR
appears and ignored until then: `/drive_raw`, `/drive`, `/safety/events`, `/teleop/cmd_vel`,
`/telemetry/.*`, `/scan`, `/rosout`, `/parameter_events`.

**mcap on the car, sqlite3 in ros-dev, and the choice is printed.** `docker/car/Dockerfile`
gains `ros-humble-rosbag2-storage-mcap` (base digest untouched); `ros-base` ships rosbag2 but
in Humble its only storage plugin is sqlite3. mcap is worth the one package because a session
can end with a kill switch or a yanked battery, and a single append-only self-describing file
survives that better than a sqlite3 bag. `bag_storage:=auto` resolves by asking the ament
index whether the plugin exists, so the same launch file records mcap on the car and sqlite3
in the container the L3 test runs in, and says which at startup.

**Rail voltage, from the INA3221 that is already on the board.** The INA226 in
`claude-docs/11-hardware.md` is not fitted and is not on the critical path for G1. The Orin
Nano carrier has its own INA3221 exposed through the hwmon sysfs ABI, so
`racer_drivers/rail_voltage_node` (Python, new) reads it at 5 Hz and publishes VDD_IN as
`/telemetry/rail_voltage_v` / `/telemetry/rail_current_a` plus every channel by label. Volts
and amps, converted from the ABI's millivolts and milliamps at the driver boundary, which is
where invariant 4 says a non-SI wire format gets converted. That factor of 1000 is a kernel
interface constant, not a vehicle parameter, and deliberately does not go in
`vehicle_params.yaml`. When the INA226 lands it publishes onto the same topics.

**A recorder that dies takes the launch with it, and that was a deliberate choice.** Three
options were on the table. Gating `/drive` on the recorder was rejected outright: it would put
a logging dependency inside safety layer 3, weakening the layer to strengthen something that
is not a safety layer at all. Publishing a `SafetyEvent` on `/safety/events` was rejected
because that topic has one publisher, a closed enum of GATES, and
`claude-docs/09-evaluation.md` counts its `PHASE_ENGAGE` records as interventions -- a
recorder crash is not an intervention and would corrupt a reported metric. What landed is an
error-level log plus a launch `Shutdown`. Measured in the container with a deliberately bogus
storage plugin: recorder exits, `FATAL [CLAUDE.md invariant 5]` at error level, whole launch
down in about a second, `pwm_output_node` taking its ordinary neutral-and-disable shutdown
path on the way out. Layers 1 and 2 are untouched and still physically downstream.

**Not verified on the car.** Everything above ran in the `ros-dev` container on the Mac. The
INA3221 sysfs paths were read from the Jetson earlier and the node is tested against a fixture
tree of ordinary files with the same shape, so the parsing is proven but the real
`/sys/bus/i2c/drivers/ina3221/...` read, the actual VDD_IN numbers, the mcap plugin on arm64,
and writing a bag to the Jetson's disk are all first-drive checks. The runbook's "Every run is
recorded" section says what to look for.

## 2026-09-21 evening -- first real ROS command path on the Jetson, proven at the mux

The classical command path ran on the actual vehicle computer for the first time:
`/drive_raw -> safety_node -> /drive -> pwm_output_node -> 50 Hz PWM -> safety-mux Pico`.
The car was physically unable to move throughout: wall adapter only, LiPo out, VESC and
steering servo unpowered, servo lead unplugged from the mux board. The Pico was on Jetson USB
running DIAG_BUILD, so what the mux SAW is measured, not inferred.

Four bugs were found by doing this, all of them in code or docs that had never been executed.

**1. The car image could not run its own launch file.** `car_teleop.launch.py` declares `viz`
with `default_value="true"` and starts `foxglove_bridge` under it; `docker/car/Dockerfile`
listed `ros-humble-foxglove-bridge` among the "dev visualization" packages it deliberately
excluded. `ros2 launch racer_bringup car_teleop.launch.py` with no arguments therefore failed
with `package 'foxglove_bridge' not found`. Resolved in favour of installing the bridge rather
than flipping the default: the bridge is the owner's DRIVING interface on this car (Foxglove
Teleop panel -> `/teleop/cmd_vel`), which makes it operator interface, not developer tooling.
The launch file's L3 test never caught it because it runs in `ros-dev` (which has the bridge)
and passes `viz:=false`, and CI never builds this image at all.

**2. The image build hung forever on an interactive tzdata prompt.** The ROS 2 Humble apt
layer pulls `tzdata` transitively; `l4t-jetpack:r36.4.0` ships no `/etc/timezone`, so debconf
stopped at `Geographic area:` with no tty to answer it. No other Dockerfile in `docker/*/` hits
this because they build on `ros:humble-ros-base`, which configured tzdata in its own build.
Fixed with `ARG DEBIAN_FRONTEND=noninteractive` (ARG not ENV, so it does not leak into the
produced image and silently default a human's prompts later).

**3. `colcon build` died in the vehicle_params codegen step.** `docker/car` sets
`ENV PYTHONPATH=/car-runtime/.venv/lib/python3.10/site-packages` for `racer_policy`'s runtime
deps. `tools/` is a uv project on CPython 3.14, and the inherited PYTHONPATH is prepended
ahead of its own site-packages, so `gen_params.py` imported a cp310 native extension under
3.14: `ModuleNotFoundError: No module named 'rpds.rpds'`. Fixed in all three CMakeLists that
invoke it (`racer_control`, `racer_drivers`, `racer_safety`) with
`cmake -E env --unset=PYTHONPATH`, not with an incantation in the car run command -- `ros-dev`
is one `export PYTHONPATH=...` away from the identical failure, and
`docs/notes/milestone-5-browser-teleop.md`'s own demo procedure does exactly that before
`colcon build`.

**4. `pwm_output_node` refused to start, twice, for two different real reasons.**

- `cannot open /sys/class/pwm/pwmchip0/pwm0/enable for writing: Permission denied`. The
  kernel creates `pwmN/` synchronously on the export write, but udev applies the group and
  mode afterwards, asynchronously. `SysfsPwmChannel::start()`'s bounded wait only waited for
  the DIRECTORY, which is already there on the first check -- so it fell straight through and
  the next write raced udev and lost. The loop's own comment already said "udev may still be
  adjusting permissions"; it just waited for the wrong thing. Now waits for the attributes to
  be present AND writable, same 500 ms budget, same refuse-rather-than-retry-forever.
- `failed writing '0' to /sys/class/pwm/pwmchip2/pwm0/enable: Invalid argument`. The Tegra PWM
  driver rejects an `enable` write while `period` is 0, and **pwmchip2 exports with period=0
  while pwmchip0 exports with its 20 ms period already set** -- which is why only the throttle
  channel failed, and why nothing caught this when the pins were driven by hand. `start()`'s
  pre-emptive `enable 0` is now skipped when the period reads 0; a channel with no period is
  emitting nothing, so there is no stale pulse for that disable to protect against. Whenever
  the period is non-zero the disable happens exactly as before.

**Container device access: `--privileged` is not needed and is not used.** The runbook's step
8 proposed a new `racer-pwm` group and a custom udev rule, both marked UNVERIFIED. Neither is
necessary: the Jetson already ships `/lib/udev/rules.d/60-jetson-gpio-common.rules`, which
chgrps `pwmchipN/{export,unexport}` and each exported channel's `period`/`duty_cycle`/`enable`
to the **`gpio`** group, and `racer` is already in it. `--group-add 999` plus read-write
bind-mounts of the two real device-tree paths
(`/sys/devices/platform/bus@0/3280000.pwm` and `.../32c0000.pwm`) is the whole story. The
container also runs as `--user 1000:1000`, i.e. unprivileged AND non-root. The `racer-pwm`
group and rule were created during this session, found redundant, and removed again; the host
is back to stock udev.

**What the mux measured.** Neutral, both channels, nothing commanding:

```
STEER gp10=1484us FRESH(10ms, in 1000-2000) | THR gp7=1485us FRESH(18ms, in 1000-2000)
HB gp5 age=4ms (timeout 100ms) OK | DECISION=CUT reason=1:RC_SIGNAL_INVALID
```

(`CUT reason 1` is the transmitter being off, which it was all session. Irrelevant here: the
`STEER`/`THR` fields are the proof that Jetson pulses reach the Pico.)

Commanding `/drive_raw` directly, at 50 Hz, through the real gate. Every `duty_cycle` matched
`pwm_mapping`'s prediction exactly:

| Command | Predicted | `duty_cycle` (ns) | Mux reports |
|---|---|---|---|
| `steering_angle: 0.2, speed: 0.0` | 1738.72 / 1500 us | 1738720 / 1500000 | `STEER 1719us FRESH`, `THR 1484us FRESH` |
| `steering_angle: 0.0, speed: 0.5` | 1500 / 1550 us | 1500000 / 1550000 | `STEER 1484us FRESH`, `THR 1563us FRESH` |
| `steering_angle: 0.0, speed: 0.0` | 1500 / 1500 us | 1500000 / 1500000 | `STEER 1484us`, `THR 1485us`, both FRESH |

Positive steering angle raises the pulse above neutral, i.e. toward `pwm_max_us`, which is
what `steering_left_is_pwm_max:=true` claims. That confirms the SIGN convention reaches the
pin; it does not confirm which way the WHEELS turn, because the servo was unpowered and
unplugged. Step 12's polarity calibration is still open.

**The mux measures on a 15.625 us grid, and full lock falls off the end of it.** Sweeping
steering with the sysfs value read back each time, every mux reading is an exact multiple of
15.625 us: 1016, 1485, 1640, 1718, 1875, 2031. A full-left command is a legitimate 2000 us
pulse, which the mux rounds to 2031 us -- outside its own inclusive 1000-2000 us validity
window -- and flags `STEER ... OUT_OF_RANGE`. Armed, that is a steering cut (reason 3) at full
lock. Full right (1016 us) is inside. **Open item, not fixed here:** the steering endpoints are
still the provisional 1000/2000 us and have to be measured anyway (the servo buzzed against
its stop at 1200 us on 2026-09-20); narrowing them away from the channel ends removes this.
Nothing was changed in the mux firmware or in `vehicle_params` for it.

**Shutdown behaviour differs by signal, and the docs now say so.** Ctrl-C / SIGINT is the
clean path: `pwm_output_node` writes neutral then DISABLES both channels, pulses stop, and the
mux reports `STEER`/`THR ... STALE (stuck or stopped)`. `docker stop` (SIGTERM) is not: both
channels stay ENABLED at neutral 1500 us and the mux keeps reading `1484us FRESH` indefinitely.
This was observed even with `exec ros2 launch` as the container's PID 1. Neither leaves a
driving pulse behind, but only Ctrl-C actually stops the pulse train.

**Browser teleop works end to end.** `foxglove_bridge` listens on `0.0.0.0:8765`; the
websocket handshake returns `101 Switching Protocols` with `sec-websocket-protocol:
foxglove.sdk.v1` both from the Jetson and from the Mac over the LAN. With
`browser_teleop:=true`, a `Twist` on `/teleop/cmd_vel` (`linear.x=0.5`, `angular.z=1.5`) drove
both channels through the full gated path: throttle 1550000 ns, steering saturated at 2000000
ns (that yaw rate at that speed asks for 0.78 rad, well past the 0.4189 limit). **A human
clicking the Foxglove Teleop panel in a real browser is still not done** -- same gap
`docs/notes/milestone-5-browser-teleop.md` already records for the sim.

**No rosbag.** `car_teleop.launch.py` starts no `ros2 bag record`, and no launch file in
`racer_bringup` does. `CLAUDE.md` invariant 5 makes that a bug rather than a gap. Not fixed
here (it is out of scope for a bring-up whose whole point was that nothing can move, and
`docs/notes/car-runtime-plan.md` already holds the design); recorded as an open item and a
manual workaround is in the runbook.

**Numbers.** Image `car:local`, `INSTALL_TORCH=skip`, base digest unchanged: 10.2 GB on disk.
The first build attempt spent about 25 minutes pulling and extracting the 3.4 GB
`l4t-jetpack:r36.4.0` base; the final successful build, against a warm base cache, took about
5 minutes (the ROS apt layer is 165 s of it). `colcon build` of all 6 workspace packages: 68 s.

**Still not done, and none of it was attempted:** no servo has moved under ROS command, no ESC
has been armed by this stack, the mux has not been kill-tested against a genuinely frozen
Jetson (roadmap 1.3), steering polarity and endpoints are uncalibrated, and the throttle map
is still the provisional open-loop 5.0 m/s full scale with an unmodelled ~1700 us start
deadzone.

## 2026-09-21 -- safety mux plausibility window widened for capture quantisation, pass-through clamped

GitHub issue #63, found during the ROS bring-up the same day. With the mux armed, the Jetson
commanded exactly 2000 us on steering (its configured `steering.pwm_max_us`, full left); the
diagnostic build reported `gp10=2031us OUT_OF_RANGE(allowed 1000-2000)` and the mux CUT on a
legal command. Full right, commanded 1000 us, measured 1016 us and passed.

- **Diagnosis.** Neither measurement is a fault. 2031.25 = 130 x 15.625 and 1015.625 = 65 x
  15.625: both are exact points on a 15.625 us pulse-width grid, and the emitting
  peripheral's own duty granularity (a 20 ms frame split 256 ways = 78.125 us) is five of
  those steps. A command that is not itself on the grid cannot be emitted on it, and at the
  edges of an inclusive window with zero tolerance the rounding points outward.
- **The grid is not the mux's timebase, so measuring finer was not available.**
  `pico/pwm_capture.c` already times edges with `time_us_64()`, a 1 us hardware timer. There
  is no coarser clock in the capture path to refine: the pulse on the wire really is 2031 us
  long. That ruled out the "measure at higher resolution" option in the issue and left
  widening the window.
- **Change.** New host-tested logic unit `logic/pwm_window.c`. The two Jetson command
  channels are accepted within their configured range widened by **62.5 us (4 x 15.625 us)
  on each side**, and an accepted pulse is **clamped back into the unwidened range** before
  it is forwarded. Against the provisional 1000-2000 us steering range: accepted
  937.5-2062.5 us, forwarded 1000-2000 us, so a 2031 us measurement drives the servo at
  2000 us and never at 2031 us.
- **Why 4 steps.** 2 steps (31.25 us) is the minimum that clears the observed error;
  2.5 steps (39.06 us) is the worst case for nearest-point rounding onto the 78.125 us
  emitter step; 4 steps is twice the observed error and past that worst case, while still
  rejecting 900 us (37.5 us below the widened floor) and 2450 us (387.5 us above the widened
  ceiling). 6 steps would have let 900 us in, which is where a tolerance stops being a
  tolerance.
- **Unchanged, deliberately.** The RC kill-switch channel is neither widened nor clamped: its
  1400/1600 us hysteresis band around the 1500 us threshold reads exactly as before. Stale,
  no-edge and genuinely out-of-range inputs still cut, with the same reasons and the same
  priority order. A cut is still both channels to neutral plus the cutoff GPIO.
  `pwm_is_valid_us()` and its suite are untouched, and `pico/pwm_capture.c`'s 200-5000 us
  noise band is untouched.
- **Tests.** Host suite goes from 320 to 511 assertions, all passing, nothing removed or
  loosened. New `tests/test_pwm_window.c` covers the two hardware numbers (2031 us accepted
  and clamped to 2000, 984/1016 us at the low edge), both window edges plus and minus one
  grid step, the tolerance boundary itself and one step past it, the fail-closed paths
  (NaN/Inf value, NaN/Inf/inverted bounds, NaN/Inf/negative tolerance all degrading to a zero
  tolerance rather than an open one), and the garbage the issue names. `test_mux_decision.c`
  gains the same cases end to end through `mux_decide()`, plus explicit cases pinning the
  kill-switch thresholds as unchanged.
- **Diagnostic build.** Each channel line now names the slack its window allows and prints
  `CLAMPED-><value>` when a pulse is accepted only because of it, and the attach banner
  states the rule. `docs/notes/mux-diagnostic-build.md` updated with the new sample lines.
- **Artifacts** (built in the documented container: debian:bookworm arm64, arm-none-eabi-gcc
  12.2, pico-sdk 2.1.0 via FetchContent; both clean, no warnings):

  | Build | Bytes | sha256 |
  |---|---|---|
  | shipping `safety_mux.uf2` | 79872 | `e7af70c1f837e7611a5b6ca955a76bdf4776eeeba7b6b3ef4a4b39eccc498663` |
  | diagnostic `safety_mux_diag.uf2` | 94208 | `86f5b6da76c7f6f75f75870345c0e9de2a43d978116f6e19e20db213160190f8` |

  The shipping binary is 512 bytes larger than the 2026-09-20 one (79360 bytes,
  `ccddd745...`), as expected: this is a change to `logic/`, so both builds change.
- **Not verified on hardware.** Nothing here has been flashed. What a bench session still has
  to confirm: that a commanded 2000 us now passes with the mux armed and that the servo sits
  at its 2000 us end stop rather than past it; that the measured grid on this board really is
  15.625 us (a scope reading, not two data points); and the interim mitigation the issue also
  asks for, which is measuring the real servo endpoints and setting the steering PWM min/max
  in `vehicle_params` inside the window (the servo buzzed at 1200 us, which this change does
  not address and is not meant to).

## 2026-09-20 -- Jetson 40-pin PWM pins enabled and confirmed with a multimeter

Ran the `docs/notes/first-boot-runbook.md` step 4/5 procedure for real, on the actual Jetson
Orin Nano Super Dev Kit, JetPack 6.2 / L4T R36.4.4. Everything below is measured, not
written-ahead.

- `sudo python3 /opt/nvidia/jetson-io/config-by-function.py -l all` lists exactly three
  PWM-capable functions on the 40-pin header: `pwm1` on physical pin 15, `pwm5` on physical
  pin 33, `pwm7` on physical pin 32. This board uses pins 15 and 33; pin 32 is unused.
- Out of the box, **none** of these were enabled. Both pins measured 0.0 V even with the PWM
  controllers exported and enabled in sysfs -- the trap here is that the kernel runs a PWM
  controller whose output the pinmux has not routed to a pad, and it does not tell you.
- Fix applied and now persisted: `sudo python3 /opt/nvidia/jetson-io/config-by-function.py -o
  dt 1="pwm1 pwm5"`, then reboot. This writes `/boot/jetson-io-hdr40-user-custom.dtbo` and
  adds it to `/boot/extlinux/extlinux.conf`, so it survives reboots and does not need
  redoing.
- After the reboot, `-l enabled` reports `pwm1` (pin 15) and `pwm5` (pin 33) enabled. Owner
  confirmed with a multimeter: **~1.64 V DC on both pin 15 (referenced to pin 14) and pin 33
  (referenced to pin 34)**, with each channel driven at 50 Hz / 50 percent duty by hand
  through sysfs. That is the expected average of a 3.3 V square wave at 50 percent duty
  (0.5 x 3.3 V = 1.65 V), so the pads are genuinely driven, not just toggling in software.
- sysfs mapping confirmed on the device: `pwmchip0` = `3280000.pwm` = header pin 15;
  `pwmchip2` = `32c0000.pwm` = header pin 33 (`pwmchip1` = `32a0000.pwm`, `pwmchip3` =
  `32e0000.pwm`, `pwmchip4` = `39c0000.tachometer`, all present but unused). Each chip's
  `npwm` is 1, so the channel index within each chip is 0.
- **Assignment decision, applied consistently everywhere:** pin 15 / `pwmchip0` = steering,
  pin 33 / `pwmchip2` = throttle. `pwm_output_node`'s declared parameter defaults
  (`steering_pwmchip=0`/`steering_pwm_channel=0`, `throttle_pwmchip=2`/
  `throttle_pwm_channel=0`) and `car_teleop.launch.py`'s launch-argument defaults now match
  this measurement (`ros_ws/src/racer_drivers/README.md`,
  `ros_ws/src/racer_bringup/launch/car_teleop.launch.py`).
- **What is verified and what is not.** Verified: the two pads carry a PWM signal at the
  right average voltage when driven by hand at 50 Hz / 50 percent duty. NOT verified:
  pulse-width accuracy under load, `pwm_output_node`'s own behaviour driving these channels,
  or anything downstream of the pins (servo, mux board, VESC PPM input). Those are still
  first-boot-runbook.md steps 10-14.

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
## 2026-09-14 -- command-path review: "brake" is a coast, reverse is off, steering polarity is the first calibration

Software and docs only; the car was not powered and nothing here has been on hardware. Full
write-up, findings table and the subject-to-change list:
`docs/notes/command-path-review-2026-09-14.md`. Three decisions belong in this log because
they are decisions about the car, not about code.

**1. Layer 3's "brake" commands zero current, which on a PPM VESC is a COAST.** `safety_node`
has exactly one lever, the `/drive` `speed` field, and every gate that fires writes 0.0 into
it. `pwm_output_node` maps speed 0 to `actuation.throttle_pwm_neutral_us`, and a neutral PPM
pulse is zero current. So "watchdog fired, braking" produces a car that keeps rolling and
slows by drag. Nothing in the code said so, and a reader of `05-safety.md` would reasonably
have assumed otherwise. This is now written at the top of `gate_logic.hpp` and
`safety_node.cpp`, and `GateResult::brake` is renamed `GateResult::zero_throttle`.
`EventSeverity::kBrake` and `racer_msgs`' `SEVERITY_BRAKE` keep their names deliberately --
they are a published interface and the evaluation metric in `09-evaluation.md`, and renaming
them would break bag compatibility to fix a comment.

Real deceleration has to come from layer 2, so the runbook gained a numbered VESC Tool step
(now step 13, before the ESC is ever fed a pulse): **set the PPM app's Control Type to
`Current No Reverse With Brake` for first boot**, because it makes a below-neutral pulse
proportional braking rather than reverse drive current, which is the direction a sign error
should fail in on a car nobody has driven; because reverse is not wanted for the first drives
anyway; and because neutral stays zero current either way. **No VESC numbers are prescribed**
-- deadband, current limits and ramping are unmeasured and stay unmentioned. The step says to
export the configuration and commit it, because `12-testing.md`'s L6 "VESC config diff" bench
check needs a committed baseline to diff against, and there is none yet.

**2. Reverse is off by default on both teleop sources.** Both clamped the speed floor at
`limits.min_velocity_mps` (-5.0, an `f1tenth_gym` value), so the throttle-down key or a
backwards Twist would have put a below-neutral pulse on an ESC whose PPM control type has
never been set -- reverse current in one mode, braking in another, nobody knows which. New
`allow_reverse` declared parameter on `keyboard_teleop_node` and `twist_teleop_adapter_node`,
default `false`, clamping the floor at 0.0 m/s; launch argument on `car_teleop.launch.py`.
Both nodes, not just the keyboard: `browser_teleop` is the one the car launch file can
actually start. `limits.min_velocity_mps` itself was NOT changed (it is a vehicle parameter,
not a teleop policy), `safety_node` still accepts negative speed (it is a gate, not a command
source), and `pwm_mapping` still maps negative below neutral. The runbook says to leave
reverse off until the VESC step is done and recorded.

**3. Steering polarity is now the FIRST bench calibration, and it happens with the ESC in the
box.** It used to be runbook step 12, after step 11.4 powered the drive battery. It needs
nothing but the servo, and there is no reason for the motor to be live while the polarity is
still unknown, so the ESC power-up is now its own later step (14). The whole left-positive
chain was traced hop by hop -- key to angle, Twist to angle, gate pass-through, angle to pulse
-- and every software hop now has a test that names the convention. The single remaining
unknown in it is `steering_left_is_pwm_max`, whose `true` default is a **guess**: no project
doc defines which pulse end is full left and nobody has put a scope on this servo. That is
what step 12 settles, and its answer goes in this log and into the launch file's default.

**Also fixed, and worth knowing before the first boot** (details in the review note):

- `pwm_output_node` measured `/drive` staleness on the **ROS clock** -- the same defect the
  2026-09-13 audit removed from three other nodes (issue #22), in the one node where it
  matters most, because its timeout is the last thing that takes a driving pulse off the wire.
  A backwards NTP step (likely on a Jetson with no RTC battery) made the age negative, which
  read as FRESH; a frozen clock made it never fire at all. Now `RCL_STEADY_TIME`, with a new
  L3 test that fails against the old source.
- All four `pwmchip`/channel parameters defaulted to `0`, so a bare `ros2 run` put **both
  pulses on `pwmchip0/pwm0`** and the throttle silently overwrote the steering 50 times a
  second. On the car that would have looked exactly like a dead servo wire. The node now
  refuses to start on a collision, and `throttle_pwm_channel` defaults to 1.
- Every zero-throttle gate now **holds** the last commanded steering angle instead of snapping
  the rack to centre in one cycle, which bypassed the node's own steering rate limiter and
  straightened a car still travelling on its arc. The TTC gate already did it this way; the
  watchdog and sanity paths did not. Now one rule, one shared function, and `safety_node`'s
  fail-closed path uses it too.

**Not fixed, filed:** `safety_node` latches `min_scan_range_m_` forever -- there is no `/scan`
staleness watchdog, so a dead LiDAR either disables the TTC gate silently or brakes forever,
depending on what the last return happened to be. Which way it should degrade is a human
decision on a safety layer and there is no LiDAR fitted, so it is recorded rather than guessed
at.

**Tests.** 283 before, 313 after in `ros-dev` (arm64). No tolerance, golden file or coverage
gate was weakened; `racer_safety`'s gate-logic branch-coverage gate stays at 100%. Two
committed assertions were INVERTED rather than relaxed, both toward fail-closed, both with the
reason written next to them: `is_stale`'s "negative age is not stale" case, and the QoS test's
"watchdog centres the steering" assertion (which now asserts the angle is HELD, and adds a
check that no angle from the incompatible publisher ever reached `/drive`). Separately, and
**reproduced on unmodified `main` before any change here**, five of `test_safety_node_launch`'s
timing-sensitive assertions fail when `colcon test` runs every package's launch tests
concurrently on this 4-CPU colima VM (283 tests, 9 failures); they are green run on their own
or with `--parallel-workers 1`. Domain IDs are already distinct per file, so it is CPU
starvation, not cross-talk. Filed as a follow-up.

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

## 2026-09-20 -- safety mux perfboard soldered: GPIO mapping confirmed against the physical board

**Change.** Cameron finished soldering the layer-1 mux perfboard and measured the actual hole
assignments against the board in hand, replacing the planned-but-unconfirmed mapping from the
2026-09-11/2026-09-12 entries above. New RP2040 GPIO assignments, `pico/main.c`'s seven
`#define`s and `firmware/safety_mux/README.md`'s pinout table and connector map updated
together in the same change:

| Signal | old GPIO (planned) | new GPIO (as-built) |
|---|---|---|
| RC receiver kill-switch channel (in) | 2 | 12 |
| Jetson steering PWM (in) | 3 | 10 |
| Jetson throttle PWM (in) | 4 | 7 |
| Jetson heartbeat (in) | 5 | 5 (unchanged) |
| Servo PWM (out) | 6 | 1 |
| ESC/VESC PWM (out) | 7 | 3 |
| Power cutoff (out) | 8 | 0 |
| Fault LED | `PICO_DEFAULT_LED_PIN` | unchanged |

**Why.** The 2026-09-11/12 entries above recorded a *planned* column layout, drawn ahead of
soldering; per those entries' own "Status" lines, nothing had been soldered, powered, or
measured yet. With the board now physically built, the planned holes did not all line up with
where the level shifter, headers, and Pico footprint actually landed on the perfboard, so
Cameron measured the as-built board directly and confirmed this mapping against it, rather
than against the drawing. This is a pin-assignment change only: no mux decision logic, no
watchdog timing, no PWM validity check, and no output/safe-state behavior changed. The
underlying pico-sdk facts already checked before this remap still hold with the new numbers:
`pico_enable_stdio_uart` is 0 and `pico_enable_stdio_usb` is 1 in `CMakeLists.txt`, so GP0/GP1
were free for GPIO 0 (power cutoff) and GPIO 1 (servo PWM); and GP1 (servo, slice 0 channel B)
and GP3 (ESC, slice 1 channel B) sit on independent PWM slices, so the two 50 Hz outputs still
cannot fight over a shared period register, while GP0 (cutoff) stays a plain digital output
and is never configured as PWM.

**What was updated in this change.** `pico/main.c`'s seven `#define`s;
`firmware/safety_mux/README.md`'s pinout table and board connector map (now marked confirmed
against the as-built board rather than planned/unverified); `planning-docs/05-safety-mux-and-kill-test.md`'s
step 9 wiring list and step 10's cutoff GPIO; `ros_ws/src/racer_drivers/README.md`'s cabling
table. The 2026-09-11/2026-09-12 entries above and `docs/notes/safety-mux-first-build.md` are
left as originally recorded -- they document the planned numbers that were true on the dates
they were written, not the as-built board.

**Rebuild.** Firmware rebuilt from this branch exactly per `docs/notes/safety-mux-first-build.md`'s
containerized build command (`debian:bookworm` arm64, `arm-none-eabi-gcc`, pico-sdk 2.1.0 via
`FetchContent`, `tools/gen_params.py` run as part of the build). Output
`firmware/safety_mux/build-artifacts/safety_mux.uf2`, 79360 bytes,
sha256 `ccddd7452d1c6dc114805d4f265a96be6565ee238b911141bc26dd2f2b132683`. Host logic tests
(`.github/scripts/safety_mux_host_tests.sh`) still pass, all 320 assertions -- nothing in
`logic/` depends on a GPIO number, so none of it needed to change.

**Status.** Pin mapping confirmed against the as-built perfboard. Still nothing powered,
flashed, or bench-tested against real hardware; roadmap task 1.3's kill test is still pending.

## 2026-09-20 -- opt-in diagnostic firmware build that reports mux state over USB

**Why.** The board is built, powered, and armed (3.3 V rail good, LED dark), the Jetson drives
header pin 15 (`pwmchip0`, steering) and pin 33 (`pwmchip2`, throttle) at a confirmed ~1.64 V
average at 50 percent duty, the heartbeat toggles header pin 7 at 50 Hz verified at kernel
level, and the Flysky radio is bound with kill on CH5 (VrA, clockwise armed, failsafe to the
kill end). But the steering servo does not respond off the board's SERVO header and goes limp
rather than holding centre, while the same servo steers correctly plugged straight into the
receiver's CH1. A meter at the servo header reads 5 V on power and about 0.24 V average on
signal, which is what a 3.3 V 1500 us 50 Hz pulse train averages to, and which a meter cannot
tell apart from a malformed waveform at the same duty. The mux reports nothing: the only
`printf` in the firmware is the refuse-to-arm fault line, so "is it cutting, and on which
condition" is unobservable from outside.

**Change.** A diagnostic variant of the same firmware, behind a new `DIAG_BUILD` CMake option
that defaults to OFF. With it ON, `pico/diag_report.c` is compiled in and prints one line over
USB serial about twice a second: every captured pulse width on GP12/GP10/GP7 with a named
reason when a channel is unusable (`NO_EDGES`, `STALE`, `OUT_OF_RANGE`), the heartbeat age
against the configured timeout and the watchdog's verdict, the decoded kill-switch position
with the arm and kill edges it is compared against, the PASS/CUT decision with the winning cut
condition numbered by priority, and the commanded servo/ESC pulse widths plus the cutoff
GPIO's read-back level. A four line banner with the pin map and the params prints each time a
USB host attaches. New files: `pico/diag_report.{h,c}`, `docs/notes/mux-diagnostic-build.md`.

**Safety impact: none.** Nothing in `logic/` was modified. Same `mux_decide()`, same priority
order, same fail-safe defaults, same outputs. The diagnostic code reads state the cycle has
already produced and prints it; it takes no decision and writes no pin. The one hook it needed
is `pwm_capture_diag()`, a read-only accessor in `pico/pwm_capture.c` that reads the same
fields `pwm_capture_read_us()` reads under the same interrupt-disabled section and writes
nothing. It and the `main.c` call sites are behind `#ifdef SAFETY_MUX_DIAG`, which only
`DIAG_BUILD=ON` defines.

**Not blocking on USB.** Checked against pico-sdk 2.1.0's
`src/rp2_common/pico_stdio_usb/stdio_usb.c` rather than assumed: `stdio_usb_out_chars()`
returns immediately with no host attached, but with a host attached and not reading it spins
until `PICO_STDIO_USB_STDOUT_TIMEOUT_US` (default 500000, half a second, i.e. 100 missed
control cycles). The diagnostic target therefore compiles with
`PICO_STDIO_USB_STDOUT_TIMEOUT_US=0` so characters are dropped instead of waited on, and
`diag_report_tick()` checks `stdio_usb_connected()` before doing any formatting work.

**Builds.** Both produced in the container from `docs/notes/safety-mux-first-build.md`
(debian:bookworm arm64, `arm-none-eabi-gcc` 12.2, pico-sdk 2.1.0 via `FetchContent`,
`tools/gen_params.py` run as a build step), zero compiler warnings, and the three touched
files additionally recompiled with `-Wall -Wextra` for zero warnings:

| Artifact | Bytes | sha256 |
|---|---|---|
| `firmware/safety_mux/build-artifacts/safety_mux.uf2` (shipping, `DIAG_BUILD=OFF`) | 79360 | `ccddd7452d1c6dc114805d4f265a96be6565ee238b911141bc26dd2f2b132683` |
| `firmware/safety_mux/build-artifacts/safety_mux_diag.uf2` (diagnostic, `DIAG_BUILD=ON`) | 90624 | `86a8291d0f552b3b2d67768b4a114c8c00d3d4911fe394cdae247a6fa8272f5b` |

The shipping hash is identical to the one recorded in the 2026-09-20 pin-remap entry above,
built before any of this existed: the default build really is byte for byte unchanged. Both
artifacts are gitignored build output, not committed.

**PWM output audit (same day, prompted by a second bench measurement).** A DC meter on GP1
read a constant 0.36 V, about 10.9 percent duty, where a 1500 us pulse in a 20 ms frame should
be 7.5 percent, about 0.25 V. `pico/pwm_output.c`'s slice, divider and wrap arithmetic was
audited against the RP2040 peripheral and pico-sdk 2.1.0's `hardware_pwm/include/hardware/pwm.h`.
**No defect was found**: 125 MHz / 62.5 = 2 MHz = 0.5 us per count, TOP+1 = 40000 counts =
20.000 ms = 50.000 Hz exactly; 62.5 is exact in the 8.4 fixed-point divider (int 62, frac 8)
with no truncation; GP1 (slice 0 channel B) and GP3 (slice 1 channel B) are separate slices,
each configured by its own `pwm_output_init_channel()` call; GP0 is slice 0 channel A but is
driven through SIO and never routed to the PWM function, so it writes no PWM register and
cannot disturb slice 0 for GP1; `pwm_output_set_us()` converts with the same 0.5 us per count
the init programmed; and the compare register is double-buffered, so the 200 Hz writes cannot
produce runt pulses.

The significant correction is that **duty cycle is `level / (TOP + 1)` and contains no clock
term**, so a divider or `clk_sys` error changes the frame RATE and the microsecond pulse
width but cannot change the DC average at all. The "period is 13.75 ms" hypothesis is
therefore ruled out as an explanation of 0.36 V; only a compare level near 4360 (about
2180 us) would do that, and nothing in the decision path can command 2180 us (neutral is
1500, the range maximum is 2000). Also worth recording: the servo header signal pin read about
0.24 V earlier the same evening, which is the expected 7.5 percent, and GP1 reads 0.36 V now,
on the same net with continuity confirmed. Both cannot be right, and a DC meter averaging a
50 Hz 7.5 percent square wave is a plausible source of the discrepancy.

Two additions came out of the audit, neither of which changes emitted code. `pico/pwm_output.c`
gained `_Static_assert`s pinning the frame arithmetic, including that `SYS_CLK_HZ` is
125000000 (the 62.5 divider's previously silent assumption, now a build failure rather than a
comment). `_Static_assert` emits no code and the shipping `.uf2` hash above is unchanged with
them in place, rebuilt to confirm. The diagnostic build gained a second `PWMREG` line per
report, read back from the peripheral registers: slice and channel, TOP, divider, compare
level, slice enabled, whether the pin is really switched to the PWM function, the measured
`clock_get_hz(clk_sys)`, and the resulting pulse width, frame period, frequency and duty. The
duty field is directly comparable to a multimeter reading, so this class of question is
answerable from the serial log instead of from a meter.

The one warning seen while auditing, a GCC `-Warray-bounds` false positive on the SDK's
inlined `pwm_set_clkdiv()`, is pre-existing: the same file on `main` produces the same two
occurrences under `-Wall -Wextra`, and the project's own build flags produce zero warnings.

**Tests.** `.github/scripts/safety_mux_host_tests.sh` still green, all 320 assertions.

**Status.** Nothing here has run on an RP2040. The example output lines in
`docs/notes/mux-diagnostic-build.md` were produced by compiling the real `diag_report.c`
formatting code on the host against scripted inputs, so the field text is exact, but whether
the CDC port enumerates, whether the 2 Hz print leaves the 200 Hz loop undisturbed on the
chip, and whether the captured numbers are right are the bench questions this build exists to
answer. Roadmap task 1.3 stays `[~]`.

## 2026-09-21 -- the heartbeat pin never drove: 40-pin header pads are tristated

**What happened.** The mux's new diagnostic build was flashed and its first useful report
was `HB gp5=NO_EDGES_EVER`: the Pico has never seen a heartbeat edge. Meter on the Jetson's
header pin 7 while `gpioset` holds `gpiochip0` line 144 high: **0.0 V**. Pin 17 reads 3.4 V
on the same meter with the same ground lead, so the meter and the pin counting are both
sound. Pins 11, 12, 13, 16, 18, 19 and 21 driven high the same way (lines 112, 50, 122, 126,
125, 135, 134): all 0.0 V. Pin 15 read 0.0 V as a plain GPIO too, and yet the same pad drove
1.64 V average once a jetson-io overlay muxed it to `pwm1`.

**Cause, verified read-only on the board.** The pad output driver is disabled.
`/sys/kernel/debug/pinctrl/2430000.pinmux/pinconf-groups` shows `tristate=1` on every pad
that reads 0.0 V (`soc_gpio59_pac6` = pin 7, `soc_gpio32_pq5` = pin 29) and `tristate=0` on
every pad a jetson-io overlay configured (`soc_gpio39_pn1` = pin 15, `soc_gpio21_ph0` = pin
33, `soc_gpio19_pg6` = pin 32). 165 of 167 pads show `(MUX UNCLAIMED)` in `pinmux-pins`; the
three jetson-io ones show `(HOG) function gp`, where `gp` is the GP PWM controller and not
"general purpose IO". The pad's TRISTATE is bit 4 of its PINMUX register in `pinmux@2430000`;
the GPIO controller is `gpio@2200000` and `gpio-tegra186.c` has no tristate bit and never
touches the pinmux one. So the GPIO controller can hold a line output-high, report it
correctly through every software path, and the pad still sits high impedance. That is
exactly what happened. The tristate hypothesis held.

**The uncomfortable part.** Every "verified toggling" claim in `tools/jetson_heartbeat/`
was kernel-internal: `gpioinfo`, `/sys/kernel/debug/gpio` showing `out hi`/`out lo`, and a
program polling that same debugfs file for a rate measurement. All of it was true and none
of it was evidence that a wire would see anything. The 2026-09-12 entry's rate measurement
(200 transitions in 1.9901 s) was measuring a register. That README now says so plainly, and
lists the meter reading and the mux's `HB ... OK` diag line as the two required checks.

**Also corrected:** header pin 32 on this carrier is `soc_gpio19_pg6` (PG.06, `gpiochip0`
line 41), **not** `gp_pwm2_px2`. `gp_pwm2_px2` is PX.02 / line 116 and is not routed to the
40-pin header at all. Both the decompiled jetson-io overlay and jetson-io's own live pinmap
agree.

**Change.** New `tools/jetson_pinmux/`: `racer-hdr40-gpio.dts`, an overlay built on the
decompiled `/boot/jetson-io-hdr40-user-custom.dtbo` from this board, plus idempotent
`install.sh`, `rollback.sh` and `verify.sh`. It configures pin 7 and pin 32 as driven GPIO
outputs (`tristate=0`, `enable-input=0`, `pull=0`, no `nvidia,function` so the mux and
GPIO_SFIO_SEL are left to the GPIO request path) and carries the pwm1 (pin 15) and pwm5
(pin 33) nodes copied verbatim so steering and throttle keep working. One overlay rather
than two, because both would create `exp-header-pinmux` under the same `&pinmux` target and
both would claim `pinctrl-0`. The cost is pwm7 on pin 32, dropped on purpose. `install.sh`
removes the jetson-io entry from the `OVERLAYS` line and `rollback.sh` puts it back.

**Status.** Prepared, **not applied**. The Jetson was touched read-only this session:
another session is using the board. The `.dts` compiles with `dtc -@` and decompiles to the
same node structure, `__symbols__`, `__fixups__` and `__local_fixups__` as the jetson-io
template, but nothing is installed and nothing has been rebooted, so "pin 7 measures 3.3 V"
remains unproven. The pin decision, 7 versus 32, is being made on the bench and no default
in `tools/jetson_heartbeat/` or `racer_drivers` was changed.

**One thing still unexplained.** Pin 7's pad reports `pull=2` (pull-up) with
`pull-up-strength=31` and still measures 0.0 V while tristated. A tristated pad with a
pull-up ought to float near 3.3 V. Noted rather than explained; it does not affect the
diagnosis or the fix, and the overlay sets `pull=0` on that pad regardless.

## 2026-09-21 morning -- pinmux overlay installed, first PASS, G1 kill test and heartbeat-loss test proven

Everything below was observed on the actual hardware this morning, wheels off the ground
throughout.

**Pinmux overlay.** `tools/jetson_pinmux/install.sh` was run for real (`dtc` 1.6.1 installed
via apt first) and the Jetson rebooted. Live pinconf for `soc_gpio59_pac6` (header pin 7)
went from `tristate=1` before to `tristate=0` after; pins 15 and 33 kept function `gp`; pin
32 (`soc_gpio19_pg6`) also reads `tristate=0` now. `extlinux`'s `OVERLAYS` line now points at
`/boot/racer-hdr40-gpio.dtbo`. Meter on the perfboard's GP5 socket hole with pin 7 held high:
3.44 V (meter reads about 3 percent high, so this is consistent with 3.3 V logic). The
heartbeat service itself was left unchanged, still on line 144.

**Pico hardware.** Pico #1 is confirmed dead: it heats within seconds on clean desktop USB
power alone, with nothing else connected. Retired. The spare Pico was flashed with
`safety_mux_diag.uf2`, seated on the perfboard, and stayed cool after several minutes of
running.

**UBEC.** The "3A-6S UBEC" (5V/6V jumper on a 3-pin header, middle pin common) reads 6.81 V
on the 6V position and 5.69 V unloaded on the 5V position (about 5.5 V real, sagging to about
5.3 V with the Pico attached). The bulk cap is on the 5V rail (top + middle pins). Pico #1
had been run on the 6V position, which is the likely cause of its death. Follow-up open item:
a Schottky diode in series to the Pico's VSYS feed, or a proper 5.0 V regulator, for margin.

**First mux PASS.** `tools/mux_diag/read_mux_diag.py` over the Pico's USB on the Jetson,
transmitter off:

```
KILL UNREADABLE NO_EDGES | HB OK age 7ms | STEER 1484us | THR 1484us | DECISION CUT reason 1:RC_SIGNAL_INVALID
```

Transmitter on, kill knob counter-clockwise:

```
KILL KILLED 1000us ... DECISION CUT reason 1:RC_KILL_SWITCH
```

Knob clockwise:

```
KILL ARMED 2000us | HB OK age 6ms | STEER 1484us | THR 1485us | DECISION PASS
```

This is the first `DECISION PASS` this mux has ever produced.

**G1 kill test, steering only.** Throttle held at 1500 us, wheels off the ground. Armed, a
1200/1500/1800/1500 us sweep on `pwmchip0` turned the front wheels left/centre/right/centre.
The identical sweep with the kill knob killed: the wheels did not move and the mux stayed
`CUT reason 1` throughout.

**Heartbeat-loss test, armed.** Steer left (`STEER 1172us`, `PASS`); `systemctl stop
racer-heartbeat` -> `HB TIMED_OUT`, `DECISION CUT reason 2:WATCHDOG_TIMEOUT`, servo
self-centred; steer right with the heartbeat dead -> `STEER 1797us` seen arriving at the mux,
`DECISION CUT`, wheels did not move; recentre and `systemctl start racer-heartbeat` -> `HB
OK`, `PASS` again. The servo buzzed while holding 1200 us, probably against its mechanical
stop -- steering endpoints are still the provisional 1000-2000 us and must be measured before
any floor driving.

**Not yet done:** VESC configuration, throttle sweep, FSESC 4.12 swap, a real kill switch
(this radio only offers VrA/VrB on CH5/CH6, not a discrete switch), the motor sensor cable
adapter.

**Tooling.** `tools/mux_diag/read_mux_diag.py`'s one-shot mode was found, during this same
session, to sometimes hang and hold `/dev/ttyACM0` open, and consecutive one-shot calls each
took about 5 s. Fixed in the same PR as this entry; see
`docs/notes/bench-session-2026-09-20.md`'s open items list.

## 2026-09-21 midday -- VESC configured, first motor spin under Jetson command, G1 bench complete

Wheels off the ground throughout. Board on the 3S pack (~12.2 V) the whole session.

**VESC configuration.** Flipsky FSESC 6.7 (hardware reports "60-no_hw_limits", VESC ID 114)
configured in VESC Tool from the owner's Windows desktop over micro-USB. VESC Tool reported
"limited mode" (a firmware version mismatch); configuration writes and read-backs worked
anyway. Firmware was deliberately not updated: the 6.7 runs below its rated input voltage and
a brown-out mid-flash could brick it, and the FSESC 4.12 replaces it soon regardless.

Sensorless FOC detection succeeded: R 6.60 mOhm, L 1.88 uH, Lq-Ld 0.29 uH, flux linkage
0.25 mWb. The detection wizard suggested 71 A motor current; overridden.

Limits written per `planning-docs/06-vesc-config-and-jetson-bringup.md` step 3: motor current
max 30 A, motor brake -15 A, battery max 25 A, battery regen -15 A, absolute max 70 A, battery
cutoff start 10.2 V / end 9.6 V, max ERPM +12000 / min -12000 (12000 ERPM is about 3 m/s tread
speed with the stock Slash 4x4 11.82:1 overall drive, 4.31 in tyres, 2 pole pairs), FET and
motor temperature start 85 C. App layer: PPM input, control type 3 (Current No Reverse With
Brake), pulses 1.0/1.5/2.0 ms, hysteresis 0.15, safe_start on, ramp 0.4 s / 0.2 s. Exported
configuration committed as `config/vesc/2026-09-21-fsesc67-motor.xml` and
`2026-09-21-fsesc67-app.xml` (PR 60, merged). Step 6 of the same planning doc, mirroring these
limits into `config/vehicle_params.yaml`, is still open -- see open items below.

**Throttle test through the mux, first motor spin.** Jetson driving the throttle PWM through
the mux, wheels off the ground, steering held at 1500 us, kill knob armed (mux
`DECISION=PASS`): 1560 us produced nothing; 1600 and 1650 us made the rear tyres click for a
few seconds without turning (a sensorless start attempt with too little current against the
4WD drivetrain); 1700 then 1750 us spun the wheels up fast, to the ERPM cap. The owner turned
the kill knob counter-clockwise while the wheels were spinning: they stopped, and the mux read
`KILL 1000us KILLED`, `DECISION=CUT reason 1:RC_KILL_SWITCH`. Throttle then returned to
1500 us. This is the first time the car has moved under Jetson command, and it proves the
motor-channel radio kill.

Consequence for software: from rest, this drivetrain needs roughly 1700 us to start
sensorless -- about 40 percent of the current throttle range. The throttle map in
`racer_drivers` / `vehicle_params` (`throttle_full_scale_mps` 5.0 provisional, PWM
1000/1500/2000) does not model this deadzone yet. A sensored hall adapter (already planned,
see the 2026-09-14 entry) is the proper fix for low-speed start; noted as an open item below
rather than worked around in software.

**G1 bench evidence, complete.** Combined with the morning's steering kill test and
heartbeat-loss test (see the 2026-09-21 morning entry above), Gate G1's bench evidence is now
complete: the teleop command path runs through layer-1 safety end to end, the kill switch is
proven on both the steering and throttle channels, and heartbeat loss is proven on steering.
See `docs/notes/bench-session-2026-09-20.md` for the full numbered results and open items, and
`docs/notes/build-status-2026-09-14.md` for the updated build-status snapshot.

**Tooling, `mux_diag` reader regression.** `tools/mux_diag/read_mux_diag.py`, as merged in PR
59, was found this session to fail against the real device
(`read_mux_diag: timed out ... no parseable mux diagnostic line seen`) while a raw
`stty -F /dev/ttyACM0 115200 raw -echo; timeout 5 cat /dev/ttyACM0` capture read perfectly good
lines on the same device at the same time. Root cause: PR 59's fix made `run_once`'s `--lines`
budget count every raw serial line read (including the diagnostic firmware's four-line attach
banner, a discarded partial first line, and stray `PWMREG` continuation lines), not only
parseable decision lines, and dropped the default from 40 to 1. On a fresh attach, the
banner prints before any verdict line, so the single allowed read under the new default
almost always landed on banner text and gave up, even with good verdict lines arriving right
behind it -- reproduced directly against a 15-line raw capture from the live device,
committed as `tools/mux_diag/tests/fixtures/mux_raw_2026-09-21.txt`. Fixed in this PR:
`run_once`'s budget now counts only parseable verdict lines, bounded overall by `--timeout`
(still 5 s by default); non-decision chatter is skipped for free. Covered by a new
fixture-driven test that drives the real `_SerialLineReader` (open/select/os.read) against a
pty loaded with the captured bytes, plus updated unit tests for the new budget semantics.

## 2026-09-21, late evening: VESC speed mode tried and reverted, 6000 ERPM cap kept

Tried PPM control type PID Speed Control No Reverse (pid_max_erpm 6000) to get a slow,
controllable spin from the sensorless Xerun 3652SD. Result: the VESC accepted the write
(read-back confirmed the mode), logged no faults, and did not turn the motor at all, even
with the mux forwarding a 1953 us pulse (verified simultaneously: /drive 4.76 m/s, Jetson duty
1975 us, mux in 1953, mux out 1953, armed, PASS). Sensorless FOC speed control from standstill
does not start this motor. Reverted the control type to Current No Reverse With Brake (3) and
kept the new 6000 ERPM cap (about 1.5 m/s at the tread), which is what
config/vesc/2026-09-21b-fsesc67-app.xml and -motor.xml now record. The hall sensor adapter
remains the real fix for low-speed control.

Also found tonight: two keyboard_teleop_node sessions running at once (two terminal tabs)
interleave zeros with the live setpoint on /drive_raw, which looks like jittery steering and
a dead throttle. One keyboard session at a time; check `ros2 topic info -v /drive_raw` shows
one publisher. A UBEC ground wire was found unplugged and reseated (battery in, no incident).
