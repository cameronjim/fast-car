# First-boot ROS 2 audit, 2026-09-13

Question asked: **will `ros_ws/src/*` run correctly on the real car the first time it is
powered on?** Not "does it pass CI", and not "does it work in the sim" -- the sim bridge
supplies `/scan`, `/sim/ground_truth_odom` and a steady `/drive` consumer, and the car
supplies none of those on day one.

Method: read every node in scope, then run each one headless in the `ros-dev:local`
container (arm64, same architecture as the Orin Nano) **without the sim bridge**, the way it
would come up on the car. Every finding below is either a reproduction that was actually run
or a code citation. Branch: `audit/first-boot-ros`.

Out of scope and untouched: `sim/bridge` (read for context only), `firmware/safety_mux`,
`tools/jetson_heartbeat`, and `ros_ws/src/racer_drivers` (another agent is building
`pwm_output_node` there in parallel; it was uncommitted and mid-build during this audit and
is excluded from every build/test invocation below with `--packages-ignore racer_drivers`).

---

## Findings

| # | Sev | Where | What happens on the car | Fix / follow-up |
|---|---|---|---|---|
| 1 | **blocker** | `racer_safety/src/safety_node.cpp:208,259,261` (pre-fix) | Watchdog age and rate-limiter dt were measured on the node's **ROS clock**. That clock is `CLOCK_REALTIME` when `use_sim_time:=false`, so an NTP correction or VM resync steps it and the measured age goes **negative** -- the root cause of issue #22, reproduced below. Under `use_sim_time:=true` with no `/clock` publisher the same clock is pinned at 0, so every measured age is exactly `0.0` and **the watchdog treats a permanently silent `/drive_raw` as permanently fresh**. | **FIXED**: both measurements moved to `RCL_STEADY_TIME`. `this->now()` still stamps outgoing messages. New `test/test_safety_node_clock_launch.py`. |
| 2 | **bug** | `racer_control/src/tracker_node.cpp:202,208` (pre-fix) | Same root cause, different node. `/odom` staleness measured on the ROS clock. A backwards wall-clock step makes the elapsed time negative, which is never `> odom_timeout_s`, so `/odom` reads "fresh" and tracker_node keeps publishing pure-pursuit commands off a **frozen pose** for the whole duration of the step. safety_node sees a healthy `/drive_raw` and passes it through. | **FIXED**: `RCL_STEADY_TIME`. New `test/test_tracker_node_clock_launch.py`. |
| 3 | **bug** | `racer_tools/twist_teleop_adapter_node.py:134,139` and `twist_teleop.py:121` (pre-fix) | The member is named `_last_twist_monotonic` but was read from `self.get_clock()`, which is not monotonic. `should_use_zero_command` tested `elapsed > timeout_s`, so a **negative** elapsed time read as *fresh*: a browser tab closing during a backwards clock step left the driver's last non-zero command republished at 50 Hz until the clock caught up. Under `use_sim_time` with no `/clock`, elapsed is always `0.0` and the timeout never fires at all. | **FIXED**: node measures on `time.monotonic()`; the pure function now fails closed on negative/NaN/inf. 6 new L1 cases. |
| 4 | **bug (latent, not fixed)** | `config/vehicle_params.yaml:128-129` | `ttc_warning_s` and `ttc_brake_s` are `null`. The plumbing handles this correctly and loudly (`safety_node` logs `ttc_brake_s=unset (untuned; TTC gate is a no-op)` at startup, verified below), but the practical consequence is that **once a LiDAR is fitted, TTC braking is still a no-op** until someone tunes those two numbers. This is the safety gate most likely to be assumed present on a first drive. | Follow-up, filed as a GitHub issue. Not a code fix: picking the thresholds is Phase 1/2 tuning work and a human decision. |
| 5 | smell | `racer_safety/src/safety_node.cpp:268-270` | `/safety/events` is emitted **every cycle** while a gate is engaged, so from boot until the tracker starts the car floods `/safety/events` with identical watchdog records at 50 Hz (measured: 248 records in ~14s). 05-safety requires every intervention to be logged, and this satisfies that, but it makes the rosbag intervention count a function of how long the operator took to start the stack rather than of how the car behaved. Interventions are a *reported evaluation metric* (09). | Follow-up, filed as a GitHub issue. Changing the emission policy is a design decision about what 09 counts, not a bug to patch quietly. |
| 6 | smell | `racer_tools/keyboard_teleop_node.py:126-140` | If `KeyboardTeleopNode()` raises (e.g. `vehicle_params_loader` cannot find the repo root), `main`'s `finally` block references the unbound `node` and the real error is masked by a `NameError`. Also needs a real TTY, so `ros2 run` from a non-interactive context dies in `termios`. Both are documented, neither can move the car. | Left alone. Noted here. |
| 7 | gap | `racer_bringup/launch/` | **There is no car launch file.** All three launch files (`bridge.launch.py`, `sim_teleop.launch.py`, `sim_autopilot.launch.py`) start `racer_gym_bridge/bridge_node`, so **none of them can launch without the sim bridge** -- they are all sim-only and none says so in its filename. There is also no `racer_drivers` node to consume `/drive`, so on the car `/drive` currently has no subscriber at all. | Recorded only. `racer_bringup/launch/car_teleop.launch.py` and `racer_drivers/pwm_output_node` are being built by another agent in parallel; deliberately not created here. |
| 8 | gap | `ros_ws/src/racer_state/` | Empty (`.gitkeep` only). No `/odom` publisher and no `/pose` publisher exists for the car. `tracker_node` therefore cannot run on the car at all yet, and safety_node's covariance gate stays a stub (`gate_logic.cpp:74-80`, `has_pose_input` hardcoded `false` at `safety_node.cpp:263`). | Recorded only; roadmap Phase 2. |
| 9 | note | `ros_ws/src/racer_policy/` | Has no `package.xml`, so `colcon test` in `ros_ws` never runs its 5 test files. This is **not** a defect: `.github/scripts/run_python_tests.sh:32` runs it separately with an explicit >=90% line gate. Worth knowing when reading a local `colcon test` summary. | None. |

### Status after PR #39 (2026-09-13)

PR #39 (`feat/car-first-boot`) merged `racer_bringup/launch/car_teleop.launch.py` and
`racer_drivers/pwm_output_node` into `main`. This does not change this audit's findings text
(not rewritten, per policy); it changes which of the findings above are still open.

- **Finding #7 (gap): closed.** A car launch file now exists
  (`racer_bringup/launch/car_teleop.launch.py`) and does not start
  `racer_gym_bridge/bridge_node`. `/drive` now has a real subscriber on the car via
  `racer_drivers/pwm_output_node`.
- **Finding #8 (gap): still open.** `ros_ws/src/racer_state/` is still empty. There is still
  no `/odom` or `/pose` publisher for the car, `tracker_node` still cannot run on the car, and
  safety_node's covariance gate is still a stub (`has_pose_input` hardcoded `false` at
  `safety_node.cpp:263`). Unaffected by PR #39; still roadmap Phase 2 work.
- Findings #1-#6 and #9 are unaffected by PR #39 and their status above still holds (#1-#3
  fixed in this audit's own branch, #4-#5 filed as follow-up issues, #6 left alone until the
  chore/first-boot-followups fix, #9 not a defect).

### Checked and found correct

These were the things most likely to be wrong, and were not.

- **safety_node with no LiDAR ever.** `/scan` never arrives, `min_scan_range_m_` stays
  `+inf`, `is_valid_range` rejects it, the TTC gate no-ops. No brake, no refusal, no throw.
- **safety_node with `/drive_raw` silent from boot.** Publishes a `{0,0}` brake on `/drive`
  at a **measured 50.13 / 50.05 / 49.99 Hz** from the first cycle, before any input ever
  arrives, with a `watchdog` `/safety/events` record. It does not wait for input.
- **Fails closed on internal exception.** The whole cycle body is inside `try/catch`
  (`safety_node.cpp:270-307`); the catch publishes brake plus an `internal_fault` event.
  Covered by the existing `inject_fault` L3 test.
- **`/safety/events` on every intervention path.** Every `return` in
  `SafetyGateLogic::evaluate` pushes an event first; the internal-fault path in the node adds
  the seventh source.
- **Negative age is treated as STALE, not fresh.** `gate_logic.cpp:127-128` brakes on a
  negative age. The task's minimum bar was already met before this audit; finding #1 removes
  the ability to produce a negative age at all, and the existing branch stays as defence in
  depth.
- **QoS compatibility across every shared topic.** `/drive_raw` reliable→reliable,
  `/drive` reliable→reliable, `/scan` best_effort→best_effort (a reliable subscriber to a
  best_effort publisher would receive nothing; there are none). `/sim/raceline` and
  `/sim/map` are transient_local depth 1 on both ends. Every profile in scope sets an
  explicit depth, per 10-conventions. Real-car note: a stock LiDAR driver publishes `/scan`
  with SensorDataQoS (best_effort), which matches safety_node.
- **Topic names and types match 04-architecture exactly** for every topic in scope
  (`/drive_raw`, `/drive`, `/scan`, `/safety/events`). `/sim/*` names are correctly
  sim-namespaced rather than squatting on architecture names.
- **Parameter names match** between all three launch files and the node declarations, and
  every parameter in scope is declared with a `ParameterDescriptor`; every numeric one in
  scope carries a range. No undeclared parameter use.
- **CLAUDE.md invariant 2** holds: every physical constant in `safety_node.cpp` and
  `tracker_node.cpp` is read from `VEHICLE_PARAMS`. The only hand-typed numbers are tuning
  knobs (lookahead gains, margin fractions, watchdog cycle count), each declared as a ROS
  parameter with a comment explaining why it is not a physical constant. The single
  hardcoded number found, `_STEERING_MAX_RAD = 0.4189` in
  `test_safety_node_launch.py:64`, is in test code and is commented as mirroring the YAML.
- **CLAUDE.md invariant 3** holds: every failure path in `racer_policy/contract.py` and
  `verify.py` is a hard `raise` from `racer_policy.errors`. No warning, no override flag, no
  catch-and-continue. Grep of both files shows 17 raises and zero warning downgrades.
- **No `use_sim_time` leak.** No launch file in scope sets it, so nothing silently depends on
  `/clock`. (This is why finding #1's frozen-clock variant is latent rather than active --
  one wrong launch argument away.)
- **arm64 clean.** No x86-specific flags in any `CMakeLists.txt` (no `-m64`, no `-march`, no
  arch-conditional blocks) and no arch-pinned Python wheels. A clean build and a fully green
  test suite on arm64 is itself the evidence.
- **Clean shutdown** on SIGINT for all five runnable nodes; the two new launch tests assert
  exit codes post-shutdown.

---

## Issue #22: root cause, reproduced

`docs/notes/milestone-5-browser-teleop.md` hypothesised the age was computed across two
processes' clocks (`safety_node`'s `now()` minus `tracker_node`'s message stamp). **That
hypothesis is wrong.** `safety_node` never read `msg->header.stamp`; it stamped receipt with
its own `this->now()` (`safety_node.cpp:208` pre-fix). One process, one clock.

The actual mechanism is simpler and worse: that one clock is steppable. With
`use_sim_time:=false`, `this->now()` resolves to `CLOCK_REALTIME`, which NTP, a VM resync
after suspend, or `date` can move in either direction. That is exactly the condition the
milestone note describes ("after extended, heavy back-to-back container use" on colima).

Reproduction, run for this audit. safety_node running normally with a healthy 50 Hz
`/drive_raw` stream; the container host's clock stepped back 5 seconds mid-run:

```
detail: /drive_raw stale or age invalid (timeout=0.060000s, age=-4.858085s); braking
detail: /drive_raw stale or age invalid (timeout=0.060000s, age=-4.875257s); braking
detail: /drive_raw stale or age invalid (timeout=0.060000s, age=-4.895444s); braking
...
```

Same reproduction after the fix: **0 negative-age records** (248 watchdog records remain over
the same window, all with positive ages in the 0.078-0.16s range -- those are genuine
staleness trips caused by `ros2 topic pub -r 50` not holding 50 Hz in a loaded container,
which is the watchdog working, not a clock defect).

The direction of the bug on the car matters. A **backwards** step produces a negative age,
which `gate_logic.cpp` already brakes on: disruptive, not dangerous. A **forwards** step
(overwhelmingly likely on a Jetson at first boot -- it has no RTC battery, so the clock jumps
when NTP first syncs after the network comes up) is the dangerous one: if a `/drive_raw`
message lands after the jump, its age is small and the watchdog passes, but `last_eval_time_`
is still pre-jump, so `dt_s` is enormous and the rate limiter's bounds
(`max_acceleration_mps2 * dt_s`, `steering_rate * dt_s`) become effectively infinite for one
cycle. A full-lock steering step and a full-scale speed step would pass the gate unclamped.
A monotonic clock removes both directions.

---

## Test results

Run in `ros-dev:local` (arm64) with `rosdep install --from-paths ros_ws/src sim/bridge`
first -- **note for whoever runs this next:** the image does *not* ship `ackermann_msgs`, so
`colcon build` in a fresh container fails at `find_package(ackermann_msgs)` until rosdep has
run. That is not a repo defect, but it is not written down anywhere either, and it cost time.

**Before (clean checkout of `main`, clean build):**

```
Summary: 217 tests, 0 errors, 0 failures, 0 skipped     <- after
Summary: 203 tests, 0 errors, 0 failures, 0 skipped     <- before
```

Baseline was fully green: 203 tests, 0 failures. No pre-existing failures to report.

**New tests, verified to FAIL against the pre-fix sources** (fixes stashed, tests kept), per
12-testing's "write the failing test first" rule:

| Test | Pre-fix |
|---|---|
| `racer_safety` `test_watchdog_brakes_on_silence_even_with_a_frozen_ros_clock` | FAIL |
| `racer_control` `test_odom_watchdog_stops_drive_raw_even_with_a_frozen_ros_clock` | FAIL |
| `racer_tools` `test_negative_elapsed_time_uses_zero` (+3 more) | FAIL |

**After:** 217 tests, 0 errors, 0 failures, 0 skipped. `clang-format --dry-run --Werror`
clean on both changed C++ files; `ruff check` and `ruff format --check` clean.

No tolerance, golden file, or coverage gate was weakened. Nothing was deleted. The existing
negative-age branch in `gate_logic.cpp` was kept even though the fix makes it unreachable
from the node, because it is the thing that made the pre-fix bug merely disruptive instead of
dangerous.

---

## What I could not verify without hardware

Everything below is untested by this audit and should not be assumed working.

- **Anything downstream of `/drive`.** There is no `vesc_node` and no `pwm_output_node` in
  the tree yet (finding #7), so the entire actuator boundary -- unit conversion, sign
  conventions, the "left positive / drive positive" rules in 04-architecture -- is unwritten
  and therefore unaudited. The wheels-off-ground sweep (12-testing L6) is the only thing that
  can check it.
- **The layer-1 mux.** Out of scope by instruction, and 05-safety is explicit that the kill
  is proved by actually freezing the Jetson (roadmap 1.3), not by reasoning. Nothing in this
  audit says anything about whether the mux works.
- **Real sensor QoS and rates.** I verified publisher/subscriber QoS compatibility *within
  this repo*. Whether the actual LiDAR driver, the actual IMU, and the ingest board publish at
  their claimed rates with the profiles assumed here is a bench measurement.
- **Whether 50 Hz holds on the Orin Nano.** The measured 50.13 Hz is a Docker container on an
  M-series Mac under no load. Tail latency and jitter on the real Jetson under real sensor
  load is roadmap 2.7's histogram, not this.
- **The Jetson's clock behaviour at first boot.** I argued above that a forwards NTP step is
  likely on a board with no RTC battery. I did not measure it. The monotonic-clock fix makes
  it not matter, which is rather the point, but the claim about Jetson clock behaviour itself
  is unverified.
- **TTC braking end to end.** Finding #4: the thresholds are `null`, so the gate has never
  fired against a real LiDAR return. The L3 test overrides them to prove the *gate* works;
  that is not the same as proving the car stops.
- **DDS on a real multi-host network.** Everything here ran on loopback in one container.
  Domain IDs, discovery across the Jetson and a laptop, and multicast behaviour on the car's
  actual network are unverified. No `RMW_IMPLEMENTATION` is pinned anywhere in the repo,
  which means the car inherits whatever the image defaults to.
- **`/pose`, `/odom`, `/ingest/*`, `/power/rail`.** No producer exists for any of them
  (finding #8). The covariance gate and the rail-voltage logging requirement in 05-safety are
  both unexercised.
