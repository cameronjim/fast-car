# Command-path review, 2026-09-14

Question asked: **is the command path correct, simple, and compatible with the hardware that
is about to exist?** Not "does it pass CI". The path under review is the whole of it, from a
keypress to a duty cycle:

```
keyboard_teleop_node | twist_teleop_adapter_node
    -> /drive_raw -> safety_node -> /drive -> pwm_output_node
    -> two 50 Hz PWM pulses -> layer-1 mux -> steering servo + VESC PPM input
```

Method: read every file in that path against `claude-docs/04`, `05`, `06`, `10` and `12`, then
build and test in `ros-dev` (arm64, the Orin Nano's architecture). Nothing here has touched
hardware; the car is still in pieces (`docs/notes/build-status-2026-09-14.md`).

Read-only context, not edited: `firmware/safety_mux/`, `tools/jetson_heartbeat/`.

---

## Findings

| # | Sev | Where | What happens on the car | Fix / follow-up |
|---|---|---|---|---|
| 1 | **bug** | `racer_drivers/src/pwm_output_node.cpp:239,246` (pre-fix) | `/drive` staleness measured on the node's **ROS clock** -- the exact defect the 2026-09-13 audit removed from `safety_node`, `tracker_node` and `twist_teleop_adapter_node` (issue #22), reintroduced in the one node where it matters most: this timeout is the LAST thing that takes a driving pulse off the wire. With `use_sim_time:=false` a backwards NTP/VM step makes the age negative, which `is_stale`'s `age_s >= timeout_s` reads as FRESH, so a dead `/drive` publisher leaves the last commanded throttle pulse on the VESC for the length of the step. With `use_sim_time:=true` and no `/clock` the age is pinned at 0.0 and the timeout **never fires at all**. | **FIXED**: measured on `RCL_STEADY_TIME`. New L3 `test/test_pwm_output_node_clock_launch.py`, which fails against the pre-fix source. |
| 2 | **bug** | `racer_drivers/src/pwm_mapping.cpp:87-95` (pre-fix) | `is_stale` did not fail closed on a **negative** age: it returned `age_s >= timeout_s`, so a backwards clock read as fresh. The committed table-driven test asserted that behaviour explicitly (`"negative age (clock went backwards) is not stale"`). `racer_safety`'s watchdog and `racer_tools`' `should_use_zero_command` both fail closed here; this one did not. | **FIXED**: negative age is stale. Test case inverted plus two more (large negative, negative epsilon). Defence in depth behind finding #1's real fix. |
| 3 | **bug** | `racer_drivers/src/pwm_output_node.cpp:75-92` (pre-fix) | All four chip/channel parameters defaulted to `0`, so `ros2 run racer_drivers pwm_output_node` with no arguments, or a launch that set only some of them, aimed **both pulses at `pwmchip0/pwm0`**. The throttle write then overwrites the steering write 50 times a second and one output physically does not exist. On the car that presents as a dead servo, i.e. as a wiring fault. | **FIXED**: new pure `validate_channel_assignment` refuses at construction naming the collision; `throttle_pwm_channel`'s default moved to `1` to match `car_teleop.launch.py`, so the all-defaults case is no longer the broken one. 4 new L1 cases. |
| 4 | **design bug** | `racer_safety/src/gate_logic.cpp:149,161` (pre-fix) | The watchdog and command-sanity gates wrote `DriveCommand{0, 0}` -- **snapping the steering to centre in one cycle**, from the code path whose job is to make the car safer. That is a step input of up to full lock that bypasses the steering rate limiter three lines below it, and mid-corner it straightens a car that is still travelling on its arc and sends it to the outside of the turn. The TTC gate had always done the opposite (zero the speed, leave the steering). Two rules for the same situation. | **FIXED**: one rule. Every zero-throttle gate now holds `previous_output.steering_angle_rad`, via a shared `zero_throttle_command()` that `safety_node`'s fail-closed exception path also uses. 6 new L1 cases; all fail against the pre-fix source. From `{0,0}` the output is still exactly `{0,0}`, so the runbook's "verify /drive is neutral" step is unaffected. |
| 5 | **semantic trap** | `gate_logic.hpp` / `safety_node.cpp` / `pwm_mapping.cpp`, all of it | "Brake" in layer 3 meant `speed = 0`, which `pwm_output_node` maps to `throttle_pwm_neutral_us`, which a VESC in PPM mode reads as **zero current, i.e. a COAST**. "Watchdog fired, braking" produced a car that keeps rolling at speed. Nothing in the code said so. | **FIXED as documentation + a rename**, not as a behaviour change (there is no other lever in software). See "Decision 1" below. `GateResult::brake` is now `GateResult::zero_throttle`; the physical meaning is written at the top of `gate_logic.hpp`, `safety_node.cpp` and the runbook's new VESC step. |
| 6 | **risk** | `racer_tools/keymap.py`, `twist_teleop.py` | Both teleop sources clamped the speed floor at `limits.min_velocity_mps` (-5.0 m/s), so the throttle-down key or a backwards Twist produced a **below-neutral pulse on a car whose VESC PPM control type has never been set**. Below neutral is reverse current in `Current` mode and proportional braking in `Current No Reverse With Brake`; nobody knows which this ESC will be in. | **FIXED**: new `allow_reverse` declared parameter on both nodes, default `false`, clamping the floor at 0.0 m/s. Launch argument on `car_teleop.launch.py`. See "Decision 2". 7 new L1 cases. |
| 7 | smell | `racer_drivers/src/pwm_output_node.cpp:247-248` (pre-fix) | `const PulsePair pulses = driver_->update(state); static_cast<void>(pulses);` -- a value computed and explicitly discarded in the 50 Hz path. | **FIXED**: `driver_->update(state);`. |
| 8 | **gap (not fixed)** | `racer_safety/src/safety_node.cpp:246-248, 320` | `min_scan_range_m_` is **latched forever**. If the LiDAR dies, the last computed range persists for the life of the node: die with a far return and the TTC gate is silently disabled; die with a near return and it brakes forever. There is no `/scan` staleness watchdog, and `/scan` is the only sensor input this node has. | Follow-up, not fixed here. Which way it should degrade (brake? derate? event only?) is a safety-layer decision and a human call, and there is no LiDAR fitted, so nothing is being made worse today. Filed in the follow-ups list below. |
| 9 | gap | `racer_bringup/launch/car_teleop.launch.py` | `start_teleop` and `browser_teleop` can both be `true`, giving **two publishers on `/drive_raw`** with nothing arbitrating. The docstring says do not; nothing enforces it. | Recorded only. Enforcing it means an `OpaqueFunction` that inspects both configurations at launch time; worth doing, not worth doing inside this review. |
| 10 | note | `racer_safety/src/safety_node.cpp:352` | `gate_limits_` is a second copy of the `SafetyLimits` already inside `gate_`, kept only so the startup log line can read it. Harmless; not worth a change on a safety file. | None. |
| 11 | note | whole path | Timeout layering is coherent and worth writing down, because it looks like duplication and is not: `safety_node` watchdog 0.06 s (3 cycles) < `pwm_output_node` `drive_timeout_s` 0.1 s < `twist_timeout_s` 0.5 s, and `limits.mux_watchdog_timeout_s` 0.1 s is a different mechanism on a different MCU. The inner timeout always fires first, so the normal failure is "safety_node commands zero", not "the driver node gives up". | None. |
| 12 | note | `firmware/safety_mux/logic/src/pwm_validity.c` (read-only) | Checked as instructed: `pwm_is_valid_us` is an **inclusive** `[min_us, max_us]` range check fed from the same `config/vehicle_params.yaml` fields the map's ends come from, so the full 1000-2000 us span the mapping can emit is accepted, saturated ends included. No mismatch. | None. New gtest pins the span. |

### Compatibility sweep (nothing found)

- **arm64.** No `-m64`, `-march`, arch conditionals or arch-pinned wheels anywhere in scope; a
  clean build and green tests on arm64 in `ros-dev` is the evidence.
- **JetPack 6 / kernel paths.** The only `/sys` path is `/sys/class/pwm`, and it is a declared
  parameter with the real path as its default. `pwmchip` numbering is already documented as
  read-off-the-device (runbook step 4).
- **ROS 2 Humble APIs.** `rclcpp::Clock{RCL_STEADY_TIME}`, `create_wall_timer`,
  `declare_parameter` with `ParameterDescriptor` ranges, `RCLCPP_ERROR_THROTTLE`,
  `rclpy.QoSProfile` -- all current in Humble, nothing deprecated in scope.
- **Python 3.10.** `from __future__ import annotations` everywhere the `X | None` syntax is
  used; no `match`, no 3.11+ typing. Runs on the Jetson's 3.10.
- **Allocation in the 50 Hz path.** `SysfsPwmChannel::write_duty_ns` uses a stack buffer and a
  held fd. `GateResult::activations` is a `std::vector` that only grows when a gate actually
  fires; already documented as accepted. Nothing new added.
- **Logging.** No `INFO` in any 50 Hz loop; the one per-cycle error path is
  `RCLCPP_ERROR_THROTTLE`.

---

## Decision 1: what "brake" means, and what the steering does while it happens

**The honest statement, now written into the code:** layer 3 has exactly one lever, the
`/drive` `speed` field, and every gate that fires writes `0.0` into it. Downstream that is
`actuation.throttle_pwm_neutral_us`, and a neutral PPM pulse is zero current -- a **coast**.
A moving car whose watchdog trips keeps rolling and slows by drag. Layer 3 cannot decelerate
this car and no wording in this repo may imply it can.

Three things were done about it, none of which pretends to be a brake:

1. **Named it.** `GateResult::brake` -> `GateResult::zero_throttle`. The physical explanation
   sits at the top of `gate_logic.hpp` (on the struct), at the top of `safety_node.cpp`, and
   in the runbook. `EventSeverity::kBrake` and `racer_msgs`' `SEVERITY_BRAKE` keep their
   names deliberately: they are a published interface and an evaluation metric
   (`claude-docs/09`), and renaming them would break bag compatibility to fix a comment. They
   now mean "this gate commanded zero", with the caveat written next to them.
2. **Pushed the real braking to where it can actually happen**: the VESC's PPM control type,
   layer 2, now a numbered runbook step (13) with a recommendation and a reason, and an
   instruction to export and commit the resulting configuration. No VESC numbers are invented
   -- deadband, current limits and ramping are unmeasured and stay unmentioned.
3. **Fixed the steering.** See below.

**Steering on a zero-throttle gate: HOLD the last commanded angle, do not centre it.**
Reasoning, in the order it weighs:

- Centring is a step input. The steering rate limiter exists because the rack cannot be slewed
  arbitrarily fast and because a step in road-wheel angle at speed is a yaw disturbance.
  Writing `0.0` straight into the output bypasses that limiter and commands up to full lock of
  travel in one cycle -- from the one code path whose entire job is to make the car safer.
- Mid-corner, centring straightens a cornering car. On a watchdog trip the car is still
  travelling along its arc, and layer 3 cannot stop it (it is a coast). Holding the arc keeps
  it roughly where it was going; straightening sends it to the outside of the corner, which on
  a track is the wall.
- Consistency. The TTC gate had always zeroed the speed and left the steering alone. The
  watchdog and sanity paths did the opposite. One rule is one fewer thing to get wrong, and
  `safety_node`'s fail-closed exception path now calls the same function the gates do, so the
  two can never drift.

The argument on the other side is that holding a full-lock command forever after the publisher
dies is not obviously better than centring. It is accepted: the speed is zero, the car is
coasting to a stop, and whatever the front wheels are doing is a smaller effect than the step
input would have been. If the bench says otherwise, this is one function
(`zero_throttle_command`) and six tests to change.

From a `{0, 0}` previous output -- a node that has never commanded anything -- the output is
still exactly `{0, 0}`, so runbook step 9 and its L3 test are untouched.

## Decision 2: reverse is off by default

`limits.min_velocity_mps` is -5.0 m/s (an `f1tenth_gym` value, like everything else in that
file), and both teleop sources clamped to it, so `s`/DOWN or a backwards Twist produced a
below-neutral pulse.

**What a below-neutral pulse does is not a property of this repo.** It is the VESC's PPM
control type: reverse drive current in `Current`, proportional braking in `Current No Reverse
With Brake`, nothing at all in the `No Reverse` variants. That setting has never been applied
to this ESC. Emitting a pulse whose meaning nobody knows, on a car nobody has driven, is not a
defensible first drive.

So: a declared `allow_reverse` parameter on **both** teleop nodes, default `false`, which
clamps the speed floor at 0.0 m/s instead of `limits.min_velocity_mps`. Both, not just the
keyboard: `browser_teleop` is the one a car launch file can actually start, so disabling
reverse on the keyboard alone would have disabled nothing. A launch argument surfaces it on
`car_teleop.launch.py`, and the runbook's teleop step says to leave it off until step 13 is
done and recorded.

What was deliberately NOT changed:

- **`limits.min_velocity_mps` stays -5.0.** It is a vehicle parameter, not a teleop policy;
  the sim and the dynamics model use it. A ROS parameter is the right place for a policy that
  changes between "first bench session" and "later".
- **`safety_node` still accepts negative speed** down to that limit. It is a gate, not a
  command source, and it must not be the thing that decides whether reverse exists -- a future
  `tracker_node` or policy could legitimately command it.
- **`pwm_mapping` still maps negative speed below neutral.** The driver is not where reverse
  gets disabled; a new gtest pins that separation so a later reader does not "helpfully"
  duplicate the clamp there.
- **The TTC gate's `speed <= 0 -> no TTC brake` branch is correct either way** and was left
  alone. If negative means reverse, the car is not moving toward the obstacle in front. If
  negative means braking, the car is decelerating and TTC braking it is redundant. It only
  becomes wrong when the car is coasting FORWARD under a zero or negative command, which is
  the already-documented "no `/odom`, so forward speed is the commanded speed" limitation
  (`config/vehicle_params.yaml`, `limits.ttc_*`), not something reverse introduces.

## Decision 3: the steering sign convention, traced end to end

`claude-docs/06-vehicle-params.md`: road-wheel angle in radians, **LEFT POSITIVE**. Every hop:

| Hop | Where | Behaviour | Pinned by |
|---|---|---|---|
| Key -> angle | `keymap.py:133-140` | `a`/`A`/`LEFT` **increases** `steering_angle_rad`; `d`/`D`/`RIGHT` decreases it | `test_keymap.py::test_steer_left_keys_increase_steering_angle_left_positive` (existing) |
| Twist -> angle | `twist_teleop.py:102-106` | `atan(L * angular_z / speed)`; `angular_z > 0` (CCW, left) with forward speed gives a **positive** angle; plain `atan` keeps the sign flip when reversing | `test_twist_teleop.py::test_positive_angular_z_with_forward_speed_is_left_positive_steering` (existing) |
| Gate | `gate_logic.cpp` | Pure pass-through for sign. Clamps against a symmetric `[-0.4189, +0.4189]`, never inverts | **NEW** `test_gate_logic.cpp::SteeringSign` group (3 cases) |
| Zero-throttle gate | `gate_logic.cpp` | Holds the sign of the previous output rather than centring | **NEW** `ZeroThrottleSteering` group (2 of its 6 cases cover both signs) |
| Angle -> pulse | `pwm_mapping.cpp:97-117` | Positive interpolates from `pwm_neutral_us` toward `pwm_max_us` when `left_is_pwm_max`, toward `pwm_min_us` when not | `TableDrivenLeftIsPwmMax` / `LeftIsPwmMin` (existing) + **NEW** `SteeringSign` group (2 cases) |
| Pulse -> servo | the servo | **UNKNOWN. This is the one unpinned hop.** | Runbook step 12, now the FIRST bench calibration |

The chain is internally consistent and every software hop is now pinned by a test that names
the convention. The single remaining unknown is `steering_left_is_pwm_max`, whose default
(`true`) is **a guess** -- no project doc defines which pulse end is full left and nobody has
put a scope on this servo. Runbook step 12 was moved ahead of powering the ESC (it needs only
the servo, and there is no reason for the motor to be live while the polarity is still
unknown) and relabelled as the first bench calibration, with the trace above summarised in it.

---

## Subject to change when the car is assembled

Every line below is provisional and must be revisited against a bench measurement. This is the
list for THIS code; `config/vehicle_params.yaml`'s own header carries the file-level version.

**Measured at the bench, then written into `config/vehicle_params.yaml`:**

| Value | Where | Replaced by |
|---|---|---|
| `steering.pwm_min_us` / `pwm_max_us` / `pwm_neutral_us` = 1000/2000/1500 | `vehicle_params.yaml:100-102` | Runbook steps 5, 10, 12 (scope on the pin, then rack-end check) |
| `actuation.throttle_pwm_min_us` / `max_us` / `neutral_us` = 1000/2000/1500 | `vehicle_params.yaml:115-117` | Runbook steps 10 and 14. **Neutral is the one that bites**: not-1500 means the cut state is a creep |
| `actuation.throttle_full_scale_mps` = 5.0 | `vehicle_params.yaml:136` | Runbook step 15.6 (sweep, wheel speed vs pulse width). Disappears entirely when `vesc_node` exists |
| `limits.global_speed_cap_mps` = 20.0 | `vehicle_params.yaml:153` | Not a validated safety cap: it is `f1tenth_gym`'s model-validity bound. Must be lowered before any floor session |
| `limits.min_velocity_mps` = -5.0 | `vehicle_params.yaml:155` | Same provenance. Only reaches the wire if `allow_reverse` is turned on |
| `limits.ttc_warning_s` = 1.0, `ttc_brake_s` = 0.5 | `vehicle_params.yaml:173-174` | Phase 1/2 braking-distance data. Untuned, and see finding #8 |
| `actuation.max_acceleration_mps2` = 9.51 | `vehicle_params.yaml:107` | `f1tenth_gym` `a_max`. It sets BOTH the gate's speed rate limit and the teleop speed step |
| `steering.min_rate_rad_per_s` / `max_rate_rad_per_s` = -3.2/3.2 | `vehicle_params.yaml:91-92` | `f1tenth_gym` `sv_min`/`sv_max`, not this servo. Sets the gate's steering rate limit and the teleop steering step |
| `steering.min_angle_rad` / `max_angle_rad` = -0.4189/0.4189 | `vehicle_params.yaml:89-90` | `f1tenth_gym` `s_min`/`s_max`. Runbook step 12's rack-end check is the first evidence either way |

**Declared ROS parameters whose defaults are guesses:**

| Parameter | Node | Default | Replaced by |
|---|---|---|---|
| `steering_left_is_pwm_max` | `pwm_output_node` | `true` | Runbook step 12. **The single unpinned hop in the sign chain.** Change the launch default once measured |
| `steering_pwmchip` / `steering_pwm_channel` | `pwm_output_node` | 0 / 0 | Runbook step 4.3, read off the device after the pinmux change and a reboot |
| `throttle_pwmchip` / `throttle_pwm_channel` | `pwm_output_node` | 0 / 1 | Same. The `1` is only there so the all-defaults case is two distinct channels, not because channel 1 is known to be right |
| `allow_reverse` | both teleop nodes | `false` | Runbook step 13 (the VESC PPM control type) decides whether turning it on is safe |
| `drive_timeout_s` | `pwm_output_node` | 0.1 s | Five missed frames at 50 Hz, a cadence not a measurement. Revisit against measured `/drive` jitter on the Orin (roadmap 2.7) |
| `watchdog_missed_cycles` | `safety_node` | 3 | `claude-docs/04`'s number. Same caveat |
| `twist_timeout_s` | `twist_teleop_adapter_node` | 0.5 s | Browser round-trip latency on the real network |
| `output_rate_hz` / `control_rate_hz` | both | 50 Hz | The servo/ESC convention. Confirmed by scope at runbook step 5, not before |

**Behaviour that is a decision, not a measurement, and should be re-examined on the bench:**

- **The coast.** Runbook step 15.5 now says to time the spin-down after releasing the key.
  That number is the first real evidence of what a watchdog trip does on this car, and it is
  what decides whether the VESC control type of decision 1 is enough.
- **Holding the steering on a zero-throttle gate** (decision 1). One function, six tests.
- **`safety_node`'s TTC forward speed is its own commanded speed**, because there is no
  `/odom`. Already documented in `vehicle_params.yaml`; stands until roadmap 2.6/2.7.
- **`min_scan_range_m_` latches** with no `/scan` watchdog (finding #8).

## Follow-ups filed, not fixed here

1. **`/scan` staleness watchdog in `safety_node`** (finding #8). Needs a decision about which
   way to degrade, which is a human call on a safety layer.
2. **Two-teleop-source guard in `car_teleop.launch.py`** (finding #9).
3. **L3 flakiness on a 4-CPU colima VM.** `racer_safety`'s `test_safety_node_launch.py` fails
   5 of its timing-sensitive assertions when `colcon test` runs every package's launch tests
   concurrently on this machine -- **reproduced on unmodified `main` before any change in this
   review** (283 tests, 9 failures), and green when the same file is run on its own or with
   `--parallel-workers 1`. Domain IDs are already distinct per test file, so this is CPU
   starvation, not cross-talk: the control loop runs at ~10 Hz instead of 50 and the
   watchdog trips inside assertions that assume it does not. `claude-docs/12-testing.md` says a
   flaky test is quarantined within a day with an issue filed; this one deserves the issue,
   and probably a `_spin_for` that waits on a condition rather than a wall-clock duration.
