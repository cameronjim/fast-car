# Hardware-arrival checklist (milestone 4, roadmap tasks 1.1-1.6)

Practical, ordered checklist for the day the BOM (`claude-docs/11-hardware.md`) actually
arrives. Written ahead of the hardware (milestone 4, `docker/train-cuda/`-style draft) so the
owner assembles and verifies instead of waiting on code. Each step names which roadmap task
it belongs to, what gets measured, and where the result gets committed. Nothing on this list
has been executed -- correct it against reality as each step actually happens, and check off
the corresponding roadmap task in `claude-docs/01-roadmap.md` (with a dated note) as its gate
is met, not before.

Two-person rule (`claude-docs/05-safety.md`): every step below that can move the car needs a
driver on the RC kill switch and a separate operator. Do not run any powered step alone.

## 0. Before opening a single box

- [ ] Re-read `claude-docs/05-safety.md` end to end. This is not optional preamble --
      Phase 1's whole point is standing up layer 1 correctly.
- [ ] Confirm `main` branch CI is green (`gh run list --branch main --limit 1`). Hardware work
      building on a broken `main` compounds the wrong problem.
- [ ] Bench safe-bag and charger ready for the 3S packs (`claude-docs/11-hardware.md` BOM) --
      LiPo handling discipline starts before the first pack is charged, not before the first
      drive.

## 1. Assembly (roadmap task 1.1)

- [ ] Assemble the Slash 4x4 roller, motor, ESC (VESC), and servo per the chassis decision in
      `claude-docs/11-hardware.md`.
- [ ] Bench-test with **wheels off the ground** before anything else touches the drivetrain --
      this is the `claude-docs/12-testing.md` L6 discipline from day one, not a step added
      once code exists: confirm the motor spins and the servo moves under manual ESC/servo
      tester input, no Jetson or RP2040 involved yet.
- [ ] Measure and record into `config/vehicle_params.yaml` (regenerate bindings with
      `python3 tools/gen_params.py` after editing, per `claude-docs/06-vehicle-params.md`
      rule 3 -- never hand-write a binding):
      - `chassis.track_width_m` (not modeled by the gym single-track model, but recorded for
        completeness and later chassis-model upgrades)
      - `tires.nominal_radius_m` (measured, drift-tracked -- this is the first entry in
        `sysid/drift/`, see task 3.3)
- [ ] Photograph the assembled chassis and commit to `docs/notes/`.
- [ ] Mark roadmap 1.1 `[x]` in `claude-docs/01-roadmap.md` with a dated note once the bench
      wheels-off test above passes.

## 2. Power tree (roadmap task 1.2)

- [ ] Wire the power tree per `claude-docs/11-hardware.md`'s wiring rules: star ground at the
      buck, short ESC power leads with bulk capacitance at the VESC, every connector
      polarized/keyed.
- [ ] Wire rail-voltage/current sensing on the compute rail. This is non-negotiable
      (`claude-docs/05-safety.md`: "Rail voltage logged on every run; a brownout must never be
      mistakable for a control failure") -- do not proceed to task 1.5 (rosbag logging)
      without it wired and readable.
- [ ] Bench-prove rail-voltage logging BEFORE any code drives the car: apply a known load,
      confirm the logged voltage trace tracks it. This is task 1.2's own done-criterion
      ("rail-voltage logging proven on bench"), independent of the mux or the Jetson.
- [ ] Mark roadmap 1.2 `[x]` once rail-voltage logging is proven on the bench.

## 3. Layer-1 safety mux (roadmap task 1.3) -- the gating step

`firmware/safety_mux/` (this milestone) is a DRAFT: host-tested decision logic, an
UNVERIFIED Pico SDK glue layer, and a proposed pinout. Read
`firmware/safety_mux/README.md` in full before wiring anything.

- [ ] Bench-measure and fill in `config/vehicle_params.yaml`'s new (currently `null`) fields
      the mux firmware needs, then regenerate bindings:
      - `steering.pwm_min_us` / `pwm_max_us` / `pwm_neutral_us` (servo channel)
      - `actuation.throttle_pwm_min_us` / `throttle_pwm_max_us` / `throttle_pwm_neutral_us`
        (ESC channel)
      - `limits.mux_watchdog_timeout_s` (start conservative -- e.g. a few hundred ms -- and
        tighten only after the kill test below proves the mux actually cuts within budget)
      - `limits.mux_kill_switch_threshold_us` (the RC receiver's kill-switch channel
        ARMED/KILL threshold; also bench-measure that channel's own valid PWM range, which is
        NOT a vehicle_params field -- see `firmware/safety_mux/README.md`'s pinout table)
- [ ] Wire the RP2040 per `firmware/safety_mux/README.md`'s proposed pinout table. Confirm
      the mux MCU and RC receiver are powered from a rail a Jetson/compute-rail failure
      cannot take down (`claude-docs/11-hardware.md` wiring rules) -- this is a wiring
      decision, nothing in firmware enforces it. The 2026-08 BOM audit adds specifics: the
      mux Pico gets its own 5V UBEC off the traction pack (never the Jetson's supply), and
      every 5V PWM line into the Pico (FS-iA6B receiver channels, and the Jetson-side PWM
      if 5V) goes through the bidirectional level shifter, because RP2040 GPIO is 3.3V-only.
      Bench-verify shifted signal integrity on a scope before the first wheels-off test.
- [ ] Repin the motor sensor cable: Hobbywing sensored motors and VESC use different 6-pin
      JST-PH sensor pinouts. Identify both pinouts (datasheet or probing), rewire, then
      verify hall order in VESC Tool's motor detection before first spin.
- [ ] Bench-sweep the 12V buck-boost on a lab supply from 9.0V to 12.6V and confirm the
      output holds 12V across the whole range (a 3S pack crosses the output voltage as it
      drains; a buck-only unit fails this test and cannot be used for the Jetson rail).
- [ ] Get an arm-none-eabi-gcc + Pico SDK toolchain on Desktop B (`claude-docs/03-
      environments.md`) and build `firmware/safety_mux/` for real:
      `cmake -S firmware/safety_mux -B firmware/safety_mux/build && cmake --build
      firmware/safety_mux/build`. Fix whatever the first real build breaks on --
      `firmware/safety_mux/pico/` has never been compiled.
- [ ] Flash the RP2040 and confirm it does NOT arm (fast fault-LED blink) if any of the
      fields above are still `null` -- this is the "fails loudly on nulls" behavior
      `pico/main.c` implements; prove it fails loudly before proving it works.
- [ ] Per-channel bench test with wheels off the ground (`claude-docs/12-testing.md` L6,
      extend `tests/bench/procedures/template_wheels_off_actuation.yaml` into a real,
      filled-in procedure for this session -- copy it, don't edit the template in place):
      - RC kill switch in KILL position -> confirm both PWM outputs go to their neutral value
        and the power-cutoff GPIO de-asserts, regardless of what the Jetson is commanding.
      - RC kill switch in ARMED position, Jetson sending valid commands -> confirm passthrough
        (servo/ESC respond to the Jetson's commands, correct direction/sign per
        `claude-docs/06-vehicle-params.md`'s conventions).
      - Stop the Jetson's heartbeat toggle (kill the process driving it, not the whole
        Jetson yet) -> confirm the mux cuts within `mux_watchdog_timeout_s`.
      - Feed an out-of-range PWM value into the steering/throttle input (signal generator or
        a deliberately bad test harness) -> confirm the mux cuts rather than passing it
        through.
      - Rail brownout drill (bench PSU, sag the rail) -> confirm the mux MCU stays alive
        (`claude-docs/12-testing.md` L6) -- it must not share the Jetson's power failure mode.
      - Write the filled-in procedure's `SessionRecord` (via `tests/bench/racer_bench`) to
        `data/bags/` or `docs/notes/` per this session's own naming, so the result is
        committed, not just observed.
- [ ] **The actual gate: the G1 kill test.** With the car assembled, wheels off the ground,
      RC kill switch ARMED, and the Jetson actually driving the car (or commanding it) --
      **physically freeze the Jetson** (e.g. `sudo systemctl stop` every racer process and
      confirm no heartbeat toggling continues, or hard-power-cycle it while the mux stays
      powered from its independent rail) and confirm the mux cuts drive/steering PWM and
      asserts the power cutoff. A human must be present and this must be observed directly,
      not inferred (`claude-docs/05-safety.md`: "Kill is tested by ACTUALLY freezing the
      Jetson ... not by reasoning about it"; `claude-docs/12-testing.md` L7).
- [ ] Only after the G1 kill test passes: mark roadmap 1.3 `[x]` in
      `claude-docs/01-roadmap.md` with a dated note describing exactly how the Jetson was
      frozen and what was observed. **Gate G1 also gates everything below task 1.6** --
      `claude-docs/01-roadmap.md`: "GATE G1: teleop with layer-1 safety demonstrated, kill
      switch proven. Fail -> fix; nothing downstream matters." Practically: tasks 1.4-1.6
      below can be assembled and dry-run in parallel with the mux work above, but the car
      does not leave the bench under its own power until G1 passes.

## 4. VESC configuration (layer 2, feeds task 1.5 and `vehicle_params.limits`)

- [ ] Flash/configure the VESC with VESC Tool: current limits, speed limits, and any thermal
      derating settings appropriate to the motor (`claude-docs/11-hardware.md`).
- [ ] Export the VESC Tool configuration and commit it (`claude-docs/11-hardware.md`: "VESC
      Tool configuration is exported and committed after every change -- it IS safety layer
      2"). Re-export and re-commit after every subsequent change, byte-for-byte diffable --
      this is one of the `claude-docs/12-testing.md` L6 bench procedures ("VESC config diff:
      exported config matches the committed layer-2 config byte-for-byte before every
      session").
- [ ] Mirror the configured current/speed limits into `config/vehicle_params.yaml`'s
      `drivetrain.current_limit_a` / `brake_current_limit_a` fields (currently `null`) and
      regenerate bindings -- `claude-docs/06-vehicle-params.md`: these fields "mirror the VESC
      layer-2 config."
- [ ] Measure and record `drivetrain.gear_ratio`, `drivetrain.pole_pairs`,
      `drivetrain.motor_kv_rad_per_s_per_v` (convert from the datasheet's RPM/V to SI at this
      boundary, `CLAUDE.md` invariant 4) from the motor/drivetrain spec.

## 5. `car` image + Jetson bring-up (roadmap task 1.4)

- [ ] Flash the Jetson Orin Nano with JetPack 6.1 per NVIDIA's setup guide.
- [ ] Follow `docker/car/build_on_jetson.md` step by step -- it is a DRAFT, written before
      this hardware existed, and WILL need correcting on first real use. Fix it in place as
      you go; do not silently work around a wrong step without updating the doc.
- [ ] Confirm SSH access from the Mac to the Jetson (`claude-docs/03-environments.md`: "The
      Mac never talks to hardware directly ... it SSHes to the car").
- [ ] Once the image builds and a trivial `ros2` command runs inside it on real hardware, mark
      roadmap 1.4 `[x]` with a dated note (mirrors `docker/train-cuda/`'s `[~]` -> `[x]`
      pattern) and correct every "DRAFT"/"never verified" claim in `docker/car/README.md`.

## 6. VESC driver + teleop through the mux (roadmap task 1.5)

- [ ] Bring up `racer_drivers/vesc_node` (current-mode command path per
      `claude-docs/04-architecture.md`'s command path diagram).
- [ ] Confirm the full command path end to end, through the mux, per that same diagram:
      `keyboard_teleop_node -> /drive_raw -> safety_node -> /drive -> vesc_node -> ESC`, with
      the RC mux physically downstream of all of it and able to override it at any point.
- [ ] Wheels-off-ground actuation sweep (`claude-docs/12-testing.md` L6) after this real
      command path exists -- fill in a real, non-template version of
      `tests/bench/procedures/template_wheels_off_actuation.yaml`: sign/direction checks for
      steering (left-positive) and throttle (drive-positive), and confirm a brake/zero
      command actually stops the wheels.
- [ ] Only after that L6 sweep passes AND gate G1 (section 3) has passed, attempt teleop with
      wheels ON the ground, two-person rule in effect.

## 7. Rosbag + rail-voltage logging (roadmap task 1.6)

- [ ] Confirm every drive session is logged as a rosbag, including the rail-voltage topic
      from section 2 (`claude-docs/05-safety.md`: "Code paths that drive the car without
      logging are bugs").
- [ ] Verify this BEFORE gate G1's final sign-off, not after -- task 1.6 is listed before the
      gate for a reason.

## What gets measured, and where it's committed (summary)

| Measurement | Goes into | Committed via |
|---|---|---|
| Chassis dimensions, tire radius | `config/vehicle_params.yaml` (`chassis`, `tires`) | direct edit + `tools/gen_params.py` regeneration |
| PWM ranges/neutrals, watchdog timeout, kill-switch threshold | `config/vehicle_params.yaml` (`steering`, `actuation`, `limits`) | direct edit + regeneration |
| Drivetrain constants (gear ratio, pole pairs, Kv, current limits) | `config/vehicle_params.yaml` (`drivetrain`) | direct edit + regeneration |
| Servo pwm<->angle calibration table | `config/vehicle_params.yaml` (`steering.pwm_to_angle_table`) | direct edit + regeneration (Phase 2/3, not this milestone) |
| VESC Tool configuration | layer-2 safety config | exported file committed to the repo (location per `claude-docs/11-hardware.md`'s bench discipline) |
| Bench session results (L6 procedures, kill test observation) | session records | `tests/bench/racer_bench`'s `SessionRecord` JSON output, or `docs/notes/` for narrative writeups |
| Harness photos | physical documentation | `docs/notes/` |
| Wheel-radius drift over time | `sysid/drift/` | per-session drift record (Phase 3, ongoing after this milestone) |
