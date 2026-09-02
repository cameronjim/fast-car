# Stage 7: first drives, teleop, and logging (roadmap 1.5, 1.6, gate G1 sign-off)

Time: 3 to 5 hours. Two-person rule throughout. This is where the software that has been
driving the simulator meets the real car.

## Steps

1. VESC driver node: bring up `racer_drivers/vesc_node` (adopt the F1TENTH `vesc_driver` for
   ROS 2 Humble rather than writing one) inside the car image, talking to the VESC over USB in
   current mode. Confirm `ros2 topic echo` shows VESC telemetry including input voltage; this
   becomes the rail log until the INA226 is installed.
2. Command path, wheels off the ground: launch `safety_node` and `vesc_node` on the Jetson,
   then `keyboard_teleop_node` from the Mac over SSH (it needs a terminal). Confirm the full
   chain from `claude-docs/04-architecture.md`: keyboard to `/drive_raw` to `safety_node` to
   `/drive` to `vesc_node` to the VESC, with the mux physically downstream and able to cut it.
   Only `safety_node` publishes `/drive`; verify with `ros2 topic info`.
3. Wheels-off actuation sweep (L6): fill in a real copy of
   `tests/bench/procedures/template_wheels_off_actuation.yaml`. Commands in, measured response
   out: steering left is positive, throttle forward is positive, a zero command stops the
   wheels, the safety node's watchdog brakes when teleop stops publishing. Commit the record.
4. RC teleop through the mux: with the mux in the loop, drive the wheels-off car from the RC
   transmitter's sticks (the mux passes RC through when the Jetson path is idle only if the
   firmware is configured that way; otherwise RC remains kill-only and teleop is by keyboard.
   Decide which and record it; the roadmap's "teleop via RC through the mux" is satisfied by
   either as long as the kill switch is proven).
5. Logging: launch the rosbag recorder so every drive records all topics plus the rail voltage
   topic, under `data/bags/<date>_<session>_<purpose>/`. Do a wheels-off drive and confirm the
   bag opens and contains the rail trace. This is task 1.6 and is verified before the gate.
6. Kill test again, now with the real driving software running (stage 5 step 17 procedure).
7. Wheels ON the ground, for the first time: a large clear indoor space, cones as boundaries,
   one person on the kill switch, one on the keyboard. Speed capped by the VESC limits from
   stage 6. Drive slowly for five minutes. Then kill it deliberately once, mid-drive, from the
   transmitter. Bag everything.
8. Weigh the finished car and update `chassis.mass_kg`. Photograph the finished harness.
9. Write `docs/notes/phase1-summary.md`: what was measured, what deviated from the plan, the
   observed kill latency, and the session bag names.

## Done when

The car drives by keyboard through the real command path, RC kill works from the driver's
hand, every drive is bagged with rail voltage, and the kill test has passed with real software.
Tick 1.5, 1.6, and record gate G1 as passed with a date in `claude-docs/01-roadmap.md`.

## Commit

Bench records, bags are not committed (they live in `data/`), `docs/notes/phase1-summary.md`,
roadmap ticks, photos.
