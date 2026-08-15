# tests

Cross-cutting test fixtures and harnesses that don't live inside a single
package. Per-package unit, property, and node tests (L1-L3) live alongside
their packages instead. See `claude-docs/12-testing.md`.

- `bags/`: curated small rosbags for L4 replay and golden tests (nominal
  segments, LiDAR dropout, stale-sensor, brownout). Empty until real bags
  arrive with hardware (roadmap task 2.8).
- `replay_harness/` (`racer_replay`): the L4 golden-comparison engine
  (stated per-field tolerances, explicit `--regenerate` flow) and the
  bag-mutation fault injectors (NaNs, timestamp jumps, dropped frames,
  out-of-order messages, frozen sensors), built over an abstract
  message-stream interface so they run today against synthetic in-memory
  streams and are meant to run unmodified against a rosbag2 reader once
  `bags/` has real content. Roadmap task 0.9.
- `sim_in_loop/` (`racer_sim_in_loop`): the L5 headless, seeded scenario
  runner and its regression assertions (lap completion, wall contact,
  lap-time band, trajectory-vs-reference tolerance), written against a
  small env/controller protocol rather than f1tenth_gym directly so the
  plumbing is unit-testable without a gym install. Roadmap task 0.9; S.2
  (tracker lap test) and S.6 (dynamics regression battery) build on this.
- `bench/` (`racer_bench`): the L6 executable-checklist engine for
  scripted, human-present bench/HIL procedures (YAML in
  `bench/procedures/`), each step either scripted (command + expected
  output) or human-confirm (yes/no prompt), producing a timestamped
  session record. `bench/procedures/template_wheels_off_actuation.yaml` is
  a placeholder template -- real procedures arrive with the hardware tasks
  that need them (Phase 1). Roadmap task 0.9.
- `sim_regression/` (`racer_sim_regression`): the S.6 sim dynamics
  regression battery -- a fixed, seeded set of sysid-style maneuvers
  (throttle step, steering step, constant-radius circle at a few speeds,
  coastdown) run through `sim/racer_gym`'s dynamics and checked against
  committed references (`racer_sim_regression/references/`) with
  `replay_harness`'s golden/tolerance engine. Runs in its own CI job on
  every push touching `sim/racer_gym/**` (see
  `.github/workflows/ci.yml`'s `sim-regression-battery` job and
  `.github/scripts/sim_regression_battery.sh`). Roadmap task S.6.
