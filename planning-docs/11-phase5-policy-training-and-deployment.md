# Stage 11: Phase 5, residual policy training and deployment (roadmap 5.1 to 5.4)

Time: 3 to 6 weeks. The training pipeline, envelope, and deployment contract already exist
and are tested; this phase runs them against the calibrated simulator and the real car.

## Steps

1. Train (5.1) with `training/racer_train` in the sim calibrated by Phase 3, with the
   randomization ranges from 3.4 and the speed envelope from the fidelity curve. SAC, reward
   equal to progress minus crash minus envelope violation, nothing else; any shaping term
   added is logged in `docs/notes/reward-confessions.md`. Long runs on Desktop A's GPU if that
   machine is in play, otherwise on the Mac's CPU with smaller budgets. Every reported policy
   is reproducible from config, git SHA, seed, and `vehicle_params` version.
2. Deploy (5.2): `policy_node` (the rclpy wiring around the existing contract loader) loads
   the policy directory through the contract and refuses on any mismatch. First runs are
   zero-shot, recorded and reported as a measurement, not a requirement. Inference at 50 Hz on
   the Jetson with the jitter histogram published.
3. Adaptation pass (5.3): the on-hardware calibration step (short fine-tuning or parameter
   adaptation as designed in `claude-docs/08-learning.md`), with before and after laps bagged.
4. Envelope live (5.4): force an out-of-distribution condition (for example an unexpected
   obstacle placement) and observe the fallback to the pure base controller with the
   `/safety/events` record. This is a track-level proof like the kill test; it needs the
   two-person rule.

## Done when

A policy directory that loads through the contract, drives the car within the envelope, and
whose fallback has been demonstrated live. Zero-shot and adapted numbers both recorded.
