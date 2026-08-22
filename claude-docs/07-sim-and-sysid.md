# 07 — Simulator, Vehicle Model, System ID

## Model upgrades over stock f1tenth_gym (sim/racer_gym/)

Required, in this order:

1. Longitudinal and lateral load transfer
2. Pacejka-style tire curves, separate front/rear parameters
3. Explicit first-order actuator dynamics for steering and throttle (measured time constants
   from `vehicle_params`)
4. Measured command-to-torque delay as a fixed transport lag

The regime table in `00-project-overview.md` governs what fidelity is claimed where. The
saturated/sliding regime is NOT modeled well and never will be by this project; the policy
envelope bounds it out instead.

## System ID protocol (sysid/batteries/)

Full battery (Phase 3, on the venue surface — parameters do not transfer across surfaces):

- Step responses: throttle and steering (→ time constants, delays)
- Constant-radius circles at increasing speed (→ friction limit, understeer gradient)
- Coastdown (→ drag)
- Figure-eights (→ combined slip; **held out for validation, never used in fitting**)

Fitting rules (sysid/fitting/):

- Report validation error on held-out maneuvers, not training-fit residuals.
- Randomization ranges for training are set from fit residuals AND the drift record, never
  intuition.
- Fitted values are written to `vehicle_params.yaml` with session id in `meta`.

## The re-ID battery (every session, ~10 min)

Two step responses, one constant-radius sweep, one coastdown. Scripted, one command
(`sysid/batteries/reid.py`). Purposes:

- Drift record: per-session parameter estimates committed to `sysid/drift/` (a deliverable)
- Session gate: parameters outside expected drift bands → investigate before driving
- First diagnostic: any "policy got worse" report starts here — did the CAR change?

## Headline fidelity deliverable

A curve: sim-vs-real trajectory error as a function of proximity to the friction limit.
This bounds where the policy can be trusted, feeds the layer-4 speed envelope, and is more
credible than any single "matches within X" number. Version it (v1 after Phase 3, final
with evaluation data).

## Failure diagnosis checklist (when hardware underperforms sim)

Instrumented to distinguish BEFORE experiments run; replay from rosbags, never re-drive to
debug. In rough likelihood order:

1. Rail voltage log (cheapest to rule out — check first, every time)
2. State estimator error (most likely, most overlooked)
3. Observation mismatch: LiDAR beam count/FOV/downsampling vs. training, dropouts, reflective
   surfaces
4. Timing/sync drift (check per-hop timestamps against sync design doc)
5. Actuator model error (compare step responses sim vs. logged)
6. Vehicle changed (re-ID drift record)
7. Tire/contact model structurally inadequate for the regime driven
8. Reward hacking: fast in sim for the wrong reason
9. Thermal derating over the session (motor/ESC temps are logged)
10. Policy generalization failure (last, not first, conclusion)
