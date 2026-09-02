# Stage 9: Phase 3, system identification (roadmap 3.1 to 3.5)

Time: 2 to 4 weeks. All of it happens on the venue surface; parameters do not transfer between
surfaces. Buy the venue tires (two identical sets) at the start of this stage.

## Steps

1. Full ID battery (3.1), scripted under `sysid/batteries/`: throttle and steering step
   responses, constant-radius circles at increasing speed until the friction limit shows,
   coastdowns, and figure-eights. Figure-eights are held out for validation and never used in
   fitting. Every run bagged, rail voltage included, tire set and ambient temperature noted.
2. Fitting (3.2) in `sysid/fitting/`: fit the upgraded gym model's parameters (Pacejka front
   and rear, actuator time constants, transport delay, drag) to the fitting maneuvers. Report
   validation error on the held-out figure-eights only. Write the fitted values into
   `vehicle_params.yaml` with the producing session id in `meta`.
3. Re-ID battery (3.3): the ten-minute version (two steps, one constant-radius sweep, one
   coastdown) as one command, `sysid/batteries/reid.py`. Run it at the start of every session
   from now on. Each run appends to `sysid/drift/`; parameters outside the expected band stop
   the session until explained.
4. Randomization ranges (3.4) for training set from fit residuals plus the drift record,
   written into `training/configs/`.
5. Fidelity curve v1 (3.5): sim versus real trajectory error as a function of proximity to the
   friction limit, from the battery data. This defines the speed envelope the policy is later
   allowed to command into. Version it in `docs/notes/`.

## Done when

Fitted parameters with validation error committed, the re-ID battery running every session
with a growing drift record, and the fidelity curve v1 plotted and committed.
