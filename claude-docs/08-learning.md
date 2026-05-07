# 08 — Learning: Residual Policy, Envelope, Deployment Contract

## Structure

- **Base controller = the tuned classical stack** (raceline optimizer + tracker from
  racer_control). The same artifact is the evaluation baseline. This is the project's
  central structural decision — see `00-project-overview.md`.
- The policy outputs a bounded RESIDUAL on the base controller's command, never a full
  command.
- Transfer strategy: sim-trained + on-hardware calibration/adaptation (primary), zero-shot
  measured and reported before adaptation (it quantifies what the sim fidelity work bought),
  on-board fine-tuning (stretch, out of scope until asked).

## Boring choices, committed

- Algorithm: SAC (fallback PPO). No algorithm shopping without a logged reason.
- Observations: low-dimensional state (from EKF/localizer: velocity, yaw rate, slip proxy,
  raceline-relative pose) + downsampled LiDAR. Exact schema lives in the contract.
- Reward: progress along raceline − crash − envelope violation. **Nothing else.** Every
  shaping term added later (including action-rate penalties) is logged in
  `docs/notes/reward-confessions.md` and treated as evidence of a modeling defect to chase.
  Order of attack for oscillation: model actuator dynamics properly → hard rate constraints
  in the env matching the physical actuator → only then a penalty.
- Domain randomization: around fitted parameters, ranges from fit residuals + drift record.

## Envelope (envelope/ library — shared train/deploy, never forked)

Hard residual bounds (fraction of base command range), rate limits, OOD fallback to pure
base controller, speed envelope from the fidelity curve. Enforced in the deployment node
AND inside the training env. Details in `05-safety.md`.

## Deployment contract (racer_policy loader)

A deployable policy is a directory containing ALL of:

- `policy.pt` + checksum
- Observation schema: field order, dtypes, **units**, LiDAR beam count/FOV/downsampling
- Normalization statistics from training time
- Action space: bounds, units, scaling, residual limits
- Actuator assumptions trained against: rate limits, delays
- `vehicle_params` version + sysid session id trained against
- Training config hash + git SHA
- `contract_version`

The deploy node **refuses to load** on any mismatch — schema version, params version,
checksum, obs schema vs. live topics. Refusal is a hard exit with a specific error, never a
warning. Do not add override flags.

## "Held-out track" semantics (for evaluation)

The raceline optimizer and tracker GET the held-out track's map — that is their job, not
cheating. The residual's WEIGHTS ARE FROZEN. The test is whether the learned component
generalizes across tracks, not whether it memorized one.

## Training operations

- Long runs on Desktop A (`train-cuda`). Checkpoints + configs stored under `data/`,
  tracked by config hash; every reported policy is reproducible from config + SHA + seed.
- Inference target: 50 Hz on the Jetson. Profile PyTorch, then ONNX Runtime, with the real
  model. TensorRT only if profiling says so (it probably won't for a small residual MLP).
- Publish the inference jitter histogram; port policy_node to C++ only if tail latency is bad.
