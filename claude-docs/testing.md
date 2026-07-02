# Testing

The bar for calling a change done depends on which side of the repo it touches.

## gym_training/

- Pure logic (frenet math, speed profiles, obs building, reward) gets plain pytest unit
  tests next to the code, runnable on any machine with the venv, no simulator needed.
- Env wrapper changes get a smoke test that steps the real env a few hundred steps and
  asserts shapes, bounds, and that reward/termination behave (progress positive when
  driving forward, terminated on collision).
- Training config changes are validated by a short real run: 10k steps, no crash,
  tensorboard scalars moving. Full training runs are experiments, not tests.
- The train/deploy contract is tested by round-trip: export a policy, rebuild the obs
  from `obs_config.json` the way the deploy node does, assert identical action output.

## ROS packages

- ROS is not installed on the Windows dev machine, so the gate there is `ast.parse` on
  every touched file plus the ament lint suite when built in WSL.
- Algorithm logic extracted out of node files (per CLAUDE.md rule 1) gets pytest like any
  other pure code.
- Behavior changes to nodes are verified live against f1tenth_gym_ros in WSL before being
  called done: launch, drive, watch the topic in question.

## Everything

- A bug fix lands with the check that would have caught it, wherever a test can reach it.
- If a change cannot be tested (launch-file plumbing, docs), say so in the report instead
  of pretending.
