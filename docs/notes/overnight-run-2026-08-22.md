# Overnight run, 2026-08-22

Autonomous overnight session. Every currently unblocked roadmap task now has an open PR.
Nothing was merged; nothing was pushed to main. Hardware Phase 1/2 tasks need physical parts
and were not started. Phase 3+ sits behind gates.

## PRs opened, in suggested merge order

The branches are stacked. Merge each PR into its base in this order and the chain unwinds
cleanly; GitHub retargets children automatically if you delete each branch after merging.

| PR | Task | Base | State |
|---|---|---|---|
| #1 | 0.1 repo skeleton | main | green |
| #2 | 0.6 CI per 12-testing.md | task/0.1 | green |
| #3 | 0.2 sim-cpu image + lockfile | task/0.6 | green |
| #4 | 0.3 train-cuda image + lockfile | task/0.2 | green, roadmap left [~] |
| #5 | 0.4 ros-dev image + lockfile | task/0.3 | green |
| #6 | 0.5 gym bridge ported to Humble | task/0.4 | green |
| #7 | 0.8 docs/conventions.md | task/0.4 | green |
| #8 | 0.7 vehicle_params schema + bindings + round-trip | task/0.4 | green |
| #9 | S.1 racer_gym model upgrades | task/0.7 | green |
| #10 | 0.9 replay/golden, fault injection, sim-in-loop, bench scaffolds | task/0.5 | green |
| #11 | S.4 envelope library | task/0.7 | green |
| #12 | S.5 deployment contract, refuse on mismatch | task/S.4 | green |
| #13 | S.2 raceline optimizer + pure pursuit tracker + L5 lap canary | task/0.9 | green |
| #14 | S.6 sim dynamics regression battery | task/S.2 | green |
| #15 | S.3 SAC residual training pipeline | task/S.2 | green |

PRs #13 and #15 contain cross-lineage merge commits (S.2 merges in S.1; S.3 merges in S.5)
because they need code from two open branches at once. Their diffs against the base include
that merged content; the last commits on each are the reviewable part.

## Task 0.3 is the one open checkbox

The train-cuda image builds in CI and the wheel is verified as +cu126, but "torch sees the
GPU" can only run on Desktop A. When you are back:
`docker build` then `docker run --rm --gpus all train-cuda:local` per docker/train-cuda/README.md,
then flip 0.3 to [x].

## Assumptions and judgment calls to review

- f1tenth_gym is pinned to 5a301bd on the v1.0.0 (gymnasium) branch everywhere, not the
  stale gym==0.19 main branch.
- The bridge is a clean-room rewrite, not a port of the Foxy repo; upstream targeted the old
  API so a port would have rewritten everything anyway.
- vehicle_params.yaml initial values cite the pinned gym defaults; unmeasurable fields are
  null with comments. meta.sysid_session_id is "none-preliminary". track_width_m is null
  because the gym has no true analogue.
- S.1 keeps one dimensionless tire load-sensitivity exponent (0.9) in code, documented as a
  model-structure constant, not a fitted vehicle quantity. If you consider that a violation
  of invariant 2, it should move into the params schema.
- S.2's L5 lap canary asserts wall-clock lap time in [8, 30] s for 2 laps. The band was
  calibrated from real CI runs which showed about 40 percent run-to-run swing from runner
  load. Known improvement: assert on sim time or step count instead.
- The canary runs the stock gym env, not racer_gym's upgraded dynamics; wiring
  racer_gym.build_env into the bridge is left as a TODO citing S.6.
- The tracker's odom-silence behavior is to stop publishing rather than brake, deferring to
  the future safety_node watchdog, per the 04-architecture degradation table.
- S.2's test launch remaps ground-truth odom to /odom (no EKF yet) and /drive_raw to /drive
  (no safety_node yet). Sim-test shims only, loudly commented; the real graph keeps the
  safety node as the sole /drive publisher.
- S.3 duplicates the pure pursuit math in Python for the training env, guarded by a
  cross-language divergence test against the C++ core. The envelope itself is the shared
  library, never duplicated.
- S.6's coastdown maneuver exercises accel-limited braking because racer_gym currently has
  no independent drag term; flagged in the code as a model gap, not fixed.
- Nightly workflow now runs a real CPU training smoke on ubuntu-latest; the Desktop A
  self-hosted runner remains for you to wire (no secrets or runner config were added).
- No branch protection or repo settings were touched, per instructions.

## Blockers hit

- No Docker on this Mac, so all image verification went through CI.
- No GPU anywhere reachable, hence 0.3 staying [~].
- Hardware tasks (Phase 1 and 2) are blocked on physical parts; not attempted.
- No PLAN.md section 3 open question turned out to block any Phase 0 or sim-track task.

## Housekeeping done on request

- All commits rewritten to author cameronjim (GitHub noreply address) with no co-author
  trailers; task branches were force-pushed once during that rewrite, before any reviews.
- The repo-local git config now commits as cameronjim.
- gh (v2.98.0) installed to ~/bin, uv (0.12.5) to ~/.local/bin.

## What is next

1. Run the GPU check on Desktop A and tick 0.3.
2. Review and merge the PR chain in the order above.
3. After merging, decide on the S.1 tire-exponent question and the lap-canary sim-time
   improvement.
4. Hardware track begins at 1.1 (chassis assembly) whenever parts are in hand; G1 is the
   next gate.
