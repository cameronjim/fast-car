# Reinforcement learning against the F1TENTH Gym

This document is the training story: what the policy sees, what it controls, what it is paid
for, how it is pushed to race speed, and what it achieved. The code lives in
`gym_training/`, is plain Python, and never imports ROS. The ROS side enters only at the
end, when a trained policy is exported for the demo.

Contents:

- [Why train outside ROS](#why-train-outside-ros)
- [Observation](#observation)
- [Action](#action)
- [Reward](#reward)
- [The speed curriculum](#the-speed-curriculum)
- [Residual RL over pure pursuit](#residual-rl-over-pure-pursuit)
- [Results](#results)
- [Deploying a policy](#deploying-a-policy)
- [Where to look next](#where-to-look-next)

## Why train outside ROS

The repo's first learned controller trained online inside ROS: one environment step per
`/scan` message, which caps training at real time and at whatever the message rate happens
to be. That path still exists (`sac_train_node`, described in
[learned-control.md](learned-control.md)) and it is kept as a comparison, but it is not how
the current policies are trained.

`gym_training/` steps the simulator directly. Eight vectorized workers run headless at
roughly 8000 physics steps per second, and a full SAC run reaches a racing policy in under
an hour on one RTX 3060. Nothing about that is possible while a ROS message rate is in the
loop.

The training environment is stable-baselines3 SAC over the F1TENTH Gym `f1tenth-v0`
environment, wrapped by `F110RLWrapper`, which owns the observation layout, the reward, the
action rescaling, and the speed cap.

## Observation

The gym publishes a features dictionary; `SingleAgentWrapper` and `FlattenObservation` turn
it into one flat vector. `gym.spaces.Dict` sorts its keys, so the flat layout is
alphabetical rather than the order written in the config. `ObsConfig` owns that layout and
every normalization constant, and writes both into `obs_config.json`.

The default sim feature set is 116 dimensions:

| Slice | Field | Normalized by |
|---|---|---|
| `[0:1]` | `ang_vel_z` | 5 rad/s |
| `[1:2]` | `delta`, the front wheel angle | 0.4189 rad |
| `[2:5]` | `frenet_pose`, arc length and lateral and heading error | track length, 5 m, pi |
| `[5:6]` | `linear_vel_x` | speed norm, 8 or 12 m/s |
| `[6:114]` | `scan`, 108 LiDAR beams | 30 m |
| `[114:116]` | the previous action | already in [-1, 1] |

Every entry is scaled into [-1, 1] and clipped there, so the policy always sees a unit box.
The previous action is appended because the policy controls a car with steering dynamics:
without it the network cannot tell which way the wheels are already pointed.

`frenet_pose` is the interesting one. It is the car's position in the track's own
coordinate frame, and it makes learning much easier, but it is a simulator luxury: a real
car has no frenet frame and neither does the ROS bridge. So the feature list is
config-driven, and the deploy runs drop it. `configs/sac_deploy.yaml` trains on
`[scan, linear_vel_x, ang_vel_z, delta]` plus the previous action, 113 dimensions, all of
which the ROS node rebuilds from `/scan` and `/odom`. Steering is the one approximation:
the node feeds back the angle it last commanded, since ROS gives no steering feedback.

`obs_config.json` marks each feature with a `deployable` flag and `export_policy.py` warns
when an export depends on a sim-only one. The deploy node refuses such a contract outright
rather than driving on a vector it cannot fill.

## Action

Two numbers in [-1, 1], rescaled by the wrapper:

```
steering = a0 * steer_max_rad                                       # +-0.4189 rad
speed    = speed_mid + a1 * speed_half                              # speed_min .. speed_cap
```

The environment's own `step()` does not clip, so the wrapper clips at both ends, and the
clipped value is what gets stored as the previous action. One control step is
`action_repeat` physics steps at 100 Hz, so the default `action_repeat: 4` is 25 Hz
control. That number ships in `obs_config.json` as `control_hz`, because a deploy node
running the policy at the wrong rate is driving a different controller.

## Reward

Per control step, computed by the wrapper rather than by the simulator's native reward
modes, so the shaping is fully under the config's control:

```
w_prog * progress_m
  - w_rate * ||a - a_prev||^2
  - collision_penalty            (on contact)
  + lap_bonus * target_lap_time_sec / lap_time_sec   (on a completed lap)
```

`progress_m` is the simulator's forward frenet arc length for the step, summed over the
repeated physics steps. Paying for distance rather than for speed is what keeps the policy
honest in corners: cutting a corner and hitting a wall ends the episode and forfeits every
future metre.

The lap term is a ratio, not a constant, so a faster lap is worth strictly more than a slow
one, and `target_lap_time_sec` sets what a full bonus means. The M2 run paid a full bonus
for a 120 s crawl, which is exactly the kind of accidental ceiling this form avoids.

Episodes end on collision, on `max_episode_steps` control steps, and on `wrong_way_steps`
consecutive steps of negative progress.

One measured caveat: `w_rate` is inert at these settings, 0.16% of the per-step reward at
every speed. Steering chatter grows with speed, so if smoothness ever matters, that term
needs strengthening rather than tuning down.

## The speed curriculum

A policy asked to drive at 8 m/s from random weights crashes before it learns anything. The
curriculum starts the speed cap low and raises it only when the policy has earned it.

`SpeedCapSchedule` watches the evaluation rounds. When the collision-free rate stays at or
above `promote_rate` across `window_rounds` consecutive rounds, and the cooldown since the
last promotion has passed, the cap moves up by `step_mps` and `SpeedCapCurriculum` applies
it to every training worker and to the eval env at once. A promotion changes what a unit
speed action means, which is why the replay buffer is large enough for old transitions to
age out rather than being reused at the new scale.

Selection matters as much as the schedule. Once every episode survives, mean reward is
ceiling-limited by truncation and stops discriminating, so `RacingEvalCallback` scores each
round on collision-free rate first and best lap time second and saves
`best/best_racing_model.zip` on that score. Residual runs in particular are not monotone:
every map collapses to 0% clean for one or more rounds mid-run and recovers, so the final
model is never the one to ship.

## Residual RL over pure pursuit

Setting `env.residual.enabled` swaps the wrapper for `ResidualPPWrapper`, which embeds a
pure pursuit planner in the training loop. The planner replans against its reference line
every 100 Hz physics step, and the policy outputs bounded deltas at the control rate rather
than absolute commands:

```
steering = clip(steer_pp + a0 * dsteer_max_rad, +-steer_max_rad)
speed    = clip(speed_pp + a1 * dspeed_max_mps, speed_min_mps, speed_cap_mps)
```

Two things follow from that shape. The policy starts from a controller that already laps,
so it spends its samples on where the planner is wrong instead of on learning to stay on
the track. And a zero action reproduces the planner exactly, which makes the baseline a
control produced by the same code path rather than a number quoted from another run.

Residual mode appends nine reference-line context features to the observation: lateral and
heading error against the line, its curvature at five horizons ahead, and the planner's own
speed and steering commands. Every one of them is `deployable: false`, because no ROS node
can rebuild them from `/scan` and `/odom`. Residual policies are sim-only until a
deploy-side planner exists.

## Results

Full table with the caveats in [gym_training/leaderboard.md](../gym_training/leaderboard.md).
Spielberg, best lap over the run:

| Controller | Best lap | Clean | Notes |
|---|---|---|---|
| pure pursuit, shipped raceline | 37.99 s | 0% crash | closed form, 100 Hz |
| SAC from scratch (M4, 9.5 cap) | 37.97 s | 100% | 25 Hz, no planner underneath |
| residual SAC over pure pursuit (M5) | 33.40 s | 100% | 50 Hz deltas over a 100 Hz planner |

The from-scratch policy ties a tuned geometric controller while driving at a quarter of its
control rate. The residual policy beats it by 4.6 s, and beats pure pursuit on Monza and
YasMarina too. The comparison is not apples to apples in either direction, which the
leaderboard spells out: control rates differ, and the RL rows are best-of-run checkpoints
evaluated over 20 randomized-start episodes while the planner rows are one deterministic
attempt.

The other measured facts worth carrying: the policy ceiling on Spielberg is about 9.5 m/s,
where it still laps 100% clean and drops to 65% at 10.0. Residual policies saturate both
delta bounds on every map, so 0.15 rad and 1.5 m/s are what limit them now, not the speed
ceiling. And a multi-map policy laps four tracks clean at a 5 m/s cap, so the feature set
generalizes even if the speed does not transfer for free.

## Deploying a policy

`export_policy.py` writes two files and nothing else needs to cross the boundary:

- `policy.pt`, a TorchScript trace of the deterministic actor: observation vector in,
  action in [-1, 1] out. The exporter checks the trace against sb3's own `predict` over a
  batch of real observations and fails if they disagree by more than 1e-5.
- `obs_config.json`, the contract: feature layout, beam count, every normalization
  constant, the clip range, `control_hz`, and the action bounds.

The ROS node reads both. Nothing is copied by hand, and no constant in the node is
commented "must match" anything.

The deploy run is `configs/sac_deploy.yaml`: Spielberg, randomized starts, the deployable
feature set, the speed cap curriculum from 4.5 to 8.0 m/s, and an observation speed norm of
12 m/s so the speed feature still resolves changes at the top of that range. Its artifacts
live in `gym_training/artifacts/m6_deploy/`.

## Where to look next

- `gym_training/README.md`: install, every command, tuning notes.
- `gym_training/leaderboard.md`: lap times and what makes each comparison honest.
- `claude-docs/master-plan.md`: milestones, and the measured environment facts that shaped
  every decision above.
- `docs/learned-control.md`: the earlier BC and online SAC stack, kept as a comparison.
