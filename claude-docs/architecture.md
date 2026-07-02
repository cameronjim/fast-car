# Architecture

Three parts: two ROS 2 packages that run controllers against a live simulator or (formerly)
the car, and a plain-Python training package that steps the F1TENTH Gym directly.

```
gym_training/  (plain python, no ROS)          ROS 2 workspace
  f1rl trains policy in f1tenth_gym    ->      learned_control/ loads exported policy
  exports policy.pt + obs_config.json          reactive_control/ classical baselines
                                               f1tenth_gym_ros (external) simulates the car
```

## The ROS 2 side

Every launch pairs one controller node with one safety node. The controller decides where
to go; the safety node watches the LiDAR and has the final say.

```
sensors -> controller -> drive command -> safety node -> /drive -> car
                                              ^
                                            LiDAR
```

Topic wiring differs by package, and it matters:

- reactive controllers publish steering on `/drive` and read their allowed speed from the
  safety node on `/speed`.
- learned controllers publish on `/drive_raw`; the learned safety node republishes the
  gated result on `/drive`.
- `/kys` is the emergency-stop flag. The safety node latches it True on a forced brake and
  publishes False again once the hold-off passes and the forward sector is clear.

| Package | Nodes | Role |
|---|---|---|
| `reactive_control` | gap_follow, wall_follow, cv, safety | classical baselines, no learning |
| `learned_control` | rl_agent, safety | runs a policy exported by `gym_training/` |
| `learned_control` | bc_demo, sac_demo, sac_train | legacy learned stack (BC + online SAC) |

The `sim:=true|false` launch argument picks the odometry topic (`/ego_racecar/odom` in
sim, `/odom` on the car). The physical car is retired, so `sim:=false` is untested
legacy.

`sac_train_node` is the legacy online trainer: one env step per `/scan` message, capped
at real time. It stays as a working comparison path, but real training happens in
`gym_training/`.

## The training side (gym_training/)

Plain Python against the f1tenth_gym API: vectorized envs, headless, faster than real time.
Never imports rclpy, and no ROS node imports from it.

| Module | Holds |
|---|---|
| `f1rl/envs/make_env.py` | yaml config to a wrapped env: `EnvConfig`, lidar, resets, vec envs |
| `f1rl/envs/obs.py` | `ObsConfig` and `ActionBounds`: flat layout, normalization, contract writer |
| `f1rl/envs/f110_wrapper.py` | the RL wrapper: action rescale and clip, action repeat, speed cap |
| `f1rl/envs/reward.py` | `ProgressReward`: frenet progress, action-rate penalty, collision, lap bonus |
| `f1rl/envs/curriculum.py` | `SpeedCapSchedule`, `SpeedCapCurriculum`, `RacingEvalCallback` |
| `f1rl/envs/residual.py` | `ResidualPPWrapper`: planner command plus a bounded policy delta |
| `f1rl/envs/opponent.py` | `GapFollowerOpponent`: the scripted rival, a port of the ROS gap follower |
| `f1rl/envs/versus.py` | `VersusEgoWrapper` and `OvertakeBonus`: two-car spawns, race state, pass reward |
| `f1rl/planners/pure_pursuit.py` | the geometric baseline, also the residual runs' inner loop |
| `f1rl/track/` | raceline io and indexing, occupancy-grid clearance, raceline generation |
| `f1rl/train.py` | builds model and callbacks from a config and runs `learn` |
| `f1rl/evaluate.py` | deterministic rollouts, lap statistics, the leaderboard writer |
| `f1rl/run_versus.py` | head-to-head rollouts: overtake rate, time to pass, failure outcomes |
| `f1rl/export_policy.py` | torchscript trace plus `obs_config.json`, with a round-trip check |
| `f1rl/run_planner.py` | pure pursuit against the raw 100 Hz env, no RL wrapper |
| `f1rl/record_video.py` | offscreen rollout video; deadlocks a concurrent training run |

Every run is one yaml in `configs/`. `runs/<name>/` collects tensorboard logs, periodic
checkpoints, `best/best_model.zip` (reward-selected, sb3's own) and
`best/best_racing_model.zip` (selected on collision-free rate first, lap time second, which
is the one to ship). `artifacts/<milestone>/` holds what a milestone is judged on: the
chosen checkpoint, its `policy.pt` and `obs_config.json`, the gate logs, and any video.

Head-to-head mode is the other config switch: `env.versus.enabled` builds the env with two
agents, replaces `SingleAgentWrapper` with `VersusEgoWrapper` (ego on `agent_0`, the scripted
gap follower driving `agent_1` every physics step), and adds `OvertakeBonus` on the outside.
The observation is unchanged, because the opponent already shows up in the ego's LiDAR: the
simulator ray-casts every agent's body into every other agent's scan. That is what lets an m5
residual policy warm start straight into a race.

Residual mode is a config switch, not a fork: `env.residual.enabled` swaps the wrapper and
appends nine reference-line context features to the observation. Those features are marked
`deployable: false`, so residual exports are sim-only until a deploy-side planner exists.
`configs/sac_deploy.yaml` is the opposite case, the deployable feature set with no frenet
pose and no context block, which is what the ROS demo runs.

The contract between the two sides is two files produced by `export_policy.py`:

- `policy.pt`, a TorchScript module: obs vector in, action in [-1, 1] out.
- `obs_config.json`: LiDAR downsample count, clip range, normalization constants, action
  bounds and speed cap. The deploy node builds its obs exactly from this file.

## Maps and racelines

No maps live in this repo. Track geometry comes from f1tenth_gym's bundled racetracks
(centerline and raceline CSVs per track) on the training side, and from `f1tenth_gym_ros`
on the demo side. Raceline generation for maps without one goes through
`gym_training/f1rl/track/generate_raceline.py`.
