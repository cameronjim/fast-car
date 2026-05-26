# gym_training

Plain-Python RL training against the F1TENTH Gym simulator. No ROS anywhere in this
package. It builds vectorized headless envs, trains stable-baselines3 SAC or PPO, and
exports a TorchScript policy plus the `obs_config.json` contract the ROS deploy node
reads.

## Install

Ubuntu (WSL2 works), Python 3.12, a CUDA GPU for SAC:

```bash
python -m venv ~/venvs/f1rl && source ~/venvs/f1rl/bin/activate
pip install torch==2.13.0 --index-url https://download.pytorch.org/whl/cu126
pip install -r requirements.txt
```

Torch must come from the cu126 index; the default PyPI wheel has the wrong CUDA build
for this stack. `requirements.txt` pins `torch==2.13.0+cu126` for the record, but pip
cannot resolve that local version without the index URL above, so install torch first.

Rendering uses OpenGL and needs a display. Headless boxes set `QT_QPA_PLATFORM=offscreen`
before any command that renders, and the MESA and ZINK lines on stderr are noise.

Maps download on first use into the `f1tenth_gym` checkout's `maps/` directory, so the
first run on a new track needs network access.

## Quickstart

```bash
pytest tests/ -q                      # logic tests plus a real-env wrapper check
pytest tests/ -q -m "not slow"        # skip the tests that step the simulator

python -m f1rl.train --config configs/sac_scratch.yaml
python -m f1rl.train --config configs/ppo_scratch.yaml --total-steps 200000

tensorboard --logdir runs

python -m f1rl.evaluate --model runs/sac_scratch/best/best_model.zip \
    --config configs/sac_scratch.yaml --episodes 20
python -m f1rl.export_policy --model runs/sac_scratch/best/best_model.zip \
    --config configs/sac_scratch.yaml --out-dir export
QT_QPA_PLATFORM=offscreen python -m f1rl.record_video \
    --model runs/sac_scratch/best/best_model.zip --config configs/sac_scratch.yaml
```

`--total-steps`, `--n-envs`, `--log-dir` and `--run-name` override the config from the
command line; everything else lives in the yaml.

## Classical baseline

`f1rl.run_planner` drives pure pursuit on the map's raceline. It bypasses the RL wrapper
and steps the raw env, so it controls at the full 100 Hz instead of the wrapper's 25 Hz.

```bash
QT_QPA_PLATFORM=offscreen python -m f1rl.run_planner --map Spielberg --laps 5
QT_QPA_PLATFORM=offscreen python -m f1rl.run_planner --map Monza --centerline --laps 3
QT_QPA_PLATFORM=offscreen python -m f1rl.run_planner --map Spielberg --laps 2 --video

python -m f1rl.evaluate --leaderboard --maps Spielberg Monza YasMarina --laps 5
python -m f1rl.evaluate --leaderboard --maps Spielberg --laps 5 \
    --model runs/sac_scratch/best/best_model.zip --config configs/sac_scratch.yaml
```

Steering is `atan(wheelbase * curvature)` onto a raceline point a velocity-scaled
lookahead `k_l * v + L0` ahead, clipped to ±0.4189 rad. The speed command is the
raceline's own `vxs` at the car's arc-length, times `speed_scale`. `--lookahead-gain`,
`--lookahead-min`, `--lookahead-max`, `--speed-scale` and `--fallback-speed` override the
tuned defaults.

Two things about the shipped racelines matter more than any tuning:

- They run very close to the walls. Spielberg clears by 0.26 m, Monza by 0.19 m and
  YasMarina by 0.00 m, against a 0.31 m wide car. Only Spielberg's is drivable; the
  others collide at crawling speed, and `--centerline` is the honest fallback there.
- Their speed profiles are conservative for this tire model, so `speed_scale` above 1.0
  is faster. On Spielberg 1.2 laps clean and 1.3 crashes.

Maps with no raceline csv leave `track.raceline` aliased to `track.centerline`, which the
planner detects and drives at `fallback_speed_mps`. Generating a speed profile from
curvature for those maps is M4+ work.

## Generating a raceline that fits the car

`f1rl.track.generate_raceline` builds a minimum-curvature line from the map's centerline
and the widths it measures off the occupancy grid, instead of trusting the widths the
centerline csv declares. It writes `<Track>_raceline_gen.csv` beside the shipped
`<Track>_raceline.csv` and never overwrites it.

```bash
QT_QPA_PLATFORM=offscreen python -m f1rl.track.generate_raceline --map Spielberg
QT_QPA_PLATFORM=offscreen python -m f1rl.track.generate_raceline --map Monza --margin 0.20

python -m f1rl.run_planner --map Spielberg --laps 5 --speed-scale 1.1 \
    --raceline-csv ~/f1tenth_gym/maps/Spielberg/Spielberg_raceline_gen.csv
```

The pipeline is `trajectory_planning_helpers`: `interp_track` and `conv_filt` to re-space
and de-noise the centerline, `calc_splines` for the offset frame, `opt_min_curv` for the
lateral shift, `create_raceline` and `calc_head_curv_an` for the line itself, then
`calc_vel_profile` and `calc_ax_profile` against a ggv built from `a_max` 9.51 m/s2,
`mu` 1.05 and a `v_switch` motor rolloff.

Three things make the result drivable where the shipped lines are not:

- The corridor is pulled in by the car's half width plus `--margin` **before** the
  optimizer sees it, so every line the solver can pick already fits.
- The corridor bound is the perpendicular clearance measured on the occupancy map, capped
  against the declared width. The declared 1.1 m is optimistic: YasMarina really has
  0.93 m, and an oblique wall is closer than its distance along the normal suggests.
- The finished line is re-measured against the map. The spline drawn through the
  optimizer's 1.5 m spaced points bulges outside the corridor between them, so the
  generator hands that bulge back to the keep-out and re-solves until the line clears.

The CLI prints min clearance, max curvature against `tan(0.4189) / 0.3302`, the speed
range and the line length, and writes nothing when either bound fails.

Generated clearance is 0.31 m on Spielberg, 0.30 m on Monza and 0.37 m on YasMarina,
against 0.26, 0.19 and 0.00 m for the shipped lines and a 0.155 m car half width. The
speed profile is planned at the full friction limit, so `speed_scale` above roughly 1.1
is too much on the tighter maps; see `leaderboard.md`.

## Residual RL over pure pursuit

Setting `env.residual.enabled` swaps `F110RLWrapper` for `ResidualPPWrapper`, which embeds
a `PurePursuitPlanner` in the training loop. The planner replans against its reference line
every 100 Hz physics step; the policy acts at the control rate and outputs bounded deltas,
not absolute commands:

```
steering = clip(steer_pp + a0 * dsteer_max_rad, ±steer_max_rad)
speed    = clip(speed_pp + a1 * dspeed_max_mps, speed_min_mps, speed_cap_mps)
```

`speed_pp` already carries the planner's `speed_scale`, so the scale is tuned once in
`residual.pure_pursuit` rather than twice. A zero action reproduces the planner exactly,
which makes the baseline a control the same code path produces, not a number quoted from
another run.

```bash
python -m f1rl.train --config configs/sac_residual.yaml
python -m f1rl.train --config configs/sac_residual_monza.yaml
python -m f1rl.train --config configs/sac_residual_yasmarina.yaml
```

`residual.reference` picks the line: `shipped`, `generated` (the `_raceline_gen.csv` beside
it), `centerline`, or `csv` with an explicit `raceline_csv` path or per-map mapping. It is
independent of `reference_line`, which only decides where resets spawn: Monza and
YasMarina spawn on the centerline because their shipped racelines sit too close to a wall
to respawn on.

The residual configs run `action_repeat: 2`, so 50 Hz control. The planner keeps steering
between policy calls, so a delta is cheap to hold for two physics steps, and halving the
repeat from the 25 Hz default doubles the correction bandwidth for the same physics cost.

Residual mode appends nine reference-line context features to the observation, after the
env features and before the previous action:

| Context slice | Field | Normalized by |
|---|---|---|
| `[0:1]` | lateral offset from the line, + to its left | `ref_lateral_m` |
| `[1:2]` | heading error against the line's tangent | pi |
| `[2:7]` | line curvature at s+5, 10, 15, 20, 30 m | `curvature_radpm` |
| `[7:8]` | planner speed command | `speed_mps` |
| `[8:9]` | planner steering command | `steer_rad` |

`ObsConfig` owns that block the same way it owns the env features, so `obs_config.json`
stays the single contract. Every context feature is `deployable: false`: the reference line
is a simulator asset and no ROS node can rebuild these from `/scan` and `/odom`, so
`export_policy` warns and residual policies stay sim-only until a deploy-side planner
exists.

Residual training is not monotone. Every map so far collapses to 0% collision-free for one
or more eval rounds mid-run and recovers, so gate on `best/best_racing_model.zip`, which
`RacingEvalCallback` selects on collision-free rate first and lap time second, never on
`final_model`.

## What the env looks like

One control step is `action_repeat` physics steps at 100 Hz, so the default
`action_repeat: 4` gives 25 Hz control, which is also the rate the deploy node must run
at. The rate lands in `obs_config.json` as `control_hz`.

Observation, 116 dims with the default config:

| Slice | Field | Normalized by |
|---|---|---|
| `[0:1]` | `ang_vel_z` | 5 rad/s |
| `[1:2]` | `delta` | 0.4189 rad |
| `[2:5]` | `frenet_pose` (s, ey, ephi) | track length, 5 m, pi |
| `[5:6]` | `linear_vel_x` | 8 m/s |
| `[6:114]` | `scan`, 108 beams | 30 m |
| `[114:116]` | previous action | already in [-1, 1] |

The order is alphabetical, not the order in the config: `gym.spaces.Dict` sorts its keys
and `FlattenObservation` follows that sort. `ObsConfig` owns the layout and every
normalization constant, and writes both into `obs_config.json`, so nothing is duplicated
by hand on the deploy side.

Actions are `[-1, 1]^2` and rescale to steering `±steer_max_rad` and speed
`speed_min_mps..speed_cap_mps`. The env's own `step()` does not clip, so the wrapper does,
at both ends. `set_speed_cap()` moves the cap at runtime for the speed curriculum.

Reward per control step:

```
w_prog * progress_m - w_rate * ||a - a_prev||^2 - collision_penalty (on contact)
                    + lap_bonus * target_lap_time_sec / lap_time_sec (on a completed lap)
```

`progress_m` is the simulator's forward Frenet arclength for the step, summed over the
repeated physics steps. Episodes end on collision, on `max_episode_steps` control steps,
and on `wrong_way_steps` consecutive control steps of negative progress.

## Sim-only versus deploy feature sets

`frenet_pose` is a simulator luxury: the ROS car has no Frenet frame, so a policy trained
on it cannot be deployed as is. The feature list is config-driven and both cases work:

- `configs/*.yaml` default to the sim feature set, `frenet_pose` included, because it
  trains faster and M2 through M5 are sim-only milestones.
- `configs/sac_deploy.yaml` is the deploy case: `features: [scan, linear_vel_x, ang_vel_z,
  delta]`, a 113-dim observation the ROS node rebuilds from `/scan` and `/odom`, with
  `obs_norm.speed_mps` raised to 12 so the speed feature still resolves changes at its
  8 m/s cap. It trains from scratch, since m4's 116-dim policy cannot warm start it.

```bash
python -m f1rl.train --config configs/sac_deploy.yaml
python -m f1rl.evaluate --model runs/sac_deploy/best/best_racing_model.zip \
    --config configs/sac_deploy.yaml --episodes 20 --speed-cap 8.0
python -m f1rl.export_policy --model runs/sac_deploy/best/best_racing_model.zip \
    --config configs/sac_deploy.yaml --out-dir artifacts/m6_deploy --speed-cap 8.0
```

The curriculum moves the cap off what the config declares, so `--speed-cap` is what makes
the exported contract and the evaluation agree with the policy that was trained.

`obs_config.json` marks each feature with a `deployable` flag, and `export_policy.py`
prints a warning when it exports a policy that depends on a sim-only feature.

## Notes for tuning

- SAC is gradient-bound, not simulation-bound. `train_freq: 64` with
  `gradient_steps: 64` batches the updates and reaches roughly 480 steps/s end to end on
  8 workers; the update-every-step default sits near 90.
- PPO runs on `device: cpu` on purpose. These MLPs are small enough that CUDA launch
  overhead dominates.
- Each worker gets its own seed and its own map from the `maps` list. `EnvConfig.seed` is
  shared across a vectorized env otherwise, and every worker replays the same rollout.
- The default `reset_strategy: RL_GRID_STATIC` spawns in the same place every episode, so
  seeded eval episodes come out nearly identical. Use `RL_GRID_RANDOM` or
  `RL_RANDOM_RANDOM` when the eval spread is the thing being measured.
