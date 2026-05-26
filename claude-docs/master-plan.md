# Master plan: the RL overhaul

The physical car is retired. The goal is a real RL agent trained from scratch against the
F1TENTH Gym API, pushed to the fastest lap times the sim's vehicle model allows, with the
ROS 2 side kept working for demos. The full approved plan lives outside the repo; this is
the working summary.

## Fixed decisions

- Training environment: WSL2 Ubuntu 24.04, venv at `~/venvs/f1rl`, RTX 3060.
- Simulator: f1tenth_gym pinned to `dev-features @ 02383a9619788af18bacddc6954cb309582cc7e0`
  (gymnasium API; `main` is still the legacy gym 0.19 package). Install torch from the
  cu126 index, use opencv-python-headless, render with `QT_QPA_PLATFORM=offscreen`.
- RL library: stable-baselines3 (SAC primary, PPO fallback). The repo's hand-rolled SAC
  stays as a comparison run against the same env.
- The legacy online ROS trainer (`sac_train_node`) stays as a working comparison path,
  not the training path.
- Demo sim: f1tenth_gym_ros (legacy physics). Small sim-to-sim gap accepted.

## Environment facts that shape the code

- Env id `f1tenth-v0`, config is a frozen `EnvConfig` dataclass, not a dict.
- Obs: use `FEATURES` observation with `LiDARConfig(num_beams=108)` native downsampling,
  `SingleAgentWrapper` + `FlattenObservation`. Dict keys sort alphabetically in the flat
  vector. A custom wrapper adds previous action and owns reward and the speed cap.
- Action: `[steering ±0.4189 rad, speed -5..20 m/s]`, and `step()` does not clip. The
  wrapper rescales from [-1, 1] and clips; the executed action is the stored action.
- Reward: wrapper-computed from `info["progress"]` (frenet arc-length per step) plus
  action-rate penalty, collision penalty, lap bonus. Native reward modes exist but the
  wrapper owns it for full control.
- Physics: ST model at 100 Hz, RK4. v_max 20 m/s, a_max 9.51, mu 1.05, wheelbase 0.33.
- Tracks download on demand into the gym checkout's `maps/` dir. Spielberg's bundled
  raceline carries a real 4.5..8.0 m/s speed profile; centerline CSVs carry track widths.
- Vectorized envs must be seeded per sub-env (a shared `EnvConfig.seed` makes identical
  workers). Throughput peaks near 8 subproc workers, ~8k steps/s at 108 beams.
- PPO trains faster on cpu than cuda for these MLP sizes (measured); SAC uses cuda.
- Two-agent env works with ego-only reward and termination: right shape for a scripted
  opponent, wrong shape for self-play. `TerminationConfig.collision_agents` defaults to
  `"ego"`, so an opponent crashing on its own does not end the episode; the crashed car has
  its pose rewound and its velocities zeroed on every contact step, which leaves it parked
  as a static obstacle that still occludes the ego's LiDAR.
- The simulator ray-casts every agent's body into every other agent's scan, so a second car
  is visible in `scan` with no observation change at all. That is why an m7 head-to-head run
  can warm start from a single-agent m5 policy: same 125 dims, same layout.
- `SingleAgentWrapper` hard-rejects `num_agents != 1`, so two-agent runs need their own
  ego-view wrapper. `reset(options={"poses": (n, 3)})` is the only way to place both cars.
- The frenet frame (and so `info["progress"]`, lap counting, `frenet_pose[0]`) is computed
  against the centerline even when resets reference the raceline. Normalize s by the
  centerline length (Spielberg 343.32 m) or it drifts each lap.
- `TerminationConfig.max_episode_steps` counts physics steps, not control steps: multiply
  by action_repeat. The wrapper runs 25 Hz control over 100 Hz physics (action_repeat 4),
  and `control_hz` ships in obs_config.json for the deploy node.
- `max_laps=None` disables lap termination but lap counting keeps firing, which is what
  makes continuous-lap training with a per-lap bonus work.
- For a single agent both grid strategies (`RL_GRID_STATIC`, `RL_GRID_RANDOM`) are
  deterministic: shuffle only permutes agent order and the start-line mask clamps to one
  waypoint. Only `RL_RANDOM_RANDOM` / `RL_RANDOM_STATIC` actually randomize. Evaluation
  uses those or it measures one trajectory n times.
- Lap counting is net centerline arclength from the reset pose, not a start-line
  crossing, so random starts and lap counts compose correctly.
- M2 result: from-scratch SAC converged in ~600k steps (42 min). The trained 3 m/s
  policy stays collision-free up to a 5 m/s cap with no retraining and first crashes at
  6 m/s, so curricula should start near 4.5-5. Eval mean reward is ceiling-limited by
  truncation once every episode survives; select on lap time and collision rate instead.
- `DomainRandomizationConfig` takes two absolute `VehicleParameters` bounds (low == high
  means fixed) and widens spaces via widest_params so spaces stay constant. Ready for M4.
- Measured SAC end to end: 477 steps/s with train_freq 64 / gradient_steps 64 on cuda,
  8 workers. PPO 1298 steps/s on cpu.
- trajectory_planning_helpers 0.79 is partly broken under numpy 2.5 / scipy 1.18:
  spline_approximation, check_normals_crossing, and iqp_handler all fail; local
  replacements live in f1rl/track/generate_raceline.py. quadprog must be >= 0.1.13
  (0.1.7's wheel fails to import).
- Shipped raceline wall clearance: Spielberg 0.26 m (drivable), Monza 0.19 m and
  YasMarina 0.00 m (undrivable for the 0.31 m car). Generated width-aware lines live as
  <Track>_raceline_gen.csv beside the shipped ones; run_planner takes --raceline-csv.
- M4 results: curriculum 4.5 to 8.0 clean, extension to a 9.5 cap gave 37.97 s best /
  38.03 s flying mean, 100% clean over 20 episodes, tying pure pursuit's 37.99 s. Real
  policy ceiling ~9.5 m/s (65% clean at 10.0). Multi-map policy laps 4 tracks clean at
  the 5.0 cap; every apparent failure was a spawn-inside-wall reset artifact.
- obs_norm.speed_mps 8.0 saturates the speed feature above 8 m/s; raising it is the
  cheapest M5 experiment. RL runs 25 Hz control vs pure pursuit's 100 Hz.
- DR at mu +-10% is inert at these speeds (control test: no-DR policy scores identically
  under randomization). A meaningful sweep needs wider mu plus mass/CoG.
- SB3 2.9.0 EvalCallback fires callback_after_eval before subclass post-processing;
  RacingEvalCallback overrides _on_event to close the eval round first.
- w_rate is inert (0.16% of per-step reward at every speed) and steering chatter grows
  with speed; if smoothness matters later, strengthen it rather than weakening.
- M5 results: residual SAC over pure pursuit beats the planner on all three maps, 100%
  clean over 20 episodes each: Spielberg 33.40 s vs 37.99, Monza 39.21 vs 42.04,
  YasMarina 43.70 vs 50.98. The planner replans at 100 Hz inside the wrapper and the
  policy adds deltas at 50 Hz (action_repeat 2). A zero action reproduces the planner
  exactly, which makes the baseline a control rather than a citation.
- Residual policies saturate both delta bounds (max |a| = 1.00 on steering and speed at
  every map), so 0.15 rad / 1.5 m/s is what limits them now, not the 12 m/s ceiling: the
  achieved top speeds are 11.07, 11.88, and 10.43 m/s.
- Residual training is not monotone. Every map collapses to 0% clean for one or more eval
  rounds mid-run and recovers; RacingEvalCallback's best-checkpoint selection is what makes
  the runs usable, so never gate on the final model.
- Monza's 1.35 speed_scale only survives a grid start on the line; under RL_RANDOM_RANDOM
  spawns it crashes 16/16, so the residual base runs 1.30 (43.62 s planner-alone).
- Reference-line context features (lateral/heading error, curvature horizons, planner speed
  and steering) go through ObsConfig as a context block appended before prev_action, and
  are all deployable:false. Residual exports are sim only until a deploy-side planner exists.
- The offscreen OpenGL video recorder deadlocks a concurrently training SAC run. Record
  videos with nothing else running.
- M6 results: the deployable feature set (`scan`, `linear_vel_x`, `ang_vel_z`, `delta`, plus
  prev_action, 113 dims) trains from scratch to the 8.0 m/s curriculum target in 700k steps
  and gates 100% clean over 20 episodes, 43.43 s best and 43.45 s flying mean. obs_norm
  speed 12.0 keeps the speed feature unsaturated at that cap. No warm start from m4 is
  possible: 116 dims against 113.
- The demo stack runs in a ros:foxy container. Foxy's default Fast-RTPS hangs spinning
  inside Node creation under WSL2; `ros-foxy-rmw-cyclonedds-cpp` plus
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` fixes it. Cyclone warns that lo is not
  multicast-capable and unicast localhost discovery works anyway.
- The f1tenth_gym_ros bridge publishes /scan and /drive at 250 Hz, the rl node's RateGate
  decimates to 25.3 Hz measured, and a latched /kys makes it publish a zero command on every
  scan instead, so /drive_raw jumps to scan rate while stopped.
- The bridge switches maps by its `map_path` parameter, and Spielberg spawns correctly at
  (0, 0, 3.4034 rad), the first centerline point and the shipped raceline's heading there.
- Live in the ROS sim the m6 policy laps Spielberg in 63.8 s against 43.4 s in training. The
  safety node's ttc thresholds cap /drive at 2.0 m/s through most corner approaches; the
  policy's own /drive_raw never leaves 6.4 to 7.9 m/s.
- Every built-in random reset spawns exactly on the reference line, so the residual planner
  has never seen an off-line start. Hand-placed spawns pay for it: the zero-residual anchor
  crashes 5/20 with no lateral jitter, 10/20 at 0.3 m and 15/20 at 0.5 m. m7 spawns at 0.15 m.
- The ported gap follower laps Spielberg clean 10/10 at every cap up to 5.5 m/s on 108 beams,
  at 91 s a lap for a 4.0 m/s cap. Its speed law interpolates over `clip_max_range_m`, so
  widening that range makes it slower, not more far-sighted: 88 s at 5.0 m, 110 s at 8.0 m.
- An even beam count has no ray dead ahead, so `(target - n // 2) * angle_increment` biases
  the aim by half a beam: 2.5 deg at 108 beams, enough to make the gap follower drift. Read
  `angle_min` off the sim and use `angle_min + target * angle_increment`.
- ament_pep257's D403 requires capitalized docstring openers, which contradicts CLAUDE.md's
  lowercase rule, and ament_copyright wants Apache headers this MIT repo does not carry. Both
  lint tests fail by construction in every package; treat colcon test's pytest results as the
  signal and read the lint list rather than gating on it.

## Milestones

| # | Milestone | State |
|---|---|---|
| M0 | WSL2 stack installed, gym smoke-tested, ROS bugs fixed | done |
| M1 | `gym_training/` skeleton: wrapper, obs, reward, eval, tensorboard | done |
| M2 | from-scratch SB3 SAC laps Spielberg at 3 m/s cap, zero warm start | done |
| M3 | pure pursuit on the raceline, leaderboard v1 | done |
| M4 | speed curriculum to 6.5-8 m/s, multi-map, light randomization | done |
| M5 | residual RL beats or ties pure pursuit | done |
| M6 | policy export, `rl_agent_node` ROS demo, README rewrite | done |
| M7 | head-to-head overtaking vs the gap follower | |

## Acceptance gates

- M2: at least 95% collision-free laps over 20 seeded eval episodes.
- M5: residual RL beats pure pursuit's lap time on at least one map.
- M6: exported policy demoed in f1tenth_gym_ros; online trainer survives 3+ auto-reset
  episodes.
