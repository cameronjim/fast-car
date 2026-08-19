# 01 — Roadmap

Work **one task at a time**, top to bottom within the active phase. The two tracks
(hardware / sim) may proceed in parallel; tasks within a track are ordered. Do not cross a
gate until its criterion is met. Mark tasks `[x]` when done, and add a one-line completion
note with date under the task.

Status legend: `[ ]` todo · `[x]` done · `[~]` in progress · `[!]` blocked (say why).

## Phase 0 — Environments and repo (hardware + sim tracks share this)

- [x] 0.1 Create repo skeleton per `02-repo-layout.md`
      Done 2026-08-22: directories, top-level READMEs, .gitignore, .clang-format added.
- [x] 0.2 `sim-cpu` image + lockfile; `f1tenth_gym` runs headless in it
      Done 2026-08-22: docker/sim-cpu/ (Dockerfile + uv.lock, f1tenth_gym
      pinned to a commit SHA on its v1.0.0/gymnasium branch, CPU-only
      torch) built and its headless gym+torch smoke test run in CI on
      every push touching that image (job sim-cpu-image).
- [~] 0.3 `train-cuda` image + lockfile on Desktop A; torch sees the GPU
      2026-08-22: docker/train-cuda/ (Dockerfile + uv.lock, f1tenth_gym
      pinned to the same commit SHA as sim-cpu, torch 2.13.0+cu126 via the
      cu126 wheel index) built and CI-verified (job train-cuda-image: image
      builds, CPU-safe sanity confirms a +cu126 torch build). `gpu_check.py`
      ships in the image but has not run yet -- no GPU exists on this Mac or
      on any GitHub-hosted runner. Still open: owner runs
      `docker run --gpus all train-cuda:local` on Desktop A per
      docker/train-cuda/README.md and confirms `torch.cuda.is_available()`
      before this ticks to `[x]`.
- [x] 0.4 `ros-dev` image (ROS 2 Humble) + lockfile
      Done 2026-08-22: docker/ros-dev/ (Dockerfile pinned to the
      ros:humble-ros-base multi-arch manifest digest, no CUDA, + uv.lock
      for pytest/hypothesis test tooling) built and its colcon toolchain
      smoke test (build+test a throwaway ament_cmake and ament_python
      package pair) run in CI on every push touching that image (job
      ros-dev-image). The interim `ros:humble` container in the
      l3-and-cpp job is kept for now (ros_ws/src has no real packages
      yet, so swapping in the built image would add job complexity for
      no coverage gain); TODO left in ci.yml/ros_build_test.sh to swap
      once real packages land.
- [x] 0.5 Port/replace the Foxy-era `f1tenth_gym_ros` bridge to Humble (known cost, scheduled)
      Done 2026-08-22: sim/bridge/racer_gym_bridge, a clean-room bridge against the pinned
      gymnasium-API f1tenth_gym (not a port -- upstream targets the old gym==0.19.0 API).
      Publishes /scan and /sim/ground_truth_odom, subscribes to /drive, offers /sim/reset.
      L1 + L3 launch_testing tests green in CI (new sim-bridge-test job, ros-dev image +
      pinned gym); docker/ros-dev also gained a pytest<9.1 pin and X11/GL libs the gym's
      renderer needs, fixes surfaced while getting this job green.
- [x] 0.6 CI per `12-testing.md`: lint, L1–L3 on every push, coverage gates (100% branch on
      envelope + safety gate logic), nightly job stub on Desktop A
      Done 2026-08-22: GitHub Actions CI is live with scaffold-aware coverage gates that
      bind automatically once a package gains source and tests; the sim-cpu/ros-dev image
      swap-in for the interim ubuntu-latest/ros:humble jobs happens with tasks 0.2/0.4.
- [x] 0.7 `vehicle_params.yaml` schema + binding generator + cross-language round-trip test
      (see `06-vehicle-params.md`, `12-testing.md` L1)
      Done 2026-08-22: config/vehicle_params.schema.json + an initial
      config/vehicle_params.yaml sourced from the f1tenth_gym default vehicle
      params (cited in the yaml, meta.sysid_session_id = "none-preliminary");
      tools/gen_params.py validates against the schema and generates Python,
      C++17, and C bindings, covered by tools/tests/ (schema refusal tests,
      hypothesis property tests, and a compiled C++/C round-trip test),
      100% coverage on gen_params.py, green in CI.
- [x] 0.8 `docs/conventions.md` seeded from `10-conventions.md`
      Done 2026-08-22: docs/conventions.md seeded, tooling preferences section included per
      03-environments.md; claude-docs stay authoritative on conflict.
- [x] 0.9 Test harnesses scaffolded: replay/golden framework + bag-mutation fault injectors
      (`12-testing.md` L4), sim-in-loop runner (L5), bench checklist runner (L6)
      Done 2026-08-22: tests/replay_harness (racer_replay), tests/sim_in_loop
      (racer_sim_in_loop), tests/bench (racer_bench) added, each with L1/L2 self-tests
      green in CI; L4-small/L5-short CI job now runs these self-tests instead of an
      always-pass placeholder. No real bags, golden files, or reference trajectories yet
      (those arrive with tasks 2.8/S.2); one placeholder bench procedure template only.

## Hardware track

### Phase 1 — Chassis, power, safety layer 1, teleop

- [ ] 1.1 Assemble chassis, motor, ESC (VESC), servo; bench-test with wheels off ground
- [ ] 1.2 Power tree per `11-hardware.md`; rail-voltage logging proven on bench
- [~] 1.3 Layer-1 safety: RC mux + power cutoff on independent MCU; kill tested with Jetson
      frozen (actually freeze it and prove the cut)
      2026-08-23: firmware/safety_mux/ drafted (milestone 4) -- mux state machine, watchdog
      timing, RC switch interpretation, and PWM validity checks are pure C
      (firmware/safety_mux/logic/), table-driven and host-tested (gcc -Wall -Wextra -Werror
      -Wpedantic, every branch, CI job safety-mux-host-tests). The Pico SDK glue
      (firmware/safety_mux/pico/) and CMakeLists.txt are written but UNVERIFIED: no Pico SDK/
      arm-none-eabi-gcc toolchain exists in this repo's containers or in CI, so none of it has
      ever compiled for or run on a real RP2040. New vehicle_params fields this firmware needs
      (steering.pwm_{min,max,neutral}_us, actuation.throttle_pwm_{min,max,neutral}_us,
      limits.mux_watchdog_timeout_s, limits.mux_kill_switch_threshold_us) are added to the
      schema, all `null` pending Phase 1 bench measurement (schema_version bumped 0.1.0 ->
      0.2.0). The roadmap 1.3 kill test (Jetson actually frozen, cut proven, human present)
      remains the only thing that makes this real -- see firmware/safety_mux/README.md and
      docs/notes/hardware-arrival-checklist.md section 3. Still `[~]`, not `[x]`.
- [~] 1.4 `car` image on Jetson (JetPack 6 base); SSH workflow from Mac documented
      2026-08-23: docker/car/ (milestone 4) authored -- Dockerfile (JetPack 6.1 / L4T
      r36.4.0 base pinned by tag + digest, ROS 2 Humble via the ROS apt repo, Jetson torch via
      an explicit build-arg wheel URL the build refuses to guess, racer runtime Python deps
      via a uv-locked linux/aarch64 lockfile) plus build_on_jetson.md's owner procedure. Never
      built or run anywhere -- no arm64/L4T hardware or trustworthy emulation exists in this
      repo's dev containers or in CI; CI runs a hadolint static-lint pass only (job
      docker-car-lint), no `docker build`. See docker/car/README.md's "What is and isn't
      verified" table. SSH build-and-launch workflow is documented in build_on_jetson.md but,
      per the same table, unexecuted. Still `[~]`, not `[x]`.
- [ ] 1.5 VESC driver node up; current-mode teleop via RC through the mux
- [ ] 1.6 rosbag logging of every drive, including rail voltage — verified before gate

**GATE G1: teleop with layer-1 safety demonstrated, kill switch proven. Fail → fix; nothing
downstream matters.**

### Phase 2 — Sensors, sync, state estimation, localization

- [ ] 2.1 Ingest MCU board: IMU + wheel speeds + steering pot, one clock, streaming to Jetson
- [ ] 2.2 Sync design doc written: measured offsets for LiDAR and VESC timelines, residual
      jitter quantified (deliverable, not a detail)
- [ ] 2.3 LiDAR mounted, driver up, scan rate verified
- [ ] 2.4 Adopt (not write) EKF: `robot_localization` or equivalent, fusing IMU + wheel
      speeds + steering angle; C++ node
- [ ] 2.5 SLAM map of the venue track; particle-filter localization (adopt F1TENTH stack)
- [ ] 2.6 Covariance gate wired into safety node: degraded pose → slow the car
- [ ] 2.7 Latency/jitter histogram published for the full control path
- [ ] 2.8 Replay test suite seeded from real bags: nominal, LiDAR dropout, stale sensor,
      timestamp jump; fault-injection tests green for estimator + safety node (L4)

**GATE G2: localization holds at ≥70% of target speed for 20 consecutive laps.
Fail → lower speed target, or pivot to scope option C.**

## Sim track (parallel with Phases 1–2; needs no car)

- [x] S.1 Model upgrades in gym fork: load transfer, Pacejka front/rear, first-order actuator
      dynamics, transport delay (see `07-sim-and-sysid.md`) -- 2026-08-22, sim/racer_gym,
      PR #9
- [x] S.2 Raceline optimizer + tracking controller running in sim; tracker lap test committed
      as the CI regression canary (L5) -- 2026-08-22: tools/raceline (synthetic stadium
      centerline + curvature/friction-limited speed profile), ros_ws/src/racer_control
      (pure pursuit, curvature-adaptive lookahead), config/tracks/gym_oval/raceline.csv,
      and the l5-tracker-lap CI job (2-lap canary; committed lap-time band widened from
      two same-code CI runs' measured wall-clock variance, 11.6-18.8s -- see that test's
      own comment; a real +-25% band was too tight given that spread), all green in CI,
      PR #13.
- [x] S.3 Training pipeline: SAC/PPO residual on base controller, envelope enforced in env
      Done 2026-08-22: training/racer_train/ (gymnasium ResidualRacerEnv wrapping
      sim/racer_gym: Python port of racer_control's pure-pursuit base controller,
      bounded residual action, training/envelope's apply() enforced in-env, reward =
      progress - crash - envelope violation only), SAC via stable-baselines3,
      training/configs/s3/ experiment config, train.py emitting a deployment contract
      ros_ws/src/racer_policy's load_contract + verify_against_environment loads.
      Cross-language divergence test: a committed fixture (deterministic synthetic
      states + the committed raceline) run through both this Python controller and a
      new racer_control pure_pursuit_cli binary, compared by a colcon CTest within
      1e-6. Nightly training-smoke job runs a real tiny SAC run on CPU (torch/SB3 kept
      out of the per-push L1 job via a separate dependency group). All green in CI,
      PR #15.
- [x] S.4 Envelope module (bounds, rate limits, OOD fallback) as a standalone library with
      100% branch coverage + the property test: any input, any state → output in bounds (L1/L2)
      Done 2026-08-22: training/envelope/ (bounds, rate limits, OOD fallback trigger + a
      reference distance scorer, speed cap, single apply() entry point), 100% branch
      coverage and the hypothesis property test green in CI, mypy clean.
- [x] S.5 Deployment contract implemented; one refusal test per mismatch class, all green (L1)
      Done 2026-08-22: ros_ws/src/racer_policy/ (plain Python package, ROS-free):
      contract.yaml + contract.schema.json, load_contract() (manifest validation,
      contract_version major check, policy.pt checksum) and
      verify_against_environment() (vehicle_params version, observation field
      order/dtype/units/missing/extra, LiDAR beam_count/fov/downsample), each mismatch a
      specific hard-refusal exception with no override. torch stays lazily imported inside
      load_model() only. One test per mismatch class plus a hypothesis property test and a
      real-vehicle_params integration test, 99.8% coverage (>=90% gate), mypy clean, green
      in CI. policy_node (rclpy wiring) is task 5.2.
- [x] S.6 Sim dynamics regression battery: sysid maneuvers in sim vs. committed references,
      run on every `racer_gym` change (L5) -- 2026-08-22: tests/sim_regression
      (racer_sim_regression) -- throttle step, steering step, constant-radius circle at
      three speeds, coastdown -- checked against committed references
      (racer_sim_regression/references/) with racer_replay's golden/tolerance engine
      (extended with a backward-compatible provenance header), plus a determinism test, the
      regression gate itself, and an injected-parameter canary test proving the comparison
      actually fails on a real dynamics change; sim-regression-battery CI job (path-filtered
      to sim/racer_gym, tests/sim_regression, tests/replay_harness), all green in CI, PR #14.

## Merged (car + sim)

### Phase 3 — System ID and model calibration

- [ ] 3.1 ID battery on venue surface: steps, constant-radius sweeps, coastdown, figure-eights
- [ ] 3.2 Fit parameters; validate on held-out maneuvers (report validation error only)
- [ ] 3.3 Standard 10-min re-ID battery scripted; run at start of every session; drift record
- [ ] 3.4 Randomization ranges set from fit residuals + drift data
- [ ] 3.5 Fidelity curve v1: sim-vs-real error vs. proximity to friction limit

### Phase 4 — Classical baseline on hardware

- [ ] 4.1 Raceline optimization for venue track(s)
- [ ] 4.2 Tracker tuned on hardware; tuning hours logged
- [ ] 4.3 Baseline laps under full evaluation protocol machinery (timing gate, interleaving
      scripts, logging)

**GATE G3: baseline laps reliably under the §09 protocol. Fail → the evaluation is
impossible for any controller; fix before training a policy.**

### Phase 5 — Policy training and deployment

- [ ] 5.1 Train residual in calibrated sim with randomization
- [ ] 5.2 Deploy via contract; zero-shot measurement recorded (report only)
- [ ] 5.3 On-hardware calibration/adaptation pass
- [ ] 5.4 Envelope verified live: forced OOD → fallback to base controller demonstrated

### Phase 6 — Evaluation

- [ ] 6.0 Analysis code tested: stats functions vs. hand-computed answers + synthetic data
      with known effect size recovered; interleaving generator property-tested (`12-testing.md`)
- [ ] 6.1 Pre-registration written and committed BEFORE first eval lap (see `09-evaluation.md`)
- [ ] 6.2 Evaluation sessions executed: paired, interleaved, ≥20 laps/condition/track, 2 tracks
      (one with frozen residual weights)
- [ ] 6.3 Analysis notebook: CIs, completion rates, interventions; no post-hoc metric changes

### Phase 7 — Writeup assembly

- [ ] 7.1 Assemble from per-phase notes (writeup is continuous; this phase is assembly)

## Standing rules

- **Definition of done for every task above:** code + the tests `12-testing.md` requires for
  the layers touched, green in CI. Bench-touching tasks additionally require their L6
  scripted procedure.
- ~10% of calendar is repair time; crashes are scheduled, not exceptions.
- Any "policy got worse" investigation starts with the re-ID battery (did the CAR change?)
  and the rail-voltage log.
