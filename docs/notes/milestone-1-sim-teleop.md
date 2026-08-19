# Milestone 1: keyboard teleop in sim through the safety gate

Status: done. Branch `milestone/1-sim-teleop`.

## What this milestone proves

Keyboard-driven remote control of the simulated car through the REAL command path
(claude-docs/04-architecture.md), end to end, before any learning is introduced:

```
keyboard_teleop_node -> /drive_raw -> safety_node -> /drive -> bridge_node (racer_gym)
```

`racer_safety/safety_node` is the sole publisher of `/drive`. No launch file, test, or
node in this milestone lets anything else publish it.

## What was built

- `ros_ws/src/racer_msgs`: `racer_msgs/SafetyEvent.msg` (stamp, severity, source gate,
  detail string). Minimal, per claude-docs/02-repo-layout.md's rule that racer_msgs exists
  only where std/ackermann messages do not fit.
- `ros_ws/src/racer_safety` (C++): `safety_node`. Gate/decision logic lives in
  `include/racer_safety/gate_logic.hpp` + `src/gate_logic.cpp`, completely separated from
  ROS plumbing (`src/safety_node.cpp`) so it is gtest-unit-testable with no ROS install.
  Enforces, every cycle:
  - watchdog on `/drive_raw` staleness (missing `watchdog_missed_cycles`, default 3, cycles
    at the node's `control_rate_hz`, default 50 Hz, means brake);
  - command sanity (a non-finite steering angle or speed is treated as garbage and brakes,
    not clamped);
  - absolute bounds clamp and rate-limit clamp on steering/speed, sourced only from the
    generated `vehicle_params` C++ binding (steering min/max angle and rate,
    `limits.min_velocity_mps`/`global_speed_cap_mps`, `actuation.max_acceleration_mps2`);
  - TTC braking from `/scan`, using the already-clamped output speed as the forward-speed
    estimate (no `/odom`/EKF feeds `safety_node` yet -- that is Phase 2);
  - a covariance-gate stub (see "What was intentionally stubbed" below).

  Every intervention publishes a `/safety/events` record. Any internal exception (or the
  test-only `inject_fault` parameter) results in a hard brake, never a passthrough.
- `ros_ws/src/racer_tools` (Python): `keyboard_teleop_node`. WASD or arrow keys step
  throttle/steering, spacebar is immediate zero/stop, `q` quits after publishing a zero
  command. Keymap/step/clamp logic (`racer_tools/keymap.py`) is a pure module unit-tested
  without a TTY; the terminal raw-mode reading loop
  (`racer_tools/keyboard_teleop_node.py`) is thin plumbing. Step sizes come from
  `vehicle_params` (one control period's worth of the vehicle's own max steering rate /
  max acceleration), not an invented number.
- `ros_ws/src/racer_bringup`: `launch/sim_teleop.launch.py` (bridge_node + safety_node;
  `start_teleop` launch argument, default false -- see the two-terminal procedure below)
  and `launch/bridge.launch.py` (moved here from `sim/bridge/racer_gym_bridge`, which had
  been carrying it as an interim location since roadmap task 0.5).
- `tests/e2e_sim_safety`: the L5-flavored end-to-end test -- launches bridge_node and
  safety_node together with no remaps, feeds scripted `/drive_raw`, asserts the sim pose
  advances, stops `/drive_raw`, and asserts the watchdog zeroes `/drive` and the simulated
  vehicle actually slows down.
- `.github/scripts/racer_safety_coverage.sh` + a `racer-safety-coverage` CI job: builds
  `racer_safety` with `--coverage` instrumentation and gates gate_logic.cpp/.hpp at 100%
  branch coverage with gcovr.
- Housekeeping (see the PR body for the full list): CI push trigger restricted to `main`
  (pull_request already covers PR branches, so this removes the duplicate push+PR run on
  every PR branch push); the "one roadmap task per branch" convention doc updated to "one
  milestone per branch"; the `sim/racer_gym` non-editable-install packaging bug fixed
  (`racer_gym.params.discover_repo_root`), with `training/racer_train`'s editable-install
  workaround dropped since it is no longer needed.

## What was intentionally stubbed

**Covariance gate**: `racer_safety::evaluate_covariance_gate` is a stub. No `/pose`
publisher exists yet (roadmap task 2.6 is the particle filter / pose estimator). The stub
fails SAFE: with no pose input it always returns "not engaged, no derate", and it is wired
into `SafetyGateLogic::evaluate()` so that its absence never disables the watchdog,
command-sanity, bounds/rate-limit, or TTC gates, which all run unconditionally regardless
of covariance-gate state. `test/test_gate_logic.cpp`'s `CovarianceStub` test group proves
this directly (e.g. a NaN command still brakes with `has_pose_input=false`).

**TTC thresholds**: `config/vehicle_params.yaml`'s `limits.ttc_warning_s`/`ttc_brake_s` are
currently `null` (not yet tuned -- see that file's own comments). `safety_node` exposes
them as ROS parameters defaulting to whatever `vehicle_params` holds (a negative default
when null, meaning "unconfigured": the TTC gate is a documented no-op with no launch file
in this milestone overriding it). Once a real value lands in `vehicle_params.yaml`, no
launch file needs to change. `tests/e2e_sim_safety` and the default `sim_teleop.launch.py`
run with TTC disabled; `racer_safety`'s own L3 launch test overrides the parameters to
prove the gate itself works ahead of real tuning data existing.

## Running it locally (Mac, Docker + Colima)

Image `ros-dev:local` must already be built (rebuild with
`docker build -t ros-dev:local docker/ros-dev` if its Dockerfile changed -- this milestone
added `gcovr` to it for the coverage gate).

```bash
export PATH="$HOME/bin:$HOME/.local/lima-dist/bin:$HOME/.local/bin:$PATH"
```

### Build and run every automated test for this milestone

```bash
docker run --rm --shm-size=1gb -v /Users/cameronjim/code/car:/repo -w /repo ros-dev:local bash -lc '
  source /opt/ros/humble/setup.bash
  cd ros_ws
  apt-get update
  rosdep update >/dev/null
  rosdep install --from-paths src --ignore-src -r -y
  colcon build --symlink-install
  colcon test --event-handlers console_direct+
  colcon test-result --verbose
'
```

The gate-logic coverage gate and the bridge+safety_node end-to-end test each build their
own combined workspace (see `.github/scripts/racer_safety_coverage.sh` and
`.github/scripts/e2e_sim_safety_test.sh`) -- run them the same way, with the repo
bind-mounted at `/repo` instead of `/workspace`.

### Drive the car (two terminals)

**Bug found while writing milestone 2 (fixed here):** the terminal 1 command below used to
build and source only `ros_ws`. That is not enough -- `bridge_node` lives in
`sim/bridge/racer_gym_bridge`, a SEPARATE colcon workspace from `ros_ws` (following this
procedure literally without building it first makes `ros2 launch` fail with `package
'racer_gym_bridge' not found`), and even once that workspace is built, `bridge_node` needs
the `gymnasium`/`f1tenth_gym` packages that live in the image's uv venv, not the system
Python -- without `PYTHONPATH` pointing at that venv's site-packages, `bridge_node` dies
immediately with `ModuleNotFoundError: No module named 'gymnasium'`. The corrected command
below builds both workspaces and sets `PYTHONPATH`, the same two-workspace + `PYTHONPATH`
shape `docs/notes/milestone-2-sim-viz.md` and `.github/scripts/sim_bridge_build_test.sh`
already use. `viz` defaults to `true` (foxglove_bridge starts alongside bridge_node/
safety_node even in this milestone's plain teleop demo), so terminal 1 also publishes port
8765 -- pass `viz:=false` after `sim_teleop.launch.py` for a foxglove-free run, or see
`docs/notes/milestone-2-sim-viz.md` to actually view it in Foxglove.

Terminal 1, from inside the container, starts the sim + safety gate:

```bash
docker run --rm -it --shm-size=1gb --name racer-sim -p 8765:8765 \
  -v /Users/cameronjim/code/car:/repo -w /repo ros-dev:local bash -lc '
  source /opt/ros/humble/setup.bash
  apt-get update
  rosdep install --from-paths ros_ws/src sim/bridge --ignore-src -r -y
  export PYTHONPATH="$(find /ros-dev/.venv/lib -maxdepth 1 -type d -name "python3.*")/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
  cd sim/bridge
  colcon build --symlink-install --base-paths .
  source install/setup.bash
  cd /repo/ros_ws
  colcon build --symlink-install
  source install/setup.bash
  ros2 launch racer_bringup sim_teleop.launch.py
'
```

Terminal 2, in a SEPARATE `docker exec -it racer-sim bash` into the SAME running container
(the `--name racer-sim` above is what makes this `docker exec` target unambiguous), runs
teleop -- this needs its own real interactive TTY, which is why it is not started by the
launch file above by default:

```bash
docker exec -it racer-sim bash -lc '
  source /opt/ros/humble/setup.bash
  source /repo/ros_ws/install/setup.bash
  ros2 run racer_tools keyboard_teleop_node
'
```

Controls: WASD or arrow keys to steer/throttle, spacebar to stop immediately, `q` to quit
(publishes a final zero command first).

`sim_teleop.launch.py` also accepts `start_teleop:=true` to start `keyboard_teleop_node` in
the same launch, but that is not the supported path: `ros2 launch` multiplexes several
processes' stdio, and only one process can reliably own the terminal for raw single-key
input. The two-terminal procedure above is the one that actually works.
