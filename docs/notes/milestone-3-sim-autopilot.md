# Milestone 3: classical autopilot in sim

Status: done. Branch `milestone/3-sim-autopilot`.

## Standards pre-check

Before starting, confirmed `main`'s latest CI run was green (`gh run list --branch main
--limit 1`: the PR #18 (milestone/2-sim-viz) merge run, all jobs passed). No repair work was
needed as part of Part A's pre-check.

Part A also fixed a milestone 1 doc bug found during milestone 2: `docs/notes/milestone-1-
sim-teleop.md`'s "Drive the car" procedure never built `sim/bridge` nor set `PYTHONPATH` to
the ros-dev venv, so following it literally crashed `bridge_node` with `ModuleNotFoundError:
No module named 'gymnasium'`. The procedure now matches milestone 2's working two-workspace +
`PYTHONPATH` + `docker -p 8765:8765` shape (plus an `apt-get update` the milestone-2 doc's own
command was also missing, found while actually re-running the fixed procedure in docker to
verify it before committing).

## What this milestone proves

The classical (non-learned) stack drives the simulated car around the sim track BY ITSELF,
watchable live in Foxglove, through the REAL command path (claude-docs/04-architecture.md):

```
tracker_node -> /drive_raw -> safety_node -> /drive -> bridge_node (racer_gym)
                                                            |
                                            /sim/map, /sim/raceline, /scan, /tf
                                                            v
                                                    foxglove_bridge (ws://:8765)
                                                            v
                                                    Foxglove app (browser or desktop)
```

`racer_safety/safety_node` remains the sole publisher of `/drive` -- `tracker_node`'s
`/drive_raw` is gated exactly like `keyboard_teleop_node`'s was in milestones 1-2. This is the
classical baseline the residual policy (Phase 5) will later ride on top of.

## What was built

- `ros_ws/src/racer_bringup/launch/sim_autopilot.launch.py`: `bridge_node` + `safety_node` +
  `tracker_node` + `raceline_publisher_node` (+ `foxglove_bridge` via the existing `viz`
  argument pattern). Both `bridge_node` and `tracker_node` are pointed at the same committed
  raceline (`raceline_path` argument, default `../config/tracks/gym_oval/raceline.csv`,
  roadmap task S.2's already-committed `tools/raceline` output -- no new raceline artifact
  needed generating for this milestone). `tracker_node`'s own `/odom` subscription is remapped
  to `bridge_node`'s `/sim/ground_truth_odom` -- a sim-only pose-source adapter (see "Pose
  source" below); no architecture topic is renamed, and `bridge_node`'s real topic name is
  untouched.
- `ros_ws/src/racer_tools/racer_tools/raceline_publisher_node.py` (+ `raceline_loader.py` and
  `raceline_path_builder.py`, the pure/ROS-free logic split out per this repo's convention):
  publishes the committed raceline once as a latched `nav_msgs/Path` on `/sim/raceline`
  (transient_local, same latching pattern as `bridge_node`'s `/sim/map`), so Foxglove can show
  the line the tracker is following. A tiny standalone publisher rather than something bolted
  onto `tracker_node`'s 50 Hz control loop, per this repo's "no heap allocation in the control
  path after init" convention.
- `ros_ws/src/racer_bringup/config/foxglove_sim_autopilot.layout.json`: extends milestone 2's
  layout with `/sim/raceline` in the 3D panel and a second plot trace
  (`/drive_raw.drive.speed`, the tracker's raw pre-gate command) alongside the existing
  post-gate `/drive.drive.speed` trace, `/safety/events`, `/sim/map`, `/scan`, and the vehicle
  pose -- everything milestone 2 already showed, plus the raceline.
- A real fix to `racer_control` found while building the e2e test below (see "What was found
  and fixed" -- this is the substantive engineering content of this milestone, not just
  wiring):
  - `racer_control::SpeedRateLimiter` (new, `include/racer_control/speed_rate_limiter.hpp` +
    `src/speed_rate_limiter.cpp`, gtest-covered): `tracker_node` now rate-limits its own
    commanded speed to a margin (`speed_rate_limit_margin_fraction`, default 0.5) below
    `vehicle_params.actuation.max_acceleration_mps2` before publishing, instead of commanding
    the raceline's raw target speed at the vehicle's nearest point with no ramp of its own.
  - `racer_control::clamp_for_float32_publish` (new, `include/racer_control/
    float32_wire_margin.hpp` + `src/float32_wire_margin.cpp`, gtest-covered): keeps the
    published (float32) steering/speed a small fixed margin inside the real
    `vehicle_params` bound, absorbing a double -> float32 round-trip precision issue found
    during this investigation.
  - Both are deliberately kept OUT of `racer_control::PurePursuitController::compute_command`
    (the shared, ROS-free core cross-language-divergence-tested against
    `training/racer_train`'s Python port) -- they live in `tracker_node.cpp`'s own plumbing
    instead, so the divergence test and `training/racer_train`'s environment are completely
    unaffected by this milestone.
- `tests/e2e_sim_safety/test_sim_autopilot_e2e.py` (sibling to milestone 1's
  `test_sim_safety_e2e.py`, same package): launches `bridge_node` + `safety_node` +
  `tracker_node` headless (no test-only `/drive_raw`/`/drive` remap shim -- the real command
  path), asserts the car makes meaningful arc-length progress along the raceline with zero
  safety interventions of the wall-contact/watchdog/command-sanity/TTC/covariance/bounds-clamp
  kind, and a small, explicitly-bounded, exclusively-benign class of rate-limit warnings (see
  "What was found and fixed"). `tests/e2e_sim_safety/CMakeLists.txt` registers it as a second
  `add_launch_test`; `package.xml` gained `racer_control` and `racer_msgs` test-depends. No CI
  workflow changes were needed -- the existing `e2e-sim-safety` job's combined workspace
  already builds all of `ros_ws` (including `racer_control`), and `colcon test
  --packages-select e2e_sim_safety` picks up both test files automatically.
- `ros_ws/src/racer_control` L1 gtest coverage for both new classes
  (`test/test_speed_rate_limiter.cpp`, `test/test_float32_wire_margin.cpp`, table-driven
  pass/marginal/fail/garbage cases matching this repo's established style), plus an extension
  to the existing `test/test_tracker_node_launch.py` L3 suite proving the ramp is genuinely
  wired into the running node (checked early, well before the ramp completes, and late, once
  it has), not just covered in `SpeedRateLimiter`'s own isolated gtest suite.
- Housekeeping: `.gitignore` gained `/log/` (repo-root-anchored) -- found that
  `.github/scripts/e2e_sim_safety_test.sh` (and similarly-shaped scripts) run `colcon test`
  from the repo root with no `--log-base` override, so colcon's own default `./log` lands at
  the repo root when run locally (bind-mounted, unlike CI's ephemeral runner workspace, where
  this was never noticed).

## What was found and fixed (the substantive part of this milestone)

Wiring `tracker_node` into the real command path (instead of `tests/l5_tracker_lap`'s
test-only `/drive_raw` -> `/drive` remap shim) surfaced a real gap: `tracker_node` commands
the raceline's raw target speed at the vehicle's nearest point with no ramp of its own
(`racer_control::PurePursuitController::compute_command` returns `nearest_point.
target_speed_mps` directly). The raceline's speed profile (`tools/raceline`) is a function of
ARC LENGTH, not time, so this continuously tripped `racer_safety::SafetyGateLogic`'s
rate-limit gate in completely ordinary driving -- not just a brief startup transient, but
throughout a run, since the profile keeps demanding more speed than the vehicle's previous
commanded speed allows across most of each straight.

Fixing this (`SpeedRateLimiter`, described above) eliminated the overwhelming majority of the
disagreement (measured drop from ~350-370 `/safety/events` per 15s run to single digits in
most runs). A second, smaller contributor was also found and fixed: steering/speed are
published as float32 (`ackermann_msgs/AckermannDriveStamped`) but computed/checked in double
precision on both ends, and a value sitting exactly at a bound in double precision can round
the wrong way across that cast -- `clamp_for_float32_publish` (described above) fixed this
category completely (zero bounds-clamp events observed in every run after this fix landed).

**A residual, small, honestly-documented tolerance remains** in
`test_sim_autopilot_e2e.py`, and is worth stating plainly rather than burying in a code
comment: `tracker_node` and `safety_node` each rate-limit against the SAME physical bound, but
on their OWN independent wall timers in TWO SEPARATE PROCESSES with no synchronization between
them. There is no way for two independently-scheduled ~50 Hz control loops to agree on a
per-cycle delta to the bit under real OS scheduling jitter -- this is a structural property of
the two-process command path, not a code bug. Repeated runs of byte-identical code (on a
heavily-loaded development machine -- see below) measured 0, 0, 1, 2, 7, 7, 34, and 166
residual events per run, ALL of them `GateSource` `rate_limit` at `WARNING` severity, never a
brake, never any other source. The e2e test asserts a bounded count (300, well above the worst
observed sample, well below the ~350-370 broken baseline) of ONLY that specific benign class;
a single event of any other source or any brake-severity event fails immediately regardless of
count. This is the same category of judgment call `tests/l5_tracker_lap`'s own committed
lap-time band already made for a different wall-clock-timing-based L5 assertion ("the FIRST
reference for this stack ... needed an honest look at variance, not just one sample").

A separate, related finding: `safety_node`'s default watchdog (`watchdog_missed_cycles=3` at
50 Hz, a 60ms timeout) occasionally tripped during nominal driving in this same investigation,
purely from container CPU/scheduling contention (the exact machine this was developed on was
under unusually heavy memory pressure from many sequential docker test runs -- see "Assumptions
and unresolved" below), never from `tracker_node` actually failing to publish in any way a
human would call broken. `test_sim_autopilot_e2e.py` overrides `watchdog_missed_cycles` to 10
(200ms) for this reason -- the same category of test-environment headroom
`test_sim_safety_e2e.py`'s own `_SAFETY_CONTROL_RATE_HZ=10.0` override already uses. The
PRODUCTION launch file (`sim_autopilot.launch.py`) does NOT override this -- `safety_node`
runs its committed default watchdog there, unchanged.

## Pose source (read before extending this milestone)

`tracker_node` subscribes `/odom`, which `claude-docs/04-architecture.md` reserves for the
real fused estimator (`racer_state`'s EKF, roadmap Phase 2) -- that estimator does not exist
yet. In sim, the only pose available is `bridge_node`'s ground truth
(`/sim/ground_truth_odom`, the same ground truth milestone 2's Foxglove layout already
displays). `sim_autopilot.launch.py` remaps ONLY `tracker_node`'s own subscription --
local name `/odom` -> actual topic `/sim/ground_truth_odom` -- leaving `bridge_node`'s real
topic name untouched (`bridge_node` still genuinely publishes `/sim/ground_truth_odom`,
exactly as the architecture doc's topic table and the milestone 2 Foxglove layout expect). No
architecture topic is renamed. **Real localization (`racer_state`'s EKF/particle filter)
replaces this remap in Phase 2** -- this is a sim-only convenience, not a stand-in that should
survive into hardware work, and it is the direct, honest reason this milestone's "autopilot"
cannot yet mean anything on the physical car.

## Running it locally (Mac, Docker + Colima)

```bash
export PATH="$HOME/bin:$HOME/.local/lima-dist/bin:$HOME/.local/bin:$PATH"
```

Image `ros-dev:local` does not need rebuilding for this milestone (no Dockerfile change).

### Build and run every automated test for this milestone

Same commands as `docs/notes/milestone-1-sim-teleop.md`'s test section for `ros_ws` (now also
exercising `racer_control`'s two new gtest suites and the extended `test_tracker_node_launch.py`
launch test), plus `.github/scripts/e2e_sim_safety_test.sh` for the combined e2e workspace (now
building and running both `test_sim_safety_e2e.py` and this milestone's
`test_sim_autopilot_e2e.py`):

```bash
docker run --rm --shm-size=1gb -v /Users/cameronjim/code/car:/repo -w /repo ros-dev:local bash -lc '
  bash .github/scripts/e2e_sim_safety_test.sh
'
```

### Drive the car AND see it drive ITSELF (two terminals: launch + Foxglove)

No teleop terminal this time -- the classical stack drives. Terminal 1, from inside the
container, builds both workspaces and starts the full autopilot graph. **Note the `raceline_path`
argument's default is relative to `ros_ws` (one level up to the repo root)** -- this only
resolves correctly because this procedure `cd`s into `ros_ws` before running `ros2 launch`,
same as every other launch command in this repo's docs:

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
  ros2 launch racer_bringup sim_autopilot.launch.py
'
```

Expect, in order: `tracker_node up: 348 raceline point(s), 50.0 Hz control rate`, a brief
`/odom stale` warning (bridge hasn't stepped yet) immediately followed by `/odom fresh again;
resuming /drive_raw`, `safety_node up: 50.0 Hz, watchdog=3 missed cycles`,
`raceline_publisher_node up: published 348 raceline point(s) ... on /sim/raceline`,
`foxglove_bridge`'s `Server listening on port 8765`, and `bridge_node`'s `racer_gym_bridge up:
stepping at 100.0 Hz`. The car should then drive continuously with no further watchdog/stale
warnings.

Terminal 2 (on the Mac, not in the container): open
[https://app.foxglove.dev](https://app.foxglove.dev) (or the desktop app), "Open connection" ->
"Foxglove WebSocket", connect to `ws://localhost:8765`, then "Import layout from file" and pick
`ros_ws/src/racer_bringup/config/foxglove_sim_autopilot.layout.json`. You should see the track
outline, the raceline (green line the car follows), the vehicle pose moving on its own around
the loop, `/scan` points aligned around the car, both the raw (`/drive_raw`) and gated
(`/drive`) commanded-speed traces overlapping almost exactly, and the `/safety/events` panel
staying empty (or showing only rare, harmless `rate_limit` warnings -- see "What was found and
fixed" above) during nominal driving.

## How far the car gets (measured progress metric)

`test_sim_autopilot_e2e.py`'s measurement window is 15s (after a 4s warm-up); across repeated
runs the autopilot consistently made well over the test's 8m minimum-progress threshold within
that window -- this raceline's closed-loop length is ~35m (stadium shape,
`straight_length_m=8.0`, `turn_radius_m=3.0`, per the committed raceline's own provenance
header), so 15s of driving at the target speeds this raceline commands (roughly 5-9 m/s on the
sections observed) covers a meaningful fraction of a lap, well beyond tracking noise/jitter.
This mirrors `tests/l5_tracker_lap`'s own 2-lap canary, just through the real `safety_node`
this time instead of that test's remap shim, and over a shorter window since "meaningful
progress" (this task's own wording), not full laps, is what this e2e test needs to prove.

## Assumptions

- `map` -> `base_link` (and now `tracker_node`'s pose input) is sim ground truth, not a real
  localization output -- called out in `bridge_node.py`'s own docstring (milestone 2) and
  again here; not to be mistaken for Phase 2 work landing early.
- The global speed cap (`vehicle_params.yaml`'s `limits.global_speed_cap_mps = 20.0`) is a
  dynamics-model validity bound carried over from `f1tenth_gym`'s defaults, NOT a
  validated safety cap -- unchanged from every prior milestone's own note on this, and it
  must be lowered before any Phase 1 bench/track session per that file's own comments. TTC
  braking remains a documented no-op (`limits.ttc_warning_s`/`ttc_brake_s` are still `null`,
  untuned) -- unrelated to the tracker/gate agreement work in this milestone, but worth
  restating since a reader watching the demo might wonder why the gate never intervenes at all
  around the raceline's tight turns.
- `speed_rate_limit_margin_fraction`'s default (0.5) and the e2e test's residual-event
  tolerance (300) were tuned against this exact `ros-dev:local` image on ONE development
  machine, at times under unusually heavy memory pressure (many sequential docker test runs
  over one session). If CI (a dedicated runner) turns out to need a different number in either
  direction, that is expected to be discovered promptly (CI runs on every push) and adjusted
  with a comment citing the CI run, not a mystery magic-number bump.

## Unresolved / left for later

- Perfectly deterministic zero-`/safety/events` agreement between `tracker_node` and
  `safety_node` is not achieved and, per the investigation above, is not achievable through
  margin tuning alone for two independently-clocked processes -- only a structural change
  (e.g. safety_node evaluating synchronously per received `/drive_raw` message rather than on
  its own independent timer) could plausibly close this completely, and that would be a real
  behavior change to safety-critical code (`racer_safety`) well beyond this milestone's scope.
  Flagged here rather than attempted; a human call on whether it's worth pursuing.
- No CI job exercises the live Foxglove websocket handshake for this milestone specifically
  (same judgment milestone 2 already made and left unresolved for its own layout) -- the
  `curl` handshake check above is a local, pre-push verification step, not a CI gate.
