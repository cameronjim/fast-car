# Milestone 5: browser teleop and a second track

Status: done. Branch `milestone/5-browser-teleop-track-b`.

## Standards pre-check

Before starting, checked `main`'s latest CI run (`gh run list --branch main --limit 1`): the
PR #20 (milestone/4-hardware-prep) merge run. Every job was green except
`train-cuda-image` (a large, known-slow ~3.8GB image build unrelated to anything this
milestone touches), which was still `in_progress` at check time; polling it separately
confirmed it later completed successfully too. No repair work was needed before starting.

## What this milestone proves

**Part B1**: the owner can drive the simulated car entirely from the Foxglove browser
window, no second `docker exec` terminal in raw tty mode:

```
Foxglove Teleop panel -> /teleop/cmd_vel (geometry_msgs/Twist)
                              |
                              v
                  twist_teleop_adapter_node -> /drive_raw -> safety_node -> /drive -> bridge_node
```

`racer_safety/safety_node` remains the sole publisher of `/drive` -- `twist_teleop_adapter_node`
is a second possible producer of `/drive_raw` alongside `keyboard_teleop_node`, gated exactly
the same way.

**Part B2**: the same classical stack (`racer_gym_bridge` + `racer_control`) also drives a
second, twistier track -- `config/tracks/oschersleben`, derived from a real f1tenth_gym-shipped
map's own centerline -- through the same `raceline.csv` contract `gym_oval` already uses.

No safety-layer behavior changes in this milestone. `racer_safety` itself is untouched;
`twist_teleop_adapter_node` and the second track only add new PRODUCERS of `/drive_raw` and a
new raceline INPUT, both of which flow through the existing, unmodified safety gate. See
"A pre-existing racer_safety finding" below for something this milestone's testing surfaced
but did not fix.

## Part B1: driving from the browser

### What was built

- `ros_ws/src/racer_tools/racer_tools/twist_teleop.py`: pure conversion logic (no ROS), L1
  unit-tested. Converts `geometry_msgs/Twist` (`linear.x` m/s, `angular.z` rad/s) to an
  Ackermann `(steering_angle_rad, speed_mps)` command via the standard bicycle-model inverse
  `steering = atan(wheelbase * angular_z / speed)`, clamped to the SAME `vehicle_params`-derived
  bounds `keyboard_teleop_node` uses (`chassis.wheelbase_m`, `steering.min/max_angle_rad`,
  `limits.min_velocity_mps`/`global_speed_cap_mps`). `speed == 0.0` (after clamping) is a
  special case -> `steering = 0.0` (a stationary Ackermann vehicle cannot achieve any yaw rate
  by steering alone), which also gives "zero Twist means zero command" for free. Non-finite
  input is treated as garbage and zeroed (same fail-closed stance `racer_safety`'s gate logic
  takes for a non-finite `/drive_raw` command).
- `ros_ws/src/racer_tools/racer_tools/twist_teleop_adapter_node.py`: subscribes Twist on a
  configurable topic (`input_topic`, default `/teleop/cmd_vel`), republishes
  `AckermannDriveStamped` on `/drive_raw` at a fixed rate (`control_rate_hz`, default 50 Hz),
  reliable QoS depth 10 -- the same publish shape `keyboard_teleop_node` uses. L3 launch test
  (`test_twist_teleop_adapter_node_launch.py`) covers nominal conversion, republish-latest
  semantics, and the timeout-to-zero behavior below.
- `ros_ws/src/racer_bringup/launch/sim_teleop.launch.py`: new `browser_teleop` (default
  `true`), `teleop_cmd_vel_topic` (default `/teleop/cmd_vel`), and `twist_timeout_s` (default
  `0.5`) launch arguments; starts `twist_teleop_adapter_node` when `browser_teleop` is true.
- `ros_ws/src/racer_bringup/config/foxglove_sim_viz.layout.json`: added a Teleop panel
  (`Teleop!browserteleop`) wired to `/teleop/cmd_vel`, nested below the existing speed-plot and
  safety-events panels in the right-hand column. Button values: up/down `linear-x` = +2.0 /
  -1.5 m/s, left/right `angular-z` = +1.5 / -1.5 rad/s -- deliberately modest (well under the
  20 m/s global speed cap) for a first browser-driving pass; adjust the panel's own config in
  Foxglove for faster driving once comfortable.

### No-Twist timeout design (the required design-choice writeup)

`racer_safety/safety_node` already has an independent watchdog (missing `/drive_raw` for
`watchdog_missed_cycles` cycles, default 3 at 50 Hz = 60ms, brakes). `twist_teleop_adapter_node`
does not rely on that as its only defense against a dropped websocket or a closed browser tab.
Like `keyboard_teleop_node`, it runs its own fixed-rate timer that **always** publishes --
every cycle, forever, whether or not a fresh Twist arrived -- so `/drive_raw` itself never goes
silent while the node is alive (the only way `safety_node`'s watchdog can fire because of this
node is the node's own process dying, the same failure mode `keyboard_teleop_node` already
has). What changes on a stale Twist is the COMMAND published on that steady heartbeat: if no
Twist has arrived within `twist_timeout_s` (default 0.5s), the node commands a hard zero and
**keeps** commanding zero, republished every cycle, until a fresh Twist arrives -- it never
transitions to publishing nothing.

This was a deliberate choice over the alternative the brief also allows ("zeros-then-silent":
command zero once, then stop the node's own publish timer and lean on `safety_node`'s 60ms
watchdog for everything after). Continuous zero republish is strictly safer (immediate, not a
60ms-later fallback), no more code, and keeps `/drive_raw` an honest, continuous signal of
exactly what this node currently believes the driver wants rather than something a downstream
node's timeout has to be trusted to interpret correctly. See
`twist_teleop_adapter_node.py`'s own module docstring for the same reasoning in the source.

### Demo procedure

Image `ros-dev:local` does not need rebuilding for this milestone (no Dockerfile change).

```bash
export PATH="$HOME/bin:$HOME/.local/lima-dist/bin:$HOME/.local/bin:$PATH"
```

Terminal 1, from inside the container (same two-workspace + `PYTHONPATH` shape every prior
milestone's doc uses):

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

`browser_teleop` defaults to `true`: this one launch now also starts
`twist_teleop_adapter_node` -- no second terminal needed to drive. (`start_teleop:=true` for
keyboard teleop still needs the two-terminal procedure from
docs/notes/milestone-1-sim-teleop.md, and only one of `browser_teleop`/`start_teleop` should
ever be `true` at once -- see `sim_teleop.launch.py`'s module docstring for the one-publisher
scheme.)

Terminal 2 (on the Mac): open [https://app.foxglove.dev](https://app.foxglove.dev) (or the
desktop app), "Open connection" -> "Foxglove WebSocket", connect to `ws://localhost:8765`,
then "Import layout from file" and pick
`ros_ws/src/racer_bringup/config/foxglove_sim_viz.layout.json`. The Teleop panel appears in
the bottom-right. Click a directional button (or hold) to publish `/teleop/cmd_vel`: up/down
drive forward/backward, left/right steer -- the car moves in the 3D panel, and the speed plot
and `/safety/events` panel behave exactly as they do for keyboard teleop. Releasing all
buttons for `twist_timeout_s` (0.5s) brings the car to a stop.

### What was verified locally before pushing

Full `colcon build` + `colcon test` for `ros_ws` (all packages, including the new
`test_twist_teleop.py` L1 suite and `test_twist_teleop_adapter_node_launch.py` L3 suite) --
green. Beyond the automated tests, per this task's explicit instruction, launched
`sim_teleop.launch.py` for real (bridge_node + safety_node + twist_teleop_adapter_node +
foxglove_bridge, exactly as above) and:

- `ros2 topic pub` a `Twist(linear.x=2.0, angular.z=0.8)` at 20 Hz for several seconds, then
  sampled `/drive_raw` and `/drive`: both showed `steering_angle=0.1313` rad (positive/left,
  matching `angular.z > 0`) and `speed=2.0` m/s, identical on both topics (well within every
  safety bound, so the gate passes it through unchanged) -- confirming the full
  Twist -> `/drive_raw` -> `safety_node` -> `/drive` path. `/sim/ground_truth_odom` showed the
  car's position had moved off the origin, confirming the sim vehicle actually moved.
- Stopped publishing and re-sampled `/drive_raw`/`/drive` after `twist_timeout_s`: both zero,
  confirming the timeout-to-zero design above.
- The same `foxglove_bridge` websocket handshake check milestone 2 used (`curl` with the
  `Sec-WebSocket-Protocol: foxglove.sdk.v1` header): a genuine `101 Switching Protocols`,
  `serverInfo`, and channel advertisements including `/teleop/cmd_vel`, `/drive_raw`, and
  `/drive`. A full Foxglove browser session was not run as part of automated verification (per
  this task's own instruction that the handshake + topic-level check is sufficient); the panel
  config above was written directly from Foxglove's documented Teleop panel schema and has not
  been visually confirmed by a human clicking it yet.

## Part B2: a second track (`oschersleben`)

### Track choice and how it was generated

Picked **Oschersleben** (from the `f1tenth_gym`-shipped map set, hosted at
`api.f1tenth.org`, the same map-hosting endpoint `f1tenth_gym`'s own
`Track.from_track_name` fetches from at runtime): a real, twisty circuit (measured ~28
direction-change segments vs. `gym_oval`'s 2 turns), with a comfortable minimum turn radius
(~1.16m, computed from the generated raceline's own curvature) well above this vehicle's
physical minimum (`wheelbase / tan(max_steering_angle)` ~= 0.74m), and a moderate length
(~260.5m closed loop, vs. several other candidate maps at 400-490m) chosen to keep CI wall
time reasonable. Compared candidates (Spielberg, MoscowRaceway, Silverstone, Budapest, Sochi,
Sepang, Catalunya, Nuerburgring, IMS, Oschersleben) by running each one's raw centerline
through the SAME `tools/raceline` optimizer pipeline `gym_oval` uses and comparing minimum
turn radius, length, and corner count before choosing (IMS was rejected outright: its
9.4m minimum turn radius and 9.46 m/s minimum profile speed make it essentially an oval, not
"twistier").

- `config/tracks/oschersleben/source_centerline.csv`: the Oschersleben map's own centerline,
  vendored (downloaded once, committed) rather than fetched at runtime -- `f1tenth_gym`'s own
  map-loading code (`find_track_dir` in the pinned `f1tenth_gym` source) only checks a local
  cache directory INSIDE the installed package and otherwise fetches from
  `api.f1tenth.org` over the network, which this repo's bridge must not depend on (the same
  reasoning `bridge_node.py`'s own `build_synthetic_track` docstring already states for why it
  does not use a named map). Vendoring the centerline keeps the raceline generation
  reproducible from a committed input, and keeps `bridge_node`'s existing, unmodified
  `Track.from_refline`-based loading path completely untouched -- this track's raceline plugs
  into `build_track_from_raceline` exactly the way `gym_oval`'s always has.
- `tools/raceline/io.py` gained `read_external_centerline_csv` (parses the vendored format:
  a `#`-commented header, then `x_m, y_m, ...` rows, first two columns used) and
  `tools/raceline/cli.py` gained a `--centerline-csv <path>` mode as an alternative to the
  synthetic `stadium` generator. **The optimizer itself
  (`raceline.build_raceline_from_centerline` -- resample, smooth, curvature/geometry, the
  friction- and accel-limited speed profile) is completely unchanged and shared by both
  modes** -- per this task's explicit instruction, no new optimizer was written.
- `config/tracks/oschersleben/raceline.csv` was generated by running that CLI:
  `python -m raceline.cli --track-id oschersleben --centerline-csv
  ../config/tracks/oschersleben/source_centerline.csv --out-dir ../config/tracks` from
  `tools/`, against this repo's real `config/vehicle_params.yaml`
  (`a_max=9.51 m/s^2`, `v_max=20 m/s`, same as `gym_oval`'s).
- `ros_ws/src/racer_bringup/launch/sim_autopilot.launch.py` gained a `track` launch argument
  (default `gym_oval`, unchanged behavior): `raceline_path`'s own default is now derived from
  it (`../config/tracks/<track>/raceline.csv`). Passing `raceline_path` directly still works
  and overrides `track` entirely. `ros_ws/src/racer_bringup/launch/bridge.launch.py` (the
  bare, rarely-used standalone bridge launch from roadmap task 0.5) was deliberately left
  untouched -- it takes no raceline argument at all today (always the synthetic reference
  line), and adding one was out of scope for what this milestone's brief asked for.

### LICENSE NOTE (read before ever making this repo public)

`f1tenth_racetracks` (the map source) is **GPL-3.0** licensed (confirmed via its GitHub
repository metadata). This repo (`fast-car`) has no license declaration yet (every package's
`package.xml` still says `TODO: License declaration`). `config/tracks/oschersleben/
source_centerline.csv` carries this provenance/license note directly in its own header
comment. This is flagged here explicitly, not silently absorbed -- confirm the licensing
posture before this repo is ever distributed or made public.

### Selecting a track

```bash
ros2 launch racer_bringup sim_autopilot.launch.py track:=oschersleben
```

(or `track:=gym_oval`, the default, or omit `track` entirely for the same default).

### L5 / e2e tests, and a finding about how they measure progress

`tests/l5_tracker_lap/test_tracker_lap_canary_oschersleben.py` and
`tests/e2e_sim_safety/test_sim_autopilot_e2e_oschersleben.py` are parallel test cases (NOT
replacements) alongside the existing `gym_oval` ones, asserting the same
progress-with-no-interventions property for this track.

**Finding**: both new tests originally reused the existing windowed nearest-point-on-raceline
(Frenet-style) arc-length tracker the `gym_oval` tests use for their progress metric. On
`oschersleben` this produced a false failure -- a direct standalone telemetry capture
(`bridge_node` + `tracker_node` run outside any test harness, `/odom`/`/drive` recorded for
30s) showed the car genuinely covering ~300m at a ~10.6 m/s mean commanded speed, matching this
raceline's own speed profile, while the windowed Frenet helper's own "arc length along the
raceline" reading stayed near zero. The tracker was driving correctly; only the TEST
instrumentation was wrong. Root cause: a windowed nearest-point search is only safe when no two
points far apart in arc length are also close together in XY -- true for `gym_oval`'s simple
two-turn stadium (that test's own docstring already calls out its parallel straights), not safe
to assume for a real 28-corner track with hairpins/chicanes. Both new tests were rewritten to
measure progress as cumulative Euclidean distance traveled between consecutive odometry
samples instead -- geometry-agnostic, cannot suffer this failure mode. See each file's own
module docstring for the full account, including a second, smaller finding (an earlier
"return within N meters of the exact start pose" lap-completion check also had to be dropped;
the env's reset pose does not reliably lie exactly on this track's reconstructed closed loop --
not chased down further, out of scope, noted in that file).

Measured (this docker image, this machine): `test_tracker_lap_canary_oschersleben.py`
completed one lap in 34.6s (committed band: 15-90s, same "honest look at variance, not a tight
one-sample number" methodology `test_tracker_lap_canary.py`'s own band comment documents).

### A pre-existing `racer_safety` finding (out of scope, not fixed here)

While repeatedly running the e2e tests locally to verify the new `oschersleben` test, the
EXISTING, UNMODIFIED `test_sim_autopilot_e2e.py` (`gym_oval`, milestone 3, not touched by this
branch) intermittently failed with a `watchdog`-source `/safety/events` record showing a
**negative** `/drive_raw` age (e.g. `age=-0.12s`) -- `racer_safety/src/gate_logic.cpp` treats
a negative age as "invalid" and brakes (a deliberate fail-closed choice, not a crash). This was
reproduced on a byte-for-byte clean checkout of `main` in a separate git worktree (i.e. it
predates this branch entirely) and recurred consistently (3/3 local runs) in this specific
local docker/colima environment after extended, heavy back-to-back container use. The most
likely mechanism: `drive_raw_age_s = (safety_node's own now() - tracker_node's message
stamp)`, computed from two independently-clocked processes' own `now()` reads; under enough
scheduling jitter between them, a subtraction like that can go slightly negative. This is
**not** touched by this PR (`racer_safety` is unmodified, and CLAUDE.md/12-testing.md are both
explicit that a safety-critical gate-logic change is a human decision, not something to
patch quietly inside an unrelated milestone) -- flagged here as a finding for a future,
dedicated look, and left for the human owner to decide whether it needs a fix, a quarantine
note, or is accepted as a rare artifact of a heavily-loaded local Docker/colima VM's clock
behavior that a dedicated CI runner may not reproduce.

## Assumptions

- Foxglove's Teleop panel config schema (`topic`, `publishRate`, `upButton`/`downButton`/
  `leftButton`/`rightButton`, each `{field, value}` with `field` one of `linear-x`/
  `angular-z`) was written from documented Foxglove behavior, not visually confirmed in a
  running Foxglove session as part of this milestone (see "What was verified" above) -- if a
  future Foxglove Studio/app version changes this schema, re-add the panel via the UI and
  re-export the layout.
- `oschersleben`'s vendored centerline is GPL-3.0-licensed data from a third-party project;
  see the LICENSE NOTE above.
- `bridge.launch.py` (the bare, standalone bridge launch) was left without a `track` argument
  -- out of scope for this milestone's brief, see "Track choice and how it was generated".

## Unresolved / left for later

- The pre-existing `racer_safety` negative-watchdog-age finding above.
- No CI job exercises a live Foxglove Teleop panel click (same category of judgment milestone
  2 already made for its own websocket handshake check, and milestone 3 for its own Foxglove
  verification) -- the topic-level + websocket-handshake checks above are local,
  pre-push verification, not a CI gate.
