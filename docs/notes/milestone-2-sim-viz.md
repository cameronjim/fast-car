# Milestone 2: live sim view via Foxglove

Status: done. Branch `milestone/2-sim-viz`.

## Standards pre-check

Before starting, confirmed `main`'s latest CI run was green
(`gh run list --branch main --limit 1`: the PR #17 merge run, all jobs passed). No repair
work was needed as part of this PR.

## What this milestone proves

The owner can **see** the simulated car while driving it with the keyboard, instead of
flying blind through milestone 1's headless teleop:

```
keyboard_teleop_node -> /drive_raw -> safety_node -> /drive -> bridge_node (racer_gym)
                                                                     |
                                                     /sim/map, /scan, /tf, /tf_static
                                                                     v
                                                            foxglove_bridge (ws://:8765)
                                                                     v
                                                         Foxglove app (browser or desktop)
```

No safety behavior changes in this milestone (bridge_node and the launch file gain new
publishers/a new optional process; `racer_safety` is untouched), so there is no safety
impact section.

## What was built

- `sim/bridge/racer_gym_bridge` (bridge_node, roadmap task 0.5's package):
  - `/sim/map` (`nav_msgs/OccupancyGrid`, latched via `transient_local` QoS, depth 1):
    published once at startup from the env's own `f1tenth_gym.envs.track.Track`
    (`track.occupancy_map` + `track.spec`), using the standard ROS map_server pixel ->
    occupancy-probability conversion (`racer_gym_bridge.conversions.build_occupancy_grid_fields`)
    -- not a hand-copied map. Works for both the default synthetic reference-line track and
    any raceline-built track (roadmap task S.2's `raceline_path` parameter).
  - TF: `map` -> `base_link` broadcast every step from the same sim ground-truth pose already
    used for `/sim/ground_truth_odom`, plus a static `base_link` -> `laser` transform so
    `/scan` renders aligned. The static transform is identity: f1tenth_gym's LiDAR model has
    no separate extrinsic of its own, and `config/vehicle_params.yaml`'s
    `sensors.lidar` mount offsets are still `null` pending a real Phase 2 measurement
    (roadmap 2.3) -- CLAUDE.md hard invariant 2 forbids inventing a substitute number, so
    identity is what is actually true of the current sim model, not a placeholder.
  - Frame names follow claude-docs/04-architecture.md's REP-105 naming (`map`, `base_link`);
    `map` -> `base_link` direct from ground truth is a documented visualization
    simplification -- there is no localization stack yet (roadmap Phase 2), so this is not a
    stand-in for a real `map` -> `odom` -> `base_link` chain.
  - No existing topic was renamed or remapped.
- `docker/ros-dev/Dockerfile`: added `ros-humble-foxglove-bridge` (apt).
- `ros_ws/src/racer_bringup/launch/sim_teleop.launch.py`: new `viz` launch argument (default
  `true`) starts `foxglove_bridge` on port 8765 alongside `bridge_node`/`safety_node`.
  `viz:=false` disables it (e.g. for a plain headless CI-style run).
- `ros_ws/src/racer_bringup/config/foxglove_sim_viz.layout.json`: a committed Foxglove layout
  -- a 3D panel (map + TF + `/scan` + the vehicle pose from `/sim/ground_truth_odom`) beside a
  speed plot (`/drive.drive.speed`, i.e. the post-safety-gate commanded speed) and a raw
  messages panel (`/safety/events`). Installed to `share/racer_bringup/config/` by
  `racer_bringup`'s `CMakeLists.txt` and also just a plain committed file you can point
  Foxglove at directly from the checked-out repo.
- Tests (claude-docs/12-testing.md L3, extending the package tests bridge_node.py already
  had): `sim/bridge/racer_gym_bridge/test/test_bridge_node.py` gained
  `test_map_is_latched_and_a_late_joining_subscriber_receives_it` (a subscriber created well
  after startup still gets the one-shot map publish),
  `test_map_is_not_delivered_to_a_volatile_subscriber` (the QoS check: a non-transient_local
  subscriber must NOT see it), `test_tf_map_to_base_link_is_broadcast_continuously` (more
  than one `map` -> `base_link` sample on `/tf`, with advancing stamps, while the node
  steps), and `test_static_laser_tf_matches_scan_frame_id` (the static `/tf_static` chain's
  child frame matches `/scan`'s own `header.frame_id`). `test_conversions.py` gained direct
  L1 coverage of `build_occupancy_grid_fields` (binarized black/white pixels, `negate`,
  mid-gray -> unknown, row-major data ordering, origin/orientation pass-through).
  `tests/e2e_sim_safety/test_sim_safety_e2e.py` got one cheap additional assertion
  (`test_sim_map_is_latched_in_the_real_command_path_graph`) since bridge_node is already
  launched there alongside `safety_node` -- proving the map publisher works in the real,
  unmodified command-path graph, not just in isolation.

## Running it locally (Mac, Docker + Colima)

```bash
export PATH="$HOME/bin:$HOME/.local/lima-dist/bin:$HOME/.local/bin:$PATH"
```

Image `ros-dev:local` must be rebuilt for this milestone (the Dockerfile changed):

```bash
docker build -t ros-dev:local docker/ros-dev
```

### Build and run every automated test for this milestone

Same commands as `docs/notes/milestone-1-sim-teleop.md`'s test section
(`colcon build --symlink-install && colcon test --event-handlers console_direct+` for
`ros_ws`, plus the package-specific scripts under `.github/scripts/` for
`sim/bridge/racer_gym_bridge` and the combined e2e workspace) -- unchanged by this milestone,
now also exercising the new map/TF tests above.

### Drive the car AND see it (three terminals)

Terminal 1 starts the sim + safety gate + foxglove_bridge. **The container must publish port
8765**, or Foxglove has nothing to connect to. Note this needs BOTH colcon workspaces built
and sourced -- `racer_gym_bridge` lives in `sim/bridge`, a separate workspace from `ros_ws`
(milestone 1's own doc omits this and will fail to find the `racer_gym_bridge` package if
`sim/bridge` was never built first) -- and `bridge_node` needs the `gymnasium`/`f1tenth_gym`
packages that live in the image's uv venv, not the system Python, so `PYTHONPATH` must
include that venv's site-packages (same as `.github/scripts/sim_bridge_build_test.sh`) or
`bridge_node` dies immediately with `ModuleNotFoundError: No module named 'gymnasium'`:

```bash
docker run --rm -it --shm-size=1gb --name racer-sim -p 8765:8765 \
  -v /Users/cameronjim/code/car:/repo -w /repo ros-dev:local bash -lc '
  source /opt/ros/humble/setup.bash
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

(`viz` defaults to `true`; pass `viz:=false` to skip starting `foxglove_bridge`, e.g. for a
plain terminal-only run.)

Terminal 2, `docker exec -it racer-sim bash` into the SAME running container, runs teleop
(needs its own real TTY, same as milestone 1):

```bash
docker exec -it racer-sim bash -lc '
  source /opt/ros/humble/setup.bash
  source /repo/ros_ws/install/setup.bash
  ros2 run racer_tools keyboard_teleop_node
'
```

Terminal 3 (on the Mac, not in the container): open
[https://app.foxglove.dev](https://app.foxglove.dev) (or the Foxglove desktop app), choose
"Open connection" -> "Foxglove WebSocket", and connect to:

```
ws://localhost:8765
```

Then "Import layout from file" and pick
`ros_ws/src/racer_bringup/config/foxglove_sim_viz.layout.json` from the checked-out repo.
You should see the track outline, the vehicle pose moving as you drive in terminal 2, the
`/scan` points aligned around the car, the commanded-speed plot moving, and any
`/safety/events` (e.g. letting go of the keys for 3 cycles triggers the watchdog brake and an
event) in the raw messages panel.

### Troubleshooting: Foxglove will not connect to `ws://localhost:8765`

- **Port not published**: `docker run` needs `-p 8765:8765` (see terminal 1 above). If the
  container is already running without it, stop and re-run with the flag -- a port cannot be
  published onto a container after it has started.
- **Wrong container / stale container**: `docker ps` to confirm `racer-sim` (or whatever
  `--name` you used) is the container actually running `sim_teleop.launch.py`, and that
  there is only one -- a second, older container still holding port 8765 will make the new
  one fail to bind (`docker run` errors on port already allocated) or silently shadow it.
- **`viz:=false` was passed, or the launch line still says `sim_teleop.launch.py` without
  args**: check terminal 1's log for the `[foxglove_bridge]` process lines (`Starting
  foxglove_bridge ...` then `Server listening on port 8765`); if absent, confirm the launch
  command did not override `viz`.
- **`ros-humble-foxglove-bridge` missing from the image**: if `ros2 launch` errors that
  `foxglove_bridge` package is not found, the image was not rebuilt after this milestone's
  Dockerfile change -- run `docker build -t ros-dev:local docker/ros-dev` again.
- **`bridge_node` died with `ModuleNotFoundError: No module named 'gymnasium'`**: `PYTHONPATH`
  was not set to the ros-dev venv's site-packages before building/sourcing, or `sim/bridge`
  was never built (see the corrected terminal 1 command above) -- `foxglove_bridge` itself
  will still start fine in this case (it only needs the ROS graph, not gym), which can look
  like "it's working" until you notice `/sim/map`/`/scan`/`/tf` never actually appear.
- **`package 'racer_gym_bridge' not found` from `ros2 launch`**: `sim/bridge`'s own
  `install/setup.bash` was never sourced (it is a separate colcon workspace from `ros_ws` --
  see terminal 1 above).
- **Firewall/VPN on the Mac** blocking outbound to `localhost:8765` from the browser tab: try
  the Foxglove desktop app instead of the browser app, which does not route through the same
  browser network stack.

## What was verified locally before pushing

Rebuilt `ros-dev:local` from the changed Dockerfile, then inside it:

- `colcon build` + `colcon test` for `ros_ws` (racer_msgs, racer_safety, racer_bringup) and
  for `sim/bridge/racer_gym_bridge`, and the combined `tests/e2e_sim_safety` workspace -- all
  green, including every new test listed above.
- Started `ros2 launch racer_bringup sim_teleop.launch.py` with `viz:=true` inside the
  container (built per the corrected two-workspace + `PYTHONPATH` procedure above) and
  confirmed `bridge_node` actually came up (no crash) and `foxglove_bridge`'s own log printed
  `Server listening on port 8765` and then advertised channels for `/sim/map`, `/tf`,
  `/tf_static`, `/scan`, `/drive`, `/drive_raw`, and `/safety/events` -- i.e. the new
  publishers are genuinely visible to Foxglove, not just present on the ROS graph.
- **Real websocket handshake, not just a listening socket**: from a second `docker exec` into
  the same container, sent a raw HTTP/1.1 Upgrade request against the bridge with `curl`.
  `foxglove_bridge` requires the `Sec-WebSocket-Protocol: foxglove.sdk.v1` header (found by
  grepping the bridge's own binary for the string in its rejection error -- a bare upgrade
  request without it gets a `400` with `Missing expected sec-websocket-protocol header`, which
  is itself proof the server is live and evaluating the request, not silently dropping it):

  ```bash
  curl --http1.1 -sS -i \
    -H 'Connection: Upgrade' -H 'Upgrade: websocket' \
    -H 'Sec-WebSocket-Version: 13' -H 'Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==' \
    -H 'Sec-WebSocket-Protocol: foxglove.sdk.v1' \
    http://localhost:8765/
  ```

  With that header, the response was a genuine `101 Switching Protocols` followed by
  `foxglove_bridge`'s own protocol-level `serverInfo` message and channel advertisements
  streamed straight over the now-upgraded connection (including a `/tf` channel with the
  `tf2_msgs/msg/TFMessage` schema) -- a full protocol-level handshake and live data exchange,
  not just an HTTP status code.

## Assumptions

- The static `base_link` -> `laser` transform is identity (see "What was built" above) --
  this will need to become a real measured offset once Phase 2 mounts a physical LiDAR
  (roadmap 2.3); nothing in this milestone hand-writes a placeholder number in
  `vehicle_params.yaml`.
- `map` -> `base_link` is sim ground truth, not a real localization output; this is called
  out in both `bridge_node.py`'s own docstring and above so it is not mistaken for Phase 2
  work landing early.

## Unresolved / left for later

- No CI job exercises `foxglove_bridge` itself (the websocket handshake check above is a
  local, pre-push verification step, not a CI gate) -- `12-testing.md` does not name a layer
  for "a demo tool's websocket server starts," and adding one felt like scope creep for a
  visualization-only milestone. If that judgment is wrong, easy to add a path-filtered job
  mirroring the `curl` check above.

## Milestone 5 addendum (2026-08-23)

`foxglove_sim_viz.layout.json`'s panel layout tree changed shape: the right-hand column now
nests a Teleop panel below the existing speed-plot/safety-events pair (three panels stacked in
that column instead of two side by side). See docs/notes/milestone-5-browser-teleop.md for
the new panel and the demo procedure that uses it; everything else this note describes about
the 3D panel, `/sim/map`, TF, and `/scan` is unchanged.
