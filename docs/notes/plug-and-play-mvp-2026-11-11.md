# Plug-and-Play MVP: run note for the first car test, 2026-11-11

**Sim only so far. Nothing here has driven the real car.** The Plug-and-Play racer drives a track
it has never seen with no map, no pose and no teleop: LiDAR scan to a local track, an identified
bicycle model, and an MPPI planner whose only output is `/drive_raw`. The command path is
unchanged: planner, `/drive_raw`, `safety_node`, `/drive`, `pwm_output_node`, the hardware RC mux.
The design is in `claude-docs/13-plug-and-play.md` (not in git).

## What was built

| Part | PR | What it is |
|---|---|---|
| Interfaces | 128 | `racer_pnp_msgs` (`LocalTrack`, `PlannerStatus`, `LapEvent`, `LapGuidance`), the model bundle JSON Schema in `config/pnp/`, CI registration of the new packages. |
| A. Track perception | 129 | `racer_track_perception`: scan preprocessing, geometric corridor tracer, numpy runtime for the net, label-free self-check, bundle loader with the refusal table, `track_perception_node`. |
| B. Dynamics | 130 | `racer_pnp_dynamics`: bicycle model reference, bundle export, the `dyn-nominal-v0` fixture used by the sim canary. |
| C. MPPI planner | 133 | `racer_planner`: C++ ROS-free MPPI core, bundle loader with golden rollout check, `mppi_planner_node`, `mppi_bench`. Publishes `/drive_raw` only. |
| D. Lap memory | 131 | `racer_laps` v0. Not used by tonight's launch. |
| Bring-up | 134 | `pnp.launch.py` (starts the two nodes and nothing else, refuses to start without a rosbag recorder in the graph, shuts down if either node exits), `/pnp/.*` in the recorder regex, the `l5-pnp` canary and this note. |
| App | 132 | A `pnp` drive mode in the racer app (whitelisted command, same pre-drive gate, recorder check and STOP ALL). Not needed for the first test: use the launch command below. |

The command path is the one in the architecture, unchanged: `mppi_planner_node` publishes
`/drive_raw`, `safety_node` is the only publisher of `/drive`. The canary asserts both.

## Sim results

`tests/l5_pnp`: bridge_node with the car's LiDAR (vehicle_params mount, 720 beams, 10 Hz, 2 cm
range noise), 50 Hz `/odom/wheel`, `track_perception_node`, `mppi_planner_node` and
`safety_node`, nothing remapped, the planner's cap 0.6 m/s, car `params`, seed 11, 3 laps each,
the dynamics bundle `config/pnp/fixtures/dyn-nominal-v0/` (nominal fixture, not a fit). The
numbers are from one local run per row in the `ros-dev` image on an Apple silicon Mac, real time.
CI runs the same cases in the `l5-pnp` job and prints the same fields.

| Track | Direction | Backend | Laps | Lap times (s) | Safety brakes | Wall contacts | Min body clearance (m) | Planner states |
|---|---|---|---|---|---|---|---|---|
| stadium, lane 1.0 m, straight 2.0 m, island r 0.4 m | ccw | geometric | 3 | 23.39, 23.33, 23.20 | 0 | 0 | 0.232 | DRIVING 3498, NO_ODOM 302 |
| stadium, same | cw | geometric | 3 | 23.28, 23.22, 23.12 | 0 | 0 | 0.230 | DRIVING 3483, NO_ODOM 277 |
| rounded_rect, lane 1.0 m, 3.0 x 2.0 m island, r 0.4 m | ccw | geometric | 3 | 30.12, 30.12, 30.07 | 0 | 0 | 0.235 | DRIVING 4518, NO_ODOM 275 |
| stadium, same as row 1 | ccw | net | 3 | 23.26, 23.25, 23.09 | 0 | 0 | 0.232 | DRIVING 3481, NO_ODOM 208 |

State counts are ticks at 50 Hz counted from process start, so the `NO_ODOM` ticks are the
start-up before the first wheel speed arrives, not driving. No `FAULT` state in any run. `/drive_raw` had one publisher (`mppi_planner_node`) and `/drive` one (`safety_node`).
For scale: the raceline tracker on the same stadium at 0.64 m/s laps in about 15 s with a map
and a pose; this driver has neither and is slower, which is expected at a 0.6 m/s cap with a
stop-within-sight terminal term.

## Jetson procedure

Everything here is a command to type on the car. Stop at the first thing that does not match.

1. Pull and copy the bundles. On the Jetson host:

   ```sh
   cd ~/car && git pull origin main
   mkdir -p ~/car/data/pnp/models
   # from the dev machine, one directory per bundle:
   rsync -a /Users/cameronjim/code/car-data/pnp/models/<perception model_id> jetson:~/car/data/pnp/models/
   rsync -a /Users/cameronjim/code/car-data/pnp/models/<dynamics model_id> jetson:~/car/data/pnp/models/
   ```

   Tonight is geometric, so only the dynamics bundle is needed on the car. The dynamics bundle is a fit on real bags. `config/pnp/fixtures/dyn-nominal-v0/` is a test
   fixture, not a fit: do not drive on it.

2. Build, inside the `car-stack` container:

   ```sh
   docker exec -it car-stack bash -lc '
     source /opt/ros/humble/setup.bash && cd /workspace/ros_ws &&
     colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
       --packages-up-to racer_bringup racer_planner racer_track_perception racer_pnp_msgs'
   ```

   Release matters: a Debug build of the planner will not meet the solve budget.

3. Planner timing. Budget: p50 under 6 ms, p99 under 12 ms.

   ```sh
   docker exec -it car-stack bash -lc '
     source /opt/ros/humble/setup.bash && source /workspace/ros_ws/install/setup.bash
     ros2 run racer_planner mppi_bench --bundle /workspace/data/pnp/models/<dynamics model_id>'
   ```

   Write p50 and p99 in the table below. Over budget: stop. The planner halves its sample count
   on a slow solve, but a budget miss here means the 768 sample plan is not what will drive.

4. Wheels off the ground (L6). Start the stack with the recorder (`car_teleop.launch.py
   lidar:=true vesc:=true`, no browser teleop, recording on, as in the first-boot runbook), then
   in a second terminal:

   ```sh
   docker exec -it car-stack bash -lc '
     source /opt/ros/humble/setup.bash && source /workspace/ros_ws/install/setup.bash
     exec ros2 launch racer_bringup pnp.launch.py \
       dynamics_bundle:=/workspace/data/pnp/models/<dynamics model_id> \
       backend:=geometric v_max_mps:=0.4'
   ```

   The launch refuses to start with "refusing to drive: no rosbag recorder in the graph" if the
   recorder is not running. That is the guard working, not a bug. Then:
   - Hold a board 0.5 m to the left of the car: the front wheels must turn right. Hold it 0.5 m
     to the right: they must turn left.
   - `ros2 topic echo /pnp/planner/status` shows `state` and `cmd_speed_mps`. `cmd_speed_mps`
     must never exceed 0.4.
   - Ctrl-C the launch: the wheels must stop within the safety watchdog time.
   - Check the kill knob.

5. Floor. Two people, kill knob in hand, `v_max_mps:=0.5`, `backend:=geometric` (the default).
   The net failed the real-bag gate, so it stays off the car; `backend:=net` remains selectable. Raise the cap in
   steps of 0.1 only after a clean run, never above the dynamics bundle's
   `io.validity.v_max_mps`.

6. After the run: copy the bag off the car, then fill in the table at the end of this note: lap
   times, brake count and the planner states seen.

## Car test log

| Run | Backend | Cap (m/s) | Laps | Lap times (s) | Safety brakes | Planner states seen | Notes |
|---|---|---|---|---|---|---|---|
| | | | | | | | |

`mppi_bench` on the Orin: p50 ___ ms, p99 ___ ms.

## Pre-flight with the fitted bundle (sim, 2026-11-12)

`dyn-v0-2026-11-11` loads (golden max error 5e-9, vehicle_params 0.14.0). `mppi_bench --speed 0.5`
on an Apple silicon Mac: p50 0.90 ms, p99 1.02 ms, max 1.04 ms. Closed loop, geometric backend,
`v_max_mps` 0.5, stadium, island r 0.4 m, 3 laps, sim car actuating with the gym's own plant
(steer gain 1, not the fitted 0.77):

| Lane | Nominal fixture | Fitted bundle |
|---|---|---|
| 1.3 m | laps, 28.5 s, clearance 0.28 m | laps, 27.5 to 28.3 s, clearance 0.22 m |
| 1.0 m | laps, 28.7 s, clearance 0.21 m | stalls before lap 1 (BLOCKED, safety brakes), no wall contact |
| 0.85 m | stalls (BLOCKED) | stalls |

With the sim car also actuating as the fitted bundle says (steer gain 0.77, accel 0.3 m/s2),
the fitted bundle stalls at lane 1.3 m too, but laps lane 1.0 m with island r 0.8 m. So the
fitted gain and acceleration limit make the island ends the hard part. Expect `BLOCKED` and a
stationary car at the ends of a 1.0 m oval; it stops, it does not hit the wall.

## What is not verified

- Nothing has run on the car. Every number above is simulation. The sim LiDAR is a clean ray cast
  plus 2 cm noise: no dropouts, no foam clutter, no motion distortion.
- The dynamics bundle in the sim canary is the nominal fixture. A bundle fitted on the real bags
  has not been checked here; its validity range bounds the speed cap.
- Solve time on the Orin is unmeasured (`mppi_bench` step above). The Mac numbers say nothing
  about it.
- The perception net failed the label-free gate on the real 2026-11-11 bags, so it does not
  drive the car. It passes the sim canary above only. `backend:=net` stays selectable.
- The geometric tracer and the net were only run on two track families (stadium, rounded
  rectangle). The real foam oval has 0.8 to 1.3 m lanes, narrower than the sim lane of 1.0 m in
  places.
- Reverse and recovery: v0 has none. If the planner reports `BLOCKED` the car stops and stays
  stopped.
- Lap memory (part D) is off: `pnp.launch.py lap_memory:=true` adds it (needs the map to base_link TF) and caps at the dynamics bundle validity. Tonight is A+B+C only.
- The app button is tested against a fake docker layer only.
