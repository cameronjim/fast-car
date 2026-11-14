# Sim time and lockstep for the L5 canaries (GitHub issue 156)

## Problem

bridge_node stepped the gym on a wall timer and stamped with the node clock, and the nodes under
test (EKF, slam_toolbox, AMCL, tracker_node) ran on wall time too. Which scan met which odometry
sample, and how far an estimator lagged, depended on CPU scheduling. On a starved 2-core runner:
mapping coverage 0.83 against 1.0, AMCL heading p95 0.144 against 0.12.

## Design

`bridge_node` parameter `sim_time` (default false: nothing changes).

* The bridge owns the clock. `SimClock` counts one env timestep per gym step (integer
  nanoseconds, starting at 1 s so no stamp is 0), `/clock` is published after every step, and
  every message (`/scan`, `/sim/ground_truth_odom`, `/odom/wheel`, TF, the map) is stamped with it.
* The nodes under test run with `use_sim_time:=true`: `odometry`, `mapping` and `localization`
  launch files got a `use_sim_time` argument (default false, so the car is unchanged), and the
  canaries set it for tracker_node and their own node.
* Pacing, two limits on how far sim time may run ahead:
  * `WallPacer`: sim time advances at most `real_time_factor_max` (default 1.0) times wall time.
    This is the cap because tracker_node, safety_node, gap_follow and the planner tick on WALL
    timers by design (their CLOCK POLICY comments), so a sim faster than real time would give them
    fewer commands per sim second than on the car. A real-time factor above 1 is therefore never
    offered to the canaries; a factor below 1 comes from the gate.
  * `LagGate`: sim time may not lead the stamped output of an acknowledging node by more than
    `lockstep_max_lag_s` (0.1 s in the canaries). Keys: an Odometry topic, `tf:<child>[:lead_s]`,
    or `<pkg>/msg/<Type>@<topic>[:lead_s]`. A key is armed by its first message, so a node still
    starting never holds the sim back. A gate shut for `lockstep_timeout_s` (0.2 s of wall time,
    below the controllers' 0.3 s /odom watchdog) opens anyway and counts a timeout, so a dead
    node cannot hang a run. The count is logged at shutdown.
* Why not a strict lockstep (wait for the planner or `/drive` before every step)? The controllers
  are wall-timed, so a `/drive` after a step does not mean it saw that step; and a hard wait on a
  node that has crashed hangs the run. A bounded lag with a timeout keeps the estimators from
  drifting behind, which was the observed failure, without making a stuck node a hung CI job.
* `tracker_node` got `ros_clock_timers` (default false, always false on the car): its control and
  lookup timers tick on the node's ROS clock instead of a wall timer, so with `use_sim_time:=true`
  its commands are tied to `/clock`. Elapsed-time measurement stays on the steady clock. The
  mapping and localisation canaries turn it on and also gate on the tracker's `/drive`
  (`ackermann_msgs/msg/AckermannDriveStamped@/drive`), so the plant does not run ahead of the
  control loop. Without this, a command still reached the plant on a wall timer and the
  trajectory varied with scheduling.
* The runner (`track_family_runner.py`) takes `--sim-time` for the PnP driver: the gate watches
  `/pnp/local_track`, which track_perception_node publishes once per scan with the scan's stamp.
* The canaries measure lap times, stalls and the TF lookup delay in sim time when it is on
  (`L5_SIM_TIME=0` runs the old wall-clock sim). Thresholds, goldens and coverage gates are
  untouched.

safety_node on `use_sim_time:=true`: it measures every elapsed time (staleness, dt, hold timers) on
`steady_clock_` and uses `this->now()` only to stamp `/drive` and `/safety/events`
(safety_node.cpp CLOCK POLICY, `on_timer`), and its timer is a wall timer, so sim time changes
stamps only. It is set only through the runner's parameters file in the test launch; no safety
source or default changed. The PnP canary exercises it end to end.

## Evidence

Stress: the ros-dev image pinned to 2 CPUs (`--cpuset-cpus`), `stress-ng --cpu 2 --cpu-load 100`
inside the same cpuset for each run, mapping canary seed 21, 1.0 m/s. The host was heavily
loaded by other work during the runs and stalled the Docker VM at times (a ROS wall clock jump of
5 to 980 s between two samples; the legacy canary fails those by design, "the run is not valid").

| mode | runs | pass | fail, real | invalid |
|---|---|---|---|---|
| legacy (wall clock) | 8 | 3 | 1 (coverage 0.834, scans dropped by slam_toolbox's full message filter) | 4 host stalls |
| sim time, /odom + tf:odom, lag 0.1, wall-timer tracker | 5 | 3 | 1 (coverage 0.84, a wrong loop closure), 1 shutdown crash of wheel_twist_node (exit -11, test body passed) | 0 |
| sim time, /odom only, lag 0.03 | 5 | 3 | 1 (coverage 0.87), 1 wall contact at the start | 0 |
| sim time + ros_clock_timers tracker + /drive gate (what the PR ships) | 5 | 4 | 1 (coverage 0.843, wrong loop closure) | 0 |

Tried and rejected: acknowledging `/pose` of slam_toolbox or AMCL (published only when the car has
moved, so while it stands still the gate sits on its timeout and sim time crawls: 11 s of sim time
in 150 s), and a lag of 0 on `/odom` (the EKF cannot publish at the current time before the clock
advances: 0 laps in 150 s).

Conclusion: sim time makes a run immune to host stalls and to wall-clock jumps, bounds estimator
lag and ties the tracker's commands to the clock, but under this stress the mapping canary still
builds a bad map in about 1 run in 5. The failing maps all have about 2200 occupied cells against
1800 to 1900 in the good ones: slam_toolbox closes a loop on the plain, featureless lane at the wrong
place. That is inside slam_toolbox (its own threads and the ceres solve), which this change does
not control. The oval and lane AMCL canaries passed in sim time (oval heading p95 0.097 against
0.12; they were not stress-looped). Issue 156 stays open.
