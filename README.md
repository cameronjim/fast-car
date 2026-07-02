# F1TENTH Autonomous Racing

Autonomous racing software for the [F1TENTH](https://f1tenth.org/) platform, a 1/10 scale
race car with a planar LiDAR and a small onboard computer. The physical car is retired, so
everything here targets the simulator: reinforcement learning policies trained from scratch
against the F1TENTH Gym, classical controllers as the baseline they have to beat, and a ROS 2
demo stack that runs a trained policy in `f1tenth_gym_ros`.

The project is in two halves, and they meet at exactly one place.

```
gym_training/  (plain python, no ROS)          ROS 2 workspace
  trains policies in f1tenth_gym       ->      learned_control/ runs the exported policy
  exports policy.pt + obs_config.json          reactive_control/ classical baselines
                                               f1tenth_gym_ros (external) simulates the car
```

Training happens on the left. Nothing there imports ROS, and the training loop steps the
simulator directly at roughly 8000 physics steps per second, which is what makes racing
policies reachable in under an hour on one GPU. The right side is the demo: it loads a
policy that was trained on the left, plus the `obs_config.json` that tells it exactly how to
rebuild the observation that policy expects. It also keeps the classical controllers and the
older learned stack runnable for comparison.

| Where | What it does |
|---|---|
| [`gym_training/`](gym_training) | SAC and PPO against the F1TENTH Gym API, pure pursuit baseline, raceline generation, policy export |
| [`learned_control/`](learned_control) | ROS 2: runs an exported policy, plus the legacy BC and online SAC nodes, plus the safety node |
| [`reactive_control/`](reactive_control) | ROS 2: gap following, wall following, camera lane following, safety node |

Deep dives:

- [docs/rl-training.md](docs/rl-training.md): observation, action, reward, the speed
  curriculum, residual RL, and the results.
- [docs/reactive-control.md](docs/reactive-control.md): gap following, wall following,
  camera following, the safety node, and the math behind each.
- [docs/learned-control.md](docs/learned-control.md): behavioural cloning and online SAC,
  the v1 approach, kept as a comparison.

## Results

Spielberg, the map every controller has run. Full table, per-map numbers, and the caveats
that make the comparison honest are in
[gym_training/leaderboard.md](gym_training/leaderboard.md).

| Controller | Best lap | Clean | Control rate |
|---|---|---|---|
| pure pursuit on the shipped raceline | 37.99 s | 0% crash, one attempt | 100 Hz |
| SAC trained from scratch (M4) | 37.97 s | 100% over 20 episodes | 25 Hz |
| residual SAC over pure pursuit (M5) | 33.40 s | 100% over 20 episodes | 50 Hz deltas over a 100 Hz planner |
| SAC on the deployable feature set (M6) | 43.43 s | 100% over 20 episodes | 25 Hz, and this is the one the ROS demo runs |

On Monza and YasMarina the shipped racelines run closer to a wall than half the car's width,
so pure pursuit needs a generated line to lap at all: 42.04 s and 50.98 s against the
residual policy's 39.21 s and 43.70 s.

The M6 row is slower on purpose. It gives up `frenet_pose`, which no real car has, and stops
its speed curriculum at 8 m/s rather than the 9.5 m/s the policy can survive, because the
demo needs to finish every lap rather than set a record. Driven live in `f1tenth_gym_ros` the
same policy laps in 63.8 s, mostly because the safety node brakes on time to collision at
every corner entry while the policy's own command never drops below 6.4 m/s.

## Training side quick start

Ubuntu, Python 3.12, a CUDA GPU for SAC. WSL2 works and is what this was developed on.

```bash
python -m venv ~/venvs/f1rl && source ~/venvs/f1rl/bin/activate
pip install torch==2.13.0 --index-url https://download.pytorch.org/whl/cu126
pip install -r gym_training/requirements.txt

cd gym_training
pytest tests/ -q

python -m f1rl.train --config configs/sac_deploy.yaml
python -m f1rl.evaluate --model runs/sac_deploy/best/best_racing_model.zip \
    --config configs/sac_deploy.yaml --episodes 20 --speed-cap 8.0
python -m f1rl.export_policy --model runs/sac_deploy/best/best_racing_model.zip \
    --config configs/sac_deploy.yaml --out-dir artifacts/m6_deploy --speed-cap 8.0
```

Torch must come from the cu126 index. Rendering needs a display, so headless boxes set
`QT_QPA_PLATFORM=offscreen` before anything that renders. Maps download on first use.

`configs/sac_deploy.yaml` is the config to copy for anything that has to run on ROS: it
trains on the features the deploy node can rebuild from `/scan` and `/odom` alone. The other
configs include `frenet_pose`, which is a simulator luxury and makes the export sim-only.
[gym_training/README.md](gym_training/README.md) has the rest of the commands.

## ROS side quick start

Linux with ROS 2, and [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) for the
simulator. That bridge targets ROS 2 Foxy, so its Docker image is the sane way to run it.
Build the image from its README, add `ros-foxy-ackermann-msgs` and a CPU build of PyTorch to
it, which is everything these two packages need that the bridge does not already install,
then put them in the same workspace:

```bash
cd /sim_ws/src
git clone <this-repo>
cd /sim_ws
colcon build
source install/local_setup.bash
```

Run the simulator bridge, then one controller launch beside it:

```bash
# classical baselines
ros2 launch reactive_control gap_follow_launch.py
ros2 launch reactive_control wall_follow_launch.py
ros2 launch reactive_control cv_launch.py

# a policy trained in gym_training
ros2 launch learned_control rl_demo_launch.py \
    policy_path:=/path/to/policy.pt obs_config_path:=/path/to/obs_config.json

# the v1 learned stack, for comparison
ros2 launch learned_control bc_launch.py
ros2 launch learned_control sac_demo_launch.py
ros2 launch learned_control sac_train_launch.py
```

An export dropped into `learned_control/policies/` ships with the package and becomes the
default for `rl_demo_launch.py`, so the two arguments above are only needed to point at
another one.

Every launch takes `sim:=true|false`, which picks `/ego_racecar/odom` or `/odom`. The
physical car is retired, so `sim:=false` is untested legacy.

Under WSL2 the Foxy default DDS hangs on node creation. Installing
`ros-foxy-rmw-cyclonedds-cpp` and exporting `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` fixes
it, and costs nothing anywhere else.

## How the ROS stack keeps itself safe

Every launch runs two nodes: a controller that decides where to go, and a safety node that
has the final say before anything reaches the car.

```
sensors ->  controller  ->  drive command  ->  safety node  ->  /drive  ->  car
                                                    ^
                                                    |
                                                  LiDAR
```

The safety node watches the LiDAR, computes the closest obstacle in the cone the car is
steering into and the time to collision, and brakes in stages. Below the full-brake
threshold it zeroes the command and latches an emergency stop on `/kys`, which the
controllers listen to, then releases it once the hold-off has passed and the forward sector
is clear again. Keeping that in its own node is what makes it safe to put an unpredictable
learned policy in the controller slot.

Topic wiring differs between the two packages, and it matters:

- reactive controllers publish steering on `/drive` and read their allowed speed from the
  safety node on `/speed`.
- learned controllers publish on `/drive_raw`, and the learned safety node republishes the
  gated result on `/drive`.

## Topics

| Topic | Type | Description |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR input |
| `/odom` or `/ego_racecar/odom` | `nav_msgs/Odometry` | odometry (physical or simulator) |
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB camera (vision controller) |
| `/drive_raw` | `ackermann_msgs/AckermannDriveStamped` | controller command before safety (learned) |
| `/speed` | `ackermann_msgs/AckermannDriveStamped` | allowed speed from the safety node (reactive) |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | final command after the safety node |
| `/kys` | `std_msgs/Bool` | emergency-stop flag |

## Configuration

ROS tuning values live in each package's `config/*.yaml` and are loaded by the launch files,
and can be changed while a node runs:

```bash
ros2 param set /safety_node ttc_fb 0.9
ros2 param set /gap_follow_node max_speed 1.2
```

Training runs are configured entirely by one yaml in `gym_training/configs/`. Anything the
trained policy needs at inference time is written into the exported `obs_config.json`, never
copied by hand into the deploy node.

## Dependencies

The ROS packages need `rclpy` and the standard message packages (`std_msgs`, `sensor_msgs`,
`nav_msgs`, `ackermann_msgs`, `rcl_interfaces`), plus `numpy`. The camera controller also
needs `opencv` and `cv_bridge`. Running an exported policy needs PyTorch, CPU is enough.
Training needs the pinned stack in `gym_training/requirements.txt`.

## License

Released under the MIT License. See [LICENSE](LICENSE).
