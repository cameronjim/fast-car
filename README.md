# F1TENTH Autonomous Racing

Autonomous driving software for the [F1TENTH](https://f1tenth.org/) platform, a
1/10 scale race car that carries a planar LiDAR, a camera, and a small onboard
computer running ROS 2. The same code runs in the [F1TENTH Gym](https://github.com/f1tenth/f1tenth_gym_ros)
simulator and on the physical car.

The goal of the project is to get the car to drive a track on its own, as fast as
it can without crashing, and to compare two very different ways of doing that:

- **Classical reactive control.** Hand-written controllers that react directly to
  the current sensor reading. No training, no data, no model. They are simple,
  predictable, and easy to reason about.
- **Learning-based control.** A neural network that takes a LiDAR scan and outputs
  steering and speed. It is trained first by copying recorded human driving
  (behavioural cloning) and then improved in simulation with reinforcement
  learning (Soft Actor-Critic).

Both approaches plug into the same safety layer, so they can be swapped without
changing how the car is kept safe.

## What is in here

The code is split into two ROS 2 packages.

| Package | Approach | Controllers | Sensors |
|---|---|---|---|
| [`reactive_control`](reactive_control) | Classical, no learning | gap following, wall following, camera lane following | LiDAR, camera |
| [`learned_control`](learned_control) | Learning-based | behavioural cloning, Soft Actor-Critic | LiDAR |

If you just want the details of how each controller works, jump to the
deep-dive docs:

- [docs/reactive-control.md](docs/reactive-control.md): gap following, wall
  following, camera following, the safety node, and the math behind each.
- [docs/learned-control.md](docs/learned-control.md): data preparation,
  behavioural cloning, Soft Actor-Critic, the reward function, and the formulas.

## How it works

Every setup runs two nodes: a **controller** that decides where to go, and a
**safety node** that has the final say before anything reaches the car.

```
sensors ->  controller  ->  drive command  ->  safety node  ->  /drive  ->  car
                                                    ^
                                                    |
                                                  LiDAR
```

The controller publishes a drive command. The safety node watches the LiDAR,
works out the closest obstacle and the time until a collision, and brakes in
stages if needed. If something is too close it stops the car and raises an
emergency-stop flag on `/kys`, which the controllers listen to. Keeping safety in
its own node means the driving logic can change freely, including an unpredictable
learned policy, without weakening the part that prevents crashes.

The reactive controllers publish steering on `/drive` and take their allowed speed
from the safety node on `/speed`. The learned controller publishes on `/drive_raw`
and the safety node republishes the gated result on `/drive`. Either way the
safety node is the last step before the car.

## Repository layout

```
f1tenth-autonomous-racing/
  reactive_control/    gap following, wall following, camera following, safety
  learned_control/     behavioural cloning + Soft Actor-Critic, safety
  docs/                deep-dive explanations of the algorithms
```

Each package is a standard ROS 2 `ament_python` package.

## Where it runs

This is ROS 2 software, so both the simulator and the car run on Linux (Ubuntu).
There is no Windows or macOS path. The code itself is identical in both places;
the only difference is which odometry topic it reads, which the `sim` launch
argument handles for you.

**Simulator.** The [F1TENTH Gym](https://github.com/f1tenth/f1tenth_gym_ros)
environment (`f1tenth_gym_ros`) is a ROS 2 bridge around the F1TENTH physics
simulator. It runs on a Linux machine, usually inside Docker, and gives you a
virtual car on a track that publishes the same topics as the real one (`/scan`,
odometry, and so on). It publishes ground-truth odometry on `/ego_racecar/odom`.
This is where you develop, train SAC, and test without risking hardware.

**Physical car.** The real F1TENTH car has a small onboard computer running Ubuntu
and ROS 2 (on the standard build this is an NVIDIA Jetson). You SSH into the car,
clone and build this workspace there, and launch with `sim:=false`. The car's own
driver stack provides the LiDAR scan and odometry on `/odom` and turns the final
`/drive` command into motor and steering signals. Your laptop is only a terminal
into the car over the network; the code runs on the car itself.

## Getting started

You need a Linux machine with ROS 2 and a workspace. To run in simulation you also
need [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros); follow its
README to bring up the simulator. To run on the car, SSH into it first and do the
following there.

Clone both packages into the `src/` folder of a ROS 2 workspace and build:

```bash
cd ~/f1tenth_ws/src
git clone <this-repo>
cd ~/f1tenth_ws
colcon build
source install/local_setup.bash
```

Every launch file takes a `sim` argument that picks the right odometry topic:

- `sim:=true` (the default) uses `/ego_racecar/odom`, which is what the simulator
  publishes.
- `sim:=false` uses `/odom`, which is what the physical car publishes.

So the same command runs in either place, you just flip one argument.

## Running the reactive controllers

Each command launches the chosen controller together with the safety node.

```bash
# LiDAR gap following
ros2 launch reactive_control gap_follow_launch.py

# wall following (follows the right-hand wall)
ros2 launch reactive_control wall_follow_launch.py

# camera lane following
ros2 launch reactive_control cv_launch.py
```

Add `sim:=false` to any of them to run on the physical car, for example:

```bash
ros2 launch reactive_control gap_follow_launch.py sim:=false
```

## Running the learned controller

```bash
# run the trained behavioural cloning policy
ros2 launch learned_control bc_launch.py

# run the trained SAC policy (inference only)
ros2 launch learned_control sac_demo_launch.py

# train SAC online in the simulator
ros2 launch learned_control sac_train_launch.py
```

The same `sim:=false` switch applies to `bc_launch.py` and `sac_demo_launch.py`
for the physical car. SAC training is meant for the simulator, since it resets
the car after a crash.

## Training your own models

This repo ships with trained weights, but you can retrain from your own driving
data. The full pipeline is:

```
ROS bag -> extract_dataset -> preprocess -> BC training -> SAC training -> demo
```

1. Record a bag while driving the car, then convert and preprocess it:

   ```bash
   python preprocessing/extract_dataset.py --bag <your_bag> --output training_data.csv
   python preprocessing/preprocess.py
   ```

2. Train the behavioural cloning model:

   ```bash
   python bc/train.py --data processed/data.csv --epochs 100 --batch-size 256 --lr 1e-3 --out bc/bc_model.pth
   ```

3. Initialize a SAC checkpoint from the BC weights, then refine it with
   `sac_train_launch.py`:

   ```bash
   python sac/train.py --bc-weights bc/bc_model.pth --out sac/sac_checkpoint.pth
   ```

See [docs/learned-control.md](docs/learned-control.md) for what each step does and
why.

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

Tuning values for each controller live in that package's `config/*.yaml` and are
loaded by the launch files. You can also change them while the car is running:

```bash
ros2 param set /safety_node ttc_fb 0.9
ros2 param set /gap_follow_node max_speed 1.2
```

## Dependencies

ROS 2 with `rclpy` and the standard message packages (`std_msgs`, `sensor_msgs`,
`nav_msgs`, `ackermann_msgs`, `rcl_interfaces`), plus `numpy`. The camera
controller also needs `opencv` and `cv_bridge`. The learned controller needs
PyTorch, and training needs `pandas`.

## License

Released under the MIT License. See [LICENSE](LICENSE).
