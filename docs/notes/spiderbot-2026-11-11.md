# Spiderbot groundwork: GPU image, pose perception, patrol behaviour (2026-11-11)

Built overnight with the car off and untouched. **Nothing here has run on the Jetson or the car.**
The GPU image was built and the models were run on CPU on the Mac; the behaviour was proven in the
sim with synthetic detections. No camera frame has been through the detector, CUDA has not run, and
the car has never driven a patrol. Roadmap 5.1 and 5.3 are `[~]`, not `[x]` (lines to tick, below).

## What works

| Piece | State |
|---|---|
| GPU car image (`docker build --build-arg INSTALL_TORCH=required --build-arg INSTALL_PERCEPTION=required`) | Builds on the Mac (aarch64, same `l4t-jetpack:r36.4.0` digest). torch 2.8.0 (CUDA 12.6 build), torchvision 0.23.0, ultralytics 8.3.40, opencv import. Wheels and weights pinned by URL and sha256. A wrong hash and perception-without-torch both fail the build (tested). |
| Detector, pose mode | `detector_node mode:=pose` (YOLOv8n-pose): `/perception/poses` (new `racer_msgs/PoseDetectionArray`, 17 COCO keypoints per person) plus the unchanged `/perception/detections`. `fusion_node input:=poses` (automatic with `perception.launch.py mode:=pose`) puts keypoints on `/perception/objects` and `/perception/target`. |
| Real models on real photographs | Three public domain photographs (`ros_ws/src/racer_perception/test/fixtures/images`, attribution beside them). Pose finds the hiker with both arms up, the airman with arms out, and all four people in the group photo. Keypoints recorded to JSON and used as rule fixtures. |
| Pose rules | `racer_apps/spiderbot/pose_rules.py`: `person_present`, `arms_raised`, `one_arm_raised`, `arm_swing` (wrist speed in body scales per second, the upper-body half of a punch). Pluggable: `register_rule(name, factory)` or `rule:=pkg.module:func`. Held over N consecutive frames by `HeldForFrames` (default 3, gap-reset). |
| Behaviour | `racer_apps` (new package), `spiderbot.launch.py`, off by default and not in `car_teleop.launch.py`. Patrol, stop, face, alert, hold, resume (below). |
| Sim proof | `tests/l5_spiderbot` (CI job `l5-spiderbot`): bridge, the real `safety_node`, the tracker on a patrol generated from waypoints, the spiderbot, a fake detector standing a person with raised arms beside `gym_oval`. Passes in 19 s: state sequence patrol, stopping, facing, hold, patrol; one alert with `facing_ok`; stands still for the hold; drives off; no second alert inside the cooldown. |
| App | Perception card and a patrol row in the Driving card, both disabled with a reason when the GPU image is not on the car (see `racer-app-2026-11-11.md` addendum). |

## What the behaviour does

```
tracker (waypoint loop, /spiderbot/patrol_cmd) -> spiderbot_node -> /drive_raw -> safety_node -> /drive
camera -> detector_node (pose) -> fusion_node (+ LiDAR range) -> /perception/objects -> spiderbot_node
```

1. **PATROL.** `spiderbot.launch.py` reads a patrol YAML (closed loop of map-frame waypoints), refuses
   it if it has fewer than three points, coincident points, or a corner tighter than the car can turn
   (curvature limit from vehicle_params, with a 0.75 margin), threads a smooth curve through it and
   writes a raceline that `racer_control`'s tracker follows on the AMCL pose, with the tracker's
   `/drive_raw` remapped to `/spiderbot/patrol_cmd`. `spiderbot_node` passes that command on, ramped
   below the safety gate's speed and steering rate bounds. If the patrol command goes silent it
   publishes nothing and `safety_node`'s own watchdog acts (the tracker's policy).
2. **Trigger.** The pose rule (`rule:=arms_raised` by default) must hold on 3 consecutive detector
   frames. One misdetection does nothing.
3. **STOPPING** (until slow or 1 s), then **FACING**: the car cannot turn on the spot, so it creeps
   forward at 0.3 m/s steering towards the fused bearing until within 0.12 rad for 0.3 s. It will not
   creep when the range is unknown or under 1 m, and gives up after 1 s unseen or 5 s total. The
   alert goes out either way, with `facing_ok` saying which.
4. **ALERT.** One `/spiderbot/alert` (`racer_msgs/SpiderbotAlert`: rule, held frames, bearing, range,
   facing result, map pose from TF) and a JSON line in `<output_dir>/spiderbot_events.jsonl`, plus a
   snapshot JPEG of the newest camera frame in `<output_dir>/snapshots/`. Every transition is logged,
   not only alerts.
5. **HOLD** for 10 s, then **resume** with a 20 s cooldown during which triggers are ignored.
6. If the detector goes quiet for 2 s the car stops (`require_perception`, default true) and resumes
   once it is back for 0.5 s.

Safety: the node publishes `/drive_raw` only (a source test and an L3 test assert nothing in
`racer_apps` publishes `/drive`; the L5 test asserts `/drive` has one publisher, `safety_node`, and
`/drive_raw` one, `spiderbot_node`). Vehicle limits come from the generated binding, none are typed
in the node or the launch file (a test greps for the old literals). Patrol speed is capped at 1.0 m/s
in the launch file on top of the vehicle limits. The mux, kill knob and layers 1 to 4 are untouched.

## Perception numbers (CPU, measured here)

Mac (Apple silicon, Docker in a colima Linux VM, 10 vCPUs), `car:perception`, torch 2.8.0 CPU path,
`python3 -m racer_perception.bench`, three 640 px photographs, 20 to 30 runs each after 3 warm-ups.
These numbers say nothing about the Jetson CPU.

| Task | Threads | Median per frame (JPEG decode, preprocess, model, postprocess) | p95 |
|---|---|---|---|
| pose | torch default (8) | 326 ms | 404 ms |
| pose | 2 | 94 ms | 116 ms |
| pose, 1280 px input | 2 | 75 ms | 104 ms |
| detect | 2 | 89 ms | 115 ms |

The model itself is almost all of it (inference about 70 to 90 ms of that, pre and post 2 to 3 ms).
More threads was **slower** in the VM (8 threads: 3.5 times the 2-thread time), so `OMP_NUM_THREADS`
matters on the Jetson's CPU too: measure `device:=cpu` there at 2, 4 and 6 before trusting any CPU
number.

**Jetson GPU: an estimate, not a measurement.** Ultralytics' Jetson guide lists a PyTorch (not
TensorRT) YOLO26n at 15.6 ms per image on the Orin Nano Super at 640 px, inference only, without
pre and post processing (https://docs.ultralytics.com/guides/nvidia-jetson/, read 2026-11-11; it has
no pose benchmark and no YOLOv8n entry). YOLOv8n-pose is the same size class. For planning: 15 to 25
ms of model time on the GPU, plus about 10 ms of JPEG decode and letterboxing on the CPU for a
1280x720 frame, so roughly 25 to 40 ms per frame, comfortably above the detector's 10 Hz cap, and
slower on a non-Super Orin Nano (lower clocks). TensorRT stays out of scope (CLAUDE.md).

## Image build (the Mac, colima aarch64)

| | |
|---|---|
| Base | `nvcr.io/nvidia/l4t-jetpack:r36.4.0@sha256:34ccf0f3...` (3.4 GB compressed, pulled once) |
| Wheels | torch 2.8.0 (226 MB) and torchvision 0.23.0 from the Jetson AI Lab `jp6/cu126` index, sha256 in the Dockerfile. The NVIDIA tree has one JetPack 6.1 torch wheel (2.5.0a0) and no torchvision. |
| Image size | skip build 18 GB, perception build 19.9 GB (+1.9 GB) (`docker images`, uncompressed; the Jetson's earlier 10.2 GB figure was before the localisation and camera layers) |
| Build time | 357 s (6 min) from `--no-cache` with the base image already local, whole image; the two new layers alone take about 75 s when the layers below are cached |
| Verified | imports; `ldd` of every torch library finds everything except `libcuda.so.1`, which the NVIDIA runtime supplies on the Jetson; hadolint clean |
| Not verified | `torch.cuda.is_available()`, any CUDA kernel, the 6.1 userspace on the 6.2.1 host (still the open question in `docker/car/README.md`), build time on the Jetson itself |

Two things the first attempts found, both fixed in the Dockerfile: the image has no `pip3` and no
`libopenblas0` (torch will not import without it), and `ultralytics` imports `requests`, which was
not in the image either (the old pin list also missed matplotlib's and pandas' own dependencies;
`perception-requirements.txt` is now a complete resolution, `uv pip compile` for aarch64 and Python
3.10).

**Licence flag.** Ultralytics' code and these weights are AGPL-3.0. The repository declares no
licence yet (`TODO: License declaration` in every `package.xml`). Fine for private use on one car;
a decision before the repository or the weights are shared. Not something to settle here.

## What is untested

- Everything on the car: no camera frame, no CUDA run, no Jetson build, no AMCL-localised patrol.
- The sim test uses the bridge's ground truth for the tracker (the sim-only adapter
  `sim_autopilot.launch.py` uses) and a fake detector. The real chain camera to detector to fusion
  to spiderbot has never run end to end; `fusion_node` has never seen a real LiDAR return behind a
  person.
- Real-world robustness of the pose rules: tested on synthetic bodies and on three photographs. No
  video, no occlusion, no children, no wheelchairs, nothing seen from the camera's mounting height.
  `arm_swing` is a first version and its threshold (4 body scales per second) is a guess.
- The camera is uncalibrated (datasheet field of view, zero yaw, no mount measured): bearing is good
  to a few degrees at best, and facing tolerance 0.12 rad (7 degrees) is about the same size. The
  facing manoeuvre may need a looser tolerance until the camera is calibrated.
- Facing steers towards a person who may be 1 to 3 m away. The LiDAR sees one plane at 0.20 m and
  `safety_node` brakes on what it sees; a person standing sideways or a child may not be in it.
- A fixed scan-and-camera clock offset is not modelled (`sensors.*.timestamp_offset_s` is null).
- The alert only logs and publishes. There is no actuator and no operator confirm step (roadmap 5.3
  asks for both).

## Next session, in order (nothing arms the car until step 7)

1. **Build the GPU image on the Jetson** (about an hour with the 3.4 GB base pull on eMMC; run it
   under `nohup`):
   ```sh
   cd ~/car && git pull
   nohup docker build --build-arg INSTALL_TORCH=required --build-arg INSTALL_PERCEPTION=required \
     -t car:perception docker/car > ~/car-perception-build.log 2>&1 &
   ```
   Then the first CUDA check, and the CPU tests of the real model on the Jetson's own CPU:
   ```sh
   docker run --rm --runtime nvidia car:perception bash -lc \
     'python3 -c "import torch; print(torch.__version__, torch.cuda.is_available(), torch.cuda.get_device_name(0))"'
   IMAGE=car:perception tools/perception_cpu_tests.sh bench
   ```
   Record the image size and build time in `docker/car/README.md`. If `is_available()` is False on
   the 6.2.1 host, that is the JetPack gap from `docker/car/README.md` ("Base image tag vs. the
   device"), not a wheel problem.
2. **GPU latency, measured.** In the perception container, device cuda, run the bench on the test
   photographs (`--device cuda`, same script, `--resize-width 1280`), then `tegrastats`. Replace the
   estimate above.
3. **Mount the camera properly.** Rigid, level, looking straight ahead, as high as is sensible for
   seeing standing people at 1 to 4 m, with a measured x, y, z and yaw from `base_link`. Write it in
   `vehicle_params` (a schema addition: needs the owner, `claude-docs/06-vehicle-params.md`), then
   `camera_yaw_rad` stops being a launch argument.
4. **Calibrate the camera with a checkerboard.** `ros-humble-camera-calibration` on the compressed
   topic (print a 9x6 board, 25 mm squares, flat on a board), producing `CameraInfo`. Use it for
   `hfov_rad` and `image_width_px` now; a distortion-aware bearing is a follow-up in `fusion.py`.
   Do the USB camera first (it is the one `perception.launch.py` defaults to); the CSI IMX219 needs
   its own `image_topic`, width and field of view.
5. **Record a patrol map.** Drive the loop with the car localising (mapping session, then save the
   map; `docs/notes/first-boot-runbook.md` "First mapping session"), note waypoints from `/pose` or
   Foxglove, write `data/patrols/<name>.yaml` (see `ros_ws/src/racer_apps/config/patrol_example.yaml`).
   `spiderbot.launch.py` will name any corner the car cannot make.
6. **Wheels off the ground first** (CLAUDE.md two-person rule for new control code), nothing armed:
   `colcon build`, start the stack with localisation, cameras, perception (the app's buttons or
   `docker run` as in `docker/car/README.md`), then the spiderbot launch with `speed_mps:=0.3`. Stand
   in front of the camera with both arms up and watch `/spiderbot/state` and the alert; confirm
   `/drive_raw` goes to zero speed through `safety_node` on the bench.
7. **Floor session**, driver on the RC kill knob: low speed, a short loop, one volunteer. Tune
   `rule_frames`, `face_tolerance_rad`, `hold_s` from the bag (the recorder now keeps
   `/perception/*` and `/spiderbot/*`).
8. Open items for the owner: the AGPL question, an actuator and its operator confirm step, whether
   `arm_swing` should become a real punch and kick classifier (needs video of the motions, not
   photographs).

## Roadmap lines to tick (`claude-docs/01-roadmap.md` is not edited here; the tracked copy,
`docs/notes/roadmap-phase5-applications.md`, is)

- 5.1 Camera perception: `[~]`. Done since 2026-10-23: image builds, models run on CPU, pose mode,
  pinned weights, fusion carries keypoints. Left for `[x]`: Jetson build, CUDA run and latency, a
  real camera frame, training or choosing a model for the apps' needs, bags from the real cameras.
- 5.3 Spiderbot patrol: `[~]`. Done: patrol on waypoints, pose rule held over frames, stop, face,
  alert, resume, sim-tested; app launch. Left: camera and patrol map on the car, a real temporal
  punch and kick rule, the actuator, the operator confirm step.
- 5.6 Follow-me: stays `[ ]`. The facing code and `/perception/target` are parts of it; no following.
- Not ticked: 5.2, 5.4, 5.7, 5.8.
