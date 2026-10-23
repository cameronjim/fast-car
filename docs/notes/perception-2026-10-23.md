# Camera perception: detector, LiDAR fusion, person target (roadmap 5.1)

Written 2026-10-23. Software only: **nothing here has run on the Jetson**, no camera frame has
been through the detector, and no weights have been downloaded. Optional and outside the thesis;
nothing in the command path reads these topics and nothing here publishes to `/drive_raw` or
`/drive`.

Package: `ros_ws/src/racer_perception` (ament_python). New messages in `racer_msgs`:
`ObjectDetection` and `ObjectDetectionArray`.

## What it is

```
/camera/usb/image_raw/compressed ──> detector_node ──> /perception/detections (vision_msgs/Detection2DArray)
                                      (YOLO, 10 Hz)            │
/scan ───────────────────────────────────────────────> fusion_node ──> /perception/objects (racer_msgs/ObjectDetectionArray)
                                                                  └──> /perception/target  (racer_msgs/ObjectDetection)
                                  optional: /perception/debug_image/compressed (boxes drawn)
```

- **detector_node** runs a YOLO model through the `ultralytics` package on at most `rate_hz`
  frames per second (default 10, newest frame wins, the rest are dropped). It publishes one
  `Detection2DArray` per processed frame, **empty when nothing was found**, so a consumer can
  tell "nothing there" from "detector dead". Class name is `hypothesis.class_id`, the box is in
  image pixels, the header is the image's header.
- **fusion_node** turns each detection into a vehicle-frame bearing and a LiDAR range.
  - Bearing: pinhole model, `f = (W/2) / tan(hfov/2)`, bearing of column `u` is
    `atan((W/2 - u) / f)` plus the camera yaw offset. Left is positive, 0 is straight ahead,
    same as `racer_safety`.
  - Range: the scan is read in a window of plus/minus half the box's angular width around that
    bearing (scan beams converted to vehicle bearing with `sensors.lidar.mount_yaw_rad`, pi on
    this car, through the generated binding, never typed here). The range is the **median of the
    closest 30 percent** of the valid returns in the window, so the wall behind a person does
    not count. A box narrower than the beam pitch still takes the nearest beam.
  - Flags: with no scan yet, a stale scan (more than `max_scan_age_s`, default 0.5 s, between
    image stamp and scan stamp) or no valid return in the window, the detection is still
    published, with `range_m` NaN, `range_valid` false and `range_status` one of `no_scan`,
    `stale_scan`, `no_returns` (`ok` otherwise).
  - **Target**: the nearest `person` with a valid range (else the highest-score person, still
    marked unranged) goes out on `/perception/target`, held over for `target_hold_s` (1 s) after
    the last sighting with its **original stamp**, and nothing is published when there is no
    target. Consumers must check the stamp. This is nearest-person selection, not identity
    tracking: two people at similar range can swap. Follow-me (5.6) needs an operator lock.

## Parameters worth knowing

| Parameter | Default | Note |
|---|---|---|
| `image_topic` | `/camera/usb/image_raw/compressed` | Switch to `/camera/csi0/image_raw/compressed` for the CSI camera, and change the FOV and width with it. |
| `model` | `yolov8n.pt` | A bare name lives in `model_cache_dir`; a path is used as given. |
| `model_cache_dir` | `~/.cache/racer_perception` | Weights are downloaded here on first run. Never committed. Mount a volume if the container is `--rm`. |
| `device` | `cpu` | `cuda` or `cuda:N`. If CUDA is requested and torch has none, the node refuses (exit 3), it does not fall back. |
| `rate_hz` | 10 | 0 or less means no limit. |
| `score_threshold` | 0.25 | |
| `publish_debug_image` | false | `debug_image:=true` in the launch file. |
| `image_width_px` | 1280 | Must match the width of the images the detector saw (it is not in the detection message). |
| `hfov_rad` | 1.48 | The ELP's lens is about 85 degrees. A datasheet figure, not calibrated; lens distortion is ignored. |
| `camera_yaw_rad` | 0 | No camera mount is measured. Added to the bearing, left positive. |
| `laser_yaw_rad` | 0 | Leave at 0: the value comes from `vehicle_params`. If set it must equal the binding or `fusion_node` refuses to start. `yaw_from_vehicle_params:=false` is for the simulator's vehicle-aligned `/scan` only. |

## Running it

The detector needs `ultralytics` and PyTorch. The default car image (`INSTALL_TORCH=skip`) has
neither, and `detector_node` there logs one error naming the build flags and exits with code 3.
Build the image with `INSTALL_TORCH=required`, `INSTALL_PERCEPTION=required` and both wheel URLs
(`docker/car/README.md`, "Perception"), then run perception **in the car-camera container**,
the one with `--runtime nvidia` that already runs `cameras.launch.py`:

```sh
cd ~/car
mkdir -p ~/perception-cache
docker run --rm -it --name car-perception --network host \
  --runtime nvidia \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/ros_ws/src/racer_bringup/config/fastdds_udp_only.xml \
  -v ~/perception-cache:/tmp/.cache/racer_perception \
  -v "$PWD":/workspace -w /workspace/ros_ws \
  car:perception bash -lc "
    source /opt/ros/humble/setup.bash && source install/setup.bash
    exec ros2 launch racer_perception perception.launch.py device:=cuda debug_image:=true"
```

It can share the car-camera container instead (`docker exec -it car-camera bash -lc ...`): the
flags it needs (`--runtime nvidia`, the UDP-only DDS profile, `--network host`) are the ones
car-camera already has. It does **not** go in car-stack, which is deliberately unprivileged and
free of torch and the GPU runtime, and `car_teleop.launch.py` does not include it. A `colcon
build` of `ros_ws` has to have run in that image first (the runbook's build step); `racer_msgs`,
`racer_tools` and `racer_perception` are the packages this needs. Without internet the first run
cannot fetch the weights: copy `yolov8n.pt` into `~/perception-cache` by hand.

Both containers share the host network and `ROS_DOMAIN_ID`, so `/perception/*` is visible from
car-stack, the Foxglove bridge and the recorder. The recorder's regex does **not** take
`/perception/*` today; record it by hand if wanted:
`ros2 bag record /perception/detections /perception/objects /perception/target /scan`.

Fusion alone (replaying recorded detections, or a different detector):
`ros2 launch racer_perception perception.launch.py detector:=false`.

## Expected load (UNMEASURED)

Nothing has been measured on the Orin Nano. For planning only: YOLOv8n is small (about 8.7 GFLOPs
at 640x640), so the GPU is expected to hold 10 Hz comfortably and the CPU to struggle; the JPEG
decode in `usb_cam`'s compressed transport plus the detector's own decode are CPU costs on top.
The node logs frames per second processed, mean inference time and drops every 10 s. First job
on the car: run `device:=cpu` and `device:=cuda` for a minute each, record those log lines and
`tegrastats`, and write them into the build log.

## Viewing it in Foxglove

Start a bridge next to it (same container, as the LiDAR runbook does):
`ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765`, connect to
`ws://<jetson>:8765`.

- Image panel on `/perception/debug_image/compressed` (needs `debug_image:=true`): boxes with
  class and score drawn.
- Raw Messages or Plot on `/perception/objects`: `detections[0].bearing_rad`, `range_m`,
  `range_status`. A person standing 2 m dead ahead should read bearing near 0 and range near 2.
- 3D panel, display frame `base_link`, `/scan` on: the object's bearing and range should land on
  the person's returns in the ring. Mirrored left and right means the camera yaw or the image
  flip is wrong, not the LiDAR (its orientation was verified 2026-10-06).

## Limits and what is not done

- Camera model is an uncalibrated pinhole with a datasheet FOV and a guessed zero yaw. Expect a
  few degrees of bearing error, more at the image edge. A calibration (checkerboard, then
  `CameraInfo`) and a measured mount are the fix; neither exists.
- Image and scan stamps are compared as they come (`sensors.*.timestamp_offset_s` is unset, the
  sync design is pending), so `max_scan_age_s` is loose by design.
- The LiDAR is a single plane at 0.20 m: it only sees the part of a person at that height, and a
  seated or small object may be missed or ranged as the floor-level thing behind it.
- COCO classes only. The thesis-adjacent apps (5.2 shirt colour, 5.3 pose) need their own models.
- Tests: pure logic (bearing, window, median, yaw rules, tracker, throttle) at L1; the nodes at
  L3 with a **fake detector backend**; the real YOLO path is covered only by a launch test that
  needs `ultralytics` and `RACER_TEST_YOLO_WEIGHTS` and is skipped everywhere else.
