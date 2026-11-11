# Camera lap counter (2026-11-12)

**Nothing here has run on the car or the Jetson, and none of the logged bags has a camera topic**
(checked all seven in `car-data/bags`: `/odom/wheel` yes, `/camera/*` no), so the defaults below
are reasoned from the synthetic tests and the 5.2 m loop, not tuned on real frames. The first job
on the car is one logged run with a camera, then the replay tool below.

## What it is

The owner's idea, as built: remember what the camera saw when the run started, and call it a lap
each time the camera sees that again. No markers, no map, no pose.

```
ref  = unit(thumbnail of the reference frame - its mean)
sim  = dot(unit(thumbnail of this frame - its mean), ref)       # cosine similarity, -1 .. 1
```

That is the normalised cross-correlation at zero shift. Removing the mean and the norm makes
`sim` blind to gain and offset (auto exposure, a darker room): the similarity of `a*x + b` with
`x` is exactly 1 for `a > 0`. A thumbnail is 64 x 48 gray, area-averaged from a JPEG that is
decoded at 1/4 scale (`cv2.IMREAD_REDUCED_GRAYSCALE_4`, so 1280x720 comes out 320x180), then a
[1 2 1] blur. Only numpy and `cv2.imdecode` are used.

Where it lives:

| Piece | File |
|---|---|
| Counter (pure numpy, no ROS, no cv2) | `ros_ws/src/racer_perception/racer_perception/lap_counter.py` |
| JPEG decode (the only cv2 use) | `racer_perception/lap_frames.py` |
| Node | `racer_perception/lap_counter_node.py`, launch `launch/lap_counter.launch.py` |
| Bag / image-folder replay | `tools/lap_replay.py` |
| App | a "Laps" gauge in `tools/racer_app/web` (`model.ts`, `live.ts`, `Dashboard.tsx`) |

It sits in `racer_perception` (a camera node, next to the detector) rather than `racer_tools`.
`racer_perception` is already the optional camera package and stays out of `car-stack`'s default
path. The replay CLI is in `tools/` next to `replay_follower.py`, because that environment
already has `mcap` and `mcap-ros2-support`; it imports the same counter and the same decoder, so
a tuned parameter set transfers to the node unchanged.

## The lap decision

1. **Waiting.** No reference yet. The first frame after the speed reaches `start_speed_mps`
   (from `/odom/wheel`) becomes the reference, or the next frame after a reset. Lap 0 starts there.
   A blank frame (pixel std below `min_reference_std`) is refused.
2. **Armed.** After the start and after every counted lap the counter is disarmed until the
   similarity has dropped below `rearm_threshold`. Standing at the start, or creeping past it, can
   never count.
3. **Window.** While armed, a frame at or above `similarity_threshold` opens a window, but only if
   `min_lap_time_s` and `min_lap_distance_m` (odometry path length) have passed since the lap
   started. A bright stretch that starts before the gates pass is thrown away whole, even if a gate
   opens in the middle of it.
4. **Peak.** The lap is recorded when the window closes (similarity back under the threshold, or
   `peak_window_max_s` elapsed, for a car parked on the line), stamped at the frame with the
   highest similarity inside it. Lap 1 is timed from the reference capture, so it is the least
   exact; later laps are peak to peak.

Heading turns (yaw rate from `/odom`, integrated per lap, 1.0 = one left turn) are reported with
each lap and in the status. They are a cross-check only: they never count or veto a lap. `null`
when `/odom` is not published (for example without the EKF).

Time is the node's ROS clock at message arrival (not image stamps), so a camera restart with a
jumped stamp cannot disturb it, and `use_sim_time` works.

## Topics

| Topic | Type | |
|---|---|---|
| in `/camera/usb/image_raw/compressed` | CompressedImage | param `image_topic`; best_effort, depth 1 |
| in `/odom/wheel` | Odometry | param `odom_topic`; `twist.linear.x` speed, `pose.position.x` along-track distance (path length is the sum of `abs(delta x)`) |
| in `/odom` | Odometry | param `yaw_rate_topic` (`''` disables); `twist.angular.z` |
| out `/lap/event` | String, JSON | one per lap: `lap, lap_time_s, best_lap_s, peak_similarity, crossing_time_s, distance_m, heading_turns` |
| out `/lap/status` | String, JSON | every `status_period_s` (1 s): `state, similarity, laps, last_lap_s, best_lap_s, armed, speed_mps, distance_m, heading_turns, odom_seen` |
| srv `/lap_counter/reset` | std_srvs/Trigger | forget the count; the NEXT frame is the new reference (call it with the car at the start line) |

Both outputs are in the recorder regex of `car_teleop.launch.py` (`/lap/(event|status)`), so a
run's laps are in its bag. The reference thumbnail is not published (it is in the replay tool's
trace instead). Nothing publishes to `/drive` or `/drive_raw`, and nothing reads these topics on
the command path.

## Running it

```
ros2 launch racer_perception lap_counter.launch.py                       # alone
ros2 launch racer_bringup cameras.launch.py lap_counter:=true            # with the cameras
ros2 launch racer_bringup cameras.launch.py lap_counter:=true lap_similarity_threshold:=0.85
ros2 service call /lap_counter/reset std_srvs/srv/Trigger
```

Default off: `cameras.launch.py` is unchanged unless `lap_counter:=true`. The node needs `cv2`
(the base car image has cv2 4.5.4, checked in the local `car:base-skip` image; the node exits
with code 3 and one error line if it is missing) and the camera and `/odom/wheel` topics
visible on the same ROS domain (the car-camera and car-stack containers share it).

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `similarity_threshold` | 0.80 | count a lap at a peak at or above this |
| `rearm_threshold` | 0.55 | after a lap similarity must first drop below this |
| `min_lap_time_s` | 6.0 | shortest possible lap (the loop is 17.6 s counter-clockwise) |
| `min_lap_distance_m` | 3.0 | shortest possible lap in odometry metres (loop 5.2 m centreline, the car drives wider) |
| `start_speed_mps` | 0.3 | reference taken at the first frame at or above this; `<= 0`: first frame |
| `rate_hz` | 15.0 | frames processed per second, the rest dropped (`<= 0`: all) |
| `decode_reduction` | 4 | JPEG decode scale 1/N, N in 1, 2, 4, 8 |
| `thumb_width_px` / `thumb_height_px` | 64 / 48 | thumbnail the similarity is computed on |
| `blur` | true | 3x3 binomial blur of the thumbnail |
| `peak_window_max_s` | 2.0 | close a still-bright window at its peak after this |
| `min_reference_std` | 2.0 | refuse a reference with pixel std (0..255) below this |
| `image_topic`, `odom_topic`, `yaw_rate_topic`, `event_topic`, `status_topic`, `status_period_s` | see above | plumbing |

Bad thresholds (`rearm >= similarity`, negatives, NaN) are a hard error at start, not a warning.

## Tuning from a logged run

1. Drive a few laps with a camera on and `cameras:=true` (the recorder takes the compressed topic),
   starting the run the way a real one starts (car at the line, then go).
2. Replay it. From the repo root:

```
cd tools && uv run --extra vision python lap_replay.py <bag dir or .mcap>
uv run --extra vision python lap_replay.py <bag> --param similarity_threshold=0.85 --param min_lap_time_s=10
uv run --extra vision python lap_replay.py --images frames/ --fps 15 --odom-csv odom.csv
```

   `--param NAME=VALUE` takes every node parameter above (`--rate-hz` and `--decode-reduction`
   too), `--trace-csv out.csv` writes every similarity sample. A bag needs a
   `/camera/*/compressed` topic (`--image-topic` if there are several); `/odom/wheel` supplies the
   speed and distance gates, `/odom` the turns. Without odometry the start gate and distance gate
   are switched off and the output says so. An image folder is timed at `--fps`; the odometry CSV
   has the columns `t_s,speed_mps,x_m[,yaw_rate_rps]` on the same clock.
3. Read the output: the laps table (time, peak similarity, distance, turns), then one row per
   second with the highest similarity as a bar, `|` at `similarity_threshold`, `:` at
   `rearm_threshold`, and `<- lap N` where one was counted, then the **near misses**: stretches
   that rose to `rearm_threshold` or above but never counted.
4. Set the numbers:
   - Laps missing, near misses at 0.7 to 0.8: lower `similarity_threshold` to just under the
     lowest true peak, keeping it well above the highest non-line stretch the trace shows.
   - Extra laps: raise `similarity_threshold`, or raise `min_lap_time_s` and
     `min_lap_distance_m` to just under the real lap (80 percent of the shortest lap is a good
     margin).
   - Laps that sit on the start line and flicker: `rearm_threshold` should be below the
     similarity the trace shows between the line and the rest of the track, and below the
     `similarity_threshold` by a clear margin.
   - Turns far from 1 per lap (oval) in the table are a sign of a missed or a doubled lap.

## What is and is not verified

Verified on the Mac: the counter on synthetic image sequences (identical frame returning, noise,
brightness change, drift at the line, partial occlusion, flicker around the threshold, parking at
the line, time and distance gates, reset, heading turns; 100 percent of lines and branches), the
node and the launch files under rclpy in the local `car:perception` image (ROS 2 Humble, cv2 4.10), the replay tool
over written bags, killed-recorder bags and image folders, the app code (vitest, tsc).

Per-frame cost on an Apple laptop CPU, 1280x720 JPEG: decode at 1/4 scale 0.9 ms, counter 0.15 ms.

**Not verified:**

- Any real camera frame. The thresholds are guesses until one logged run is replayed.
- The per-frame cost on the Jetson CPU (the budget is 5 ms; expect about 1 to 4 ms for the decode).
- That the camera and the stack really exchange `/odom/wheel` across the two containers (the
  counter needs both topics in one ROS domain).
- Camera placement and aliasing: a forward camera sees the start line from the other side of the
  room every lap, but a track with two similar-looking straights could match the wrong one. The
  near misses list shows it; a lower `rearm_threshold` does not fix it, a higher
  `similarity_threshold` or `min_lap_distance_m` does.
- Rolling-shutter or motion blur at speed at the reference frame (the reference is taken while
  moving, at the first frame over `start_speed_mps`).
