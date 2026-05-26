# Reactive control (classical, no learning)

This document explains the hand-written controllers in the `reactive_control`
package and the math behind them. None of these need training data. They react
directly to the current LiDAR scan (and, for the camera follower, the current
image).

All three controllers share one safety node, and all of them publish steering to
`/drive` while taking their allowed speed from the safety node on `/speed`. The
safety node always has the final say on speed.

Contents:

- [LiDAR geometry and conventions](#lidar-geometry-and-conventions)
- [PID controller](#pid-controller)
- [Safety node (automatic emergency braking)](#safety-node-automatic-emergency-braking)
- [Gap following](#gap-following)
- [Wall following](#wall-following)
- [Vision path following](#vision-path-following)

## LiDAR geometry and conventions

The car uses a planar laser scanner that returns an array of ranges, one per
angular step. On the F1TENTH platform the scanner covers 270 degrees at 0.25
degrees per step, which gives 1080 readings. Index 0 is the far right of the
car, the middle index is straight ahead, and the last index is the far left.

To convert an angle in degrees to an array index:

```
index = (angle_deg + 135) * 4
```

The `+135` shifts the -135 degree start to zero, and the `*4` converts from 0.25
degree steps to index counts. Positive angles are to the left of the car,
negative angles are to the right.

A pattern that shows up in several places is converting a lateral safety margin
into a number of rays. If an obstacle is at distance `d` and we want to protect a
lateral half-width `w` around it, the half-angle it subtends is `atan2(w, d)`.
Converting that angle to a ray count at 0.25 degrees per ray:

```
rays = atan2(w, d) * (180 / pi) * 4
```

The code writes `180 * 4 / 3.14` for the `(180 / pi) * 4` factor.

## PID controller

`pid.py` is a standard PID controller shared by the gap, wall, and camera
followers. Given an error `e(t)` it outputs:

```
u(t) = Kp * e(t) + Ki * integral(e dt) + Kd * de/dt
```

The time step `dt` is measured from the ROS clock between callbacks. The first
call uses a small placeholder `dt` of 0.01 seconds, and any non-positive `dt` is
floored to `1e-6` to avoid dividing by zero. The integral term is clamped to the
range `[-100, 100]` to prevent integral windup if the error stays large for a
long time.

## Safety node (automatic emergency braking)

The safety node is the only thing that talks to the car's drive topic with the
authority to stop it. It watches the LiDAR and the car's speed, estimates how
long until a collision, and brakes in stages.

Time to collision (TTC) is the closest obstacle distance divided by the current
forward speed:

```
TTC = min_distance / v
```

When the car is basically stationary (`v` near zero), TTC is meaningless, so it
is treated as infinite and only the raw distance threshold can trigger a stop.

The node does not look at the whole scan. It looks inside a cone that points
where the car is actually steering. The center ray of the cone is found from the
last commanded steering angle:

```
target_ray = steering_angle / angle_increment + N / 2
```

The half-width of the cone (in rays) comes from the ray-count formula above with
a lateral margin of 0.7 meters:

```
danger_zone = atan2(0.7, range[target_ray]) * (180 / pi) * 4
```

The minimum distance inside that cone drives a staged response, from least to
most aggressive:

| Stage | Condition | Action |
|---|---|---|
| NONE | TTC above `ttc_pb1` | normal speed |
| PB1 | TTC below `ttc_pb1` | partial brake (mild) |
| PB2 | TTC below `ttc_pb2` | partial brake (stronger) |
| FB | TTC below `ttc_fb`, or distance below `distance_threshold` | full brake, latch emergency stop |

On full brake the node latches an emergency stop and publishes it on `/kys`, so
the controllers know to stop too. The reactive safety node also tries to recover:
its timer checks whether the forward region has cleared and releases the stop if
it has.

On Ctrl+C the node does not cut the motor instantly. It enters a winding-down
state, drops the commanded speed to zero, and keeps running briefly so it can
keep steering while the car coasts to a stop. A second Ctrl+C shuts down right
away.

The `learned_control` package has its own copy of the safety node with two
differences: it reads the controller command on `/drive_raw` instead of `/drive`,
and it adds a wall-avoidance steering bias. The bias compares the minimum
clearance on the left and right of the car:

```
left_push  = max(0, side_margin - left_clearance)
right_push = max(0, side_margin - right_clearance)
bias       = wall_steer_gain * (right_push - left_push)
```

The bias is clamped to `+/- max_wall_steer_bias` and added to the steering angle,
nudging the car away from whichever wall is too close.

## Gap following

Gap following (`gap_follow_node`) steers toward the largest open space in front of the
car. It runs in four steps.

1. **Clip the scan.** Ranges are clipped to `clip_max_range` so that very far or
   invalid readings do not dominate.

2. **Find disparities.** A disparity is a large jump between two neighbouring
   readings, which usually marks the edge of an obstacle:

   ```
   |range[i] - range[i+1]| > disparity_threshold
   ```

3. **Inflate obstacles.** For each disparity, the nearer of the two readings is
   the close edge of an obstacle. The scan is "padded" by overwriting the rays
   just past that edge with the near distance, so the car treats the obstacle as
   wider than a single ray. The number of rays to pad uses the ray-count formula
   with the vehicle half-width:

   ```
   danger_zone = atan2(vehicle_half_width, near) * (180 / pi) * 4
   ```

   This is what stops the car from trying to thread a gap that is too narrow for
   its body.

4. **Pick the gap.** Inside a forward cone (from `cone_left_fraction` to
   `cone_right_fraction` of the scan), every ray longer than
   `free_space_threshold` counts as free space. The node finds the longest run of
   contiguous free rays, breaking ties toward straight ahead, and aims at the
   center of that run. The steering angle is the offset of that target ray from
   the center ray:

   ```
   angle = (target_ray - center_ray) * angle_increment
   ```

   A separate corner check looks at the far-left or far-right rays depending on
   the turn direction. If that whole region is closer than `corner_min_clearance`
   it is treated as a dead end and the car is told to go straight instead.

The steering angle goes through the PID controller, and the speed comes from the
safety node.

## Wall following

Wall following (`wall_follow_node`) keeps the car a fixed distance from the right-hand
wall. It uses two LiDAR rays: ray `a` pointing forward-right at -20 degrees and
ray `b` pointing hard right at -90 degrees. The angle between them is
`theta = 70` degrees.

From the two distances it estimates the car's orientation relative to the wall:

```
alpha = arctan( (a * cos(theta) - b) / (a * sin(theta)) )
```

The current perpendicular distance to the wall is:

```
AB = b * cos(alpha)
```

Steering on the current distance alone reacts too late, so the controller looks
ahead by projecting the distance forward by however far the car will travel in
the next time step:

```
CD = AB + v * dt * sin(alpha)
```

The error fed to the PID controller is the difference between the desired wall
distance and this lookahead distance:

```
error = target_distance - CD
```

The PID output is the steering angle. Speed starts from `max_speed`, is reduced
on sharp turns by `K_speed * |steering|` (with a floor at `min_speed`), and is
then capped at whatever the safety node currently allows.

## Vision path following

The camera follower (`cv_node`) keeps the car centered on the track using a
forward RGB camera instead of the LiDAR. Each frame is processed like this:

1. **Make a mask of the track.** The image is converted to grayscale, cleaned up
   with an erode then dilate using a 9x9 kernel (this removes small specks and
   fills small holes), and thresholded to black and white at a value of 127.

2. **Pick the track contour.** Among the contours in the mask, the node keeps the
   first one that is low enough in the frame (`y > 200`) and large enough
   (`area > 10000`). If nothing qualifies, no command is published that frame.

3. **Find the steering target.** It samples a single horizontal row of the mask
   (row 400) and takes the mean x position of the white pixels. That mean is the
   horizontal target. If the row has no track pixels, the controller treats the
   path as straight.

4. **Steer toward it.** The horizontal offset from the image center becomes an
   angle, with a small constant bias to the right so the camera keeps a good view
   of the track:

   ```
   x_target = x - image_width / 2
   angle    = -atan2(x_target, y) - 0.2
   ```

   The angle goes through the PID controller. Speed comes from the safety node.
