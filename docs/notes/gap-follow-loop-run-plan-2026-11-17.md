# Car run plan: gap_follow race-2026-11-16 on the indoor loop

For a person at the track, the next session. Why this profile exists:
`docs/notes/gap-follow-loop-2026-11-16.md`. Nothing here has been driven yet. The owner says go
before Step 1.

Goal = laps of the 5 m indoor loop at 1.0 then 1.3 m/s, both directions, zero safety_node
refusals, zero contacts.

## Before anything moves

* Two people (05-safety: first run of new control settings). One on the RC kill, one on the
  laptop. Phone films every run.
* Kill switch check and the re-ID battery first, as every session (05-safety, 07).
* Car checkout on PR 203's branch `feat/gap-follow-loop-2026-11-16` (or main once merged), then
  rebuild in the container:

```sh
docker exec -it car-stack bash -lc 'source /opt/ros/humble/setup.bash && cd /workspace/ros_ws && colcon build --packages-select racer_control && source install/setup.bash && ros2 launch racer_control gap_follow.launch.py --show-args | grep -A2 race-2026-11-16'
```

  * The profile must be listed. If not, the build is stale; stop.
* Same loop as 2026-11-16, same island. Note anything that changed (wall gaps, boxes moved).
* Battery charged; write down the VESC input voltage at rest.
* Bag every run, separately from the launch (the 2026-11-16 runs only had the teleop bag):

```sh
docker exec -it car-stack bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/ros_ws/install/setup.bash && ros2 bag record -s mcap -o /workspace/data/bags/gf1116_<tag> /scan /drive_raw /drive /safety/events /odom/wheel /gap_follow/status'
```

  * Check the bag grows (`ls -l` twice, 10 s apart) before the car moves.

## Launch

```sh
docker exec -it car-stack bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/ros_ws/install/setup.bash && exec ros2 launch racer_control gap_follow.launch.py profile:=race-2026-11-16 <arguments of the step>'
```

* `reverse_escape:=false` on every run of Steps 1 to 3.
  * With it on, the car reverses on its own after a refusal. Today a refusal is the stop
    condition anyway, so the car just holds and a person picks it up.
  * Step 4 turns it back on only if 1 to 3 were clean.
* Ctrl-C ends the run. Standing start, car on the lane centre, pointing along the lane.

## Steps

Step 0. Wheels off the ground

* `max_speed_mps:=1.0 reverse_escape:=false`, 10 s.
* Watch: wheels turn toward the open side when a hand is held near one side, speed request at or
  below 1.0, Ctrl-C stops the wheels, RC kill stops the wheels.
* Stop if: any item fails. Stay off the floor.

Step 1. 1.0 m/s ccw, short

* `max_speed_mps:=1.0 reverse_escape:=false`, ccw, 30 s (about 4 laps).
* Pass = 0 ttc brakes in the bag (command below), no contact (video).
* Stop if: a refusal, a contact, or the rear comes within a hand of the wall (watch the corners,
  the L 0.6 run got to 7 cm). Go to "If it refuses".

Step 2. 1.0 m/s ccw then cw, long

* Same arguments, 60 s ccw, then 60 s cw.
* Pass and stop rules as Step 1.

Step 3. 1.3 m/s, both directions

* `max_speed_mps:=1.3 reverse_escape:=false`, 45 s ccw, then 45 s cw.
* Only if Steps 1 and 2 passed.
* Pass and stop rules as Step 1.

Step 4. As the profile ships

* `max_speed_mps:=1.3` (reverse_escape on, the profile default), 60 s ccw.
* Only if Step 3 passed. Escapes expected 0; any escape is a fail.

## Counting refusals, after each run

On the laptop, after copying the bag over:

```sh
uv run --project tools python -c "import sys; sys.path.insert(0, 'tools'); from follower_replay.bag import read_bag; from follower_replay.analysis import event_summary; print(event_summary(read_bag(sys.argv[1]).events))" <bag>
```

* Refusal = a `ttc` entry under `brakes`. Pass needs none.
* `ttc` under `engages` without a brake = warning zone, not a refusal. Write the count down
  anyway.
* `watchdog` brakes at the start or end of a run = launch/Ctrl-C; note them, do not count them.
* Checked on the 2026-11-16 teleop bag: it prints 39 ttc brakes over the whole session.

## If it refuses

* Stop the step. Do not go faster, do not turn reverse_escape back on to get through it.
* Write down: run, direction, which corner, rough time in the run.
* Keep the bag. The analysis is the 2026-11-16 note's method (brake windows, scan-matched
  speed); the sim reproduces from the map with `--plant-steer-tau` / `--plant-command-delay`.
* Likely next knob = steering lag. The note puts it at 0.05 to 0.2 s, not pinned; a refusal at
  1.0 m/s means the car is slower than the sim's slow plant (0.25 s lag + 0.15 s delay).
* safety_node is not changed. The profile is the only thing that moves.

## Not in this plan

* Faster than 1.3 m/s.
* race-2026-11-12 (pre-registered, unchanged).
* The wheel odometry 1.15 x reading and the rear overhang are open items in the 2026-11-16 note;
  this session collects data on both (the bag has `/odom/wheel`, the video shows the rear) but
  does not fix them.
