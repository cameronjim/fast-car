# Replaying a recorded run through the gap follower

`tools/replay_follower.py` takes a bag the car recorded, feeds every `/scan` through the same
follower code `gap_follow_node` runs, and tells you what the follower would have steered and
why, next to what the car actually did. It replaces the throwaway harness used on 2026-10-06
(the one behind the "62 of 63 scans" numbers in `reactive-control-port-2026-10-05.md`).

Nothing here runs on the Jetson and nothing drives the car. Copy the bag off, replay it on the
Mac.

## Pieces

| File | What |
|---|---|
| `ros_ws/src/racer_control/src/gap_follow_replay_cli.cpp` | ROS-free C++ CLI: scans in as text, one JSON line per scan out. Runs `GapFollower::process` (sanitize, extend disparities, select gap with forward preference and switch margin, steering gain and lane centring, swept-path clamp, corner override) and the node's steering low-pass. Physical constants come from the generated vehicle_params binding, like the node. |
| `ros_ws/src/racer_control/test/replay/` | The CLI's ctest: five synthetic scans, a golden file, and checks of sign convention, clamp, corner override, the low-pass against its closed form, and parameter validation. |
| `tools/replay_follower.py` | The command you run. Thin. |
| `tools/follower_replay/` | The logic: `bag.py` (mcap reading), `replay.py` (profiles, scan file, running the CLI), `analysis.py` (table, summary, CSV), `plots.py`, `pipeline.py`. |
| `tools/tests/test_replay_follower.py` | pytest for all of it, on bags written on the fly (indexed, unindexed, cut mid-record). |

The CLI's parameter table (names, defaults, ranges) is `gap_follow_node`'s, row for row. A
pytest compares the two source files, so changing a default in the node without the CLI fails
the tools tests.

## 1. Get the bag

Bags are under `~/car/data/bags/<timestamp>_car_teleop/` on the Jetson. From the Mac:

```sh
rsync -av racer@10.0.0.226:~/car/data/bags/2026-10-06T22-12-40_car_teleop/ \
  ~/code/car/data/bags/2026-10-06T22-12-40_car_teleop/
```

`data/` is gitignored, so the copy never lands in a commit. The recorder is often killed rather
than stopped, which leaves an mcap with no footer. That is fine: the tool streams the records
from the front and says `NOTE truncated/unindexed` in the summary. Messages in a last,
unfinished chunk cannot be recovered, so the end of such a bag can be a second or two short.
Pass either the `.mcap` file or the bag directory.

The bag needs `/scan` (the launch records it with `lidar:=true`). `/drive_raw`, `/drive` and
`/safety/events` are optional; the comparison columns are blank without them.

## 2. Build the CLI (once, and after any change to the follower)

In the `ros-dev` container, with the rest of the workspace:

```sh
docker run --rm -v "$PWD":/repo ros-dev:local bash -c '
  source /opt/ros/humble/setup.bash && cd /repo/ros_ws &&
  colcon build --packages-select racer_control'
# the binary: ros_ws/install/racer_control/lib/racer_control/gap_follow_replay_cli
```

Or natively on the Mac, which needs only a C++17 compiler and `uv` (the CLI does not touch
ROS; the generated header comes from `tools/gen_params.py` exactly as in the colcon build):

```sh
uv run --project tools python tools/gen_params.py --params config/vehicle_params.yaml \
  --schema config/vehicle_params.schema.json --out-dir /tmp/replay_gen
R=ros_ws/src/racer_control
clang++ -std=c++17 -O2 -I$R/include -I/tmp/replay_gen $R/src/gap_follow_replay_cli.cpp \
  $R/src/gap_follow.cpp $R/src/laser_scan.cpp -o /tmp/gap_follow_replay_cli
```

The tool finds the binary from `--cli`, then `$GAP_FOLLOW_REPLAY_CLI`, then the colcon install
path above, then `PATH`.

## 3. Run it

```sh
export GAP_FOLLOW_REPLAY_CLI=/tmp/gap_follow_replay_cli
uv run --project tools --extra plots python tools/replay_follower.py \
  data/bags/2026-10-06T22-12-40_car_teleop --profile floor-2026-10-06 --plot
```

`--extra plots` installs matplotlib; leave it and `--plot` off for the summary and CSV only.

- `--profile NAME`: a profile of `gap_follow.launch.py` (default `floor-2026-10-06`; `none` is
  the node defaults). The tool reads `PROFILES` from the launch file itself, so it is always the
  set the car launches with. (The launch file imports `launch_ros`, which only exists in a ROS
  install, so the module is parsed with `ast` rather than imported. Same values.)
- `--param NAME=VALUE`, repeatable: override on top of the profile, as a launch argument does.
  `--param swept_path_clamp=false --param centering_gain=0` is how you ask "what did that
  feature change". An unknown name, a value outside the node's range, or a LiDAR yaw that
  disagrees with the binding is refused, as the node would refuse to start.
- `--out-dir DIR`: default `data/replay/<bag>-<profile>`.
- `--plot-scans N` (default 6, evenly spaced), `--plot-at T,T,...` (seconds from the first
  scan), `--plot-flagged` (spread the N scans over clamped or corner-override scans only).
- `--straight-threshold-rad`: the left / right / straight tally treats `|steering|` below this
  as straight (default 0.05).

## 4. Read the output

The summary, from a synthetic five-scan bag (not a recorded run; it is the test fixture):

```
bag: synthetic.mcap (0.4 s of scans)
profile: floor-2026-10-06
scans: 5 (4 accepted by the core, 0 with no gap found)
steering, replay (before low-pass), |s| > 0.05 rad: left 1, right 1, straight 2
steering, replay (low-passed):  left 1, right 1, straight 3
steering, recorded /drive_raw:  left 4, right 0, straight 1
swept-path clamp: 1 of 4 scans (25.0 %)
corner override:  0 of 4 scans (0.0 %)
vs recorded /drive_raw over 5 scans: mean |difference| 0.176 rad, opposite sign in 1 of 2 scans where both turn
/safety/events: 1 brake engagement(s) (ttc 1)
  engagements by gate: ttc 1
  ttc releases: 1, total 0.2 s, median 0.2 s, longest 0.2 s
```

- **accepted by the core**: scans the follower could use (usable geometry, some valid return,
  forward covered). The rates below are over these. A rejected scan does not replace the latest
  result, as in the node.
- **steering, replay (before low-pass)**: the steering after the swept-path clamp and corner
  override, which is what the node low-passes. **(low-passed)** is the node's 50 Hz first-order
  filter run for the ticks between this scan and the next; **recorded /drive_raw** is the last
  command the car published in that same interval. Left is positive.
- **swept-path clamp / corner override**: how often each reduced or zeroed the steering. A high
  clamp rate with a large `wanted` steering is the follower asking for a turn the car cannot
  make past something beside it.
- **vs recorded**: how closely the replay reproduces the car. A bag recorded with different
  parameters than the profile will differ; a large mean difference with matching parameters
  means something outside the replay shaped the command (the reverse escape, a stale scan).
- **/safety/events**: interventions counted the way `claude-docs/09-evaluation.md` counts them,
  one per `PHASE_ENGAGE` record. **Brakes** are engagements with severity BRAKE. **Latch
  durations** come from the `PHASE_RELEASE` records (`ttc` is the forward gate, `ttc_reverse`
  the rear one); an engagement with no release means the bag ended while held.

Files in `--out-dir`:

| File | What |
|---|---|
| `per_scan.csv` | One row per scan: `stamp_s`, `t_s`, `valid`, `gap_found`, `target_bearing_rad`, `target_range_m`, `wanted_steering_rad` (gain times bearing plus centring, before the clamps), `steering_rad`, `filtered_steering_rad`, `clamped`, `corner_blocked`, `centering_rad`, `recorded_drive_raw_rad`, `recorded_drive_rad` (the gated command). Blank where there is no value. |
| `summary.json` | The summary above, machine-readable, with the resolved parameters. |
| `scans.png` | Chosen scans in the vehicle frame (car pointing up, left on the left, rear axle at the origin, LiDAR yaw and mount from the binding). Red star: the chosen target. Dashed blue arc: wanted steering; solid blue: replay output; green: recorded `/drive_raw`. |
| `steering.png` | Replay steering against the recorded `/drive_raw` over time, with ticks where the clamp or corner override fired and the released TTC latches shaded. |
| `scans.txt`, `params.json` | The exact inputs given to the CLI, so a single scan can be re-run by hand: `gap_follow_replay_cli --params params.json --scans scans.txt`. |

## What a replay is, and is not

- **Open loop.** The recorded scans are what the car saw on the path it actually drove. The
  replay's steering does not move the car, so the replay answers "given this scan, what would
  the follower say", not "where would the car have gone". Use the sim for the second question.
- **Steering only.** The speed law, the reverse escape and safety_node are not replayed. The
  safety events and the recorded commands are shown beside the replay, not recomputed.
- **Low-pass timing is modelled**, not recorded: ticks at `control_rate_hz` between scan
  stamps, a `/scan` gap past `scan_timeout_s` stops the ticks, the first tick after a gap steps
  by zero. The scan header stamp is used, not the arrival time.
- **`target_range_median_scans` and the speed keys are accepted and ignored**: they do not
  change the steering.

## Worked example: the 2026-10-06 floor test

The question that day was why the car, on a lane with a wall on one side and scattered objects
on the other, kept steering into the open room and clipping the apex object on left corners.
The bag was `2026-10-06T22-12-40_car_teleop`. What the old harness found, from
`reactive-control-port-2026-10-05.md`: the follower chose a gap between the objects, into the
room, in 62 of 63 scans under every parameter set tried, and 62 of 63 scans chose a gap whose
turn passed within the car's width of an object beside the car.

With this tool the same two findings are two runs and a look at two columns:

```sh
R=data/bags/2026-10-06T22-12-40_car_teleop
# 1. gap choice: the old behaviour (no forward preference), then with it
python tools/replay_follower.py $R --profile none --out-dir /tmp/r_none --plot --plot-scans 8
python tools/replay_follower.py $R --profile none --param forward_preference=0.6 \
  --out-dir /tmp/r_fwd --plot --plot-scans 8
# 2. the apex object: with the swept-path clamp off, then on (the floor profile)
python tools/replay_follower.py $R --profile floor-2026-10-06 --param swept_path_clamp=false \
  --out-dir /tmp/r_noclamp
python tools/replay_follower.py $R --profile floor-2026-10-06 --plot-flagged
```

What to look at:

1. Gap choice. In `scans.png` of `r_none`, the red star (the target) is in the side opening
   between the objects when the follower has picked the room over the lane. Compare the same
   stamps in `r_fwd`: the star should move to the lane ahead, and `target_bearing_rad` in
   `per_scan.csv` should move toward 0 for those rows. `--plot-at` with the same times in both
   runs gives matching panels.
2. The apex object. With the clamp off, `steering_rad` equals `wanted_steering_rad` in every
   scan, so the solid blue arc in `scans.png` runs through anything beside the car. With the
   clamp on, `clamped` is true in the scans where an object lies beside the car on the turn-in
   side and `steering_rad` is smaller than `wanted_steering_rad`; the clamp rate in the summary
   is the "62 of 63" figure, measured by the tool instead of counted by hand.

The numbers in the first paragraph are the ones recorded in the 2026-10-06 note. The bag was
not on hand when this tool was written (only a synthetic bag was available), so the output of
these commands on `2026-10-06T22-12-40_car_teleop` is not reproduced here. Paste the two clamp
rates and the gap-choice shift into this section the first time it is run on the real bag.
