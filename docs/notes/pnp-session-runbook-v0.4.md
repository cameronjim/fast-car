# Car session runbook, Plug-and-Play v0.4

For a person at the track. The full procedure with every launch command is
`tests/bench/procedures/pnp_session_v04.yaml` (design `claude-docs/13-plug-and-play.md` 16.7 and
16.8). Pass lines are quoted from `docs/notes/preregistration-pnp-v0.4.md` (v0.4) and the v0.2 and
v0.3 files. The v0.3 runbook and procedure are unchanged.

Steps 1, 2 and 9 (the M-SID2 excitation blocks) are worth a session by themselves.
If time runs short, do those.

How the 2026-11-14 run is reported, everywhere: "GP3 not met as registered; 20 clean laps
observed, camera count 10 of 20". A 20 lap run at this session is GP3-R1 (Step 1b), a
repeat under the unchanged rule of v0.2 4.3, never GP3.

## Before anything moves

- Two people. Kill knob in hand. Wheels off the ground before the floor (Step 0).
- Write down the VESC input voltage (5 s mean at rest) and the 0.8 V window every run must start
  inside. Write each run's start voltage in the session file.
- Every run: ccw first, standing start, the bag growing (`ls -l <bag>` twice, ten seconds apart).
  The launch refuses without a recorder; that is the guard working.
- Nothing in this session sets a scheduling policy, a CPU governor, a watchdog limit, a limiter dt
  or a safety parameter. Where it says "owner decision, not taken", write the table down and go
  on.
- Every PnP run bags `/drive_raw`, `/drive`, `/safety/events` and, when brief 42 is on main,
  `/pnp/planner/timing` (skip it if it is not on main).
- Wall contacts are what the two people and the phone video say. The video decides.

### Gotchas of 2026-11-14

- A root-owned `tools/.venv` breaks colcon (the build runs `uv` in `tools/`). If the build fails
  on it, `sudo chown -R` it back to the car user before the build, not during a step.
- The app's lap counter toggle persists across runs. Look at it before every run; a run with the
  counter off has no camera laps.
- If a PR merges mid-session, pull main again and rebuild before the next step. The car's
  checkout may be on a feature branch with hot-copied files.
- The app refuses a cap above the active bundle's validity. That is correct. Do not work around
  it; Step 3 exists for that case.
- AMCL publishes only after 5 cm of motion, and carrying the car leaves it stale (Step 8).

## Launch (inside the container)

```sh
docker exec -it car-stack bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/ros_ws/install/setup.bash && exec ros2 launch racer_bringup pnp.launch.py <arguments of the step>'
```

After a run, on the laptop. `<bag>` is the bag directory, `<out>` the session folder.

## Step 0 Build, wheels off the ground, camera counter bench check

- Launch: `dynamics_bundle:=/workspace/data/pnp/models/<dyn> backend:=geometric v_max_mps:=0.4 lap_counter:=true require_lap_counter:=true`.
- First: `uv run --project tools --extra session python tools/pnp_session_report.py check-prereg`.
  It must print that the SHA-256 matches. If it refuses, pull main.
- Watch: the wheels turn right with the board on the left, left with the board on the right,
  `cmd_speed_mps` never above 0.4, Ctrl-C stops the wheels within the watchdog time, the kill knob
  works. Camera container first, at the 10 Hz record rate of the counter's camera
  (`ros2 launch racer_bringup cameras.launch.py lap_record_rate_hz:=10.0`): hold the start line mark in front of the camera three times,
  the counter counts three laps.
- Stop when: any item fails. Stay off the floor.
- The number: none. A checklist.

## Step 1 C timing and D3-C0 (minimum session)

- Launch: `v_max_mps:=0.5` then `1.0`, with `lap_counter:=true require_lap_counter:=true lap_memory:=true lap_mode:=observe`
  with the camera container already up at `lap_record_rate_hz:=10.0`. Per cap: ccw, ccw, cw, cw, 10 laps each, one launch per run,
  only `v_max_mps` changes. The phone films the lane and the start line for all eight runs.
  (The registered 4.2 needs at least 8 standing starts at caps 0.5 and 1.0, so two runs per
  direction per cap.)
- Watch: contacts (people and video), the bag growing, the counter toggle.
- Stop when: a contact, or a BLOCKED hold. Note it and decide with the other person.
- The command that prints the number, per bag:

```sh
uv run --project tools --extra session python tools/pnp_session_report.py v04 --bag <bag> --caps 0.5,0.5,0.5,0.5,1.0,1.0,1.0,1.0 --lapref-dir <out>/lapref --out <out>/step1_v04.json
```

  Add `--owner-accepted-camera --m-cam-held` only if both are true (v0.4 4.4). Otherwise the
  reference is the phone video: write each run's lap times from the video as a CSV
  (`run,lap,lap_time_s`) and add `--times <csv>`.
- What it has to be: v0.2 4.2, judged again: watchdog at most 0.2 per minute, `rate_limit` at
  most 0.2 per minute, no event of either kind in the first 2.0 s of at least 7 of 8 starts, TTC
  and obstacle brakes 0, no contact. A miss is reported with its counts and goes to the owner as a
  decision (real-time scheduling), not taken here. D3-C0: lap count equals the reference's in both
  runs and at least 18 of the 20 lap times within 0.3 s. `lapref` (`tools/lapref.py <bag> --run
  <start> <end> --out <out>/lapref/<run_id>.json`) is printed next to it and decides nothing.
- Next to it, `racer_eval timing_chain --bag <bag>` (M-C2, M-T2, M-C3, no threshold; brief 42,
  skip if not on main, and it needs `/pnp/planner/timing` in the bag). v0.2 4.2 is judged as
  before. The report prints the GP3 wording above and whether the tool is on this checkout.
- The report ends with a table of which later steps these results allow. Run it with
  `--b4-commit <sha>` (the commit battery B4 ran on) so it can check the deployed planner and
  lap code (Steps 5 and 7), and with `--result key=met` for B4, B5 and B6 results.
- If D3-C0 fails: D does not drive the cap this session (Steps 5 and 7 are not run).

## Step 1b GP3-R1 (optional, after Step 1)

- Only if M-CAM held (the changed camera counter was accepted). Launch as Step 1 at
  `v_max_mps:=1.0` in observe mode: one continuous run, one launch, 20 consecutive laps, ccw,
  nobody touches the car.
- Laps are counted by the camera counter and cross-checked by wheel distance (at least 0.9
  times 20 lap lengths); a BLOCKED hold over 5 s fails it; no wall contact.
- It is named GP3-R1 (GP3-R2 after it), its outcome is its own, and it is never reported as GP3.

## Step 2 Dynamics confirmation (minimum session)

- No candidate exists that can be switched on, so nothing drives in this step. It only records
  that M-B6 (brief 47) will score the frozen v0.4 models offline on this session's bags.
- Rollback is not needed: the deployed bundle `dyn-v0.1-2026-11-12` stays on the car.

## Step 3 A cap above 1.3

- Only if the active bundle's `io.validity.v_max_mps` is above 1.3 and the owner agrees. The app
  refuses a cap above the validity; that is correct.
- Launch: `v_max_mps:=<new_cap>`, lap memory off, two runs per direction, ccw first. Runs at 1.3
  of the same session are the comparison; drive two per direction at 1.3 first if there are none.
- Watch: contacts, any TTC or obstacle brake. Stop at the first of either.
- The command: `python -m racer_eval session` with a session file that has a `cap_step` block
  (`new`, `reference`, `bundle_manifest`, and per run `direction` and `lap_memory: off`).
- What it has to be (v0.4 6.4): no TTC or obstacle brake, no contact, mean wheel speed not lower
  than at 1.3, the cap within the bundle's validity. Every item is printed.
- A 20 lap run at another cap is a repeat at that cap, named GP3-R1 at 1.0 only, never GP3.

## Step 4 Fused backend in shadow, then driving

- Only with the frozen fused bundle (brief 38) on the car.
- Shadow first: `backend:=geometric v_max_mps:=1.0 shadow_perception_bundle:=<fused>`. Read the
  shadow node latency p99 from `/pnp/shadow/perception/status`. It has to be under 10 ms on the
  Orin before it drives.
- Then `backend:=net perception_bundle:=<fused> v_max_mps:=1.0`, ccw, next to one geometric run
  in the same direction in the same battery window.
- What it has to be (v0.4 3.4, v0.2 section 2): mean track confidence at least the geometric
  run's minus 0.03 and no new TTC or obstacle brake. Written as "same layout family". Whether a
  variant counts as a layout the net never trained on is the owner's decision.

## Step 5 D v0 on the plain donut

- Only if D3-S0 and D3-S1 (B4, PR 185) and D3-C0 (Step 1) passed, AND either the deployed
  planner and lap code are the code B4 ran on (the report checks the git trees of
  `ros_ws/src/racer_laps` and `ros_ws/src/racer_planner` against `--b4-commit`, and a dirty or
  hot-copied directory fails it) or B6 passed on the changed code (D5-S0 and D5-S1). Check the table from the v04
  report, or run it again with `--result d3_s0=met --result d3_s1=met`.
- Launch: `v_max_mps:=1.0 v_lap1_mps:=0.5 lap_memory:=true lap_mode:=v0`, 10 laps per run, two
  runs per direction, ccw first, each start inside the battery window.
- What it has to be: 10 laps, no contact, the cap rises lap over lap (the `laps` report).
- Rollback: `lap_memory:=false`.

## Step 6 Variant layout 1, D off

- Build variant 1 by the protocol below and check its form first. Without a passing check the
  runs are "variant not documented".
- Launch: geometric, `lap_memory` off, `v_max_mps:=0.5` then `1.0`, ccw first, two runs per
  direction, with `shadow_perception_bundle:=<fused>` if the bundle exists.
- What it has to be: laps complete, no contact. The fused shadow numbers are reported as a
  variant, not as an unseen layout.
- Variant 1 is driven by this step, so it is NOT the GP4-V variant.

## Step 7 GP4-V on variant 2

- Only if D4-S0 and D4-S1 (B5) and Step 5 passed, AND either the deployed code is the code B4
  ran on or B6 passed (D5-S0, D5-S1, D6-S0 and D6-S1).
- Build variant 2, a NEW variant, never driven. Write the interleave seed into its form BEFORE
  the first run, check the form against variant 1's (positions must not repeat), and print the
  run order:

```sh
uv run --project tools --extra session python tools/pnp_session_report.py variant-form --form <out>/variant2_form.yaml --earlier-forms <out>/variant1_form.yaml --conditions d_on,d_off --runs-per-condition 5
```

- Runs: at least 5 D-on (`lap_mode:=v0`, or `v1` only where the v0.3 1.4 table allows it,
  `v_lap1_mps:=0.5`, `v_max_mps` the ceiling min(validity, 1.5)) and 5 D-off at the lap-1 cap
  (`v_max_mps:=0.5`, lap memory off), in the printed order, direction ccw, 10 laps each, every
  start inside the battery window. One run per condition is timed from the phone video.
- The command: `python -m racer_eval session` with a session file whose `gp4v` block names the D-on
  and D-off conditions and the form (`form: <out>/variant2_form.yaml`, `driven_variants: [variant-1]`).
  It refuses a form with no receipt or one edited after its check.
- What it has to be (v0.2 5.2, on a variant): every D-on run completes 10 laps; Delta = T1 -
  T_late positive in every run, the 95 percent t interval of the mean Delta excludes 0, median r
  at most 0.80; contacts over the D-on runs no more than over the D-off runs. It is reported as
  "GP4-V, lap learning on an obstacle variant of a known track", never as GP4. A failed gate is a
  result: write it down and stop.

## Step 8 Tracker and PnP back to back

- Plain donut, objects removed. Start the tracker from the app's tracker card (AMCL placed where
  the car is, the 1.5 m/s line), then PnP at cap 1.0 with lap memory off. Same direction, 5 laps
  each, one battery window, each run's start voltage in the session file (`direction`, `driver`:
  `tracker` or `pnp`, `start_voltage_v`).
- The command: `python -m racer_eval session` with a `same_session: {}` block. It writes
  `tables.json` and prints the table: laps, mean and best lap time, wheel distance per lap,
  direction, start voltage. It refuses runs of different sessions or directions. There is no
  gate. With fewer than 5 laps each no comparison is claimed (v0.4 6.5).

## Step 9 Excitation runs, M-SID2 (minimum session)

Registered in `docs/notes/preregistration-pnp-v0.5-dynamics.md` section 5 (yaml block
`excitation` of `evaluation/prereg/pnp-v0.5-dynamics.yaml`, SHA-256
`2767f297d951c11683b40834b864b832e8cbd0b01343e2765681e11b22e86a82`; design 13 section 18.6).
This replaces the earlier 9a (PnP with lateral offsets) and 9b (data laps). Commands are model
inputs, not imitation targets.

Print the plan first, on the laptop or in the container. It reads the yaml, prints the table
with the exact launch lines and refuses a plan above 600 s:

```sh
python3 tools/pnp_session/excitation_plan.py --dyn <dyn> --out <out>
```

### 9a The table (552 s planned, 600 s budget)

Every run in the stack with `safety_node` and the recorder, two people, kill in hand. All
clockwise blocks first, then counter-clockwise. Gap follow lines run inside the container
(`ros2 launch racer_control gap_follow.launch.py <arguments>`), stopped with Ctrl-C when the
time is up.

| # | Block | Launch arguments | Drive for | Sim check (3 laps) |
|---|---|---|---|---|
| 1 | P cw | block P below | 96 s | not simulated here |
| 2 | G cw 1.0 | `profile:=race-2026-11-12 max_speed_mps:=1.0 a_lat_max_mps2:=2.0` | 30 s | no contact, peak 1.00 m/s |
| 3 | G cw 1.3 | `profile:=race-2026-11-12 max_speed_mps:=1.3 a_lat_max_mps2:=2.0` | 45 s | no contact, peak 1.29 m/s |
| 4 | L cw | `profile:=lane-2026-10-25 target_mode:=gap_deepest max_speed_mps:=0.6 reverse_escape:=false` | 40 s | no contact, peak 0.41 m/s, 0.09 m body clearance at the closest |
| 5 | F cw 1.4 | `profile:=race-2026-11-12 max_speed_mps:=1.4 a_lat_max_mps2:=2.4` | 20 s | no contact, peak 1.39 m/s |
| 6 | F cw 1.5 | `profile:=race-2026-11-12 max_speed_mps:=1.5 a_lat_max_mps2:=2.8` | 60 s | no contact, peak 1.49 m/s, one rate_limit event |
| 7 to 11 | the same counter-clockwise, without G at 1.0 | | 96, 45, 40, 20, 60 s | no contact in any (peaks 1.29, 0.42, 1.39, 1.49 m/s) |

Block P (the cap schedule tool is unchanged), for each direction:

```sh
python3 tools/pnp_session/cap_schedule.py --manifest /workspace/data/pnp/models/<dyn>/manifest.yaml --v-max-mps 1.3 --write-params <out>/excitation_params.yaml
ros2 launch racer_bringup pnp.launch.py dynamics_bundle:=/workspace/data/pnp/models/<dyn> backend:=geometric v_max_mps:=1.3 lap_counter:=true require_lap_counter:=true params_file:=<out>/excitation_params.yaml
python3 tools/pnp_session/cap_schedule.py --manifest /workspace/data/pnp/models/<dyn>/manifest.yaml --v-max-mps 1.3
```

The tool steps the cap 0.5, 1.0, 0.7, 1.3 m/s, 8 s each, three cycles (96 s), never publishes a
drive topic, refuses a cap above the bundle's validity or `v_max_mps` (correct, do not work
around it) and refuses to start if the lap node publishes guidance (`lap_memory` stays off).
If the tool dies the planner falls back to `v_no_guidance_mps`, the lowest cap. If
`cap_schedule.py` is not on the car: block P is PnP at cap 1.3 for 96 s per direction with a
STOP and a standing start every 30 s; say so in the report.

Rules:

- A wall contact ends the blocks of that direction.
- Rung F 1.5 only if rung F 1.4 had no contact.
- Block F only with the owner's yes at the track.
- If L stalls twice in its first 20 s, stop it and go on. Its time is not given to another block.
- A safety brake is not a contact.

Said plainly:

- The lateral limit values 2.4 and 2.8 m/s^2 have NOT been driven on the car. 2.0 has (by the
  PnP runs at cap 1.3).
- The lane profile with the deepest-ray target (block L) may stall against the island.

Sim check (L5, development only, decides nothing). Brief 40's round donut (case O1) is not on
main, so the world is the `2026-11-13-pnp-lap1` map of the car's own track, driven by
`tests/l5_reactive_bench/track_family_runner.py --track map --driver gap_follow`, car as
measured, noise on, seed 11, 3 laps per direction, real-time factor 1.00, at most 3 runs in
parallel. All 10 lines: no wall contact, no stall, no reverse escape, no obstacle or TTC brake.
Peak speed reached (odometry): G 1.0 1.00, G 1.3 1.29, L 0.41 to 0.42 (cap 0.6; mean 0.35),
F 1.4 1.39, F 1.5 1.49 m/s; lap time 6.4 to 6.5 s (G 1.0), 5.3 to 5.6 s (G 1.3), 12.7 s (L), 5.0
to 6.1 s (F 1.4), 4.7 to 5.0 s (F 1.5). Safety events: F 1.5 had one rate_limit at about 6.6
s in both directions (a limiter event, not a contact). L is the tightest line: the closest body
clearance was 0.08 to 0.09 m against 0.23 m or more for the others, so it is the block most
likely to touch or stall on the floor. No line was changed.

### 9b After the session

- On the dev machine, for every bag: `uv run --project tools --extra session python tools/pnp_data.py runs <bag>`.
- The dynamics split of this session is brief 47's (every run 55 percent, 2.0 s, the rest). The
  v0.4 `uv run --project tools --extra session python tools/pnp_data.py split <bag> --session <date>` still runs for everything else. Fits
  refuse a bag with no committed split.
- The number: none. The bags are the deliverable.

### Owner step list (excitation only, about 12 minutes with changeovers)

1. Two people, kill knob in hand, car on the floor, start voltage written down (Before anything
   moves). Check the app's lap counter toggle.
2. `python3 tools/pnp_session/excitation_plan.py --dyn <dyn> --out <out>` and read the table.
3. Clockwise first. Block P: the three block P commands above with `<dyn>` and `<out>` filled.
   Drive 96 s, Ctrl-C the launch.
4. G cw: launch line of row 2 for 30 s, row 3 for 45 s. Ctrl-C between them. Check the bag is
   growing (`ls -l <bag>` twice, ten seconds apart) before each.
5. L cw: row 4 for 40 s. Stall twice in the first 20 s: stop it and move on.
6. F cw: say "yes" at the track, then row 5 for 20 s, and row 6 for 60 s only if row 5 had no
   contact.
7. Counter-clockwise: the same without G at 1.0 (rows 7 to 11).
8. Any wall contact: stop that direction's remaining blocks and write it down (the video
   decides).

## The variant layout protocol

A variant is the plain donut plus objects. All of these, written on the form
(`<out>/variantN_form.yaml`, saved next to the bag, with the photo file beside it):

- At least two solid objects, each against a wall (no free-standing object in the lane).
- The lane changes by at least 0.15 m at two or more places (the protrusion of each).
- The remaining lane at each object is at least 0.80 m (the narrowest lane the car has driven),
  measured and written down.
- Positions not used in an earlier session or variant, and not driven before the protocol's first
  run (`driven_before_first_protocol_run: false`).
- Solid to the LiDAR at its mounting height: each object's `height_m` is compared with the LiDAR
  height from the vehicle binding plus 0.02 m. Heavy enough not to move when touched. Nothing
  reflective, nothing glass.
- Per object: id, `height_m`, `size_m` (two numbers), `wall` (outer or inner), `distance_along_outer_wall_m`
  from the start line, `protrusion_m`, `lane_width_m`, `against_wall`, `heavy`, `reflective_or_glass`.
  Per form: `variant_id`, `photo`, `seed` (an integer, written before the first run),
  `driven_before_first_protocol_run`.
- Check it: `pnp_session_report.py variant-form --form <form>`. It writes
  `<form>.checked.json` (the form's SHA-256 and the numbers used). Edit the form after that and
  the receipt no longer matches: check again.
- Step 6 and Step 7 use different variants (the strict reading of "never driven" in v0.2 5.2).
  Do not call a variant an unseen layout.

## Lap gate v3: D8-C0 (brief 50)

Added with the lap gate v3 attempt; nothing above this heading changed. The step "observe mode, two
runs of at least 10 laps, one per direction" ends with this command on the two bags, one bag per run:

    cd tools && uv run --all-extras python -m pnp_session lap-c0 --bag <ccw bag> --bag <cw bag> \
        [--video laps.csv] --out <session dir>/lap_c0.json

It prints three blocks for the same runs: D3-C0 (`laps.d3c0`, unchanged), the D7-C0 rule and D8-C0
(items (i) to (iv) and, for D8-C0, item (v), the lap node's `on_scan_ms_p99` at most 80 ms). The D8-C0
block is the gate. The reference is the camera lap counter, or with `--video` the phone video (a CSV
with the columns `run,lap,lap_time_s,t_cross_s,distance_m`; the run name is the bag's name). A run
whose reference count is not the wheel distance count is REPEAT: drive it again. The lap node must be
the freeze commit (`evaluation/prereg/freeze/b8.json`) with `ring_enabled` left at its default.
`--replay` recomputes the events through the checked-out gate and labels the report REGRESSION, NOT A
GATE: never use it as the car result.

## Camera lap source: D9-C0 and D v0 on the camera (brief 51)

Added with the camera lap source attempt; nothing above this heading changed.

**Superseded by "Camera lap source: D10-C0 and D v0 on the camera (brief 51c)" below. Read that
section; this one is kept as written.**

1. **Start view.** Put the car on a taped mark (four corners) so that it faces something
   that looks like nowhere else on the lap: a door frame, furniture, a box or poster placed
   for the purpose, within about 3 m, with large light and dark areas. Not a bare wall. Not
   along a long straight where the view hardly changes as the car approaches. Lights as they
   will stay. Nobody walks through the view. Every launch of this section starts from this
   mark; the cw runs get their own mark and their own view.
2. **Test pass, one run per direction.**
   `ros2 launch racer_bringup pnp.launch.py lap_memory:=true lap_mode:=observe lap_source:=start_scan lap_shadow:=camera v_max_mps:=<the ceiling D will get>`.
   Before switching to auto, wait for the log line that the camera source is READY (D9-P0):
   camera `running`, 0 laps, similarity at least 0.90, car standing. If it does not come,
   relaunch; do not drive a judged run without it. Drive 11 laps, count them aloud, stop.
3. **Score.** `cd tools && uv run python -m pnp_session lap-src --bag <ccw> --bag <cw> --laps-counted 11 --laps-counted 11`.
   D9-C0 per direction. The same two bags give D8-C0 through `lap-c0`. A direction that fails
   may be tried again after the start view was changed, at most 3 attempts per direction; every
   attempt is reported.
4. **D v0, only in a direction whose D9-C0 passed, same mark, same lights, same session.**
   `ros2 launch racer_bringup pnp.launch.py lap_memory:=true lap_mode:=v0 lap_source:=camera lap_shadow:=start_scan v_lap1_mps:=0.5 v_max_mps:=<ceiling>`.
   Wait for READY. 10 laps. The owner holds the kill switch. Stop the run at any wall
   contact, at a second doubtful lap in the status, or at a second obstacle brake.
   If the status says `reference_suspect`, stop at once and relaunch from the mark; the run is
   not evidence. If it happens twice in a direction, change the start view and repeat the test
   pass (step 2) for that direction.
5. **After.** `lap-src --drive --bag <bag>` for the lap table. Write the session note the same
   day.

## Camera lap source: D10-C0 and D v0 on the camera (brief 51c)

Added with the start gate; nothing above this heading changed except the one line that marks the
brief 51 section as superseded. Steps 1, 3 and 5 are those of brief 51 with the changes named.

1. **Start view.** As brief 51 step 1. The mark, the view and the lights are the session's.
2. **Test pass, one run per direction.**
   `ros2 launch racer_bringup pnp.launch.py lap_memory:=true lap_mode:=observe lap_source:=start_scan lap_shadow:=camera v_max_mps:=<the ceiling D will get>`.
   **The launch does not start the planner until the camera source is ready.** It waits for the
   counter, calls `/lap_counter/reset`, then `lap_source_ready_wait` waits (at most 30 s) for the
   camera lap node to say ready on two consecutive statuses: camera `running`, 0 laps, similarity
   at least 0.90. Only then the planner starts. There is nothing to wait for by eye any more: the
   old rule "wait for READY before switching to auto" is replaced by the gate. If the launch
   refuses (one ERROR line beginning "refusing to drive: the camera lap source is not ready:" with
   the lap node's own reason, then it shuts down), the planner never started and the car did not
   move: fix the start view, relaunch. Switch to auto when the planner reports. Drive 11 laps,
   count them aloud, stop.
3. **Score.** `cd tools && uv run python -m pnp_session lap-src --d10 --bag <ccw> --bag <cw> --laps-counted 11 --laps-counted 11`.
   D10-C0 per direction: items (i) to (v) as D9-C0, with the same thresholds, and item (vi): the
   first planner status of the bag is later than the second of two consecutive ready statuses of
   the camera lap node. The `lapref` period and autocorrelation peak are printed per bag, with no
   threshold (the record grows). Attempts as before: at most 3 per direction per session, a new one
   only after the start view was changed, every attempt reported.
4. **D v0, only in a direction whose D10-C0 passed, same mark, same lights, same session, and
   only after the D10 gates passed on B10 at the freeze commit that is on the car.**
   `ros2 launch racer_bringup pnp.launch.py lap_memory:=true lap_mode:=v0 lap_source:=camera v_lap1_mps:=0.5 v_max_mps:=<ceiling>`.
   The lidar lap gate shadow (`lap_shadow:=start_scan`) is not started by default and not part of
   this command; add it only on purpose, for data. 10 laps. The owner holds the kill switch. Stop
   the run at any wall contact, at a second doubtful lap in the status, or at a second obstacle
   brake. If the status says `reference_suspect`, stop at once and relaunch from the mark; the run is not evidence. If it
   happens twice in a direction, change the start view and repeat the test pass (step 2).
5. **After.** `lap-src --drive --bag <bag>` for the lap table. Write the session note the same day.
