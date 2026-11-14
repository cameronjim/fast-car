# PnP replay harness

An open-loop replay of a recorded bag through the real Plug-and-Play code: the numpy perception
package (`racer_track_perception`, the object `track_perception_node` calls) and the real planner
(`planner_replay_cli`, a ROS-free build of `racer_planner::Planner`, one tick per `K` line of a
stream). `tools/pnp_replay.py` reads `/scan` and `/odom/wheel` (and the recorded
`/pnp/local_track` and `/pnp/planner/status`) from the bag, finds the PnP runs (`R0`, `R1`, ...)
and fast laps (`L`), and writes one JSON of perception and planner metrics per segment. Nothing in
it drives a car.

## Two commands

    uv run --project tools python tools/pnp_replay.py <bag> --segment R1 --check-fidelity \
        --dynamics-bundle <dynamics bundle> --v-max 0.4 \
        --out data/pnp_replay/r1.json
    uv run --project tools python tools/pnp_replay.py <bag> --segment R0 R1 R2 R3 L \
        ... --compare tools/pnp_replay/baselines/v0-2026-11-12.json

The first read of a bag takes seconds (the mcap summary index is used and only these topics are
decoded) and is cached as an `.npz` next to the output. `--check-fidelity` replays the geometric
tracer over the recorded scans and fails if any centreline station differs from the recorded
track by more than 1e-5 m. That holds only for the tracer that recorded the bag: the
2026-11-12 bag was recorded with geometric-v0 (difference exactly 0.0 on R0 to R3), and main's
tracer is v1 now, so against that bag the check reports the v0 to v1 difference and exits 1;
point `PNP_REPLAY_PERCEPTION_SRC` at an old checkout's `racer_track_perception/src` to replay
with the old tracer. `--tracks recorded|replay|none`
chooses where the planner's tracks come from. `planner_replay_cli` is built with the `racer_planner`
package; on a Mac copy the Linux binary out of the `ros-dev` build and set
`PLANNER_REPLAY_DOCKER_CLI` (and `PLANNER_REPLAY_DOCKER_IMAGE`) to run it in docker.
`tools/pnp_replay/baselines/v0-2026-11-12.json` holds the metrics of the unmodified v0 driver
(commit 8766bec: geometric-v0, planner with the NO_TRACK self-check, `max_intrusion_fraction`
0.10 for R0 and 0.20 for R1 to R3, tracks replayed with 0.03 s latency, `--clip-speed-to-cap`
for L, perception metrics of L over every fifth moving scan). The planner's v0 parameters no
longer exist on main; to regenerate that file run the tool from that commit.

## What open loop shows

The scans do not react to the replayed commands. So the replay shows whether the planner would have
stopped, crawled (a speed command under the plant's dead zone) or chattered (steering sign flips)
at each recorded pose, with the recorded perception or a replayed one. It does not show a lap, and
it cannot show a car recovering from its own mistake. Closed-loop questions go to the L5 canaries
with the car-like plant (`tests/l5_pnp`).

Two differences from the car, both visible in the numbers: the solver here runs 768 samples every
tick (about 1 ms per solve on a laptop), while the car's sample budget halved to about 480 samples
because its solves are slower, so recorded steering chatter is a little higher (std 0.024 rad
against 0.019; `--set samples=480` gives 0.024 on R1); and the replayed `K` ticks are exactly 0.02 s apart, the car's were 0.017 to 0.023 s.
Segment `L` was driven by another driver at 1 m/s, above the planner's cap, so `--clip-speed-to-cap`
feeds `min(wheel speed, v_max)` for it, and says so in the output.

## Planner v0.1 with recovery in the replay

The replayed planner may enter RECOVERING. The wheel odometry is the recorded one and does not
follow the replayed commands, so the wheel stall detector (block reason 3) would fire wherever the
recorded car stood still while the replayed planner drives: run replays with
`--set stall_hold_s=600`. The replay has no safety node, so the gated `/drive` is taken to pass
what the planner published (recovery needs it to be fresh). The speed shares do not count
RECOVERING ticks, and the steering change statistics use only the changes between two consecutive
DRIVING ticks (a jump across a recovery or a stop is not a change of the command). The v0 driver's
steering numbers recomputed with that rule from the committed baseline CSVs (old rule in
brackets): std R0 0.018 (0.018), R1 0.019 (0.020), R2 0.020 (0.021), R3 0.022 (0.022), L 0.022
(0.023) rad; sign flips per s 24.2, 27.6, 25.5, 26.8, 24.2: the rule moves the v0 numbers by under
0.002 rad. A start from rest ramps to the dead zone floor in equal steps of at most 0.45 of the
vehicle's acceleration limit per tick: up to two ticks in the dead zone, then the floor. Planner CSV
columns appended: `recovery_attempts`.

Planner v0.2 (timing, design 14.7) changes what a replay shows. The first ticks are STARTING (state
0, speed 0): the planner holds still until 25 consecutive callbacks each arrived within 30 ms of the
previous one and the solver has run on the real inputs for as many ticks, so the zero speed share
of a short cut includes that warm start (25 ticks is 0.5 s, 2.5 percent of a 20 s run before any
recorded gap restarts the count). A K line that arrives under half a tick after the last publish
republishes the last command (`bunched` 1). The output slews by the measured time between ticks,
not by the nominal 0.02 s. CSV columns appended, in this order: `elapsed_s` (the time the slew
used), `bunched`, `plan_stale`, `gap_reset`, `start_missing` (the first condition STARTING still
lacks, `StartMissing` of planner_logic.hpp), then the elapsed time check between two consecutive
rows: `steer_step_rad` against `steer_allow_rad` (0.45 times the steering rate limit times the real
time between the two ticks), `speed_up_step_mps` (growth of the speed magnitude) against
`speed_allow_mps`, and `slew_violation` (1 when either step is over its allowance). `now_s` is
written with microsecond resolution so that the allowance can be recomputed from the CSV. Replaying
with the recorded tick times of a bag (the `/pnp/planner/status` stamps) instead of the regular
0.02 s grid is what shows the elapsed time slew at work; the stream's K lines carry the tick times.

