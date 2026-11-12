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
        --dynamics-bundle <dynamics bundle> --v-max 0.4 --set conf_intrusion_zero=0.20 \
        --out data/pnp_replay/r1.json
    uv run --project tools python tools/pnp_replay.py <bag> --segment R0 R1 R2 R3 L \
        ... --compare tools/pnp_replay/baselines/v0-2026-11-12.json

The first read of a bag takes seconds (the mcap summary index is used and only these topics are
decoded) and is cached as an `.npz` next to the output. `--check-fidelity` replays the geometric
tracer over the recorded scans and fails if any centreline station differs from the recorded
track by more than 1e-5 m (it is exactly 0.0 on the 2026-11-12 bag). `--tracks recorded|replay|none`
chooses where the planner's tracks come from. `planner_replay_cli` is built with the `racer_planner`
package; on a Mac copy the Linux binary out of the `ros-dev` build and set
`PLANNER_REPLAY_DOCKER_CLI` (and `PLANNER_REPLAY_DOCKER_IMAGE`) to run it in docker.
`tools/pnp_replay/baselines/v0-2026-11-12.json` holds the metrics of the unmodified v0 driver.

## What open loop shows

The scans do not react to the replayed commands. So the replay shows whether the planner would have
stopped, crawled (a speed command under the plant's dead zone) or chattered (steering sign flips)
at each recorded pose, with the recorded perception or a replayed one. It does not show a lap, and
it cannot show a car recovering from its own mistake. Closed-loop questions go to the L5 canaries
with the car-like plant (`tests/l5_pnp`).

Two differences from the car, both visible in the numbers: the solver here runs 768 samples every
tick (about 1 ms per solve on a laptop), while the car's sample budget halved to about 480 samples
because its solves are slower, so recorded steering chatter is a little higher (std 0.024 rad
against 0.019); and the replayed `K` ticks are exactly 0.02 s apart, the car's were 0.017 to 0.023 s.
Segment `L` was driven by another driver at 1 m/s, above the planner's cap, so `--clip-speed-to-cap`
feeds `min(wheel speed, v_max)` for it, and says so in the output.

## Planner v0.1 in the replay

The replayed planner builds the free-space field of every scan, takes the track confidence from
the replayed or recorded track, and may enter RECOVERING. Two things to know when reading the
numbers. The wheel odometry is the recorded one and does not follow the replayed commands, so the
wheel stall detector (block reason 3) would fire wherever the recorded car stood still while the
replayed planner drives: run replays with `--set stall_hold_s=600`. The replay has no safety node,
so the gated `/drive` is taken to pass what the planner published (recovery needs it to be fresh).
The speed shares do not count RECOVERING ticks, and the steering change statistics use only the
changes between two consecutive DRIVING ticks (a jump across a recovery or a stop is not a change
of the command). Planner CSV columns appended: `track_confidence`, `freespace_goal_x_m`,
`freespace_goal_y_m`, `freespace_reach_m`, `block_reason`, `recovery_attempts`.

Steering change statistics of the v0 driver, recomputed with the consecutive-pair rule from the
committed baseline CSVs (old rule in brackets): std R0 0.018 (0.018), R1 0.019 (0.020), R2 0.020
(0.021), R3 0.022 (0.022), L 0.022 (0.023) rad; sign flips per s 24.2, 27.6, 25.5, 26.8, 24.2. The
rule changes the v0 numbers by under 0.002 rad, so the comparison with `baselines/v0-2026-11-12.json`
stands. The fixture's recorded tracks are geometric-v0; main's tracer is geometric-v1 since #145,
so the harness (which runs the current tracer) no longer reproduces that bag's tracks and
`--check-fidelity` against it fails. The self-test stays in the suite against a frozen test-only
copy of the v0 tracer (`tools/tests/fixtures/geometric_v0_tracer.py`, swapped in for the length of
the test), and `test_harness_runs_the_current_tracer` pins that the harness itself uses the
current one. Delete the copy together with the 2026-11-12 fixture.
