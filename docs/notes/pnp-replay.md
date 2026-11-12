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
        --dynamics-bundle <dynamics bundle> --v-max 0.4 --set max_intrusion_fraction=0.20 \
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
against 0.019; `--set samples=480` gives 0.024 on R1); and the replayed `K` ticks are exactly 0.02 s apart, the car's were 0.017 to 0.023 s.
Segment `L` was driven by another driver at 1 m/s, above the planner's cap, so `--clip-speed-to-cap`
feeds `min(wheel speed, v_max)` for it, and says so in the output.
