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


## Fault injection (roadmap P3.4)

`tools/pnp_replay/faults.py` mutates a recording (a pure function of the `Recording` dict, a time
window in bag seconds of ARRIVAL time, and a seed; the input is never changed): `nan_ranges` (10,
50 or 100 percent of beams set to NaN, inf or 0; wheel speed NaN), `drop`, `stamp_jump_forward` and
`stamp_jump_backward` (header stamps plus or minus 5 s, arrival unchanged), `out_of_order`
(adjacent pairs of messages swap places in arrival order), `frozen` (the last message before the
window repeated, stamp included), `frozen_fresh_stamp` (the payload repeated, the stamp advancing)
and `track_garbage` (the recorded `/pnp/local_track` replaced by NaN arrays, or a valid_count of
17). Float scan ranges with NaN do not fit the recording's uint16 quarter millimetres, so a mutated
recording carries `scan_f`, which `bag.scan_ranges` reads first.

`tools/pnp_replay/fault_checks.py` holds the rules and `tools/pnp_replay/fault_run.py` the cases
(`CASES`, one id per fault, stream and size). Perception is checked on what
`TrackPerceiver.process` returns for the mutated scans (never raises, arrays of 16, valid_count 0 to
16 with the leading entries finite, self-check numbers never infinite, a scan with no usable beam
gives valid_count 0). The planner is checked on the CSV of `planner_replay_cli` fed the mutated
stream, with the tracks of the recording: the always rules (finite, speed within
[-reverse limit, v_cap] and v_max, steering within the vehicle limits, nothing between 0 and the
speed floor outside a start ramp), the hard timeouts (no good scan for more than 0.60 s: speed 0
and NO_SCAN from 0.60 s plus two ticks on; no good odometry for more than 0.20 s: NO_ODOM; DRIVING
again within 1.0 s of the end), the short faults (the speed does not exceed what it was before the
fault plus one ramp step; only DRIVING, NO_SCAN and NO_ODOM appear), 100 percent unusable scans
(BLOCKED or NO_SCAN with speed 0), garbage tracks (never NO_TRACK, track confidence 0 within a
tick) and out of order scans (no negative age). The bounds are the HARD timeouts of the v0.2
design, so they hold for a planner with a step at 0.35 s and 0.10 s and for one that makes
staleness gradual.

What the planner cannot see is not asserted. A frozen scan with fresh stamps (plausible ranges, a
fresh header) is not detected: the test says so and checks the always rules. Wheel odometry is
stamped with its ARRIVAL time (`on_odom` in the node, the `O` line of the stream), so a stamp jump,
a repeated or an out of order sample is invisible to it; only silence (dropped samples, NaN
speeds, which the node does not let refresh the clock) is detected. Those faults are caught, if at
all, by layer 3 and layer 1. Three track faults cannot be sent through the stream (a track of
another length, another frame, spacing 0.19: the `T` line is fixed at 16 stations, base_link,
0.20 m); `faults.track_garbage` refuses them, and the node's own validity check and gtest cover
them.

The per-push tests are `tools/tests/test_pnp_replay_faults.py` (`test_perception[<case>]` and
`test_planner[<case>]` on `tools/tests/fixtures/pnp_fault_2026-11-13.npz`: 8 s from the start of R2
and 6 s from the middle of R7 of bag 2026-11-13T22-20-18; `python -m pnp_replay.cut_fixture <bag>
--fault-fixture` regenerates it) and `.github/scripts/pnp_replay_test.sh` runs them with the built
CLI. A finding against the planner is a strict xfail with the bag time and what was commanded.

The full-bag run, a local command (every case at three seeded windows inside each PnP run, both
consumers, JSON of the violations and a table):

    cd tools && PLANNER_REPLAY_DOCKER_CLI=<Linux planner_replay_cli> uv run python -m \
        pnp_replay.fault_run /Users/cameronjim/code/car-data/bags/2026-11-13T22-20-18_car_teleop \
        --dynamics-bundle /abs/path/config/pnp/fixtures/dyn-car-2026-11-12 --v-max 0.5 \
        --v-max-segment R2=1.0 R5=0.7 R6=0.7 R7=1.0 R8=1.0 --seed 20261113 --windows 3

There the short-fault speed bound is max(speed before, the unfaulted replay's speed at that tick)
plus one step, so a fault is not blamed for an acceleration the run was making anyway; the pytest
cases use the literal rule on the steady 1.0 m/s stretch of R7. On a Mac under docker every path
passed to the CLI must be absolute and under a directory docker shares (run pytest with
`--basetemp` under your home, not the default `/private/var/folders`).

## Rate allowance and the 2026-11-13 baseline

`--ticks recorded` replays the planner at the arrival times of the recorded `/pnp/planner/status`
(the car's bunching and gaps) instead of every 0.02 s. `planner.rate_allowance` counts the
consecutive-publish pairs whose steering step, or growth of the speed's magnitude, exceeds 0.45 of
the vehicle's rate limit (`steering.max_rate_rad_per_s`, `actuation.max_acceleration_mps2` of the
generated bindings) times the elapsed time between the two, the elapsed time clamped to one tick.
On the uniform grid it is 0 (P0); on the recorded ticks it is not. `--window NAME=FROM:TO` names a
window and `--v-max-segment SEG=M_S` gives a run its own cap, so one invocation covers all runs.
`tools/pnp_replay/baselines/2026-11-13-v0.1.json` is the planner on main (v0.1, bundle
dyn-car-2026-11-12, tracer geometric-v1 replayed with 0.03 s latency, fidelity 0.0 m against the
recording on every run) over R0 to R8 of bag 2026-11-13T22-20-18 and the gap follow segment P0
(16.6 to 113.9 s, cap 0.54, planner cap 0.55, uniform ticks, no status to take ticks from):

    uv run python tools/pnp_replay.py <bag> --cache <recording.npz> \
        --segment R0 R1 R2 R3 R4 R5 R6 R7 R8 P0 --window P0=16.6:113.9 \
        --dynamics-bundle config/pnp/fixtures/dyn-car-2026-11-12 --v-max 0.5 \
        --v-max-segment R2=1.0 --v-max-segment R5=0.7 --v-max-segment R6=0.7 \
        --v-max-segment R7=1.0 --v-max-segment R8=1.0 --v-max-segment P0=0.55 \
        --ticks recorded --tracks replay --check-fidelity \
        --set solve_halve_over_ms=200 --set stall_hold_s=600 --out 2026-11-13-v0.1.json

The planner of brief 25 is accepted at zero violation pairs on every run.
