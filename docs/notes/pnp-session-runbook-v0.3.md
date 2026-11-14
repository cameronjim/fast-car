# Car session runbook, Plug-and-Play v0.3

One page for the car. The full procedure, with every launch command spelled out, is
`tests/bench/procedures/pnp_session_v03.yaml` (design: `claude-docs/13-plug-and-play.md` 15.8). The
pass lines are quoted from `docs/notes/preregistration-pnp-v0.3.md` (v0.3) and
`preregistration-pnp-v0.2.md` (v0.2).

## Before anything moves

- Two people. Kill knob in hand. Wheels off the ground before the floor (step 0).
- Write down the VESC input voltage (5 s mean at rest) and the 0.8 V window every run must start
  inside.
- Every run: ccw first, two runs per direction, standing start, the bag must be growing
  (`ls -l <bag>` twice, ten seconds apart). The launch refuses without a recorder; that is the
  guard working.
- Nothing in this session sets a scheduling policy, a CPU governor or a safety parameter. Where it
  says "owner decision, not taken", write the table down and go on.

## Launch (inside the container)

```sh
docker exec -it car-stack bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/ros_ws/install/setup.bash && exec ros2 launch racer_bringup pnp.launch.py <arguments of the step>'
```

After a run, on the laptop: `<bag>` is the bag directory, `<out>` the session folder.

```sh
uv run --project tools --extra session python tools/pnp_session_report.py timing --bag <bag> --caps <caps> --out <out>/timing.json
uv run --project tools --extra session python tools/pnp_session_report.py laps --bag <bag> --out <out>/laps.json
```

## Steps

| Step | Command (arguments after `pnp.launch.py`) | Pass line | If it fails |
|---|---|---|---|
| 0 Build, bench, wheels off | `dynamics_bundle:=<dyn> backend:=geometric v_max_mps:=0.4`, plus `mppi_bench --background-load` | bench p50 under 6 ms and p99 under 12 ms; L6 checklist passed | stay off the floor, fix first |
| 1 Host facts (read only) | `pnp_session_report.py host-facts --out-dir <out>/host_facts` while the driver runs | none, stored in the log | nothing was set |
| 1 C v0.2 and lap observe | `v_max_mps:=0.5` then `1.0`, `lap_counter:=true require_lap_counter:=true lap_memory:=true lap_mode:=observe`, 10 laps, ccw ccw cw cw at each cap | v0.2 4.2: watchdog and rate_limit at most 0.2 per minute each, no event in the first 2.0 s of at least 7 of 8 starts, TTC and obstacle brakes 0, no contact; v0.3 1.5 D3-C0: lap count equals the camera counter's and 18 of 20 lap times within 0.3 s | report the counts; real-time scheduling goes to the owner; a D3-C0 fail skips steps 6 and 7 |
| 2 GP3 | `v_max_mps:=1.0`, observe mode, one launch, 20 laps ccw, nobody touches the car | v0.2 4.3: 20 consecutive laps, no wall contact, no BLOCKED hold over 5 s | the first attempt is the result; write it down |
| 3 Deskew | step 1 at cap 1.0 plus `deskew:=true` (skip if not on main or M-DSK not accepted) | latency p99 under 5 ms, no new event, mean speed not lower | relaunch with `deskew:=false` |
| 4 Yaw channel | candidate bundle as `dynamics_bundle`, only after `confirm` printed PASS (skip if not on main) | v0.3 2.4: P1 and Y1 ratios at most 1.00 over at least 120 s, then no new brake, mean speed at cap 1.0 not lower | deployed bundle `dyn-v0.1-2026-11-12` |
| 5 Net in shadow | step 1 plus `shadow_perception_bundle:=<perc>` | shadow latency p99 under 10 ms | relaunch without it |
| 6 D v0 | `v_max_mps:=1.0 v_lap1_mps:=0.5 lap_memory:=true lap_mode:=v0`, only if D3-S0, D3-S1 and D3-C0 passed | 10 laps, no contact, cap rises | `lap_memory:=false` |
| 7 D v1 | `lap_mode:=v1`, the variant the v0.3 1.4 table allows | 10 laps, no contact | `lap_mode:=v0` |
| 8 New layout | GP4 runs, at least 5 D-on runs, seeded interleaved order | v0.2 5.2: every run completes 10 laps, Delta positive in every run, interval excludes 0, median r at most 0.80, contacts no more than D off | no D |

Steps that need another brief's work (`lap_mode:=observe` brief 29, `deskew` brief 33, `confirm`
brief 31) are marked "skip if not on main" in the YAML.

## After each step

Read the timing report before the next step. A failed step rolls back to the previous
configuration and the session goes on with it. Wall contacts are what the two people and the phone
video say; the video decides a disagreement.
