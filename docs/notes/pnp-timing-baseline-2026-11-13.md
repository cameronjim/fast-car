# Timing baseline, 2026-11-13 (R0 to R8, planner v0.1)

Evidence for the owner's open timing questions (pre-registration PnP v0.3 section 5, M-T). Nothing
here is a recommendation. Produced by

```sh
uv run --project tools --extra session python tools/pnp_session_report.py timing \
  --bag /Users/cameronjim/code/car-data/bags/2026-11-13T22-20-18_car_teleop \
  --caps-from-split evaluation/prereg/pnp-v0.2-split.yaml --limiter-cli <limiter_whatif_cli>
```

The counts of v0.2 section 4.2 reproduce: 0 TTC or obstacle, `watchdog` 8, `rate_limit` 13, 7 of 9
starts with an event. The 4.2 acceptance for C is a miss on this bag, as the pre-registration says.

What the tables say:

1. Watchdog limit: by recorder receive time 14 `/drive_raw` gaps in the windows are above 60 ms, 7 above 80 ms, 2 above 100 ms and 1 above 150 ms. The 8 recorded watchdog engagements saw ages of 68 to 359 ms (p50 71 ms) and 7 of them match a receive gap above 60 ms.
2. Rate limiter dt: the 13 recorded `rate_limit` engagements carry dt of 0.3 to 21.5 ms (p50 15.3 ms), 3 of them under 10 ms. Replayed through the unchanged limiter with the nominal 20 ms period, each cycle judged from the recorded previous output, the windows give 5 engagements against 17 with the recorded dt.
3. The replay does not reproduce the recorded events: with the recorded dt, 11 of 13 `rate_limit` engagements matched, 2 missed, 6 extra, and 3 of 8 watchdog engagements matched. That is a FAIL of the "must reproduce" wording. The bag does not record when safety_node received each `/drive_raw`; the replay takes the header stamp, which can pick the wrong command at a cycle.

Schema pnp-session/1. docs/notes/preregistration-pnp-v0.3.md section 5 (M-T). Counting window of each run: first non-zero `/drive_raw` command to the last `/pnp/planner/status` (v0.2 4.2). Percentiles are linear interpolation.

### `/drive_raw` gaps, recorder receive time

| run | window s | msgs | p50 ms | p99 ms | max ms | >40 ms | >60 ms | >80 ms | >100 ms | >150 ms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| R0 | 51.3 | 2562 | 20.0 | 34.1 | 59.0 | 10 | 0 | 0 | 0 | 0 |
| R1 | 51.4 | 2566 | 20.0 | 34.9 | 60.3 | 13 | 1 | 0 | 0 | 0 |
| R2 | 19.7 | 985 | 20.0 | 35.8 | 59.1 | 4 | 0 | 0 | 0 | 0 |
| R3 | 56.6 | 2822 | 20.0 | 34.7 | 92.7 | 12 | 4 | 3 | 0 | 0 |
| R4 | 40.0 | 2000 | 20.0 | 33.2 | 88.5 | 6 | 3 | 2 | 0 | 0 |
| R5 | 26.9 | 1346 | 19.9 | 36.6 | 61.2 | 8 | 1 | 0 | 0 | 0 |
| R6 | 26.0 | 1298 | 19.9 | 36.7 | 196.5 | 8 | 2 | 1 | 1 | 1 |
| R7 | 24.3 | 1212 | 20.0 | 38.3 | 136.7 | 9 | 2 | 1 | 1 | 0 |
| R8 | 17.2 | 861 | 19.9 | 36.8 | 71.5 | 8 | 1 | 0 | 0 | 0 |
| pooled | 313.4 | 15652 | 20.0 | 35.2 | 196.5 | 78 | 14 | 7 | 2 | 1 |

### `/drive_raw` gaps, header stamp

| run | window s | msgs | p50 ms | p99 ms | max ms | >40 ms | >60 ms | >80 ms | >100 ms | >150 ms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| R0 | 51.3 | 2562 | 20.0 | 31.3 | 48.7 | 3 | 0 | 0 | 0 | 0 |
| R1 | 51.4 | 2566 | 20.0 | 31.9 | 60.4 | 10 | 1 | 0 | 0 | 0 |
| R2 | 19.7 | 985 | 20.0 | 31.8 | 62.1 | 2 | 1 | 0 | 0 | 0 |
| R3 | 56.6 | 2822 | 20.0 | 30.1 | 88.8 | 6 | 3 | 1 | 0 | 0 |
| R4 | 40.0 | 2000 | 20.0 | 31.4 | 85.4 | 4 | 1 | 1 | 0 | 0 |
| R5 | 26.9 | 1346 | 20.0 | 32.8 | 47.8 | 4 | 0 | 0 | 0 | 0 |
| R6 | 26.0 | 1298 | 20.0 | 33.7 | 84.8 | 5 | 1 | 1 | 0 | 0 |
| R7 | 24.3 | 1212 | 20.0 | 33.4 | 76.1 | 4 | 1 | 0 | 0 | 0 |
| R8 | 17.2 | 861 | 20.0 | 33.1 | 53.4 | 5 | 0 | 0 | 0 | 0 |
| pooled | 313.4 | 15652 | 20.0 | 32.2 | 88.8 | 43 | 8 | 3 | 0 | 0 |

### Planner (`/pnp/planner/status`)

| run | ticks | solve p50 ms | solve p99 ms | solve max ms | tick gap p50 ms | tick gap p99 ms | tick gap max ms | bunched | FAULT ticks | STARTING ticks | STARTING ms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| R0 | 2562 | 2.8 | 12.2 | 21.8 | 20.0 | 31.3 | 48.7 | 29 | 0 | 0 | - |
| R1 | 2566 | 2.9 | 12.9 | 30.5 | 20.0 | 31.9 | 60.4 | 40 | 0 | 0 | - |
| R2 | 985 | 2.7 | 13.1 | 27.8 | 20.0 | 31.8 | 62.1 | 12 | 0 | 0 | - |
| R3 | 2822 | 3.1 | 12.5 | 43.8 | 20.0 | 30.1 | 88.8 | 30 | 0 | 0 | - |
| R4 | 2000 | 3.2 | 13.1 | 30.9 | 20.0 | 31.4 | 85.4 | 30 | 0 | 0 | - |
| R5 | 1346 | 2.5 | 13.5 | 32.2 | 20.0 | 32.8 | 47.8 | 17 | 0 | 0 | - |
| R6 | 1298 | 2.1 | 14.5 | 27.0 | 20.0 | 33.7 | 84.8 | 25 | 0 | 0 | - |
| R7 | 1221 | 3.0 | 12.7 | 36.0 | 20.0 | 33.4 | 76.1 | 16 | 0 | 0 | - |
| R8 | 861 | 2.1 | 13.0 | 35.8 | 20.0 | 33.1 | 53.4 | 13 | 0 | 0 | - |
| pooled | 15661 | 2.8 | 13.1 | 43.8 | 20.0 | 32.2 | 88.8 | 212 | 0 |  |  |

Plan age at publish: not carried by PlannerStatus. Tick gaps are header-stamp gaps; bunched is a gap under 10 ms (half a tick).

Status gaps above each limit:

| run | clock | >40 ms | >60 ms | >80 ms | >100 ms | >150 ms |
| --- | --- | --- | --- | --- | --- | --- |
| R0 | stamp | 3 | 0 | 0 | 0 | 0 |
| R0 | recv | 9 | 0 | 0 | 0 | 0 |
| R1 | stamp | 10 | 1 | 0 | 0 | 0 |
| R1 | recv | 15 | 2 | 0 | 0 | 0 |
| R2 | stamp | 2 | 1 | 0 | 0 | 0 |
| R2 | recv | 4 | 1 | 0 | 0 | 0 |
| R3 | stamp | 6 | 3 | 1 | 0 | 0 |
| R3 | recv | 12 | 4 | 2 | 0 | 0 |
| R4 | stamp | 4 | 1 | 1 | 0 | 0 |
| R4 | recv | 6 | 3 | 2 | 0 | 0 |
| R5 | stamp | 4 | 0 | 0 | 0 | 0 |
| R5 | recv | 8 | 1 | 0 | 0 | 0 |
| R6 | stamp | 5 | 1 | 1 | 0 | 0 |
| R6 | recv | 9 | 2 | 1 | 1 | 1 |
| R7 | stamp | 4 | 1 | 0 | 0 | 0 |
| R7 | recv | 10 | 3 | 1 | 1 | 0 |
| R8 | stamp | 5 | 0 | 0 | 0 | 0 |
| R8 | recv | 9 | 1 | 0 | 0 | 0 |
| pooled | stamp | 43 | 8 | 3 | 0 | 0 |
| pooled | recv | 82 | 17 | 6 | 2 | 1 |

### `/safety/events` ENGAGE by source, inside the window

| run | watchdog | rate_limit | ttc+reverse | other | watchdog /min | rate_limit /min | first 2 s: watchdog | first 2 s: rate_limit | start event |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| R0 | 0 | 1 | 0 | 0 | 0.00 | 1.17 | 0 | 0 | no |
| R1 | 0 | 1 | 0 | 0 | 0.00 | 1.17 | 0 | 0 | no |
| R2 | 0 | 1 | 0 | 0 | 0.00 | 3.05 | 0 | 1 | yes |
| R3 | 3 | 2 | 0 | 0 | 3.18 | 2.12 | 2 | 0 | yes |
| R4 | 2 | 2 | 0 | 0 | 3.00 | 3.00 | 1 | 1 | yes |
| R5 | 0 | 2 | 0 | 0 | 0.00 | 4.46 | 0 | 2 | yes |
| R6 | 1 | 3 | 0 | 0 | 2.31 | 6.92 | 1 | 2 | yes |
| R7 | 2 | 0 | 0 | 0 | 4.93 | 0.00 | 2 | 0 | yes |
| R8 | 0 | 1 | 0 | 0 | 0.00 | 3.48 | 0 | 1 | yes |
| pooled | 8 | 13 | 0 | 0 | 1.53 | 2.49 | 6 | 7 | 7/9 |

rate_limit, the limiter's own dt parsed from the event detail (pooled): n 13, min 0.3 ms, p50 15.3 ms, max 21.5 ms, under half a tick (10 ms) 3, unparsed 0
watchdog, the age parsed from the event detail (pooled): n 8, min 67.9 ms, p50 70.9 ms, max 359.1 ms, timeouts [0.06], unparsed 0

### Watchdog what-if

| limit ms | receive gaps above it (pooled) |
| --- | --- |
| 40 | 78 |
| 60 | 14 |
| 80 | 7 |
| 100 | 2 |
| 150 | 1 |

Receive time at the recorder is not safety_node's own clock: safety_node measures the age of /drive_raw on its steady clock from its own receive time. A gap seen here can be absent there and the other way round.

Agreement of the 60 ms row with the recorded `watchdog` engagements (tolerance 30 ms): 14 gaps above the limit, 8 recorded engagements, 7 matched, 1 recorded without a gap, 7 gaps without a recorded engagement.

### Rate limiter what-if

| replay | clipped cycles | rate_limit engagements | watchdog engagements |
| --- | --- | --- | --- |
| recorded dt, each cycle from the recorded previous output | 20 | 17 | 5 |
| nominal dt, each cycle from the recorded previous output | 7 | 5 | 5 |
| recorded dt, free-running chain | 8 | 7 | 5 |
| nominal dt, free-running chain | 5 | 4 | 5 |

Recorded `rate_limit` engagements in the windows: 13. In the table the counts are of the replay inside the counting windows.

Reproduction with the recorded dt (rows without a suffix: each cycle judged from the recorded previous output; `free_running`: the replay's own chain):

| events | recorded | replayed | matched | missed | extra | dt detail max diff |
| --- | --- | --- | --- | --- | --- | --- |
| rate_limit | 13 | 17 | 11 | 2 | 6 | 0.002 ms (11 pairs) |
| rate_limit_free_running | 13 | 7 | 2 | 11 | 5 | 0.001 ms (2 pairs) |
| watchdog | 8 | 5 | 3 | 5 | 2 | - |
| watchdog_free_running | 8 | 5 | 3 | 5 | 2 | - |

Not modelled: no scan (the obstacle gate never trips), no wheel odometry; the age of /drive_raw uses the command's header stamp as its arrival, not safety_node's own receive time.

### v0.2 4.2 acceptance for C

Outcome: miss (wall contacts as given: 0, not observable from a bag)

| item |  | description | measured |  | threshold | note |
| --- | --- | --- | --- | --- | --- | --- |
| 4.2-a | PASS | at least 300 s of PnP driving | 313.439 | >= | 300.0 |  |
| 4.2-b | FAIL | at least 8 standing starts at caps 0.5 and 1.0 | 7 | >= | 8 |  |
| 4.2-c | PASS | starts at both caps 0.5 and 1.0 | 2 | >= | 2 | caps seen [0.5, 1.0] |
| 4.2-d | FAIL | watchdog events per minute | 1.531 | <= | 0.2 | 8 events |
| 4.2-e | FAIL | rate_limit events per minute | 2.489 | <= | 0.2 | 13 events |
| 4.2-f | FAIL | starts with no watchdog or rate_limit event in the first 2.0 s | 2 | >= | 7 | 2 of 7 starts |
| 4.2-g | PASS | TTC and obstacle brakes at caps up to 1.0 | 0 | <= | 0 |  |
| 4.2-h | PASS | wall contacts | 0 | <= | 0 |  |

### Baseline check

Expected (pre-registered): {'obstacle': 0, 'watchdog': 8, 'rate_limit': 13, 'starts_with_event': 7, 'starts': 9}
Measured: {'obstacle': 0, 'watchdog': 8, 'rate_limit': 13, 'starts_with_event': 7, 'starts': 9}
Result: MATCH
