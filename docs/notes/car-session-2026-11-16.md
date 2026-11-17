# Car session, 2026-11-15 evening to 2026-11-16 (bags 2026-11-16T02-18 to 23-04 UTC)

Analysis only. No node code, parameter or launch file changed. Everything below is read from five
bags in `car-data/bags/` and the run logs in `car-data/pnp/exc-2026-11-16/` (`DEVIATIONS.txt`, one
launch log per block), plus the owner's observations as given. Local time is UTC-8, so the
`02-18-54` and `02-29-27` bags are the evening of 2026-11-15 and the other three are the
afternoon of 2026-11-16. This is "the 2026-11-15 session" of the v0.5 dynamics pre-registration;
section 8 lists what that registration planned against what was driven.

## 1. Headlines

1. **Safety timing.** In the two 20 lap runs (`02-18-54` and `22-38-47`) safety_node recorded no
   `watchdog` and no `rate_limit` engagement inside the counting window of pre-registration v0.2
   4.2 (113.9 s and 117.0 s). The 2026-11-14 run of the same kind had 3 and 2 in 115 s (2.6 per
   minute). The two PnP excitation blocks P ccw and P cw did not repeat that: 6 and 5 events in
   102.4 s and 101.6 s (3.5 and 3.0 per minute, target 0.2). Pooled over the four full PnP runs
   (434.9 s): `watchdog` 4 (0.55 per minute), `rate_limit` 7 (0.97 per minute).
2. **Where the glitches come from.** The receipt log (`/safety/cycle`) was not enabled in any of
   these bags, so the registered leg breakdown cannot be made. From what is recorded: in 8 of 11
   watchdog engagements that can be located, safety_node's own cycle (the `/drive` stamps) had
   run 45 ms or more late, usually while the planner and the recorder saw commands arriving on
   time. Half of the `rate_limit` engagements in the PnP runs (10 of 20) are bunched cycles
   (dt under 12 ms, a late cycle then a catch-up). Planner solve time is not the cause (p99 11 to
   14 ms). Most events fall in the first 5 s after a planner launch (section 5).
3. **GP3-R1.** The bag `22-38-47` supports the owner's recorded outcome: the camera counter has
   20 consecutive events with peaks 0.916 to 0.950, one launch, wheel distance 106.0 m. Wall
   contact cannot be seen in a bag; it rests on the owner and the film (not viewed here). The
   earlier attempt `02-18-54` counted 10 events with peaks 0.802 to 0.873.
4. **Gap follow (race profile) refuses at the first corners.** 55 TTC brake engagements in 10
   blocks, 25 of them latched (at least 1.5 s), and the 25 match the 25 reverse escapes in the
   launch logs one for one. No `watchdog` event in any gap follow block. Details for the fix task
   in section 7.
5. **Recorder.** Of five bags only `02-29-27` was not finalised (section 3).

## 2. How the bags were read

- mcap's `StreamReader` read leniently, record by record, stopping at a truncated tail. No ROS
  needed; message definitions come from the bag. Four of five bags have a footer and a
  `metadata.yaml`; the readers got every message.
- Counting window of a PnP run: first non-zero `/drive_raw` command to the last
  `/pnp/planner/status` (v0.2 4.2). `ENGAGE` records only (`phase == 0`); a `watchdog` after the
  planner stopped is the stop mechanism and is not counted. For a gap follow block: first
  non-zero `/drive_raw` after the launch to the SIGINT line of the launch log. Launch log times
  are epoch seconds on the same clock as the bag log times.
- Direction is from measured heading: the car has no yaw sensor in these bags (`/odom/wheel`
  pose and angular z are constant zero, `/vesc/sensors/imu` is empty), so heading is dead reckoned
  from `/odom/wheel` speed and the `/drive` steering, `v tan(delta) / wheelbase`, wheelbase
  0.3302 m from `vehicle_params.yaml`. Sign is reliable (ccw positive). Scale is about 8 percent
  high: on `22-38-47` it gives 21.7 turns for 20 laps. Heading and position drift, so they are
  used for sign, rough turn counts and "which part of a lap", not for coordinates.
- Speed is `/odom/wheel` `twist.linear.x`, signed, over the whole window (standing starts and
  escapes included); the moving-only median is added where it differs.
- The scripts were scratch, not committed. The numbers can be rederived from the method above.

## 3. The bags, and the unfinalised recorder

| bag (UTC) | local | readable | finalised | messages read | what is in it |
|---|---|---|---|---|---|
| 02-18-54 | 11-15 18:18 | 620.1 s | yes | 367,695 | PnP, one run, 20 lap attempt |
| 02-29-27 | 11-15 18:29 | 164.8 s | **no** | 93,250 | idle stack, nothing commanded |
| 22-38-47 | 11-16 14:38 | 385.4 s | yes | 236,735 | PnP, GP3-R1 |
| 22-45-39 | 11-16 14:45 | 1100.0 s | yes | 659,832 | PnP launches, P ccw, gap follow ccw blocks |
| 23-04-09 | 11-16 15:04 | 713.2 s | yes | 430,203 | P cw, gap follow cw blocks |

For the four finalised bags the lenient reader's message counts equal the counts in
`metadata.yaml`. The owner expected several unfinalised bags; in fact one is: `02-29-27`. It has
no `metadata.yaml`, no footer and no summary. The reader consumed every record up to a truncated
last record (8,240 `/drive`, 8,244 `/odom/wheel`, 1,636 `/scan`, 1,823 `/rosout`; nothing on
`/drive_raw`, `/pnp/*`, `/lap/*`, `/safety/events` or the record cameras). The file is 22,835,200
bytes, exactly 5,575 pages of 4,096, which is what a writer that was never closed leaves behind
after its last buffered write reached disk. What the bag says about why, which is not much:

- How the recordings end differs. `02-18-54`, `22-38-47` and `22-45-39` end 4.2 to 5.2 s after
  the viewer's `foxglove_bridge` removed its channels (the nodes going away), so the recorder
  was given time to close. `23-04-09` ends 0.1 s after the viewer's last unsubscribe with no
  such lines and was nevertheless finalised; `02-29-27` ends 0.4 s after one in the same way and
  was not. So the tail pattern does not by itself tell the two apart.
- Rails give no explanation: Jetson 5 V rail 5.016 V at the last 5 Hz sample (no sag visible at
  that rate), VESC input 11.0 V. A brownout shorter than 200 ms would not show.
- None of the 12 launch logs has an exit -11 (the Fast-DDS shutdown race of
  `shutdown-segv-2026-11-14.md`). The P blocks stop with SIGINT and `lap_counter_node` (and in P cw
  `track_perception_node` too) exit 1 with an rclpy `ExternalShutdownException` traceback, which
  is cosmetic and unrelated to the recorder.

So the bag cannot separate a power loss on the Jetson from the recorder being stopped without
time to close. Host logs (docker, dmesg, the app) would; none are in the data. The owner's note
that the battery was low fits either. The bag shows no command was ever issued
(`/drive` is all zero, no `/drive_raw`, odometry zero, VESC current 0.00 A), so it cannot say why
the car "would not drive": it records an idle car. The VESC input was 11.0 V flat, the lowest
resting reading of the five bags (11.2, 12.0, 11.9, 11.6 at the start of the others).

## 4. Runs

Cap is the planner `v_cap` or the gap follow `max_speed`. Duration is the window of section 2.
Speed is odometry. Turns are dead reckoned heading turns (ccw positive). Rail is the VESC input
voltage (0.1 V resolution) just before the window, its minimum in the window, and just after;
the Jetson 5 V rail stayed between 5.000 and 5.024 V in every window.

| run | what drove | cap m/s | direction (heading) | s | v p50 / p95 / max | turns | VESC V before / min / after |
|---|---|---|---|---|---|---|---|
| 02-18-54 | PnP geometric, dyn-v0.1 | 1.0 | ccw | 113.9 | 0.92 / 1.00 / 1.04 | +21.4 | 11.1 / 11.0 / 11.1 |
| 22-38-47 | PnP geometric, dyn-v0.1 | 1.0 | ccw | 117.0 | 0.94 / 1.01 / 1.09 | +21.7 | 12.0 / 11.9 / 11.9 |
| aborted launch (lap counter check) | PnP | ramp to 0.5 | ccw | 3.7 | 0.36 / 0.48 / 0.48 | +0.07 | 11.8 / 11.8 / 11.8 |
| P ccw | PnP, cap schedule | 0.5 / 1.0 / 0.7 / 1.3 | ccw | 102.4 | 0.66 / 1.21 / 1.38 | +14.9 | 11.8 / 11.7 / 11.7 |
| G ccw 1.0 | gap follow, race profile | 1.0 | ccw | 32.1 | 0.79 / 1.04 / 1.20 | +2.7 | 11.7 / 11.7 / 11.7 |
| G ccw 1.3 | same | 1.3 | ccw | 47.4 | 0.49 / 1.30 / 1.45 | +3.9 | 11.7 / 11.6 / 11.7 |
| L ccw 0.6 | gap follow, lane profile, deepest ray | 0.6 | ccw | 42.0 | 0.35 / 0.42 / 0.48 | +3.2 | 11.6 / 11.6 / 11.6 |
| F ccw 1.4 | gap follow, race profile | 1.4 | ccw | 22.4 | 0.34 / 1.36 / 1.47 | +1.3 | 11.6 / 11.6 / 11.6 |
| F ccw 1.5 | same | 1.5 | ccw, then turned (section 7.3) | 63.1 | 0.37 / 1.47 / 1.60 | +3.9 | 11.6 / 11.5 / 11.6 |
| P cw | PnP, cap schedule | 0.5 / 1.0 / 0.7 / 1.3 | cw | 101.6 | 0.67 / 1.25 / 1.43 | -15.9 | 11.5 / 11.5 / 11.5 |
| G cw 1.0 | gap follow, race | 1.0 | cw | 32.4 | 0.93 / 1.04 / 1.18 | -4.1 | 11.5 / 11.4 / 11.5 |
| G cw 1.3 | same | 1.3 | cw | 48.8 | 0.97 / 1.35 / 1.48 | -6.0 | 11.5 / 11.4 / 11.4 |
| L cw 0.6 | gap follow, lane | 0.6 | cw | 43.9 | 0.35 / 0.44 / 0.55 | -3.3 | 11.4 / 11.4 / 11.4 |
| F cw 1.4 | gap follow, race | 1.4 | cw | 23.0 | 1.03 / 1.44 / 1.53 | -3.0 | 11.4 / 11.4 / 11.4 |
| F cw 1.5 | same | 1.5 | cw | 64.0 | 1.22 / 1.52 / 1.62 | -10.0 | 11.4 / 11.3 / 11.4 |

Notes. Direction by heading agrees with the labels for every block except the tail of F ccw 1.5.
The aborted launch is the `require_lap_counter` refusal; the earlier abort (bad bundle path) never
commanded the car. Measured speed exceeds the cap by up to 8 to 14 percent (odometry max 1.09 at
cap 1.0, 1.48 at 1.3, 1.62 at 1.5). Driving time of the
excitation blocks: command windows sum to 623 s, time with odometry above 0.05 m/s sums to 578 s,
against the owner's cap of 600 s of driving; which one the cap means is for the owner.

Lap counter events in the P blocks: P ccw 12, P cw 13 (peaks 0.82 to 0.97). One P ccw event
covers two laps (11.24 m). Lap times follow the cap schedule (4.7 to 13.7 s). Not a gate.

## 5. Safety node timing

### 5.1 Events by run

Inside the counting window; the per-minute figure is `watchdog` plus `rate_limit` over the window.
"Before" is the events from 3 s ahead of the first command to the first command (planner already
publishing zero commands), which the registered rule does not count.

| run | `watchdog` | `rate_limit` | TTC brake | TTC info | glitches per min | before |
|---|---|---|---|---|---|---|
| 02-18-54 | 0 | 0 | 0 | 0 | 0.00 | 1 watchdog, 1 rate_limit at -0.74 s |
| 22-38-47 | 0 | 0 | 0 | 0 | 0.00 | none |
| aborted launch | 2 | 3 | 0 | 0 | (3.7 s) | 2 rate_limit |
| P ccw | 1 | 5 | 0 | 0 | 3.51 (wd 0.59, rl 2.93) | 4 watchdog, 6 rate_limit |
| P cw | 3 | 2 | 0 | 0 | 2.95 (wd 1.77, rl 1.18) | 1 watchdog, 1 rate_limit |
| pooled 4 full runs, 434.9 s | 4 | 7 | 0 | 0 | 1.52 (wd 0.55, rl 0.97) | |

With the aborted launch counted as a PnP run (438.6 s): 6 and 10, 2.19 per minute. By the
registered rule, 4.2 asks for at least 300 s and at least 8 standing starts at caps 0.5 and 1.0;
this session has 4 full starts (and the lap runs start at the lap-guidance cap, not 0.5), so the
4.2 acceptance cannot be judged from it and no verdict is given. The counts are reported and the
thresholds do not move. In the first 2.0 s after the first command: 02-18-54 and 22-38-47 have
no event, P ccw has one (+0.03 s), P cw has three (+0.25 s, +1.30 s and a rate_limit at
+1.32 s).

Gap follow blocks (419 s in total): `watchdog` 0. `rate_limit` 111, of which 0 are bunched
cycles. These are the limiter doing its job on command steps: 57 land within 0.3 s of a TTC
brake engage or release, and 64 within 0.1 s of a speed request step above 0.3 m/s. They are not
timing glitches and are not in the 0.2 per minute figure.

### 5.2 The receipt log

`publish_cycle_record` defaults to false and no launch file sets it, so no bag has
`/safety/cycle`. The registered receipt breakdown (planner, DDS delivery, safety timer) cannot be
made from these bags and none is claimed. For the next session the parameter has to be set at
the safety_node launch; this note changes nothing.

### 5.3 What the recorded streams do show

Per window, gaps in milliseconds. "Stamp" is `/drive_raw` header stamp (the planner's own
clock at publish), "receive" is the recorder's log time, "cycle" is the `/drive` stamp (safety_node's
own gate cycle). The recorder's receive time is a noisy witness: in P cw it saw 16 gaps above 60 ms
and its own receipt latency has a p99 of 45 ms.

| run | stamp p50 / p99 / max | stamp >40 / >60 | receive p99 / max | receive >60 / >100 | cycle p99 / max | cycle >40 / >60 | solve p50 / p99 / max ms |
|---|---|---|---|---|---|---|---|
| 02-18-54 | 20.0 / 27.5 / 47.1 | 3 / 0 | 34.0 / 82.6 | 5 / 0 | 25.1 / 48.3 | 2 / 0 | 2.07 / 10.4 / 36.9 |
| 22-38-47 | 20.0 / 26.1 / 63.8 | 3 / 1 | 31.3 / 95.4 | 3 / 0 | 24.9 / 43.8 | 1 / 0 | 2.45 / 11.1 / 21.3 |
| P ccw | 20.0 / 27.6 / 60.9 | 9 / 1 | 36.2 / 104.7 | 7 / 1 | 26.2 / 81.7 | 7 / 2 | 2.54 / 13.8 / 44.7 |
| P cw | 20.0 / 29.3 / 68.2 | 18 / 1 | 50.1 / 83.4 | 16 / 0 | 26.1 / 47.3 | 3 / 0 | 1.75 / 12.4 / 27.9 |

Each PnP launch logs one "a solve took N ms, over the budget; the sample count is halved" warning
at its start (47.8, 23.1, 19.6, 17.9 and 27.4 ms; on 22-38-47 the status shows 256 samples at the
first tick and 128 by cycle 582). Scan age at the planner is p50 0.15 s, p99 0.21 s in both lap runs.

### 5.4 The 12 non-stop watchdog engagements

Located by the age in the event text (limit 0.06 s): the latched command is the last `/drive_raw`
stamped before the receipt time, and the table reads its gap to the next command at the planner
(stamp) and at the recorder, and the safety cycle gap before the event. 11 of the 12 can be
located (the 02-18-54 one, 0.74 s before the first command, has no command to match). Planner
side means a stamp gap of 45 ms or more.

| where | event (s into bag) | age ms | planner stamp gap ms | recorder gap ms | safety cycle gap ms | reading |
|---|---|---|---|---|---|---|
| aborted, in window | 194.96 | 67.3 | 62.7 | 78.7 | 50.3 | planner late and cycle late |
| aborted, in window | 195.46 | 77.4 | 18.5 | 26.2 | 54.1 | cycle late, commands on time |
| P ccw, before | 263.02 | 69.7 | 21.8 | 337.2 | 15.7 | recorder gap at startup |
| P ccw, before | 263.47 | 61.9 | 19.5 | 21.9 | 49.8 | cycle late, commands on time |
| P ccw, before | 263.66 | 65.0 | 10.4 | 49.9 | 68.4 | cycle late, recorder gap |
| P ccw, before | 264.09 | 80.2 | 19.7 | 0.3 | 52.1 | cycle late, commands on time |
| P ccw, in window | 269.51 | 70.1 | 50.5 | 29.1 | 81.7 | planner (44.7 ms solve) and cycle late |
| P cw, before | 66.99 | 84.6 | 16.9 | 13.2 | 66.7 | cycle late, commands on time |
| P cw, in window | 67.98 | 83.9 | 40.0 | 27.3 | 47.3 | cycle late, commands on time |
| P cw, in window | 69.04 | 63.3 | 31.8 | 38.6 | 24.2 | none above 45 ms (63 ms, just over) |
| P cw, in window | 103.32 | 60.8 | 19.9 | 50.6 | 23.7 | recorder gap |
| 02-18-54, before | 275.51 | 72.4 | not located | | 15.8 | not located |

Reading, with its limit: 8 of 11 had a safety cycle 45 ms or more late (the gate's own stamps),
6 of 11 had commands arriving on time at both planner and recorder yet the node reported an age
above 60 ms, 2 were planner late, 3 had a recorder gap of 45 ms or more. That points at
safety_node's own scheduling (or at a stall of the whole machine that also delays its subscriber)
more than at the planner or at delivery to the recorder, but only the receipt log can separate
"the node did not run" from "the message did not reach the node". The 20 `rate_limit`
engagements before and in the PnP windows split 10 bunched (dt 2.4 to 11.8 ms: a late cycle then
a short one) and 10 at a normal dt (steps in the command). Of the 11 in-window events of P ccw
and P cw, 7 fall within 5.2 s of the run's first command and 4 later (P ccw at +21.6 s and
+26.3 s, P cw at +35.58 s and +35.61 s). Counting the "before" events too, 19 of the 23 events of
the two P blocks lie between 1.4 s before and 5.2 s after the first command. The two lap runs,
launched from the app, had none in their windows.

## 6. The two 20 lap runs and the camera lap counter

### 6.1 02-18-54 (an attempt, counter 10 of 20)

Ten counter events, crossings (s after the first command): 9.2, 15.0, 20.6, 32.0, 37.5, 48.7,
59.9, 65.5, 77.0, 99.5. Lap times 6.20, 5.78, 5.57, 11.39, 5.56, 11.20, 11.16, 5.66, 11.45,
22.51 s; the doubled and quadrupled entries are missed laps (the counter's own path length per
event is 5.1 to 5.3 m for a single lap, 10.2 to 10.5 m for two, 20.6 m for four). Peaks of the
counted events: 0.873, 0.829, 0.824, 0.814, 0.802, 0.815, 0.846, 0.806, 0.803, 0.854; threshold
0.80, so three counted peaks were within 0.01 of it. No event after 99.5 s although the car
drove 14.4 s more (about 2.6 laps). The counter's `armed` flag was true at every 1 Hz status
sample, so re-arming was not the limit; the misses are below-threshold peaks, as the owner
said. The bag stores a peak only for counted laps; the 1 Hz `/lap/status` similarity read 0.67,
0.80, 0.54 and 0.45 near four of the missed crossings, which are samples, not peaks, and so are
lower bounds that neither confirm nor contradict "below 0.80".

Physical laps, independent of the counter: crossings extrapolated at 5.62 s from the first one
fit 19 crossings by 110 s; wheel distance 103.1 m is 19.6 lap lengths of 5.25 m; heading turns
21.4 (19.7 after the 8 percent scale seen on 22-38-47). The owner counted 20 laps; these
independent measures read 19 to 20, so I do not state 20 from the bag. The count criterion of
4.3 is not met in either reading (10 events).

### 6.2 22-38-47 (GP3-R1)

Twenty events, laps 1 to 20 in order, crossings from 8.9 to 116.2 s after the first command.
Lap 1 took 8.00 s and laps 2 to 20 took 5.47 to 5.80 s (mean 5.64). Peaks 0.916 to 0.950
(threshold 0.80). Path length per lap 5.15 to 5.34 m for laps 2 to 20, 5.48 m for lap 1. The
counter's `armed` flag cycles false then true after every lap, as designed.

What I checked against the registered criteria of 4.3 (one continuous run, one launch, nobody
touches the car, 20 consecutive laps on one layout, no wall contact, count from the camera
counter, wheel distance at least 0.9 times 20 lap lengths, cap 1.0, ccw first, BLOCKED hold at
most 5 s, a recovery without contact does not fail):

- Count: 20 consecutive camera counter events, no gap, no lap index repeated. Met in the bag.
- One launch: the planner's cycle counter runs 89 to 5987 without a reset, `/drive_raw` is
  continuous, the counter's lap index never resets. Consistent with one launch. "Nobody touches
  the car" is not observable from a bag.
- Wheel distance: 106.0 m over the window, 20.1 lap lengths of 5.27 m (the counter's median),
  against a floor of 94.9 m (0.9 times 20). Met. The lap length comes from the counter's own
  odometry path, so this checks the camera count against the wheels, not the wheels alone.
- Cap and direction: planner `v_cap` never above 1.00; odometry max 1.09 m/s (0.4 percent of
  the time above 1.05). Heading +21.7 turns, ccw. Planner status shows `dyn-v0.1-2026-11-12`
  and `geometric-v1`.
- Start: the planner entered BLOCKED (speed floor, 0.6 s) then RECOVERING (2.5 s, 125 reverse
  commands) before and at the first command, one recovery attempt; odometry shows no reverse
  motion (no sample below -0.05 m/s). Longest BLOCKED or RECOVERING stretch 3.1 s, under 5 s.
  This is why lap 1 took 8.0 s. A recovery without contact does not fail the run and is counted.
- Safety: 0 `watchdog`, 0 `rate_limit`, 0 TTC or obstacle events inside the window.
- Wall contact: not observable here. Owner observed none and filmed it; I did not view the film.
  Bag proxies only: no TTC brake, no safety events, the planner's own minimum plan clearance over
  the run was 0.109 m (a planning number, not a measured gap to the wall).

So the bag holds the registered criteria that a bag can hold; contact rests on the owner and the
video. "GP3-R1 met" is the owner's recorded outcome under owner decision 4 of 2026-11-14. Two
facts to keep next to it: the start position was moved to face a more distinctive view after
`02-18-54`, and `02-18-54` was itself a 20 lap attempt on the evening before that did not meet the
camera count (10). Whether that attempt or `22-38-47` is "the" repeat under "the first attempt at
a configuration is the result" (v0.2 4.3) is not decided here. The roadmap line says the first
attempt (2026-11-14) result stands.

Why the counter did better, from its design (`lap_counter.py`): it takes the first frame after
the car's speed reaches the start speed as its reference, so moving the start to a distinctive
view changed the reference. Peaks rose from 0.80 to 0.87 to 0.92 to 0.95. In the 1 Hz status of
22-38-47 the similarity is above 0.55 for 2 to 3 samples around each crossing; in 02-18-54 the
samples are mostly below 0.35 apart from the crossings themselves.

## 7. Excitation blocks

### 7.1 PnP blocks P ccw and P cw

Cap schedule 0.5, 1.0, 0.7, 1.3 m/s from `blockP.sh`, launched over SSH, planner parameters from
`excitation_params.yaml` (`use_lap_guidance` true, `v_no_guidance_mps` 0.5). Windows 102.4 s and
101.6 s. Timing events in section 5. Both blocks reached the 1.3 rung (odometry max 1.38 and
1.43). BLOCKED episodes (speed floor, each short): P ccw one at +39.25 s of 2 ticks; P cw three,
at +0.19 s (2 ticks), +37.58 s and +69.61 s (15 ticks, 0.30 s each). P ccw also shows 6 ticks of
NO_ODOM before the first command.

### 7.2 Gap follow blocks: what the safety node did

| block | brake engagements (latched, at least 1.5 s) | reverse escapes (log) | `rate_limit` | `watchdog` | TTC info | min speed m/s |
|---|---|---|---|---|---|---|
| G ccw 1.0 | 5 (3) | 3 | 7 | 0 | 12 | -0.45 |
| G ccw 1.3 | 13 (4) | 4 | 26 | 0 | 18 | -0.45 |
| L ccw 0.6 | 0 (0) | 0 | 0 | 0 | 1 | 0.00 |
| F ccw 1.4 | 6 (2) | 2 | 9 | 0 | 11 | -0.45 |
| F ccw 1.5 | 15 (9) | 9 | 30 | 0 | 31 | -0.45 |
| G cw 1.0 | 1 (1) | 1 | 2 | 0 | 3 | -0.45 |
| G cw 1.3 | 3 (2) | 2 | 12 | 0 | 11 | -0.43 |
| L cw 0.6 | 1 (0) | 0 | 1 | 0 | 2 | -0.02 |
| F cw 1.4 | 2 (1) | 1 | 6 | 0 | 3 | -0.42 |
| F cw 1.5 | 9 (3) | 3 | 18 | 0 | 16 | -0.44 |

The escape counts equal the owner's tally in `DEVIATIONS.txt` for G ccw 1.0 and 1.3 (3 and 4),
F ccw 1.4 and 1.5 (2 and 9). The ccw blocks are worse than the cw blocks in every pair except L:
39 brake engagements (18 latched) in the ccw blocks against 16 (7 latched) in the cw blocks. No
contact anywhere (owner).

### 7.3 F ccw 1.5 turned around

Dead reckoned heading rises to +1657 deg at 46 s, then falls to +1420 deg by the end of the
block, a net 237 deg back. The block has four latched refusals in a row at 48.1, 51.3, 54.8 and
59.6 s (the owner counted three), each asking for right lock (-0.28, -0.33, -0.45 and -0.45
rad), that is, turning clockwise against the way the block started. After the last escape the car drove forward at 1.35 m/s for about 1 s with
steering near zero and the block ended. The bag therefore agrees that the car ended pointed the
other way round, and has only 2.6 s of clockwise turning to show for it: split the block at 46 s
for direction (ccw before, reversed after). Forward distance after 46 s is 3.7 m.

### 7.4 L 0.6, the rear near the wall

No safety event and no brake in L ccw (one advisory TTC), one brake in L cw (0.12 s). From the
scan: distance from each return to the body outline (rear axle origin, 0.14 m rear overhang,
0.44 m to the front bumper, 0.31 m wide, lidar at x 0.285 m facing backwards by pi, own body
returns removed), restricted to the rear half of the body:

- L ccw: closest return 0.066 m at 17.1 s, two episodes under 0.15 m (starting 15.9 s and 28.9 s).
- L cw: closest 0.064 m at 17.7 s, three episodes under 0.15 m (2.8 s, 17.4 s, 32.3 s).
- The front half reached 0.085 m (ccw) and 0.068 m (cw) at its closest.

The owner saw about 3 near misses of the rear in L ccw; the scan shows 2 episodes under 0.15 m
and a closest return of 6.6 cm. The lidar mount is "measured with a phone, about +/- 0.02 m"
(`vehicle_params.yaml`), so read these as 5 to 9 cm. The rear half includes the side of the
tail, so a wall beside the tail counts. No node watches the rear while the car drives forward.

### 7.5 Where gap follow is refused (for the separate fix task)

Latched refusals only (25 of 55; the other 30 last under 1.5 s and release without an escape).
Turn-in is the last time in the 3 s before the engage that the requested steering crossed
0.15 rad from below. Speeds are odometry. Heading is dead reckoned from the start pose of the
block, direction normalised so progress is positive.

- Turn-in speed p25 / p50 / p75 / max: 0.46 / 0.81 / 0.98 / 1.17 m/s (G blocks median 0.96, F
  blocks 0.72). The speed requested at turn-in has median 1.14 m/s, max 1.48.
- Turn-in to engage: p25 / p50 / p75 / max 0.57 / 1.08 / 1.68 / 2.71 s.
- At the engage: the node has already cut its request to 0.36 m/s (its creep value) with steering
  at or near lock (0.45 rad requested in 18 of 25, at least 0.43). Measured speed p25 / p50 / p75
  / max 0.34 / 0.39 / 0.71 / 1.35 m/s. TTC at the engage p50 0.53 s against a 0.60 s brake limit
  (min 0.002 s), so the in-path distance is about 0.2 m at that speed (rough: the gate judges
  speed by its own rule).
- So the pattern is: turn in at about 0.8 to 1.0 m/s, steering walks up to lock, the car is
  within 0.2 m of the wall in its swept path, the gate latches for 2.0 to 2.5 s, the escape
  reverses 0.29 m, and it tries again. 5 of the 25 happen within 3 s of the block's start from a
  standing start (G ccw 1.0 at 2.1 s, G ccw 1.3 at 1.3 s, F ccw 1.4 at 1.1 s, F ccw 1.5 at
  1.2 s, G cw 1.0 at 1.1 s), at dead reckoned heading 10 to 51 deg: the first corner.
- Where on the lap: by dead reckoned heading from the start pose, the 25 latched refusals fall
  in 60 deg bins as 8 (0 to 60), 2 (60 to 120), 8 (120 to 180), 4 (180 to 240), 0 (240 to
  300), 3 (300 to 360), that is, two groups near the start corner and the next one about 150 deg
  on. Absolute lap position needs scan matching to the saved map (`2026-11-13-pnp-lap1`), which
  was not done here, and heading drifts, so this is a hint, not coordinates.
- The race profile at cap 1.0 already refuses (G ccw 1.0: 3 latched in 32 s). Latched refusals
  per 10 s of window: ccw G 1.0 0.93, G 1.3 0.84, F 1.4 0.89, F 1.5 1.43, L 0; cw G 1.0 0.31,
  G 1.3 0.41, F 1.4 0.43, F 1.5 0.47, L 0.

## 8. Against the registered plan (v0.5 dynamics, section 5.1)

- Order: the plan runs clockwise first. The owner set the car ccw, so ccw ran first, then cw
  (`DEVIATIONS.txt`). Direction labels are right by heading except the tail of F ccw 1.5.
- G 1.0 ccw (30 s) is extra to the plan. G 1.3 ccw is planned. F 1.5 was driven in both
  directions after F 1.4 had no contact.
- Planned driving was 552 s; see section 4 for the 623 s and 578 s totals against the cap of 600.
- Two PnP launches aborted before block P (bad bundle path; then a `require_lap_counter`
  refusal about 3 s after the planner began driving, which races the counter start-up and
  lets the planner drive before the check). They are in section 5 as the "aborted launch".
- No wall contact in any block, today (owner). Section 7.4 shows two close rear approaches.

## 9. What this does not show

- Wall contact, "nobody touched the car", and the owner's lap count of 20 for 02-18-54: not
  observable from the bags.
- The receipt-log breakdown of 4.2 and the planner to delivery to safety_node split: no
  `/safety/cycle` in any bag.
- Why the stack lost its recorder on `02-29-27`, and why the car would not drive then.
- Absolute track positions of the gap follow refusals.
- Why P ccw and P cw had events and the two lap runs had none. The P blocks were launched over
  SSH with the cap schedule script running next to the planner, and 19 of their 23 events sit
  within 5.2 s of the first command; that is an observation, not a diagnosis.
