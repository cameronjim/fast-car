# Where the 65 to 75 ms stalls come from, 2026-11-22 bag (checklist P4; analysis only, nothing changed)

Read-only analysis for item P4 of `claude-docs/fix-checklist-2026-11-22.md`. No code, launch file,
container flag or parameter was changed, and nothing was run on the car. Every fix below needs a
brief, and any that touches `racer_safety`, `racer_drivers`, firmware or `vehicle_params` needs
owner approval (decision O4).

## Answer

**Most likely cause (high confidence): CPU contention.** `safety_node` and `mppi_planner_node` run
as ordinary `SCHED_OTHER` threads, and since the cameras came in (2026-11-13) the Orin is busy
enough that a 50 Hz thread that becomes runnable can wait tens of milliseconds for a core. The
stalls during driving hit one process at a time: whichever thread lost the race for a core. Nothing
in the bag points at the recorder, the disk, the camera record frames or a whole-system freeze.

**Second class (high confidence): node start-up.** The three largest safety cycles (238, 290 and
319 ms) happen while `pnp.launch.py` starts its three Python nodes and the `ros2 service call`
that resets the lap counter. One of them stopped a second process too, so these are system-wide.
The car is standing with the watchdog engaged, so they do no harm.

**The rate_limit at the planner's first command is not a stall.** It is a steering slew. The
watchdog holds the last steering of the previous run (0.42 to 0.43 rad). The planner's first
command steers 0.0 because it has not heard `/drive` yet. At 3.2 rad/s the slew takes 0.13 s,
and the logged engagements last 0.117 to 0.140 s.

What the bag cannot tell apart: a thread that is runnable but not scheduled, and a thread blocked
in the kernel (memory reclaim from zram, a page fault, a slow socket write). The planner's solve
points at scheduling: it does no I/O and takes no locks, its median is 2.1 ms, and it still took
41 ms of wall time. The bench procedure below separates the two.

## Method

The bag is `car-data/session-2026-11-22/2026-11-23T03-50-32_car_teleop_0.mcap`. It was read with
mcap's `StreamReader`, CRC off, up to the truncated tail, which is the same approach as
`tools/pnp_session/streamread.py`. 1,211,731 messages, 30.1 min of `/drive`. Times below are the
last four digits of the epoch second, as in the checklist (7042.09 is 1795407042.09).

Each process has a heartbeat with its own header stamps:

| process | heartbeat | nominal |
|---|---|---|
| `safety_node` (car-stack) | `/safety/cycle` stamp and `cycle_dt_s` | 20 ms |
| `mppi_planner_node` publish thread (car-stack, pnp launch) | `/drive_raw` header stamp | 20 ms |
| `mppi_planner_node` solve thread | `/pnp/planner/timing` `solve_start_s`, `solve_end_s` | 20 ms |
| VESC driver and `vesc_odometry_node` | `/vesc/sensors/core`, `/odom/wheel` header stamps | 20 ms |
| recorder | `log_time` of every message | continuous |

A stall that shows up in several heartbeats at once is system-wide. A stall that shows up in only
one is that process (or that thread) not running. Recorder writes come from the MCAP chunk records
(501 uncompressed chunks of about 790 kB, one every 1.6 to 5.9 s). The camera frames are the
`/camera/*/record/compressed` log times. The lap counter's processing is not recorded per frame
(only `/lap/status` at 1 Hz), so it can only be judged by its rate.

## Evidence

### 1. The counts

`/safety/cycle`, the whole bag: 90,250 cycles, `cycle_dt_s` p50 20.00 ms, p99 23.92 ms. 531 are over
25 ms, 101 over 30 ms, 16 over 45 ms and 9 over 60 ms. The stamp gaps agree with `cycle_dt_s` to
within 2.2 ms. Planner publish ticks (`/pnp/planner/timing`): 126 gaps over 30 ms and 11 over 45 ms
inside the runs.

The five planner runs are 6241.85 to 6312.51, 6430.21 to 6535.88, 6671.76 to 6752.32, 6883.47 to
6945.07, and 7035.32 to 7236.51 (D v0, 201 s).

By state, in 10 s windows, with "launch" meaning the 15 s before a run starts:

| state | minutes | cycles >25 ms /min | >30 ms /min | >45 ms /min | CPU+GPU rail W |
|---|---|---|---|---|---|
| idle (stack + cameras, no pnp) | 20.0 | 4.2 | 0.85 | 0.10 | 1.75 |
| launch (pnp nodes starting) | 1.5 | 71.3 | 21.3 | 6.7 | 2.33 |
| run (pnp driving) | 8.7 | 39.2 | 6.0 | 0.46 | 2.52 |

Outside launches, the correlation between the count of cycles over 25 ms per 10 s window and the
mean `VDD_CPU_GPU_CV` power is **0.81** (n = 172 windows). Windows under 1.9 W average 0.44 late
cycles each; windows at 2.2 W or more average 5.7 to 8.4.

### 2. During driving, one process stalls at a time

This table lists every safety cycle over 45 ms and every planner publish gap over 45 ms, with the
largest gap of each heartbeat inside that window (ms). The lidar is about 100 ms by design and the
rail topic about 200 ms.

| time | event | length | safety | planner | VESC odom | recorder |
|---|---|---|---|---|---|---|
| 6233.09 | safety dt (launch) | 290.0 | **290.0** | (not up) | **310.4** | 66.5 |
| 6425.01 | safety dt (launch) | 319.3 | **319.3** | (not up) | 24.3 | 23.2 |
| 7025.44 | safety dt (launch) | 237.9 | **237.9** | (not up) | 31.6 | 33.0 |
| 6258.43 | planner gap | 57.1 | 21.8 | **57.1** | 28.6 | 13.6 |
| 6893.88 | planner gap | 56.1 | 21.1 | **56.1** | 28.9 | 13.2 |
| **7042.09** | safety dt | 65.7 | **65.7** | 22.2 | 25.5 | 14.5 |
| **7093.44** | planner gap | 57.5 | 30.1 | **57.5** | 25.1 | 26.6 |
| 7100.20 | planner gap | 54.3 | 21.2 | **54.3** | 28.0 | 18.5 |
| **7102.99** | planner gap | 71.1 | 23.0 | **71.1** | 27.4 | 60.4 |
| 7121.85 | planner gap | 47.3 | 22.6 | **47.3** | 25.2 | 39.0 |
| 7225.65 | planner gap | 47.9 | 20.3 | **47.9** | 26.1 | 14.5 |

The other safety cycles over 45 ms are all within a few seconds of a launch, with the car standing:
5879.46, 5884.92, 6232.26, 6233.41, 6421.49, 6421.56, 6423.72, 6752.66 to 6752.98, 6874.64 and
6875.39.

**7042.09, the safety node's own stall.** The planner stamped `/drive_raw` at 7042.020, .038, .060,
.078 and .098, and the recorder received each within 1 to 13 ms. `safety_node` ran a cycle at
7042.022 and then nothing until 7042.088. In that cycle `drive_raw_count` had moved by one: the
messages stamped .038 and .060 had not yet been taken by its subscription. The next cycle, 21 ms
later, had taken three. Neither the timer nor the subscription callback ran for 65 ms, while every
other heartbeat kept its cadence. The process (one thread, `SingleThreadedExecutor`) was not
running. The watchdog fired (age 65.3 ms, limit 60 ms). The `rate_limit` 21 ms later, lasting
11 ms, is the speed ramp back from that one braked cycle.

**7093.44 and 7102.99, the planner's stalls.** The checklist's 74 and 75 ms are recorder receive
gaps. The header-stamp gaps are 57.5 and 71.1 ms. During both, `safety_node` kept to 20 to 30 ms.
At 7102.99 the planner then published two ticks 0.5 ms apart: rcl caught its timer up. At 7093.44
the message stamped .437 reached the recorder 17 ms after its stamp, so the publish thread stopped
between taking the stamp and the DDS write. The solve thread stalled separately: at 7093 there
were no solves between 7093.389 and 7093.488, and at 7102.99 the solve at 7102.956 ran while the
publish thread was stuck. The threads of one process stall separately, which fits threads waiting
for a core better than a process-wide lock.

**The solve gets preempted.** Over 23,529 solves the wall time was p50 2.1 ms, p90 5.6 ms, p99
12.7 ms and max 41.0 ms: 504 over 10 ms and 12 over 20 ms. The solve is pure arithmetic on buffers
it owns. When a 2 ms computation takes 41 ms of wall time, its thread was descheduled partway
through.

**Why 57 ms trips a 60 ms watchdog.** The watchdog compares the command's age with 60 ms. That age
is the planner's gap plus the phase between the two 50 Hz timers. In the D v0 run the phase sat at
the edge of a period: median age at the safety cycle 16.7 ms, p10 0.7 ms, p90 21.9 ms. In 9,956
cycles, 2,050 reused the previous command and 2,041 skipped one. In the four observe runs the
median age was 8 to 12 ms, with 27 to 122 reuses. The phase depends on when each node started, so
it is random per launch. A planner gap of about 45 ms trips the watchdog in a launch like D v0's,
and about 50 ms in a launch like the observe runs.

### 3. The recorder, camera frames and scans do not line up with the stalls

For each set of stall windows, the table counts the windows that contain the event. The baseline
is the same count with the event series shifted at random by up to ±30 s, 200 shifts, as mean ±
standard deviation.

| stalls | recorder chunk written | USB record frame | CSI record frame | scan stamp |
|---|---|---|---|---|
| safety dt >30 ms (101) | 3 (1.7 ± 1.3) | 11 (10.9 ± 4.2) | 10 (10.7 ± 3.9) | 47 (39.0 ± 5.6) |
| safety dt >45 ms (16) | 1 (0.5 ± 0.7) | 0 (2.5 ± 2.2) | 0 (2.4 ± 2.2) | 10 (10.6 ± 1.2) |
| planner gap >40 ms (13) | 0 (0.3 ± 0.5) | 1 (2.6 ± 1.3) | 3 (2.5 ± 1.4) | 10 (6.8 ± 2.5) |

None of these is distinguishable from chance. The planner gaps lean slightly towards scan arrivals
(1.3 standard deviations). On a scan, the planner's own scan thread and `track_perception_node`
start their work at the same moment, which is what a CPU burst would look like, but 13 events
cannot show it. The recorder writes about 0.2 MB/s with no compression. It is not a plausible
source of the stalls, and the 7042.09 recorder heartbeat shows it was not stalled itself.

### 4. Across sessions: the stalls arrived with the cameras

These are `/drive` header-stamp gaps (`safety_node`'s output, so every bag has them, including
those from before `/safety/cycle` existed). The rail column is the median `VDD_CPU_GPU_CV` power.

| bag | min | what ran | /drive gaps >30 / >45 / >60 ms | CPU+GPU W |
|---|---|---|---|---|
| 2026-10-24T23-13-18 | 13.4 | lidar, gap follow, no cameras | 0 / 0 / 0 | 0.61 |
| 2026-11-11T03-54-11 | 8.8 | lidar, VESC telemetry, no cameras | 1 / 1 / 1 | 0.65 |
| 2026-11-11T04-20-19 | 3.9 | the same | 0 / 0 / 0 | |
| 2026-11-13T22-00-40 | 19.5 | cameras, gap follow, EKF, AMCL, **no pnp** | 101 / 3 / 1 | 2.28 |
| 2026-11-13T22-20-18 | 58.2 | cameras, pnp, gap follow | 47 / 11 / 3 | 1.68 |
| 2026-11-14T22-07-50 | 24.7 | cameras, pnp, lap counter | 156 / 22 / 12 | 2.24 |
| 2026-11-16T22-45-39 | 18.3 | cameras, pnp, lap counter, gap follow | 57 / 14 / 4 | 1.92 |
| 2026-11-23T03-50-32 | 30.1 | cameras, pnp, lap counter | 101 / 16 / 9 | 1.92 |

Before the cameras, `safety_node` went 26 minutes with one gap over 30 ms. Every bag since has
tens of them, including one with no pnp at all. The stack grew in the same week (EKF, AMCL,
telemetry), so this does not prove the cameras alone. What it shows is that the CPU and GPU rail
power went up about threefold, and the late cycles went with it.

The cameras are a heavy load as configured. `cameras.launch.py` runs the USB camera at 1280x720 and
60 fps through `mjpeg2rgb`, which decodes every frame on the CPU, and then the compressed transport
re-encodes every frame to JPEG on the CPU. The CSI camera runs at 1280x720 and 60 fps through
`videoconvert` on the CPU, and is also JPEG-encoded on the CPU. That is about 120 JPEG encodes and
60 decodes per second at 720p. Downstream, only the lap counter (15 Hz) and the record streams
(5 to 10 Hz) use them. The full-rate USB stream also crosses from car-camera to car-stack over UDP
loopback (the UDP-only profile), roughly 60 frames per second. The doc string in
`camera_usb.launch.py` already says this load is "UNVERIFIED at 720p60".

### 5. Launch-time stalls

The three largest cycles, 290, 319 and 238 ms, sit between "process started" for
`track_perception_node`, `lap_memory_node` and `lap_counter_node`, and the lap counter's reset
(the launch logs `pnp-observe-205702.log`, `pnp-observe-210013.log` and `pnp-dv0-cw-211016.log`).
In that window Python interpreters import their modules, a `ros2 service call` process starts, and
every new DDS participant is discovered by every existing one. At 6233.09 the VESC odometry process
stalled at the same moment (310 ms, and it logged a VESC gap), so these are system-wide. Discovery
also explains a related observation. After the planner's first `/drive_raw`, `safety_node` took
0.17 to 0.71 s before it received any of them (`drive_raw_age_s` kept growing through 6430.0 to
6430.4, 6883.5 to 6884.2 and 7035.2 to 7035.4). In the D v0 launch, the recorder received the
first ten messages in one bunch, 196 to 16 ms late. The planner's `/drive` subscription matched
late as well. It first heard `/drive` 182, 777 and 748 ms after its first recorded tick in runs 2 to 4, and
at once in run 5.

### 6. The rate_limit at the first command

| launch | first `/drive_raw` steer | held `/drive` steer | rate_limit engaged |
|---|---|---|---|
| 6241.91 | 0.065 | 0.053 | none |
| 6429.99 | 0.000 | 0.421 | 0.123 s |
| 6672.28 | 0.000 | 0.430 | 0.117 s |
| 6883.47 | 0.000 | 0.432 | 0.140 s |
| 7035.20 | -0.430 | -0.430 | none |

The watchdog's brake keeps the previous steering (`zero_throttle_command`), so `/drive` still
carries the last steering of the previous run. In launches 2 to 4 the planner had not yet heard
`/drive` (section 5), so it commanded 0.0, and the limiter slewed 0.43 rad at
`steering.max_rate_rad_per_s` 3.2, which takes 0.134 s. In launch 5 it had heard `/drive` and
started at the held angle, and nothing engaged. The speed was 0 throughout. This is a start-up
ordering effect, not a timing fault.

## Bench procedure (L6, wheels up; written, not run)

**Goal:** confirm or reject CPU contention as the cause of late cycles, and measure how much each
load contributes. Two people, kill knob in hand, wheels off the ground, rail voltage logged
(invariant 5). A full battery, and the same battery for every configuration. Nothing in this
procedure changes a parameter of `safety_node`, the planner, the VESC or the mux. Step G changes
scheduling and runs only if the owner approves it.

**Run length and order.** Each configuration runs for 3 minutes of planner output (9,000 safety
cycles). In a run like the D v0 run, that is about 18 cycles over 30 ms and about 1.4 over 45 ms.
The >45 ms count is too sparse to judge on in 3 minutes, so the configurations are compared on
the >25 ms and >30 ms counts and on the safety thread's run-queue wait. Run the order
A, B, C, D, E, F, A, which repeats A at the end to catch battery or thermal drift. Repeat the
whole order if the two A runs differ by more than a factor of 1.5.

**Planner arguments for every configuration:** `backend:=geometric
dynamics_bundle:=/workspace/data/pnp/models/dyn-v0.1-2026-11-12 v_max_mps:=0.5`, plus the
arguments listed in the table below.

| cfg | stack | cameras | pnp extras | recorder |
|---|---|---|---|---|
| A, full stack (as on 11-22) | `car_teleop.launch.py lidar:=true vesc:=true` | `cameras.launch.py lap_record_rate_hz:=10.0` | `lap_counter:=true require_lap_counter:=true lap_memory:=true lap_source:=camera lap_mode:=observe` | stack recorder (default) |
| B, light recorder | as A with `record:=false` | as A | as A | a separate `ros2 bag record` of `/safety/cycle /safety/events /drive /drive_raw /pnp/planner/timing /pnp/planner/status /telemetry/rail_voltage_v /telemetry/rail_current_a` and the `/telemetry/rail/.*` topics, so the run is still logged and pnp's recorder check passes |
| C, no cameras | as A | not running (Ctrl-C the cameras launch; car-camera idle) | `lap_counter:=false lap_memory:=false` | as A |
| D, no lap counter | as A | as A | `lap_counter:=false lap_memory:=false` | as A |
| E, no cameras + synthetic load | as C | as C | as C | as A, plus 4 busy loops on the host (below), to bring the CPU back up without cameras |
| F, idle reference | as A | as A | pnp not running | as A, 3 min |

Synthetic load for E. This installs nothing and stops on its own after 180 s:

```sh
for i in 1 2 3 4; do timeout 180 sh -c 'while :; do :; done' & done
```

**Host measurements, every configuration.** Run these on the Jetson host as read-only commands
and save the output next to the bag. Before the run:

```sh
nvpmodel -q; cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
cat /sys/devices/system/cpu/online; uptime; free -m; zramctl 2>/dev/null; cat /proc/swaps
SP=$(pgrep -f 'racer_safety/lib/racer_safety/safety_node' | head -1)
PP=$(pgrep -f mppi_planner_node | head -1)
for p in $SP $PP; do chrt -p $p; ps -L -o tid,cls,rtprio,ni,psr,comm -p $p; done
for c in car-stack car-camera; do id=$(docker inspect -f '{{.Id}}' $c); \
  cat /sys/fs/cgroup/system.slice/docker-$id.scope/cpu.weight \
      /sys/fs/cgroup/system.slice/docker-$id.scope/cpu.max 2>/dev/null; done
```

Snapshots taken before and after the 3 minutes:

```sh
cat /proc/pressure/cpu /proc/pressure/memory /proc/pressure/io
for p in $SP $PP; do for t in /proc/$p/task/*; do echo "$t $(cat $t/comm) $(cat $t/schedstat)"; done; done
ps -o pid,maj_flt,min_flt -p $SP,$PP; grep ctxt /proc/$SP/status /proc/$PP/status
for c in car-stack car-camera; do id=$(docker inspect -f '{{.Id}}' $c); \
  cat /sys/fs/cgroup/system.slice/docker-$id.scope/cpu.stat; done
```

Running alongside, for the 3 minutes:

```sh
vmstat 1 180 > vmstat.txt &
tegrastats --interval 1000 > tegrastats.txt &   # stop it after 180 s
mpstat -P ALL 1 180 > mpstat.txt 2>/dev/null &   # if sysstat is present
top -H -b -d 10 -n 18 > top_threads.txt &
```

The second field of each `/proc/<pid>/task/<tid>/schedstat` line is the nanoseconds that thread
spent runnable but waiting for a CPU. Its change over the run, for the `safety_node` thread and
the planner's three executor threads, is the most direct measure of the cause.

**Optional, for one A run (needs root and `trace-cmd` already on the host, installed only if the
owner agrees):** run `trace-cmd record -e sched:sched_wakeup -e sched:sched_switch -o a.dat sleep 60`
during driving. For each `/safety/cycle` over 30 ms in that minute, `trace-cmd report` shows when
the safety thread was woken, when it got a CPU, and which threads held every CPU in between.
This is proof per event, not a statistic.

**Counting, from each bag.** Use the lenient reader, since a bag may be unfinalised:

```python
from pathlib import Path
from pnp_session.streamread import iter_stream  # from tools/: PYTHONPATH=. uv run --project . python count.py
dts, solves = [], {}
for _, topic, m in iter_stream(Path("<bag>.mcap"), ["/safety/cycle", "/pnp/planner/timing"]):
    if topic == "/safety/cycle":
        dts.append(m.cycle_dt_s)
    elif m.solve_start_s > 0:
        solves[m.solve_start_s] = m.solve_end_s - m.solve_start_s
for lim in (0.025, 0.030, 0.045, 0.060):
    print(f">{lim*1e3:.0f} ms: {sum(d > lim for d in dts)} of {len(dts)}")
print("solve wall >10 ms:", sum(s > 0.010 for s in solves.values()), "of", len(solves))
```

Count the `/drive_raw` header-stamp gaps over 30 and 45 ms the same way, and the `watchdog` and
`rate_limit` engagements (phase ENGAGE) from `/safety/events`.

**Reading the results (rules fixed before the run):**

1. **Contention confirmed** if all of the following hold. The safety thread's run-queue wait per
   minute and its count of cycles over 25 ms are highest in A and lowest in C and F. C cuts A's
   count of cycles over 30 ms by at least 70 percent. E brings the count back to within a factor
   of 2 of A. The solve wall-time p99 follows the same order.
2. **Something about the cameras other than CPU** (USB interrupts, softirq from UDP loopback) if C
   is clean but E is not late at a similar `tegrastats` CPU load. Then compare `%irq` and `%soft`
   per CPU in `mpstat` between A and E.
3. **Memory** if `maj_flt` of `safety_node` or the planner rises during a run, swap use grows, or
   `/proc/pressure/memory` "some" grows by more than about 1 percent of the run time. That would
   point at reclaim and zram rather than scheduling.
4. **Quota throttling** if `cpu.stat` shows `nr_throttled` growing for car-stack. That would be a
   container limit, not contention.
5. B against A gives the recorder's share. D against A gives the lap counter's share.

**G, only with owner approval (it changes scheduling, decision O4).** This is one more A run with
the two control processes raised at run time, with no code or launch change:
`chrt -f -p 50 $SP` and `chrt -f -a -p 40 $PP`. This needs root on the host. `chrt -p` must show
`SCHED_FIFO` afterwards. `safety_node` does two scan reductions and the gate per cycle, and the
planner's solve has a median of 2.1 ms per 20 ms tick, so neither should hold a core for long.
The kernel's RT throttling (`/proc/sys/kernel/sched_rt_runtime_us`, read it first) keeps 5 percent for everything
else. If the safety cycles over 30 ms and the planner gaps over 45 ms almost disappear while A's
load is unchanged, the cause is confirmed and fix 3 below works. If the DDS listener threads
(also `SCHED_OTHER`) are what makes commands arrive late, `drive_raw_age_s` will still spike while
`cycle_dt_s` stays clean. That would show that fix 3 needs fix 1 or 2 alongside it.

Roll back G by restarting the stack, or with `chrt -o -p 0` on each pid. Nothing persists.

## Candidate fixes, ranked (none implemented)

1. **Cut the camera load at its source.** Run USB and CSI at 30 fps or less (or at a lower
   resolution), and turn the CSI camera off for D runs (`csi:=false`). The lap counter does not use
   it, and the D record streams need 5 to 10 Hz. These are existing launch arguments (`usb_fps`,
   `csi_fps`, `csi`): no code, no change to the command path, and easy to reverse. The bags before
   the cameras had essentially no late cycles at a third of today's CPU power. The lap counter at
   30 Hz (fix S1) would need `usb_fps` of at least 30. This is the owner's session choice, and C,
   D and E above measure it first.
2. **Keep the cameras off the control cores.** Give car-camera a CPU subset, for example
   `--cpuset-cpus=0-3` on its `docker run`, and optionally a lower `--cpu-shares`. This is a run
   flag on the camera container only, with no `racer_safety` change. The Python nodes in car-stack
   can still compete, so it is weaker than fix 3. Read `/sys/devices/system/cpu/online` first: the
   number of online cores depends on the nvpmodel mode.
3. **Real-time priority for the two 50 Hz loops.** Run `safety_node`'s executor thread and the
   planner's publish thread as `SCHED_FIFO` (through a launch `prefix` with `chrt`, or by setting
   the thread policy in code). The containers would need `--cap-add=SYS_NICE` and
   `--ulimit rtprio=<n>`. This is the strongest guard against CPU contention, and the per-cycle
   cost is small, but it touches `racer_safety`'s launch and the container
   flags. It needs a brief and O4. The DDS listener threads that deliver `/drive_raw` remain
   `SCHED_OTHER`, so it should come with fix 1 or 2, and step G checks whether they matter.
4. **Phase and watchdog margin.** Today a planner gap of about 45 to 50 ms trips the 60 ms
   watchdog, because the two timers' phase is random per launch and was at the edge in D v0.
   Options: the planner sets its publish phase from `/drive` receipts, or the safety cycle is
   triggered from `/drive_raw` arrival, or the watchdog moves to 4 cycles. These hide the stalls
   rather than remove them, and the second and third change `racer_safety` timing (brief and O4).
   They should come after fixes 1 to 3.
5. **First-command steering slew.** The planner could stay in STARTING until it has heard `/drive`,
   and then start from the gated steering. This is a `racer_planner` change, and it is harmless
   today because the speed is 0. A smaller related change: the lap counter reset could be called
   from inside the launch instead of by starting a separate `ros2 service call` process, which
   takes one Python start out of the launch window. Launch-time stalls happen with the car
   standing and the watchdog engaged, so they come last.
