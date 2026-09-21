# Car runtime plan: eventual container lifecycle (design only, follow-up)

Status: **design doc, not implemented.** Two things in it are no longer speculative as of
2026-09-21, and are marked inline below: the container's device access (RESOLVED -- stock
`gpio` group, no `--privileged`, no custom udev rule) and the absence of rosbag recording
(CONFIRMED on the device). Everything else here is still design. `docs/notes/first-boot-runbook.md` has the operator
start the `car` image by hand, and that is correct for first boot -- a human should be present
for every command-path change until roadmap 1.3's kill test passes and G1 is signed off
(`claude-docs/01-roadmap.md`). This document specifies the shape the container lifecycle
should take once the car is driven routinely, so that work is not designed from scratch under
time pressure later. No systemd unit or compose file is added by this doc; that is explicitly
future work, tracked below.

## Why this is a follow-up, not now

- First boot needs a human watching every step (`docs/notes/first-boot-runbook.md`'s standing
  rules: two people, wheels off the ground, kill switch armed). An auto-starting container
  that brings up the command path on power-on works against that, not with it, until the mux
  is actually kill-tested.
- The right launch file to run is still a per-session choice (teleop vs. autopilot vs.
  eventually the residual policy -- see `docker/car/Dockerfile`'s "No CMD" comment), and PWM
  chip/channel numbers are launch arguments discovered per `first-boot-runbook.md` step 4, not
  yet stable constants a unit file could hard-code.
- Designing the eventual shape now, while the manual runbook is still fresh, is cheap and
  avoids re-deriving these decisions later.

## The eventual shape

### systemd unit (preferred over compose)

A single `car-stack.service` on the Jetson host, not a `docker compose` file: this repo has
exactly one container to manage on the car (the `car` image), no multi-container
orchestration need, and `racer-heartbeat.service` (below) already establishes the "one
systemd unit per host-level responsibility" pattern this project uses. Compose would add a
dependency and a second lifecycle model for no benefit at this scale.

Sketch (values in brackets are still open questions, listed below):

```ini
[Unit]
Description=Racer car ROS stack (car image + launch file)
After=docker.service network-online.target
Wants=network-online.target
# Deliberately NOT ordered relative to racer-heartbeat.service (see "Relationship to the
# heartbeat service" below) -- they are independent by design.

[Service]
Type=simple
ExecStartPre=-/usr/bin/docker rm -f car-stack
ExecStart=/usr/bin/docker run --rm --name car-stack \
  --network host \
  --user 1000:1000 \
  --group-add 999 \
  -e HOME=/tmp \
  -v /sys/devices/platform/bus@0/3280000.pwm:/sys/devices/platform/bus@0/3280000.pwm \
  -v /sys/devices/platform/bus@0/32c0000.pwm:/sys/devices/platform/bus@0/32c0000.pwm \
  -v /opt/racer/car:/workspace -w /workspace/ros_ws \
  -v <rosbag output dir>:/bags \
  car:local bash -lc '
    source /opt/ros/humble/setup.bash && source install/setup.bash
    exec ros2 launch racer_bringup <car_teleop.launch.py|car_autopilot.launch.py> \
      bag_dir:=/bags <args>'
Restart=on-failure
RestartSec=2
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

**No `--runtime nvidia`.** Dropped from the sketch 2026-09-21: nothing in the control stack
touches CUDA, and the 2026-09-21 bring-up ran the whole command path without it. Add it back
when `racer_policy` inference actually runs on the car (roadmap 5.x), which is also when the
image needs torch.

**Restart policy: `on-failure`, not `always`.** A crash mid-drive should not silently
respawn the stack with the servo/ESC still powered and the operator possibly not watching --
`on-failure` restarts a clean exit-1 (e.g. a real bug), but a `SIGKILL`/OOM/hard fault getting
auto-relaunched unattended is exactly the kind of "software driving the car with nobody
present" scenario `CLAUDE.md`'s safety framing exists to prevent. This mirrors
`racer-heartbeat.service`'s own `Restart=always` choice being fine for a passive GPIO toggle
but wrong here, where "restart" means "start commanding actuators again."

**Log location.** `journalctl -u car-stack` for the container's own stdout/stderr (same
pattern as `racer-heartbeat.service`), separate from rosbags -- see below. Container logs
should stay small (node startup/shutdown lines, exceptions); anything voluminous belongs in
the bag, not the journal.

**Open questions this sketch does not resolve (need an owner decision before implementation):**

1. Does this unit start the stack armed-and-ready, or does an operator still SSH in and run
   `ros2 launch` inside it by hand for now, with the unit only guaranteeing the container
   image and mount points are correct? (Leaning toward the latter for longer than feels
   satisfying, given the G1 gate above.)
2. Which launch file is the default once there is a default -- teleop, autopilot, or a
   launch-argument the unit reads from a small host-side config file
   (`/etc/racer/car-stack.env`, sourced by `EnvironmentFile=`)? Per-machine config via launch
   arguments is the existing convention (`claude-docs/10-conventions.md`), so the latter is
   the likely answer, but it is not designed here.
3. The PWM chip/channel numbers are now known on the one device measured so far (pin 15 =
   `pwmchip0` = steering, pin 33 = `pwmchip2` = throttle, confirmed 2026-09-20; see
   `docs/notes/build-log.md`), but they are still a per-device fact, not a project constant --
   confirm again before hard-coding them into a unit file on different hardware. Re-derive the
   two bind-mount paths the same way, with `readlink -f /sys/class/pwm/pwmchipN`.

   **RESOLVED 2026-09-21 (device access).** The sketch above used to say `--group-add
   <racer-pwm GID>`, pending a custom group and udev rule that `first-boot-runbook.md` step 8
   proposed and marked UNVERIFIED. Neither is needed and neither exists: the Jetson already
   ships `/lib/udev/rules.d/60-jetson-gpio-common.rules`, which grants the stock **`gpio`**
   group (GID 999 on this device) write access to `pwmchipN/{export,unexport}` and to each
   exported channel's `period`/`duty_cycle`/`enable`, and `racer` is already a member. The
   `racer-pwm` group and rule were created during the 2026-09-21 bring-up, found redundant,
   and removed; the host is back to stock udev. VERIFIED on the device: both channels driven
   for real from an unprivileged, non-root (`--user 1000:1000`) container with nothing but
   `--group-add 999` and the two read-write bind-mounts above. Note the GID is a per-device
   fact like the chip numbers -- a unit file should read it from `getent group gpio` rather
   than hard-code 999.

### How rosbag recording starts with the stack (CLAUDE.md invariant 5)

**Resolved 2026-09-21 (GitHub issue #64, roadmap 1.6). This is no longer a design question,
and the sketch above no longer starts a second `ros2 bag record` process of its own.**

`car_teleop.launch.py` starts the recorder and `rail_voltage_node` itself, on by default:

- **One lifecycle, not two.** The recorder is a launch-managed process, so it starts with the
  stack, stops with the stack, and cannot outlive it or be outlived by it. The earlier sketch
  -- two background processes in one `ExecStart` under one `Restart=on-failure` -- had a race
  the launch file simply does not have.
- **Rail voltage is a normal topic.** `racer_drivers/rail_voltage_node` publishes the Jetson
  INA3221's rails at 5 Hz as `std_msgs/Float32` in volts and amps under `/telemetry/...`, so
  one recorder captures command path and rail voltage together, which is what the invariant's
  "rosbag + rail voltage" phrasing asks for. The INA226 in `claude-docs/11-hardware.md` is
  still unfitted; when it lands it publishes onto the same topics and nothing else changes.
- **A run without a bag IS a startup failure now.** If the recorder exits, the launch logs at
  error level and shuts itself down (about a second, measured), and `pwm_output_node` takes
  its ordinary fail-closed shutdown path on the way out. The mechanism the old text left "to
  whoever implements this" is a launch `OnProcessExit` handler emitting `Shutdown`, not a
  wrapper script or a supervisor. Critically it is NOT a gate on `/drive`: putting a logging
  dependency inside safety layer 3 would weaken the layer it sits in to strengthen something
  that is not a safety layer at all (`claude-docs/05-safety.md`).

What the unit above still owns is only WHERE the bags go: `-v <rosbag output dir>:/bags` plus
`bag_dir:=/bags`. Interactively, `bag_dir` defaults to `/workspace/data/bags`, which is the
repo's gitignored `data/` through the existing workspace mount -- see
`docs/notes/first-boot-runbook.md`'s "Every run is recorded".

Storage format is chosen at launch: mcap when `ros-humble-rosbag2-storage-mcap` is installed
(it is, in `docker/car`), sqlite3 otherwise, and the choice is logged rather than assumed.

### Relationship to the heartbeat service (host-level, independent, by design)

`racer-heartbeat.service` (`tools/jetson_heartbeat/systemd/racer-heartbeat.service`) already
exists, is running on the bench Jetson, and is explicitly designed with
`DefaultDependencies=no` / `After=sysinit.target` specifically so it starts before, and does
not depend on, anything ROS/Docker/network related (see that unit's own comments and
`tools/jetson_heartbeat/README.md`'s "Why not vehicle_params" section for the full reasoning).
`car-stack.service` above deliberately does **not** order itself relative to
`racer-heartbeat.service` in either direction, and never will:

- The heartbeat's entire safety value is that the layer-1 mux sees it independent of whether
  the ROS stack, Docker, or even the network is up. If `car-stack.service` depended on it, or
  it depended on `car-stack.service`, a container-lifecycle bug could take down the one
  signal the mux uses to decide the Jetson is alive -- exactly backwards.
- Conversely, `car-stack.service` starting or stopping should never touch
  `racer-heartbeat.service`'s unit state. An operator restarting the ROS stack mid-session
  (e.g. to pick up a new launch argument) must not cause even a momentary heartbeat gap.

The two units share a host and nothing else: no `Requires=`, no `After=`, no `PartOf=`. This
is the intended design, not an oversight to fix later.

## What is NOT part of this plan

- No compose file, per the "systemd unit preferred" reasoning above.
- No auto-start on boot yet (`WantedBy=multi-user.target` in the sketch is aspirational, for
  the day this is enabled) -- enabling it is a separate, explicit decision after G1.
- No implementation in this PR. This document is the design; the unit file, the wrapper
  script, and the enable/disable decision are follow-up work, filed as a roadmap item once a
  phase actually needs it (candidate: end of stage 7 / G1 in `planning-docs/07-first-drives-
  teleop-and-logging.md`, once teleop-with-logging is routine enough that manual `docker run`
  every session is the bottleneck, not the safety margin).
