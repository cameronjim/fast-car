# tools/jetson_heartbeat -- Jetson-side heartbeat for the layer-1 safety mux

Roadmap task 1.3 (`claude-docs/01-roadmap.md`), layer-1 safety mux (`claude-docs/05-safety.md`,
`firmware/safety_mux/`). This is the Jetson half of the heartbeat the mux's watchdog reads --
see `firmware/safety_mux/pico/heartbeat_input.h`'s comment for what the signal IS and why it
is a raw GPIO toggle and not a message of any kind: a lightweight process toggling one GPIO
line at a fixed rate, independent of ROS and of the network stack, whose only job is to prove
this Jetson's low-level I/O is alive and being serviced. The mux cuts drive if edges stop for
longer than `config/vehicle_params.yaml`'s `limits.mux_watchdog_timeout_s` (currently 0.1 s,
PROVISIONAL).

## Status

Built, installed, and running as a systemd service on the bench Jetson (racer-car,
10.0.0.226) as of 2026-09-12. The GPIO line genuinely toggles (verified by two independent
software-only methods below). It has **not** been connected to the mux board -- that plug
does not exist yet per `firmware/safety_mux/README.md`'s own status -- and the toggle rate has
**not** been confirmed with a loopback jumper, oscilloscope, or multimeter. See "Verification"
and "What remains unproven" below before treating this as a bench-tested part of the safety
chain.

## GPIO mapping

**Jetson 40-pin header physical pin 7 == `gpiochip0` line 144 (kernel name `PAC.06`, global
gpio number 492 on this board's numbering).** Pin 9 (adjacent, standard header ground) is the
shared return.

This was **not** assumed from a generic Jetson pinout diagram. It was read off this exact
board's own live pinmux device tree, because Orin's header-pin-to-pad mapping is
board/carrier-specific and NVIDIA ships the authoritative table as data, not as a fixed
constant:

1. `sudo python3 /opt/nvidia/jetson-io/config-by-pin.py -l` confirms this board's header 1 is
   "Jetson 40pin Header" (the physical connector we're using) and that pin 7 currently has no
   special-function overlay applied (`config-by-pin.py -p 7` -> `unused`), i.e. it is in its
   default GPIO mode, not routed to an alternate peripheral (e.g. `aud`).
2. `jetson-io`'s own `Jetson.board.Board` + `Jetson.header.Header` classes parse the running
   kernel's pinmux device tree (`/opt/nvidia/jetson-io/Jetson/header.py`,
   `_header_parse_pinmap`) to build the pin-number -> pinmux-node table NVIDIA's own tool uses
   to label pins. Calling `header.pins.get_name(7)` directly returns `soc_gpio59_pac6` -- the
   SoC's own pinmux register name for pin 7, sourced from the live DT, not a documentation
   PDF.
3. `soc_gpio59_pac6`'s pad-name suffix, `pac6`, is Tegra234's own `PAC.06` naming (port AC,
   bit 6) -- exactly the line name `gpioinfo gpiochip0` prints for line 144:
   `line 144: "PAC.06" unused input active-high`. The two independent naming schemes (jetson-io's
   pinmux-node name, libgpiod's line name) agree without any manual lookup table in between.
4. `/sys/kernel/debug/gpio` confirms `gpiochip0` numbers its lines starting at global gpio
   348 (`gpiochip0: GPIOs 348-511 ... tegra234-gpio`), so line 144 is global gpio 492 -- this
   is the number that shows up as `gpio-492` in that same debugfs file, used below to watch the
   line toggle independent of the character-device API racer-heartbeat itself uses.

Reproduce steps 1-3 on the bench Jetson:

```sh
sudo python3 /opt/nvidia/jetson-io/config-by-pin.py -l
sudo python3 /opt/nvidia/jetson-io/config-by-pin.py -p 7        # -> unused (default GPIO mode)
gpioinfo gpiochip0 | grep 'PAC.06'                                # -> line 144
sudo python3 - <<'EOF'
import sys; sys.path.insert(0, '/opt/nvidia/jetson-io')
from Jetson import board
b = board.Board(); h = b.get_board_headers()[0]; b.set_active_header(h)
print(b.header.pins.get_name(7))   # -> soc_gpio59_pac6
EOF
```

Ground (pin 9) was confirmed the same way: `b.header.pin_get_label(9)` -> `GND`.

## Requirements

- Fixed-rate square-wave toggle. Default 50 Hz (an edge every 10 ms), configurable with
  `--rate-hz`. 50 Hz gives roughly 10 edges per the mux's current 0.1 s watchdog window; if
  that timeout is retuned, re-check the margin (see "Why not vehicle_params" below) rather
  than assuming 50 Hz is still comfortably fast enough.
- No dependency on ROS, the network stack, or the desktop session: a single static-linked-
  against-only-libgpiod binary, launched by systemd directly (`DefaultDependencies=no`,
  `After=sysinit.target`), not by anything in `ros_ws/`.
- **It must NOT refuse to start when ROS is not running, and must not stop when ROS stops.**
  This is a requirement, not an omission. The heartbeat's claim is "this Jetson's kernel is
  alive and servicing low-level I/O", nothing more. `claude-docs/05-safety.md` has layer 1
  catching a "Jetson freeze, Linux hang, software crash, ROS deadlock" -- four different
  failures that this one signal has to distinguish itself from, which it cannot do if its own
  liveness is tied to any of them. The corollary matters as much: a running heartbeat is NOT
  evidence that ROS, the control stack, or anything above the kernel is healthy. Nothing may
  be built on top of it that assumes otherwise; `/drive_raw` staleness is layer 3's job
  (`racer_safety`), not this signal's.
- No heap allocation and no drift accumulation in the toggle loop: the loop tracks an
  absolute deadline (`clock_gettime` once at start, then `clock_nanosleep(CLOCK_MONOTONIC,
  TIMER_ABSTIME, ...)` advancing that same deadline by one half-period each iteration) instead
  of sleeping a relative amount each time, which would accumulate scheduling latency as drift.
- Any error (bad argument, can't open the chip, can't claim the line, `clock_nanosleep`
  failure) is a nonzero exit with a message on stderr -- never a silent stop. Verified in
  "Verification" below.
- The GPIO line and the rate are both configurable (`--chip`, `--line`, `--rate-hz`), with the
  pin mapping above as the default.

### Why not `vehicle_params`

`limits.mux_watchdog_timeout_s` (`config/vehicle_params.yaml`, currently `0.1` PROVISIONAL) is
deliberately **not** read by this program at runtime, even though CLAUDE.md invariant 2 asks
physical constants to come from the generated `vehicle_params` binding. Two reasons:

1. This process exists specifically to not depend on anything else on the Jetson (per
   `firmware/safety_mux/pico/heartbeat_input.h`'s comment). Loading the generated Python
   binding means depending on `tools/gen_params.py`'s toolchain (PyYAML, jsonschema) being
   present and the YAML file being parseable -- exactly the kind of dependency this program is
   supposed to not need, on a process whose failure mode (silently stopping) is invisible from
   the Jetson side.
2. The mux MCU itself is the thing that actually compares heartbeat age against the timeout
   (`firmware/safety_mux/logic/src/watchdog.c`, via its own generated C binding). This
   program's only obligation is to toggle comfortably faster than whatever that timeout is;
   it does not need the exact value to do its job, only a rate an operator has chosen with
   margin in mind (see "Requirements" above).

If a future task wants this program to refuse to start unless its configured rate is provably
faster than the current `vehicle_params` timeout, that is a reasonable enhancement -- flagged
here rather than added silently, since it reintroduces the dependency above.

## Verification

Performed on the bench Jetson (racer-car, 10.0.0.226, JetPack 6.2 / L4T R36.4.4) on
2026-09-12. Full detail and dated narrative: `docs/notes/build-log.md` (2026-09-12 entry).

**Proven, by software only:**

1. **The pin mapping** (see "GPIO mapping" above) -- read from this board's own live pinmux
   device tree via NVIDIA's own `jetson-io` tooling, cross-checked against `gpioinfo`'s
   independently-named line list, not assumed.
2. **The binary correctly requests and drives the line as output.** `gpioinfo gpiochip0`
   shows line 144 going from `unused input` to `"racer-heartbeat" output [used]` while the
   service runs, and back to `unused` after it stops.
3. **The kernel's own GPIO register reflects the commanded value**, read through a code path
   independent of the character-device API the program uses:
   `sudo cat /sys/kernel/debug/gpio | grep gpio-492` shows `out hi` / `out lo` matching
   `gpioset`'s commanded value during a manual test, and shows the `racer-heartbeat` consumer
   label and a changing level while the systemd service runs.
4. **The toggle rate**, measured by a small standalone C program
   (not committed -- a throwaway diagnostic, see build-log) that repeatedly re-opens and
   re-reads `/sys/kernel/debug/gpio` in a tight loop for 2 seconds, counting value
   transitions on `gpio-492` with `clock_gettime(CLOCK_MONOTONIC)` timestamps: **200
   transitions in 1.9901 s, implied average edge interval 0.0100 s** -- i.e. 100 edges/second,
   matching the expected 50 Hz square wave (edge every 10 ms) exactly. This samples the same
   hardware register the pad output comes from, independent of `racer-heartbeat`'s own gpiod
   calls, so it is not merely checking that the program believes it's toggling at the right
   rate.
5. **Error handling.** `--line 99999` (an out-of-range line) exits 1 with
   `cannot get line 99999 on 'gpiochip0': Invalid argument` on stderr. Starting a second
   instance while one already holds the line exits 1 with `cannot request line 144 on
   'gpiochip0' as output (already claimed by another process?): Device or resource busy`.
   `SIGTERM` (what `systemctl stop` sends) produces a clean `stopping on signal, releasing
   line 144` and exit 0; `SIGKILL` (simulating a crash) is caught by systemd's `Restart=always`
   and the service comes back with a new PID within about a second. Note that about a second
   is ten times the mux's 0.1 s watchdog window: a crash of this process IS a cut, by design,
   and `RestartSec` is not tuned to hide that.
6. **The systemd service** is enabled, starts at boot (`multi-user.target`), and restarts on
   failure -- confirmed live with `systemctl status racer-heartbeat` (active, running) and by
   `systemctl kill -s SIGKILL` followed by `systemctl status` showing a fresh PID.

**Reviewed 2026-09-14** (`docs/notes/firmware-review-2026-09-14.md`, question 4), which found
and fixed: systemd's default start rate limit would have stopped restarting this unit after
about five seconds of any persistent failure (a busy line, a renamed gpiochip), so
`StartLimitIntervalSec=0` is now set; `DefaultDependencies=no` had dropped the shutdown
ordering, so `Conflicts=`/`Before=shutdown.target` are now explicit; `--line` accepted signed
input through `strtoul` (`-18446744073709551615` parsed as line 1); and `--rate-hz` had no
upper bound, so a typo could turn the toggle loop into a spin. The verification above still
stands: none of those touched the toggle loop or the pin mapping.

**NOT proven, and not claimed:**

- That the electrical signal genuinely reaches the physical header pin 7 pad on this specific
  board's connector (as opposed to the SoC's internal GPIO register, which is what all of the
  above actually observes). The pinmux mapping is sourced from NVIDIA's own live device tree
  for this exact carrier, which is about as authoritative as software can get, but no
  multimeter or oscilloscope probe has touched pin 7 itself.
- The actual voltage levels (assumed 3.3 V logic, per `firmware/safety_mux/README.md`'s
  "the heartbeat is already 3.3 V" note) and rise/fall times -- not measured.
- Real-time jitter under CPU load, thermal throttling, or during a `car` image /ROS boot --
  only measured with the Jetson otherwise idle.
- That the mux MCU actually sees and interprets this signal -- the mux board and its JETSON
  connector do not exist yet (`firmware/safety_mux/README.md`'s own status), so this cannot be
  tested until they do.
- That this keeps working across a JetPack update. `gpiochip0` is resolved by NAME and line
  144 by offset; Orin's chip enumeration order between `tegra234-gpio` and
  `tegra234-gpio-aon` is not a contract, so an update that renumbers them makes this exit 1
  on every start. The mux fails safe (no edges is a cut) and systemd now retries forever, but
  the robust fix is to resolve the line by its own name (`PAC.06`) via `gpiod_line_find()`
  and fall back to chip+offset. Flagged, not done -- see that review note's follow-ups.

**What a loopback jumper would additionally prove, and how:** connect a single jumper wire
from **physical pin 7** (this heartbeat's signal) to **physical pin 29** (`gpiochip0` line
105, kernel name `PQ.05`, confirmed free/unused the same way as pin 7 above), then run
`sudo gpiomon --num-events=200 gpiochip0 105` while the service runs and check the wall-clock
span the 200 edges took (expect close to 2.0 s at 50 Hz). This is an interrupt-timed count on
a *different* GPIO controller line electrically driven by pin 7's actual pad, which is a
strictly stronger proof than the debugfs polling above (that polling shares no interrupt path
with the pad, but also doesn't touch a second, physically wired pin). Nothing else --
oscilloscope, multimeter -- is needed to close this gap; a jumper and `gpiomon` is enough. This
was not done in this pass because it requires a human to place the physical jumper.

## Installing on the Jetson

```sh
# On the Jetson (or scp the tree over and run these there):
sudo apt-get install -y libgpiod-dev
cd tools/jetson_heartbeat
make all                              # builds ./racer-heartbeat
sudo mkdir -p /opt/racer/jetson_heartbeat
sudo cp racer-heartbeat /opt/racer/jetson_heartbeat/racer-heartbeat
sudo cp systemd/racer-heartbeat.service /etc/systemd/system/racer-heartbeat.service
sudo systemctl daemon-reload
sudo systemctl enable --now racer-heartbeat.service
systemctl status racer-heartbeat.service     # should show "active (running)"
journalctl -u racer-heartbeat -f             # live log
```

`make test` builds and runs the host-runnable unit tests (`tests/`) with plain `gcc`, no
`libgpiod` needed -- this is what CI runs
(`.github/scripts/jetson_heartbeat_host_tests.sh`).

## Layout

```
tools/jetson_heartbeat/
├── README.md                    this file
├── Makefile
├── include/jetson_heartbeat/
│   └── config.h                 pure config struct, arg parsing, rate->period math
├── src/
│   ├── config.c                 implementation of the above (no libgpiod dependency)
│   └── main.c                   the GPIO toggle loop (libgpiod, clock_nanosleep)
├── tests/                       host-runnable unit tests of config.c, plain gcc, no gpiod
│   ├── framework.h
│   ├── main.c
│   └── test_heartbeat_config.c
└── systemd/
    └── racer-heartbeat.service
```

## What's next before this is bench-tested (not just installed)

1. Solder the mux board's JETSON connector (`firmware/safety_mux/README.md`'s connector map)
   and wire pin 7 + pin 9 to it.
2. Loopback-jumper edge-rate confirmation described above (human places the wire).
3. Confirm the Pico's `heartbeat_input.c` (once built and flashed, `pico/` is currently
   unverified on hardware per that directory's own README) actually observes edges from this
   signal and that `heartbeat_input_age_s()` tracks them.
4. Roadmap 1.3's actual kill test: freeze the Jetson for real, prove the mux cuts, human
   present, wheels off the ground.
