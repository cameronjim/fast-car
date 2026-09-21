# Bench session 2026-09-20 to 2026-09-21: first power-up

Status at the end of the night: car assembled and powered, radio bound, mux running,
servo does not move. Root cause found and a fix prepared but not yet applied.

## What was proven

Read from the mux diagnostic firmware over the Pico's USB (`/dev/ttyACM0` on the Jetson):

```
KILL  gp12 = 2000us  FRESH  -> ARMED
HB    gp5  = NO_EDGES_EVER   TIMED_OUT
STEER gp10 = 1484us  FRESH
THR   gp7  = 1484us  FRESH
DECISION = CUT  reason = 2:WATCHDOG_TIMEOUT
OUT   servo gp1 = 1500us  esc gp3 = 1500us  50.00 Hz  duty 7.50%
```

- Radio chain works end to end: FS-i6X bound to FS-iA6B, CH5 on knob VrA, clockwise is
  armed, counter-clockwise is kill, failsafe stored at the kill end.
- Jetson steering (header pin 15) and throttle (pin 33) reach the Pico.
- Mux outputs are correct: 1500 us, 50.00 Hz, both channels.
- The steering servo is good: it steers when plugged straight into receiver CH1.
- Continuity confirmed: Pico GP1 to servo header signal, servo header ground to Pico ground.
- The one failing input is the heartbeat. The mux cuts for exactly that reason.

## Root cause

The Jetson toggles gpiochip0 line 144 (PAC.06, header pin 7) correctly at kernel level,
but that pad boots with its output driver tristated, so nothing reaches the physical pin.
Meter on pin 7 with the line held high: 0.0 V. Only pads that a jetson-io overlay
configures (pins 15, 32, 33) have tristate cleared, which is why PWM on 15 and 33 works.
`/sys/kernel/debug/pinctrl/2430000.pinmux/pinconf-groups` shows tristate=1 for pin 7 and
tristate=0 for the three PWM pads. Full write-up in `tools/jetson_pinmux/README.md`.

The heartbeat was only ever verified with debugfs and gpioinfo, never with a meter on the
pin. That is what let this hide since 2026-09-12.

## Fix, prepared and reviewed, NOT yet applied

`tools/jetson_pinmux/racer-hdr40-gpio.dts` clears tristate on header pins 7 and 32 and
carries the pwm1 (pin 15) and pwm5 (pin 33) pad config, so the heartbeat stays on pin 7
and no wire moves. `install.sh` compiles first, backs up `extlinux.conf`, and replaces the
jetson-io overlay entry. `rollback.sh` restores the backup. Nothing is active until reboot.

## Hardware notes

- UBEC #1 read about 6.3 V (meter reads roughly 3 percent high). That exceeds the Pico
  regulator's absolute maximum. Pico #1 later ran hot enough to smoke. Pico #1 is retired.
- UBEC #2 is a 3A-6S unit with a 5V/6V jumper, set to 5V, about 5.3 V under load. Usable.
  A proper 5.0 V regulator for the mux rail is on the wish list.
- UBEC input + and - were shorted briefly by a loose wire. Re-terminated, no visible damage.
- Battery 12.2 V, cool, no swelling.
- Jetson header pins are too tight to probe reliably by hand; probe the perfboard socket
  holes instead, where the same signals land.
- pwmchip map with the three-PWM overlay: pwmchip0 = 3280000 (pin 15), pwmchip1 = 32a0000,
  pwmchip2 = 32c0000 (pin 33), pwmchip3 = 32e0000, pwmchip4 = tachometer.

## Morning checklist

Wheels off the ground throughout. Kill knob counter-clockwise until told otherwise.

1. Battery in. Jetson boots (about 40 s). Confirm UBEC output is 5.0 to 5.5 V before
   any Pico is seated.
2. On the Jetson: `sudo apt-get install -y device-tree-compiler` if `dtc` is missing, then
   `cd ~/car/tools/jetson_pinmux && sudo ./install.sh && sudo reboot`.
3. After reboot: `sudo systemctl stop racer-heartbeat && sudo ./verify.sh 144 15`.
   Expect `tristate=0`. With the Pico socket empty, probe the GP5 hole (Pico pin 7 position,
   ground is the hole beside it): expect 3.3 V while it holds. Then
   `sudo systemctl start racer-heartbeat`.
4. Flash the spare Pico with `safety_mux_diag.uf2`, seat it, plug its USB into the Jetson.
   Touch-check it for heat in the first minute.
5. Read the diagnostic: `python3 ~/car/tools/mux_diag/read_mux_diag.py` (or add `--watch`);
   expect `HB OK` and, with the knob clockwise, `DECISION PASS`. Raw fallback:
   `sudo cat /dev/ttyACM0`. If still `NO_EDGES_EVER`, stop and read
   `tools/jetson_pinmux/README.md` troubleshooting before touching wiring.
6. Steering sweep from the Jetson (1200 / 1500 / 1800 us on pwmchip0) with the knob
   killed: wheels must not move. Knob armed: wheels must steer. That is the G1 kill test.
7. Only then: VESC Tool configuration on the FSESC 6.7 (undervoltage cutoff, conservative
   current limit, PPM "Current No Reverse With Brake", sensorless detection) and a
   throttle sweep with wheels off the ground.

## Results, morning of 2026-09-21

Numbered against the checklist above where a step maps directly; two additional bench
findings from the same session (7 and 8 below) are included because they resolve open items
from this doc even though they were not separate numbered checklist steps.

1. (checklist 1) Battery in, Jetson boots, UBEC output confirmed before any Pico was seated
   -- PASSED. 5V position: 5.69 V unloaded, sagging to about 5.3 V loaded (in range); the
   6V position (6.81 V) is not used for the Pico.
2. (checklist 2) Pinmux overlay installed, `dtc` 1.6.1 installed via apt, rebooted --
   PASSED. `extlinux`'s `OVERLAYS` line now points at `/boot/racer-hdr40-gpio.dtbo`.
3. (checklist 3) Post-reboot verify -- PASSED. `soc_gpio59_pac6` (pin 7) tristate went
   1 -> 0; pins 15 and 33 kept function `gp`; pin 32 (`soc_gpio19_pg6`) also reads
   tristate=0. Meter on the GP5 socket hole with pin 7 held high: 3.44 V (meter reads about
   3 percent high).
4. (checklist 4) Spare Pico flashed with `safety_mux_diag.uf2`, seated, USB into the Jetson
   -- PASSED. Stayed cool after several minutes running. Pico #1 confirmed dead separately
   (heats within seconds on clean desktop USB alone) and retired.
5. (checklist 5) Diagnostic read -- PASSED, including the first `DECISION PASS` this mux has
   ever produced. Transmitter off:
   ```
   KILL UNREADABLE NO_EDGES | HB OK age 7ms | STEER 1484us | THR 1484us | DECISION CUT reason 1:RC_SIGNAL_INVALID
   ```
   Transmitter on, knob counter-clockwise:
   ```
   KILL KILLED 1000us ... DECISION CUT reason 1:RC_KILL_SWITCH
   ```
   Knob clockwise:
   ```
   KILL ARMED 2000us | HB OK age 6ms | STEER 1484us | THR 1485us | DECISION PASS
   ```
6. (checklist 6) G1 kill test, steering only, throttle held at 1500 us -- PASSED. Armed
   sweep 1200/1500/1800/1500 us on `pwmchip0` turned the front wheels left/centre/right/
   centre. Identical sweep with the knob killed: wheels did not move, mux stayed
   `CUT reason 1` throughout.
7. Heartbeat-loss test (not a separate checklist step, added this session) -- PASSED. Armed,
   steer left (`STEER 1172us`, `PASS`); `systemctl stop racer-heartbeat` -> `HB TIMED_OUT`,
   `DECISION CUT reason 2:WATCHDOG_TIMEOUT`, servo self-centred; steer right with the
   heartbeat dead -> `STEER 1797us` seen arriving, `DECISION CUT`, wheels did not move;
   recentre, `systemctl start racer-heartbeat` -> `HB OK`, `PASS` again.
8. UBEC root cause for Pico #1's death identified -- PASSED (finding, not a checklist step).
   Pico #1 had been run on the UBEC's 6V position (6.81 V), which is the likely cause.
9. (checklist 7) VESC Tool configuration and throttle sweep -- PASSED, 2026-09-21 midday.
   VESC configured per `planning-docs/06-vesc-config-and-jetson-bringup.md` step 3 (see
   `docs/notes/build-log.md`'s 2026-09-21 midday entry for the full detected parameters and
   limits); exported config committed as `config/vesc/2026-09-21-fsesc67-motor.xml` and
   `2026-09-21-fsesc67-app.xml`. Throttle test through the mux, wheels off the ground,
   steering held at 1500 us, knob armed (`DECISION=PASS`): 1560 us produced nothing; 1600
   and 1650 us made the rear tyres click for a few seconds without turning; 1700 then
   1750 us spun the wheels up fast, to the ERPM cap. Knob turned counter-clockwise while
   spinning:
   ```
   KILL 1000us KILLED | DECISION=CUT reason 1:RC_KILL_SWITCH
   ```
   wheels stopped, throttle returned to 1500 us. First time the car has moved under Jetson
   command; motor-channel radio kill proven. Combined with checklist step 6 (steering kill,
   proven above) and the heartbeat-loss test (item 7 above), Gate G1's bench evidence is now
   complete.

Open items: real switch for the kill channel (this radio only offers VrA/VrB on CH5 and
CH6); 5.0 V regulator for the mux rail, or a Schottky diode in series to the Pico's VSYS
feed, for margin (UBEC 5V position sags to about 5.3 V loaded); FSESC 4.12 swap when it
arrives; sensor cable adapter; provisional vehicle_params replaced with measured values;
measure steering endpoints (the servo buzzed holding 1200 us this morning, probably against
its mechanical stop -- 1000-2000 us is still provisional); `tools/mux_diag/read_mux_diag.py`
one-shot hang and slow consecutive-call behavior fixed in this PR (see that tool's tests);
throttle start deadzone -- this drivetrain needs roughly 1700 us (about 40 percent of the
throttle range) to start sensorless from rest, and the throttle map in `racer_drivers` /
`vehicle_params` does not model it yet (2026-09-21 midday); mirror the VESC limits into
`config/vehicle_params.yaml` per planning-docs/06 step 6 -- still TODO (2026-09-21 midday);
sensored hall adapter for low-speed start, the proper fix for the throttle deadzone above
(planned, not yet in hand); `tools/mux_diag/read_mux_diag.py`'s `--lines` budget counted raw
serial lines instead of parseable verdict lines, which made it fail against the live device
on a fresh attach (the four-line banner ate the single-line default budget) even though the
raw serial stream was fine -- fixed in this PR (2026-09-21 midday; see
`docs/notes/build-log.md`).
