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

Open items after that: real switch for the kill channel (this radio only offers VrA/VrB on
CH5 and CH6), 5.0 V regulator for the mux rail, FSESC 4.12 swap when it arrives, sensor
cable adapter, provisional vehicle_params replaced with measured values.
