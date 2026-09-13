# Build log

Dated entries for physical build decisions: what changed on the car or on a board, why, and
what still has to be proven on the bench before the decision counts as correct. Design docs
say how things are meant to be; this file says when a choice was made and on what grounds.
Nothing here is a test result unless it says it was observed.

## 2026-09-12 -- kill-switch board: one 4-pin Jetson header, and the command path

**Change.** The three Jetson inputs on the layer-1 mux perfboard are consolidated into a
single 4-pin male header: STEER SIG, THROTTLE SIG, HEARTBEAT, GND, pin 1 at the left and
marked with a paint pen. It replaces the earlier plan of three separate connectors (a 3-pin
steering, a 3-pin throttle, a 2-pin heartbeat). The KILL input and the SERVO output stay
3-pin, and the VESC output stays 3-pin with the +5V position left empty. See
`firmware/safety_mux/README.md`'s board connector map for the full layout.

**Why.** A voltage is a difference measured against a ground, so all three Jetson signals
have to be referenced to the same ground the Jetson sent them from. That is one wire, not
three. At 50 Hz, with microamp currents and microsecond edges, one shared return carries all
three signals without any crosstalk worth measuring, so separate returns per signal buy
nothing and cost two more connectors to get backwards. The receiver and servo headers stay
3-pin for a different reason: their middle pin is power, not reference. The board feeds 5 V
to the receiver and the servo. The Jetson powers itself, so its header has no 5 V pin at all,
which also removes any chance of back-feeding the Jetson from the board's rail.

**Cost of the change.** One connector that can be plugged in reversed. Reversed, it puts the
heartbeat where steering belongs, and no firmware check can see that. Mitigation is physical:
pin 1 marked on the board, a matching mark on the plug, and a keyed cable.

**Failure mode, claimed but not yet proven.** All three Jetson inputs are pull-down on the
Pico (`pico/pwm_capture.c` and `pico/heartbeat_input.c` both call `gpio_pull_down`), so an
unplugged or broken cable reads as a steady low: no pulses and no heartbeat toggle, which the
watchdog and the PWM validity check both treat as a cut. That is the intended behavior and it
is written down here as a claim, not a result. The bench test that settles it ("unplug the
Jetson cable mid-run, confirm both outputs go neutral within the watchdog timeout and the
cutoff opens") is now in `planning-docs/05-safety-mux-and-kill-test.md` and in
`docs/notes/hardware-arrival-checklist.md`. Until it has been observed on a scope, this
paragraph is a prediction.

**Command path decision.** The Jetson commands the motor by PWM, through the mux board, into
the VESC's PPM input. The VESC's USB link is for telemetry and configuration only, and
nothing in the drive path goes through it. The reason is that the mux only has authority over
what passes through it: a USB current-control path would run around the mux entirely and put
the motor under the Jetson's direct control, which is exactly what layer 1 exists to prevent.
This costs some fidelity compared with current control, and it is accepted for now. USB
current control can be reconsidered only after the GPIO 8 power-cutoff circuit physically
exists and has been kill-tested, because at that point the mux can cut motor power outright
rather than only cutting the command. That circuit does not exist yet.

**Status.** Planned layout only. Nothing soldered, nothing powered, nothing measured.
Running record of the physical build: decisions, deliveries, measurements, and mistakes, in
the order they happened. Per `claude-docs/10-conventions.md`, writeup is continuous. Newest
entries at the bottom.

## 2026-09-10

- Jetson Orin Nano Super Dev Kit arrived (Arrow, the last unit). JetPack 6.2.1 SD-card image
  (`jp62-r1-orin-nano-sd-card-image.zip`, Jetson Linux 36.4.4) downloaded; balenaEtcher
  downloaded. Flash pending. Firmware-version gate noted: a 2026 unit should already carry
  36.x UEFI; if it does not boot the JP6 card, the JetPack 5.1.3 bridge path applies.
- Both Picos confirmed to have male headers pre-soldered, so the kill-switch board becomes a
  socketed perfboard (female headers for the Pico and level shifter, male headers for the
  off-board plugs, screw terminals for 5 V in). No breadboard needed.
- Miuzei 151-piece perfboard + header kit ordered (amazon.ca B0H11M27WN).

## 2026-09-11

- Truck (Slash 4x4 HD VX3, green), Zeee 3S 5200 packs, and the rest of the Amazon cart
  arrived. Motor (Xerun 3652SD G3 4500KV) and the VESC still in transit.
- No stock battery and no charger on hand. **The SkyRC S65 was never actually ordered**: the
  order plan listed it on the xtremerc order but it was not on the receipt, and the shop's
  own email ("as long as you already have batteries and a charger") was the missed warning.
  Replacement ordered: 80 W / 6 A B6-class balance charger, amazon.ca B0G9MDKCW3, chosen
  because it includes an XT60-to-EC5 lead (plug-and-charge on the Zeee packs). Charge plan:
  LiPo, 3S, balance mode, 5.0 A (about 1C).
- **VESC minimum-voltage mismatch found.** Flipsky specs the FSESC 6.7 at 14-60 V, 4S
  minimum (their product page and their own 4.20-vs-6.7 comparison post). The traction pack
  is 3S: 12.6 V full, ~9.6 V empty, below spec across the whole range. The 2026-08
  compatibility audit checked current, sensors, and connectors but not minimum voltage.
  Options considered: bench-test the 6.7 at 3S (out-of-spec on the drive path, rejected once
  an in-spec part was available), two packs in series (6S would over-speed and overheat the
  2-3S motor, rejected), 4S packs (same motor problem, rejected). Decision: cancel the 6.7
  (delayed anyway) and order the **Flipsky FSESC 4.12, 50 A, 8-60 V, 3S-rated**, amazon.ca
  B0CSDYF7RF, delivery Sep 22-29. Flipsky-direct Mini FSESC 4.20 ($56 USD) was the faster
  alternative if express shipping had been chosen.
- FSESC 4.12 compatibility check against every interface: 3S voltage in spec; EC5 soldered
  onto its bare 12 AWG leads; 50 A continuous vs ~30-40 A motor draw with layer-2 limits at
  ~40 A; 60,000 ERPM cap = ~30,000 motor RPM = ~15 m/s at the wheels (double what the car
  needs); hall-sensor port for the Xerun's cable (repin still required); PPM input driven
  directly by the mux board's 3.3 V GPIO 7 pulses; its 5 V BEC pin left unconnected on the
  board; USB telemetry to the Jetson unchanged; ~60x40x20 mm, smaller than the stock ESC.
- Stock Traxxas ESC ruled out for the project: sensorless-only (no hall socket), no computer
  interface or telemetry, no programmable or exportable limits (layer 2 lives in the VESC
  config). It goes in the spares bag.
- Kill-switch board wiring plan written against `firmware/safety_mux/pico/main.c`: GPIO 2/3/4
  inputs (kill, steering, throttle) through the level shifter; GPIO 5 heartbeat direct;
  GPIO 6/7 servo/VESC outputs direct; GPIO 8 power cutoff reserved; Pico powered via VSYS
  from the UBEC's 5 V; shifter HV = 5 V, LV = Pico 3V3 OUT; single shared ground.
- Today's work order: mechanical check of the truck, photograph then remove stock ESC and
  receiver (servo and motor stay), then session 1 at the shop (sockets and headers only, no
  wiring until the layout photo is checked).
