# Build log

Dated entries for physical build decisions: what changed on the car or on a board, why, and
what still has to be proven on the bench before the decision counts as correct. Design docs
say how things are meant to be; this file says when a choice was made and on what grounds.
Nothing here is a test result unless it says it was observed.

## 2026-09-12 -- kill-switch board: the Pico was drawn mirrored, corrected

**Change.** The first board drawing had the Pico mirrored: the GPIO row was drawn on the
wrong side of the chip. Viewed from the top with the USB socket facing the board's left edge,
pins 1 to 20 run along the lower row (pin 1 bottom-left) and pins 40 down to 21 along the
upper row. The drawing had those two rows swapped, so every GPIO in the wiring plan was on
the power row and VSYS, 3V3 OUT and the pin-38 ground were on the GPIO row.

**How it was caught.** Cameron spotted it while reading the layout, before any soldering.
Nothing physical was affected, because nothing physical exists yet.

**What changed.** Only the row numbers. No hole column moved: VSYS is still column 6, 3V3 OUT
still column 9, GP2 to GP8 still columns 8 to 15. The plan now reads VSYS (6,12), 3V3 OUT
(9,12), GND pin 38 (7,12) on the upper row, and GP2 (8,19), GP3 (9,19), GP4 (10,19), GP5
(11,19), GND pin 8 (12,19), GP6 (13,19), GP7 (14,19), GP8 (15,19) on the lower row. The
continuity check before first power-up now probes the row-19 holes.

**Consequence for the wiring.** The three level-shifted signals and the heartbeat now have to
reach the far side of the Pico. They are routed on the underside and pass beneath the Pico's
body. That is safe because components sit on the top face and every wire is soldered on the
copper face, so there is nothing but bare board under the Pico. Noted on the build page.

**Flip-mirror warning added.** Both figures are drawn from the top, and the columns mirror
left-to-right the moment the board is turned over to solder: column 1 ends up on the right.
Column 1 and row 1 get marked with a paint pen on both faces before any soldering, and every
count starts from the marked corner.

**Status.** Planned layout only. Still nothing soldered, nothing powered, nothing measured.
The orientation above is from the Pico datasheet pinout, not from a board in hand.

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
