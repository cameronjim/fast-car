# Build status, 2026-09-14

A single readable snapshot of where the physical build actually is, as reported by Cameron on
2026-09-14. This is a status page, not a procedure -- the procedures live in `planning-docs/`
and `docs/notes/hardware-arrival-checklist.md`. Where something is unknown below, it says so;
nothing here is inferred progress.

## Parts: what is actually in the build right now

| Part | Status | Notes |
|---|---|---|
| Flipsky FSESC 6.7 | **In the build now** | Out of spec for this pack (14-60 V / 4S minimum on a 3S pack), a deliberate and informed choice. See "ESC decision" below. |
| Flipsky FSESC 4.12 | In transit, not yet in the build | 3S-rated, ordered 2026-09-11, arriving roughly Sep 22-29. Intended controller for real driving; will replace the 6.7. |
| Separate 1000 uF bulk capacitors | **Dropped, will not be fitted** | Stay in the parts bin. See "Capacitor decision" below. |
| Hobbywing Xerun 3652SD G3 motor | Mounted, stock pinion retained | Phase wires being soldered direct to the motor (see below). |
| Stock ESC and receiver | Removed | Receiver bagged, not used in this build. |
| Kill-switch / ingest perfboard | In progress | Terminal pins, Pico socket, and level-shifter sockets soldered; inter-component wiring in progress. |
| Jetson Orin Nano | Set up, currently powered off | JetPack 6.2, WiFi, SSH key + passwordless sudo, heartbeat service verified. |

### Capacitor decision

The FSESC has three onboard bulk capacitors, and the battery leads to it are short. The
separate 1000 uF capacitors in the BOM are not being fitted. This closes the "optional
insurance" item noted in `claude-docs/11-hardware.md`'s BOM audit. `planning-docs/02-bench-
prep-and-soldering.md` and `docs/notes/hardware-arrival-checklist.md` no longer instruct
fitting them.

### ESC decision

Proceeding with the FSESC 6.7 for the test build, knowing it is specced 14-60 V (4S minimum)
against a 3S pack. This is a deliberate, informed decision, not a repeat of the 2026-09-11
mismatch finding (`docs/notes/build-log.md`, and `claude-docs/11-hardware.md`'s audit note).
Most likely outcome: it boots and runs at light load. Untested: sustained load. The 3S-rated
FSESC 4.12 remains the intended controller for real driving and is still on the way. The swap
is expected to stay a small job because the connectors are the only shared work between the
two ESCs.

## Physical build: done / in progress / blocked / not started

| Item | Status |
|---|---|
| Motor mounted, stock pinion | Done |
| Motor phase wires soldered to VESC | In progress (direct solder, individual heat shrink per joint; only one female bullet connector was on hand, so bullet connectors were skipped for now) |
| Stock ESC and receiver removed | Done |
| Chassis parts laid out for assembly | In progress |
| Perfboard terminal pins, Pico socket, level-shifter sockets | Done |
| Perfboard inter-component wiring | In progress |
| Perfboard flashed with firmware | Not started |
| Perfboard bench tested | Not started |
| Pico <-> perfboard pin mapping finalized | **Blocked** -- pending Cameron's confirmation of the final holes |
| EC5 connectors on VESC battery leads | Not started |
| First LiPo charge | Blocked -- charger has not arrived |
| Jetson setup (JetPack, WiFi, SSH, heartbeat) | Done |
| Jetson PWM pinmux enable / `pwmchip` numbering | Not started |
| Jetson-to-mux-board wiring (4 lines) | Not started |
| Wheels-off bench test | Not started |
| G1 kill test | Not started |

## Forward plan, in order, with what blocks what

1. **Finish soldering the perfboard wires.** In progress now.
2. **Solder the VESC phase wires to the motor** (direct, heat shrink each joint). In progress
   now; expected to be redone when the FSESC 4.12 arrives and swaps in.
3. **Lay out and assemble parts on the car** (deck layout, standoffs, routing).

Then, before anything drives, in the order the existing docs require:

4. **EC5 connectors onto the VESC's battery leads.**
5. **First LiPo charge**, once the charger arrives (the B6-class balance charger ordered
   2026-09-11, `docs/notes/build-log.md`). Arrival date is not confirmed here; check before
   assuming it has landed.
6. **Finalize the Pico pin mapping and flash the firmware.** Blocked on step 4 of "physical
   build" above (Cameron confirming the final holes against the physical board). Once
   confirmed, the firmware `#define`s and `firmware/safety_mux/README.md`'s pinout table get
   updated together, in the same change.
7. **Continuity checks before any power** (no shorts between battery + and -, per
   `planning-docs/README.md`'s standing rules).
8. **Bench power-up of the rails with a multimeter** (buck-boost and UBEC outputs, per
   `planning-docs/04-power-tree.md`).
9. **Jetson PWM pinmux enable and `pwmchip` numbering confirmation**
   (`docs/notes/first-boot-runbook.md` steps 4-5).
10. **Wire the four Jetson lines to the mux board** (steering, throttle, heartbeat, ground --
    `firmware/safety_mux/README.md`'s board connector map).
11. **Wheels-off bench test sequence** (per-channel actuation tests,
    `planning-docs/05-safety-mux-and-kill-test.md` section D).
12. **The G1 kill test** (`planning-docs/05-safety-mux-and-kill-test.md` section E). Nothing
    drives on the ground before this passes.

Steps 1-3 have no hard ordering dependency between each other, but step 2's joints get redone
when the FSESC 4.12 arrives, so there is no benefit to rushing ahead of the perfboard work.
Step 6 is the one confirmed blocker on the list right now: everything from step 6 onward that
touches the mux board waits on it. Step 5 (first charge) is blocked on the charger, separately
from step 6.

## What needs no hardware assembly

These can happen any time the Jetson is powered, independent of the perfboard, the motor, or
the chassis work:

- Jetson PWM pinmux enable and `pwmchip` numbering confirmation (step 9 above).
- Any further Jetson-side software or configuration work (SSH, the car image build, ROS
  workspace build on-device) per `docs/notes/first-boot-runbook.md` steps 1-3, 6.
- Reviewing/updating firmware and docs once the pin mapping is confirmed (step 6 is a doc/code
  update as much as a physical one, though flashing itself needs the perfboard in hand).
