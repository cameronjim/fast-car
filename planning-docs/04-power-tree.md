# Stage 4: power tree (roadmap 1.2)

Time: 3 to 4 hours. Owner: teammate wires, Cameron does the logging software side.

Target topology (from `claude-docs/11-hardware.md` and the 2026-08 BOM audit):

```
3S pack --EC5--> Y harness --> VESC (motor)
                          --> buck-boost --> 12 V --> Jetson barrel jack
                          --> UBEC --> 5 V --> mux Pico + RC receiver   (independent rail)
```

The mux rail is independent by construction: the UBEC hangs off the pack, not off the Jetson
or the buck-boost. A Jetson brownout cannot take the kill switch down.

## Steps

1. Commission the batteries: charge both Zeee packs on the SkyRC S65 at 3S LiPo, 5.0 A,
   balance mode, in the bag, attended. Record each pack's per-cell voltages after charge in
   `docs/notes/build-log.md` (the charger displays them). Label the packs A and B.
2. Mount the buck-boost and UBEC on the chassis deck on standoffs, away from the motor and
   VESC heat. Star ground: all grounds meet at the harness, not daisy-chained.
3. With NO battery connected, plug the whole harness together and do the continuity checks:
   no path between pack + and -, VESC input polarity correct, barrel plug center pin positive.
4. First power-up with the pack: nothing else connected to the regulators' outputs. Measure:
   buck-boost output 12.0 V, UBEC output 5.0 V (plus or minus 0.1). VESC powers up (LED). If
   any reading is off, unplug and stop.
5. Connect the Jetson barrel plug to the buck-boost (Jetson arrives in stage 6 if not yet here;
   in that case connect a 1 A dummy load or skip). Confirm 12 V holds under Jetson boot.
6. Rail-voltage logging. The BOM's INA226 sensor is in the deferred cart, so for this stage the
   VESC's own input-voltage telemetry is the rail log (it measures pack voltage at the VESC,
   which is the same node the regulators hang off). Cameron brings up VESC telemetry on the
   Mac via USB (stage 6 configures the VESC) and records a trace while stepping a load on the
   motor (wheels off). The trace must visibly track the load step. When the INA226 arrives it
   is wired on the 12 V compute rail and becomes `/power/rail` per `claude-docs/04-architecture.md`.
7. Brownout drill preview: while logging, briefly unplug and replug the buck-boost's input at
   the Y harness with the UBEC still powered. The UBEC output must not flinch (multimeter on
   its 5 V). This is the physical proof the rails are independent; it is repeated formally in
   stage 5.
8. Tidy: zip-tie the harness clear of the driveshafts and steering linkage. Photograph.

## Done when

Both regulators hold their voltages with everything connected, the rails are demonstrably
independent, and a logged voltage trace tracks a bench load. Tick roadmap 1.2 with a dated note.

## Commit

Trace screenshot or CSV and the pack commissioning voltages in `docs/notes/build-log.md`; photos.
