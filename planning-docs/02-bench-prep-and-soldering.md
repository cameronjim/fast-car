# Stage 2: bench prep and soldering (shop session)

Roadmap: feeds 1.1 and 1.2. Time: 3 to 5 hours at the engineering shop. Owner: teammate, with
Cameron present for the connector-family decisions.

This stage does all the soldering in one or two shop visits so the rest of the build is
plugging keyed connectors together. Bring: the connector kit, the VESC, the buck-boost, the
UBEC, the level shifter boards, the Picos, the capacitors, the barrel pigtail, and a phone.

## Soldering standards

- Shop station at 320 to 350 C with a small tip for boards and header pins; 380 to 400 C with a
  wide chisel tip for battery-gauge wire and connectors.
- Every joint shiny and concave, no dull gray blobs, no stray strands. Photograph the first few
  joints and send them to Claude before continuing.
- Heat shrink over every exposed conductor. Red for positive, black for negative, always.
- After each assembly, multimeter continuity check: no path between + and -.

## Steps

1. Decide the connector standard. Recommendation: EC5 on everything that carries battery
   current (battery, VESC input, buck-boost input, UBEC input via a splitter or harness), so
   the packs plug in without adapters. Write the decision into `docs/notes/build-log.md`.
2. VESC battery leads: solder an EC5 male onto the FSESC 6.7's bare battery wires (red to +,
   black to -). Solder one or two 1000uF capacitors across those leads as close to the VESC as
   the leads allow, observing capacitor polarity (the stripe marks negative). Heat shrink.
3. Power splitter harness: build a Y harness with one EC5 female (battery side) feeding three
   outputs: the VESC, the buck-boost input, and the UBEC input. Use 14 AWG for the VESC branch
   and 18 to 20 AWG for the two regulator branches. Fuse the two regulator branches if the shop
   has inline fuse holders (5 A for the buck-boost, 3 A for the UBEC); note it if not.
4. Buck-boost output: solder the DC barrel pigtail to the converter's output terminals (center
   pin positive, verify with the multimeter on the plug). Heat shrink.
5. Charger adapter: the SkyRC S65 ships with an XT60 charge lead. Build one XT60-male to
   EC5-female adapter so the packs can be charged.
6. Level shifter boards: solder header pins onto two boards (one spare). Solder header pins
   onto both Picos if they arrived without them.
7. UBEC output: the UBEC has a servo-style 3-pin output. Leave it as is; it plugs into a header
   later.
8. Buck-boost bench sweep: with a shop lab supply on the converter input, sweep from 9.0 V to
   12.6 V and record the output at 9.0, 10.0, 11.1, 12.0, and 12.6 V. Output must stay at
   12 V (plus or minus 0.3 V) across the whole range. Then load the output with a 2 A load
   (shop resistor or load bank) and repeat at 9.0 V. Record all readings in
   `docs/notes/build-log.md`. A unit that sags at low input is not usable for the Jetson rail.
9. Motor sensor cable: identify the Hobbywing 6-pin sensor pinout from the motor's datasheet
   or by probing (hall outputs switch when a magnet passes; temperature pin reads a thermistor
   resistance; find +5 V and GND first). Compare with the VESC sensor port order (GND, +5V, H1,
   H2, H3, TEMP). Repin the JST-PH connector so the wires land in the VESC's order. Label the
   cable. This is verified in stage 6 by VESC Tool's motor detection.
10. Photograph every finished harness and adapter for `docs/notes/photos/`.

## Done when

All power connectors are on, the splitter harness and charger adapter exist, the buck-boost
holds 12 V across the sweep under load, the sensor cable is repinned and labeled, and the
photos are committed.

## Commit

Sweep readings and connector decisions in `docs/notes/build-log.md`; photos in
`docs/notes/photos/`.
