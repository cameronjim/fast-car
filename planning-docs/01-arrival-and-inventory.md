# Stage 1: arrival and inventory

Roadmap: preparation for 1.1. Time: 1 to 2 hours. Nothing gets powered in this stage.

## What arrives

Three deliveries: the Amazon cart (electronics), the xtremerc order (truck, motor, charger),
and the Jetson from Arrow. They will not land the same day; this stage repeats per box.

## Steps

1. Photograph each box's contents laid out on a table and send the photos to Claude. Claude
   checks them against the ordered list below and names each part and its job.
2. Check every item off this list as it arrives:
   - Traxxas Slash 4x4 HD VX3 RTR (green), with its own transmitter, stock motor, stock ESC,
     steering servo, and wheels
   - Hobbywing EZRun 3665SD G3 sensored motor, 4000KV
   - SkyRC S65 balance charger
   - Flipsky FSESC 6.7 (the VESC)
   - Zeee 3S 5200mAh EC5 batteries, two
   - LiPo fireproof bag
   - DROK 9-36V to 12V 5A buck-boost converter (confirm the label says 12V 5A)
   - 1000uF 35V electrolytic capacitors, pack of 12
   - Mardatt connector kit (confirm it contains EC5 male and female pairs, XT60 pairs, wire,
     heat shrink)
   - M3 nylon standoff kit
   - Raspberry Pi Pico, two
   - Flysky FS-i6X transmitter and FS-iA6B receiver
   - 4-channel bidirectional logic level shifter boards
   - 5V/3A UBEC
   - SanDisk 64GB microSD
   - DC barrel pigtail, 5.5mm outer, 2.5mm inner
   - NVIDIA Jetson Orin Nano Super Developer Kit (part 945-13766-0000-000)
3. Open the Zeee battery box and read the pack dimensions off the label. Photograph the label.
   Do not charge yet.
4. Open the motor box last and only after xtremerc has answered whether a 3665-length can fits
   the Slash HD VX3 motor mount. If the answer is no, the sealed motor returns and Claude
   sources a 3650-length sensored motor. If xtremerc never answered, measure instead: motor
   can length (a 3665 is about 65 mm) versus the free length between the motor mount and the
   nearest obstruction on the chassis, with the stock motor removed (see stage 3, step 3).
5. Drive the stock truck as delivered, with its own transmitter, for ten minutes. This proves
   the chassis, servo, wheels, and drivetrain are healthy before anything is changed. Charge
   the Traxxas-supplied battery only if one was included; otherwise skip the drive until stage
   4 when the Zeee packs are commissioned.
6. Battery briefing (Claude walks through it, five minutes): cell count and voltages, the
   balance lead, why the charger's 3S setting matters, storage charge level, what a puffed
   pack looks like, the bag.
7. Write `docs/notes/build-log.md` with the arrival date of each box and any deviations from
   the list. This file is the running build diary from here on.

## Done when

Every part is inventoried and photographed, the stock truck has been driven or its drive is
scheduled for stage 4, the motor-fit question is answered, and `docs/notes/build-log.md` exists.

## Commit

`docs/notes/build-log.md` and the unboxing photos under `docs/notes/photos/`.
