# Phase 2 parts, sourced 2026-09-30

Shopping list for roadmap Phase 2 (`claude-docs/01-roadmap.md`: ingest board, LiDAR, state
estimation) plus the optional camera and track items the owner asked about. Sourcing rule from
the owner: order from amazon.ca with fast shipping where possible, and from at most one other
Canadian seller for the rest. That other seller is **pishop.ca** for hobby boards and
**digikey.ca** for the two parts nobody else stocks (INA260, ICM-20948). If you want to keep it
to exactly two stores, use digikey.ca as the second store and swap the pishop.ca lines for the
digikey.ca equivalents listed in each section.

All prices CAD before tax and shipping. Shipping and stock lines were read off the live
listing on the date given in each section, delivering to Vancouver V6T. Anything not read from
a live page is marked UNVERIFIED. Re-check prices before ordering; amazon.ca seller-shipped
lines move weekly.

Already built, do not buy: the Xerun-to-VESC hall sensor adapter (straight-through, done
2026-09-29). The Traxxas RPM sensor is optional now that the VESC reports sensored ERPM.

## Essentials only (2026-10-02)

The owner has standoffs and wire already, so the minimum order to get LiDAR and cameras
running is the lines below. Everything is plug-in: the RPLIDAR adapter goes into a Jetson USB
port, CSI cameras clip into the two 22-pin sockets, USB cameras plug in. Nothing to solder.

| Item | Store | Price | Link |
|---|---|---|---|
| RPLIDAR C1, 10 Hz | amazon.ca | $93.99 | https://www.amazon.ca/dp/B0CT31PH8S |
| Yahboom IMX219 120 degree CSI camera with 22-pin cable, x2 | amazon.ca | $44.19 each plus $5.05 shipping | https://www.amazon.ca/dp/B0C5844MWQ |
| ELP AR0234 global-shutter USB 3 camera, 120 fps | amazon.ca | $142.19 plus $5.32 shipping | https://www.amazon.ca/dp/B0H3F5RCS7 |

Total about $340 before tax. The Yahboom boards include the 22-pin ribbon, so the separate
15-to-22 cable pack is only needed with the Pi Camera V2 or Waveshare modules.

Higher scan-rate LiDAR, priced 2026-10-02: the only verified real upgrade is the Hokuyo
UST-10LX (40 Hz, 43 k samples/s, 0.25 degree, 10 m, 270 degree, Ethernet, 12 V), the
standard F1TENTH sensor, USD 1,200 at Acroname (about CAD 1,650 plus duties, shipping to
Canada UNVERIFIED). It is not plug-in: its AWG28 power leads get crimped to the 12 V rail,
and it takes the Jetson's only Ethernet port, so the dev link moves to Wi-Fi or a USB
Ethernet adapter. The RPLIDAR S3 ($689, 20 Hz at its maximum setting, delivery late October
to mid November) is the middle option. The A3M1 and every YDLIDAR and LDROBOT unit checked stay
under 20 Hz, so they are not upgrades over the C1. Neither digikey.ca nor amazon.ca carries
Hokuyo; mouser.ca and robotshop blocked the check.

## A. LiDAR and cameras (roadmap 2.3; cameras optional)

Researched 2026-09-30 from live amazon.ca product pages. Everything in this section is on
amazon.ca, so no second store is needed for it.

### A1. 2D LiDAR: Slamtec RPLIDAR C1

| Item | Seller | Price | Link | Stock and shipping |
|---|---|---|---|---|
| RPLIDAR C1 (Waveshare listing) | amazon.ca, Ships from Amazon, sold by waveshare | $93.99 | https://www.amazon.ca/dp/B0CT31PH8S | "Only 9 left in stock", "FREE delivery Thursday, October 8" |
| Fallback: RPLIDAR C1, DFRobot DFR1138 | digikey.ca | $109.36 | https://www.digikey.ca/en/products/detail/dfrobot/DFR1138/23028657 | 105 in stock, delivery time UNVERIFIED |

0.05 to 12 m, 5 kHz sample rate, 8 to 12 Hz scan (10 Hz typical), 0.72 degree resolution,
UART at 460800 through the included USB adapter, 5 V at 260 mA from one Jetson USB port,
official ROS 2 driver `sllidar_ros2`. The Amazon page does not list box contents; Waveshare's
own product page lists the adapter and a USB A to C cable, and Waveshare is the seller, so the
adapter is very likely included (UNVERIFIED on the Amazon page). The DigiKey box contents are
verified.

Why the C1 and not something faster: on a small indoor track, range past about 6 m is wasted,
so the decision is scan rate, angular density and driver quality. Every option under $300
tops out at 10 to 13 Hz, so the cheaper alternatives (LD19 $110.88 B0B1V8D36H, A1M8 $145.59
B07TJW5SXF, both arriving October 2 from a US Amazon warehouse) cost more and scan no faster.
Only the A2M12 ($302.39, October 14) and S2 ($503.99, October 12) reach 15 Hz, at three to
five times the price. At 3 to 5 m/s the car moves 0.3 to 0.5 m between 10 Hz scans; that is
handled in software by deskewing each scan with odometry, not by buying a faster head.

| Model | Range | Scan rate | Sample rate | Price | Shipping |
|---|---|---|---|---|---|
| RPLIDAR C1 | 12 m | 8 to 12 Hz | 5 kHz | $93.99 | October 8 |
| LDROBOT LD19 | 12 m | 5 to 13 Hz | 4.5 to 8 kHz (UNVERIFIED) | $110.88 | October 2, US warehouse |
| RPLIDAR A1M8 | 12 m | 2 to 10 Hz | 8 kHz | $145.59 | October 2, US warehouse |
| RPLIDAR A2M12 | 12 m | 5 to 15 Hz | 16 kHz | $302.39 | October 14, adapter not listed |
| RPLIDAR S2 | 30 m | 8 to 15 Hz | 32 kHz | $503.99 | October 12 |

### A2. LiDAR mount hardware

| Item | Seller | Price | Link | Stock and shipping |
|---|---|---|---|---|
| Nylon M2 / M2.5 / M3 hex standoff kit, 360 pcs | amazon.ca, Ships from Amazon | $15.99 | https://www.amazon.ca/dp/B0B9YLRMMT | In Stock, free delivery October 7 or paid next day |

Covers both M2.5 and M3, since the C1 hole size was not read from the datasheet (UNVERIFIED).
Nylon creeps under vibration: use lock nuts or swap to brass standoffs for the final mount.
The mount plate itself is a printed part.

### A3. Cameras (optional, for recognition experiments; not part of the LiDAR thesis)

Connector facts for the Orin Nano dev kit: it has two 22-pin 0.5 mm CSI sockets, and nearly
every IMX219 board (Pi Camera V2 and clones) has a 15-pin connector, so a 15-pin to 22-pin
ribbon is required. Do NOT buy the Raspberry Pi Camera Module 3 (IMX708) or its clones: stock
JetPack 6 has no driver for it. IMX219 works with the stock jetson-io overlay. IMX477 needs a
vendor driver package, so it is the second choice.

| Item | Seller | Price | Link | Stock and shipping |
|---|---|---|---|---|
| Raspberry Pi Camera Module V2, genuine IMX219, 62 degree | amazon.ca, Ships from Amazon, sold by Lyfestyle Things | $35.18 | https://www.amazon.ca/dp/B01ER2SKFS | "Only 5 left", free delivery October 13 or paid next day |
| YINETTECH 15-pin to 22-pin CSI ribbon, 4 pack (2 x 15 cm, 2 x 30 cm) | amazon.ca, Ships from Amazon | $16.59 | https://www.amazon.ca/dp/B0DHVCRQQT | In Stock, free delivery October 4 |
| Waveshare IMX219-160, 160 degree wide view | amazon.ca, Ships from Amazon, sold by waveshare | $26.99 | https://www.amazon.ca/dp/B07H2D4WYR | "Only 2 left", free delivery October 8 |
| Logitech C920x USB webcam, 1080p30, UVC | amazon.ca, sold by Amazon.ca | $79.99 | https://www.amazon.ca/dp/B085TFF7M1 | In Stock, free delivery October 4 |

Pick one: the V2 plus the ribbon pack ($51.77) is the lowest-risk CSI path (1080p30 or 720p60
on the Jetson through Argus / GStreamer). The C920x is plug-and-play over USB with the
`usb_cam` or `v4l2_camera` ROS 2 drivers, but it is rolling shutter at 30 fps, so expect
blur and skew at speed. The 160 degree module sees both track edges near the car and needs
lens calibration for the barrel distortion. Global-shutter USB alternative if the camera work
gets serious: ELP AR0234 colour, 1920x1200 at 120 fps, $142.19, https://www.amazon.ca/dp/B0H3F5RCS7
(US Amazon warehouse, October 2). Depth cameras are out of budget for an experiment
(RealSense D435i about $700, OAK-D Lite about $660, search-page prices only).

## B. Ingest board sensors (roadmap 2.1)

Researched 2026-09-21. Corrections to the original BOM wording: SparkFun SEN-19029 is a GNSS
receiver, not an IMU; the Adafruit AS5600 board is product 6357 (not 5744); Adafruit 2167 is the
3 mm break-beam pair (2168 is the 5 mm pair).

### B1. IMU at the CG (SPI to the ingest Pico)

| Item | Seller | Price | Link | Stock and shipping |
|---|---|---|---|---|
| SparkFun 9DoF IMU Breakout ICM-20948 (Qwiic), SEN-15335 | digikey.ca | $32.77 | https://www.digikey.ca/en/products/detail/sparkfun-electronics/SEN-15335/10279707 | 305 in stock, status Obsolete, "fast delivery to Canada, typically within 24 hours" |

Buy two: the board is marked obsolete and there is no hobby-priced ICM-42688-P breakout in
Canada (TDK eval boards only, $81 to $186). SPI up to 7 MHz on the 0.1 inch header, I2C on the
Qwiic port, 1.95 to 3.6 V. Needs a header strip and six jumper leads.

### B2. Steering position (I2C to the ingest Pico)

| Item | Seller | Price | Link | Stock and shipping |
|---|---|---|---|---|
| DUTTY AS5600 magnetic encoder module, 2 pcs | amazon.ca | $12.99 | https://www.amazon.ca/dp/B0DD7B64FB | In Stock, Ships from Amazon, "fastest delivery" two days |
| AroudightElive NdFeB 6 x 2.5 mm diametric magnets, 50 pcs | amazon.ca | $48.15 | https://www.amazon.ca/dp/B0FP4TJPH7 | In Stock, seller QQmay, free delivery within a week |

The DUTTY package list shows only the two modules, so treat the magnet as not included, and
the AroudightElive diametric claim is title-only (UNVERIFIED). Cheaper import alternative:
HiLetgo 2x AS5600 with magnet, $14.99, https://www.amazon.ca/dp/B09KGWC1PT. Mount: printed
6 mm bore hub on the servo horn or kingpin, magnet 0.5 to 3 mm above the chip, on axis.

### B3. Rail current and voltage sensor (roadmap note under 1.6)

| Item | Seller | Price | Link | Stock and shipping |
|---|---|---|---|---|
| Adafruit INA260, product 4226 | digikey.ca | $15.55 | https://www.digikey.ca/en/products/detail/adafruit-industries-llc/4226/10130492 | 1,734 in stock |

Integrated 2 milliohm shunt, 36 V, 15 A continuous, I2C, 3 or 5 V logic. Goes in series on the
12 V rail between the buck-boost and the Jetson. amazon.ca lists it as "Currently unavailable"
and pishop.ca has no listing. Do not substitute a generic INA226 module: its 0.1 ohm shunt
gives 0.82 A full scale and the rail draws several amps.

### B4. Timing gate (evaluation, `claude-docs/09-evaluation.md`)

| Item | Seller | Price | Link | Stock and shipping |
|---|---|---|---|---|
| IR break-beam sensor pair, 3 mm LEDs (Adafruit 2167 equivalent) | pishop.ca | $7.95 | https://www.pishop.ca/product/ir-break-beam-sensor-3mm-leds/ | In stock, ships from Canada |

About 25 cm range, 3.3 to 5.5 V, open-collector output: power from 5 V, pull the output up to
3.3 V and read it on a Pico GPIO. If the track is wider than 25 cm, the Taiss E3F-R2NK
retroreflective switch (NPN, 2 m range, $28.93, https://www.amazon.ca/dp/B073WCHTD6, ships
internationally) works from the 12 V rail with a 10 K pull-up to 3.3 V on the output. Never the
PNP variant, which sources 12 V into the Pico.

### B5. Wiring for the above

| Item | Seller | Price | Link | Stock and shipping |
|---|---|---|---|---|
| elechawk Qwiic / STEMMA QT cable kit, 12 cables incl. female-header breakout | amazon.ca | $11.99 | https://www.amazon.ca/dp/B08HQ1VSVL | In Stock, Ships from Amazon |
| Adafruit STEMMA QT JST-SH 4-pin cable, 100 mm (buy 2) | pishop.ca | $1.95 each | https://www.pishop.ca/product/stemma-qt-qwiic-jst-sh-4-pin-cable-100mm-long/ | In stock |
| ELEGOO 120 pcs 20 cm Dupont wires, M-M / M-F / F-F | amazon.ca | $12.99 | https://www.amazon.ca/dp/B01EV70C78 | In Stock, Ships from Amazon |

Skip silicone hookup wire and heat shrink if the Mardatt kit still covers them (Haerkn 22 AWG
$21.98 B07TFF9FTM and Ginsco heat shrink $13.99 B01MFA3OFA otherwise). digikey.ca equivalent
for the Qwiic cable: SparkFun PRT-14427 flexible Qwiic 100 mm, $2.86.

## C. Track furniture and tires

Researched 2026-09-21 from amazon.ca search pages only (product pages would not load that
day), so the ASINs are UNVERIFIED; search the quoted titles.

| Item | Seller | Price | Where | Shipping |
|---|---|---|---|---|
| "7 Inch Plastic Sport Training Traffic Cones, Set of 10, 5 Colours" | amazon.ca | $19.99 | https://www.amazon.ca/s?k=6+inch+traffic+cones+pack | Free delivery over $35, shipped by Amazon |
| "6 Pack Pool Noodles Soft Foam Flexible Swimming Pool Noodles" | amazon.ca | $18.69 | https://www.amazon.ca/s?k=foam+pool+noodles+pack | Import, slow (about two weeks) |
| SLOOSH foam pool noodles, 6 pack, 48 in (fast alternative) | amazon.ca | $68.39 | same search | Shipped by Amazon, two to four days |
| "Thin Super Glue Liquid, Super Fast CA Glue with Anti-Clog Cap and Microtips" | amazon.ca | $11.99 | https://www.amazon.ca/s?k=thin+CA+glue+RC+tire+cyanoacrylate | Shipped by Amazon, next day |

Pool noodles are cheaper in person at Canadian Tire (56 in, seasonal, price UNVERIFIED).

Tires: keep the stock Slash 4x4 SCT tires for asphalt and concrete (standard compound, long
life). For dirt, loose clay or short high-grip sessions, Traxxas 5871R (S1 compound, 4.3 x
1.7 in, MSRP USD 18.95) is a direct swap on the 12 mm hex. Buy the spare set in the same
compound as the set you run: compound is part of the vehicle identity for system ID
(`claude-docs/07-sim-and-sysid.md`).

## D. Optional: independent wheel-speed sensor

Only worth it as a drivetrain-independent cross-check; the VESC already reports sensored ERPM.

| Item | Seller | Price | Link | Stock |
|---|---|---|---|---|
| Traxxas 6520 long RPM sensor | Big Boys With Cool Toys (Mississauga) | $17.98 | https://www.bigboyswithcooltoys.ca/products/tra6520-sensor-rpm-long-3x4mm-bcs-2-3x4-gs-2 | In stock |
| Traxxas 6538 telemetry magnet holders (spur gear) | Big Boys With Cool Toys | $4.98 | https://www.bigboyswithcooltoys.ca/products/tra6538-telemetry-trigger-magnet-holders-spur-gear | Listed |

No published electrical spec: scope the output before connecting it to a 3.3 V GPIO. This adds
a third seller, so leave it out unless you want the cross-check.

## E. Optional: water squirter (out of scope for the thesis)

Not part of the project (`claude-docs/00-project-overview.md`). If built, it runs from its own
4xAA pack and touches nothing on the safety mux rail or the drive topics. Parts (all amazon.ca,
Ships from Amazon unless noted, 2026-09-21): DEVMO 5 pcs 3 to 6 V brushless pump $23.99
B07T4ZNJR5; uxcell 6 mm ID silicone tube $12.19 B08L34RHX7; Restaurantware 8 oz squeeze bottle
$11.31 B0721QBXN7; 10 pcs 15 A MOSFET trigger module (3.3 V trigger) $16.99 B0D2XTC6XC;
DAIERTEK 4xAA holder with switch $12.59 B09N1GDWQ9; Adafruit mini pan-tilt kit $29.95 at
pishop.ca. Subtotal about $107.

## Recommended order

Two stores: amazon.ca for everything below except the two digikey.ca lines. Subtotals before
BC GST and PST. Amazon shipping is free on these lines once the Amazon-shipped part of the
order passes $35.

| # | Item | Store | Price |
|---|---|---|---|
| 1 | RPLIDAR C1, B0CT31PH8S | amazon.ca | $93.99 |
| 2 | Nylon standoff kit, B0B9YLRMMT | amazon.ca | $15.99 |
| 3 | AS5600 modules x2, B0DD7B64FB | amazon.ca | $12.99 |
| 4 | 6 x 2.5 mm diametric magnets, B0FP4TJPH7 | amazon.ca | $48.15 |
| 5 | Qwiic cable kit, B08HQ1VSVL | amazon.ca | $11.99 |
| 6 | Dupont wire set, B01EV70C78 | amazon.ca | $12.99 |
| 7 | Traffic cones x10 and thin CA glue (search titles in section C) | amazon.ca | $31.98 |
| | amazon.ca subtotal | | $228.08 |
| 8 | ICM-20948 breakout x2, SEN-15335 | digikey.ca | $65.54 |
| 9 | INA260 breakout, Adafruit 4226 | digikey.ca | $15.55 |
| 11 | Qwiic cable 100 mm, PRT-14427 | digikey.ca | $2.86 |
| | digikey.ca subtotal | | $83.95 |
| | Core total | | $312.03 |
| 12 | Optional camera: Pi Camera V2 plus 15-to-22 ribbon pack | amazon.ca | $51.77 |
| 13 | Optional camera: Logitech C920x | amazon.ca | $79.99 |

Timing gate: DigiKey does not stock the Adafruit break-beam pair (checked 2026-09-30). Either
accept pishop.ca as a third store for the $7.95 pair, or add the Taiss retroreflective gate
from section B4 ($28.93) to the amazon.ca order. Pool noodles: buy in person or accept the
slow import listing.

## Not verified

RPLIDAR C1 Amazon box contents and mounting hole size; DigiKey delivery time to Vancouver;
the AS5600 magnet inclusion and the AroudightElive diametric claim; all section C ASINs;
Logitech C920x field of view; LD19 sample rate; depth camera prices.

