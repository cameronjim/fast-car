# 11 — Hardware: BOM, Wiring, Sensor Sync

## Chassis decision (blocking — confirm before Phase 1)

Default: **Traxxas Slash 4x4** (grip, pace, F1TENTH parts compatibility). Consequence
accepted consciously: NO undriven wheels, so no free ground-speed reference — ground speed
is estimated (IMU + LiDAR odometry fusion), and wheel-speed sensors measure driven-wheel
speed, which combined with estimated ground speed IS the slip signal. The 2WD alternative
(free ground truth from undriven fronts, less grip) was considered and declined.

## Core BOM (Phase 1)

| Item | Notes | ~CAD |
|---|---|---|
| Slash 4x4 roller | | 400–600 |
| Sensored brushless 3652/3660, 3300–4000 KV | sensored strongly recommended: corner-exit low-RPM torque is where sensorless is weakest; ~$30 insurance, unfixable later | 120 |
| VESC 6 MkVI / Flipsky FSESC 6.7 | layer-2 current/speed limits configured here, mirrored in `vehicle_params.limits` | 130–300 |
| 2D LiDAR | scan rate > range for this use | 200–400 |
| Jetson Orin Nano 8GB | | 350 |
| Dedicated IMU at CG (ICM-42688-P class) | NOT the VESC's IMU | 40 |
| Wheel-speed sensors | see chassis decision above | 60 |
| Steering position feedback | pot or magnetic encoder on the rack; pwm↔angle table measured, not assumed linear | 30 |
| Rail voltage/current sensing on compute rail | logged EVERY run | 20 |
| 2× 3S 5000mAh, charger, safe bag | | 200 |
| 12V/5A buck with headroom + bulk capacitance | OR separate compute pack — either is fine; rail logging is the non-negotiable, not the topology | 30 |
| Ingest MCU (RP2040/Teensy) | see Sync below | 15 |
| Safety mux MCU + RC receiver | layer 1; independent of Jetson in power and code | 40 |
| **Timing gate** (IR break-beam or fixed camera) | evaluation must not time laps with the estimator under test | 50 |
| **Track furniture**: foam/flex barriers, cones | what the car hits is a budget line | 50–100 |
| **Tires matched to venue surface + 1 spare set, same compound** | compound is part of vehicle identity for sysid | 60 |
| Connectors, wire, standoffs, deck | | 90 |
| Spares: A-arms, LiDAR mount | buy BEFORE the first crash | 60 |

Total roughly **$2000–2550 CAD**. Phase 2+ (option B only, not now): B-G431B-ESC1, STM32G4,
logic analyzer.

## Sensor sync (design deliverable, `docs/sync-design.md`)

- **Ingest board** (firmware/ingest/): one small MCU reads IMU (SPI), wheel-speed sensors,
  steering pot; timestamps everything against ITS clock; streams framed packets to the
  Jetson over USB-serial. It does NO fusion, NO filtering, has NO custom PCB — it is a
  week of glue firmware, not the cut sensor-hub project. Scope creep here is a defect.
- LiDAR and VESC keep their own timelines. Offsets to the ingest clock are MEASURED (not
  assumed), documented, and stored in `vehicle_params.sensors`.
- Deliverable: expected residual jitter and its effect on state estimation at target speed,
  written down before Phase 2 closes (roadmap 2.2).

## Wiring rules

- Layer-1 mux is physically between all software and the actuators; its MCU and receiver are
  powered such that a Jetson/rail failure cannot take them down.
- Star ground at the buck; ESC power leads short with bulk capacitance at the VESC.
- Every connector polarized/keyed; harness photographed and committed to `docs/notes/`.
- Motor/ESC temperature telemetry from VESC is logged (thermal derating is on the failure
  diagnosis checklist).

## Bench discipline (Desktop B)

VESC Tool configuration is exported and committed after every change (it IS safety layer 2).
Firmware flashing, scope work, and dyno-style bench tests happen on Desktop B; results that
matter get written into `vehicle_params` or `docs/notes/`.
