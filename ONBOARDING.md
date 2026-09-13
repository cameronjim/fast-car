# Onboarding: 1/10 Autonomous Racer (hardware lane)

Welcome. This project turns a Traxxas Slash 4x4 RC truck into a self-driving research car. The software already exists and drives a simulated copy of this exact car (watch it: `docs/notes/milestone-3-sim-autopilot.md`). What remains is the physical build, and that is where you come in: you are the electrical/mechanical half of a two-person build team.

## What the machine is

- Stock RC truck, keeping its chassis, drivetrain, suspension, and steering servo.
- Swapped in: a sensored brushless motor (Hobbywing EZRun 3665SD, 4000KV) and a programmable motor controller (Flipsky FSESC 6.7, a VESC).
- Added: an NVIDIA Jetson Orin Nano (the computer), a hardware kill switch (RP2040 + Flysky RC receiver on its own power rail), a 12V buck-boost rail for the Jetson, and later a LiDAR plus small sensors.

Power tree: battery to VESC to motor; battery to buck-boost (12V) to Jetson; battery to a dedicated 5V UBEC for the kill-switch MCU. The kill switch NEVER shares a rail with the computer.

## The one rule that outranks everything

Safety layering. The hardware kill switch (RC mux) is the only layer treated as a guarantee; software layers only reduce risk. Nothing may bypass it, share its power, or reconfigure it. Read `claude-docs/05-safety.md` before wiring anything (ask Cameron for the claude-docs folder; it is deliberately not in git).

## Your likely lane

- Soldering: EC5/XT60 connectors, ESC leads, capacitor across the VESC input, header pins. Shop-quality joints matter most on the high-current paths.
- Power bring-up: bench-sweep the buck-boost 9.0 to 12.6V input and confirm flat 12V out; verify every rail with a multimeter before anything is connected downstream.
- The mux board: RP2040 + level shifters (receiver PWM is 5V, RP2040 pins are 3.3V-only) wired per `firmware/safety_mux/README.md`'s pinout table.
- Bench tests with wheels off the ground, then the kill test: Jetson deliberately frozen, prove the mux cuts the motor.

## How to work with Claude here

Open Claude Code in this repo and just describe or photograph what is in front of you. The step-by-step build script is `docs/notes/hardware-arrival-checklist.md`; every measurement you take gets written into `config/vehicle_params.yaml` or a note, never left in your head. You do not need to know ROS or the software stack; the software is Cameron's and Claude's problem.

## Mac setup (both of us are on Macs)

ROS 2 does not run natively on macOS and does not need to. Everything runs in Docker on the Mac (`claude-docs/03-environments.md`); the car itself runs Linux on the Jetson. Setup: install Docker Desktop (or ask Claude for the no-admin Colima route), clone the repo, then `docker build -t ros-dev:local docker/ros-dev`. All builds, tests, and the simulator run inside that image; the live sim view streams to your browser via Foxglove on port 8765, so no Linux desktop is ever needed. Ask Claude to walk you through the sim demo in `docs/notes/milestone-5-browser-teleop.md` as your first task; if you can drive the simulated car from your browser, your environment is correct.

## House rules

- LiPos: charge attended, in the fireproof bag, with the SkyRC balance charger only.
- Wheels off the ground for any test of new wiring or code.
- Photograph the harness as built; photos get committed to `docs/notes/`.
- Every physical constant measured goes in `config/vehicle_params.yaml`, never hard-coded anywhere.
