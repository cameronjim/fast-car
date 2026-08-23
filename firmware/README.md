# firmware

Firmware for the two microcontrollers outside the Jetson's software stack. `safety_mux`
is the layer-1 MCU running the RC mux and power cutoff that no software node may bypass or
reconfigure; `ingest` is the RP2040/Teensy board that timestamps IMU, wheel-speed, and
steering sensors on one clock before streaming to the Jetson. See
`claude-docs/02-repo-layout.md`, `claude-docs/05-safety.md`, and `claude-docs/11-hardware.md`.
