# docker/car/patches

Patches the car image applies to third-party source it builds at a pinned commit. Each one is
applied with `git apply` right after the pinned checkout, so a patch that no longer applies
fails the image build instead of being skipped.

## vesc_driver-readonly.patch

Against [f1tenth/vesc](https://github.com/f1tenth/vesc) `153998df` (ros2 branch head,
2023-03-27), package `vesc_driver` only. Makes the driver telemetry-only, because on this car
the motor is commanded ONLY by the PPM pulse through the layer-1 safety mux (`CLAUDE.md`
invariant 1, `claude-docs/05-safety.md`, `claude-docs/11-hardware.md`).

| Change | Why |
|---|---|
| The six command subscriptions (`commands/motor/{duty_cycle,current,brake,speed,position}`, `commands/servo/position`) and the servo "sensor" publisher are removed | the driver offers no command interface at all, so nothing in the ROS graph can reach the VESC through it |
| `VescInterface::setDutyCycle/setCurrent/setBrake/setSpeed/setPosition/setServo` are no-ops | second, independent line: even if a subscription were added back, no set-command frame is ever written to the serial port |
| The IMU poll is dropped | the car's IMU is the dedicated one at the CG, never the VESC's; also halves UART traffic |
| `temp_fet` / `temp_motor` are copied into `VescState` | upstream parses them and leaves the message fields at 0 |
| `baud` parameter (default 115200) | upstream hard-codes 115200 |
| Serial flow control `NONE` (was `HARDWARE`) | upstream targets the VESC's USB CDC port; on the Jetson's 3-wire UART hardware flow control would wait on a CTS line that is not connected |

What the patched driver still sends: `COMM_FW_VERSION` (at start-up and on shutdown) and
`COMM_GET_VALUES` (50 Hz). Verified 2026-10-07 in the ros-dev container against a fake VESC on
a pseudo-terminal: with publishers running on every upstream command topic, the fake VESC
received only those two command ids.

`docker/car/Dockerfile` refuses the build if the patched `vesc_driver.cpp` still contains
`create_subscription` or `vesc_interface.cpp` still contains `send(VescPacketSet`, and
`ros_ws/src/racer_bringup/test/test_vesc_telemetry_launch.py` checks the patch text in CI.
Regenerate the patch from a checkout of the pinned commit with `git diff > ...` after editing,
never by hand-editing hunks.
