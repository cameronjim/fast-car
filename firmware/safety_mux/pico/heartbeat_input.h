// Pico SDK glue: tracks the age of the Jetson-side heartbeat signal. UNVERIFIED ON HARDWARE
// -- see firmware/safety_mux/README.md. Deliberately NOT unit-tested (real GPIO/interrupt
// hardware access, no host equivalent); the AGE VALUE this produces is fed into
// safety_mux/watchdog.h's watchdog_timed_out(), which IS host-tested.
//
// PROPOSED signal design (pending bench decision, see README.md pinout table): a simple
// square-wave GPIO toggle from the Jetson at a fixed rate (e.g. a lightweight process
// toggling a GPIO line, independent of any ROS/network stack being alive), NOT a UART
// message -- claude-docs/05-safety.md requires this MCU to share no code path with the
// Jetson's software stack, and the simplest signal that still proves "the Jetson's low-level
// I/O is alive and being serviced" is a raw digital toggle, not a parsed protocol.
#ifndef SAFETY_MUX_PICO_HEARTBEAT_INPUT_H_
#define SAFETY_MUX_PICO_HEARTBEAT_INPUT_H_

#include <stdint.h>

// Configures `gpio` as the heartbeat input and arms edge detection. Call once at startup.
void heartbeat_input_init(uint gpio);

// Seconds since the last heartbeat edge was observed on the configured GPIO. Returns +Inf if
// no edge has ever been observed (e.g. at power-on before the Jetson starts toggling it) --
// watchdog_timed_out() treats +Inf exactly like any other non-finite age: timed out.
double heartbeat_input_age_s(void);

#endif  // SAFETY_MUX_PICO_HEARTBEAT_INPUT_H_
