// DIAGNOSTIC BUILD ONLY -- this whole file is compiled only when the firmware is configured
// with -DDIAG_BUILD=ON (firmware/safety_mux/CMakeLists.txt). The shipping build never
// compiles or links it, and pico/main.c's only references to it are inside
// `#ifdef SAFETY_MUX_DIAG`.
//
// WHAT THIS IS FOR: a human at the bench, with a USB cable and a serial terminal, watching
// why the mux is cutting. See docs/notes/mux-diagnostic-build.md for how to build it, how to
// attach to it, and how to read a line.
//
// WHAT THIS IS NOT: part of the safety system. It reads state that the decision has already
// produced and prints it. It takes no decision, changes no output, holds no state that any
// decision reads, and calls nothing in firmware/safety_mux/logic/ -- the decision logic,
// its priority order, and its fail-safe defaults are identical in both builds.
//
// REAL-TIME: diag_report_tick() is called once per 200 Hz control cycle and returns
// immediately except roughly twice per second. When it does print, it does not block: it
// prints nothing at all unless a USB host has the CDC port open (stdio_usb_connected()), and
// the diagnostic target compiles pico_stdio_usb with PICO_STDIO_USB_STDOUT_TIMEOUT_US=0 so a
// host that has stopped reading causes characters to be DROPPED, never waited on. See that
// note's "Not blocking on USB" section for the SDK code this is based on.
#ifndef SAFETY_MUX_PICO_DIAG_REPORT_H_
#define SAFETY_MUX_PICO_DIAG_REPORT_H_

#ifdef SAFETY_MUX_DIAG

#include "pico/types.h"
#include "safety_mux/mux_decision.h"
#include "safety_mux/mux_params.h"

// The GPIO numbers the report prints next to each field. Passed in from pico/main.c's
// #defines rather than duplicated here: main.c stays the ONLY place a GPIO number is written
// down in this firmware (firmware/safety_mux/README.md's pinout table).
typedef struct {
  uint kill_gpio;
  uint steering_gpio;
  uint throttle_gpio;
  uint heartbeat_gpio;
  uint servo_gpio;
  uint esc_gpio;
  uint cutoff_gpio;
} DiagGpioMap;

// Call once, after params are accepted and before the control loop starts.
void diag_report_init(const DiagGpioMap* gpios, const MuxParams* params);

// Call once per control cycle, at the END of the cycle, with that cycle's actual inputs and
// the decision that was actually applied to the pins. Returns immediately unless it is time
// to print (about twice a second) and a USB host is attached.
void diag_report_tick(const MuxInput* input, const MuxOutput* output);

#endif  // SAFETY_MUX_DIAG
#endif  // SAFETY_MUX_PICO_DIAG_REPORT_H_
