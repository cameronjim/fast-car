// Pico SDK glue: measures pulse width (microseconds) on up to three GPIO inputs using
// GPIO edge interrupts + time_us_64(). UNVERIFIED ON HARDWARE -- see
// firmware/safety_mux/README.md. Deliberately NOT unit-tested (it is real GPIO/interrupt
// hardware access, no host equivalent); firmware/safety_mux/logic/ is the tested half.
//
// This is a simple interrupt-based capture, not a PIO program. A PIO-based capture would
// free the CPU from per-edge interrupt overhead and is a reasonable upgrade once this is on
// the bench and interrupt jitter is actually measured against the timing budget -- not
// attempted here, in keeping with "keep it dead simple" (CLAUDE.md) for a first draft.
#ifndef SAFETY_MUX_PICO_PWM_CAPTURE_H_
#define SAFETY_MUX_PICO_PWM_CAPTURE_H_

#include <stdint.h>

#include "pico/types.h"  // Pico SDK's `uint` typedef, used in the signatures below

// Registers a GPIO input to be pulse-width captured. Must be called once per channel before
// the first pwm_capture_read_us() call for that GPIO. Sets the pin to input with a pull-down
// (an idle/disconnected receiver line reads low, not floating) and enables a
// rising+falling-edge interrupt on it.
void pwm_capture_init_channel(uint gpio);

// Returns the most recently completed pulse width on `gpio`, in microseconds. Returns -1.0
// (not a valid PWM value under any vehicle_params bound, see pwm_is_valid_us()) when there
// is no BELIEVABLE recent pulse, which covers three distinct failures with one value:
//
//   - no complete rising-then-falling edge pair has ever been observed (power-on before the
//     first pulse, or a truly disconnected source);
//   - the last such pulse is older than the capture's staleness window (a stuck-high or
//     stuck-low line produces no further edges, so without this a width captured before the
//     fault would be reported as current forever -- the input is dead, not steady);
//   - every recent edge pair was too short or too long to be a real pulse (noise), so
//     nothing refreshed the channel.
//
// This function never blocks; it reads the most recent value an interrupt handler already
// recorded, with interrupts briefly disabled so the width and its timestamp cannot come from
// two different pulses.
double pwm_capture_read_us(uint gpio);

#ifdef SAFETY_MUX_DIAG
// DIAGNOSTIC BUILD ONLY (-DDIAG_BUILD=ON, see firmware/safety_mux/CMakeLists.txt and
// docs/notes/mux-diagnostic-build.md). Compiled out entirely of the shipping build.
//
// Read-only view of one capture channel's internal state. It exists because
// pwm_capture_read_us() collapses three distinct failures into the single value -1.0 (never
// saw a pulse / last pulse is stale / only implausible noise arrived), and a human at the
// bench needs to know WHICH. It changes nothing: same reads, same interrupt-disabled
// section, no state is written, no timestamp is refreshed, and nothing the mux decision
// consumes is touched.
#include <stdbool.h>

typedef struct {
  bool registered;         // pwm_capture_init_channel() has been called for this GPIO
  bool have_pulse;         // a plausible complete pulse has been captured at some point
  uint32_t last_pulse_us;  // that pulse's width; meaningless unless have_pulse
  uint64_t age_us;         // since that pulse's falling edge; 0 unless have_pulse
  bool stale;              // age_us exceeded the capture staleness window
  uint32_t max_age_us;     // that window (PWM_CAPTURE_MAX_AGE_US), so the report can name it
} PwmCaptureDiag;

PwmCaptureDiag pwm_capture_diag(uint gpio);
#endif  // SAFETY_MUX_DIAG

#endif  // SAFETY_MUX_PICO_PWM_CAPTURE_H_
