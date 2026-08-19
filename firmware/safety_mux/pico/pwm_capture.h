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

// Registers a GPIO input to be pulse-width captured. Must be called once per channel before
// the first pwm_capture_read_us() call for that GPIO. Sets the pin to input with a pull-down
// (an idle/disconnected receiver line reads low, not floating) and enables a
// rising+falling-edge interrupt on it.
void pwm_capture_init_channel(uint gpio);

// Returns the most recently completed pulse width on `gpio`, in microseconds. Returns -1.0
// (not a valid PWM value under any vehicle_params bound, see pwm_is_valid_us()) if no
// complete rising-then-falling edge pair has ever been observed on this channel -- e.g. at
// power-on before the first pulse, or if the source is truly disconnected. This function
// never blocks; it reads the most recent value an interrupt handler already recorded.
double pwm_capture_read_us(uint gpio);

#endif  // SAFETY_MUX_PICO_PWM_CAPTURE_H_
