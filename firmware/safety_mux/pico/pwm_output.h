// Pico SDK glue: drives a standard 50 Hz hobby servo/ESC PWM signal on a GPIO using the
// RP2040's hardware PWM peripheral. UNVERIFIED ON HARDWARE -- see
// firmware/safety_mux/README.md. Deliberately NOT unit-tested (real hardware PWM peripheral
// access, no host equivalent).
#ifndef SAFETY_MUX_PICO_PWM_OUTPUT_H_
#define SAFETY_MUX_PICO_PWM_OUTPUT_H_

#include <stdint.h>

// Configures `gpio` as a 50 Hz PWM output (the standard hobby servo/ESC frame rate). Call
// once per output channel before the first pwm_output_set_us() call.
void pwm_output_init_channel(uint gpio);

// Sets the pulse width on `gpio` to `pulse_us` microseconds within its 20 ms (50 Hz) frame.
// Every call to mux_decide() produces a value for this on every cycle (claude-docs/05-
// safety.md: layer 1 either passes through or cuts to a defined neutral -- there is no
// "leave the output as it was" case at this layer).
void pwm_output_set_us(uint gpio, double pulse_us);

#endif  // SAFETY_MUX_PICO_PWM_OUTPUT_H_
