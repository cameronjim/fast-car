// Pico SDK glue: drives a standard 50 Hz hobby servo/ESC PWM signal on a GPIO using the
// RP2040's hardware PWM peripheral. UNVERIFIED ON HARDWARE -- see
// firmware/safety_mux/README.md. Deliberately NOT unit-tested (real hardware PWM peripheral
// access, no host equivalent).
#ifndef SAFETY_MUX_PICO_PWM_OUTPUT_H_
#define SAFETY_MUX_PICO_PWM_OUTPUT_H_

#include <stdint.h>

#include "pico/types.h"  // Pico SDK's `uint` typedef, used in the signatures below

// Drives `gpio` LOW as a plain GPIO output. Call this for every PWM output pin in the FIRST
// lines of main(), before params are read and before anything else is initialized: an
// RP2040's GPIOs come out of reset as high-impedance inputs, and a floating servo input can
// twitch the servo while a floating ESC input is what some ESCs arm on. LOW is "no pulses",
// a defined non-arming state -- it is NOT the mux's neutral output and does not stand in for
// it; it is what the pins hold for the few milliseconds before a neutral value is known, and
// what they keep if the firmware refuses to arm (pico/main.c's fault_halt_missing_param).
void pwm_output_init_safe(uint gpio);

// Configures `gpio` as a 50 Hz PWM output (the standard hobby servo/ESC frame rate) already
// carrying `initial_pulse_us`. Call once per output channel, after pwm_output_init_safe()
// and after the params that supply that neutral have been accepted. The initial level is
// programmed BEFORE the slice is enabled and before the pin is switched to the PWM function,
// so the pin never carries a 0%-duty frame between "this is a PWM output" and "this PWM
// output says neutral".
void pwm_output_init_channel(uint gpio, double initial_pulse_us);

// Sets the pulse width on `gpio` to `pulse_us` microseconds within its 20 ms (50 Hz) frame.
// A non-finite or negative `pulse_us` drives no pulse at all, and a value longer than the
// frame is clamped to the frame: the microsecond-to-tick conversion is a cast, which is
// undefined for NaN and wraps past 32767.5 us.
// Every call to mux_decide() produces a value for this on every cycle (claude-docs/05-
// safety.md: layer 1 either passes through or cuts to a defined neutral -- there is no
// "leave the output as it was" case at this layer).
void pwm_output_set_us(uint gpio, double pulse_us);

#endif  // SAFETY_MUX_PICO_PWM_OUTPUT_H_
