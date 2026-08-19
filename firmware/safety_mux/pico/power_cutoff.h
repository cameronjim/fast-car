// Pico SDK glue: drives the power-cutoff GPIO. UNVERIFIED ON HARDWARE -- see
// firmware/safety_mux/README.md. Deliberately NOT unit-tested (a single GPIO write, no
// meaningful host equivalent).
//
// PROPOSED (pending bench decision, see README.md pinout table): this GPIO drives a
// normally-open relay or high-side MOSFET gate in the motor/servo power path, active-HIGH =
// power ENABLED. Active-HIGH (rather than active-low = enabled) is the deliberate fail-safe
// choice: if this GPIO's driver circuit loses power or the RP2040 resets/browns out, the
// line floats or goes low, and power stays CUT -- the failure mode of the driving circuit
// itself defaults to safe.
#ifndef SAFETY_MUX_PICO_POWER_CUTOFF_H_
#define SAFETY_MUX_PICO_POWER_CUTOFF_H_

#include <stdbool.h>
#include <stdint.h>

// Configures `gpio` as the power-cutoff control output, initialized LOW (power cut) --
// power is only ever enabled by an explicit power_cutoff_set_enabled(gpio, true) call, never
// by GPIO reset-default state.
void power_cutoff_init(uint gpio);

// `enabled = true` asserts the GPIO HIGH (power path enabled, i.e. NOT cut);
// `enabled = false` drives it LOW (power path cut). Called every cycle from mux_decide()'s
// `cut` field, unconditionally -- see pico/main.c.
void power_cutoff_set_enabled(uint gpio, bool enabled);

#endif  // SAFETY_MUX_PICO_POWER_CUTOFF_H_
