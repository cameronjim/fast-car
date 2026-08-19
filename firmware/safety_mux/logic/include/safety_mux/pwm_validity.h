// Pure PWM pulse-width range check -- no Pico SDK, no hardware access, host-testable.
#ifndef SAFETY_MUX_PWM_VALIDITY_H_
#define SAFETY_MUX_PWM_VALIDITY_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Returns true iff `pulse_us` is a finite value within [min_us, max_us] (inclusive). Used to
// reject a glitched, disconnected, or out-of-spec PWM reading before it is trusted anywhere
// in the mux decision (claude-docs/05-safety.md: "Fails CLOSED"). NaN/+-Inf are always
// invalid regardless of bounds. `min_us`/`max_us` are not validated relative to each other
// here -- this is a pure range check; a caller that supplies min_us > max_us gets "always
// invalid" out of the ordinary comparisons below, which is itself a safe (fail-closed)
// outcome, not a crash or an unmodeled case.
bool pwm_is_valid_us(double pulse_us, double min_us, double max_us);

#ifdef __cplusplus
}
#endif

#endif  // SAFETY_MUX_PWM_VALIDITY_H_
