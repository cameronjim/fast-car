#include "safety_mux/pwm_window.h"

#include <math.h>
#include <stddef.h>

bool pwm_window_accept_us(double pulse_us, double min_us, double max_us, double tolerance_us,
                          double* clamped_out_us) {
  // BOUNDS FIRST, same order and same reasoning as pwm_validity.c: a non-finite bound makes
  // every comparison below false, which without this check reads as "accepted" nowhere but is
  // still the fail-OPEN shape to avoid. An INVERTED pair (min > max) is rejected explicitly
  // rather than left to the comparisons, because widening both sides by the tolerance can
  // make a narrowly inverted pair (say min 1020, max 1000) overlap again and accept a pulse
  // that pwm_is_valid_us() would have rejected. A broken calibration must not become a pass.
  if (!isfinite(min_us) || !isfinite(max_us) || min_us > max_us) {
    return false;
  }
  if (!isfinite(pulse_us)) {
    return false;
  }

  // A NaN, infinite, or negative tolerance degrades to zero: the window narrows back to the
  // exact configured range. That is the fail-closed direction. An infinite tolerance left
  // as-is would accept every finite pulse ever measured.
  double tol = (isfinite(tolerance_us) && tolerance_us > 0.0) ? tolerance_us : 0.0;

  if (pulse_us < (min_us - tol)) {
    return false;
  }
  if (pulse_us > (max_us + tol)) {
    return false;
  }

  if (clamped_out_us != NULL) {
    // Clamp into the UNWIDENED window. The tolerance decides what is believed; it never
    // decides what is forwarded.
    double clamped = pulse_us;
    if (clamped < min_us) {
      clamped = min_us;
    }
    if (clamped > max_us) {
      clamped = max_us;
    }
    *clamped_out_us = clamped;
  }
  return true;
}
