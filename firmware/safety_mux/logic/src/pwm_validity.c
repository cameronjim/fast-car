#include "safety_mux/pwm_validity.h"

#include <math.h>

bool pwm_is_valid_us(double pulse_us, double min_us, double max_us) {
  // The BOUNDS are checked before the value. A non-finite bound makes both comparisons below
  // false, so a NaN min_us or max_us would have made EVERY pulse read as valid -- fail-open,
  // the one direction layer 1 may never fail in (claude-docs/05-safety.md). The bounds come
  // from config/vehicle_params.yaml via the generated binding, so a NaN there is a config
  // defect rather than a runtime event, but this function is the last thing standing between
  // that defect and a passed-through command.
  if (!isfinite(min_us) || !isfinite(max_us)) {
    return false;
  }
  if (!isfinite(pulse_us)) {
    return false;
  }
  if (pulse_us < min_us) {
    return false;
  }
  if (pulse_us > max_us) {
    return false;
  }
  return true;
}
