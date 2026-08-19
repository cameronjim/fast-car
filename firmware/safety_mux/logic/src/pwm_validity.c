#include "safety_mux/pwm_validity.h"

#include <math.h>

bool pwm_is_valid_us(double pulse_us, double min_us, double max_us) {
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
