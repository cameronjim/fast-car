#include "safety_mux/rc_switch.h"

#include <math.h>

#include "safety_mux/pwm_validity.h"

RcSwitchPosition rc_switch_read(double switch_pwm_us, double kill_threshold_us,
                                double hysteresis_us, double signal_min_us,
                                double signal_max_us, RcSwitchPosition previous) {
  if (!pwm_is_valid_us(switch_pwm_us, signal_min_us, signal_max_us)) {
    return RC_SWITCH_SIGNAL_INVALID;
  }
  // A non-finite threshold would make `pwm >= threshold + hyst` and `pwm < threshold - hyst`
  // both false, landing on "hold previous" -- which can hold ARMED, i.e. fail-open on a
  // broken config. Cut instead. (mux_params.c refuses to arm on such a value in the first
  // place; this is the second line.)
  if (!isfinite(kill_threshold_us)) {
    return RC_SWITCH_KILL;
  }
  double band = (isfinite(hysteresis_us) && hysteresis_us > 0.0) ? hysteresis_us : 0.0;

  if (switch_pwm_us >= kill_threshold_us + band) {
    return RC_SWITCH_ARMED;
  }
  if (switch_pwm_us < kill_threshold_us - band) {
    return RC_SWITCH_KILL;
  }
  // Inside the dead band: hold, but never hold ARMED out of an unknown previous state.
  return (previous == RC_SWITCH_ARMED) ? RC_SWITCH_ARMED : RC_SWITCH_KILL;
}
