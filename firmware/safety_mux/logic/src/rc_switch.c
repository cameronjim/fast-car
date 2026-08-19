#include "safety_mux/rc_switch.h"

#include "safety_mux/pwm_validity.h"

RcSwitchPosition rc_switch_read(double switch_pwm_us, double kill_threshold_us,
                                 double signal_min_us, double signal_max_us) {
  if (!pwm_is_valid_us(switch_pwm_us, signal_min_us, signal_max_us)) {
    return RC_SWITCH_SIGNAL_INVALID;
  }
  if (switch_pwm_us < kill_threshold_us) {
    return RC_SWITCH_KILL;
  }
  return RC_SWITCH_ARMED;
}
