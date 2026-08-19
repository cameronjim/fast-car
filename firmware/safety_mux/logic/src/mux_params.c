#include "safety_mux/mux_params.h"

#include <stddef.h>

// Deliberately not std::function/table-driven: this is a fixed, small, ordered list of
// fields, and a table would need a way to point back at RawMuxParamFields's specific member
// -- an array of member-offsets is more indirection than this earns. Each check is a literal
// `if`, matching this project's convention for safety decision logic
// (ros_ws/src/racer_safety/src/gate_logic.cpp's clamp_value/is_finite_command comment).
MuxParamsResult mux_params_from_raw(RawMuxParamFields raw) {
  if (!raw.steering_pwm_min_us.is_set) {
    MuxParamsResult result = {0};
    result.ok = false;
    result.missing_field = "steering_pwm_min_us";
    return result;
  }
  if (!raw.steering_pwm_max_us.is_set) {
    MuxParamsResult result = {0};
    result.ok = false;
    result.missing_field = "steering_pwm_max_us";
    return result;
  }
  if (!raw.steering_pwm_neutral_us.is_set) {
    MuxParamsResult result = {0};
    result.ok = false;
    result.missing_field = "steering_pwm_neutral_us";
    return result;
  }
  if (!raw.throttle_pwm_min_us.is_set) {
    MuxParamsResult result = {0};
    result.ok = false;
    result.missing_field = "throttle_pwm_min_us";
    return result;
  }
  if (!raw.throttle_pwm_max_us.is_set) {
    MuxParamsResult result = {0};
    result.ok = false;
    result.missing_field = "throttle_pwm_max_us";
    return result;
  }
  if (!raw.throttle_pwm_neutral_us.is_set) {
    MuxParamsResult result = {0};
    result.ok = false;
    result.missing_field = "throttle_pwm_neutral_us";
    return result;
  }
  if (!raw.watchdog_timeout_s.is_set) {
    MuxParamsResult result = {0};
    result.ok = false;
    result.missing_field = "watchdog_timeout_s";
    return result;
  }
  if (!raw.kill_switch_threshold_us.is_set) {
    MuxParamsResult result = {0};
    result.ok = false;
    result.missing_field = "kill_switch_threshold_us";
    return result;
  }
  if (!raw.rc_signal_min_us.is_set) {
    MuxParamsResult result = {0};
    result.ok = false;
    result.missing_field = "rc_signal_min_us";
    return result;
  }
  if (!raw.rc_signal_max_us.is_set) {
    MuxParamsResult result = {0};
    result.ok = false;
    result.missing_field = "rc_signal_max_us";
    return result;
  }

  MuxParamsResult result;
  result.ok = true;
  result.missing_field = NULL;
  result.params.steering_pwm_min_us = raw.steering_pwm_min_us.value;
  result.params.steering_pwm_max_us = raw.steering_pwm_max_us.value;
  result.params.steering_pwm_neutral_us = raw.steering_pwm_neutral_us.value;
  result.params.throttle_pwm_min_us = raw.throttle_pwm_min_us.value;
  result.params.throttle_pwm_max_us = raw.throttle_pwm_max_us.value;
  result.params.throttle_pwm_neutral_us = raw.throttle_pwm_neutral_us.value;
  result.params.watchdog_timeout_s = raw.watchdog_timeout_s.value;
  result.params.kill_switch_threshold_us = raw.kill_switch_threshold_us.value;
  result.params.rc_signal_min_us = raw.rc_signal_min_us.value;
  result.params.rc_signal_max_us = raw.rc_signal_max_us.value;
  return result;
}
