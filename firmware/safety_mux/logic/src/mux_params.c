#include "safety_mux/mux_params.h"

#include <math.h>
#include <stddef.h>

// One refusal, named. Every exit from mux_params_from_raw() that is not "ok" goes through
// here, so the three problem kinds can never disagree about how they report themselves.
static MuxParamsResult refuse(const char* field, MuxParamProblem problem) {
  MuxParamsResult result = {0};
  result.ok = false;
  result.missing_field = field;
  result.problem = problem;
  return result;
}

MuxParamsResult mux_params_from_raw(RawMuxParamFields raw) {
  // A local {field, name} table rather than the run of hand-written `if`s this function used
  // to be. That was defensible while the only check was `is_set`; with a finiteness check
  // next to it the literal form is 22 near-identical blocks, and the thing most likely to go
  // wrong in 22 near-identical blocks is one of them naming the wrong field. The order here
  // IS the reporting order and matches RawMuxParamFields' declaration order, which is what
  // the tests pin.
  const struct {
    const RawMuxField* field;
    const char* name;
  } checks[] = {
      {&raw.steering_pwm_min_us, "steering_pwm_min_us"},
      {&raw.steering_pwm_max_us, "steering_pwm_max_us"},
      {&raw.steering_pwm_neutral_us, "steering_pwm_neutral_us"},
      {&raw.throttle_pwm_min_us, "throttle_pwm_min_us"},
      {&raw.throttle_pwm_max_us, "throttle_pwm_max_us"},
      {&raw.throttle_pwm_neutral_us, "throttle_pwm_neutral_us"},
      {&raw.watchdog_timeout_s, "watchdog_timeout_s"},
      {&raw.kill_switch_threshold_us, "kill_switch_threshold_us"},
      {&raw.kill_switch_hysteresis_us, "kill_switch_hysteresis_us"},
      {&raw.rc_signal_min_us, "rc_signal_min_us"},
      {&raw.rc_signal_max_us, "rc_signal_max_us"},
  };

  for (size_t i = 0; i < sizeof(checks) / sizeof(checks[0]); ++i) {
    if (!checks[i].field->is_set) {
      return refuse(checks[i].name, MUX_PARAM_PROBLEM_MISSING);
    }
    // A NaN or +-Inf that made it through the schema into the generated binding poisons every
    // comparison downstream (see pwm_validity.c's and watchdog.c's own guards for what that
    // looks like). Refuse to arm instead.
    if (!isfinite(checks[i].field->value)) {
      return refuse(checks[i].name, MUX_PARAM_PROBLEM_NOT_FINITE);
    }
  }

  MuxParams p;
  p.steering_pwm_min_us = raw.steering_pwm_min_us.value;
  p.steering_pwm_max_us = raw.steering_pwm_max_us.value;
  p.steering_pwm_neutral_us = raw.steering_pwm_neutral_us.value;
  p.throttle_pwm_min_us = raw.throttle_pwm_min_us.value;
  p.throttle_pwm_max_us = raw.throttle_pwm_max_us.value;
  p.throttle_pwm_neutral_us = raw.throttle_pwm_neutral_us.value;
  p.watchdog_timeout_s = raw.watchdog_timeout_s.value;
  p.kill_switch_threshold_us = raw.kill_switch_threshold_us.value;
  p.kill_switch_hysteresis_us = raw.kill_switch_hysteresis_us.value;
  p.rc_signal_min_us = raw.rc_signal_min_us.value;
  p.rc_signal_max_us = raw.rc_signal_max_us.value;

  // Structural checks. These are about the CUT output being expressible and the decision
  // being makeable at all, not about taste: see mux_params.h.
  if (!(p.steering_pwm_min_us <= p.steering_pwm_neutral_us &&
        p.steering_pwm_neutral_us <= p.steering_pwm_max_us)) {
    return refuse("steering_pwm_neutral_us", MUX_PARAM_PROBLEM_RANGE);
  }
  if (!(p.throttle_pwm_min_us <= p.throttle_pwm_neutral_us &&
        p.throttle_pwm_neutral_us <= p.throttle_pwm_max_us)) {
    return refuse("throttle_pwm_neutral_us", MUX_PARAM_PROBLEM_RANGE);
  }
  if (p.watchdog_timeout_s <= 0.0) {
    return refuse("watchdog_timeout_s", MUX_PARAM_PROBLEM_RANGE);
  }
  if (p.kill_switch_hysteresis_us < 0.0) {
    return refuse("kill_switch_hysteresis_us", MUX_PARAM_PROBLEM_RANGE);
  }
  if (!(p.rc_signal_min_us < p.rc_signal_max_us)) {
    return refuse("rc_signal_max_us", MUX_PARAM_PROBLEM_RANGE);
  }
  // A threshold outside the receiver's own valid range makes the kill switch undecidable:
  // one of ARMED/KILL becomes unreachable for every pulse the receiver can physically send.
  if (p.kill_switch_threshold_us < p.rc_signal_min_us ||
      p.kill_switch_threshold_us > p.rc_signal_max_us) {
    return refuse("kill_switch_threshold_us", MUX_PARAM_PROBLEM_RANGE);
  }

  MuxParamsResult result;
  result.ok = true;
  result.missing_field = NULL;
  result.problem = MUX_PARAM_PROBLEM_NONE;
  result.params = p;
  return result;
}
