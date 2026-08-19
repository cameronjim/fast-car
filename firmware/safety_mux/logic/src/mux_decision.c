#include "safety_mux/mux_decision.h"

#include "safety_mux/pwm_validity.h"
#include "safety_mux/rc_switch.h"
#include "safety_mux/watchdog.h"

static MuxOutput cut_with_reason(MuxParams params, MuxCutReason reason) {
  MuxOutput out;
  out.cut = true;
  out.steering_out_us = params.steering_pwm_neutral_us;
  out.throttle_out_us = params.throttle_pwm_neutral_us;
  out.reason = reason;
  return out;
}

MuxOutput mux_decide(MuxInput input, MuxParams params) {
  // 1. RC kill switch -- checked first, before anything Jetson-side. An unreadable channel
  // (RC_SWITCH_SIGNAL_INVALID) is treated exactly like RC_SWITCH_KILL: never assume ARMED
  // from a signal the MCU cannot actually interpret (claude-docs/05-safety.md fail-closed).
  RcSwitchPosition switch_position =
      rc_switch_read(input.rc_kill_switch_pwm_us, params.kill_switch_threshold_us,
                     params.rc_signal_min_us, params.rc_signal_max_us);
  if (switch_position == RC_SWITCH_KILL) {
    return cut_with_reason(params, MUX_REASON_RC_KILL_SWITCH);
  }
  if (switch_position == RC_SWITCH_SIGNAL_INVALID) {
    return cut_with_reason(params, MUX_REASON_RC_SIGNAL_INVALID);
  }

  // 2. Jetson heartbeat watchdog -- the guarantee against a frozen/hung/crashed Jetson
  // (claude-docs/05-safety.md layer 1's whole reason to exist).
  if (watchdog_timed_out(input.jetson_heartbeat_age_s, params.watchdog_timeout_s)) {
    return cut_with_reason(params, MUX_REASON_WATCHDOG_TIMEOUT);
  }

  // 3. Per-channel Jetson PWM validity -- a glitched-but-alive command signal is not the
  // same failure as a frozen Jetson, but it is just as unsafe to forward.
  if (!pwm_is_valid_us(input.jetson_steering_pwm_us, params.steering_pwm_min_us,
                        params.steering_pwm_max_us)) {
    return cut_with_reason(params, MUX_REASON_STEERING_PWM_INVALID);
  }
  if (!pwm_is_valid_us(input.jetson_throttle_pwm_us, params.throttle_pwm_min_us,
                        params.throttle_pwm_max_us)) {
    return cut_with_reason(params, MUX_REASON_THROTTLE_PWM_INVALID);
  }

  // 4. Nominal: pass the Jetson's own commanded PWM straight through.
  MuxOutput out;
  out.cut = false;
  out.steering_out_us = input.jetson_steering_pwm_us;
  out.throttle_out_us = input.jetson_throttle_pwm_us;
  out.reason = MUX_REASON_NORMAL;
  return out;
}
