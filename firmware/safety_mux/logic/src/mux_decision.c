#include "safety_mux/mux_decision.h"

#include "safety_mux/pwm_window.h"
#include "safety_mux/rc_switch.h"
#include "safety_mux/watchdog.h"

static MuxOutput cut_with_reason(MuxParams params, MuxCutReason reason,
                                 RcSwitchPosition switch_position) {
  MuxOutput out;
  out.cut = true;
  out.steering_out_us = params.steering_pwm_neutral_us;
  out.throttle_out_us = params.throttle_pwm_neutral_us;
  out.reason = reason;
  out.switch_position = switch_position;
  return out;
}

MuxOutput mux_decide(MuxInput input, MuxParams params) {
  // 1. RC kill switch -- checked first, before anything Jetson-side. An unreadable channel
  // (RC_SWITCH_SIGNAL_INVALID) is treated exactly like RC_SWITCH_KILL: never assume ARMED
  // from a signal the MCU cannot actually interpret (claude-docs/05-safety.md fail-closed).
  RcSwitchPosition switch_position =
      rc_switch_read(input.rc_kill_switch_pwm_us, params.kill_switch_threshold_us,
                     params.kill_switch_hysteresis_us, params.rc_signal_min_us,
                     params.rc_signal_max_us, input.previous_switch_position);
  if (switch_position == RC_SWITCH_KILL) {
    return cut_with_reason(params, MUX_REASON_RC_KILL_SWITCH, switch_position);
  }
  if (switch_position == RC_SWITCH_SIGNAL_INVALID) {
    return cut_with_reason(params, MUX_REASON_RC_SIGNAL_INVALID, switch_position);
  }

  // 2. Jetson heartbeat watchdog -- the guarantee against a frozen/hung/crashed Jetson
  // (claude-docs/05-safety.md layer 1's whole reason to exist).
  if (watchdog_timed_out(input.jetson_heartbeat_age_s, params.watchdog_timeout_s)) {
    return cut_with_reason(params, MUX_REASON_WATCHDOG_TIMEOUT, switch_position);
  }

  // 3. Per-channel Jetson PWM validity -- a glitched-but-alive command signal is not the
  // same failure as a frozen Jetson, but it is just as unsafe to forward.
  //
  // The window is the configured range widened by PWM_WINDOW_DEFAULT_TOLERANCE_US on each
  // side, and the accepted pulse is clamped back into the UNWIDENED range before it is
  // forwarded -- see pwm_window.h for the hardware observation (GitHub issue #63) that made
  // this necessary and for how the tolerance was chosen. pwm_window_accept_us() writes the
  // clamped value only when it accepts, so these two locals cannot carry an unclamped or
  // unverified number past this point.
  double steering_out_us = 0.0;
  double throttle_out_us = 0.0;
  if (!pwm_window_accept_us(input.jetson_steering_pwm_us, params.steering_pwm_min_us,
                            params.steering_pwm_max_us, PWM_WINDOW_DEFAULT_TOLERANCE_US,
                            &steering_out_us)) {
    return cut_with_reason(params, MUX_REASON_STEERING_PWM_INVALID, switch_position);
  }
  if (!pwm_window_accept_us(input.jetson_throttle_pwm_us, params.throttle_pwm_min_us,
                            params.throttle_pwm_max_us, PWM_WINDOW_DEFAULT_TOLERANCE_US,
                            &throttle_out_us)) {
    return cut_with_reason(params, MUX_REASON_THROTTLE_PWM_INVALID, switch_position);
  }

  // 4. Nominal: pass the Jetson's own commanded PWM through, clamped to the configured range.
  // Inside the range (every command the Jetson issues that is not at an end stop) the clamp
  // is the identity, so this is still passthrough, not a filter.
  MuxOutput out;
  out.cut = false;
  out.steering_out_us = steering_out_us;
  out.throttle_out_us = throttle_out_us;
  out.reason = MUX_REASON_NORMAL;
  out.switch_position = switch_position;
  return out;
}
