#include <math.h>
#include <stddef.h>

#include "framework.h"
#include "safety_mux/mux_decision.h"

static MuxParams fixture_params(void) {
  MuxParams p;
  p.steering_pwm_min_us = 1000.0;
  p.steering_pwm_max_us = 2000.0;
  p.steering_pwm_neutral_us = 1500.0;
  p.throttle_pwm_min_us = 1000.0;
  p.throttle_pwm_max_us = 2000.0;
  p.throttle_pwm_neutral_us = 1520.0;  // deliberately distinct from steering's neutral, so a
                                       // test asserting "the RIGHT neutral went to the RIGHT
                                       // channel" cannot pass by accident
  p.watchdog_timeout_s = 0.5;
  p.kill_switch_threshold_us = 1500.0;
  p.rc_signal_min_us = 1000.0;
  p.rc_signal_max_us = 2000.0;
  return p;
}

// A nominal, everything-fine input: switch armed, fresh heartbeat, both PWMs valid and
// distinct from either neutral value (so passthrough is unambiguous from a cut).
static MuxInput nominal_input(void) {
  MuxInput in;
  in.rc_kill_switch_pwm_us = 1900.0;   // armed (>= 1500 threshold)
  in.jetson_heartbeat_age_s = 0.01;    // fresh
  in.jetson_steering_pwm_us = 1300.0;  // valid, not neutral
  in.jetson_throttle_pwm_us = 1700.0;  // valid, not neutral
  return in;
}

typedef struct {
  const char* name;
  MuxInput input;
  bool expected_cut;
  MuxCutReason expected_reason;
} MuxDecisionCase;

void test_mux_decision_suite(void) {
  MuxParams params = fixture_params();

  // --- Nominal passthrough --------------------------------------------------------------
  {
    MuxInput in = nominal_input();
    MuxOutput out = mux_decide(in, params);
    CHECK(!out.cut, "nominal input -> not cut");
    CHECK(out.reason == MUX_REASON_NORMAL, "nominal input -> reason NORMAL");
    CHECK(out.steering_out_us == in.jetson_steering_pwm_us,
          "nominal input -> steering passed through unchanged");
    CHECK(out.throttle_out_us == in.jetson_throttle_pwm_us,
          "nominal input -> throttle passed through unchanged");
  }

  // --- Single-fault cases: everything else nominal, one thing wrong ---------------------
  {
    MuxInput in = nominal_input();
    in.rc_kill_switch_pwm_us = 1000.0;  // below threshold -> KILL
    MuxOutput out = mux_decide(in, params);
    CHECK(out.cut, "RC switch KILL -> cut");
    CHECK(out.reason == MUX_REASON_RC_KILL_SWITCH, "RC switch KILL -> reason RC_KILL_SWITCH");
    CHECK(out.steering_out_us == params.steering_pwm_neutral_us,
          "RC switch KILL -> steering forced to neutral");
    CHECK(out.throttle_out_us == params.throttle_pwm_neutral_us,
          "RC switch KILL -> throttle forced to neutral");
  }
  {
    MuxInput in = nominal_input();
    in.rc_kill_switch_pwm_us = NAN;  // unreadable channel
    MuxOutput out = mux_decide(in, params);
    CHECK(out.cut, "RC switch unreadable -> cut");
    CHECK(out.reason == MUX_REASON_RC_SIGNAL_INVALID,
          "RC switch unreadable -> reason RC_SIGNAL_INVALID (never assumed ARMED)");
  }
  {
    MuxInput in = nominal_input();
    in.jetson_heartbeat_age_s = 0.5;  // exactly at timeout
    MuxOutput out = mux_decide(in, params);
    CHECK(out.cut, "heartbeat at timeout -> cut");
    CHECK(out.reason == MUX_REASON_WATCHDOG_TIMEOUT,
          "heartbeat at timeout -> reason WATCHDOG_TIMEOUT");
  }
  {
    MuxInput in = nominal_input();
    in.jetson_steering_pwm_us = 3000.0;  // out of range
    MuxOutput out = mux_decide(in, params);
    CHECK(out.cut, "invalid steering PWM -> cut");
    CHECK(out.reason == MUX_REASON_STEERING_PWM_INVALID,
          "invalid steering PWM -> reason STEERING_PWM_INVALID");
    CHECK(out.throttle_out_us == params.throttle_pwm_neutral_us,
          "invalid steering PWM -> throttle ALSO forced to neutral (no partial cut)");
  }
  {
    MuxInput in = nominal_input();
    in.jetson_throttle_pwm_us = -1.0;  // out of range
    MuxOutput out = mux_decide(in, params);
    CHECK(out.cut, "invalid throttle PWM -> cut");
    CHECK(out.reason == MUX_REASON_THROTTLE_PWM_INVALID,
          "invalid throttle PWM -> reason THROTTLE_PWM_INVALID");
    CHECK(out.steering_out_us == params.steering_pwm_neutral_us,
          "invalid throttle PWM -> steering ALSO forced to neutral (no partial cut)");
  }

  // --- Priority order: when multiple faults are true at once, the highest-priority one is
  // reported. This is the behavior that actually encodes claude-docs/05-safety.md's ordering
  // (the RC kill switch must always win), not just a side effect of test ordering above.
  {
    MuxInput in = nominal_input();
    in.rc_kill_switch_pwm_us = 1000.0;  // KILL
    in.jetson_heartbeat_age_s = 5.0;    // ALSO timed out
    in.jetson_steering_pwm_us = 9999.0;  // ALSO invalid
    MuxOutput out = mux_decide(in, params);
    CHECK(out.reason == MUX_REASON_RC_KILL_SWITCH,
          "RC kill switch wins over a simultaneous watchdog timeout and invalid PWM");
  }
  {
    MuxInput in = nominal_input();
    in.jetson_heartbeat_age_s = 5.0;     // timed out
    in.jetson_steering_pwm_us = 9999.0;  // ALSO invalid
    in.jetson_throttle_pwm_us = 9999.0;  // ALSO invalid
    MuxOutput out = mux_decide(in, params);
    CHECK(out.reason == MUX_REASON_WATCHDOG_TIMEOUT,
          "watchdog timeout wins over simultaneous steering/throttle PWM faults");
  }
  {
    MuxInput in = nominal_input();
    in.jetson_steering_pwm_us = 9999.0;  // invalid
    in.jetson_throttle_pwm_us = 9999.0;  // ALSO invalid
    MuxOutput out = mux_decide(in, params);
    CHECK(out.reason == MUX_REASON_STEERING_PWM_INVALID,
          "steering PWM fault is checked (and reported) before throttle PWM fault");
  }

  // --- Boundary: commanding exactly the neutral value during normal operation is ordinary
  // passthrough, not itself treated as a fault or confused with a cut.
  {
    MuxInput in = nominal_input();
    in.jetson_steering_pwm_us = params.steering_pwm_neutral_us;
    in.jetson_throttle_pwm_us = params.throttle_pwm_neutral_us;
    MuxOutput out = mux_decide(in, params);
    CHECK(!out.cut, "commanding neutral value directly is still ordinary passthrough");
    CHECK(out.reason == MUX_REASON_NORMAL, "commanding neutral value -> reason NORMAL");
    CHECK(out.steering_out_us == params.steering_pwm_neutral_us,
          "commanding neutral steering value passes through as itself");
  }
}
