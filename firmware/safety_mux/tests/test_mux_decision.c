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
  p.kill_switch_hysteresis_us = 0.0;  // most cases below are about the decision ORDER, not
                                      // the dead band; the hysteresis block sets its own
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
  in.previous_switch_position = RC_SWITCH_ARMED;
  return in;
}

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
    in.rc_kill_switch_pwm_us = 1000.0;   // KILL
    in.jetson_heartbeat_age_s = 5.0;     // ALSO timed out
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

  // --- Power-on: the seed value pico/main.c uses for previous_switch_position -----------
  // At power-on nothing has been captured yet, so the kill channel reads the -1.0 sentinel
  // (pwm_capture_read_us) and the previous position is seeded RC_SWITCH_SIGNAL_INVALID. The
  // first cycle must therefore be a cut, whatever else is true.
  {
    MuxInput in = nominal_input();
    in.rc_kill_switch_pwm_us = -1.0;       // pwm_capture's "nothing captured yet" sentinel
    in.jetson_heartbeat_age_s = INFINITY;  // heartbeat_input_age_s()'s "no edge ever" value
    in.jetson_steering_pwm_us = -1.0;
    in.jetson_throttle_pwm_us = -1.0;
    in.previous_switch_position = RC_SWITCH_SIGNAL_INVALID;
    MuxOutput out = mux_decide(in, params);
    CHECK(out.cut, "power-on state (nothing captured, no heartbeat) -> cut");
    CHECK(out.reason == MUX_REASON_RC_SIGNAL_INVALID,
          "power-on state -> the unreadable kill channel is the reason, checked first");
    CHECK(out.steering_out_us == params.steering_pwm_neutral_us,
          "power-on state -> steering neutral");
    CHECK(out.throttle_out_us == params.throttle_pwm_neutral_us,
          "power-on state -> throttle neutral");
    CHECK(out.switch_position == RC_SWITCH_SIGNAL_INVALID,
          "power-on state -> switch_position reported back for the next cycle");
  }

  // --- KILL -> ARMED transition: no single-cycle glitch of a stale captured pulse --------
  // The cycle the switch arms on is decided from THAT cycle's captured steering/throttle
  // values. If those are stale or invalid, the arming cycle still cuts; passthrough only
  // happens on a cycle where every input is independently valid. There is no "arm now,
  // validate next cycle" path in mux_decide() for a stale pulse to slip through.
  {
    MuxInput in = nominal_input();
    in.rc_kill_switch_pwm_us = 1900.0;  // just moved to ARMED this cycle
    in.previous_switch_position = RC_SWITCH_KILL;
    in.jetson_steering_pwm_us = -1.0;  // nothing captured on this channel yet
    MuxOutput out = mux_decide(in, params);
    CHECK(out.cut, "arming cycle with a never-captured steering channel -> still cut");
    CHECK(out.reason == MUX_REASON_STEERING_PWM_INVALID,
          "arming cycle -> the stale channel is named, not passed through");
  }
  {
    MuxInput in = nominal_input();
    in.rc_kill_switch_pwm_us = 1900.0;
    in.previous_switch_position = RC_SWITCH_KILL;
    in.jetson_heartbeat_age_s = 5.0;  // Jetson not actually alive at the moment of arming
    MuxOutput out = mux_decide(in, params);
    CHECK(out.cut, "arming cycle with a stale heartbeat -> still cut");
    CHECK(out.reason == MUX_REASON_WATCHDOG_TIMEOUT, "arming cycle -> watchdog still wins");
  }
  {
    MuxInput in = nominal_input();
    in.rc_kill_switch_pwm_us = 1900.0;
    in.previous_switch_position = RC_SWITCH_KILL;
    MuxOutput out = mux_decide(in, params);
    CHECK(!out.cut, "arming cycle with every input valid -> passthrough on that same cycle");
    CHECK(out.switch_position == RC_SWITCH_ARMED, "arming cycle -> ARMED reported back");
  }

  // --- Kill-switch hysteresis, through the whole decision -------------------------------
  {
    MuxParams hyst = fixture_params();
    hyst.kill_switch_hysteresis_us = 100.0;  // ARM >= 1600, KILL < 1400, hold between

    MuxInput in = nominal_input();
    in.rc_kill_switch_pwm_us = 1500.0;  // parked exactly on the threshold
    in.previous_switch_position = RC_SWITCH_KILL;
    MuxOutput out = mux_decide(in, hyst);
    CHECK(out.cut, "dead band while KILLed -> stays cut");
    CHECK(out.reason == MUX_REASON_RC_KILL_SWITCH, "dead band while KILLed -> reason KILL");

    in.rc_kill_switch_pwm_us = 1650.0;  // past the arm edge
    in.previous_switch_position = out.switch_position;
    out = mux_decide(in, hyst);
    CHECK(!out.cut, "past the arm edge -> passthrough");

    in.rc_kill_switch_pwm_us = 1500.0;  // back into the dead band, now while ARMED
    in.previous_switch_position = out.switch_position;
    out = mux_decide(in, hyst);
    CHECK(!out.cut, "dead band while ARMED -> stays passthrough, no flap");

    in.rc_kill_switch_pwm_us = 1350.0;  // past the kill edge
    in.previous_switch_position = out.switch_position;
    out = mux_decide(in, hyst);
    CHECK(out.cut, "past the kill edge -> cut");
    CHECK(out.reason == MUX_REASON_RC_KILL_SWITCH, "past the kill edge -> reason KILL");
  }

  // --- A broken timeout must not disable the watchdog -----------------------------------
  {
    MuxParams broken = fixture_params();
    broken.watchdog_timeout_s = NAN;
    MuxInput in = nominal_input();
    MuxOutput out = mux_decide(in, broken);
    CHECK(out.cut, "NaN watchdog timeout -> cut, not a watchdog that never trips");
    CHECK(out.reason == MUX_REASON_WATCHDOG_TIMEOUT, "NaN watchdog timeout -> reason WATCHDOG");
  }
  // --- Broken PWM bounds must not make every command valid -------------------------------
  {
    MuxParams broken = fixture_params();
    broken.steering_pwm_max_us = NAN;
    MuxInput in = nominal_input();
    MuxOutput out = mux_decide(in, broken);
    CHECK(out.cut, "NaN steering bound -> cut, not everything-is-valid");
    CHECK(out.reason == MUX_REASON_STEERING_PWM_INVALID, "NaN steering bound -> reason STEERING");
  }
  {
    MuxParams broken = fixture_params();
    broken.kill_switch_threshold_us = NAN;
    MuxInput in = nominal_input();
    MuxOutput out = mux_decide(in, broken);
    CHECK(out.cut, "NaN kill threshold -> cut, never a held ARMED");
    CHECK(out.reason == MUX_REASON_RC_KILL_SWITCH, "NaN kill threshold -> reason KILL");
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
