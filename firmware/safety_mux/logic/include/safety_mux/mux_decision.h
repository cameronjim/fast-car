// The layer-1 mux state machine (claude-docs/05-safety.md) -- no Pico SDK, no hardware
// access, host-testable. This is the single function firmware/safety_mux/pico/main.c calls
// once per control cycle; everything upstream of it (PWM capture, heartbeat detection) and
// downstream of it (PWM output, the power-cutoff GPIO) is Pico SDK glue that lives only in
// pico/.
#ifndef SAFETY_MUX_MUX_DECISION_H_
#define SAFETY_MUX_MUX_DECISION_H_

#include <stdbool.h>

#include "safety_mux/mux_params.h"
#include "safety_mux/rc_switch.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  MUX_REASON_NORMAL = 0,            // passthrough, everything nominal
  MUX_REASON_RC_KILL_SWITCH,        // RC kill switch is in the KILL position
  MUX_REASON_RC_SIGNAL_INVALID,     // RC kill-switch channel unreadable -- fails closed as a cut
  MUX_REASON_WATCHDOG_TIMEOUT,      // Jetson heartbeat stale
  MUX_REASON_STEERING_PWM_INVALID,  // Jetson-side steering PWM out of range/glitched
  MUX_REASON_THROTTLE_PWM_INVALID,  // Jetson-side throttle PWM out of range/glitched
} MuxCutReason;

// One control cycle's worth of raw MCU-observed input. Every PWM/age reading is used exactly
// as read -- no smoothing or filtering happens here (claude-docs/11-hardware.md: this MCU is
// "a week of glue firmware", not a filtering stage); this struct's job is only to carry the
// cycle's inputs into mux_decide(), never to interpret sensor noise itself.
typedef struct {
  double rc_kill_switch_pwm_us;
  double jetson_heartbeat_age_s;
  double jetson_steering_pwm_us;
  double jetson_throttle_pwm_us;
  // The kill switch's position as of the PREVIOUS cycle -- MuxOutput.switch_position from
  // the last mux_decide() call, which pico/main.c carries forward in a local. It exists only
  // so the kill switch's dead band can hold its position (rc_switch.h); mux_decide() stays a
  // pure function of its arguments, which is what keeps the table-driven tests honest. Seed
  // it with RC_SWITCH_SIGNAL_INVALID at power-on: that holds as KILL, never as ARMED.
  RcSwitchPosition previous_switch_position;
} MuxInput;

typedef struct {
  bool cut;  // true => power-cutoff GPIO asserted AND both PWM outputs forced to neutral
  double steering_out_us;
  double throttle_out_us;
  MuxCutReason reason;
  // This cycle's resolved kill-switch position. Feed it back as the next cycle's
  // MuxInput.previous_switch_position; nothing else consumes it.
  RcSwitchPosition switch_position;
} MuxOutput;

// The layer-1 decision (claude-docs/05-safety.md). Priority order, checked in this exact
// sequence:
//
//   1. RC kill switch (KILL or an unreadable channel) -- the only input a human directly
//      holds, checked before anything Jetson-side is even looked at.
//   2. Jetson heartbeat watchdog -- the guarantee against a frozen/hung/crashed Jetson.
//   3. Per-channel Jetson PWM validity (steering, then throttle) -- the last line of defense
//      against a glitched-but-alive command signal.
//   4. Otherwise: passthrough.
//
// This ordering is deliberate and is exactly what test_mux_decision.c's "multiple faults at
// once" cases pin down: whichever of these is checked first is reported as `reason` when
// more than one is simultaneously true. ANY cut forces BOTH outputs to their configured
// neutral value (never just one channel) and reports `cut = true` -- claude-docs/05-safety.md
// describes layer 1 as a cut, not a partial degrade, and safety_mux does not implement one.
MuxOutput mux_decide(MuxInput input, MuxParams params);

#ifdef __cplusplus
}
#endif

#endif  // SAFETY_MUX_MUX_DECISION_H_
