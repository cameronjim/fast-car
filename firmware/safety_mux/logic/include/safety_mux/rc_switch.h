// Pure RC kill-switch channel interpretation -- no Pico SDK, no hardware access,
// host-testable.
#ifndef SAFETY_MUX_RC_SWITCH_H_
#define SAFETY_MUX_RC_SWITCH_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  RC_SWITCH_ARMED = 0,
  RC_SWITCH_KILL = 1,
  RC_SWITCH_SIGNAL_INVALID = 2,
} RcSwitchPosition;

// Interprets the RC receiver's kill-switch channel PWM reading. `switch_pwm_us` must first
// be a valid pulse within [signal_min_us, signal_max_us] (the receiver's own valid PWM
// range -- see firmware/safety_mux/README.md's pinout table; this is a property of the RC
// receiver hardware, not a vehicle_params field) or the switch reads as
// RC_SWITCH_SIGNAL_INVALID, which mux_decision.c treats identically to RC_SWITCH_KILL: an
// unreadable kill-switch channel is itself a reason to cut, never a reason to assume ARMED
// (claude-docs/05-safety.md fail-closed). Within the valid range, `switch_pwm_us >=
// kill_threshold_us` reads ARMED; below it reads KILL. `kill_threshold_us` comes from
// vehicle_params.limits.mux_kill_switch_threshold_us.
RcSwitchPosition rc_switch_read(double switch_pwm_us, double kill_threshold_us,
                                double signal_min_us, double signal_max_us);

#ifdef __cplusplus
}
#endif

#endif  // SAFETY_MUX_RC_SWITCH_H_
