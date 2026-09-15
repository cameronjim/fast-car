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

// Half-width of the dead band around the kill threshold, microseconds. A three-position or
// worn two-position switch, a trimmed transmitter, or ordinary receiver jitter can park this
// channel within a few microseconds of the threshold; without a dead band the mux would then
// flap ARMED/KILL at the 200 Hz loop rate, pulsing the ESC and the power cutoff.
//
// COMPILE-TIME, AND THAT IS A KNOWN GAP. This is a filtering constant for a specific
// transmitter/receiver pair, so it belongs in config/vehicle_params.yaml next to
// limits.mux_kill_switch_threshold_us once the channel has been scoped (a
// limits.mux_kill_switch_hysteresis_us field). It is NOT added there yet: that file is being
// edited on another branch, and the right value is a measurement (how much this channel
// actually wanders), not a convention. 100 us against the provisional 1500 us threshold and
// a 1000-2000 us range means ARMED at or above 1600 us, KILL below 1400 us, hold in between
// -- see docs/notes/firmware-review-2026-09-14.md's subject-to-change list.
#define RC_SWITCH_DEFAULT_HYSTERESIS_US 100.0

// Interprets the RC receiver's kill-switch channel PWM reading.
//
// `switch_pwm_us` must first be a valid pulse within [signal_min_us, signal_max_us] (the
// receiver's own valid PWM range -- see firmware/safety_mux/README.md's pinout table; this is
// a property of the RC receiver hardware, not a vehicle_params field) or the switch reads as
// RC_SWITCH_SIGNAL_INVALID, which mux_decision.c treats identically to RC_SWITCH_KILL: an
// unreadable kill-switch channel is itself a reason to cut, never a reason to assume ARMED
// (claude-docs/05-safety.md fail-closed).
//
// Within the valid range, with `hysteresis_us` as the half-width of a dead band around
// `kill_threshold_us` (vehicle_params.limits.mux_kill_switch_threshold_us):
//
//   pwm >= threshold + hysteresis        -> ARMED
//   pwm <  threshold - hysteresis        -> KILL
//   otherwise (inside the dead band)     -> hold `previous`
//
// `previous` is the position this function returned on the last cycle; pico/main.c keeps it.
// Holding means the dead band cannot itself CHANGE the position, in either direction, so the
// channel has to travel the full band to flip. A `previous` of RC_SWITCH_SIGNAL_INVALID
// (which is also what pico/main.c seeds it with at power-on) holds as KILL, never as ARMED:
// coming up, or coming back from an unreadable channel, with the switch parked in the dead
// band must not arm the car. A non-finite or negative `hysteresis_us` is treated as zero,
// which degrades to the plain `>= threshold` comparison rather than to anything fail-open.
RcSwitchPosition rc_switch_read(double switch_pwm_us, double kill_threshold_us,
                                double hysteresis_us, double signal_min_us, double signal_max_us,
                                RcSwitchPosition previous);

#ifdef __cplusplus
}
#endif

#endif  // SAFETY_MUX_RC_SWITCH_H_
