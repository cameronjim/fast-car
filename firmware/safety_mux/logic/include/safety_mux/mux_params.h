// Physical-constant loading and null-checking for the mux decision logic -- no Pico SDK, no
// hardware access, host-testable.
//
// This header deliberately does NOT depend on tools/gen_params.py's generated
// vehicle_params_generated.h (this whole logic/ tree has zero codegen dependency, so it
// builds and tests with plain gcc, no generation step -- see
// firmware/safety_mux/README.md). firmware/safety_mux/pico/main.c is the one translation
// unit that bridges the two: it populates a RawMuxParamFields below from
// VEHICLE_PARAMS.steering / .actuation / .limits (the generated C binding,
// CLAUDE.md invariant 2: never hand-write a physical constant) and calls
// mux_params_from_raw().
#ifndef SAFETY_MUX_MUX_PARAMS_H_
#define SAFETY_MUX_MUX_PARAMS_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Concrete (all-present) physical constants the mux decision logic needs.
typedef struct {
  double steering_pwm_min_us;
  double steering_pwm_max_us;
  double steering_pwm_neutral_us;
  double throttle_pwm_min_us;
  double throttle_pwm_max_us;
  double throttle_pwm_neutral_us;
  double watchdog_timeout_s;
  double kill_switch_threshold_us;
  // Half-width of the kill switch's ARM/KILL dead band -- see rc_switch.h's
  // RC_SWITCH_DEFAULT_HYSTERESIS_US. Not a vehicle_params field YET (see that comment); it
  // flows through RawMuxParamFields anyway so it is checked, named, and refused on exactly
  // the same footing as everything else here the day it becomes one.
  double kill_switch_hysteresis_us;
  // RC receiver's own valid PWM signal range for the kill-switch channel. Not a
  // vehicle_params field (it describes the RC receiver/transmitter pair, not the vehicle) --
  // see firmware/safety_mux/README.md's pinout table for where this is meant to come from
  // (the receiver's datasheet, bench-verified before flight).
  double rc_signal_min_us;
  double rc_signal_max_us;
} MuxParams;

// One raw (possibly-unset) field, mirroring a nullable vehicle_params leaf's `<field>` +
// `<field>_is_set` pair the way tools/gen_params.py's C emitter generates it
// (config/vehicle_params.schema.json, claude-docs/06-vehicle-params.md rule 3).
typedef struct {
  double value;
  bool is_set;
} RawMuxField;

typedef struct {
  RawMuxField steering_pwm_min_us;
  RawMuxField steering_pwm_max_us;
  RawMuxField steering_pwm_neutral_us;
  RawMuxField throttle_pwm_min_us;
  RawMuxField throttle_pwm_max_us;
  RawMuxField throttle_pwm_neutral_us;
  RawMuxField watchdog_timeout_s;
  RawMuxField kill_switch_threshold_us;
  RawMuxField kill_switch_hysteresis_us;
  RawMuxField rc_signal_min_us;
  RawMuxField rc_signal_max_us;
} RawMuxParamFields;

// Why a set of params was refused. `field` names which one to go and look at.
typedef enum {
  MUX_PARAM_PROBLEM_NONE = 0,
  MUX_PARAM_PROBLEM_MISSING,     // still null in config/vehicle_params.yaml
  MUX_PARAM_PROBLEM_NOT_FINITE,  // set, but NaN or +-Inf
  MUX_PARAM_PROBLEM_RANGE,       // set and finite, but not a usable value or ordering
} MuxParamProblem;

typedef struct {
  bool ok;
  MuxParams params;           // valid only if ok == true
  const char* missing_field;  // valid only if ok == false; a static string literal, never freed
  MuxParamProblem problem;    // MUX_PARAM_PROBLEM_NONE iff ok
} MuxParamsResult;

// Checks that every field in `raw` is set, finite, and structurally usable. A field being
// null is legitimate elsewhere in this project pre-hardware (config/vehicle_params.yaml has
// plenty of nulls for genuinely unmeasured values) -- but THIS firmware can never safely
// assume a default for an unmeasured PWM range or an untuned watchdog timeout (that is
// exactly the kind of silent-default CLAUDE.md invariant 2 exists to prevent: "never
// hand-write ... a unit conversion, or sign convention"; an invented PWM bound is the same
// class of defect).
//
// Reports the FIRST problem found, in the struct's declared order for missing/non-finite
// fields and then in the order the structural checks are written, so pico/main.c's startup
// failure can name exactly what is wrong rather than refusing generically.
//
// The structural checks exist because the params decide the CUT output, not just what is
// accepted: `min <= neutral <= max` on both channels is what makes "a cut drives neutral"
// mean "a cut drives something the channel can actually express", and a non-positive
// watchdog timeout or an inverted receiver range is a config that cannot produce a safe
// decision at all. This is a refusal, not a repair: nothing here clamps a bad value into a
// good one.
MuxParamsResult mux_params_from_raw(RawMuxParamFields raw);

#ifdef __cplusplus
}
#endif

#endif  // SAFETY_MUX_MUX_PARAMS_H_
