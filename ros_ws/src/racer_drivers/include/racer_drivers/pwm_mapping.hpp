// racer_drivers/pwm_mapping.hpp -- ROS-free, sysfs-free command-to-pulse mapping for
// pwm_output_node (claude-docs/10-conventions.md: "Gate/decision logic is always separated
// from node plumbing so it is testable without ROS", claude-docs/12-testing.md L1).
//
// Nothing in this header includes rclcpp, touches a file, or knows what a topic is. Every
// decision the actuator path makes -- clamping, the neutral fallback, the staleness
// decision, the angle/speed to pulse-width mapping -- lives here so it can be table-driven
// gtest-ed at bounds, epsilon over bounds, NaN and inf (claude-docs/12-testing.md L1).
//
// UNITS. Everything crossing this API in SI (radians, m/s, seconds) EXCEPT pulse widths,
// which are microseconds: a raw driver-boundary value, exactly as
// config/vehicle_params.yaml's steering.pwm_*_us / actuation.throttle_pwm_*_us declare them
// (CLAUDE.md invariant 4 -- non-SI is allowed only at a driver boundary and must be noted in
// the schema, which it is). This file is that boundary.
//
// SOURCE OF THE NUMBERS. Not one physical constant is written here (CLAUDE.md invariant 2).
// `MappingConfig` is a plain carrier; pwm_output_node.cpp fills it from the GENERATED
// vehicle_params binding and nowhere else.
#ifndef RACER_DRIVERS_PWM_MAPPING_HPP_
#define RACER_DRIVERS_PWM_MAPPING_HPP_

#include <optional>
#include <string>
#include <vector>

namespace racer_drivers {

/// One servo channel's pulse-width calibration, microseconds.
struct PwmCalibration {
  double min_us{0.0};
  double neutral_us{0.0};
  double max_us{0.0};
};

/// Everything the mapping needs. Built by the node from vehicle_params + declared ROS
/// parameters; never hand-populated with literals outside tests.
struct MappingConfig {
  PwmCalibration steering;
  PwmCalibration throttle;

  /// vehicle_params steering.min_angle_rad / max_angle_rad. Road-wheel angle, LEFT POSITIVE
  /// (claude-docs/06-vehicle-params.md), so min is the right-hand limit (negative) and max
  /// the left-hand limit (positive).
  double steering_min_angle_rad{0.0};
  double steering_max_angle_rad{0.0};

  /// Full-scale speed reference for the provisional open-loop throttle map:
  /// vehicle_params actuation.throttle_full_scale_mps. A command at +this maps to
  /// throttle.max_us, at -this to throttle.min_us. Deliberately NOT
  /// limits.global_speed_cap_mps, which is an f1tenth_gym model-validity bound (20 m/s) and
  /// made a 1 m/s command only 25 us off neutral, likely inside the VESC's PPM deadband
  /// (GitHub issue #40). This is an actuation scale; the safety clamp is speed_cap_mps below.
  double speed_full_scale_mps{0.0};

  /// vehicle_params limits.global_speed_cap_mps. The command magnitude is clamped to this
  /// BEFORE the map is applied, exactly as it was when the cap was also the full scale. With
  /// a full scale below the cap, a command between the two saturates the pulse at the channel
  /// end rather than exceeding it -- the pulse can never leave [throttle.min_us,
  /// throttle.max_us] either way. Keeping the cap here means lowering it in vehicle_params
  /// still tightens this node, not only safety_node.
  double speed_cap_mps{0.0};

  /// Which end of the steering pulse range corresponds to a LEFT (positive) road-wheel
  /// angle. MEASURED on the car 2026-09-21 (docs/notes/bench-session-2026-09-20.md,
  /// docs/notes/build-log.md): a shorter pulse turns the wheels left. Sourced from
  /// config/vehicle_params.yaml's steering.pwm_left_bound through the generated binding
  /// (pwm_output_node.cpp), NOT a declared ROS parameter default -- CLAUDE.md invariant 2
  /// treats a sign convention as a physical constant, and it used to be an unmeasured node
  /// parameter default (`true`) before this measurement existed. Get it backwards and the
  /// car steers the wrong way.
  bool left_is_pwm_max{true};

  /// Seconds since the last /drive message after which the node outputs neutral. Node
  /// tuning (a cadence), not a physical constant -- see pwm_output_node.cpp for why this is
  /// deliberately NOT vehicle_params' limits.mux_watchdog_timeout_s.
  double drive_timeout_s{0.1};
};

/// A required vehicle_params field, paired with the name to blame if it is null.
struct RequiredField {
  std::string name;             ///< dotted vehicle_params path, e.g. "steering.pwm_min_us"
  std::optional<double> value;  ///< std::nullopt == null in config/vehicle_params.yaml
};

/// Refuse-on-null, exactly like firmware/safety_mux's logic/mux_params.c: if any field is
/// std::nullopt, return a message naming the FIRST such field (and every other missing one)
/// so an operator knows what to measure. Returns std::nullopt when all fields are present.
/// Never substitutes a default -- CLAUDE.md invariant 2 and 3.
std::optional<std::string> find_missing_fields(const std::vector<RequiredField>& fields);

/// Structural sanity on an assembled config: finite values, min < neutral < max on both
/// channels, a two-sided steering range (min < 0 < max), a positive speed full scale, a
/// positive speed cap, and a positive timeout. Returns std::nullopt when the config is usable, else
/// the reason. This is a refusal, not a repair: nothing here clamps a bad config into a good one.
std::optional<std::string> validate_config(const MappingConfig& config);

/// The /drive-side state the mapping decides from.
struct CommandState {
  bool has_command{false};  ///< false until the first /drive message arrives
  double age_s{0.0};        ///< seconds since that message; ignored when !has_command
  double steering_angle_rad{0.0};
  double speed_mps{0.0};
};

/// True when the command must be ignored in favour of neutral: no command yet, a
/// non-finite age, a NEGATIVE age, or an age at-or-past the timeout. At exactly the timeout
/// it is stale (fail closed on the boundary).
///
/// A negative age means the clock the caller measured with went backwards. The node measures
/// on RCL_STEADY_TIME and so cannot produce one; this branch is defence in depth against a
/// future caller that measures on a steppable clock, and it is the same fail-closed stance
/// racer_safety's gate_logic takes for a negative `drive_raw_age_s` and racer_tools'
/// `should_use_zero_command` takes for a negative elapsed time. Without it a backwards clock
/// step reads as FRESH and leaves the last commanded pulse on the wire.
bool is_stale(const CommandState& state, double timeout_s);

/// Refuse a steering/throttle channel assignment that puts BOTH pulses on the SAME sysfs PWM
/// channel. Every parameter here defaults to 0 on the node, so a `ros2 run` with no arguments
/// (or a launch file that sets only some of them) silently aims both channels at
/// pwmchip0/pwm0: the throttle write then overwrites the steering write 50 times a second,
/// one output physically does not exist, and on the car that looks like a wiring fault rather
/// than a configuration one. Returns std::nullopt when the assignment is usable, else the
/// reason. Pure so it is L1-testable without sysfs.
std::optional<std::string> validate_channel_assignment(int steering_chip, int steering_channel,
                                                       int throttle_chip, int throttle_channel);

/// Both channels' pulse widths for one cycle, microseconds.
struct PulsePair {
  double steering_us{0.0};
  double throttle_us{0.0};
};

/// Map a road-wheel angle (rad, left positive) to a steering pulse width (us).
/// Non-finite input maps to neutral (fail closed). Finite input is clamped to
/// [steering_min_angle_rad, steering_max_angle_rad] and then interpolated linearly from
/// neutral_us out to whichever pulse end `left_is_pwm_max` says is left. Interpolating each
/// side from neutral separately (rather than one line across the whole range) keeps the
/// zero-angle command exactly on the calibrated neutral even when the calibration is
/// asymmetric, which matters because neutral is the fail-closed output.
double steering_angle_to_pulse_us(const MappingConfig& config, double steering_angle_rad);

/// Map a commanded speed (m/s) to a throttle pulse width (us).
///
/// PROVISIONAL AND OPEN LOOP. The VESC is in PPM mode, where a pulse commands duty or
/// current, NOT speed: there is no feedback here and no claim that commanding X m/s produces
/// X m/s. This is a linear stand-in scaled by actuation.throttle_full_scale_mps so the car
/// can be driven at all on a first boot, to be replaced by the real closed-loop VESC driver
/// (claude-docs/04-architecture.md's vesc_node) when it exists. Non-finite input maps to
/// neutral. Finite input is first clamped to +/- speed_cap_mps
/// (limits.global_speed_cap_mps), then scaled by speed_full_scale_mps; a magnitude between
/// the full scale and the cap saturates at the channel end, and the returned pulse is always
/// inside [throttle.min_us, throttle.max_us].
double speed_to_pulse_us(const MappingConfig& config, double speed_mps);

/// The whole per-cycle decision: stale (or absent, or garbage) command -> neutral on both
/// channels; otherwise the mapped pair. This is the ONLY function the node's timer calls.
PulsePair compute_outputs(const MappingConfig& config, const CommandState& state);

/// Both channels at their calibrated neutral -- the startup output, the timeout output, the
/// exception output and the shutdown output.
PulsePair neutral_outputs(const MappingConfig& config);

/// Pulse width (us) to sysfs duty_cycle (ns), rounded to nearest, never negative and never
/// longer than `period_ns` (a duty longer than the period is rejected by the kernel with
/// EINVAL, which at 50 Hz / 20 ms cannot happen with sane calibration but is clamped rather
/// than trusted).
unsigned long long pulse_us_to_duty_ns(double pulse_us, unsigned long long period_ns);

}  // namespace racer_drivers

#endif  // RACER_DRIVERS_PWM_MAPPING_HPP_
