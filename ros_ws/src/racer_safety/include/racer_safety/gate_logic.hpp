// racer_safety gate/decision logic (roadmap milestone 1, claude-docs/05-safety.md layer 3).
//
// ROS-free (no rclcpp) so it is gtest-unit-testable with no ROS install
// (claude-docs/12-testing.md L1: "racer_safety gate logic (separated from node plumbing
// precisely so it is testable without ROS)"; claude-docs/10-conventions.md: "Gate/decision
// logic is always separated from node plumbing"). `safety_node` (src/safety_node.cpp) owns
// all ROS plumbing (subscribing /drive_raw and /scan, publishing /drive and
// /safety/events) and calls `SafetyGateLogic::evaluate` once per cycle; nothing here
// allocates on the heap (claude-docs/10-conventions.md: "No heap allocation in the 50 Hz
// control path after init") except for the (small, bounded) `events` vector returned per
// cycle, which only grows when an intervention actually fires.
//
// Design (see also this file's neighboring test/test_gate_logic.cpp for the full
// table-driven pass/marginal/fail/garbage matrix):
//
//   1. Watchdog: if `drive_raw_age_s` has exceeded `watchdog_missed_cycles *
//      control_period_s` (claude-docs/04-architecture.md: "Watchdog: missing /drive_raw for
//      3 cycles -> brake command"), the cycle short-circuits to a hard brake ({0, 0})
//      immediately -- there is no valid fresh command to reason about further.
//   2. Command sanity: a non-finite (NaN/Inf) steering angle or speed in the incoming
//      command is garbage, not a bounds violation -- it short-circuits to a hard brake the
//      same way (claude-docs/05-safety.md: "Fails CLOSED: any internal error -> brake
//      command, not passthrough" -- garbage input is exactly the kind of thing that must not
//      be clamped into something that LOOKS sane and get passed through).
//   3. Otherwise: absolute bounds clamp (steering angle, speed) against vehicle_params
//      limits, then a rate-limit clamp against the PREVIOUS cycle's output (steering rate,
//      and speed increase only -- braking/decelerating is never rate-limited), then the TTC
//      gate (claude-docs/05-safety.md: "TTC braking from /scan"), which can override the
//      (already-clamped) output speed to zero on a hard TTC-brake threshold or emit an
//      advisory-only warning event in the warning zone with no command change.
//   4. Covariance gate: STUB. No /pose source exists yet (roadmap task 2.6); see
//      `evaluate_covariance_gate` below. Per this milestone's instructions, the stub must
//      fail SAFE, meaning absent pose input must not disable any of the other gates above --
//      it is evaluated independently and never gates whether steps 1-3 run (see
//      test_gate_logic.cpp's "covariance stub does not disable other gates" cases).
//
// Every field that would naturally come from `config/vehicle_params.yaml` is threaded in
// through `SafetyLimits`, populated ONLY from the generated vehicle_params C++ binding by
// safety_node.cpp (CLAUDE.md invariant 2: never hand-write a physical constant) -- this
// header and its .cpp never include the generated binding themselves, exactly like
// racer_control's core/tracker_node split.
#ifndef RACER_SAFETY_GATE_LOGIC_HPP_
#define RACER_SAFETY_GATE_LOGIC_HPP_

#include <optional>
#include <string>
#include <vector>

namespace racer_safety {

// Bounds and tuning thresholds the gate logic enforces. Every physical bound here comes
// from the generated vehicle_params binding (never hand-typed) except `watchdog_missed_cycles`
// and `control_period_s`, which are node-level tuning (the watchdog cadence), not vehicle
// physics. `ttc_warning_s`/`ttc_brake_s` are `std::optional` because
// config/vehicle_params.yaml's `limits.ttc_warning_s`/`limits.ttc_brake_s` are currently
// `null` (not yet tuned -- see that file's comments); when unset, the TTC gate is a documented
// no-op (see evaluate()'s doc comment) rather than inventing an untuned threshold.
struct SafetyLimits {
  double steering_min_rad = 0.0;
  double steering_max_rad = 0.0;
  double steering_rate_min_rad_per_s = 0.0;
  double steering_rate_max_rad_per_s = 0.0;
  double speed_min_mps = 0.0;          // reverse cap, vehicle_params limits.min_velocity_mps (<= 0)
  double speed_max_mps = 0.0;          // vehicle_params limits.global_speed_cap_mps
  double max_acceleration_mps2 = 0.0;  // vehicle_params actuation.max_acceleration_mps2
  std::optional<double> ttc_warning_s;
  std::optional<double> ttc_brake_s;
  int watchdog_missed_cycles = 3;
  double control_period_s = 0.02;  // 1 / 50 Hz, claude-docs/04-architecture.md
};

struct DriveCommand {
  double steering_angle_rad = 0.0;
  double speed_mps = 0.0;
};

// Which gate raised a given SafetyEvent -- mirrors racer_msgs/SafetyEvent.msg's `source`
// string field one-to-one (see gate_source_to_string below).
enum class GateSource {
  kWatchdog,
  kCommandSanity,
  kBoundsClamp,
  kRateLimit,
  kTtc,
  kCovariance,
  kInternalFault,
};

enum class EventSeverity {
  kInfo,
  kWarning,
  kBrake,
};

struct SafetyEvent {
  GateSource source;
  EventSeverity severity;
  std::string detail;
};

// Everything the gate needs to know about this cycle. All fields are read defensively:
// non-finite/out-of-range sensor or timing values never propagate NaN/Inf into the output
// command or throw -- see evaluate()'s handling of each (test_gate_logic.cpp's "garbage
// input" cases cover every one).
struct GateInput {
  DriveCommand command;           // latest received /drive_raw (or the last cached one, if stale)
  double drive_raw_age_s = 0.0;   // seconds since /drive_raw was last received
  double dt_s = 0.0;              // seconds since evaluate() was last called (for rate limits)
  double min_scan_range_m = 0.0;  // nearest valid /scan return; +inf if none this cycle
  // Covariance gate stub inputs (TODO roadmap task 2.6: wire from /pose once it exists).
  // `has_pose_input` is always false until then; `pose_covariance_trace` is unused while it
  // is.
  bool has_pose_input = false;
  double pose_covariance_trace = 0.0;
};

struct GateResult {
  DriveCommand output;
  bool brake = false;  // true iff output is a hard brake ({0, 0})
  std::vector<SafetyEvent> events;
};

// Covariance gate stub (roadmap task 2.6: no /pose source exists yet). Returns the speed-cap
// fraction to apply (1.0 = no derate) and whether the gate is even meaningfully engaged.
// With `has_pose_input == false` this ALWAYS returns {fraction: 1.0, engaged: false} --
// deliberately a no-op, not a guess -- and `evaluate()` never lets this stub's result gate
// whether the watchdog/sanity/bounds/rate/TTC checks above run (fail SAFE: absent pose input
// disables only the covariance derate itself, never the rest of the pipeline).
struct CovarianceGateResult {
  double speed_fraction = 1.0;
  bool engaged = false;
};

CovarianceGateResult evaluate_covariance_gate(bool has_pose_input, double pose_covariance_trace);

std::string gate_source_to_string(GateSource source);

class SafetyGateLogic {
 public:
  explicit SafetyGateLogic(SafetyLimits limits) : limits_(limits) {}

  // `previous_output` is the DriveCommand this same evaluate() call chain produced last
  // cycle (rate limits are relative to what was actually COMMANDED, not to what was
  // requested), seeded to {0, 0} by the node at startup.
  GateResult evaluate(const GateInput& input, const DriveCommand& previous_output) const;

 private:
  DriveCommand clamp_to_bounds(const DriveCommand& cmd) const;
  DriveCommand rate_limit(const DriveCommand& cmd, const DriveCommand& previous_output,
                          double dt_s) const;

  SafetyLimits limits_;
};

}  // namespace racer_safety

#endif  // RACER_SAFETY_GATE_LOGIC_HPP_
