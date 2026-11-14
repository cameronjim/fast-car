#include "racer_session_tools/whatif_core.hpp"

#include <limits>
#include <optional>
#include <vehicle_params_generated.hpp>

#include "racer_safety/gate_logic_formatting.hpp"

namespace racer_session_tools {

using racer_safety::EventPhase;
using racer_safety::EventSeverity;
using racer_safety::GateSource;

racer_safety::SafetyLimits default_limits() {
  const auto& vp = VEHICLE_PARAMS;
  racer_safety::SafetyLimits limits;
  limits.steering_min_rad = vp.steering.min_angle_rad;
  limits.steering_max_rad = vp.steering.max_angle_rad;
  limits.steering_rate_min_rad_per_s = vp.steering.min_rate_rad_per_s;
  limits.steering_rate_max_rad_per_s = vp.steering.max_rate_rad_per_s;
  limits.speed_min_mps = vp.limits.min_velocity_mps;
  limits.speed_max_mps = vp.limits.global_speed_cap_mps;
  limits.max_acceleration_mps2 = vp.actuation.max_acceleration_mps2;
  // As safety_node: a threshold at or below zero is "unconfigured".
  if (vp.limits.ttc_brake_s.has_value() && *vp.limits.ttc_brake_s > 0.0) {
    limits.ttc_brake_s = *vp.limits.ttc_brake_s;
  }
  if (vp.limits.ttc_warning_s.has_value() && *vp.limits.ttc_warning_s > 0.0) {
    limits.ttc_warning_s = *vp.limits.ttc_warning_s;
  }
  limits.min_forward_clearance_m = vp.limits.min_forward_clearance_m;
  limits.speed_bound_deceleration_mps2 = vp.actuation.stop_deceleration_mps2;
  racer_safety::MeasuredSpeedModel model;
  model.latency_s = vp.actuation.stop_latency_s;
  model.peak_acceleration_mps2 = vp.actuation.peak_acceleration_mps2;
  limits.measured_speed_model = model;
  limits.obstacle_steering_hold_after_s = vp.limits.obstacle_steering_hold_after_s;
  limits.watchdog_missed_cycles = 3;     // safety_node's declared default
  limits.control_period_s = 1.0 / 50.0;  // safety_node's control_rate_hz default
  return limits;
}

double nominal_period_s(const racer_safety::SafetyLimits& limits) {
  return limits.control_period_s;
}

Replayer::Replayer(const racer_safety::SafetyLimits& limits) : gate_(limits), previous_{0.0, 0.0} {}

void Replayer::seed(const racer_safety::DriveCommand& output) {
  previous_ = output;
  latched_ = false;
  reverse_latched_ = false;
  hold_timer_s_.reset();
  bound_forward_ = 0.0;
  bound_reverse_ = 0.0;
  tracker_ = racer_safety::GateEventTracker();
}

CycleOutput Replayer::step(const CycleInput& in) {
  racer_safety::GateInput input;
  input.command = in.command;
  input.drive_raw_age_s = in.age_s;
  input.dt_s = in.dt_s;
  input.min_scan_range_m = std::numeric_limits<double>::infinity();
  input.min_rear_scan_range_m = std::numeric_limits<double>::infinity();
  input.ttc_brake_latched = latched_;
  input.reverse_brake_latched = reverse_latched_;
  input.obstacle_hold_timer_s = hold_timer_s_;
  input.speed_bound_forward_mps = bound_forward_;
  input.speed_bound_reverse_mps = bound_reverse_;
  input.has_pose_input = false;
  input.pose_covariance_trace = 0.0;
  const racer_safety::GateResult result = gate_.evaluate(input, previous_);

  CycleOutput out;
  out.output = result.output;
  for (const auto& a : result.activations) {
    if (a.source == GateSource::kRateLimit) {
      out.rate_limit_clipped = true;
    }
    if (a.source == GateSource::kWatchdog) {
      out.watchdog_tripped = true;
    }
  }
  for (const racer_safety::SafetyEventRecord& r :
       tracker_.update(result.activations, in.stamp_s, result.releases)) {
    EventOut e;
    e.t_s = in.stamp_s;
    e.engage = r.phase == EventPhase::kEngage;
    e.source = racer_safety::gate_source_to_string(r.source);
    e.severity = r.severity == EventSeverity::kBrake     ? "brake"
                 : r.severity == EventSeverity::kWarning ? "warning"
                                                         : "info";
    e.detail = r.detail;
    e.duration_s = r.duration_s;
    out.events.push_back(std::move(e));
  }
  previous_ = result.output;
  latched_ = result.ttc_brake_latched;
  reverse_latched_ = result.reverse_brake_latched;
  hold_timer_s_ = result.obstacle_hold_timer_s;
  bound_forward_ = result.speed_bound_forward_mps;
  bound_reverse_ = result.speed_bound_reverse_mps;
  return out;
}

}  // namespace racer_session_tools
