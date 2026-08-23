#include "racer_safety/gate_logic.hpp"

#include <cmath>

#include "racer_safety/gate_logic_formatting.hpp"

namespace racer_safety {

namespace {

// Plain, explicit-branch helpers (deliberately NOT std::clamp/std::min) so every decision
// this file makes is a literal `if` in this translation unit -- the 100% branch-coverage
// gate (claude-docs/12-testing.md) needs to be satisfiable predictably regardless of
// optimization level or how a standard-library template happens to get inlined.

double clamp_value(double value, double lo, double hi) {
  if (value < lo) {
    return lo;
  }
  if (value > hi) {
    return hi;
  }
  return value;
}

bool is_finite_command(const DriveCommand& cmd) {
  if (!std::isfinite(cmd.steering_angle_rad)) {
    return false;
  }
  if (!std::isfinite(cmd.speed_mps)) {
    return false;
  }
  return true;
}

// Garbage/negative timing is the least-permissive (safest) input: no time is treated as
// having elapsed, so no rate-limited change is allowed this cycle, rather than guessing a
// nominal period or letting NaN/Inf propagate into a rate-limit bound.
double safe_dt(double dt_s) {
  if (!std::isfinite(dt_s)) {
    return 0.0;
  }
  if (dt_s < 0.0) {
    return 0.0;
  }
  return dt_s;
}

// A /scan return only counts for TTC if it is a finite, positive distance. Garbage
// (NaN/Inf/non-positive) is treated as "no valid range this cycle" -- ignored, not
// hallucinated into an obstacle -- rather than forcing a brake off of nonsense sensor data.
bool is_valid_range(double range_m) {
  if (!std::isfinite(range_m)) {
    return false;
  }
  if (range_m <= 0.0) {
    return false;
  }
  return true;
}

}  // namespace

CovarianceGateResult evaluate_covariance_gate(bool has_pose_input, double pose_covariance_trace) {
  // TODO(roadmap task 2.6): once /pose (PoseWithCovarianceStamped) exists, derate
  // `speed_fraction` from `pose_covariance_trace` against a tuned threshold
  // (claude-docs/04-architecture.md: "Pose covariance above gate threshold -> safety_node
  // ramps speed cap down"). Until then this is a deliberate no-op stub: `has_pose_input` is
  // always false in the real graph (no /pose publisher exists), and this function must never
  // be the thing that decides whether the OTHER gates in SafetyGateLogic::evaluate() run --
  // see that function's covariance-gate step, which applies this result unconditionally
  // (multiplying by speed_fraction, a no-op at 1.0) rather than branching on it.
  (void)pose_covariance_trace;  // unused until roadmap task 2.6 wires a real derate
  if (!has_pose_input) {
    return CovarianceGateResult{/*speed_fraction=*/1.0, /*engaged=*/false};
  }
  // Reachable only by a caller that explicitly sets has_pose_input=true (no production code
  // path does this yet -- test_gate_logic.cpp exercises it directly to prove the stub, even
  // if "engaged", still fails safe by not inventing a derate it has no data to justify).
  return CovarianceGateResult{/*speed_fraction=*/1.0, /*engaged=*/true};
}

DriveCommand SafetyGateLogic::clamp_to_bounds(const DriveCommand& cmd) const {
  DriveCommand out = cmd;
  out.steering_angle_rad =
      clamp_value(out.steering_angle_rad, limits_.steering_min_rad, limits_.steering_max_rad);
  out.speed_mps = clamp_value(out.speed_mps, limits_.speed_min_mps, limits_.speed_max_mps);
  return out;
}

DriveCommand SafetyGateLogic::rate_limit(const DriveCommand& cmd,
                                         const DriveCommand& previous_output, double dt_s) const {
  DriveCommand out = cmd;

  const double steer_delta = cmd.steering_angle_rad - previous_output.steering_angle_rad;
  const double steer_delta_min = limits_.steering_rate_min_rad_per_s * dt_s;
  const double steer_delta_max = limits_.steering_rate_max_rad_per_s * dt_s;
  const double clamped_steer_delta = clamp_value(steer_delta, steer_delta_min, steer_delta_max);
  out.steering_angle_rad = previous_output.steering_angle_rad + clamped_steer_delta;

  const double speed_delta = cmd.speed_mps - previous_output.speed_mps;
  if (speed_delta > 0.0) {
    // Only accelerating (increasing commanded speed) is rate-limited; decelerating/braking is
    // never rate-limited (claude-docs/05-safety.md never asks for slower braking).
    const double max_speed_delta = limits_.max_acceleration_mps2 * dt_s;
    if (speed_delta > max_speed_delta) {
      out.speed_mps = previous_output.speed_mps + max_speed_delta;
    } else {
      out.speed_mps = previous_output.speed_mps + speed_delta;
    }
  }
  return out;
}

GateResult SafetyGateLogic::evaluate(const GateInput& input,
                                     const DriveCommand& previous_output) const {
  GateResult result;

  // 1. Watchdog (claude-docs/04-architecture.md: "missing /drive_raw for 3 cycles -> brake
  // command") -- short-circuits everything else below; a stale/garbage age means there is no
  // fresh command to reason about further.
  const double watchdog_timeout_s =
      static_cast<double>(limits_.watchdog_missed_cycles) * limits_.control_period_s;
  bool watchdog_tripped = false;
  if (!std::isfinite(input.drive_raw_age_s)) {
    watchdog_tripped = true;
  } else if (input.drive_raw_age_s < 0.0) {
    watchdog_tripped = true;
  } else if (input.drive_raw_age_s >= watchdog_timeout_s) {
    watchdog_tripped = true;
  }
  if (watchdog_tripped) {
    result.output = DriveCommand{0.0, 0.0};
    result.brake = true;
    result.events.push_back(
        SafetyEvent{GateSource::kWatchdog, EventSeverity::kBrake,
                    formatting::watchdog_detail(watchdog_timeout_s, input.drive_raw_age_s)});
    return result;
  }

  // 2. Command sanity: non-finite (NaN/Inf) input is garbage, not a bounds violation -- it
  // also short-circuits to a hard brake rather than being clamped into something that looks
  // sane and passed through (claude-docs/05-safety.md fail-closed).
  if (!is_finite_command(input.command)) {
    result.output = DriveCommand{0.0, 0.0};
    result.brake = true;
    result.events.push_back(SafetyEvent{GateSource::kCommandSanity, EventSeverity::kBrake,
                                        formatting::command_sanity_detail()});
    return result;
  }

  // 3a. Absolute bounds clamp against vehicle_params limits.
  DriveCommand cmd = clamp_to_bounds(input.command);
  bool bounds_clamped = false;
  if (cmd.steering_angle_rad != input.command.steering_angle_rad) {
    bounds_clamped = true;
  } else if (cmd.speed_mps != input.command.speed_mps) {
    bounds_clamped = true;
  }
  if (bounds_clamped) {
    result.events.push_back(SafetyEvent{
        GateSource::kBoundsClamp, EventSeverity::kWarning,
        formatting::bounds_clamp_detail(limits_.steering_min_rad, limits_.steering_max_rad,
                                        limits_.speed_min_mps, limits_.speed_max_mps)});
  }

  // 3b. Rate-limit clamp relative to the previous OUTPUT (what was actually commanded last
  // cycle, not what was requested), then re-clamp to absolute bounds defensively.
  const double dt_s = safe_dt(input.dt_s);
  DriveCommand rate_limited = rate_limit(cmd, previous_output, dt_s);
  rate_limited = clamp_to_bounds(rate_limited);
  bool rate_clamped = false;
  if (rate_limited.steering_angle_rad != cmd.steering_angle_rad) {
    rate_clamped = true;
  } else if (rate_limited.speed_mps != cmd.speed_mps) {
    rate_clamped = true;
  }
  if (rate_clamped) {
    result.events.push_back(SafetyEvent{GateSource::kRateLimit, EventSeverity::kWarning,
                                        formatting::rate_limit_detail(dt_s)});
  }
  cmd = rate_limited;

  // 3c. TTC gate (claude-docs/05-safety.md: "TTC braking from /scan"). Uses the
  // already-clamped/rate-limited output speed as the forward-speed estimate: no state
  // estimator (/odom) feeds safety_node in this milestone (the EKF is roadmap phase 2), so
  // the commanded speed is the best available proxy for how fast the vehicle is about to be
  // told to go. Reversing/stopped (speed <= ~0) is never TTC-braked -- not moving toward an
  // obstacle is not what TTC protects against. `limits_.ttc_brake_s`/`ttc_warning_s` are
  // `std::optional` because config/vehicle_params.yaml's limits.ttc_brake_s/ttc_warning_s
  // are currently null (not yet tuned) -- an unconfigured TTC gate is a documented no-op,
  // not an invented threshold (CLAUDE.md invariant 2).
  const double forward_speed_mps = cmd.speed_mps > 0.0 ? cmd.speed_mps : 0.0;
  if (!limits_.ttc_brake_s.has_value()) {
    // TTC gate not configured; no-op.
  } else if (!is_valid_range(input.min_scan_range_m)) {
    // No valid /scan return this cycle; no-op.
  } else if (forward_speed_mps <= 1e-6) {
    // Not moving forward; no-op.
  } else {
    const double ttc_s = input.min_scan_range_m / forward_speed_mps;
    if (ttc_s <= *limits_.ttc_brake_s) {
      cmd.speed_mps = 0.0;
      result.brake = true;
      result.events.push_back(
          SafetyEvent{GateSource::kTtc, EventSeverity::kBrake,
                      formatting::ttc_brake_detail(ttc_s, *limits_.ttc_brake_s)});
    } else {
      bool in_warning_zone = false;
      if (limits_.ttc_warning_s.has_value()) {
        if (ttc_s <= *limits_.ttc_warning_s) {
          in_warning_zone = true;
        }
      }
      if (in_warning_zone) {
        result.events.push_back(
            SafetyEvent{GateSource::kTtc, EventSeverity::kInfo,
                        formatting::ttc_warning_detail(ttc_s, *limits_.ttc_warning_s)});
      }
    }
  }

  // 4. Covariance gate stub (roadmap task 2.6). Evaluated unconditionally and applied
  // unconditionally (multiplying by speed_fraction, a no-op at the stub's fixed 1.0) --
  // per this milestone's instructions, the stub must fail SAFE, meaning the ABSENCE of a
  // real /pose source must never disable the watchdog/sanity/bounds/rate/TTC gates above,
  // which have already run unconditionally by this point regardless of this step's outcome.
  const CovarianceGateResult covariance =
      evaluate_covariance_gate(input.has_pose_input, input.pose_covariance_trace);
  cmd.speed_mps *= covariance.speed_fraction;
  if (covariance.engaged) {
    result.events.push_back(SafetyEvent{GateSource::kCovariance, EventSeverity::kWarning,
                                        formatting::covariance_detail(covariance.speed_fraction)});
  }

  result.output = cmd;
  return result;
}

}  // namespace racer_safety
