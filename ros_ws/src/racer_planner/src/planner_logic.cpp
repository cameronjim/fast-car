#include "racer_planner/planner_logic.hpp"

#include <algorithm>
#include <cmath>

namespace racer_planner {

double age_s(double now_s, double stamp_s) {
  if (!std::isfinite(now_s) || !std::isfinite(stamp_s)) return std::nan("");
  return now_s - stamp_s;
}

bool is_stale(double now_s, double stamp_s, double timeout_s) {
  const double a = age_s(now_s, stamp_s);
  if (!std::isfinite(a)) return true;
  return a > timeout_s || a < -timeout_s;
}

double ramp_down(double x, double full, double zero) {
  if (!std::isfinite(x)) return 0.0;
  if (!(zero > full)) return x <= full ? 1.0 : 0.0;
  if (x <= full) return 1.0;
  if (x >= zero) return 0.0;
  return (zero - x) / (zero - full);
}

double ramp_up(double x, double zero, double full) {
  if (!std::isfinite(x)) return 0.0;
  if (!(full > zero)) return x >= zero ? 1.0 : 0.0;
  if (x <= zero) return 0.0;
  if (x >= full) return 1.0;
  return (x - zero) / (full - zero);
}

double track_confidence_raw(const TrackSummary& t, double now_s, const LogicParams& p) {
  if (!t.present || t.validity != TrackValidity::Ok) return 0.0;
  if (!std::isfinite(t.intrusion_fraction) || !std::isfinite(t.boundary_residual_m)) return 0.0;
  const double age = age_s(now_s, t.stamp_s);
  if (!std::isfinite(age)) return 0.0;
  if (age < -p.conf_age_zero_s) return 0.0;  // a timestamp jump into the future
  const double c = std::min(
      std::min(ramp_down(t.intrusion_fraction, p.conf_intrusion_full, p.conf_intrusion_zero),
               ramp_down(t.boundary_residual_m, p.conf_residual_full_m, p.conf_residual_zero_m)),
      std::min(ramp_up(static_cast<double>(t.valid_count), p.conf_valid_zero, p.conf_valid_full),
               ramp_down(std::max(age, 0.0), p.conf_age_full_s, p.conf_age_zero_s)));
  return std::min(std::max(c, 0.0), 1.0);
}

double track_confidence(double c_raw, double c_prev, double tick_s, double rise_per_s) {
  const double raw = std::isfinite(c_raw) ? std::min(std::max(c_raw, 0.0), 1.0) : 0.0;
  const double prev = std::isfinite(c_prev) ? std::min(std::max(c_prev, 0.0), 1.0) : 0.0;
  const double rise = std::isfinite(tick_s) && std::isfinite(rise_per_s)
                          ? std::max(0.0, rise_per_s) * std::max(0.0, tick_s)
                          : 0.0;
  return std::min(raw, prev + rise);
}

Gate evaluate_gate(const LogicInputs& in, const LogicParams& p) {
  Gate g;
  if (in.internal_error) {
    g.state = PlannerState::Fault;
    return g;
  }
  if (!in.has_odom || is_stale(in.now_s, in.odom_stamp_s, p.odom_timeout_s)) {
    g.state = PlannerState::NoOdom;
    return g;
  }
  if (!in.has_scan || is_stale(in.now_s, in.scan_stamp_s, p.scan_timeout_s)) {
    g.state = PlannerState::NoScan;
    return g;
  }
  g.state = PlannerState::Driving;
  g.may_solve = true;
  return g;
}

double speed_cap(const LogicParams& p, const LogicInputs& in, double confidence) {
  double cap = p.v_max_mps;
  if (p.use_lap_guidance && in.has_guidance && std::isfinite(in.guidance_v_cap_mps) &&
      !is_stale(in.now_s, in.guidance_stamp_s, p.guidance_timeout_s)) {
    cap = std::min(cap, in.guidance_v_cap_mps);
  }
  if (!std::isfinite(cap)) cap = 0.0;
  const double c = std::isfinite(confidence) ? std::min(std::max(confidence, 0.0), 1.0) : 0.0;
  const double f = std::min(std::max(p.freespace_speed_fraction, 0.0), 1.0);
  cap *= f + (1.0 - f) * c;
  return std::min(std::max(0.0, cap), std::max(0.0, p.v_max_mps));
}

Command command_for_state(PlannerState state, const Command& solver_cmd, double last_steering_rad,
                          double v_cap_mps, double steer_min_rad, double steer_max_rad,
                          double reverse_limit_mps) {
  auto clean_steer = [&](double s) {
    if (!std::isfinite(s)) return 0.0;
    return std::min(std::max(s, steer_min_rad), steer_max_rad);
  };
  Command c;
  switch (state) {
    case PlannerState::Driving: {
      const double sp = std::isfinite(solver_cmd.speed_mps) ? solver_cmd.speed_mps : 0.0;
      c.speed_mps = std::min(std::max(sp, 0.0), std::max(0.0, v_cap_mps));
      c.steering_rad = clean_steer(solver_cmd.steering_rad);
      break;
    }
    case PlannerState::Recovering: {
      const double sp = std::isfinite(solver_cmd.speed_mps) ? solver_cmd.speed_mps : 0.0;
      c.speed_mps =
          std::min(std::max(sp, -std::max(0.0, reverse_limit_mps)), std::max(0.0, v_cap_mps));
      c.steering_rad = clean_steer(solver_cmd.steering_rad);
      break;
    }
    case PlannerState::Blocked:
      c.speed_mps = 0.0;
      c.steering_rad = clean_steer(solver_cmd.steering_rad);
      break;
    case PlannerState::NoOdom:
    case PlannerState::NoScan:
    case PlannerState::NoTrack:
    case PlannerState::Starting:
      c.speed_mps = 0.0;
      c.steering_rad = clean_steer(last_steering_rad);
      break;
    case PlannerState::Fault:
    default:
      c.speed_mps = 0.0;
      c.steering_rad = 0.0;
      break;
  }
  return c;
}

double lowpass_step(double previous, double wanted, double tick_s, double tau_s) {
  if (!std::isfinite(wanted)) return previous;
  if (!std::isfinite(previous) || !(tau_s > 0.0) || !(tick_s > 0.0)) return wanted;
  return previous + (wanted - previous) * (1.0 - std::exp(-tick_s / tau_s));
}

Command rate_limit(const Command& wanted, const Command& previous, double max_steer_step_rad,
                   double max_speed_up_step_mps, double speed_floor_mps) {
  Command c;
  c.steering_rad =
      previous.steering_rad +
      std::min(std::max(wanted.steering_rad - previous.steering_rad, -max_steer_step_rad),
               max_steer_step_rad);
  if (wanted.speed_mps < 0.0) {
    // reverse: the magnitude may grow by one step from zero (or from the current reverse speed);
    // coming from forward the brake to zero is free and the reverse grows from zero
    const double base = std::min(previous.speed_mps, 0.0);
    c.speed_mps = std::max(wanted.speed_mps, base - max_speed_up_step_mps);
    return c;
  }
  const double base = std::max(previous.speed_mps, 0.0);  // from reverse: to zero at once
  c.speed_mps = std::min(wanted.speed_mps, base + max_speed_up_step_mps);
  if (speed_floor_mps > 0.0 && max_speed_up_step_mps > 0.0 && base < speed_floor_mps &&
      wanted.speed_mps >= speed_floor_mps) {
    // up to the floor in equal steps, each within the limit
    const double steps = std::ceil(speed_floor_mps / max_speed_up_step_mps - 1e-9);
    c.speed_mps =
        std::min(wanted.speed_mps, std::min(speed_floor_mps, base + speed_floor_mps / steps));
  }
  return c;
}

float float32_within(double x, double lo, double hi) {
  if (!std::isfinite(x)) return 0.0f;
  x = std::min(std::max(x, lo), hi);
  float f = static_cast<float>(x);
  if (static_cast<double>(f) > hi) f = std::nextafter(f, -INFINITY);
  if (static_cast<double>(f) < lo) f = std::nextafter(f, INFINITY);
  return f;
}

BlockReason DrivabilityMonitor::update(const DrivabilityInputs& in, double now_s) {
  now_s_ = now_s;
  const double floor_eps = std::max(in.speed_floor_mps, 1e-3);
  bool now[kReasons];
  now[0] = !in.no_plan && in.all_collide;
  now[1] = !now[0] && (in.no_plan || !(in.plan_speed_mps >= floor_eps));
  now[2] = in.wheel_speed_mps < p_.stall_wheel_speed_mps && in.published_speed_mps >= floor_eps;
  now[3] = in.gate_fresh && in.gated_speed_mps <= 0.0 && in.published_speed_mps > 0.0;
  for (int i = 0; i < kReasons; ++i) {
    if (now[i] && !active_[i]) since_s_[i] = now_s;
    active_[i] = now[i] && std::isfinite(now_s);
  }
  if (active_[0]) return BlockReason::AllCollide;
  if (active_[1]) return BlockReason::BelowSpeedFloor;
  if (active_[2] && held_s(BlockReason::WheelStalled) >= p_.stall_hold_s) {
    return BlockReason::WheelStalled;
  }
  if (active_[3] && held_s(BlockReason::GateRefusing) >= p_.gate_hold_s) {
    return BlockReason::GateRefusing;
  }
  return BlockReason::None;
}

bool DrivabilityMonitor::active(BlockReason r) const {
  const int i = static_cast<int>(r) - 1;
  return i >= 0 && i < kReasons && active_[i];
}

double DrivabilityMonitor::held_s(BlockReason r) const {
  const int i = static_cast<int>(r) - 1;
  if (i < 0 || i >= kReasons || !active_[i]) return 0.0;
  return std::max(0.0, now_s_ - since_s_[i]);
}

void DrivabilityMonitor::reset() {
  for (int i = 0; i < kReasons; ++i) active_[i] = false;
}

SampleBudget::SampleBudget(const SampleBudgetParams& p)
    : p_(p), floor_(std::min(p.min_samples, p.max_samples)), current_(p.max_samples) {}

int SampleBudget::update(double solve_time_s, bool* halved) {
  if (halved != nullptr) *halved = false;
  if (!std::isfinite(solve_time_s)) return current_;
  if (solve_time_s > p_.halve_over_s) {
    fast_ticks_ = 0;
    if (current_ > floor_) {
      current_ = std::max(floor_, current_ / 2);
      if (halved != nullptr) *halved = true;
    }
  } else if (solve_time_s < p_.restore_under_s) {
    if (++fast_ticks_ >= p_.restore_after_ticks && current_ < p_.max_samples) {
      fast_ticks_ = 0;
      current_ = std::min(p_.max_samples, std::max(current_ + 1, current_ * 5 / 4));
    }
  } else {
    fast_ticks_ = 0;
  }
  return current_;
}

}  // namespace racer_planner
