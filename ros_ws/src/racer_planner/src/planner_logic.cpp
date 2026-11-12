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

bool selfcheck_over_limits(double boundary_residual_m, double intrusion_fraction,
                           const LogicParams& p) {
  if (!std::isfinite(boundary_residual_m) || !std::isfinite(intrusion_fraction)) return true;
  return boundary_residual_m > p.max_boundary_residual_m ||
         intrusion_fraction > p.max_intrusion_fraction;
}

bool SelfCheckHold::update(bool over, double now_s, double hold_s) {
  if (!over || !std::isfinite(now_s)) {
    over_since_valid_ = false;
    return false;
  }
  if (!over_since_valid_) {
    over_since_valid_ = true;
    over_since_s_ = now_s;
  }
  return (now_s - over_since_s_) >= hold_s;
}

Gate evaluate_gate(const LogicInputs& in, const LogicParams& p, SelfCheckHold* hold) {
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
  const TrackSummary& t = in.track;
  bool no_track = !t.present || is_stale(in.now_s, t.stamp_s, p.track_timeout_s) ||
                  t.validity != TrackValidity::Ok || t.valid_count < p.min_valid_stations;
  // The self-check is evaluated every tick a track is usable, so the hold timer sees continuous
  // time; an unusable track resets it because the NO_TRACK state already covers that.
  if (no_track) {
    if (hold != nullptr) hold->reset();
  } else if (hold != nullptr) {
    const bool over = selfcheck_over_limits(t.boundary_residual_m, t.intrusion_fraction, p);
    if (hold->update(over, in.now_s, p.selfcheck_hold_s)) no_track = true;
  }
  if (no_track) {
    g.state = PlannerState::NoTrack;
    return g;
  }
  g.state = PlannerState::Driving;
  g.may_solve = true;
  return g;
}

double speed_cap(const LogicParams& p, const LogicInputs& in) {
  double cap = p.v_max_mps;
  if (p.use_lap_guidance && in.has_guidance && std::isfinite(in.guidance_v_cap_mps) &&
      !is_stale(in.now_s, in.guidance_stamp_s, p.guidance_timeout_s)) {
    cap = std::min(cap, in.guidance_v_cap_mps);
  }
  if (!std::isfinite(cap)) cap = 0.0;
  return std::max(0.0, cap);
}

Command command_for_state(PlannerState state, const Command& solver_cmd, double last_steering_rad,
                          double v_cap_mps, double steer_min_rad, double steer_max_rad) {
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
  c.speed_mps = std::min(wanted.speed_mps, previous.speed_mps + max_speed_up_step_mps);
  if (speed_floor_mps > 0.0 && c.speed_mps > 0.0 && c.speed_mps < speed_floor_mps &&
      wanted.speed_mps >= speed_floor_mps) {
    c.speed_mps = speed_floor_mps;
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
