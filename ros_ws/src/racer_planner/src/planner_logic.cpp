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

Age classify_age(double now_s, double stamp_s, double timeout_s, double hard_timeout_s) {
  const double a = age_s(now_s, stamp_s);
  if (!std::isfinite(a)) return Age::Hard;
  if (a < -timeout_s) return Age::Hard;  // a timestamp jump into the future
  if (a <= timeout_s) return Age::Fresh;
  if (a <= std::max(hard_timeout_s, timeout_s)) return Age::Soft;
  return Age::Hard;
}

Gate evaluate_gate(const LogicInputs& in, const LogicParams& p) {
  Gate g;
  if (in.internal_error) {
    g.state = PlannerState::Fault;
    return g;
  }
  if (!in.has_odom || classify_age(in.now_s, in.odom_stamp_s, p.odom_timeout_s,
                                   p.odom_hard_timeout_s) == Age::Hard) {
    g.state = PlannerState::NoOdom;
    return g;
  }
  if (!in.has_scan || classify_age(in.now_s, in.scan_stamp_s, p.scan_timeout_s,
                                   p.scan_hard_timeout_s) == Age::Hard) {
    g.state = PlannerState::NoScan;
    return g;
  }
  g.state = PlannerState::Driving;
  g.may_solve = true;
  return g;
}

double speed_cap(const LogicParams& p, const LogicInputs& in, double confidence) {
  double cap = p.v_max_mps;
  if (p.use_lap_guidance) {
    const bool guidance_ok = in.has_guidance && std::isfinite(in.guidance_v_cap_mps) &&
                             !in.guidance_profile_bad &&
                             !is_stale(in.now_s, in.guidance_stamp_s, p.guidance_timeout_s);
    if (guidance_ok) {
      cap = std::min(cap, in.guidance_v_cap_mps);
    } else {
      // NaN is not a limit anyone meant: it counts as 0.
      const double fallback = std::isnan(p.v_no_guidance_mps) ? 0.0 : p.v_no_guidance_mps;
      cap = std::min(cap, fallback);
    }
  }
  if (!std::isfinite(cap)) cap = 0.0;
  const double c = std::isfinite(confidence) ? std::min(std::max(confidence, 0.0), 1.0) : 0.0;
  const double f = std::min(std::max(p.freespace_speed_fraction, 0.0), 1.0);
  cap *= f + (1.0 - f) * c;
  // Scan staleness: full up to scan_timeout_s, a linear ramp to 0 at scan_hard_timeout_s.
  if (!in.has_scan) {
    cap = 0.0;
  } else {
    switch (classify_age(in.now_s, in.scan_stamp_s, p.scan_timeout_s, p.scan_hard_timeout_s)) {
      case Age::Fresh:
        break;
      case Age::Soft:
        cap *= ramp_down(age_s(in.now_s, in.scan_stamp_s), p.scan_timeout_s, p.scan_hard_timeout_s);
        break;
      case Age::Hard:
        cap = 0.0;
        break;
    }
  }
  // Wheel speed staleness: no acceleration on a guess.
  if (!in.has_odom) {
    cap = 0.0;
  } else {
    switch (classify_age(in.now_s, in.odom_stamp_s, p.odom_timeout_s, p.odom_hard_timeout_s)) {
      case Age::Fresh:
        break;
      case Age::Soft: {
        const double last =
            std::isfinite(in.wheel_speed_mps) ? std::max(in.wheel_speed_mps, 0.0) : 0.0;
        cap = std::min(cap, last + std::max(p.odom_stale_cap_margin_mps, 0.0));
        break;
      }
      case Age::Hard:
        cap = 0.0;
        break;
    }
  }
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

double slew_elapsed_s(double elapsed_s, double tick_s) {
  if (!(elapsed_s > 0.0)) return 0.0;  // negative, zero and NaN
  return std::min(elapsed_s, std::max(tick_s, 0.0));
}

Command slew_limit(const Command& wanted, const Command& previous, double elapsed_s,
                   const SlewLimits& limits) {
  const double e = slew_elapsed_s(elapsed_s, limits.tick_s);
  const double steer_step = limits.fraction * limits.steering_rate_rad_per_s * e;
  const double up_step = limits.fraction * limits.max_acceleration_mps2 * e;
  Command c = rate_limit(wanted, previous, steer_step, up_step, 0.0);
  const double nominal_up = limits.fraction * limits.max_acceleration_mps2 * limits.tick_s;
  const double base = std::max(previous.speed_mps, 0.0);
  if (limits.speed_floor_mps > 0.0 && nominal_up > 0.0 && e > 0.0 && wanted.speed_mps >= 0.0 &&
      base < limits.speed_floor_mps && wanted.speed_mps >= limits.speed_floor_mps) {
    // The start from rest: equal steps of the on-time ramp, scaled by the elapsed share of a tick.
    const double steps = std::ceil(limits.speed_floor_mps / nominal_up - 1e-9);
    const double step = limits.speed_floor_mps / steps * (e / limits.tick_s);
    c.speed_mps = std::min(wanted.speed_mps, std::min(limits.speed_floor_mps, base + step));
  }
  return c;
}

Control interpolate_plan(const Control* seq, int n, double dt_s, double age_s) {
  if (seq == nullptr || n <= 0) return Control{};
  if (n == 1 || !(age_s > 0.0) || !(dt_s > 0.0)) return seq[0];
  const double pos = age_s / dt_s;
  if (!(pos < static_cast<double>(n - 1))) return seq[n - 1];
  const int k = static_cast<int>(pos);
  const double w = pos - static_cast<double>(k);
  Control c;
  c.speed_cmd = (1.0 - w) * seq[k].speed_cmd + w * seq[k + 1].speed_cmd;
  c.steering_cmd = (1.0 - w) * seq[k].steering_cmd + w * seq[k + 1].steering_cmd;
  return c;
}

const char* start_missing_name(StartMissing m) {
  switch (m) {
    case StartMissing::None:
      return "none";
    case StartMissing::NoPlan:
      return "no fresh plan snapshot (the solver has not produced one in time)";
    case StartMissing::Scan:
      return "scan not fresh";
    case StartMissing::Odom:
      return "wheel speed not fresh";
    case StartMissing::Track:
      return "local track not fresh";
    case StartMissing::TickTiming:
      return "publish ticks not arriving on time (need consecutive ticks within the gap limit)";
    case StartMissing::SolverWarmup:
      return "solver not yet warm on the real inputs (need consecutive on-time solves)";
    case StartMissing::Gated:
      return "gated /drive not seen (safety_node not alive?)";
  }
  return "unknown";
}

StartGateResult start_gate(const StartGateInputs& in, const LogicParams& p) {
  StartGateResult r;
  if (!in.plan_fresh) {
    r.missing = StartMissing::NoPlan;
  } else if (!in.scan_fresh) {
    r.missing = StartMissing::Scan;
  } else if (!in.odom_fresh) {
    r.missing = StartMissing::Odom;
  } else if (in.track_ever && !in.track_fresh) {
    r.missing = StartMissing::Track;
  } else if (in.publish_good_ticks < p.start_ticks) {
    r.missing = StartMissing::TickTiming;
  } else if (in.solve_good_ticks < p.start_ticks) {
    r.missing = StartMissing::SolverWarmup;
  } else if (p.require_gated_for_start && !in.gated_seen) {
    r.missing = StartMissing::Gated;
  }
  r.leave = r.missing == StartMissing::None;
  return r;
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
