#include "racer_planner/planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace racer_planner {

namespace {
// The solver's parameters with what only the planner knows filled in.
MppiParams solver_params(const PlannerConfig& c) {
  MppiParams m = c.mppi;
  m.tick_s = c.tick_s;
  // BRIEF 14 (one line): speed floor = dynamics.nominal.speed_deadband_mps + a margin. Until the
  // bundle carries the dead zone the floor is whatever the config says (0.0 by default: off).
  m.speed_floor_mps = std::max(0.0, c.mppi.speed_floor_mps);
  return m;
}
}  // namespace

Planner::Planner(const PlannerConfig& config, const DynamicsParams& dynamics,
                 const VehicleGeometry& geometry)
    : cfg_(config),
      geometry_(geometry),
      model_tick_(dynamics, geometry, config.tick_s),
      ego_(geometry.wheelbase_m, dynamics.nominal.understeer_s2_per_m2),
      solver_(solver_params(config), DynamicsModel(dynamics, geometry, dynamics.dt_s), geometry,
              std::max(config.max_samples, config.mppi.samples)),
      budget_([&] {
        SampleBudgetParams b = config.budget;
        b.max_samples = std::min(
            std::max(std::min(config.max_samples, config.mppi.samples), kMinSamples), kMaxSamples);
        return b;
      }()),
      horizon_(std::max(2, config.mppi.horizon_steps)),
      delay_ticks_(std::min(
          25, std::max(0, static_cast<int>(
                              std::lround(dynamics.nominal.command_delay_s / config.tick_s))))) {
  cfg_.mppi = solver_params(config);
  cfg_.max_samples = std::min(std::max(config.max_samples, kMinSamples), kMaxSamples);
  solver_.set_samples(std::min(cfg_.max_samples, cfg_.mppi.samples));
}

TickOutput Planner::tick(const TickInput& in) {
  try {
    return run(in);
  } catch (...) {
    TickOutput out;
    out.state = PlannerState::Fault;
    out.cmd = Command{0.0, 0.0};
    out.est_speed_mps = est_.v;
    out.est_steering_rad = est_.delta;
    plan_valid_ = false;
    solver_.reset();
    try {
      finish_tick(out.cmd);
    } catch (...) {
    }
    return out;
  }
}

void Planner::finish_tick(const Command& published) {
  const Control sent{published.speed_mps, published.steering_rad};
  Control acting = sent;
  if (delay_ticks_ > 0) {
    acting = ring_[ring_head_];
    ring_[ring_head_] = sent;
    ring_head_ = (ring_head_ + 1) % delay_ticks_;
  }
  const State next = model_tick_.step(est_, acting);
  est_.delta = next.delta;
  est_.v = next.v;  // replaced by the wheel speed at the start of the next tick when fresh
  est_.x = est_.y = est_.yaw = 0.0;
  last_published_ = published;
}

TickOutput Planner::run(const TickInput& in) {
  TickOutput out;
  if (in.has_odom && std::isfinite(in.wheel_speed_mps)) est_.v = std::max(0.0, in.wheel_speed_mps);
  ego_.push(in.now_s, est_.v, est_.delta);

  LogicInputs li;
  li.now_s = in.now_s;
  li.has_odom = in.has_odom && std::isfinite(in.wheel_speed_mps);
  li.odom_stamp_s = in.odom_stamp_s;
  li.has_scan = in.has_scan && in.field != nullptr;
  li.scan_stamp_s = in.scan_stamp_s;
  if (in.track != nullptr) {
    li.track.present = true;
    li.track.stamp_s = in.track->stamp_s;
    li.track.validity = validate_track(*in.track);
    li.track.valid_count = in.track->valid_count;
    li.track.boundary_residual_m = in.track->boundary_residual_m;
    li.track.intrusion_fraction = in.track->intrusion_fraction;
  }
  li.has_guidance = in.has_guidance;
  li.guidance_stamp_s = in.guidance_stamp_s;
  li.guidance_v_cap_mps = in.guidance_v_cap_mps;

  const Gate gate = evaluate_gate(li, cfg_.logic, &hold_);
  const double v_cap = speed_cap(cfg_.logic, li);
  out.v_cap_mps = v_cap;
  out.est_speed_mps = est_.v;
  out.est_steering_rad = est_.delta;
  auto age_or_neg = [&](bool present, double stamp) {
    const double a = present ? age_s(in.now_s, stamp) : -1.0;
    return std::isfinite(a) ? a : -1.0;
  };
  out.track_age_s = age_or_neg(in.track != nullptr, in.track != nullptr ? in.track->stamp_s : 0.0);
  out.scan_age_s = age_or_neg(li.has_scan, in.scan_stamp_s);
  out.samples = solver_.samples();

  PlannerState state = gate.state;
  Command wanted;
  plan_valid_ = false;
  if (gate.may_solve) {
    const Pose2 frame_change = ego_.relative_pose(in.scan_stamp_s, in.track->stamp_s);
    if (!projector_.build(*in.track, frame_change)) {
      state = PlannerState::NoTrack;
    } else {
      ego_pose_ = ego_.relative_pose(in.scan_stamp_s, in.now_s);
      State x0;
      x0.x = ego_pose_.x;
      x0.y = ego_pose_.y;
      x0.yaw = ego_pose_.yaw;
      x0.v = est_.v;
      x0.delta = est_.delta;
      for (int i = 0; i < delay_ticks_; ++i) {
        x0 = model_tick_.step(x0, ring_[(ring_head_ + i) % delay_ticks_]);
      }
      SolveInput si;
      si.x0 = x0;
      si.prev_cmd = Control{last_published_.speed_mps, last_published_.steering_rad};
      si.v_cap_mps = v_cap;
      si.track = &projector_;
      si.field = in.field;
      const auto t0 = std::chrono::steady_clock::now();
      const SolveOutput so = solver_.solve(si);
      out.solve_time_s =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      if (!std::isfinite(so.cmd.speed_cmd) || !std::isfinite(so.cmd.steering_cmd)) {
        throw std::runtime_error("solver produced a non finite command");
      }
      out.samples = so.samples;
      out.best_cost = so.best_cost;
      out.plan_progress_m = so.plan_progress_m;
      out.plan_min_clearance_m = so.plan_min_clearance_m;
      out.collision_fraction = so.collision_fraction;
      state = so.all_collide ? PlannerState::Blocked : PlannerState::Driving;
      wanted = Command{so.cmd.speed_cmd, so.cmd.steering_cmd};
      plan_valid_ = true;
      bool halved = false;
      solver_.set_samples(budget_.update(out.solve_time_s, &halved));
      out.samples_halved = halved;
    }
  }
  if (!plan_valid_) solver_.reset();

  Command c = command_for_state(state, wanted, last_published_.steering_rad, v_cap,
                                geometry_.steering_min_rad, geometry_.steering_max_rad);
  const double floor_mps = cfg_.mppi.speed_floor_mps;
  if (state == PlannerState::Driving) {
    // Low-pass the steering (the state restarts from the last published value on entering
    // Driving) and keep the speed in {0} U [floor, cap].
    if (!steer_lp_valid_) {
      steer_lp_ = last_published_.steering_rad;
      steer_lp_valid_ = true;
    }
    steer_lp_ = lowpass_step(steer_lp_, c.steering_rad, cfg_.tick_s, cfg_.logic.steer_output_tau_s);
    c.steering_rad = steer_lp_;
    c.speed_mps = snap_speed(c.speed_mps, floor_mps, v_cap);
  } else {
    steer_lp_valid_ = false;
  }
  if (state == PlannerState::Driving || state == PlannerState::Blocked) {
    const double f = cfg_.logic.rate_limit_fraction;
    c = rate_limit(c, last_published_, f * geometry_.steering_max_rate_rad_per_s * cfg_.tick_s,
                   f * geometry_.max_acceleration_mps2 * cfg_.tick_s, floor_mps);
  }
  out.state = state;
  out.cmd = c;
  finish_tick(c);
  return out;
}

}  // namespace racer_planner
