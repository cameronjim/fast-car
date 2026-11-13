#include "racer_planner/planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace racer_planner {

namespace {
// The solver's parameters with what only the planner knows filled in: the control tick for the
// warm start and the speed floor, the bundle's speed dead zone plus a margin.
MppiParams solver_params(const PlannerConfig& c, const DynamicsParams& dynamics) {
  MppiParams m = c.mppi;
  m.tick_s = c.tick_s;
  m.speed_floor_mps = std::max(0.0, dynamics.nominal.speed_deadband_mps) +
                      std::max(0.0, c.mppi.speed_floor_margin_mps);
  return m;
}
}  // namespace

Planner::Planner(const PlannerConfig& config, const DynamicsParams& dynamics,
                 const VehicleGeometry& geometry)
    : cfg_(config),
      geometry_(geometry),
      model_tick_(dynamics, geometry, config.tick_s),
      ego_(geometry.wheelbase_m, dynamics.nominal.understeer_s2_per_m2),
      solver_(solver_params(config, dynamics), DynamicsModel(dynamics, geometry, dynamics.dt_s),
              geometry, std::max(config.max_samples, config.mppi.samples)),
      budget_([&] {
        SampleBudgetParams b = config.budget;
        b.max_samples = std::min(
            std::max(std::min(config.max_samples, config.mppi.samples), kMinSamples), kMaxSamples);
        return b;
      }()),
      drivability_(config.drivability),
      recovery_(
          [&] {
            RecoveryParams r = config.recovery;
            r.hold_stall_s = config.drivability.stall_hold_s;
            r.hold_gate_s = config.drivability.gate_hold_s;
            r.d_hard_m = config.mppi.d_hard_m + r.recover_margin_m;
            r.corner_radius_m = config.mppi.corner_radius_m;
            return r;
          }(),
          geometry, solver_params(config, dynamics).speed_floor_mps),
      horizon_(std::max(2, config.mppi.horizon_steps)),
      delay_ticks_(std::min(
          25, std::max(0, static_cast<int>(
                              std::lround(dynamics.nominal.command_delay_s / config.tick_s))))) {
  cfg_.mppi = solver_params(config, dynamics);
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
    recovery_.reset();
    recovery_was_active_ = false;
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
  // After a stall long enough for safety_node's watchdog to brake, its output is 0: restart the
  // speed ramp from there (a start from rest) rather than step from the old speed.
  if (std::isfinite(in.now_s)) {
    if (have_last_tick_ && cfg_.logic.gap_reset_s > 0.0 &&
        in.now_s - last_tick_s_ > cfg_.logic.gap_reset_s) {
      last_published_.speed_mps = 0.0;
      out.gap_reset = true;
    }
    last_tick_s_ = in.now_s;
    have_last_tick_ = true;
  }
  if (in.has_odom && std::isfinite(in.wheel_speed_mps)) est_.v = std::max(0.0, in.wheel_speed_mps);
  // The dead reckoning takes the signed wheel speed (the car backs up while recovering).
  ego_.push(in.now_s,
            in.has_odom && std::isfinite(in.wheel_speed_mps) ? in.wheel_speed_mps : est_.v,
            est_.delta);

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

  const Gate gate = evaluate_gate(li, cfg_.logic);
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
  wanted.steering_rad = last_published_.steering_rad;
  BlockReason reason = BlockReason::None;
  plan_valid_ = false;
  const double floor_mps = cfg_.mppi.speed_floor_mps;
  bool plan_moves = false;
  if (gate.may_solve) {
    // Track confidence: how far the track prior is trusted. A track that cannot be projected
    // counts as none.
    double c_raw = track_confidence_raw(li.track, in.now_s, cfg_.logic);
    bool track_ok = false;
    const Pose2 frame_change =
        in.track != nullptr ? ego_.relative_pose(in.scan_stamp_s, in.track->stamp_s) : Pose2{};
    if (c_raw > 0.0 && in.track != nullptr) track_ok = projector_.build(*in.track, frame_change);
    if (!track_ok) c_raw = 0.0;
    confidence_ = track_confidence(c_raw, confidence_, cfg_.tick_s, cfg_.logic.conf_rise_per_s);
    if (!track_ok) confidence_ = 0.0;
    const bool has_free = in.freespace != nullptr && in.freespace->valid();
    ego_pose_ = ego_.relative_pose(in.scan_stamp_s, in.now_s);
    if (has_free) {
      const double cy = std::cos(-ego_pose_.yaw), sy = std::sin(-ego_pose_.yaw);
      const double dx = in.freespace->goal().x - ego_pose_.x;
      const double dy = in.freespace->goal().y - ego_pose_.y;
      out.has_freespace_goal = true;
      out.freespace_goal_x_m = cy * dx - sy * dy;
      out.freespace_goal_y_m = sy * dx + cy * dy;
      out.freespace_reach_m = in.freespace->reach_m();
    }
    const double v_cap = speed_cap(cfg_.logic, li, confidence_);
    out.v_cap_mps = v_cap;
    DrivabilityInputs di;
    di.speed_floor_mps = floor_mps;
    di.wheel_speed_mps = est_.v;
    di.published_speed_mps = last_published_.speed_mps;
    di.gate_fresh =
        in.has_gated && !is_stale(in.now_s, in.gated_stamp_s, cfg_.logic.gated_timeout_s);
    di.gated_speed_mps = in.gated_speed_mps;
    if (!(confidence_ > 0.0 || has_free)) {
      // No track weight and no free-space field: nothing to plan toward.
      di.no_plan = true;
      solver_.reset();
    } else {
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
      si.track = confidence_ > 0.0 ? &projector_ : nullptr;
      si.field = in.field;
      si.confidence = confidence_;
      si.freespace = has_free ? in.freespace : nullptr;
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
      wanted = Command{so.cmd.speed_cmd, so.cmd.steering_cmd};
      plan_valid_ = true;
      di.all_collide = so.all_collide;
      di.plan_speed_mps = so.plan_speed_mps;
      plan_moves = so.plan_speed_mps >= std::max(floor_mps, 1e-3);
      bool halved = false;
      solver_.set_samples(budget_.update(out.solve_time_s, &halved));
      out.samples_halved = halved;
    }
    reason = drivability_.update(di, in.now_s);
    RecoveryInput ri;
    ri.now_s = in.now_s;
    ri.tick_s = cfg_.tick_s;
    ri.reason = reason;
    ri.reason_held_s = drivability_.held_s(reason);
    ri.wheel_speed_mps = std::isfinite(in.wheel_speed_mps) ? in.wheel_speed_mps : 0.0;
    ri.published_speed_mps = last_published_.speed_mps;
    ri.gate_fresh = di.gate_fresh;
    ri.gated_speed_mps = in.gated_speed_mps;
    ri.field = in.field;
    ri.freespace = has_free ? in.freespace : nullptr;
    ri.ego = ego_pose_;
    const RecoveryOutput ro = recovery_.update(ri);
    out.recovery_attempts = ro.attempts;
    if (ro.active) {
      state = PlannerState::Recovering;
      wanted = ro.cmd;
      reason = BlockReason::None;
      drivability_.reset();
    } else if (ro.exhausted) {
      state = PlannerState::Blocked;
      reason = BlockReason::RecoveryExhausted;
    } else {
      state = reason == BlockReason::None ? PlannerState::Driving : PlannerState::Blocked;
    }
    if (recovery_was_active_ && !ro.active) {  // a recovery ended or was abandoned
      solver_.reset();
      drivability_.reset();
    }
    recovery_was_active_ = ro.active;
  } else {
    drivability_.reset();
    recovery_.reset();
    recovery_was_active_ = false;
    out.v_cap_mps = speed_cap(cfg_.logic, li, confidence_);
  }
  if (!plan_valid_) solver_.reset();
  out.track_confidence = confidence_;
  out.block_reason = reason;
  const double v_cap = out.v_cap_mps;

  Command c = command_for_state(state, wanted, last_published_.steering_rad, v_cap,
                                geometry_.steering_min_rad, geometry_.steering_max_rad,
                                std::max(cfg_.recovery.recover_speed_mps, floor_mps));
  if (state == PlannerState::Driving) {
    // Low-pass the steering (the state restarts from the last published value on entering
    // Driving) and keep the speed in {0} U [floor, cap].
    if (!steer_lp_valid_) {
      steer_lp_ = last_published_.steering_rad;
      steer_lp_valid_ = true;
    }
    steer_lp_ = lowpass_step(steer_lp_, c.steering_rad, cfg_.tick_s, cfg_.logic.steer_output_tau_s);
    c.steering_rad =
        std::fabs(steer_lp_ - last_published_.steering_rad) < cfg_.logic.steer_output_hysteresis_rad
            ? last_published_.steering_rad
            : steer_lp_;
    // A plan that moves (its mean speed over the first 0.5 s is at least the floor) is commanded
    // at least the floor from the first step, so Driving never publishes 0 for a moving plan.
    const double moving_min = plan_moves ? std::max(floor_mps, 1e-3) : 0.0;
    c.speed_mps = snap_speed(std::max(c.speed_mps, moving_min), floor_mps, v_cap);
  } else {
    steer_lp_valid_ = false;
  }
  if (state == PlannerState::Driving || state == PlannerState::Blocked ||
      state == PlannerState::Recovering) {
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
