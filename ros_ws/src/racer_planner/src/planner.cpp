#include "racer_planner/planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
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

constexpr int kMaxModelSteps = 64;  // ring steps one publish may advance (a very late tick)
}  // namespace

Planner::Planner(const PlannerConfig& config, const DynamicsParams& dynamics,
                 const VehicleGeometry& geometry)
    : cfg_(config),
      geometry_(geometry),
      model_pub_(dynamics, geometry, config.tick_s),
      plan_dt_s_(dynamics.dt_s),
      model_roll_(dynamics, geometry, config.tick_s),
      model_ego_(dynamics, geometry, config.tick_s),
      ego_(&model_roll_),
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
      track_gate_(config.consistency),
      horizon_(std::max(2, config.mppi.horizon_steps)),
      delay_ticks_(std::min(
          25, std::max(0, static_cast<int>(
                              std::lround(dynamics.nominal.command_delay_s / config.tick_s))))) {
  if (horizon_ > kMaxPlanSteps) {
    throw std::invalid_argument("racer_planner: horizon_steps above kMaxPlanSteps");
  }
  // Step contract 2: the scan frame is dead reckoned by ground motion, like the rollouts. A
  // contract 1 bundle leaves the dead reckoning exactly as it was.
  if (dynamics.step_contract == 2) ego_.set_ground_model(&model_ego_);
  cfg_.mppi = solver_params(config, dynamics);
  cfg_.max_samples = std::min(std::max(config.max_samples, kMinSamples), kMaxSamples);
  solver_.set_samples(std::min(cfg_.max_samples, cfg_.mppi.samples));
  slew_.tick_s = cfg_.tick_s;
  slew_.steering_rate_rad_per_s = geometry_.steering_max_rate_rad_per_s;
  slew_.max_acceleration_mps2 = geometry_.max_acceleration_mps2;
  slew_.fraction = cfg_.logic.rate_limit_fraction;
  slew_.speed_floor_mps = cfg_.mppi.speed_floor_mps;
  start_slew_ = slew_;
  start_slew_.fraction = std::min(cfg_.logic.start_steer_fraction, slew_.fraction);
  reverse_limit_mps_ = std::max(cfg_.recovery.recover_speed_mps, cfg_.mppi.speed_floor_mps);
}

TickOutput Planner::tick(const TickInput& in) {
  solve(in);
  PublishInput p;
  p.now_s = in.now_s;
  p.has_gated = in.has_gated;
  p.gated_stamp_s = in.gated_stamp_s;
  p.gated_steering_rad = in.gated_steering_rad;
  p.gated_speed_mps = in.gated_speed_mps;
  p.gated_seq = in.gated_seq;
  p.gated_receipt_s = in.gated_receipt_s;
  return publish(p);
}

void Planner::store_snapshot(const PlanSnapshot& s) {
  std::lock_guard<std::mutex> lock(mu_);
  snapshot_ = s;
}

// -- solve side --------------------------------------------------------------------------------

SolveReport Planner::solve(const TickInput& in) {
  try {
    return solve_impl(in);
  } catch (...) {
    try {
      return solve_fault(in);
    } catch (...) {
      SolveReport r;
      r.state = PlannerState::Fault;
      return r;
    }
  }
}

SolveReport Planner::solve_fault(const TickInput& in) {
  plan_valid_ = false;
  solver_.reset();
  recovery_.reset();
  drivability_.reset();
  recovery_was_active_ = false;
  solve_good_ticks_ = 0;
  PlanSnapshot snap;
  snap.valid = true;
  snap.state = PlannerState::Fault;
  snap.internal_fault = true;
  snap.plan_time_s = in.now_s;
  snap.dt_s = plan_dt_s_;
  store_snapshot(snap);
  last_solve_state_ = PlannerState::Fault;
  SolveReport r;
  r.state = PlannerState::Fault;
  return r;
}

SolveReport Planner::solve_impl(const TickInput& in) {
  SolveReport rep;
  const double tick = cfg_.tick_s;
  const double now = in.now_s;
  bool gap_ok = false;  // this callback arrived within start_tick_gap_s of the previous callback
  if (std::isfinite(now)) {
    if (have_last_solve_) {
      const double raw = now - last_solve_s_;        // since the last real solve
      const double raw_cb = now - last_solve_cb_s_;  // since the last callback of any kind
      if (raw < 0.0 || raw_cb < 0.0) {               // the clock went backwards: start over
        have_last_solve_ = false;
        solve_good_ticks_ = 0;
      } else if (raw < 0.5 * tick) {  // a catch-up callback just after a late one: nothing new
        bunched_ticks_.fetch_add(1, std::memory_order_relaxed);
        last_solve_cb_s_ = now;
        rep.bunched = true;
        rep.state = last_solve_state_;
        return rep;
      } else {
        gap_ok = raw_cb <= cfg_.logic.start_tick_gap_s;
      }
    }
  }

  Shared sh;
  {
    std::lock_guard<std::mutex> lock(mu_);
    sh = shared_;
  }
  State est = sh.est;
  est.x = est.y = est.yaw = 0.0;

  const LogicParams& lg = cfg_.logic;
  const bool odom_valid = in.has_odom && std::isfinite(in.wheel_speed_mps);
  const bool odom_fresh = odom_valid && classify_age(now, in.odom_stamp_s, lg.odom_timeout_s,
                                                     lg.odom_hard_timeout_s) == Age::Fresh;
  if (odom_fresh) {
    est.v = std::max(0.0, in.wheel_speed_mps);
    std::lock_guard<std::mutex> lock(mu_);
    shared_.est.v = est.v;  // the model follows the measurement
  }
  // The dead reckoning takes the signed wheel speed (the car backs up while recovering). While
  // the wheel speed is stale it takes the model's own estimate.
  // The commands acting now (the delayed ones) are the inputs of the residual and of a contract 2
  // learned block; a bundle with neither never reads them.
  const Control acting_now =
      delay_ticks_ > 0 ? sh.ring[static_cast<std::size_t>(sh.ring_head)]
                       : Control{sh.last_published.speed_mps, sh.last_published.steering_rad};
  ego_.push(now, odom_fresh ? in.wheel_speed_mps : est.v, est.delta, acting_now.speed_cmd,
            acting_now.steering_cmd);

  // The one-frame bridge (track_consistency.hpp): a new track frame that jumps and narrows against
  // the last accepted one is replaced by that accepted track for its lifetime, at most one frame
  // in a row and only while the accepted track is younger than max_hold_age_s. Everything the
  // solve takes from the track (confidence, projection) uses the selected one.
  const LocalTrackData* track = in.track;
  {
    Pose2 accepted_in_newest;
    bool covered = false;
    if (track_gate_.is_new_frame(in.track) && track_gate_.has_accepted()) {
      accepted_in_newest =
          ego_.relative_pose(in.track->stamp_s, track_gate_.accepted_stamp_s(), &covered);
    }
    const std::uint32_t before = track_gate_.bridged_frames();
    track = track_gate_.select(in.track, now, accepted_in_newest, covered);
    if (track_gate_.bridged_frames() != before) {
      rep.track_bridged = true;
      bridged_frames_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  LogicInputs li;
  li.now_s = now;
  li.has_odom = odom_valid;
  li.odom_stamp_s = in.odom_stamp_s;
  li.wheel_speed_mps = in.wheel_speed_mps;
  // A scan with fewer than min_scan_points usable points (every beam NaN, inf or 0, a covered
  // lidar) is not a free road: it is no scan.
  li.has_scan = in.has_scan && in.field != nullptr &&
                in.field->seed_count() >= static_cast<std::size_t>(std::max(0, lg.min_scan_points));
  li.scan_stamp_s = in.scan_stamp_s;
  if (track != nullptr) {
    li.track.present = true;
    li.track.stamp_s = track->stamp_s;
    li.track.validity = validate_track(*track);
    li.track.valid_count = track->valid_count;
    li.track.boundary_residual_m = track->boundary_residual_m;
    li.track.intrusion_fraction = track->intrusion_fraction;
  }
  li.has_guidance = in.has_guidance;
  li.guidance_stamp_s = in.guidance_stamp_s;
  li.guidance_v_cap_mps = in.guidance_v_cap_mps;
  li.guidance_profile_bad = in.guidance_profile_bad;

  const Gate gate = evaluate_gate(li, lg);
  PlanSnapshot snap;
  snap.valid = true;
  snap.plan_time_s = now;
  snap.dt_s = plan_dt_s_;
  TickOutput& out = snap.status;
  out.est_speed_mps = est.v;
  out.est_steering_rad = est.delta;
  out.track_bridged = track_gate_.last_pick() == TrackPick::kBridged;
  auto age_or_neg = [&](bool present, double stamp) {
    const double a = present ? age_s(now, stamp) : -1.0;
    // A stamp a few ms ahead of the tick (a swapped pair of scans) is clock skew, not a negative
    // age: it counts as 0 (the staleness classes already treat it as fresh).
    return std::isfinite(a) ? std::max(a, 0.0) : -1.0;
  };
  out.track_age_s = age_or_neg(in.track != nullptr, in.track != nullptr ? in.track->stamp_s : 0.0);
  out.scan_age_s = age_or_neg(li.has_scan, in.scan_stamp_s);
  out.samples = solver_.samples();
  out.plan_time_s = now;
  snap.scan_fresh = li.has_scan && classify_age(now, in.scan_stamp_s, lg.scan_timeout_s,
                                                lg.scan_hard_timeout_s) == Age::Fresh;
  snap.odom_fresh = odom_fresh;
  snap.track_ever = in.track != nullptr;
  snap.track_fresh = in.track != nullptr && classify_age(now, in.track->stamp_s, lg.conf_age_full_s,
                                                         lg.conf_age_full_s) == Age::Fresh;

  PlannerState state = gate.state;
  Command wanted;
  wanted.steering_rad = sh.last_published.steering_rad;
  BlockReason reason = BlockReason::None;
  plan_valid_ = false;
  const double floor_mps = cfg_.mppi.speed_floor_mps;
  bool plan_moves = false;
  bool solver_ran = false;
  if (gate.may_solve) {
    // Track confidence: how far the track prior is trusted. A track that cannot be projected
    // counts as none.
    double c_raw = track_confidence_raw(li.track, now, lg);
    bool track_ok = false;
    const Pose2 frame_change =
        track != nullptr ? ego_.relative_pose(in.scan_stamp_s, track->stamp_s) : Pose2{};
    if (c_raw > 0.0 && track != nullptr) track_ok = projector_.build(*track, frame_change);
    if (!track_ok) c_raw = 0.0;
    confidence_ = track_confidence(c_raw, confidence_, tick, lg.conf_rise_per_s);
    if (!track_ok) confidence_ = 0.0;
    const bool has_free = in.freespace != nullptr && in.freespace->valid();
    ego_pose_ = ego_.relative_pose(in.scan_stamp_s, now);
    if (has_free) {
      const double cy = std::cos(-ego_pose_.yaw), sy = std::sin(-ego_pose_.yaw);
      const double dx = in.freespace->goal().x - ego_pose_.x;
      const double dy = in.freespace->goal().y - ego_pose_.y;
      out.has_freespace_goal = true;
      out.freespace_goal_x_m = cy * dx - sy * dy;
      out.freespace_goal_y_m = sy * dx + cy * dy;
      out.freespace_reach_m = in.freespace->reach_m();
    }
    const double v_cap = speed_cap(lg, li, confidence_);
    out.v_cap_mps = v_cap;
    DrivabilityInputs di;
    di.speed_floor_mps = floor_mps;
    di.wheel_speed_mps = est.v;
    di.published_speed_mps = sh.last_published.speed_mps;
    di.gate_fresh = in.has_gated && !is_stale(now, in.gated_stamp_s, lg.gated_timeout_s);
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
      x0.v = est.v;
      x0.delta = est.delta;
      for (int i = 0; i < delay_ticks_; ++i) {
        x0 = model_roll_.step(x0,
                              sh.ring[static_cast<std::size_t>((sh.ring_head + i) % delay_ticks_)]);
      }
      SolveInput si;
      si.x0 = x0;
      si.prev_cmd = Control{sh.last_published.speed_mps, sh.last_published.steering_rad};
      si.v_cap_mps = v_cap;
      si.track = confidence_ > 0.0 ? &projector_ : nullptr;
      si.field = in.field;
      si.confidence = confidence_;
      si.freespace = has_free ? in.freespace : nullptr;
      si.gate = in.gate;
      si.wheel_speed_mps = est.v;
      if (in.lap != nullptr && in.lap->active) {
        si.lap = in.lap;
        si.lap_station_now_m = ego_.distance_between(in.lap->stamp_s, in.now_s);
      }
      const auto t0 = std::chrono::steady_clock::now();
      const SolveOutput so = solver_.solve(si);
      out.solve_time_s =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      if (!std::isfinite(so.cmd.speed_cmd) || !std::isfinite(so.cmd.steering_cmd)) {
        throw std::runtime_error("solver produced a non finite command");
      }
      solver_ran = true;
      out.samples = so.samples;
      out.best_cost = so.best_cost;
      out.plan_progress_m = so.plan_progress_m;
      out.plan_min_clearance_m = so.plan_min_clearance_m;
      out.collision_fraction = so.collision_fraction;
      out.has_final_sequence = true;
      out.final_sequence = so.final_sequence;
      out.unchecked_final_collided = so.unchecked_final_collided;
      cost_terms_array(so.final_cost, out.cost_terms.data());
      out.speed_limiter = so.speed_limiter;
      if (so.speed_limiter == SpeedLimiter::kCap && confidence_ < 1.0 &&
          v_cap < speed_cap(lg, li, 1.0) - 1e-9) {
        // The cap is below what full track confidence would allow: the track's validity, not the
        // configured or lap cap, holds the speed.
        out.speed_limiter = SpeedLimiter::kValidity;
      }
      wanted = Command{so.cmd.speed_cmd, so.cmd.steering_cmd};
      plan_valid_ = true;
      di.all_collide = so.all_collide;
      di.plan_speed_mps = so.plan_speed_mps;
      plan_moves = so.plan_speed_mps >= std::max(floor_mps, 1e-3);
      bool halved = false;
      solver_.set_samples(budget_.update(out.solve_time_s, &halved));
      out.samples_halved = halved;
      rep.samples_halved = halved;
    }
    if (sh.starting) {
      // Warm start: the solver runs on the real inputs, its output is discarded except for the
      // steering target. No drivability or recovery decisions are made before the car may move.
      drivability_.reset();
      recovery_.reset();
      recovery_was_active_ = false;
      state = PlannerState::Starting;
    } else {
      reason = drivability_.update(di, now);
      RecoveryInput ri;
      ri.now_s = now;
      ri.tick_s = tick;
      ri.reason = reason;
      ri.reason_held_s = drivability_.held_s(reason);
      ri.wheel_speed_mps = std::isfinite(in.wheel_speed_mps) ? in.wheel_speed_mps : 0.0;
      ri.published_speed_mps = sh.last_published.speed_mps;
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
    }
  } else {
    drivability_.reset();
    recovery_.reset();
    recovery_was_active_ = false;
    out.v_cap_mps = speed_cap(lg, li, confidence_);
  }
  if (!plan_valid_) solver_.reset();
  out.track_confidence = confidence_;
  out.block_reason = reason;

  // Consecutive on-time solves on the real inputs (the warm start gate reads the count).
  if (solver_ran && gap_ok) {
    solve_good_ticks_ = std::min(solve_good_ticks_ + 1, 1000000);
  } else {
    solve_good_ticks_ = 0;
  }
  snap.solve_good_ticks = solve_good_ticks_;

  snap.state = state;
  snap.plan_moves = plan_moves;
  snap.hold = wanted;
  snap.use_sequence = plan_valid_ && state != PlannerState::Recovering &&
                      (state == PlannerState::Driving || state == PlannerState::Blocked ||
                       state == PlannerState::Starting);
  if (snap.use_sequence) {
    const std::vector<Control>& mean = solver_.mean_sequence();
    snap.n = static_cast<int>(std::min<std::size_t>(mean.size(), kMaxPlanSteps));
    std::copy(mean.begin(), mean.begin() + snap.n, snap.seq.begin());
  }
  store_snapshot(snap);

  if (std::isfinite(now)) {
    last_solve_s_ = now;
    last_solve_cb_s_ = now;
    have_last_solve_ = true;
  }
  last_solve_state_ = state;
  rep.state = state;
  rep.solver_ran = solver_ran;
  rep.solve_time_s = out.solve_time_s;
  return rep;
}

// -- publish side ------------------------------------------------------------------------------

// Model bookkeeping in publish order: the delay ring and the steering estimate follow what was
// published, until the gated /drive says what the safety node made of it (apply_gated). A late
// publish advances the ring by the ticks that passed (the command before it kept acting
// meanwhile).
void Planner::advance_model(const Command& published, int steps, const Command& before) {
  std::lock_guard<std::mutex> lock(mu_);
  State est = shared_.est;
  Pending pend;
  for (int s = 0; s < steps; ++s) {
    const Command& c = (s + 1 < steps) ? before : published;
    const Control sent{c.speed_mps, c.steering_rad};
    Control acting = sent;
    const State est_before = est;
    const int slot = shared_.ring_head;
    if (delay_ticks_ > 0) {
      acting = shared_.ring[static_cast<std::size_t>(shared_.ring_head)];
      shared_.ring[static_cast<std::size_t>(shared_.ring_head)] = sent;
      shared_.ring_head = (shared_.ring_head + 1) % delay_ticks_;
    }
    const State next = model_pub_.step(est, acting);
    est.delta = next.delta;
    est.v = next.v;  // replaced by the wheel speed at the next solve when fresh
    est.x = est.y = est.yaw = 0.0;
    pend.slot = slot;
    pend.est_before = est_before;
  }
  shared_.est = est;
  shared_.last_published = published;
  shared_.starting = starting_;
  pend.valid = steps > 0;
  pend.published = published;
  pend.publish_s = pending_publish_s_;  // set by the caller before this call
  pending_ = pend;
  for (std::size_t i = sent_hist_.size() - 1; i > 0; --i) sent_hist_[i] = sent_hist_[i - 1];
  sent_hist_[0] = published;
  sent_hist_n_ = std::min(sent_hist_n_ + 1, static_cast<int>(sent_hist_.size()));
}

namespace {
bool same_float(const Command& a, const Command& b) {
  return static_cast<float>(a.speed_mps) == static_cast<float>(b.speed_mps) &&
         static_cast<float>(a.steering_rad) == static_cast<float>(b.steering_rad);
}
}  // namespace

// The previous tick's ring entry takes the gated value when one arrived since that tick's publish
// (claude-docs/13 17.4.3). A gated message is matched to a tick by value first: one equal to what
// was published (as the float32 the topic carries) leaves the double as it was, so with /drive
// equal to /drive_raw the state is bit identical to what it was before this existed; one equal to
// an older tick's command is that tick's late message and says nothing about this one; any other
// is the gate's output for the pending tick. No matching message: the published command stands.
void Planner::apply_gated(const PublishInput& in, double now) {
  const bool new_msg = in.has_gated && in.gated_seq != last_gated_seq_;
  if (in.has_gated) last_gated_seq_ = in.gated_seq;
  if (!pending_.valid) return;
  const Pending pend = pending_;
  pending_.valid = false;
  if (!new_msg || !std::isfinite(in.gated_receipt_s) || !(in.gated_receipt_s >= pend.publish_s) ||
      !(in.gated_receipt_s <= now) || !std::isfinite(in.gated_speed_mps) ||
      !std::isfinite(in.gated_steering_rad)) {
    gated_fallback_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  const Command g{in.gated_speed_mps, in.gated_steering_rad};
  if (same_float(g, pend.published)) {
    gated_equal_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  for (int j = 1; j < sent_hist_n_; ++j) {
    if (same_float(g, sent_hist_[static_cast<std::size_t>(j)])) {
      gated_fallback_.fetch_add(1, std::memory_order_relaxed);  // an older tick's late message
      return;
    }
  }
  gated_corrected_.fetch_add(1, std::memory_order_relaxed);
  std::lock_guard<std::mutex> lock(mu_);
  if (delay_ticks_ > 0) {
    shared_.ring[static_cast<std::size_t>(pend.slot)] = Control{g.speed_mps, g.steering_rad};
  } else {
    // No ring: the command already acted for one step. Move the estimate by the difference the
    // gated command makes to that step.
    const State a = model_pub_.step(pend.est_before,
                                    Control{pend.published.speed_mps, pend.published.steering_rad});
    const State b = model_pub_.step(pend.est_before, Control{g.speed_mps, g.steering_rad});
    shared_.est.delta += b.delta - a.delta;
    shared_.est.v = std::max(0.0, shared_.est.v + b.v - a.v);
  }
}

TickOutput Planner::publish(const PublishInput& in) {
  try {
    return publish_impl(in);
  } catch (...) {
    TickOutput out;
    out.state = PlannerState::Fault;
    out.cmd = Command{0.0, 0.0};
    steer_lp_valid_ = false;
    try {
      advance_model(out.cmd, 1, last_published_);
    } catch (...) {
    }
    last_published_ = out.cmd;
    last_out_ = out;
    return out;
  }
}

TickOutput Planner::publish_impl(const PublishInput& in) {
  const double tick = cfg_.tick_s;
  const double now = in.now_s;
  const LogicParams& lg = cfg_.logic;
  const bool finite_now = std::isfinite(now);

  // Two times: the last REAL publish (a plan snapshot through the shaping) decides whether this
  // callback is bunched and how many model ticks have passed; the last message of ANY kind (a
  // republish counts) is what safety_node's own measured dt runs against, so it sets the slew
  // allowance and the watchdog gap.
  double raw_real = std::numeric_limits<double>::quiet_NaN();
  double raw_any = std::numeric_limits<double>::quiet_NaN();
  if (finite_now && have_last_publish_) {
    raw_any = now - last_publish_s_;
    raw_real = now - last_real_s_;
    if (raw_any < 0.0 || raw_real < 0.0) {  // the clock went backwards: treat this as the first
      have_last_publish_ = false;
      publish_good_ticks_ = 0;
      raw_any = raw_real = std::numeric_limits<double>::quiet_NaN();
    }
  }
  // Consecutive callbacks within start_tick_gap_s of the previous one (a bunched one counts).
  publish_good_ticks_ = (have_last_publish_ && raw_any <= lg.start_tick_gap_s)
                            ? std::min(publish_good_ticks_ + 1, 1000000)
                            : 0;
  if (have_last_publish_ && raw_real < 0.5 * tick) {
    // A callback just after a late one: the previous command again, nothing re-solved or
    // re-shaped. It is a message safety_node sees, so the next real publish measures its elapsed
    // time from it.
    bunched_ticks_.fetch_add(1, std::memory_order_relaxed);
    TickOutput o = last_out_;
    o.bunched = true;
    o.gap_reset = false;
    o.left_starting = false;
    last_publish_s_ = now;
    return o;
  }
  const bool first = !have_last_publish_;
  const double elapsed = first ? tick : slew_elapsed_s(raw_any, tick);
  const bool gap = !first && lg.gap_reset_s > 0.0 && raw_any > lg.gap_reset_s;
  apply_gated(in, now);

  PlanSnapshot snap;
  {
    std::lock_guard<std::mutex> lock(mu_);
    snap = snapshot_;
  }
  const bool have_snap = snap.valid;
  const double age = have_snap ? now - snap.plan_time_s : std::numeric_limits<double>::quiet_NaN();
  const bool plan_fresh = have_snap && !is_stale(now, snap.plan_time_s, lg.plan_timeout_s);

  TickOutput out = have_snap ? snap.status : TickOutput{};
  out.bunched = false;
  out.elapsed_s = elapsed;
  out.plan_age_s = std::isfinite(age) ? age : 0.0;
  if (gap) {
    // After a gap long enough for safety_node's watchdog to brake, its output is 0: restart the
    // speed ramp from there (a start from rest) rather than step from the old speed.
    last_published_.speed_mps = 0.0;
    out.gap_reset = true;
  }
  if (first && !have_starting_since_) {
    starting_since_s_ = finite_now ? now : 0.0;
    have_starting_since_ = true;
  }

  PlannerState state = have_snap ? snap.state : PlannerState::Starting;
  // A return to Driving after a stop longer than start_stop_hold_s goes through STARTING again.
  if (!starting_ && have_snap && plan_fresh && state == PlannerState::Driving && finite_now &&
      lg.start_stop_hold_s > 0.0 && now - last_nonzero_s_ > lg.start_stop_hold_s) {
    starting_ = true;
    start_seeded_ = false;
    starting_since_s_ = now;
  }
  const bool moving_or_deciding =
      state == PlannerState::Driving || state == PlannerState::Blocked ||
      state == PlannerState::Recovering || state == PlannerState::Starting;
  if (starting_ && moving_or_deciding) state = PlannerState::Starting;
  if (!have_snap) state = PlannerState::Starting;

  bool stale = false;
  if (have_snap && !plan_fresh && is_moving_state(state)) {
    stale = true;
    state = PlannerState::Fault;
  }

  if (starting_ && !start_seeded_) {
    if (lg.require_gated_for_start && !in.has_gated && finite_now &&
        now - starting_since_s_ < lg.start_gated_grace_s) {
      // The gated /drive has not been seen yet, so where the steering starts from is not known:
      // say nothing on /drive_raw (whatever the state) for a moment instead of guessing 0
      // against a held steering.
      out.state = state;
      out.cmd = Command{0.0, last_published_.steering_rad};
      out.drive_suppressed = true;
      out.start_missing = StartMissing::Gated;
      out.starting_s = std::max(0.0, now - starting_since_s_);
      out.start_timed_out = out.starting_s >= lg.start_timeout_s;
      return out;
    }
    // The steering the gated /drive holds is where this run's output starts from.
    last_published_.steering_rad =
        in.has_gated ? in.gated_steering_rad : last_published_.steering_rad;
    last_published_.steering_rad =
        std::isfinite(last_published_.steering_rad)
            ? std::min(std::max(last_published_.steering_rad, geometry_.steering_min_rad),
                       geometry_.steering_max_rad)
            : 0.0;
    start_seeded_ = true;
  }

  if (state == PlannerState::Starting && starting_) {
    StartGateInputs gi;
    gi.plan_fresh = plan_fresh;
    gi.scan_fresh = have_snap && snap.scan_fresh;
    gi.odom_fresh = have_snap && snap.odom_fresh;
    gi.track_ever = have_snap && snap.track_ever;
    gi.track_fresh = have_snap && snap.track_fresh;
    gi.publish_good_ticks = publish_good_ticks_;
    gi.solve_good_ticks = have_snap ? snap.solve_good_ticks : 0;
    gi.gated_seen = in.has_gated;
    const StartGateResult r = start_gate(gi, lg);
    out.start_missing = r.missing;
    if (r.leave) {
      starting_ = false;
      out.left_starting = true;
      last_nonzero_s_ = finite_now ? now : last_nonzero_s_;
    }
    out.starting_s = finite_now ? std::max(0.0, now - starting_since_s_) : 0.0;
    out.start_timed_out = !r.leave && out.starting_s >= lg.start_timeout_s;
  }

  // The control the snapshot gives for now: the plan indexed by the time since it was made.
  Command wanted;
  wanted.steering_rad = last_published_.steering_rad;
  if (have_snap && !stale) {
    if (snap.use_sequence) {
      const Control u = interpolate_plan(snap.seq.data(), snap.n, snap.dt_s, age);
      wanted = Command{u.speed_cmd, u.steering_cmd};
    } else {
      wanted = snap.hold;
    }
  }
  const double floor_mps = cfg_.mppi.speed_floor_mps;
  const double v_cap = have_snap ? snap.status.v_cap_mps : 0.0;
  Command c;
  if (stale) {
    // The solve side stopped delivering: stop, keep the steering. The speed ramp restarts from 0
    // with the next fresh snapshot (the last published speed is 0 from here on).
    c = Command{0.0, last_published_.steering_rad};
    steer_lp_valid_ = false;
  } else {
    c = command_for_state(state, wanted, last_published_.steering_rad, v_cap,
                          geometry_.steering_min_rad, geometry_.steering_max_rad,
                          reverse_limit_mps_);
    if (state == PlannerState::Starting && have_snap) {
      // Speed 0, the steering slews toward the solver's target.
      c.steering_rad = std::isfinite(wanted.steering_rad)
                           ? std::min(std::max(wanted.steering_rad, geometry_.steering_min_rad),
                                      geometry_.steering_max_rad)
                           : last_published_.steering_rad;
      // Hold the steering where it is until the ticks have proven on time (see
      // LogicParams::start_steer_hold_ticks).
      if (publish_good_ticks_ < lg.start_steer_hold_ticks)
        c.steering_rad = last_published_.steering_rad;
    }
  }
  if (state == PlannerState::Driving) {
    // Low-pass the steering (the state restarts from the last published value on entering
    // Driving) and keep the speed in {0} U [floor, cap].
    if (!steer_lp_valid_) {
      steer_lp_ = last_published_.steering_rad;
      steer_lp_valid_ = true;
    }
    steer_lp_ = lowpass_step(steer_lp_, c.steering_rad, elapsed, lg.steer_output_tau_s);
    c.steering_rad =
        std::fabs(steer_lp_ - last_published_.steering_rad) < lg.steer_output_hysteresis_rad
            ? last_published_.steering_rad
            : steer_lp_;
    // A plan that moves (its mean speed over the first 0.5 s is at least the floor) is commanded
    // at least the floor from the first step, so Driving never publishes 0 for a moving plan.
    const double moving_min = (have_snap && snap.plan_moves) ? std::max(floor_mps, 1e-3) : 0.0;
    c.speed_mps = snap_speed(std::max(c.speed_mps, moving_min), floor_mps, v_cap);
  } else {
    steer_lp_valid_ = false;
  }
  if (state == PlannerState::Driving || state == PlannerState::Blocked ||
      state == PlannerState::Recovering || state == PlannerState::Starting) {
    c = slew_limit(c, last_published_, elapsed,
                   state == PlannerState::Starting ? start_slew_ : slew_);
    // Never ask for more than the gate can follow in one cycle above what it last OUTPUT. The
    // slew above is relative to the last command, and the gate's output can be far below it: a
    // /drive_raw that reached safety_node late although it was published on time trips the
    // watchdog (output 0) while this planner's gap stayed under gap_reset_s, and the next command
    // then asked for the full speed from a standstill, which the gate clipped (found 2026-11-17 in
    // the canary's stall cases, see test_planner_resume_safety.cpp). In a normal ramp the gated
    // speed is one or two commands behind, a step or two below what is asked, so this does not
    // bind there. 0.95 of max_acceleration * tick, under what the gate allows in a cycle.
    if (finite_now && in.has_gated && std::isfinite(in.gated_speed_mps) &&
        !is_stale(now, in.gated_stamp_s, lg.gated_timeout_s)) {
      // A gate output in reverse counts as 0: growth forward is counted from zero.
      const double follow_mps = std::max(in.gated_speed_mps, 0.0) +
                                0.95 * slew_.max_acceleration_mps2 * slew_elapsed_s(tick, tick);
      if (c.speed_mps > follow_mps) c.speed_mps = follow_mps;
    }
  }

  out.state = state;
  out.cmd = c;
  out.plan_stale = stale;
  if (stale) out.v_cap_mps = 0.0;
  out.est_steering_rad = [&] {
    std::lock_guard<std::mutex> lock(mu_);
    return shared_.est.delta;
  }();

  int steps = 1;
  if (!first && std::isfinite(raw_real)) {
    steps = std::min(std::max(static_cast<int>(std::lround(raw_real / tick)), 1), kMaxModelSteps);
  }
  const Command before = last_published_;
  pending_publish_s_ = finite_now ? now : 0.0;
  advance_model(c, steps, before);
  last_published_ = c;
  if (c.speed_mps != 0.0 && finite_now) last_nonzero_s_ = now;
  if (finite_now) {
    last_publish_s_ = now;
    last_real_s_ = now;
    have_last_publish_ = true;
  }
  last_out_ = out;
  return out;
}

}  // namespace racer_planner
