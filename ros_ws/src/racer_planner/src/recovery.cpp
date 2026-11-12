#include "racer_planner/recovery.hpp"

#include <algorithm>
#include <cmath>

namespace racer_planner {

namespace {
constexpr double kStepM = 0.025;  // path length per kinematic step of the plan

struct Pose {
  double x, y, yaw;
};

// One kinematic bicycle step of path length ds (negative backwards) at steering delta.
inline Pose advance(const Pose& p, double ds, double tan_delta, double wheelbase) {
  return Pose{p.x + ds * std::cos(p.yaw), p.y + ds * std::sin(p.yaw),
              p.yaw + ds * tan_delta / wheelbase};
}
}  // namespace

RecoveryPlan plan_recovery(const DistanceField& field, const FreespaceField* freespace,
                           const Pose2& ego, const VehicleGeometry& g, const RecoveryParams& p) {
  RecoveryPlan best;
  const Footprint fp(g, p.corner_radius_m);
  const bool have_field = freespace != nullptr && freespace->valid();
  const int n_rev = std::max(1, p.reverse_arcs);
  const int n_fwd = std::max(1, p.forward_arcs);
  const int rev_steps = std::max(1, static_cast<int>(std::ceil(p.recover_distance_m / kStepM)));
  const int fwd_steps = std::max(1, static_cast<int>(std::ceil(p.forward_distance_m / kStepM)));
  auto steering_at = [&](int i, int n) {
    return n == 1 ? 0.5 * (g.steering_min_rad + g.steering_max_rad)
                  : g.steering_min_rad + (g.steering_max_rad - g.steering_min_rad) * i / (n - 1);
  };
  double best_score = -1e300;
  for (int a = 0; a < n_rev; ++a) {
    const double delta = steering_at(a, n_rev);
    const double tan_d = std::tan(delta);
    Pose pose{ego.x, ego.y, ego.yaw};
    bool clear = true;
    for (int s = 0; s < rev_steps && clear; ++s) {
      pose = advance(pose, -p.recover_distance_m / rev_steps, tan_d, g.wheelbase_m);
      clear = fp.clearance_reverse(field, pose.x, pose.y, std::cos(pose.yaw), std::sin(pose.yaw)) >=
              p.d_hard_m;
    }
    if (!clear) continue;
    ++best.clear_arcs;
    const double phi_start = have_field
                                 ? freespace->phi(pose.x + g.wheelbase_m * std::cos(pose.yaw),
                                                  pose.y + g.wheelbase_m * std::sin(pose.yaw))
                                 : 0.0;
    double arc_score = -1e300;
    for (int f = 0; f < n_fwd; ++f) {
      const double fd = steering_at(f, n_fwd);
      const double tan_f = std::tan(fd);
      Pose q = pose;
      bool fwd_clear = true;
      double min_clear = 1e9;
      for (int s = 0; s < fwd_steps && fwd_clear; ++s) {
        q = advance(q, p.forward_distance_m / fwd_steps, tan_f, g.wheelbase_m);
        const double c = fp.clearance(field, q.x, q.y, std::cos(q.yaw), std::sin(q.yaw));
        min_clear = std::min(min_clear, c);
        fwd_clear = c >= p.d_hard_m;
      }
      if (!fwd_clear) continue;
      double score = min_clear;
      if (have_field) {
        score = phi_start - freespace->phi(q.x + g.wheelbase_m * std::cos(q.yaw),
                                           q.y + g.wheelbase_m * std::sin(q.yaw));
      }
      arc_score = std::max(arc_score, score);
    }
    if (arc_score > best_score + 1e-9) {
      best_score = arc_score;
      best.ok = arc_score > -1e299;
      best.steering_rad = delta;
      best.length_m = p.recover_distance_m;
      best.score = arc_score;
      best.arc = a;
    }
  }
  if (best_score <= -1e299) {
    const int clear_arcs = best.clear_arcs;
    best = RecoveryPlan{};
    best.clear_arcs = clear_arcs;
  }
  return best;
}

Recovery::Recovery(const RecoveryParams& p, const VehicleGeometry& g, double speed_floor_mps)
    : p_(p), g_(g), reverse_speed_(std::max(p.recover_speed_mps, std::max(speed_floor_mps, 0.0))) {}

void Recovery::reset() {
  phase_ = RecoveryPhase::Idle;
  attempts_ = 0;
  plan_ = RecoveryPlan{};
  reversed_m_ = 0.0;
  forward_m_ = 0.0;
  have_next_plan_ = false;
  gate_bad_ = false;
  release_ = false;
}

bool Recovery::start_due(const RecoveryInput& in) const {
  switch (in.reason) {
    case BlockReason::AllCollide:
    case BlockReason::BelowSpeedFloor:
      return in.reason_held_s >= p_.hold_collide_s;
    case BlockReason::WheelStalled:
      return in.reason_held_s >= p_.hold_stall_s;
    case BlockReason::GateRefusing:
      return in.reason_held_s >= p_.hold_gate_s;
    default:
      return false;
  }
}

RecoveryOutput Recovery::out(const RecoveryInput&, bool active, double speed) const {
  RecoveryOutput o;
  o.active = active;
  o.cmd.speed_mps = speed;
  o.cmd.steering_rad = active ? plan_.steering_rad : 0.0;
  o.phase = phase_;
  o.exhausted = phase_ == RecoveryPhase::Exhausted;
  o.attempts = attempts_;
  return o;
}

void Recovery::begin(const RecoveryInput& in) {
  if (in.field == nullptr) {
    have_next_plan_ = true;
    next_plan_s_ = in.now_s + p_.recover_retry_s;
    return;
  }
  plan_ = plan_recovery(*in.field, in.freespace, in.ego, g_, p_);
  if (!plan_.ok) {  // stay blocked and plan again later
    have_next_plan_ = true;
    next_plan_s_ = in.now_s + p_.recover_retry_s;
    return;
  }
  ++attempts_;
  forward_m_ = 0.0;
  reversed_m_ = 0.0;
  started_s_ = in.now_s;
  gate_bad_ = false;
  phase_ = RecoveryPhase::Stopping;
  have_next_plan_ = false;
}

void Recovery::abort(const RecoveryInput& in) {
  phase_ = RecoveryPhase::Idle;
  gate_bad_ = false;
  have_next_plan_ = true;
  next_plan_s_ = in.now_s + p_.recover_retry_s;
}

void Recovery::finish() {
  phase_ = RecoveryPhase::Idle;
  gate_bad_ = false;
  have_next_plan_ = false;
  reversed_m_ = 0.0;
}

RecoveryOutput Recovery::update(const RecoveryInput& in) {
  const double tick = std::isfinite(in.tick_s) && in.tick_s > 0.0 ? in.tick_s : 0.02;
  const double wheel = std::isfinite(in.wheel_speed_mps) ? in.wheel_speed_mps : 0.0;
  if (phase_ != RecoveryPhase::Reversing && wheel > 0.0) forward_m_ += wheel * tick;
  if (forward_m_ >= p_.recover_progress_m) {
    attempts_ = 0;
    forward_m_ = 0.0;
  }
  switch (phase_) {
    case RecoveryPhase::Idle:
      if (attempts_ >= p_.recover_max_attempts) {
        phase_ = RecoveryPhase::Exhausted;
        release_ = false;
        return out(in, false, 0.0);
      }
      if (start_due(in) && (!have_next_plan_ || in.now_s >= next_plan_s_)) {
        begin(in);
        if (phase_ == RecoveryPhase::Stopping) return out(in, true, 0.0);
      }
      return out(in, false, 0.0);
    case RecoveryPhase::Stopping: {
      if (!in.gate_fresh || in.now_s - started_s_ >= p_.recover_max_s) {
        abort(in);
        return out(in, false, 0.0);
      }
      if (std::fabs(wheel) < p_.stop_wheel_speed_mps) {
        phase_ = RecoveryPhase::Reversing;
        reversed_m_ = 0.0;
        return out(in, true, -reverse_speed_);
      }
      return out(in, true, 0.0);
    }
    case RecoveryPhase::Reversing: {
      reversed_m_ += -wheel * tick;
      if (!in.gate_fresh) {
        abort(in);
        return out(in, false, 0.0);
      }
      // The request is negative: if the gate does not pass a negative speed for the debounce time
      // its rear gate refused it.
      if (in.published_speed_mps < 0.0 && in.gated_speed_mps >= 0.0) {
        if (!gate_bad_) {
          gate_bad_ = true;
          gate_bad_since_s_ = in.now_s;
        }
        if (in.now_s - gate_bad_since_s_ >= p_.recover_gate_debounce_s) {
          abort(in);
          return out(in, false, 0.0);
        }
      } else {
        gate_bad_ = false;
      }
      if (reversed_m_ >= plan_.length_m || in.now_s - started_s_ >= p_.recover_max_s) {
        finish();
        return out(in, false, 0.0);
      }
      return out(in, true, -reverse_speed_);
    }
    case RecoveryPhase::Exhausted: {
      if (attempts_ < p_.recover_max_attempts) {  // the car moved forward again by itself
        phase_ = RecoveryPhase::Idle;
        release_ = false;
        return out(in, false, 0.0);
      }
      if (in.reason == BlockReason::None) {
        if (!release_) {
          release_ = true;
          release_since_s_ = in.now_s;
        }
        if (in.now_s - release_since_s_ >= p_.recover_release_s) {
          attempts_ = 0;
          forward_m_ = 0.0;
          phase_ = RecoveryPhase::Idle;
          release_ = false;
          return out(in, false, 0.0);
        }
      } else {
        release_ = false;
      }
      return out(in, false, 0.0);
    }
  }
  return out(in, false, 0.0);
}

}  // namespace racer_planner
