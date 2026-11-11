#include "racer_planner/ego_motion.hpp"

#include <cmath>

namespace racer_planner {

EgoMotion::EgoMotion(double wheelbase_m, double understeer_s2_per_m2)
    : wheelbase_(wheelbase_m), understeer_(understeer_s2_per_m2) {}

void EgoMotion::clear() {
  head_ = 0;
  count_ = 0;
}

void EgoMotion::push(double t_s, double v_mps, double delta_rad) {
  if (!std::isfinite(t_s) || !std::isfinite(v_mps) || !std::isfinite(delta_rad)) return;
  if (count_ > 0 && !(t_s > at(count_ - 1).t)) return;
  if (count_ < kCapacity) {
    buf_[(head_ + count_) % kCapacity] = Sample{t_s, v_mps, delta_rad};
    ++count_;
  } else {
    buf_[head_] = Sample{t_s, v_mps, delta_rad};
    head_ = (head_ + 1) % kCapacity;
  }
}

void EgoMotion::advance(Pose2* pose, const Sample& s, double dt) const {
  if (!(dt > 0.0)) return;
  const double w = s.v * std::tan(s.delta) / wheelbase_ / (1.0 + understeer_ * s.v * s.v);
  const double th = pose->yaw;
  if (std::fabs(w) > 1e-9) {
    pose->x += s.v * (std::sin(th + w * dt) - std::sin(th)) / w;
    pose->y += s.v * (std::cos(th) - std::cos(th + w * dt)) / w;
  } else {
    pose->x += s.v * dt * std::cos(th);
    pose->y += s.v * dt * std::sin(th);
  }
  pose->yaw = th + w * dt;
}

Pose2 EgoMotion::forward(double t_from, double t_to, bool* covered) const {
  Pose2 pose;
  if (count_ == 0) {
    if (covered != nullptr) *covered = false;
    return pose;
  }
  if (covered != nullptr) *covered = !(t_from < at(0).t);
  // Value in force at t_from: the last sample at or before it, else the first sample.
  std::size_t cur = 0;
  while (cur + 1 < count_ && at(cur + 1).t <= t_from) ++cur;
  double t = t_from;
  for (std::size_t i = cur + 1; i < count_; ++i) {
    const double ti = at(i).t;
    if (ti >= t_to) break;
    advance(&pose, at(cur), ti - t);
    t = ti;
    cur = i;
  }
  advance(&pose, at(cur), t_to - t);
  return pose;
}

Pose2 EgoMotion::relative_pose(double t_from_s, double t_to_s, bool* covered) const {
  if (!std::isfinite(t_from_s) || !std::isfinite(t_to_s)) {
    if (covered != nullptr) *covered = false;
    return Pose2{};
  }
  if (t_to_s >= t_from_s) return forward(t_from_s, t_to_s, covered);
  // Pose of the car at the earlier time in the frame of the later one: invert the forward motion.
  const Pose2 f = forward(t_to_s, t_from_s, covered);
  const double c = std::cos(f.yaw);
  const double s = std::sin(f.yaw);
  Pose2 inv;
  inv.yaw = -f.yaw;
  inv.x = -(c * f.x + s * f.y);
  inv.y = -(-s * f.x + c * f.y);
  return inv;
}

}  // namespace racer_planner
