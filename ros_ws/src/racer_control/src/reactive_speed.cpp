#include "racer_control/reactive_speed.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace racer_control {

double range_based_speed(double range_m, double k_speed_per_s, double min_speed_mps,
                         double max_speed_mps) {
  if (!std::isfinite(range_m) || range_m < 0.0 || !std::isfinite(k_speed_per_s)) {
    return min_speed_mps;
  }
  // min/max rather than std::clamp: std::clamp is undefined if min > max, and the nodes
  // validate that ordering at startup but this core should not rely on it.
  return std::min(std::max(k_speed_per_s * range_m, min_speed_mps), max_speed_mps);
}

double apply_steering_slowdown(double speed_mps, double steering_rad, double k_steer,
                               double max_steering_rad) {
  if (!std::isfinite(speed_mps) || !std::isfinite(steering_rad) || !std::isfinite(k_steer) ||
      !std::isfinite(max_steering_rad) || max_steering_rad <= 0.0) {
    return 0.0;
  }
  const double lock_fraction = std::min(1.0, std::abs(steering_rad) / max_steering_rad);
  const double factor = 1.0 - k_steer * lock_fraction;
  return std::max(0.0, speed_mps * factor);
}

RollingMedian::RollingMedian(std::size_t window)
    : ring_(std::max<std::size_t>(window, 1), 0.0), scratch_(ring_.size(), 0.0) {}

double RollingMedian::push(double value) {
  if (std::isfinite(value)) {
    ring_[next_] = value;
    next_ = (next_ + 1) % ring_.size();
    count_ = std::min(count_ + 1, ring_.size());
  }
  if (count_ == 0) {
    return value;
  }
  // The held values are ring_[0, count_) until the window first fills and the whole ring after
  // that; which slot is oldest does not matter to a median.
  const auto begin = scratch_.begin();
  const auto end = begin + static_cast<std::ptrdiff_t>(count_);
  const auto mid = begin + static_cast<std::ptrdiff_t>(count_ / 2);
  std::copy(ring_.begin(), ring_.begin() + static_cast<std::ptrdiff_t>(count_), begin);
  std::nth_element(begin, mid, end);
  if (count_ % 2 == 1) {
    return *mid;
  }
  // Even count: mean of the two middle values. After nth_element the lower middle value is the
  // largest element before `mid`.
  const double lower = *std::max_element(begin, mid);
  return 0.5 * (lower + *mid);
}

void RollingMedian::reset() {
  next_ = 0;
  count_ = 0;
}

ReactiveSpeedCommand::ReactiveSpeedCommand(const ReactiveSpeedConfig& config)
    : config_(config),
      filter_(config.speed_time_constant_s),
      limiter_(config.max_acceleration_mps2) {}

double ReactiveSpeedCommand::update(double target_range_m, double steering_rad, double dt_s) {
  const double raw =
      apply_steering_slowdown(range_based_speed(target_range_m, config_.k_speed_per_s,
                                                config_.min_speed_mps, config_.max_speed_mps),
                              steering_rad, config_.k_steer, config_.max_steering_rad);
  // Skipped rather than run with tau 0: FirstOrderLowPass holds its output on a zero dt, which
  // would change the first cycle after a reset from the unfiltered chain's behaviour.
  const double smoothed = config_.speed_time_constant_s > 0.0 ? filter_.update(raw, dt_s) : raw;
  return limiter_.limit(smoothed, dt_s);
}

void ReactiveSpeedCommand::reset() {
  filter_.reset();
  limiter_ = SpeedRateLimiter(config_.max_acceleration_mps2);
}

}  // namespace racer_control
