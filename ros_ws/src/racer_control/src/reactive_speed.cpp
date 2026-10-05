#include "racer_control/reactive_speed.hpp"

#include <algorithm>
#include <cmath>

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

}  // namespace racer_control
