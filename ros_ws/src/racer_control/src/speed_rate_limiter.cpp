#include "racer_control/speed_rate_limiter.hpp"

#include <cmath>

namespace racer_control {

double SpeedRateLimiter::limit(double raw_speed_mps, double dt_s) {
  const double safe_dt_s = (std::isfinite(dt_s) && dt_s > 0.0) ? dt_s : 0.0;
  const double delta = raw_speed_mps - previous_speed_mps_;

  double result = raw_speed_mps;
  if (delta > 0.0) {
    // Only accelerating (increasing commanded speed) is rate-limited; decelerating/braking
    // is never rate-limited (same asymmetry as racer_safety::SafetyGateLogic::rate_limit --
    // claude-docs/05-safety.md never asks for slower braking).
    const double max_delta = max_acceleration_mps2_ * safe_dt_s;
    result = delta > max_delta ? previous_speed_mps_ + max_delta : raw_speed_mps;
  }

  previous_speed_mps_ = result;
  return result;
}

}  // namespace racer_control
