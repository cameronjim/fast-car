#include "racer_state/servo_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace racer_state {

namespace {

bool finite(double value) { return std::isfinite(value); }

}  // namespace

ServoModel::ServoModel(const ServoModelConfig& config) : config_(config) {
  if (!finite(config.min_angle_rad) || !finite(config.max_angle_rad) ||
      !finite(config.min_rate_rad_per_s) || !finite(config.max_rate_rad_per_s)) {
    throw std::invalid_argument("ServoModel: steering limits must be finite");
  }
  if (config.min_angle_rad >= config.max_angle_rad) {
    throw std::invalid_argument("ServoModel: min_angle_rad must be below max_angle_rad");
  }
  if (config.min_rate_rad_per_s >= 0.0 || config.max_rate_rad_per_s <= 0.0) {
    throw std::invalid_argument(
        "ServoModel: min_rate_rad_per_s must be negative and max_rate_rad_per_s positive");
  }
  if (config.time_constant_s.has_value() &&
      (!finite(*config.time_constant_s) || *config.time_constant_s <= 0.0)) {
    throw std::invalid_argument("ServoModel: time_constant_s must be positive when set");
  }
  reset(0.0);
}

double ServoModel::clamp_angle(double angle_rad) const {
  return std::clamp(angle_rad, config_.min_angle_rad, config_.max_angle_rad);
}

void ServoModel::reset(double angle_rad) {
  angle_rad_ = finite(angle_rad) ? clamp_angle(angle_rad) : 0.0;
  target_rad_ = angle_rad_;
}

double ServoModel::step(double commanded_rad, double dt_s) {
  if (finite(commanded_rad)) {
    target_rad_ = clamp_angle(commanded_rad);
  }
  if (!finite(dt_s) || dt_s <= 0.0) {
    return angle_rad_;
  }
  double desired = target_rad_;
  if (config_.time_constant_s.has_value()) {
    // Exact discretisation of d(angle)/dt = (target - angle) / tau over dt.
    const double decay = std::exp(-dt_s / *config_.time_constant_s);
    desired = target_rad_ + (angle_rad_ - target_rad_) * decay;
  }
  const double change = std::clamp(desired - angle_rad_, config_.min_rate_rad_per_s * dt_s,
                                   config_.max_rate_rad_per_s * dt_s);
  angle_rad_ = clamp_angle(angle_rad_ + change);
  return angle_rad_;
}

}  // namespace racer_state
