#include "racer_state/wheel_twist.hpp"

#include <cmath>
#include <stdexcept>

namespace racer_state {

namespace {

bool valid_std(double value) { return std::isfinite(value) && value >= 0.0; }

}  // namespace

void validate(const WheelTwistConfig& config) {
  if (!std::isfinite(config.wheelbase_m) || config.wheelbase_m <= 0.0) {
    throw std::invalid_argument("WheelTwistConfig: wheelbase_m must be positive");
  }
  if (!valid_std(config.speed_std_floor_mps) || !valid_std(config.speed_std_fraction) ||
      !valid_std(config.steering_std_rad) || !valid_std(config.stale_steering_std_rad) ||
      !valid_std(config.yaw_rate_std_floor_rad_per_s) || !valid_std(config.lateral_std_mps)) {
    throw std::invalid_argument("WheelTwistConfig: every std must be finite and >= 0");
  }
}

std::optional<WheelTwist> compute_wheel_twist(double speed_mps, double steering_rad,
                                              bool steering_fresh, const WheelTwistConfig& config) {
  if (!std::isfinite(speed_mps) || !std::isfinite(steering_rad)) {
    return std::nullopt;
  }
  const double length = config.wheelbase_m;
  const double tan_delta = std::tan(steering_rad);
  const double cos_delta = std::cos(steering_rad);

  WheelTwist twist;
  twist.vx_mps = speed_mps;
  twist.vy_mps = 0.0;
  twist.wz_rad_per_s = speed_mps * tan_delta / length;

  const double speed_std =
      config.speed_std_floor_mps + config.speed_std_fraction * std::abs(speed_mps);
  twist.var_vx = speed_std * speed_std;
  twist.var_vy = config.lateral_std_mps * config.lateral_std_mps;

  const double steering_std =
      steering_fresh ? config.steering_std_rad : config.stale_steering_std_rad;
  const double d_wz_d_delta = speed_mps / (length * cos_delta * cos_delta);
  const double d_wz_d_v = tan_delta / length;
  twist.var_wz = d_wz_d_delta * d_wz_d_delta * steering_std * steering_std +
                 d_wz_d_v * d_wz_d_v * twist.var_vx +
                 config.yaw_rate_std_floor_rad_per_s * config.yaw_rate_std_floor_rad_per_s;
  return twist;
}

WheelTwistEstimator::WheelTwistEstimator(const ServoModelConfig& servo,
                                         const WheelTwistConfig& twist, double drive_timeout_s)
    : servo_(servo), twist_config_(twist), drive_timeout_s_(drive_timeout_s) {
  validate(twist_config_);
  if (!std::isfinite(drive_timeout_s) || drive_timeout_s <= 0.0) {
    throw std::invalid_argument("WheelTwistEstimator: drive_timeout_s must be positive");
  }
}

void WheelTwistEstimator::on_drive(double steering_rad, double steady_s) {
  if (!std::isfinite(steering_rad) || !std::isfinite(steady_s)) {
    return;
  }
  commanded_steering_rad_ = steering_rad;
  last_drive_steady_s_ = steady_s;
  has_drive_ = true;
}

bool WheelTwistEstimator::steering_stale(double steady_s) const {
  return !has_drive_ || !(steady_s - last_drive_steady_s_ <= drive_timeout_s_);
}

std::optional<WheelTwist> WheelTwistEstimator::on_wheel_speed(double speed_mps, double stamp_s,
                                                              double steady_s) {
  if (!std::isfinite(speed_mps) || !std::isfinite(stamp_s)) {
    return std::nullopt;
  }
  double dt_s = 0.0;
  if (has_stamp_) {
    dt_s = stamp_s - last_stamp_s_;
  }
  // A stamp that goes backwards neither advances the servo nor becomes the new reference, so a
  // single bad stamp cannot make the next good one look like a long gap.
  if (!has_stamp_ || dt_s > 0.0) {
    last_stamp_s_ = stamp_s;
    has_stamp_ = true;
  }
  // With no /drive yet the wheels are assumed straight (the servo's power-on neutral); the
  // stale variance says how little that is worth.
  servo_.step(commanded_steering_rad_, dt_s);
  return compute_wheel_twist(speed_mps, servo_.angle_rad(), !steering_stale(steady_s),
                             twist_config_);
}

}  // namespace racer_state
