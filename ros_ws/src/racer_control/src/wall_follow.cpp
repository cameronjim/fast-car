#include "racer_control/wall_follow.hpp"

#include <cmath>

namespace racer_control {

namespace {
bool is_valid_return(double r, const ScanInput& scan) {
  return std::isfinite(r) && r > 0.0 && r >= scan.range_min && r <= scan.range_max;
}
}  // namespace

bool wall_rays_valid(const WallFollowConfig& config) {
  const double a = config.ray_a_bearing_rad;
  const double b = config.ray_b_bearing_rad;
  if (!std::isfinite(a) || !std::isfinite(b) || a == 0.0 || b == 0.0) {
    return false;
  }
  if ((a > 0.0) != (b > 0.0)) {
    return false;
  }
  const double theta = std::abs(a - b);
  return std::abs(b) > std::abs(a) && theta < M_PI;
}

double wall_steering_sign(const WallFollowConfig& config) {
  return config.ray_b_bearing_rad < 0.0 ? 1.0 : -1.0;
}

std::optional<double> range_at_vehicle_bearing(const ScanInput& scan, double vehicle_bearing_rad,
                                               double laser_yaw_offset_rad) {
  const auto index = index_for_vehicle_bearing(scan, vehicle_bearing_rad, laser_yaw_offset_rad);
  if (!index) {
    return std::nullopt;
  }
  return static_cast<double>(scan.ranges[*index]);
}

std::optional<WallMeasurement> measure_wall(const ScanInput& scan, const WallFollowConfig& config) {
  if (!wall_rays_valid(config) || !is_usable(scan)) {
    return std::nullopt;
  }
  const auto range_a =
      range_at_vehicle_bearing(scan, config.ray_a_bearing_rad, config.laser_yaw_offset_rad);
  const auto range_b =
      range_at_vehicle_bearing(scan, config.ray_b_bearing_rad, config.laser_yaw_offset_rad);
  if (!range_a || !range_b || !is_valid_return(*range_a, scan) ||
      !is_valid_return(*range_b, scan)) {
    return std::nullopt;
  }
  const double a = *range_a;
  const double b = *range_b;
  const double theta = std::abs(config.ray_a_bearing_rad - config.ray_b_bearing_rad);

  WallMeasurement m;
  m.alpha_rad = std::atan((a * std::cos(theta) - b) / (a * std::sin(theta)));
  m.distance_m = b * std::cos(m.alpha_rad);
  m.lookahead_distance_m = m.distance_m + config.lookahead_m * std::sin(m.alpha_rad);
  const double error = config.target_distance_m - m.lookahead_distance_m;
  m.error_m = std::abs(error) < config.deadband_m ? 0.0 : error;
  return m;
}

}  // namespace racer_control
