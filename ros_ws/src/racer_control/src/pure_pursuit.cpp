#include "racer_control/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>

namespace racer_control {

double PurePursuitController::lookahead_distance_m(double curvature_1pm) const {
  const double k = std::abs(curvature_1pm);
  const double frac = std::min(k / config_.lookahead_curvature_ref_1pm, 1.0);
  return config_.lookahead_max_m - (config_.lookahead_max_m - config_.lookahead_min_m) * frac;
}

PurePursuitCommand PurePursuitController::compute_command(const Raceline& raceline, double x_m,
                                                          double y_m, double yaw_rad) const {
  const std::size_t nearest = raceline.nearest_index(x_m, y_m);
  const RacelinePoint& nearest_point = raceline.at(nearest);

  const double lookahead_m = lookahead_distance_m(nearest_point.curvature_1pm);
  const std::size_t target_index = raceline.advance_to_lookahead(nearest, x_m, y_m, lookahead_m);
  const RacelinePoint& target = raceline.at(target_index);

  // World-frame vector to the lookahead target, rotated into the vehicle body frame
  // (REP-103: x forward, y left) by -yaw_rad.
  const double dx = target.x_m - x_m;
  const double dy = target.y_m - y_m;
  const double cos_neg_yaw = std::cos(-yaw_rad);
  const double sin_neg_yaw = std::sin(-yaw_rad);
  const double lx = cos_neg_yaw * dx - sin_neg_yaw * dy;
  const double ly = sin_neg_yaw * dx + cos_neg_yaw * dy;

  const double lookahead_dist2 = lx * lx + ly * ly;
  double commanded_curvature_1pm = 0.0;
  if (lookahead_dist2 > 1e-9) {
    // Standard pure-pursuit curvature: kappa = 2*y / L^2, with y the lateral offset of the
    // lookahead point in the vehicle frame and L its distance. A target to the LEFT
    // (ly > 0) gives positive curvature, matching the left-positive sign convention.
    commanded_curvature_1pm = 2.0 * ly / lookahead_dist2;
  }

  double steering_angle_rad = std::atan(commanded_curvature_1pm * config_.wheelbase_m);
  steering_angle_rad = std::clamp(steering_angle_rad, -config_.max_steering_angle_rad,
                                  config_.max_steering_angle_rad);

  return PurePursuitCommand{steering_angle_rad, nearest_point.target_speed_mps};
}

double yaw_from_quaternion(double w, double x, double y, double z) {
  // Standard planar-yaw-from-quaternion formula (equivalent to tf2::getYaw / the yaw
  // component of the ZYX Euler decomposition); REP-103 yaw is counter-clockwise positive
  // about z, which this formula already respects.
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace racer_control
