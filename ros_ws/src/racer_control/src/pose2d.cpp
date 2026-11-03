#include "racer_control/pose2d.hpp"

#include <cmath>

#include "racer_control/laser_scan.hpp"

namespace racer_control {

Pose2D compose(const Pose2D& parent_from_mid, const Pose2D& mid_from_child) {
  const double c = std::cos(parent_from_mid.yaw_rad);
  const double s = std::sin(parent_from_mid.yaw_rad);
  Pose2D out;
  out.x_m = parent_from_mid.x_m + c * mid_from_child.x_m - s * mid_from_child.y_m;
  out.y_m = parent_from_mid.y_m + s * mid_from_child.x_m + c * mid_from_child.y_m;
  out.yaw_rad = wrap_angle(parent_from_mid.yaw_rad + mid_from_child.yaw_rad);
  return out;
}

}  // namespace racer_control
