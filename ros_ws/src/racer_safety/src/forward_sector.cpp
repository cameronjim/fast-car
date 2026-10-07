#include "racer_safety/forward_sector.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>

namespace racer_safety {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kHalfPi = 0.5 * kPi;
// Two yaw values this close are the same yaw (same tolerance as racer_control).
constexpr double kYawAgreementTolerance = 1e-9;

// Explicit branches rather than one long && chain, for the same reason as gate_logic.cpp:
// the 100% branch-coverage gate needs every decision to be a literal, testable `if`.
bool is_trustworthy_geometry(const ScanGeometry& geometry, double laser_yaw_rad,
                             double half_angle_rad, double corridor_half_width_m) {
  if (!std::isfinite(geometry.angle_min_rad)) {
    return false;
  }
  if (!std::isfinite(geometry.angle_increment_rad)) {
    return false;
  }
  if (geometry.angle_increment_rad <= 0.0) {
    return false;
  }
  if (!std::isfinite(laser_yaw_rad)) {
    return false;
  }
  if (!std::isfinite(half_angle_rad)) {
    return false;
  }
  if (half_angle_rad <= 0.0) {
    return false;
  }
  if (!std::isfinite(corridor_half_width_m)) {
    return false;
  }
  if (corridor_half_width_m <= 0.0) {
    return false;
  }
  return true;
}

// The swept-path inputs (forward_sector.hpp "ARC CORRIDOR"). Same explicit-branch style.
bool is_trustworthy_path(const PathGeometry& path, double requested_steering_rad) {
  if (!std::isfinite(path.wheelbase_m)) {
    return false;
  }
  if (path.wheelbase_m <= 0.0) {
    return false;
  }
  if (!std::isfinite(path.max_steering_angle_rad)) {
    return false;
  }
  if (path.max_steering_angle_rad < 0.0) {
    return false;
  }
  // tan() changes sign at pi/2, so a larger "max" would bend the arc the wrong way.
  if (path.max_steering_angle_rad >= kHalfPi) {
    return false;
  }
  if (!std::isfinite(path.lidar_mount_x_m)) {
    return false;
  }
  if (!std::isfinite(path.lidar_mount_y_m)) {
    return false;
  }
  if (!std::isfinite(requested_steering_rad)) {
    return false;
  }
  return true;
}

double clamp_symmetric(double value, double bound) {
  if (value > bound) {
    return bound;
  }
  if (value < -bound) {
    return -bound;
  }
  return value;
}

// Straight corridor: a head-relative return (x ahead, y left) is in the path if it is ahead of
// the head and within the half width of the car's centreline. Distance: x, from the head.
std::optional<double> straight_corridor_distance_m(double x, double y, double half_width_m,
                                                   double mount_y_m) {
  if (x <= 0.0) {
    return std::nullopt;
  }
  if (std::abs(y + mount_y_m) > half_width_m) {
    return std::nullopt;
  }
  return x;
}

// Rear straight corridor (forward_sector.hpp "REAR CORRIDOR"): mirrored rear-axle frame
// coordinates (xm behind the rear axle, ym left). In the path if behind the rear bumper line and
// within the half width of the centreline. Distance: from the rear bumper line.
std::optional<double> rear_straight_corridor_distance_m(double xm, double ym, double half_width_m,
                                                        double rear_overhang_m) {
  const double behind_bumper_m = xm - rear_overhang_m;
  if (behind_bumper_m <= 0.0) {
    return std::nullopt;
  }
  if (std::abs(ym) > half_width_m) {
    return std::nullopt;
  }
  return behind_bumper_m;
}

// Arc corridor (forward_sector.hpp "ARC CORRIDOR"): the rear axle follows a circle of signed
// radius `radius_m` (left positive) about (0, radius_m) in the rear-axle frame. (xr, yr) is the
// return in that frame and (ref_x, ref_y) the reference point the distance is measured from
// (the head going forward; the rear bumper line's centre in the mirrored frame going backward,
// "REAR CORRIDOR"). Distance: the arc length the rear axle travels until the reference point
// reaches the return's arc angle.
std::optional<double> arc_corridor_distance_m(double xr, double yr, double half_width_m,
                                              double radius_m, double ref_x, double ref_y) {
  const double abs_radius_m = std::abs(radius_m);
  double turn_sign = 1.0;
  if (radius_m < 0.0) {
    turn_sign = -1.0;
  }
  const double rho_m = std::hypot(xr, yr - radius_m);
  if (std::abs(rho_m - abs_radius_m) > half_width_m) {
    return std::nullopt;
  }
  // Arc angle from the car's position (the rear axle), in the direction of travel.
  const double phi = std::atan2(xr, abs_radius_m - turn_sign * yr);
  if (phi <= 0.0) {
    return std::nullopt;
  }
  if (phi >= kHalfPi) {
    return std::nullopt;
  }
  const double phi_ref = std::atan2(ref_x, abs_radius_m - turn_sign * ref_y);
  const double arc_m = abs_radius_m * (phi - phi_ref);
  if (arc_m <= 0.0) {
    return std::nullopt;
  }
  return arc_m;
}

}  // namespace

double wrap_angle_rad(double angle_rad) {
  double wrapped = std::fmod(angle_rad + kPi, kTwoPi);
  if (wrapped < 0.0) {
    wrapped += kTwoPi;
  }
  return wrapped - kPi;
}

bool is_usable_return(float range_m, const ScanGeometry& geometry) {
  const double range = static_cast<double>(range_m);
  if (!std::isfinite(range)) {
    return false;
  }
  if (range <= 0.0) {
    return false;
  }
  if (std::isfinite(geometry.range_min_m)) {
    if (range < geometry.range_min_m) {
      return false;
    }
  }
  if (std::isfinite(geometry.range_max_m)) {
    if (range > geometry.range_max_m) {
      return false;
    }
  }
  return true;
}

double corridor_half_width_m(double chassis_width_m, double margin_m) {
  return 0.5 * chassis_width_m + margin_m;
}

namespace {

// The rear corridor's extra input. Same explicit-branch style.
bool is_trustworthy_rear_overhang(double rear_overhang_m) {
  if (!std::isfinite(rear_overhang_m)) {
    return false;
  }
  if (rear_overhang_m < 0.0) {
    return false;
  }
  return true;
}

// The in-path test for either direction (forward_sector.hpp "ARC CORRIDOR" and "REAR
// CORRIDOR"). `reverse` false is min_path_distance_m exactly; `rear_overhang_m` is only read
// when `reverse` is true.
double path_distance_m(const ScanGeometry& geometry, const std::vector<float>& ranges,
                       double laser_yaw_rad, double half_angle_rad, double corridor_half_width_m,
                       const PathGeometry& path, double requested_steering_rad, bool reverse,
                       double rear_overhang_m) {
  bool use_path = false;
  if (is_trustworthy_geometry(geometry, laser_yaw_rad, half_angle_rad, corridor_half_width_m)) {
    if (is_trustworthy_path(path, requested_steering_rad)) {
      if (!reverse) {
        use_path = true;
      } else if (is_trustworthy_rear_overhang(rear_overhang_m)) {
        use_path = true;
      }
    }
  }
  // Only read when use_path is true, so the steering and the wheelbase are finite here.
  bool straight = true;
  double radius_m = std::numeric_limits<double>::infinity();
  if (use_path) {
    const double delta_rad = clamp_symmetric(requested_steering_rad, path.max_steering_angle_rad);
    if (std::abs(delta_rad) >= kStraightSteeringEpsilonRad) {
      straight = false;
      radius_m = path.wheelbase_m / std::tan(delta_rad);
    }
  }
  double min_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    if (!is_usable_return(ranges[i], geometry)) {
      continue;
    }
    const double range = static_cast<double>(ranges[i]);
    // Garbage input: slant range over the whole scan (see the header).
    double distance = range;
    if (use_path) {
      const double laser_bearing =
          geometry.angle_min_rad + static_cast<double>(i) * geometry.angle_increment_rad;
      const double vehicle_bearing = wrap_angle_rad(laser_bearing + laser_yaw_rad);
      // Outer bound: never consider a return outside the sector, whatever the margin. The rear
      // sector is centred on the car's -x axis.
      double off_axis_rad = vehicle_bearing;
      if (reverse) {
        off_axis_rad = wrap_angle_rad(vehicle_bearing - kPi);
      }
      if (std::abs(off_axis_rad) > half_angle_rad) {
        continue;
      }
      const double x = range * std::cos(vehicle_bearing);
      const double y = range * std::sin(vehicle_bearing);
      std::optional<double> in_path;
      if (!reverse) {
        if (straight) {
          in_path = straight_corridor_distance_m(x, y, corridor_half_width_m, path.lidar_mount_y_m);
        } else {
          in_path = arc_corridor_distance_m(x + path.lidar_mount_x_m, y + path.lidar_mount_y_m,
                                            corridor_half_width_m, radius_m, path.lidar_mount_x_m,
                                            path.lidar_mount_y_m);
        }
      } else {
        // Mirrored rear-axle frame: behind the rear axle is +xm (forward_sector.hpp "REAR
        // CORRIDOR"); the reference point is the rear bumper line's centre.
        const double xm = -(x + path.lidar_mount_x_m);
        const double ym = y + path.lidar_mount_y_m;
        if (straight) {
          in_path =
              rear_straight_corridor_distance_m(xm, ym, corridor_half_width_m, rear_overhang_m);
        } else {
          in_path = arc_corridor_distance_m(xm, ym, corridor_half_width_m, radius_m,
                                            rear_overhang_m, 0.0);
        }
      }
      if (!in_path.has_value()) {
        continue;
      }
      distance = *in_path;
    }
    if (distance < min_distance) {
      min_distance = distance;
    }
  }
  return min_distance;
}

}  // namespace

double min_path_distance_m(const ScanGeometry& geometry, const std::vector<float>& ranges,
                           double laser_yaw_rad, double half_angle_rad,
                           double corridor_half_width_m, const PathGeometry& path,
                           double requested_steering_rad) {
  return path_distance_m(geometry, ranges, laser_yaw_rad, half_angle_rad, corridor_half_width_m,
                         path, requested_steering_rad, /*reverse=*/false,
                         /*rear_overhang_m=*/0.0);
}

double min_rear_path_distance_m(const ScanGeometry& geometry, const std::vector<float>& ranges,
                                double laser_yaw_rad, double half_angle_rad,
                                double corridor_half_width_m, const PathGeometry& path,
                                double rear_overhang_m, double requested_steering_rad) {
  return path_distance_m(geometry, ranges, laser_yaw_rad, half_angle_rad, corridor_half_width_m,
                         path, requested_steering_rad, /*reverse=*/true, rear_overhang_m);
}

double min_corridor_distance_m(const ScanGeometry& geometry, const std::vector<float>& ranges,
                               double laser_yaw_rad, double half_angle_rad,
                               double corridor_half_width_m) {
  // Any valid path model with a straight request; the wheelbase is irrelevant on a straight.
  PathGeometry straight_path;
  straight_path.wheelbase_m = 1.0;
  return min_path_distance_m(geometry, ranges, laser_yaw_rad, half_angle_rad, corridor_half_width_m,
                             straight_path, 0.0);
}

std::optional<double> resolve_laser_yaw_rad(std::optional<double> binding_rad, double parameter_rad,
                                            bool from_vehicle_params) {
  if (!std::isfinite(parameter_rad)) {
    return std::nullopt;
  }
  if (!from_vehicle_params) {
    return parameter_rad;
  }
  if (!binding_rad.has_value()) {
    return parameter_rad;
  }
  if (!std::isfinite(*binding_rad)) {
    return std::nullopt;
  }
  if (parameter_rad == 0.0) {
    return *binding_rad;
  }
  if (std::abs(parameter_rad - *binding_rad) <= kYawAgreementTolerance) {
    return *binding_rad;
  }
  return std::nullopt;
}

}  // namespace racer_safety
