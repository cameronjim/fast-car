#include "racer_safety/forward_sector.hpp"

#include <cmath>
#include <cstddef>
#include <limits>

namespace racer_safety {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
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

double min_corridor_distance_m(const ScanGeometry& geometry, const std::vector<float>& ranges,
                               double laser_yaw_rad, double half_angle_rad,
                               double corridor_half_width_m) {
  const bool use_corridor =
      is_trustworthy_geometry(geometry, laser_yaw_rad, half_angle_rad, corridor_half_width_m);
  double min_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    if (!is_usable_return(ranges[i], geometry)) {
      continue;
    }
    const double range = static_cast<double>(ranges[i]);
    // Garbage geometry: slant range over the whole scan (see the header).
    double distance = range;
    if (use_corridor) {
      const double laser_bearing =
          geometry.angle_min_rad + static_cast<double>(i) * geometry.angle_increment_rad;
      const double vehicle_bearing = wrap_angle_rad(laser_bearing + laser_yaw_rad);
      // Outer bound: never consider a return outside the sector, whatever the margin.
      if (std::abs(vehicle_bearing) > half_angle_rad) {
        continue;
      }
      const double x = range * std::cos(vehicle_bearing);
      const double y = range * std::sin(vehicle_bearing);
      // Primary filter: ahead of the head and inside the car-width corridor.
      if (x <= 0.0) {
        continue;
      }
      if (std::abs(y) > corridor_half_width_m) {
        continue;
      }
      distance = x;
    }
    if (distance < min_distance) {
      min_distance = distance;
    }
  }
  return min_distance;
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
