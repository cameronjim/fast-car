#include "racer_control/laser_scan.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace racer_control {

namespace {
constexpr double kTwoPi = 2.0 * M_PI;

bool is_free_space(float r, const ScanInput& scan) {
  if (std::isnan(r)) {
    return false;
  }
  if (std::isinf(r)) {
    return r > 0.0f;
  }
  return static_cast<double>(r) > scan.range_max;
}

bool is_in_range(float r, const ScanInput& scan) {
  if (!std::isfinite(r) || r <= 0.0f) {
    return false;
  }
  const double rd = static_cast<double>(r);
  return rd >= scan.range_min && rd <= scan.range_max;
}
}  // namespace

double wrap_angle(double angle_rad) {
  double wrapped = std::fmod(angle_rad + M_PI, kTwoPi);
  if (wrapped < 0.0) {
    wrapped += kTwoPi;
  }
  return wrapped - M_PI;
}

bool is_usable(const ScanInput& scan) {
  return !scan.ranges.empty() && std::isfinite(scan.angle_increment) &&
         scan.angle_increment > 0.0 && std::isfinite(scan.angle_min) &&
         std::isfinite(scan.range_min) && std::isfinite(scan.range_max) &&
         scan.range_max > scan.range_min;
}

bool is_full_circle(const ScanInput& scan) {
  const double coverage = static_cast<double>(scan.ranges.size()) * scan.angle_increment;
  return coverage >= kTwoPi - 0.5 * scan.angle_increment;
}

double laser_bearing_of_index(const ScanInput& scan, std::size_t index) {
  return scan.angle_min + static_cast<double>(index) * scan.angle_increment;
}

std::optional<std::size_t> index_for_laser_bearing(const ScanInput& scan,
                                                   double laser_bearing_rad) {
  if (!is_usable(scan) || !std::isfinite(laser_bearing_rad)) {
    return std::nullopt;
  }
  const std::size_t n = scan.ranges.size();
  // Offset from angle_min, counter-clockwise, in [0, 2 pi).
  double offset = std::fmod(laser_bearing_rad - scan.angle_min, kTwoPi);
  if (offset < 0.0) {
    offset += kTwoPi;
  }
  const double half_step = 0.5 * scan.angle_increment;
  // A bearing just clockwise of angle_min lands near 2 pi; it belongs to ray 0.
  if (kTwoPi - offset <= half_step) {
    return 0;
  }
  const auto index = static_cast<std::size_t>(std::llround(offset / scan.angle_increment));
  if (index < n) {
    return index;
  }
  if (is_full_circle(scan)) {
    return index % n;
  }
  return std::nullopt;
}

std::optional<std::size_t> index_for_vehicle_bearing(const ScanInput& scan,
                                                     double vehicle_bearing_rad,
                                                     double laser_yaw_offset_rad) {
  return index_for_laser_bearing(scan, vehicle_bearing_rad - laser_yaw_offset_rad);
}

std::size_t sanitize_ranges(const ScanInput& scan, double clip_max_range_m,
                            std::vector<double>& out) {
  const std::size_t n = scan.ranges.size();
  out.resize(n);
  constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();

  // Pass 1: classify. Valid rays get their final value; invalid rays are marked with NaN for
  // the fill passes below (never returned: every NaN is overwritten before returning).
  std::size_t valid_count = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const float r = scan.ranges[i];
    if (is_in_range(r, scan)) {
      out[i] = std::min(static_cast<double>(r), clip_max_range_m);
      ++valid_count;
    } else if (is_free_space(r, scan)) {
      out[i] = clip_max_range_m;
      ++valid_count;
    } else {
      out[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }
  if (valid_count == 0) {
    for (std::size_t i = 0; i < n; ++i) {
      out[i] = 0.0;
    }
    return 0;
  }
  if (valid_count == n) {
    return valid_count;
  }

  // Pass 2: fill each run of invalid rays from its nearest valid neighbours. Each run is
  // scanned once to find its right neighbour and once to fill it, so the total work is O(n).
  std::size_t last_valid = kNone;
  for (std::size_t i = 0; i < n; ++i) {
    if (!std::isnan(out[i])) {
      last_valid = i;
      continue;
    }
    // Find next valid to the right.
    std::size_t next_valid = kNone;
    for (std::size_t j = i + 1; j < n; ++j) {
      if (!std::isnan(out[j])) {
        next_valid = j;
        break;
      }
    }
    // Fill the whole invalid run [i, next_valid) in one go, so the total work stays O(n).
    const std::size_t run_end = (next_valid == kNone) ? n : next_valid;
    for (std::size_t k = i; k < run_end; ++k) {
      const std::size_t dist_left = (last_valid == kNone) ? kNone : k - last_valid;
      const std::size_t dist_right = (next_valid == kNone) ? kNone : next_valid - k;
      double fill;
      if (dist_left < dist_right) {
        fill = out[last_valid];
      } else if (dist_right < dist_left) {
        fill = out[next_valid];
      } else {
        fill = std::min(out[last_valid], out[next_valid]);
      }
      out[k] = fill;
    }
    // Rays in the run are now filled (non-NaN); skip to the end of the run. The loop's ++i
    // lands on next_valid, which updates last_valid.
    if (run_end == n) {
      break;
    }
    i = run_end - 1;
  }
  return valid_count;
}

std::optional<double> resolve_laser_yaw_offset(std::optional<double> binding_rad,
                                               double parameter_rad, bool from_vehicle_params) {
  if (!std::isfinite(parameter_rad)) {
    return std::nullopt;
  }
  if (!from_vehicle_params || !binding_rad.has_value()) {
    return parameter_rad;
  }
  if (!std::isfinite(*binding_rad)) {
    return std::nullopt;
  }
  if (parameter_rad == 0.0 || std::abs(parameter_rad - *binding_rad) <= 1e-9) {
    return *binding_rad;
  }
  return std::nullopt;
}

}  // namespace racer_control
