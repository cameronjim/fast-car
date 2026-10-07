// Parking-slot detection and lane measurement (see include/racer_control/park_slot.hpp).
#include "racer_control/park_slot.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace racer_control {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr std::size_t kEdgeBins = 3;
constexpr std::size_t kMinRowFitBins = 4;

bool usable_return(float range, const ScanInput& scan) {
  const double r = static_cast<double>(range);
  return std::isfinite(r) && r > 0.0 && r >= scan.range_min && r <= scan.range_max;
}

// Rear-axle frame position of ray i (x forward, y left).
void ray_position(const ScanInput& scan, std::size_t i, double yaw, double mount_x, double mount_y,
                  double& x, double& y) {
  const double r = static_cast<double>(scan.ranges[i]);
  const double bearing = laser_bearing_of_index(scan, i) + yaw;
  x = r * std::cos(bearing) + mount_x;
  y = r * std::sin(bearing) + mount_y;
}

double median_of(std::vector<double>& values) {
  std::sort(values.begin(), values.end());
  const std::size_t n = values.size();
  if (n % 2 == 1) {
    return values[n / 2];
  }
  return 0.5 * (values[n / 2 - 1] + values[n / 2]);
}

}  // namespace

double side_sign(ParkSide side) { return side == ParkSide::kLeft ? 1.0 : -1.0; }

const char* park_side_name(ParkSide side) { return side == ParkSide::kLeft ? "left" : "right"; }

const char* slot_reject_name(SlotReject reject) {
  switch (reject) {
    case SlotReject::kNone:
      return "none";
    case SlotReject::kBadInput:
      return "unusable scan or configuration";
    case SlotReject::kNoReturns:
      return "no returns on the parking side";
    case SlotReject::kNoPocket:
      return "no pocket";
    case SlotReject::kTooShort:
      return "pocket too short";
    case SlotReject::kTooShallow:
      return "pocket too shallow";
    case SlotReject::kRowAngle:
      return "row not parallel to the car";
  }
  return "unknown";
}

bool is_usable_detector_config(const SlotDetectorConfig& c) {
  const double values[] = {c.laser_yaw_offset_rad, c.lidar_mount_x_m, c.lidar_mount_y_m,
                           c.lateral_min_m,        c.lateral_max_m,   c.window_back_m,
                           c.window_ahead_m,       c.bin_m,           c.jump_min_m,
                           c.depth_min_m,          c.length_min_m,    c.gap_fill_m,
                           c.max_row_angle_rad};
  for (double v : values) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return c.bin_m > 0.0 && c.lateral_min_m >= 0.0 && c.lateral_max_m > c.lateral_min_m &&
         c.window_back_m + c.window_ahead_m > c.bin_m && c.jump_min_m > 0.0 &&
         c.depth_min_m > 0.0 && c.length_min_m > 0.0 && c.gap_fill_m >= 0.0 &&
         c.max_row_angle_rad > 0.0;
}

SlotDetection SlotDetector::detect(const ScanInput& scan) {
  SlotDetection out;
  const SlotDetectorConfig& c = config_;
  if (!is_usable(scan) || !is_usable_detector_config(c)) {
    out.reject = SlotReject::kBadInput;
    return out;
  }
  const double sign = side_sign(c.side);
  const double x_lo = -c.window_back_m;
  const double x_hi = c.window_ahead_m;
  const std::size_t n = static_cast<std::size_t>(std::ceil((x_hi - x_lo) / c.bin_m));
  bins_.assign(n, kInf);

  bool any = false;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    if (!usable_return(scan.ranges[i], scan)) {
      continue;
    }
    double x = 0.0;
    double y = 0.0;
    ray_position(scan, i, c.laser_yaw_offset_rad, c.lidar_mount_x_m, c.lidar_mount_y_m, x, y);
    if (x < x_lo || x >= x_hi) {
      continue;
    }
    const double lateral = sign * y;
    const double opposite = -lateral;
    if (lateral >= c.lateral_min_m && lateral <= c.lateral_max_m) {
      const std::size_t b = std::min(n - 1, static_cast<std::size_t>((x - x_lo) / c.bin_m));
      bins_[b] = std::min(bins_[b], lateral);
      any = true;
    } else if (opposite >= c.lateral_min_m && opposite <= c.lateral_max_m) {
      if (!out.far_side_lateral_m || opposite < *out.far_side_lateral_m) {
        out.far_side_lateral_m = opposite;
      }
    }
  }
  if (!any) {
    out.reject = SlotReject::kNoReturns;
    return out;
  }

  // 1. Short holes between occupied bins take the larger neighbour.
  const std::size_t fill_bins = static_cast<std::size_t>(std::floor(c.gap_fill_m / c.bin_m + 1e-9));
  std::size_t i = 0;
  while (i < n) {
    if (std::isfinite(bins_[i])) {
      ++i;
      continue;
    }
    std::size_t j = i;
    while (j < n && !std::isfinite(bins_[j])) {
      ++j;
    }
    // Empty run [i, j).
    if (i > 0 && j < n && j - i <= fill_bins) {
      const double fill = std::max(bins_[i - 1], bins_[j]);
      for (std::size_t k = i; k < j; ++k) {
        bins_[k] = fill;
      }
    }
    i = j;
  }

  // 2. Reference face: 25th percentile of the occupied bins.
  occupied_.clear();
  for (double v : bins_) {
    if (std::isfinite(v)) {
      occupied_.push_back(v);
    }
  }
  std::sort(occupied_.begin(), occupied_.end());
  const double face_ref = occupied_[(occupied_.size() - 1) / 4];
  const double candidate_level = face_ref + c.jump_min_m;
  auto is_candidate = [&](std::size_t k) { return bins_[k] >= candidate_level; };

  // 6 (computed first, used in 5). Row line through the face bins.
  double sx = 0.0;
  double sy = 0.0;
  double sxx = 0.0;
  double sxy = 0.0;
  std::size_t face_count = 0;
  for (std::size_t k = 0; k < n; ++k) {
    if (std::isfinite(bins_[k]) && !is_candidate(k)) {
      const double x = x_lo + (static_cast<double>(k) + 0.5) * c.bin_m;
      sx += x;
      sy += bins_[k];
      sxx += x * x;
      sxy += x * bins_[k];
      ++face_count;
    }
  }
  double lateral_slope = 0.0;
  if (face_count >= kMinRowFitBins) {
    const double cnt = static_cast<double>(face_count);
    const double denom = cnt * sxx - sx * sx;
    if (std::abs(denom) > 1e-12) {
      lateral_slope = (cnt * sxy - sx * sy) / denom;
    }
  }
  // Lateral distance grows along +x on the parking side: on the left that is +y (heading +),
  // on the right -y (heading -).
  out.row_angle_rad = std::atan(sign * lateral_slope);
  if (face_count >= kMinRowFitBins && std::abs(out.row_angle_rad) > c.max_row_angle_rad) {
    out.reject = SlotReject::kRowAngle;
    return out;
  }

  // 3-5. Candidate runs.
  bool have_best = false;
  std::vector<double> edge;
  edge.reserve(kEdgeBins);
  i = 0;
  while (i < n) {
    if (!is_candidate(i)) {
      ++i;
      continue;
    }
    std::size_t j = i;
    while (j < n && is_candidate(j)) {
      ++j;
    }
    // Candidate run [i, j).
    const std::size_t first = i;
    const std::size_t last = j - 1;
    i = j;
    if (first == 0 || last == n - 1) {
      continue;  // not bounded: its end is outside the window
    }
    edge.clear();
    for (std::size_t k = first; k > 0 && edge.size() < kEdgeBins; --k) {
      if (is_candidate(k - 1)) {
        break;
      }
      edge.push_back(bins_[k - 1]);
    }
    const double before = median_of(edge);
    edge.clear();
    for (std::size_t k = last + 1; k < n && edge.size() < kEdgeBins; ++k) {
      if (is_candidate(k)) {
        break;
      }
      edge.push_back(bins_[k]);
    }
    const double after = median_of(edge);
    const double row = std::max(before, after);
    double inside = kInf;
    bool seen = false;
    for (std::size_t k = first; k <= last; ++k) {
      if (std::isfinite(bins_[k])) {
        seen = true;
        inside = std::min(inside, bins_[k]);
      }
    }
    const double back = std::min(inside, c.lateral_max_m);
    const double depth = back - row;
    const double near_x = x_lo + static_cast<double>(first) * c.bin_m;
    const double far_x = x_lo + static_cast<double>(last + 1) * c.bin_m;
    const double length = far_x - near_x;
    const bool accept = depth >= c.depth_min_m && length >= c.length_min_m;
    if (accept || !have_best || length > out.candidate_length_m) {
      out.near_x_m = near_x;
      out.far_x_m = far_x;
      out.row_lateral_m = row;
      out.back_lateral_m = back;
      out.depth_m = depth;
      out.length_m = length;
      out.back_seen = seen;
      out.candidate_length_m = length;
      out.candidate_depth_m = depth;
      have_best = true;
    }
    if (accept) {
      out.found = true;
      out.reject = SlotReject::kNone;
      return out;
    }
  }
  if (!have_best) {
    out.reject = SlotReject::kNoPocket;
  } else if (out.candidate_length_m < c.length_min_m) {
    out.reject = SlotReject::kTooShort;
  } else {
    out.reject = SlotReject::kTooShallow;
  }
  return out;
}

LaneMeasurement measure_lane(const ScanInput& scan, double laser_yaw_offset_rad,
                             double lidar_mount_x_m, double lidar_mount_y_m, double x_min_m,
                             double x_max_m, double lateral_min_m, double lateral_max_m) {
  LaneMeasurement out;
  if (!is_usable(scan)) {
    return out;
  }
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    if (!usable_return(scan.ranges[i], scan)) {
      continue;
    }
    double x = 0.0;
    double y = 0.0;
    ray_position(scan, i, laser_yaw_offset_rad, lidar_mount_x_m, lidar_mount_y_m, x, y);
    if (x < x_min_m || x > x_max_m) {
      continue;
    }
    const double a = std::abs(y);
    if (a < lateral_min_m || a > lateral_max_m) {
      continue;
    }
    std::optional<double>& side = y > 0.0 ? out.left_m : out.right_m;
    if (!side || a < *side) {
      side = a;
    }
  }
  return out;
}

}  // namespace racer_control
