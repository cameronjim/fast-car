#include "racer_control/gap_follow.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace racer_control {

void extend_disparities(const std::vector<double>& ranges, double disparity_threshold_m,
                        double half_width_m, double angle_increment, std::vector<double>& out) {
  out.assign(ranges.begin(), ranges.end());
  const std::size_t n = ranges.size();
  if (n < 2 || !std::isfinite(angle_increment) || angle_increment <= 0.0 ||
      !std::isfinite(half_width_m) || half_width_m <= 0.0) {
    return;
  }
  for (std::size_t i = 0; i + 1 < n; ++i) {
    if (std::abs(ranges[i] - ranges[i + 1]) <= disparity_threshold_m) {
      continue;
    }
    const bool near_on_low_side = ranges[i] < ranges[i + 1];
    const double near = std::max(0.0, near_on_low_side ? ranges[i] : ranges[i + 1]);
    // atan2(w, 0) = pi/2: an obstacle at the lens blocks a full quarter turn.
    const double bubble_rad = std::atan2(half_width_m, near);
    const auto bubble_rays = static_cast<std::int64_t>(std::ceil(bubble_rad / angle_increment));
    const std::int64_t start =
        near_on_low_side ? static_cast<std::int64_t>(i + 1) : static_cast<std::int64_t>(i);
    const std::int64_t step = near_on_low_side ? 1 : -1;
    for (std::int64_t s = 0; s < bubble_rays; ++s) {
      const std::int64_t k = start + step * s;
      if (k < 0 || k >= static_cast<std::int64_t>(n)) {
        break;
      }
      const auto ku = static_cast<std::size_t>(k);
      out[ku] = std::min(out[ku], near);
    }
  }
}

namespace {

// Maps a cone-local position j (0-based, ascending laser bearing) to a scan index.
struct Cone {
  std::size_t n;
  std::int64_t forward;
  std::int64_t offset_lo;  // offset of j = 0 relative to the forward ray (<= 0)
  std::int64_t offset_hi;  // >= 0
  bool wraps;

  std::size_t length() const { return static_cast<std::size_t>(offset_hi - offset_lo + 1); }
  std::int64_t offset_of(std::size_t j) const { return offset_lo + static_cast<std::int64_t>(j); }
  std::size_t index_of(std::size_t j) const {
    std::int64_t idx = forward + offset_of(j);
    if (wraps) {
      const auto nn = static_cast<std::int64_t>(n);
      idx = ((idx % nn) + nn) % nn;
    }
    return static_cast<std::size_t>(idx);
  }
};

}  // namespace

std::optional<GapSelection> select_gap(const ScanInput& geometry, const std::vector<double>& ranges,
                                       double free_space_threshold_m, double cone_half_angle_rad,
                                       double laser_yaw_offset_rad, GapTarget target,
                                       const GapPreference& preference) {
  const auto unit_fraction = [](double v) { return std::isfinite(v) && v >= 0.0 && v <= 1.0; };
  if (!is_usable(geometry) || ranges.size() != geometry.ranges.size() ||
      !std::isfinite(cone_half_angle_rad) || cone_half_angle_rad < 0.0 ||
      !unit_fraction(preference.forward_preference) || !unit_fraction(preference.switch_margin)) {
    return std::nullopt;
  }
  const auto forward = index_for_vehicle_bearing(geometry, 0.0, laser_yaw_offset_rad);
  if (!forward) {
    return std::nullopt;
  }
  const std::size_t n = ranges.size();
  const auto nn = static_cast<std::int64_t>(n);
  const auto f = static_cast<std::int64_t>(*forward);
  auto half_rays =
      static_cast<std::int64_t>(std::floor(cone_half_angle_rad / geometry.angle_increment + 1e-9));

  Cone cone{n, f, 0, 0, is_full_circle(geometry)};
  if (cone.wraps) {
    // Never visit a ray twice.
    half_rays = std::min(half_rays, (nn - 1) / 2);
    cone.offset_lo = -half_rays;
    cone.offset_hi = half_rays;
  } else {
    cone.offset_lo = std::max(-half_rays, -f);
    cone.offset_hi = std::min(half_rays, nn - 1 - f);
  }

  const auto bearing_of = [&](std::size_t index) {
    return wrap_angle(laser_bearing_of_index(geometry, index) + laser_yaw_offset_rad);
  };

  GapSelection result;
  result.target_index = *forward;
  result.gap_first = *forward;
  result.gap_last = *forward;
  result.target_vehicle_bearing_rad = bearing_of(*forward);

  const std::size_t m = cone.length();
  const double inc = geometry.angle_increment;
  // Vehicle bearing of cone position j, unwrapped: forward_bearing + offset_of(j) * inc.
  const double forward_bearing = result.target_vehicle_bearing_rad;
  const double p = preference.forward_preference;
  // Score (see GapPreference in the header). With p == 0 this is exactly the angular width,
  // which orders gaps exactly as their ray counts do, so the choice matches the old
  // integer "widest run" comparison bit for bit.
  const auto score_of = [&](std::size_t start, std::size_t end) {
    const double width = static_cast<double>(end - start + 1) * inc;
    if (p == 0.0) {
      return width;
    }
    const double mid_offset =
        0.5 * static_cast<double>(cone.offset_of(start) + cone.offset_of(end));
    const double centre_bearing = forward_bearing + mid_offset * inc;
    return width * std::max(0.0, 1.0 - p * (1.0 - std::cos(centre_bearing)));
  };

  // Cone position of the ray nearest the previous target bearing, if hysteresis is on and
  // that bearing is inside the cone.
  std::optional<std::size_t> previous_j;
  if (preference.switch_margin > 0.0 && preference.previous_target_vehicle_bearing_rad &&
      std::isfinite(*preference.previous_target_vehicle_bearing_rad)) {
    const double rel =
        wrap_angle(*preference.previous_target_vehicle_bearing_rad - forward_bearing);
    const std::int64_t pj = static_cast<std::int64_t>(std::llround(rel / inc)) - cone.offset_lo;
    if (pj >= 0 && pj < static_cast<std::int64_t>(m)) {
      previous_j = static_cast<std::size_t>(pj);
    }
  }

  // Best-scoring run of free rays; ties go to the run whose centre is nearest forward.
  bool found = false;
  std::size_t best_start = 0;
  std::size_t best_end = 0;
  double best_score = 0.0;
  std::int64_t best_centre_dist = 0;
  bool incumbent_found = false;
  std::size_t incumbent_start = 0;
  std::size_t incumbent_end = 0;
  double incumbent_score = 0.0;
  std::size_t j = 0;
  while (j < m) {
    if (!(ranges[cone.index_of(j)] > free_space_threshold_m)) {
      ++j;
      continue;
    }
    const std::size_t start = j;
    while (j < m && ranges[cone.index_of(j)] > free_space_threshold_m) {
      ++j;
    }
    const std::size_t end = j - 1;
    const double score = score_of(start, end);
    const std::size_t centre_j = (start + end) / 2;
    const std::int64_t centre_dist = std::abs(cone.offset_of(centre_j));
    if (!found || score > best_score || (score == best_score && centre_dist < best_centre_dist)) {
      found = true;
      best_start = start;
      best_end = end;
      best_score = score;
      best_centre_dist = centre_dist;
    }
    if (previous_j && *previous_j >= start && *previous_j <= end) {
      incumbent_found = true;
      incumbent_start = start;
      incumbent_end = end;
      incumbent_score = score;
    }
  }
  if (!found) {
    return result;
  }
  // Hysteresis: keep the gap that still contains last cycle's target unless the best gap
  // beats it by more than the margin.
  if (incumbent_found && !(best_score > incumbent_score * (1.0 + preference.switch_margin))) {
    best_start = incumbent_start;
    best_end = incumbent_end;
  }

  std::size_t target_j = (best_start + best_end) / 2;
  if (target == GapTarget::kDeepest) {
    double best_range = -1.0;
    std::int64_t best_dist = 0;
    for (std::size_t k = best_start; k <= best_end; ++k) {
      const double r = ranges[cone.index_of(k)];
      const std::int64_t dist = std::abs(cone.offset_of(k));
      if (r > best_range || (r == best_range && dist < best_dist)) {
        best_range = r;
        best_dist = dist;
        target_j = k;
      }
    }
  }

  result.gap_found = true;
  result.gap_first = cone.index_of(best_start);
  result.gap_last = cone.index_of(best_end);
  result.target_index = cone.index_of(target_j);
  result.target_vehicle_bearing_rad = bearing_of(result.target_index);
  return result;
}

bool corner_blocked(const ScanInput& geometry, const std::vector<double>& ranges,
                    double steering_rad, double sector_inner_rad, double sector_outer_rad,
                    double min_clearance_m, double laser_yaw_offset_rad) {
  if (!std::isfinite(steering_rad) || steering_rad == 0.0 || !is_usable(geometry) ||
      ranges.size() != geometry.ranges.size()) {
    return false;
  }
  const double lo = steering_rad > 0.0 ? sector_inner_rad : -sector_outer_rad;
  const double hi = steering_rad > 0.0 ? sector_outer_rad : -sector_inner_rad;
  std::size_t rays_in_sector = 0;
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double bearing = wrap_angle(laser_bearing_of_index(geometry, i) + laser_yaw_offset_rad);
    if (bearing < lo || bearing > hi) {
      continue;
    }
    ++rays_in_sector;
    if (!(ranges[i] < min_clearance_m)) {
      return false;
    }
  }
  return rays_in_sector > 0;
}

double steering_from_bearing(double bearing_rad, double gain, double max_steering_rad) {
  if (!std::isfinite(bearing_rad) || !std::isfinite(gain)) {
    return 0.0;
  }
  return std::min(std::max(gain * bearing_rad, -max_steering_rad), max_steering_rad);
}

namespace {

bool is_valid_return(float range_m, const ScanInput& scan) {
  const double r = static_cast<double>(range_m);
  return std::isfinite(r) && r > 0.0 && r >= scan.range_min && r <= scan.range_max;
}

}  // namespace

SweptPathClamp clamp_steering_to_swept_path(const ScanInput& scan, double laser_yaw_offset_rad,
                                            double steering_rad, const SweptPathGeometry& g) {
  SweptPathClamp out;
  out.steering_rad = steering_rad;
  const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
  if (!std::isfinite(steering_rad) || steering_rad == 0.0 || std::abs(steering_rad) >= M_PI_2 ||
      !is_usable(scan) || !std::isfinite(laser_yaw_offset_rad) || !positive(g.wheelbase_m) ||
      !positive(g.half_width_m) || !positive(g.lookahead_m) || !std::isfinite(g.margin_m) ||
      g.margin_m < 0.0 || !std::isfinite(g.body_front_x_m) || !std::isfinite(g.lidar_mount_x_m) ||
      !std::isfinite(g.lidar_mount_y_m)) {
    return out;
  }
  // Everything below is worked as a LEFT turn; a right turn mirrors y.
  const double side = steering_rad > 0.0 ? 1.0 : -1.0;
  const double c = g.half_width_m + g.margin_m;
  const double wanted_k = std::tan(std::abs(steering_rad)) / g.wheelbase_m;
  double k = wanted_k;
  // Each pass either finds no violator (done) or strictly lowers k past some return's k_max,
  // after which that return can never violate again: at most one pass per return.
  for (std::size_t pass = 0; pass <= scan.ranges.size(); ++pass) {
    if (k <= 0.0) {
      break;
    }
    const double radius = 1.0 / k;
    const double outer = std::hypot(g.body_front_x_m, radius + c);
    double next_k = k;
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      if (!is_valid_return(scan.ranges[i], scan)) {
        continue;
      }
      const double r = static_cast<double>(scan.ranges[i]);
      const double bearing = laser_bearing_of_index(scan, i) + laser_yaw_offset_rad;
      const double x = r * std::cos(bearing) + g.lidar_mount_x_m;
      const double y = side * (r * std::sin(bearing) + g.lidar_mount_y_m);
      if (!(y > g.half_width_m)) {
        continue;  // other side, or inside the car's width (the TTC gate's job)
      }
      // Ahead along the turn: arc angle in (0, pi/2), i.e. x > 0 and y < R, and within the
      // lookahead of rear-axle arc length.
      if (!(x > 0.0) || !(y < radius)) {
        continue;
      }
      const double phi = std::atan2(x, radius - y);
      if (radius * phi > g.lookahead_m) {
        continue;
      }
      // Clear on the inside of the inner flank circle, or outside the outer corner circle.
      const double k_max = y > c ? 2.0 * (y - c) / (x * x + y * y - c * c) : 0.0;
      if (k <= k_max) {
        continue;
      }
      if (std::hypot(x, radius - y) > outer) {
        continue;
      }
      next_k = std::min(next_k, k_max);
    }
    if (next_k == k) {
      break;
    }
    k = next_k;
  }
  if (k < wanted_k) {
    out.clamped = true;
    out.steering_rad = side * std::atan(k * g.wheelbase_m);
  }
  return out;
}

double FirstOrderLowPass::update(double input, double dt_s) {
  if (!std::isfinite(input) || !std::isfinite(dt_s) || dt_s <= 0.0) {
    return output_;
  }
  if (!(time_constant_s_ > 0.0)) {
    output_ = input;
    return output_;
  }
  output_ += dt_s / (time_constant_s_ + dt_s) * (input - output_);
  return output_;
}

GapFollowResult GapFollower::process(const ScanInput& scan) {
  GapFollowResult result;
  if (!is_usable(scan)) {
    return result;
  }
  if (sanitize_ranges(scan, config_.clip_max_range_m, sanitized_) == 0) {
    return result;
  }
  extend_disparities(sanitized_, config_.disparity_threshold_m,
                     config_.half_width_m + config_.safety_margin_m, scan.angle_increment,
                     extended_);
  GapPreference preference;
  preference.forward_preference = config_.forward_preference;
  preference.switch_margin = config_.gap_switch_margin;
  preference.previous_target_vehicle_bearing_rad = previous_target_vehicle_bearing_rad_;
  const auto selection =
      select_gap(scan, extended_, config_.free_space_threshold_m, config_.cone_half_angle_rad,
                 config_.laser_yaw_offset_rad, config_.target, preference);
  if (!selection) {
    return result;
  }
  if (selection->gap_found) {
    previous_target_vehicle_bearing_rad_ = selection->target_vehicle_bearing_rad;
  } else {
    previous_target_vehicle_bearing_rad_.reset();
  }
  result.valid = true;
  result.gap_found = selection->gap_found;
  result.target_vehicle_bearing_rad = selection->target_vehicle_bearing_rad;
  result.target_range_m = extended_[selection->target_index];
  result.steering_rad = steering_from_bearing(
      selection->target_vehicle_bearing_rad, config_.steering_gain, config_.max_steering_angle_rad);
  result.wanted_steering_rad = result.steering_rad;
  if (config_.swept_path_clamp) {
    SweptPathGeometry geometry = config_.swept_path;
    geometry.half_width_m = config_.half_width_m;
    geometry.margin_m = config_.safety_margin_m;
    const SweptPathClamp clamp = clamp_steering_to_swept_path(scan, config_.laser_yaw_offset_rad,
                                                              result.steering_rad, geometry);
    result.swept_path_clamped = clamp.clamped;
    result.steering_rad = clamp.steering_rad;
  }
  result.corner_blocked =
      corner_blocked(scan, sanitized_, result.steering_rad, config_.corner_sector_inner_rad,
                     config_.corner_sector_outer_rad, config_.corner_min_clearance_m,
                     config_.laser_yaw_offset_rad);
  if (result.corner_blocked) {
    result.steering_rad = 0.0;
  }
  return result;
}

}  // namespace racer_control
