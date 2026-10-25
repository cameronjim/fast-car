#include "racer_control/gap_follow.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>

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
  const double c = g.half_width_m + g.margin_m;  // for the outer circle
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
      // Clear on the inside of the inner flank circle, or outside the outer corner circle. A
      // return beside the car up to the front axle (x <= wheelbase) is approached only by the
      // flank's off-tracking, so the turn may take at most half the clearance the car already
      // has to it, and never needs more than the margin (gap_follow.hpp, BESIDE THE CAR).
      const double clearance =
          x <= g.wheelbase_m ? std::min(g.margin_m, 0.5 * (y - g.half_width_m)) : g.margin_m;
      const double cr = g.half_width_m + clearance;
      const double k_max = y > cr ? 2.0 * (y - cr) / (x * x + y * y - cr * cr) : 0.0;
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

namespace {

// Median of values[0, count), reordering them. count must be > 0. Even count: the mean of
// the two middle values (same convention as RollingMedian).
double median_in_place(std::vector<double>& values, std::size_t count) {
  const auto first = values.begin();
  const auto mid = first + static_cast<std::ptrdiff_t>(count / 2);
  const auto last = first + static_cast<std::ptrdiff_t>(count);
  std::nth_element(first, mid, last);
  const double upper = *mid;
  if (count % 2 == 1) {
    return upper;
  }
  const double lower = *std::max_element(first, mid);
  return 0.5 * (lower + upper);
}

}  // namespace

LaneWalls measure_lane_walls(const ScanInput& scan, double laser_yaw_offset_rad,
                             double sector_half_angle_rad, double max_range_m,
                             double lidar_mount_y_m, std::vector<double>& left_scratch,
                             std::vector<double>& right_scratch) {
  LaneWalls walls;
  const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
  if (!is_usable(scan) || !std::isfinite(laser_yaw_offset_rad) ||
      !positive(sector_half_angle_rad) || !positive(max_range_m) ||
      !std::isfinite(lidar_mount_y_m)) {
    return walls;
  }
  const std::size_t n = scan.ranges.size();
  if (left_scratch.size() < n) {
    left_scratch.resize(n);
  }
  if (right_scratch.size() < n) {
    right_scratch.resize(n);
  }
  std::size_t n_left = 0;
  std::size_t n_right = 0;
  for (std::size_t i = 0; i < n; ++i) {
    if (!is_valid_return(scan.ranges[i], scan)) {
      continue;
    }
    const double r = static_cast<double>(scan.ranges[i]);
    if (r > max_range_m) {
      continue;
    }
    const double bearing = wrap_angle(laser_bearing_of_index(scan, i) + laser_yaw_offset_rad);
    const double lateral = r * std::sin(bearing) + lidar_mount_y_m;  // left positive
    if (bearing > 0.0 && bearing <= sector_half_angle_rad) {
      if (lateral > 0.0) {
        left_scratch[n_left++] = lateral;
      }
    } else if (bearing < 0.0 && bearing >= -sector_half_angle_rad) {
      if (lateral < 0.0) {
        right_scratch[n_right++] = -lateral;
      }
    }
  }
  if (n_left > 0) {
    walls.left_m = median_in_place(left_scratch, n_left);
  }
  if (n_right > 0) {
    walls.right_m = median_in_place(right_scratch, n_right);
  }
  return walls;
}

double centering_steering(const LaneWalls& walls, double centering_gain) {
  const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
  if (!std::isfinite(centering_gain) || centering_gain == 0.0 || !walls.left_m || !walls.right_m ||
      !positive(*walls.left_m) || !positive(*walls.right_m)) {
    return 0.0;
  }
  const double left = *walls.left_m;
  const double right = *walls.right_m;
  return centering_gain * (left - right) / (left + right);
}

std::optional<TargetMode> target_mode_from_string(std::string_view name) {
  if (name == "gap_centre") {
    return TargetMode::kGapCentre;
  }
  if (name == "gap_deepest") {
    return TargetMode::kGapDeepest;
  }
  if (name == "lane_centre") {
    return TargetMode::kLaneCentre;
  }
  return std::nullopt;
}

const char* target_mode_name(TargetMode mode) {
  switch (mode) {
    case TargetMode::kGapCentre:
      return "gap_centre";
    case TargetMode::kGapDeepest:
      return "gap_deepest";
    case TargetMode::kLaneCentre:
      return "lane_centre";
  }
  return "gap_centre";
}

std::optional<TargetModeResolution> resolve_target_mode(std::string_view target_mode,
                                                        bool target_deepest_ray) {
  const auto mode = target_mode_from_string(target_mode);
  if (!mode) {
    return std::nullopt;
  }
  if (!target_deepest_ray) {
    return TargetModeResolution{*mode, false};
  }
  if (*mode == TargetMode::kLaneCentre) {
    return std::nullopt;
  }
  return TargetModeResolution{TargetMode::kGapDeepest, true};
}

std::size_t load_lane_points(const ScanInput& scan, double laser_yaw_offset_rad,
                             double clip_max_range_m, const LaneGeometry& g, LanePoints& out) {
  out.count = 0;
  out.wraps = false;
  const auto finite = [](double v) { return std::isfinite(v); };
  if (!is_usable(scan) || !finite(laser_yaw_offset_rad) || !finite(clip_max_range_m) ||
      !(clip_max_range_m > 0.0) || !finite(g.half_width_m) || !finite(g.body_front_x_m) ||
      !finite(g.lidar_mount_x_m) || !finite(g.lidar_mount_y_m)) {
    return 0;
  }
  const std::size_t n = scan.ranges.size();
  if (out.x.size() < n) {
    out.x.resize(n);
    out.y.resize(n);
    out.chain.resize(n);
    out.chain_size.resize(n);
  }
  std::size_t count = 0;
  std::size_t chain = 0;
  for (std::size_t i = 0; i < n; ++i) {
    if (!is_valid_return(scan.ranges[i], scan)) {
      continue;
    }
    const double r = static_cast<double>(scan.ranges[i]);
    if (!(r < clip_max_range_m)) {
      continue;  // free space
    }
    const double bearing = laser_bearing_of_index(scan, i) + laser_yaw_offset_rad;
    const double x = r * std::cos(bearing) + g.lidar_mount_x_m;
    const double y = r * std::sin(bearing) + g.lidar_mount_y_m;
    if (x >= 0.0 && x <= g.body_front_x_m && std::abs(y) <= g.half_width_m) {
      continue;  // the car itself
    }
    if (count > 0 && std::hypot(x - out.x[count - 1], y - out.y[count - 1]) > kLaneChainGapM) {
      ++chain;
    }
    out.x[count] = x;
    out.y[count] = y;
    out.chain[count] = chain;
    ++count;
  }
  // A full-circle scan whose last and first returns are close is continuous across the seam:
  // the last chain is the first one continued.
  const bool wrap =
      count > 1 && is_full_circle(scan) &&
      std::hypot(out.x[0] - out.x[count - 1], out.y[0] - out.y[count - 1]) <= kLaneChainGapM;
  if (wrap) {
    const std::size_t last = out.chain[count - 1];
    for (std::size_t k = count; k-- > 0 && out.chain[k] == last;) {
      out.chain[k] = 0;
    }
  }
  for (std::size_t c = 0; c <= chain && c < n; ++c) {
    out.chain_size[c] = 0;
  }
  for (std::size_t k = 0; k < count; ++k) {
    ++out.chain_size[out.chain[k]];
  }
  out.wraps = wrap;
  out.count = count;
  return count;
}

namespace {

// A return counts as boundary when its chain has at least kLaneMinChainPoints returns (a chair
// leg, a stray return is not a wall).
bool counts_as_boundary(const LanePoints& points, std::size_t k) {
  return points.chain_size[points.chain[k]] >= kLaneMinChainPoints;
}

// The nearest counted return on each side of the line through q along the unit heading (hx, hy),
// among those within window_m of q: left is (p - q) x h > 0 (left of the heading), right < 0. A
// return on the line itself is on neither side. Returns of the chain `not_left` are never on the
// left and those of `not_right` never on the right (the trace's side memory, see CENTRELINE
// TRACE). If both sides' nearest returns are then on one chain (a wall across the heading), the
// nearer side keeps it and the other side takes its nearest return off that chain.
struct SidedNearest {
  std::optional<std::size_t> left;
  std::optional<std::size_t> right;
  double left_m = 0.0;
  double right_m = 0.0;
};

SidedNearest nearest_each_side(const LanePoints& points, double qx, double qy, double hx, double hy,
                               double window_m, std::optional<std::size_t> not_left,
                               std::optional<std::size_t> not_right) {
  const auto search = [&](std::optional<std::size_t> skip_left,
                          std::optional<std::size_t> skip_right, SidedNearest& out) {
    double best_left2 = window_m * window_m;
    double best_right2 = best_left2;
    out.left.reset();
    out.right.reset();
    for (std::size_t k = 0; k < points.count; ++k) {
      if (!counts_as_boundary(points, k)) {
        continue;
      }
      const double dx = points.x[k] - qx;
      const double dy = points.y[k] - qy;
      const double d2 = dx * dx + dy * dy;
      const double cross = hx * dy - hy * dx;
      const std::size_t chain = points.chain[k];
      if (cross > 0.0 && d2 <= best_left2 && skip_left != chain) {
        best_left2 = d2;
        out.left = k;
      } else if (cross < 0.0 && d2 <= best_right2 && skip_right != chain) {
        best_right2 = d2;
        out.right = k;
      }
    }
    out.left_m = std::sqrt(best_left2);
    out.right_m = std::sqrt(best_right2);
  };
  SidedNearest out;
  search(not_left, not_right, out);
  if (out.left && out.right && points.chain[*out.left] == points.chain[*out.right]) {
    const std::size_t chain = points.chain[*out.left];
    if (out.left_m <= out.right_m) {
      search(not_left, chain, out);
    } else {
      search(chain, not_right, out);
    }
  }
  return out;
}

}  // namespace

LaneBoundaries find_lane_boundaries(const LanePoints& points, const LaneGeometry& g,
                                    double max_width_m) {
  LaneBoundaries b;
  if (points.count == 0 || !std::isfinite(max_width_m) || !(max_width_m > 0.0) ||
      !std::isfinite(g.body_front_x_m)) {
    return b;
  }
  const double cx = 0.5 * g.body_front_x_m;
  double best_left = std::numeric_limits<double>::infinity();
  double best_right = std::numeric_limits<double>::infinity();
  std::size_t left_chain = 0;
  std::size_t right_chain = 0;
  for (std::size_t k = 0; k < points.count; ++k) {
    if (!counts_as_boundary(points, k)) {
      continue;
    }
    const double dx = points.x[k] - cx;
    const double y = points.y[k];
    if (y > 0.0 && std::abs(dx) <= y) {
      const double d = std::hypot(dx, y);
      if (d < best_left) {
        best_left = d;
        left_chain = points.chain[k];
      }
    } else if (y < 0.0 && std::abs(dx) <= -y) {
      const double d = std::hypot(dx, y);
      if (d < best_right) {
        best_right = d;
        right_chain = points.chain[k];
      }
    }
  }
  if (std::isfinite(best_left)) {
    b.left_m = best_left;
  }
  if (std::isfinite(best_right)) {
    b.right_m = best_right;
  }
  // One wall on both sides (a ring, a dead end) or a lane too wide to be one: no width.
  if (b.left_m && b.right_m && left_chain != right_chain && best_left + best_right <= max_width_m) {
    b.width_m = best_left + best_right;
  }
  return b;
}

LanePoint trace_lane_centre(const LanePoints& points, double lookahead_m, double half_width_m,
                            double window_m, std::vector<LanePoint>& trace) {
  trace.clear();
  LanePoint at;
  if (!std::isfinite(lookahead_m) || !(lookahead_m > 0.0) || !std::isfinite(half_width_m) ||
      !(half_width_m > 0.0) || !std::isfinite(window_m) || !(window_m > 0.0)) {
    return at;
  }
  const double h = half_width_m;
  const auto steps = static_cast<std::size_t>(std::ceil(lookahead_m / kLaneTraceStepM - 1e-9));
  if (trace.capacity() < steps + 1) {
    trace.reserve(steps + 1);
  }
  double hx = 1.0;  // the trace's local heading, a unit vector; starts along the car
  double hy = 0.0;
  std::optional<std::size_t> left_chain;  // the chains of the last step's nearest returns
  std::optional<std::size_t> right_chain;
  // One corrector move: from `at` with heading (hx, hy), onto the centreline (the medial line of
  // the two nearest returns, or h from the only one), with the lane heading there. Step 0 is the
  // start (the car's place on the centreline: the rear axle moved across the lane onto it).
  const auto correct = [&]() {
    const SidedNearest n =
        nearest_each_side(points, at.x, at.y, hx, hy, window_m, right_chain, left_chain);
    left_chain.reset();
    right_chain.reset();
    double tx = hx;
    double ty = hy;
    if (n.left && n.right) {
      left_chain = points.chain[*n.left];
      right_chain = points.chain[*n.right];
      const double lx = points.x[*n.left];
      const double ly = points.y[*n.left];
      const double rx = points.x[*n.right];
      const double ry = points.y[*n.right];
      const double nx = lx - rx;
      const double ny = ly - ry;
      const double length = std::hypot(nx, ny);
      if (length > 1e-9) {
        tx = ny / length;
        ty = -nx / length;
        if (tx * hx + ty * hy < 0.0) {
          tx = -tx;
          ty = -ty;
        }
        const double mx = 0.5 * (lx + rx);
        const double my = 0.5 * (ly + ry);
        const double along = (at.x - mx) * tx + (at.y - my) * ty;
        at.x = mx + along * tx;
        at.y = my + along * ty;
      }
    } else if (n.left || n.right) {
      const std::size_t s = n.left ? *n.left : *n.right;
      (n.left ? left_chain : right_chain) = points.chain[s];
      const double d = n.left ? n.left_m : n.right_m;
      if (d > 1e-9) {
        const double ux = (at.x - points.x[s]) / d;  // away from the wall
        const double uy = (at.y - points.y[s]) / d;
        tx = uy;
        ty = -ux;
        if (tx * hx + ty * hy < 0.0) {
          tx = -tx;
          ty = -ty;
        }
        at.x = points.x[s] + h * ux;
        at.y = points.y[s] + h * uy;
      }
    }
    // Never back toward the car: the heading stays within a right angle of the car's.
    if (tx < 0.0) {
      tx = 0.0;
      ty = ty < 0.0 ? -1.0 : 1.0;
    }
    hx = tx;
    hy = ty;
  };
  correct();
  trace.push_back(at);
  double walked = 0.0;
  for (std::size_t k = 0; k < steps; ++k) {
    const double ds = std::min(kLaneTraceStepM, lookahead_m - walked);
    at.x += ds * hx;
    at.y += ds * hy;
    walked += ds;
    correct();
    trace.push_back(at);
  }
  return at;
}

double pure_pursuit_steering(const LanePoint& aim, double wheelbase_m, double max_steering_rad) {
  if (!std::isfinite(aim.x) || !std::isfinite(aim.y) || !std::isfinite(wheelbase_m) ||
      !(wheelbase_m > 0.0) || !std::isfinite(max_steering_rad)) {
    return 0.0;
  }
  const double chord = std::hypot(aim.x, aim.y);
  if (chord < 1e-9) {
    return 0.0;
  }
  const double sin_alpha = aim.y / chord;
  const double steering = std::atan(2.0 * wheelbase_m * sin_alpha / chord);
  return std::min(std::max(steering, -max_steering_rad), max_steering_rad);
}

namespace {

bool probe_geometry_usable(const ScanInput& scan, double laser_yaw_offset_rad, double travel_m,
                           const ArcProbeGeometry& g) {
  const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
  const auto non_negative = [](double v) { return std::isfinite(v) && v >= 0.0; };
  return is_usable(scan) && std::isfinite(laser_yaw_offset_rad) && positive(travel_m) &&
         positive(g.wheelbase_m) && positive(g.half_width_m) && non_negative(g.margin_m) &&
         non_negative(g.front_x_m) && non_negative(g.rear_x_m) &&
         std::isfinite(g.lidar_mount_x_m) && std::isfinite(g.lidar_mount_y_m);
}

// The inflated body rectangle about the rear axle (gap_follow.hpp ArcProbeGeometry).
struct ProbeBody {
  double x_lo, x_hi, half_y;
  bool contains(double x, double y) const {
    return x >= x_lo && x <= x_hi && std::abs(y) <= half_y;
  }
};

ProbeBody probe_body(const ArcProbeGeometry& g) {
  return ProbeBody{-g.rear_x_m - g.margin_m, g.front_x_m + g.margin_m, g.half_width_m + g.margin_m};
}

// Loads the scan's valid returns into `points` as interleaved rear-axle (x, y), skipping any
// already inside the body at the start pose. Returns the number of points.
std::size_t load_probe_points(const ScanInput& scan, double laser_yaw_offset_rad,
                              const ArcProbeGeometry& g, std::vector<double>& points) {
  const ProbeBody body = probe_body(g);
  if (points.size() < 2 * scan.ranges.size()) {
    points.resize(2 * scan.ranges.size());
  }
  std::size_t count = 0;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    if (!is_valid_return(scan.ranges[i], scan)) {
      continue;
    }
    const double r = static_cast<double>(scan.ranges[i]);
    const double bearing = laser_bearing_of_index(scan, i) + laser_yaw_offset_rad;
    const double x = r * std::cos(bearing) + g.lidar_mount_x_m;
    const double y = r * std::sin(bearing) + g.lidar_mount_y_m;
    if (body.contains(x, y)) {
      continue;
    }
    points[2 * count] = x;
    points[2 * count + 1] = y;
    ++count;
  }
  return count;
}

bool arc_clear_for_points(const std::vector<double>& points, std::size_t count, double steering_rad,
                          double travel_m, const ArcProbeGeometry& g) {
  const ProbeBody body = probe_body(g);
  const double k = std::tan(steering_rad) / g.wheelbase_m;
  const auto steps = static_cast<std::size_t>(std::ceil(travel_m / kArcProbeStepM));
  for (std::size_t step = 1; step <= steps; ++step) {
    const double s = std::min(static_cast<double>(step) * kArcProbeStepM, travel_m);
    // Rear-axle pose after s of travel: heading theta = k s, position on the circle.
    const double theta = k * s;
    const double ct = std::cos(theta);
    const double st = std::sin(theta);
    double tx = s;
    double ty = 0.0;
    if (std::abs(k) > 1e-12) {
      tx = st / k;
      ty = (1.0 - ct) / k;
    }
    for (std::size_t p = 0; p < count; ++p) {
      const double dx = points[2 * p] - tx;
      const double dy = points[2 * p + 1] - ty;
      // Into the body frame at that pose (rotate by -theta).
      if (body.contains(ct * dx + st * dy, -st * dx + ct * dy)) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

bool forward_arc_clear(const ScanInput& scan, double laser_yaw_offset_rad, double steering_rad,
                       double travel_m, const ArcProbeGeometry& geometry,
                       std::vector<double>& points) {
  if (!probe_geometry_usable(scan, laser_yaw_offset_rad, travel_m, geometry) ||
      !std::isfinite(steering_rad) || std::abs(steering_rad) >= M_PI_2) {
    return true;
  }
  const std::size_t count = load_probe_points(scan, laser_yaw_offset_rad, geometry, points);
  return arc_clear_for_points(points, count, steering_rad, travel_m, geometry);
}

bool any_forward_arc_clear(const ScanInput& scan, double laser_yaw_offset_rad,
                           double max_steering_rad, double travel_m,
                           const ArcProbeGeometry& geometry, std::vector<double>& points) {
  if (!probe_geometry_usable(scan, laser_yaw_offset_rad, travel_m, geometry) ||
      !std::isfinite(max_steering_rad) || max_steering_rad < 0.0 || max_steering_rad >= M_PI_2) {
    return true;
  }
  const std::size_t count = load_probe_points(scan, laser_yaw_offset_rad, geometry, points);
  for (std::size_t c = 0; c < kArcProbeCandidates; ++c) {
    const double fraction =
        2.0 * static_cast<double>(c) / static_cast<double>(kArcProbeCandidates - 1) - 1.0;
    if (arc_clear_for_points(points, count, fraction * max_steering_rad, travel_m, geometry)) {
      return true;
    }
  }
  return false;
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
  const auto selection = select_gap(
      scan, extended_, config_.free_space_threshold_m, config_.cone_half_angle_rad,
      config_.laser_yaw_offset_rad,
      config_.target_mode == TargetMode::kGapDeepest ? GapTarget::kDeepest : GapTarget::kCentre,
      preference);
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
  if (config_.target_mode == TargetMode::kLaneCentre) {
    // Step 4' (gap_follow.hpp trace_lane_centre): pure pursuit to the lane centreline.
    LaneGeometry geometry;
    geometry.half_width_m = config_.half_width_m;
    geometry.body_front_x_m = config_.swept_path.body_front_x_m;
    geometry.lidar_mount_x_m = config_.swept_path.lidar_mount_x_m;
    geometry.lidar_mount_y_m = config_.swept_path.lidar_mount_y_m;
    load_lane_points(scan, config_.laser_yaw_offset_rad, config_.clip_max_range_m, geometry,
                     lane_points_);
    const LaneBoundaries boundaries = find_lane_boundaries(
        lane_points_, geometry, kLaneMaxWidthFactor * config_.lane_default_half_width_m);
    result.lane_measured_width_m = boundaries.width_m;
    if (boundaries.width_m) {
      lane_half_width_m_ = 0.5 * *boundaries.width_m;
    }
    result.lane_half_width_m = lane_half_width_m_.value_or(config_.lane_default_half_width_m);
    result.lane_aim =
        trace_lane_centre(lane_points_, config_.lane_lookahead_m, result.lane_half_width_m,
                          kLaneWindowFactor * config_.lane_default_half_width_m, lane_trace_);
    result.lane_valid = true;
    result.steering_rad = pure_pursuit_steering(result.lane_aim, config_.swept_path.wheelbase_m,
                                                config_.max_steering_angle_rad);
    // The speed law reads the free range toward the aim point, seen from the LiDAR head.
    const double bearing = std::atan2(result.lane_aim.y - config_.swept_path.lidar_mount_y_m,
                                      result.lane_aim.x - config_.swept_path.lidar_mount_x_m);
    result.target_vehicle_bearing_rad = wrap_angle(bearing);
    if (const auto index = index_for_vehicle_bearing(scan, bearing, config_.laser_yaw_offset_rad)) {
      result.target_range_m = extended_[*index];
    }
  } else if (config_.centering_gain == 0.0) {
    // Centring off: exactly the pipeline without it, bit for bit.
    result.steering_rad =
        steering_from_bearing(selection->target_vehicle_bearing_rad, config_.steering_gain,
                              config_.max_steering_angle_rad);
  } else {
    result.lane_walls = measure_lane_walls(
        scan, config_.laser_yaw_offset_rad, config_.centering_sector_half_angle_rad,
        config_.centering_max_range_m, config_.swept_path.lidar_mount_y_m, lane_left_, lane_right_);
    result.centering_steering_rad = centering_steering(result.lane_walls, config_.centering_gain);
    // The push is added to gain * bearing BEFORE the clamp, so the sum saturates at the
    // steering limit (a hard gap turn is not undone by the push).
    // steering_from_bearing with an infinite limit is the unclamped gain * bearing (0 for a
    // non-finite input), and the centring push is always finite.
    const double gap_steering =
        steering_from_bearing(selection->target_vehicle_bearing_rad, config_.steering_gain,
                              std::numeric_limits<double>::infinity());
    const double max_rad = config_.max_steering_angle_rad;
    result.steering_rad =
        std::min(std::max(gap_steering + result.centering_steering_rad, -max_rad), max_rad);
  }
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
