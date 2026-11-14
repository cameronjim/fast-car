#include "racer_planner/gate_predictor.hpp"

#include <algorithm>
#include <cmath>

namespace racer_planner {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = 0.5 * kPi;
// Mirrors racer_safety kStraightSteeringEpsilonRad (forward_sector.hpp): a request under this is
// a straight corridor. test_gate_divergence.cpp fails if the two drift apart.
constexpr double kStraightSteeringEpsilonRad = 1e-3;
// Squared radius pre-filter margin of the arc test: a return is skipped before the square root
// only when it is outside the band by more than this fraction, so the exact test below decides
// every borderline case with the gate's own arithmetic.
constexpr double kBandGuard = 1e-9;

double wrap_angle(double a) {
  double w = std::fmod(a + kPi, 2.0 * kPi);
  if (w < 0.0) w += 2.0 * kPi;
  return w - kPi;
}

bool usable_return(float range_m, const GateScan& s) {
  const double r = static_cast<double>(range_m);
  if (!std::isfinite(r)) return false;
  if (r <= 0.0) return false;
  if (std::isfinite(s.range_min_m) && r < s.range_min_m) return false;
  if (std::isfinite(s.range_max_m) && r > s.range_max_m) return false;
  return true;
}

double clamp_symmetric(double v, double bound) {
  if (v > bound) return bound;
  if (v < -bound) return -bound;
  return v;
}

// forward_sector.cpp leading_edge_arc_angle_rad.
double leading_edge_arc_angle(double rho, double abs_radius, double half_width, double body_x) {
  const double inner = std::fmax(abs_radius - half_width, 0.0);
  const double flank_x = std::sqrt(std::fmax(rho * rho - inner * inner, 0.0));
  const double lead_x = std::fmin(body_x, flank_x);
  return std::atan2(lead_x, std::sqrt(std::fmax(rho * rho - lead_x * lead_x, 0.0)));
}

// One steering angle, prepared once per table rebuild.
struct Prepared {
  bool straight = true;
  double radius = std::numeric_limits<double>::infinity();
  double abs_radius = 0.0;
  double turn_sign = 1.0;
  double lo2 = 0.0;  // squared inner bound, shrunk by the guard
  double hi2 = 0.0;  // squared outer bound, grown by the guard
};

// forward_sector.cpp arc_corridor_distance_m, forward, body reference. Returns +infinity when the
// return is not in the path.
double arc_distance(double xr, double yr, const GateGeometry& g, const Prepared& p) {
  const double inf = std::numeric_limits<double>::infinity();
  const double rho = std::hypot(xr, yr - p.radius);
  if (rho < p.abs_radius - g.corridor_half_width_m) return inf;
  if (rho > std::hypot(g.body_front_x_m, p.abs_radius + g.corridor_half_width_m)) return inf;
  const double phi = std::atan2(xr, p.abs_radius - p.turn_sign * yr);
  if (phi <= 0.0) return inf;
  if (phi >= kHalfPi) return inf;
  const double phi_ref =
      leading_edge_arc_angle(rho, p.abs_radius, g.corridor_half_width_m, g.body_front_x_m);
  const double arc = p.abs_radius * (phi - phi_ref);
  if (arc <= 0.0) return inf;
  if (rho > p.abs_radius + g.corridor_half_width_m) {
    if (arc > g.outer_corner_horizon_m) return inf;
  }
  return arc;
}

bool trustworthy_scan(const GateScan& s) {
  if (!std::isfinite(s.angle_min_rad)) return false;
  if (!std::isfinite(s.angle_increment_rad)) return false;
  if (s.angle_increment_rad <= 0.0) return false;
  return true;
}

}  // namespace

bool gate_geometry_is_usable(const GateGeometry& g) {
  const double v[] = {g.wheelbase_m,           g.steering_min_rad,
                      g.steering_max_rad,      g.max_steering_angle_rad,
                      g.lidar_mount_x_m,       g.lidar_mount_y_m,
                      g.lidar_mount_yaw_rad,   g.body_front_x_m,
                      g.corridor_half_width_m, g.sector_half_angle_rad};
  for (double x : v) {
    if (!std::isfinite(x)) return false;
  }
  if (std::isnan(g.outer_corner_horizon_m) || g.outer_corner_horizon_m <= 0.0) return false;
  return g.wheelbase_m > 0.0 && g.steering_min_rad < g.steering_max_rad &&
         g.max_steering_angle_rad >= 0.0 && g.max_steering_angle_rad < kHalfPi &&
         g.body_front_x_m >= 0.0 && g.corridor_half_width_m > 0.0 && g.sector_half_angle_rad > 0.0;
}

bool gate_limits_are_usable(const GateLimits& l) {
  return std::isfinite(l.ttc_brake_s) && l.ttc_brake_s > 0.0 && std::isfinite(l.ttc_warning_s) &&
         l.ttc_warning_s > 0.0 && std::isfinite(l.min_forward_clearance_m) &&
         l.min_forward_clearance_m > 0.0;
}

void gate_distances(const GateGeometry& g, const GateScan& scan, const double* deltas,
                    std::size_t count, double* out) {
  const double inf = std::numeric_limits<double>::infinity();
  count = std::min(count, kGateMaxSteerings);
  for (std::size_t k = 0; k < count; ++k) out[k] = inf;
  if (scan.ranges == nullptr) return;

  // The gate's fail closed reading of a scan it cannot place: the slant range, whole scan.
  if (!trustworthy_scan(scan)) {
    double m = inf;
    for (std::size_t i = 0; i < scan.count; ++i) {
      if (!usable_return(scan.ranges[i], scan)) continue;
      m = std::min(m, static_cast<double>(scan.ranges[i]));
    }
    for (std::size_t k = 0; k < count; ++k) out[k] = m;
    return;
  }

  Prepared prep[kGateMaxSteerings];
  const double hw = g.corridor_half_width_m;
  for (std::size_t k = 0; k < count; ++k) {
    Prepared& p = prep[k];
    const double d = clamp_symmetric(deltas[k], g.max_steering_angle_rad);
    if (!std::isfinite(deltas[k]) || std::abs(d) < kStraightSteeringEpsilonRad) {
      // A non finite request is garbage to the gate (command sanity); the straight arc is taken,
      // the table never asks for one.
      p.straight = true;
      continue;
    }
    p.straight = false;
    p.radius = g.wheelbase_m / std::tan(d);
    p.abs_radius = std::abs(p.radius);
    p.turn_sign = p.radius < 0.0 ? -1.0 : 1.0;
    const double lo = std::fmax(p.abs_radius - hw, 0.0) * (1.0 - kBandGuard);
    const double hi = std::hypot(g.body_front_x_m, p.abs_radius + hw) * (1.0 + kBandGuard);
    p.lo2 = lo * lo;
    p.hi2 = hi * hi;
  }

  for (std::size_t i = 0; i < scan.count; ++i) {
    if (!usable_return(scan.ranges[i], scan)) continue;
    const double range = static_cast<double>(scan.ranges[i]);
    const double laser_bearing =
        scan.angle_min_rad + static_cast<double>(i) * scan.angle_increment_rad;
    const double vehicle_bearing = wrap_angle(laser_bearing + g.lidar_mount_yaw_rad);
    if (std::abs(vehicle_bearing) > g.sector_half_angle_rad) continue;
    const double x = range * std::cos(vehicle_bearing);
    const double y = range * std::sin(vehicle_bearing);
    const double xr = x + g.lidar_mount_x_m;
    const double yr = y + g.lidar_mount_y_m;
    const double ahead = x + g.lidar_mount_x_m - g.body_front_x_m;
    const bool in_straight = ahead > 0.0 && std::abs(y + g.lidar_mount_y_m) <= hw;
    for (std::size_t k = 0; k < count; ++k) {
      const Prepared& p = prep[k];
      double d;
      if (p.straight) {
        if (!in_straight) continue;
        d = ahead;
      } else {
        const double dy = yr - p.radius;
        const double r2 = xr * xr + dy * dy;
        if (r2 < p.lo2 || r2 > p.hi2) continue;
        d = arc_distance(xr, yr, g, p);
      }
      if (d < out[k]) out[k] = d;
    }
  }
}

double gate_distance(const GateGeometry& g, const GateScan& scan, double delta_rad) {
  double out = std::numeric_limits<double>::infinity();
  gate_distances(g, scan, &delta_rad, 1, &out);
  return out;
}

GateTable::GateTable(const GateGeometry& geometry, const GateLimits& limits, std::size_t max_beams)
    : geometry_(geometry), limits_(limits), scan_(max_beams, 0.0f) {
  d_.fill(std::numeric_limits<double>::infinity());
  for (int j = 0; j < kGateSteeringValues; ++j) {
    const double f = static_cast<double>(j) / static_cast<double>(kGateSteeringValues - 1);
    deltas_[static_cast<std::size_t>(j)] =
        geometry_.steering_min_rad + (geometry_.steering_max_rad - geometry_.steering_min_rad) * f;
  }
}

double GateTable::steering(int j) const { return deltas_[static_cast<std::size_t>(j)]; }

void GateTable::build(const GateScan& scan) {
  gate_distances(geometry_, scan, deltas_.data(), kGateSteeringValues, d_.data());
  valid_ = true;
  exact_ok_ = scan.ranges != nullptr && scan.count <= scan_.size();
  if (exact_ok_) {
    std::copy(scan.ranges, scan.ranges + scan.count, scan_.begin());
    scan_view_ = scan;
    scan_view_.ranges = scan_.data();
  }
}

double GateTable::exact_distance(double delta_rad) const {
  if (!valid_) return std::numeric_limits<double>::infinity();
  if (!exact_ok_ || std::isnan(delta_rad)) return distance(delta_rad);
  return gate_distance(geometry_, scan_view_, delta_rad);
}

double GateTable::distance(double delta_rad) const {
  if (!valid_) return std::numeric_limits<double>::infinity();
  if (std::isnan(delta_rad)) return *std::min_element(d_.begin(), d_.end());
  const double lo = geometry_.steering_min_rad;
  const double hi = geometry_.steering_max_rad;
  const double c = std::min(std::max(delta_rad, lo), hi);
  const double u = (c - lo) / (hi - lo) * static_cast<double>(kGateSteeringValues - 1);
  int i = static_cast<int>(std::floor(u));
  i = std::min(std::max(i, 0), kGateSteeringValues - 2);
  return std::min(d_[static_cast<std::size_t>(i)], d_[static_cast<std::size_t>(i) + 1]);
}

bool gate_would_trip_at(double d_m, double v_cmd_mps, double v_wheel_mps,
                        const GateLimits& limits) {
  if (!(v_cmd_mps > 0.0)) return false;  // zero, away or NaN: never judged
  if (!(d_m > 0.0) || !std::isfinite(d_m)) return false;
  const double v_wheel = std::isfinite(v_wheel_mps) && v_wheel_mps > 0.0 ? v_wheel_mps : 0.0;
  const double v_judged = std::max(v_cmd_mps, v_wheel);
  if (d_m / v_judged <= limits.ttc_brake_s) return true;
  return d_m < limits.min_forward_clearance_m;
}

bool would_trip(const GateTable& table, double delta_rad, double v_cmd_mps, double v_wheel_mps,
                const GateLimits& limits) {
  return gate_would_trip_at(table.distance(delta_rad), v_cmd_mps, v_wheel_mps, limits);
}

bool would_trip_exact(const GateTable& table, double delta_rad, double v_cmd_mps,
                      double v_wheel_mps, const GateLimits& limits) {
  return gate_would_trip_at(table.exact_distance(delta_rad), v_cmd_mps, v_wheel_mps, limits);
}

}  // namespace racer_planner
