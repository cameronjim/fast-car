// Shared arc primitives for park_node (see include/racer_control/park_geometry.hpp).
#include "racer_control/park_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace racer_control {

namespace {

constexpr double kStraightCurvature = 1e-9;
// racer_safety's straight-path guard (forward_sector.hpp kStraightSteeringEpsilonRad), restated
// for the prediction.
constexpr double kGateStraightSteeringRad = 1e-3;

double cross(const Point2& o, const Point2& a, const Point2& b) {
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

bool on_segment(const Point2& p, const Segment2& s) {
  return std::min(s.a.x, s.b.x) - 1e-12 <= p.x && p.x <= std::max(s.a.x, s.b.x) + 1e-12 &&
         std::min(s.a.y, s.b.y) - 1e-12 <= p.y && p.y <= std::max(s.a.y, s.b.y) + 1e-12;
}

int orientation(const Point2& o, const Point2& a, const Point2& b) {
  const double c = cross(o, a, b);
  if (c > 1e-15) {
    return 1;
  }
  if (c < -1e-15) {
    return -1;
  }
  return 0;
}

bool segments_intersect(const Segment2& s, const Segment2& t) {
  const int o1 = orientation(s.a, s.b, t.a);
  const int o2 = orientation(s.a, s.b, t.b);
  const int o3 = orientation(t.a, t.b, s.a);
  const int o4 = orientation(t.a, t.b, s.b);
  if (o1 != o2 && o3 != o4) {
    return true;
  }
  if (o1 == 0 && on_segment(t.a, s)) {
    return true;
  }
  if (o2 == 0 && on_segment(t.b, s)) {
    return true;
  }
  if (o3 == 0 && on_segment(s.a, t)) {
    return true;
  }
  if (o4 == 0 && on_segment(s.b, t)) {
    return true;
  }
  return false;
}

bool inside_body_local(const Point2& local, const ParkBody& body) {
  return local.x >= -body.rear_x_m && local.x <= body.front_x_m &&
         std::abs(local.y) <= body.half_width_m;
}

// racer_safety's arc corridor distance (forward_sector.cpp arc_corridor_distance_m), restated.
std::optional<double> arc_distance(double xr, double yr, double half_width_m, double radius_m,
                                   double body_x_m, double horizon_m, double ref_x, double ref_y) {
  const double abs_radius = std::abs(radius_m);
  const double turn_sign = radius_m < 0.0 ? -1.0 : 1.0;
  const double rho = std::hypot(xr, yr - radius_m);
  if (rho < abs_radius - half_width_m) {
    return std::nullopt;
  }
  if (rho > std::hypot(body_x_m, abs_radius + half_width_m)) {
    return std::nullopt;
  }
  const double phi = std::atan2(xr, abs_radius - turn_sign * yr);
  if (phi <= 0.0 || phi >= M_PI_2) {
    return std::nullopt;
  }
  const double phi_ref = std::atan2(ref_x, abs_radius - turn_sign * ref_y);
  const double arc = abs_radius * (phi - phi_ref);
  if (arc <= 0.0) {
    return std::nullopt;
  }
  if (rho > abs_radius + half_width_m && arc > horizon_m) {
    return std::nullopt;
  }
  return arc;
}

}  // namespace

bool is_usable_body(const ParkBody& body) {
  const double values[] = {body.wheelbase_m, body.half_width_m, body.front_x_m,
                           body.rear_x_m,    body.lidar_x_m,    body.lidar_y_m};
  for (double v : values) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return body.wheelbase_m > 0.0 && body.half_width_m > 0.0 && body.front_x_m > 0.0 &&
         body.rear_x_m >= 0.0;
}

double wrap_pi(double angle_rad) {
  double wrapped = std::fmod(angle_rad + M_PI, 2.0 * M_PI);
  if (wrapped < 0.0) {
    wrapped += 2.0 * M_PI;
  }
  return wrapped - M_PI;
}

double curvature_for_steering(double steering_rad, double wheelbase_m) {
  if (!std::isfinite(steering_rad) || !std::isfinite(wheelbase_m) || wheelbase_m <= 0.0) {
    return 0.0;
  }
  return std::tan(steering_rad) / wheelbase_m;
}

Pose2 advance_pose(const Pose2& pose, double curvature, double distance_m) {
  Pose2 out = pose;
  if (std::abs(curvature) < kStraightCurvature) {
    out.x += distance_m * std::cos(pose.yaw);
    out.y += distance_m * std::sin(pose.yaw);
    return out;
  }
  const double yaw1 = pose.yaw + curvature * distance_m;
  out.x += (std::sin(yaw1) - std::sin(pose.yaw)) / curvature;
  out.y -= (std::cos(yaw1) - std::cos(pose.yaw)) / curvature;
  out.yaw = yaw1;
  return out;
}

Point2 to_world(const Pose2& pose, const Point2& local) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {pose.x + c * local.x - s * local.y, pose.y + s * local.x + c * local.y};
}

Point2 to_local(const Pose2& pose, const Point2& world) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  const double dx = world.x - pose.x;
  const double dy = world.y - pose.y;
  return {c * dx + s * dy, -s * dx + c * dy};
}

std::array<Point2, 4> body_corners(const Pose2& pose, const ParkBody& body) {
  return {to_world(pose, {body.front_x_m, body.half_width_m}),
          to_world(pose, {body.front_x_m, -body.half_width_m}),
          to_world(pose, {-body.rear_x_m, -body.half_width_m}),
          to_world(pose, {-body.rear_x_m, body.half_width_m})};
}

double point_segment_distance(const Point2& p, const Segment2& s) {
  const double dx = s.b.x - s.a.x;
  const double dy = s.b.y - s.a.y;
  const double len2 = dx * dx + dy * dy;
  double t = 0.0;
  if (len2 > 0.0) {
    t = std::clamp(((p.x - s.a.x) * dx + (p.y - s.a.y) * dy) / len2, 0.0, 1.0);
  }
  return std::hypot(p.x - (s.a.x + t * dx), p.y - (s.a.y + t * dy));
}

double segment_segment_distance(const Segment2& s, const Segment2& t) {
  if (segments_intersect(s, t)) {
    return 0.0;
  }
  return std::min({point_segment_distance(s.a, t), point_segment_distance(s.b, t),
                   point_segment_distance(t.a, s), point_segment_distance(t.b, s)});
}

double body_segment_distance(const Pose2& pose, const ParkBody& body, const Segment2& segment) {
  if (inside_body_local(to_local(pose, segment.a), body) ||
      inside_body_local(to_local(pose, segment.b), body)) {
    return 0.0;
  }
  const std::array<Point2, 4> c = body_corners(pose, body);
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < c.size(); ++i) {
    const Segment2 edge{c[i], c[(i + 1) % c.size()]};
    best = std::min(best, segment_segment_distance(edge, segment));
    if (best <= 0.0) {
      return 0.0;
    }
  }
  return best;
}

Clearance body_clearance(const Pose2& pose, const ParkBody& body,
                         const std::vector<Segment2>& obstacles) {
  Clearance out;
  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    const double d = body_segment_distance(pose, body, obstacles[i]);
    if (d < out.distance_m) {
      out.distance_m = d;
      out.obstacle_index = i;
    }
  }
  return out;
}

namespace {

// The rear-axle travel at which sample `i` of `n` (n >= 1 intervals) of a motion sits.
double sample_travel(double total_m, std::size_t i, std::size_t n) {
  return total_m * static_cast<double>(i) / static_cast<double>(n);
}

std::size_t sample_intervals(double distance_m, double step_m) {
  const double steps = std::ceil(std::abs(distance_m) / step_m);
  return std::max<std::size_t>(1, static_cast<std::size_t>(steps));
}

}  // namespace

SweepResult sweep_body(const Pose2& start, const ParkMotion& motion, const ParkBody& body,
                       const std::vector<Segment2>& obstacles, double required_clearance_m,
                       double step_m, bool stop_at_violation) {
  SweepResult out;
  const double curvature = curvature_for_steering(motion.steering_rad, body.wheelbase_m);
  out.end = advance_pose(start, curvature, motion.distance_m);
  if (!(step_m > 0.0) || !std::isfinite(motion.distance_m)) {
    out.clear = false;
    out.min_clearance_m = 0.0;
    return out;
  }
  const std::size_t n = sample_intervals(motion.distance_m, step_m);
  for (std::size_t i = 0; i <= n; ++i) {
    const double travel = sample_travel(motion.distance_m, i, n);
    const Clearance c = body_clearance(advance_pose(start, curvature, travel), body, obstacles);
    if (c.distance_m < out.min_clearance_m) {
      out.min_clearance_m = c.distance_m;
      out.worst_travel_m = std::abs(travel);
      out.worst_obstacle = c.obstacle_index;
    }
    if (stop_at_violation && c.distance_m < required_clearance_m) {
      break;
    }
  }
  out.clear = out.min_clearance_m >= required_clearance_m;
  return out;
}

double gate_threshold_m(const ParkGateModel& gate, double speed_mps) {
  double threshold = gate.floor_m;
  if (gate.ttc_brake_s > 0.0) {
    threshold = std::max(threshold, std::abs(speed_mps) * gate.ttc_brake_s);
  }
  return threshold + gate.margin_m;
}

double gate_cull_radius_m(double steering_rad, bool reverse, const ParkBody& body,
                          const ParkGateModel& gate, double threshold_m) {
  const double delta = std::clamp(steering_rad, -gate.max_steering_rad, gate.max_steering_rad);
  const double chw = gate.corridor_half_width_m;
  const double straight_bound = threshold_m + chw + std::abs(body.lidar_y_m);
  if (std::abs(delta) < kGateStraightSteeringRad) {
    return straight_bound + 1e-6;
  }
  const double abs_radius = std::abs(body.wheelbase_m / std::tan(delta));
  const double lead_x = reverse ? body.rear_x_m : body.front_x_m;
  const double r_out = std::hypot(lead_x, abs_radius + chw);
  const double arc_bound = (r_out - abs_radius + chw) + r_out * threshold_m / abs_radius;
  return std::max(straight_bound, arc_bound) + 1e-6;
}

double predicted_gate_distance_m(const Pose2& pose, double steering_rad, bool reverse,
                                 const std::vector<Point2>& points, const ParkBody& body,
                                 const ParkGateModel& gate, double ignore_beyond_m) {
  const double delta = std::clamp(steering_rad, -gate.max_steering_rad, gate.max_steering_rad);
  const bool straight = std::abs(delta) < kGateStraightSteeringRad;
  const double radius = straight ? 0.0 : body.wheelbase_m / std::tan(delta);
  const double chw = gate.corridor_half_width_m;
  const Point2 ref = reverse ? Point2{-body.rear_x_m, 0.0} : Point2{body.lidar_x_m, body.lidar_y_m};
  const double cull2 = ignore_beyond_m * ignore_beyond_m;
  double best = std::numeric_limits<double>::infinity();
  for (const Point2& world : points) {
    const Point2 p = to_local(pose, world);
    if (std::isfinite(ignore_beyond_m)) {
      const double dx = p.x - ref.x;
      const double dy = p.y - ref.y;
      if (dx * dx + dy * dy > cull2) {
        continue;
      }
    }
    // Outer sector bound, seen from the head (centred on -x for the rear corridor).
    const double bearing = std::atan2(p.y - body.lidar_y_m, p.x - body.lidar_x_m);
    const double off_axis = reverse ? wrap_pi(bearing - M_PI) : bearing;
    if (std::abs(off_axis) > gate.sector_half_angle_rad) {
      continue;
    }
    std::optional<double> d;
    if (!reverse) {
      if (straight) {
        const double ahead = p.x - body.lidar_x_m;
        if (ahead > 0.0 && std::abs(p.y) <= chw) {
          d = ahead;
        }
      } else {
        d = arc_distance(p.x, p.y, chw, radius, body.front_x_m, gate.outer_corner_horizon_m,
                         body.lidar_x_m, body.lidar_y_m);
      }
    } else {
      const double xm = -p.x;
      const double ym = p.y;
      if (straight) {
        const double behind = xm - body.rear_x_m;
        if (behind > 0.0 && std::abs(ym) <= chw) {
          d = behind;
        }
      } else {
        d = arc_distance(xm, ym, chw, radius, body.rear_x_m, gate.outer_corner_horizon_m,
                         body.rear_x_m, 0.0);
      }
    }
    if (d && *d < best) {
      best = *d;
    }
  }
  return best;
}

GateSweepResult sweep_gate(const Pose2& start, const ParkMotion& motion, double speed_mps,
                           const std::vector<Point2>& points, const ParkBody& body,
                           const ParkGateModel& gate, double step_m, bool stop_at_violation) {
  GateSweepResult out;
  if (!gate.enabled || motion.distance_m == 0.0) {
    return out;
  }
  if (!(step_m > 0.0) || !std::isfinite(motion.distance_m)) {
    out.clear = false;
    out.min_distance_m = 0.0;
    return out;
  }
  const bool reverse = motion.distance_m < 0.0;
  const double curvature = curvature_for_steering(motion.steering_rad, body.wheelbase_m);
  const double threshold = gate_threshold_m(gate, speed_mps);
  const double cull = gate_cull_radius_m(motion.steering_rad, reverse, body, gate, threshold);
  const std::size_t n = sample_intervals(motion.distance_m, step_m);
  for (std::size_t i = 0; i <= n; ++i) {
    const double travel = sample_travel(motion.distance_m, i, n);
    const double d =
        predicted_gate_distance_m(advance_pose(start, curvature, travel), motion.steering_rad,
                                  reverse, points, body, gate, cull);
    if (d < out.min_distance_m) {
      out.min_distance_m = d;
      out.worst_travel_m = std::abs(travel);
    }
    if (stop_at_violation && d <= threshold) {
      break;
    }
  }
  out.clear = out.min_distance_m > threshold;
  return out;
}

std::vector<Point2> sample_segments(const std::vector<Segment2>& segments, double spacing_m) {
  std::vector<Point2> out;
  if (!(spacing_m > 0.0)) {
    return out;
  }
  for (const Segment2& s : segments) {
    const double len = std::hypot(s.b.x - s.a.x, s.b.y - s.a.y);
    const std::size_t n = sample_intervals(len, spacing_m);
    for (std::size_t i = 0; i <= n; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(n);
      out.push_back({s.a.x + t * (s.b.x - s.a.x), s.a.y + t * (s.b.y - s.a.y)});
    }
  }
  return out;
}

}  // namespace racer_control
