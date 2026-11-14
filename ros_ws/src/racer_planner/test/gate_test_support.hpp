// Helpers shared by the gate predictor, divergence and closed loop tests: the gate geometry from
// the binding, synthetic LiDAR scans made of wall segments, and a seeded random scene generator.
#ifndef RACER_PLANNER_GATE_TEST_SUPPORT_HPP_
#define RACER_PLANNER_GATE_TEST_SUPPORT_HPP_

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "gate_binding.hpp"
#include "racer_planner/gate_predictor.hpp"

namespace racer_planner::test {

inline GateConfig binding_gate() {
  std::string why;
  const auto c = gate_from_binding(VEHICLE_PARAMS, &why);
  if (!c) {
    ADD_FAILURE() << why;
    return GateConfig{};
  }
  return *c;
}

inline double wrap_pi(double a) {
  double w = std::fmod(a + M_PI, 2.0 * M_PI);
  if (w < 0.0) w += 2.0 * M_PI;
  return w - M_PI;
}

// A 720 beam, 360 degree scan, beam 0 at -pi (the RPLIDAR C1 mounted with yaw pi sees ahead of the
// car at the middle of the array).
struct ScanData {
  std::vector<float> ranges;
  double angle_min_rad = -M_PI;
  double angle_increment_rad = 2.0 * M_PI / 720.0;
  double range_min_m = 0.0;
  double range_max_m = 0.0;

  explicit ScanData(int beams = 720)
      : ranges(static_cast<std::size_t>(beams), std::numeric_limits<float>::infinity()),
        angle_increment_rad(2.0 * M_PI / beams) {}

  GateScan view() const {
    GateScan s;
    s.ranges = ranges.data();
    s.count = ranges.size();
    s.angle_min_rad = angle_min_rad;
    s.angle_increment_rad = angle_increment_rad;
    s.range_min_m = range_min_m;
    s.range_max_m = range_max_m;
    return s;
  }

  // Vehicle bearing (about the LiDAR head, after the mount yaw) of beam i.
  double vehicle_bearing(const GateGeometry& g, int i) const {
    return wrap_pi(angle_min_rad + i * angle_increment_rad + g.lidar_mount_yaw_rad);
  }

  // Head frame (x ahead, y left, vehicle axes) of a return on beam i at `range`.
  void point(const GateGeometry& g, int i, double range, double* x, double* y) const {
    const double b = vehicle_bearing(g, i);
    *x = range * std::cos(b);
    *y = range * std::sin(b);
  }
};

inline ScanData make_scan(int beams = 720) {
  ScanData s(beams);
  s.range_min_m = VEHICLE_PARAMS.sensors.lidar_spec.range_min_m;
  s.range_max_m = VEHICLE_PARAMS.sensors.lidar_spec.range_max_m;
  return s;
}

struct Segment {
  double x1, y1, x2, y2;  // head frame, vehicle axes
};

// Nearest intersection of the ray from the head at vehicle bearing `b` with the segments, or inf.
inline double cast(const std::vector<Segment>& segs, double b) {
  const double dx = std::cos(b), dy = std::sin(b);
  double best = std::numeric_limits<double>::infinity();
  for (const Segment& s : segs) {
    const double ex = s.x2 - s.x1, ey = s.y2 - s.y1;
    const double den = dx * ey - dy * ex;
    if (std::fabs(den) < 1e-12) continue;
    const double t = (s.x1 * ey - s.y1 * ex) / den;
    const double u = (s.x1 * dy - s.y1 * dx) / den;
    if (t > 0.0 && u >= 0.0 && u <= 1.0 && t < best) best = t;
  }
  return best;
}

inline ScanData scan_of(const GateGeometry& g, const std::vector<Segment>& segs, int beams = 720) {
  ScanData s = make_scan(beams);
  for (int i = 0; i < beams; ++i) {
    const double r = cast(segs, s.vehicle_bearing(g, i));
    if (std::isfinite(r) && r <= s.range_max_m) s.ranges[static_cast<std::size_t>(i)] = float(r);
  }
  return s;
}

// A wall across the road at x (head frame), spanning y in [-half_span, half_span].
inline std::vector<Segment> wall_across_road(double x_head, double half_span = 3.0) {
  return {Segment{x_head, -half_span, x_head, half_span}};
}

// A seeded random scene: a mix of walls across the road at random angles, side walls, poles and
// noise, with dropouts and a few garbage returns. Returns the segments' scan.
struct SceneGenerator {
  std::mt19937_64 rng;
  explicit SceneGenerator(std::uint64_t seed) : rng(seed) {}

  double u(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); }
  int pick(int n) { return std::uniform_int_distribution<int>(0, n - 1)(rng); }

  ScanData make(const GateGeometry& g) {
    std::vector<Segment> segs;
    const int n_obj = 1 + pick(4);
    for (int k = 0; k < n_obj; ++k) {
      const int kind = pick(4);
      if (kind == 0) {  // a wall across the road at a random angle
        const double cx = u(0.05, 2.2), cy = u(-0.6, 0.6), a = u(-1.2, 1.2), h = u(0.2, 1.5);
        segs.push_back({cx - h * std::sin(a), cy + h * std::cos(a), cx + h * std::sin(a),
                        cy - h * std::cos(a)});
      } else if (kind == 1) {  // a side wall roughly along the road
        const double y0 = u(-1.0, 1.0), a = u(-0.5, 0.5), x0 = u(-0.3, 1.0), len = u(0.5, 3.0);
        segs.push_back({x0, y0, x0 + len * std::cos(a), y0 + len * std::sin(a)});
      } else if (kind == 2) {  // a pole: a very short segment
        const double cx = u(0.05, 2.0), cy = u(-0.8, 0.8);
        segs.push_back({cx, cy, cx + 0.03, cy + 0.03});
      } else {  // a corner: two walls meeting
        const double cx = u(0.1, 2.0), cy = u(-0.8, 0.8), a = u(-1.5, 1.5);
        segs.push_back({cx, cy, cx + 1.5 * std::cos(a), cy + 1.5 * std::sin(a)});
        segs.push_back({cx, cy, cx + 1.5 * std::cos(a + 1.57), cy + 1.5 * std::sin(a + 1.57)});
      }
    }
    ScanData s = make_scan();
    const double noise = u(0.0, 0.02);
    const double dropout = u(0.0, 0.2);
    std::normal_distribution<double> nd(0.0, noise > 0.0 ? noise : 1e-9);
    for (int i = 0; i < 720; ++i) {
      double r = cast(segs, s.vehicle_bearing(g, i));
      if (!std::isfinite(r) || r > s.range_max_m) continue;
      if (u(0.0, 1.0) < dropout) {
        const int how = pick(4);
        if (how == 0) s.ranges[static_cast<std::size_t>(i)] = 0.0f;
        continue;
      }
      r += nd(rng);
      s.ranges[static_cast<std::size_t>(i)] = float(std::max(r, 0.01));
    }
    // A little garbage now and then.
    if (pick(20) == 0) {
      for (int k = 0; k < 5; ++k) {
        const int i = pick(720);
        const int how = pick(3);
        s.ranges[static_cast<std::size_t>(i)] =
            how == 0 ? std::numeric_limits<float>::quiet_NaN() : (how == 1 ? -1.0f : 100.0f);
      }
    }
    return s;
  }
};

}  // namespace racer_planner::test

#endif  // RACER_PLANNER_GATE_TEST_SUPPORT_HPP_
