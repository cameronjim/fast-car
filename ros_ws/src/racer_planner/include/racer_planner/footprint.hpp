// The body as circles on the distance field, all numbers from VehicleGeometry.
//
// The body rectangle runs from -rear_overhang_m to wheelbase_m + front_overhang_m with half width
// width_m / 2. Seven circles in the body frame (origin on the rear axle):
//   * three body circles of radius width_m / 2 centred on the centre line at x = body_min +
//     width / 2, the middle, and body_max - width / 2. They touch the front and the sides of the
//     rectangle and never reach past it.
//   * two front corner circles of radius corner_radius_m centred at (body_max - r, +/-(width/2 -
//     r)), and two rear corner circles likewise at body_min + r. They sit inside the rectangle,
//     tangent to the two edges of their corner.
// Driving forward is checked with the three body circles and the two front corner circles;
// driving backward with all seven. Nothing is outside the rectangle (reach 0 in front and to the
// side). The rectangle corner itself is left uncovered by r * (sqrt(2) - 1) (about 2.5 cm at the
// default 6 cm corner radius), less than the hard collision clearance d_hard_m.
#ifndef RACER_PLANNER_FOOTPRINT_HPP_
#define RACER_PLANNER_FOOTPRINT_HPP_

#include <algorithm>
#include <array>
#include <cmath>

#include "racer_planner/distance_field.hpp"
#include "racer_planner/vehicle_geometry.hpp"

namespace racer_planner {

struct Circle {
  double x = 0.0;  // centre in the body frame, from the rear axle
  double y = 0.0;
  double r = 0.0;
};

class Footprint {
 public:
  static constexpr double kDefaultCornerRadiusM = 0.06;
  static constexpr int kBodyCircles = 3;
  static constexpr int kForwardCircles = 5;  // body + front corners
  static constexpr int kAllCircles = 7;      // + rear corners

  explicit Footprint(const VehicleGeometry& g, double corner_radius_m = kDefaultCornerRadiusM) {
    const double hw = g.width_m / 2.0;
    radius_ = hw;
    corner_radius_ = std::min(std::max(corner_radius_m, 1e-3), hw);
    const double x_lo = g.body_min_x_m(), x_hi = g.body_max_x_m();
    offsets_[0] = x_lo + hw;
    offsets_[1] = 0.5 * (x_lo + x_hi);
    offsets_[2] = x_hi - hw;
    for (int k = 0; k < kBodyCircles; ++k) circles_[k] = Circle{offsets_[k], 0.0, radius_};
    const double r = corner_radius_;
    circles_[3] = Circle{x_hi - r, hw - r, r};
    circles_[4] = Circle{x_hi - r, -(hw - r), r};
    circles_[5] = Circle{x_lo + r, hw - r, r};
    circles_[6] = Circle{x_lo + r, -(hw - r), r};
  }

  // Radius of the three body circles (half the width).
  double radius() const { return radius_; }
  double corner_radius() const { return corner_radius_; }
  // Body circle centre offsets along the body x axis from the rear axle.
  const std::array<double, 3>& offsets() const { return offsets_; }
  const std::array<Circle, kAllCircles>& circles() const { return circles_; }

  // Clearance of a forward moving pose (rear axle position and yaw): min over the body and front
  // corner circles of the distance field at the centre minus the radius. Negative means a scan
  // point is inside a circle.
  double clearance(const DistanceField& field, double x, double y, double cos_yaw,
                   double sin_yaw) const {
    return min_clearance(field, x, y, cos_yaw, sin_yaw, kForwardCircles);
  }

  // The same with all seven circles, for reverse arcs.
  double clearance_reverse(const DistanceField& field, double x, double y, double cos_yaw,
                           double sin_yaw) const {
    return min_clearance(field, x, y, cos_yaw, sin_yaw, kAllCircles);
  }

  // True when the point (in the body frame) lies inside at least one of the seven circles.
  bool covers_body_point(double bx, double by) const {
    for (const Circle& c : circles_) {
      const double dx = bx - c.x, dy = by - c.y;
      if (dx * dx + dy * dy <= c.r * c.r + 1e-12) return true;
    }
    return false;
  }

 private:
  double min_clearance(const DistanceField& field, double x, double y, double cos_yaw,
                       double sin_yaw, int count) const {
    double best = 1e9;
    for (int k = 0; k < count; ++k) {
      const Circle& c = circles_[static_cast<std::size_t>(k)];
      const double d =
          field.distance(x + c.x * cos_yaw - c.y * sin_yaw, y + c.x * sin_yaw + c.y * cos_yaw) -
          c.r;
      if (d < best) best = d;
    }
    return best;
  }

  double radius_ = 0.0;
  double corner_radius_ = 0.0;
  std::array<double, 3> offsets_{};
  std::array<Circle, kAllCircles> circles_{};
};

}  // namespace racer_planner

#endif  // RACER_PLANNER_FOOTPRINT_HPP_
