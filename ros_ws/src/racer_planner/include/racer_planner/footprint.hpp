// The body as three circles on the centre line. The body rectangle runs from -rear_overhang_m to
// wheelbase_m + front_overhang_m with half width width_m / 2. Circle centres sit at the middle of
// each third, radius sqrt((length / 6)^2 + (width / 2)^2), so every rectangle corner is covered.
#ifndef RACER_PLANNER_FOOTPRINT_HPP_
#define RACER_PLANNER_FOOTPRINT_HPP_

#include <array>
#include <cmath>

#include "racer_planner/distance_field.hpp"
#include "racer_planner/vehicle_geometry.hpp"

namespace racer_planner {

class Footprint {
 public:
  explicit Footprint(const VehicleGeometry& g) {
    const double len = g.body_length_m();
    radius_ = std::sqrt((len / 6.0) * (len / 6.0) + (g.width_m / 2.0) * (g.width_m / 2.0));
    for (int k = 0; k < 3; ++k) {
      offsets_[k] = g.body_min_x_m() + (k + 0.5) * len / 3.0;
    }
  }

  double radius() const { return radius_; }
  // Circle centre offsets along the body x axis from the rear axle.
  const std::array<double, 3>& offsets() const { return offsets_; }

  // Clearance of a pose (rear axle position and yaw): min over circles of the distance field at
  // the circle centre minus the radius. Negative means a scan point is inside a circle.
  double clearance(const DistanceField& field, double x, double y, double cos_yaw,
                   double sin_yaw) const {
    double best = 1e9;
    for (int k = 0; k < 3; ++k) {
      const double d = field.distance(x + offsets_[k] * cos_yaw, y + offsets_[k] * sin_yaw);
      if (d < best) best = d;
    }
    return best - radius_;
  }

  // True when the point (in the body frame) lies inside at least one circle.
  bool covers_body_point(double bx, double by) const {
    for (int k = 0; k < 3; ++k) {
      const double dx = bx - offsets_[k];
      if (dx * dx + by * by <= radius_ * radius_ + 1e-12) return true;
    }
    return false;
  }

 private:
  double radius_ = 0.0;
  std::array<double, 3> offsets_{};
};

}  // namespace racer_planner

#endif  // RACER_PLANNER_FOOTPRINT_HPP_
