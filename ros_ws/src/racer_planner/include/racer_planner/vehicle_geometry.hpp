// Plain vehicle constants the ROS-free core needs. The node, the bench and the tests fill this
// from the generated vehicle_params binding (src/binding_geometry.hpp); the core never includes
// the generated header and never types a physical number (CLAUDE.md invariant 2).
#ifndef RACER_PLANNER_VEHICLE_GEOMETRY_HPP_
#define RACER_PLANNER_VEHICLE_GEOMETRY_HPP_

#include <cmath>
#include <string>

namespace racer_planner {

struct VehicleGeometry {
  double wheelbase_m = 0.0;
  double width_m = 0.0;
  double rear_overhang_m = 0.0;   // aft of the rear axle
  double front_overhang_m = 0.0;  // forward of the front axle
  double steering_min_rad = 0.0;
  double steering_max_rad = 0.0;
  double steering_max_rate_rad_per_s = 0.0;
  double max_acceleration_mps2 = 0.0;
  double stop_deceleration_mps2 = 0.0;
  double lidar_mount_x_m = 0.0;
  double lidar_mount_y_m = 0.0;
  double lidar_mount_yaw_rad = 0.0;
  std::string schema_version;
  std::string sysid_session_id;

  // Body rectangle in base_link: x from -rear_overhang_m to wheelbase_m + front_overhang_m.
  double body_min_x_m() const { return -rear_overhang_m; }
  double body_max_x_m() const { return wheelbase_m + front_overhang_m; }
  double body_length_m() const { return body_max_x_m() - body_min_x_m(); }
};

inline bool geometry_is_usable(const VehicleGeometry& g) {
  const double v[] = {g.wheelbase_m,
                      g.width_m,
                      g.rear_overhang_m,
                      g.front_overhang_m,
                      g.steering_min_rad,
                      g.steering_max_rad,
                      g.steering_max_rate_rad_per_s,
                      g.max_acceleration_mps2,
                      g.stop_deceleration_mps2,
                      g.lidar_mount_x_m,
                      g.lidar_mount_y_m,
                      g.lidar_mount_yaw_rad};
  for (double x : v) {
    if (!std::isfinite(x)) return false;
  }
  return g.wheelbase_m > 0.0 && g.width_m > 0.0 && g.rear_overhang_m >= 0.0 &&
         g.front_overhang_m >= 0.0 && g.steering_min_rad < g.steering_max_rad &&
         g.steering_max_rate_rad_per_s > 0.0 && g.max_acceleration_mps2 > 0.0 &&
         g.stop_deceleration_mps2 > 0.0;
}

}  // namespace racer_planner

#endif  // RACER_PLANNER_VEHICLE_GEOMETRY_HPP_
