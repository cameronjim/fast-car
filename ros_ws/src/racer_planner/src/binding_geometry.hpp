// Fills the ROS-free VehicleGeometry from the generated vehicle_params binding. Included by the
// node, the bench and the tests, never by racer_planner_core.
#ifndef RACER_PLANNER_BINDING_GEOMETRY_HPP_
#define RACER_PLANNER_BINDING_GEOMETRY_HPP_

#include <optional>
#include <string>
#include <vehicle_params_generated.hpp>

#include "racer_planner/vehicle_geometry.hpp"

namespace racer_planner {

// nullopt (with the reason in *why) when a field the planner needs is null in vehicle_params.
inline std::optional<VehicleGeometry> geometry_from_binding(const VehicleParams& p,
                                                            std::string* why) {
  const auto& l = p.sensors.lidar;
  if (!l.mount_x_m.has_value() || !l.mount_y_m.has_value() || !l.mount_yaw_rad.has_value()) {
    if (why != nullptr)
      *why = "vehicle_params sensors.lidar.mount_x_m / mount_y_m / mount_yaw_rad is null";
    return std::nullopt;
  }
  VehicleGeometry g;
  g.wheelbase_m = p.chassis.wheelbase_m;
  g.width_m = p.chassis.width_m;
  g.rear_overhang_m = p.chassis.rear_overhang_m;
  g.front_overhang_m = p.chassis.front_overhang_m;
  g.steering_min_rad = p.steering.min_angle_rad;
  g.steering_max_rad = p.steering.max_angle_rad;
  g.steering_max_rate_rad_per_s = p.steering.max_rate_rad_per_s;
  g.max_acceleration_mps2 = p.actuation.max_acceleration_mps2;
  g.stop_deceleration_mps2 = p.actuation.stop_deceleration_mps2;
  g.lidar_mount_x_m = *l.mount_x_m;
  g.lidar_mount_y_m = *l.mount_y_m;
  g.lidar_mount_yaw_rad = *l.mount_yaw_rad;
  g.schema_version = p.meta.schema_version;
  g.sysid_session_id = p.meta.sysid_session_id;
  if (!geometry_is_usable(g)) {
    if (why != nullptr) *why = "vehicle_params geometry is not usable";
    return std::nullopt;
  }
  return g;
}

}  // namespace racer_planner

#endif  // RACER_PLANNER_BINDING_GEOMETRY_HPP_
