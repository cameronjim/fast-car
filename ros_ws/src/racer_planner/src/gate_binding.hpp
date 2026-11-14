// Fills GateGeometry and GateLimits from the generated vehicle_params binding, with the same
// numbers racer_safety's safety_node hands to its in-path test and its obstacle gate. Included by
// the node, the replay CLI, the bench and the tests, never by racer_planner_core.
#ifndef RACER_PLANNER_GATE_BINDING_HPP_
#define RACER_PLANNER_GATE_BINDING_HPP_

#include <optional>
#include <string>
#include <vehicle_params_generated.hpp>

#include "racer_planner/gate_predictor.hpp"

namespace racer_planner {

struct GateConfig {
  GateGeometry geometry;
  GateLimits limits;
};

// nullopt (with the reason in *why) when a field the gate prediction needs is null in
// vehicle_params, the way safety_node refuses to start on the same fields.
inline std::optional<GateConfig> gate_from_binding(const VehicleParams& p, std::string* why) {
  const auto& l = p.sensors.lidar;
  if (!l.mount_x_m.has_value() || !l.mount_y_m.has_value() || !l.mount_yaw_rad.has_value()) {
    if (why != nullptr) {
      *why = "vehicle_params sensors.lidar.mount_x_m / mount_y_m / mount_yaw_rad is null";
    }
    return std::nullopt;
  }
  if (!p.limits.ttc_brake_s.has_value() || !p.limits.ttc_warning_s.has_value()) {
    if (why != nullptr) *why = "vehicle_params limits.ttc_brake_s / ttc_warning_s is null";
    return std::nullopt;
  }
  GateConfig c;
  GateGeometry& g = c.geometry;
  g.wheelbase_m = p.chassis.wheelbase_m;
  g.steering_min_rad = p.steering.min_angle_rad;
  g.steering_max_rad = p.steering.max_angle_rad;
  g.max_steering_angle_rad = p.steering.max_angle_rad;
  g.lidar_mount_x_m = *l.mount_x_m;
  g.lidar_mount_y_m = *l.mount_y_m;
  g.lidar_mount_yaw_rad = *l.mount_yaw_rad;
  g.body_front_x_m = p.chassis.wheelbase_m + p.chassis.front_overhang_m;
  g.corridor_half_width_m = 0.5 * p.chassis.width_m + p.limits.obstacle_corridor_margin_m;
  g.sector_half_angle_rad = p.limits.ttc_forward_sector_half_angle_rad;
  g.outer_corner_horizon_m = p.limits.outer_corner_horizon_m;
  c.limits.ttc_brake_s = *p.limits.ttc_brake_s;
  c.limits.ttc_warning_s = *p.limits.ttc_warning_s;
  c.limits.min_forward_clearance_m = p.limits.min_forward_clearance_m;
  if (!gate_geometry_is_usable(g) || !gate_limits_are_usable(c.limits)) {
    if (why != nullptr) *why = "vehicle_params gate geometry or limits are not usable";
    return std::nullopt;
  }
  return c;
}

}  // namespace racer_planner

#endif  // RACER_PLANNER_GATE_BINDING_HPP_
