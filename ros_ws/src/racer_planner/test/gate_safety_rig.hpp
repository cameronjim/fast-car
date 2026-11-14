// racer_safety's in-path distance and gate, set up from the vehicle_params binding exactly the way
// safety_node does it. Test only: this compiles racer_safety's own sources read only (see the
// racer_planner CMakeLists.txt), production code never does.
#ifndef RACER_PLANNER_GATE_SAFETY_RIG_HPP_
#define RACER_PLANNER_GATE_SAFETY_RIG_HPP_

#include "gate_test_support.hpp"
#include "racer_safety/forward_sector.hpp"
#include "racer_safety/gate_logic.hpp"

namespace racer_planner::test {

// What safety_node builds, from the same binding fields.
struct SafetyRig {
  racer_safety::PathGeometry path;
  double laser_yaw_rad = 0.0;
  double half_angle_rad = 0.0;
  double corridor_half_width_m = 0.0;

  SafetyRig() {
    const VehicleParams& p = VEHICLE_PARAMS;
    path.wheelbase_m = p.chassis.wheelbase_m;
    path.max_steering_angle_rad = p.steering.max_angle_rad;
    path.lidar_mount_x_m = *p.sensors.lidar.mount_x_m;
    path.lidar_mount_y_m = *p.sensors.lidar.mount_y_m;
    path.body_front_x_m = p.chassis.wheelbase_m + p.chassis.front_overhang_m;
    path.outer_corner_horizon_m = p.limits.outer_corner_horizon_m;
    path.distance_from_body = true;
    laser_yaw_rad = *p.sensors.lidar.mount_yaw_rad;
    half_angle_rad = p.limits.ttc_forward_sector_half_angle_rad;
    corridor_half_width_m =
        racer_safety::corridor_half_width_m(p.chassis.width_m, p.limits.obstacle_corridor_margin_m);
  }

  double distance(const ScanData& s, double delta) const {
    racer_safety::ScanGeometry geom;
    geom.angle_min_rad = s.angle_min_rad;
    geom.angle_increment_rad = s.angle_increment_rad;
    geom.range_min_m = s.range_min_m;
    geom.range_max_m = s.range_max_m;
    return racer_safety::min_path_distance_m(geom, s.ranges, laser_yaw_rad, half_angle_rad,
                                             corridor_half_width_m, path, delta);
  }
};

racer_safety::SafetyLimits safety_limits(bool with_model) {
  const VehicleParams& p = VEHICLE_PARAMS;
  racer_safety::SafetyLimits l;
  l.steering_min_rad = p.steering.min_angle_rad;
  l.steering_max_rad = p.steering.max_angle_rad;
  l.steering_rate_min_rad_per_s = p.steering.min_rate_rad_per_s;
  l.steering_rate_max_rad_per_s = p.steering.max_rate_rad_per_s;
  l.speed_min_mps = p.limits.min_velocity_mps;
  l.speed_max_mps = p.limits.global_speed_cap_mps;
  l.max_acceleration_mps2 = p.actuation.max_acceleration_mps2;
  l.ttc_warning_s = *p.limits.ttc_warning_s;
  l.ttc_brake_s = *p.limits.ttc_brake_s;
  l.min_forward_clearance_m = p.limits.min_forward_clearance_m;
  l.obstacle_steering_hold_after_s = p.limits.obstacle_steering_hold_after_s;
  l.speed_bound_deceleration_mps2 = p.actuation.stop_deceleration_mps2;
  if (with_model) {
    racer_safety::MeasuredSpeedModel m;
    m.latency_s = p.actuation.stop_latency_s;
    m.peak_acceleration_mps2 = p.actuation.peak_acceleration_mps2;
    l.measured_speed_model = m;
  }
  return l;
}

}  // namespace racer_planner::test

#endif  // RACER_PLANNER_GATE_SAFETY_RIG_HPP_
