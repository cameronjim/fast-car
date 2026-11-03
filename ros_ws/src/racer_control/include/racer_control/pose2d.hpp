// Planar pose composition for tracker_node's map-frame pose (2026-11-03 localisation).
//
// With localisation running, the EKF's /odom pose is in the odom frame (smooth, drifting) and
// AMCL or slam_toolbox publishes map -> odom (jumpy, drift-free). The pose the raceline is
// written in is map -> base_link = (map -> odom) composed with (odom -> base_link). tracker_node
// reads the first from TF and the second from /odom, and composes them here. ROS-free.
#ifndef RACER_CONTROL_POSE2D_HPP_
#define RACER_CONTROL_POSE2D_HPP_

namespace racer_control {

struct Pose2D {
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
};

// parent_T_child = parent_T_mid composed with mid_T_child; yaw wrapped to [-pi, pi) by
// racer_control::wrap_angle (laser_scan.hpp).
Pose2D compose(const Pose2D& parent_from_mid, const Pose2D& mid_from_child);

}  // namespace racer_control

#endif  // RACER_CONTROL_POSE2D_HPP_
