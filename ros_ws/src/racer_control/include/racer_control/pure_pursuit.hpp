// Pure pursuit tracking controller with curvature-adaptive lookahead (roadmap task S.2).
//
// ROS-free (no rclcpp) so it is gtest-unit-testable with no ROS install
// (claude-docs/12-testing.md L1, claude-docs/10-conventions.md "gate/decision logic is
// always separated from node plumbing"). `tracker_node` owns the ROS plumbing (subscribing
// /odom, publishing /drive_raw at 50 Hz) and calls `PurePursuitController::compute_command`
// once per cycle; nothing here allocates on the heap in `compute_command` or
// `lookahead_distance_m` (claude-docs/10-conventions.md: no heap allocation in the 50 Hz
// control path after init).
//
// Sign convention (claude-docs/06-vehicle-params.md, REP-103): steering angle is the
// road-wheel angle in radians, LEFT positive; yaw is counter-clockwise positive, x forward,
// y left. A lookahead target to the vehicle's left (positive y in the vehicle frame)
// produces a positive commanded curvature and therefore a positive (left) steering angle --
// verified against the exact left/right/straight cases from that doc in
// test/test_pure_pursuit.cpp.
#ifndef RACER_CONTROL_PURE_PURSUIT_HPP_
#define RACER_CONTROL_PURE_PURSUIT_HPP_

#include "racer_control/raceline.hpp"

namespace racer_control {

struct PurePursuitConfig {
  double wheelbase_m;
  // Curvature-adaptive lookahead: lookahead_max_m on straights (curvature 0), shrinking
  // linearly to lookahead_min_m as |curvature| approaches lookahead_curvature_ref_1pm (and
  // clamped at lookahead_min_m beyond that), so the controller looks less far ahead in
  // tight turns (where a distant lookahead point cuts the corner) and farther ahead on
  // straights (where a short lookahead causes steering chatter).
  double lookahead_min_m;
  double lookahead_max_m;
  double lookahead_curvature_ref_1pm;
  // Steering saturation, from vehicle_params (steering.min_angle_rad / max_angle_rad are
  // symmetric in the committed config; this takes the positive bound and clamps
  // symmetrically -- see tracker_node.cpp for where this is populated from the generated
  // binding, never hand-typed).
  double max_steering_angle_rad;
};

struct PurePursuitCommand {
  double steering_angle_rad;
  double speed_mps;
};

class PurePursuitController {
 public:
  explicit PurePursuitController(PurePursuitConfig config) : config_(config) {}

  // curvature_1pm is the SIGNED curvature of the raceline at the vehicle's current nearest
  // point (only its magnitude matters for the lookahead formula).
  double lookahead_distance_m(double curvature_1pm) const;

  // x_m, y_m, yaw_rad: current vehicle pose in the same frame the raceline was generated
  // in (REP-103 map/odom frame). Looks up the nearest raceline point for the speed command
  // and the curvature used by the lookahead formula, walks forward to the lookahead point,
  // and computes steering from the standard pure-pursuit curvature formula
  // (`2*y_vehicle_frame / lookahead_distance^2`).
  PurePursuitCommand compute_command(const Raceline& raceline, double x_m, double y_m,
                                     double yaw_rad) const;

 private:
  PurePursuitConfig config_;
};

// Planar (yaw-only) heading from a geometry_msgs/Quaternion's components, without pulling
// in a tf2 dependency for a single formula. Pure math, so it is gtest-unit-tested directly
// (test/test_pure_pursuit.cpp) rather than only exercised indirectly through tracker_node.
double yaw_from_quaternion(double w, double x, double y, double z);

}  // namespace racer_control

#endif  // RACER_CONTROL_PURE_PURSUIT_HPP_
