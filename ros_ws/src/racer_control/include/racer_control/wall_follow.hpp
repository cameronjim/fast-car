// Wall-following reactive controller core (GitHub issue 26 item g), ported from the old
// Python reactive_control/wall_logic.py. ROS-free and gtest-covered
// (test/test_wall_follow.cpp); wall_follow_node owns the ROS plumbing and the PID (pid.hpp).
//
// Geometry (the standard two-ray construction, unchanged from the old code): ray b points at
// the wall (default -90 deg, the right side), ray a is theta further forward (default -20
// deg, so theta = 70 deg). With a = range along ray a, b = range along ray b:
//     alpha        = atan((a cos(theta) - b) / (a sin(theta)))   wall angle vs heading
//     D            = b cos(alpha)                                current distance to the wall
//     D_lookahead  = D + L sin(alpha)                            projected L metres ahead
//     error        = target_distance - D_lookahead               (0 inside the deadband)
// alpha > 0 means the car is heading away from the wall. error > 0 means "too close".
//
// What changed: the old lookahead was L = speed * dt, one control period of travel, which is
// a few centimetres and made the lookahead term nearly inert. L is now a distance parameter
// (lookahead_m, default about 0.5 m). The ray bearings are parameters in the VEHICLE frame,
// left positive: negative bearings follow the RIGHT wall, positive bearings the LEFT wall,
// and the steering sign flips with the side (wall_steering_sign). A ray bearing outside the
// scan's coverage is reported as no measurement (the old code silently clamped it to the end
// ray, which measured some other direction).
//
// Sign check against LEFT POSITIVE steering (claude-docs/06-vehicle-params.md), pinned in
// test/test_wall_follow.cpp: right wall, car too far from it -> error < 0 -> steering < 0
// (turn right, toward the wall). Left wall, car too far -> steering > 0.
#ifndef RACER_CONTROL_WALL_FOLLOW_HPP_
#define RACER_CONTROL_WALL_FOLLOW_HPP_

#include <optional>

#include "racer_control/laser_scan.hpp"

namespace racer_control {

struct WallFollowConfig {
  double ray_a_bearing_rad = 0.0;  // forward ray, vehicle frame, left positive
  double ray_b_bearing_rad = 0.0;  // wall-normal ray, same side as ray a, |b| > |a|
  double target_distance_m = 0.0;
  double lookahead_m = 0.0;
  double deadband_m = 0.0;
  double laser_yaw_offset_rad = 0.0;
};

// True when both bearings are finite, non-zero, on the same side, |b| > |a|, and the angle
// between them is less than pi. The node refuses to start on a config that fails this.
bool wall_rays_valid(const WallFollowConfig& config);

// +1 for a right wall (negative bearings), -1 for a left wall. steering = sign * PID(error).
double wall_steering_sign(const WallFollowConfig& config);

// Raw range of the ray nearest a VEHICLE-frame bearing, or nullopt if the bearing is outside
// the scan's coverage.
std::optional<double> range_at_vehicle_bearing(const ScanInput& scan, double vehicle_bearing_rad,
                                               double laser_yaw_offset_rad);

struct WallMeasurement {
  double alpha_rad = 0.0;
  double distance_m = 0.0;            // D
  double lookahead_distance_m = 0.0;  // D_lookahead
  double error_m = 0.0;               // target - D_lookahead, 0 inside the deadband
};

// nullopt when the config is invalid, the scan is unusable, either ray is outside the scan's
// coverage, or either range is not a valid return (NaN, inf, <= 0, below range_min, above
// range_max). The node treats nullopt as "no wall in view": steer straight and reset the PID
// so the next valid sample does not take a derivative across the gap.
std::optional<WallMeasurement> measure_wall(const ScanInput& scan, const WallFollowConfig& config);

}  // namespace racer_control

#endif  // RACER_CONTROL_WALL_FOLLOW_HPP_
