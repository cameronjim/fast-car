// Wheel twist: the body-frame velocity the EKF fuses, from wheel speed and modelled steering.
//
// Kinematic bicycle about the rear axle (base_link's origin, REP-105, see vehicle_params
// sensors.lidar's comment): with the rear wheels rolling without lateral slip, the rear axle
// moves along the body x axis, so
//
//   vx = v                      (the VESC's wheel speed, signed: negative in reverse)
//   vy = 0                      (sent as a measurement with a small variance: the
//                                non-holonomic constraint is the most useful thing we know)
//   wz = v * tan(delta) / L     (delta road-wheel angle, LEFT positive; L the wheelbase)
//
// Signs follow claude-docs/06-vehicle-params.md: left steering forwards gives positive
// (counter-clockwise) yaw rate; the same steering in reverse gives negative yaw rate, which
// the formula does by itself because v is negative.
//
// Variances (the EKF needs honest ones, not zeros):
//
//   var(vx) = (speed_std_floor + speed_std_fraction * |v|)^2
//   var(wz) = (v / (L cos^2 delta))^2 * steering_std^2   (steering model error)
//           + (tan(delta) / L)^2 * var(vx)              (speed error)
//           + yaw_rate_std_floor^2                      (slip, wheelbase error)
//   var(vy) = lateral_std^2
//
// steering_std is steering_std_rad while /drive is fresh and stale_steering_std_rad when it
// is not (no command means no idea where the wheels point; the EKF should lean on the LiDAR).
//
// ROS-free; the node (wheel_twist_node.cpp) only moves numbers in and out.
#ifndef RACER_STATE_WHEEL_TWIST_HPP_
#define RACER_STATE_WHEEL_TWIST_HPP_

#include <optional>

#include "racer_state/servo_model.hpp"

namespace racer_state {

struct WheelTwistConfig {
  double wheelbase_m = 0.0;  // chassis.wheelbase_m
  double speed_std_floor_mps = 0.02;
  double speed_std_fraction = 0.03;
  double steering_std_rad = 0.03;
  double stale_steering_std_rad = 0.4;
  double yaw_rate_std_floor_rad_per_s = 0.02;
  double lateral_std_mps = 0.05;
};

// Throws std::invalid_argument unless the wheelbase is positive and every std is finite and
// non-negative.
void validate(const WheelTwistConfig& config);

struct WheelTwist {
  double vx_mps = 0.0;
  double vy_mps = 0.0;
  double wz_rad_per_s = 0.0;
  double var_vx = 0.0;
  double var_vy = 0.0;
  double var_wz = 0.0;
};

// nullopt when the speed or steering angle is not finite. steering_rad is assumed to be within
// the rack limits already (ServoModel clamps), so cos(delta) is far from zero.
std::optional<WheelTwist> compute_wheel_twist(double speed_mps, double steering_rad,
                                              bool steering_fresh, const WheelTwistConfig& config);

// The node's whole decision logic: keeps the servo model and the /drive staleness, turns each
// wheel-speed sample into a twist.
//
// Two clocks, on purpose (the same split as racer_control/tracker_node.cpp's CLOCK POLICY):
//   * stamp_s, the /odom/wheel header stamp, drives the servo model, so a bag replays the
//     same way it was recorded. A stamp that goes backwards or repeats advances nothing.
//   * steady_s, the monotonic clock at receipt, measures /drive staleness, which is about
//     whether the command path is alive now, not about data time.
class WheelTwistEstimator {
 public:
  // Throws std::invalid_argument on a bad config or a non-positive drive_timeout_s.
  WheelTwistEstimator(const ServoModelConfig& servo, const WheelTwistConfig& twist,
                      double drive_timeout_s);

  // A /drive steering command received at steady_s. A non-finite angle is ignored (and so
  // counts as silence once the timeout runs out).
  void on_drive(double steering_rad, double steady_s);

  // One /odom/wheel sample. nullopt for a non-finite speed or stamp (nothing is published).
  std::optional<WheelTwist> on_wheel_speed(double speed_mps, double stamp_s, double steady_s);

  // True when /drive has never arrived or has been silent for longer than drive_timeout_s.
  bool steering_stale(double steady_s) const;

  double steering_estimate_rad() const { return servo_.angle_rad(); }

 private:
  ServoModel servo_;
  WheelTwistConfig twist_config_;
  double drive_timeout_s_;
  bool has_drive_ = false;
  double last_drive_steady_s_ = 0.0;
  double commanded_steering_rad_ = 0.0;
  bool has_stamp_ = false;
  double last_stamp_s_ = 0.0;
};

}  // namespace racer_state

#endif  // RACER_STATE_WHEEL_TWIST_HPP_
