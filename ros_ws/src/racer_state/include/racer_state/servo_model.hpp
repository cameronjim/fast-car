// ServoModel: what the front wheels are probably doing, given what /drive asked for.
//
// There is no steering sensor on the car yet (roadmap 2.1's steering pot does not exist), so
// the yaw rate fed to the EKF is computed from the COMMANDED steering angle passed through a
// model of the servo: the command clamped to the rack limits, then followed through a
// first-order lag (vehicle_params steering.time_constant_s, when it has been measured) and a
// rate limit (steering.min_rate_rad_per_s / max_rate_rad_per_s). With time_constant_s still
// null the lag is skipped and the model is a pure rate limit, which is exactly what
// f1tenth_gym's steering does (dynamic_models.pid_steer: full steering velocity towards the
// target until it is reached), so in the sim the model is exact and on the car it is the best
// we have until the servo's step response is measured.
//
// ROS-free (claude-docs/10-conventions.md: logic separated from node plumbing) and every
// number comes from the caller, which reads the generated vehicle_params binding.
#ifndef RACER_STATE_SERVO_MODEL_HPP_
#define RACER_STATE_SERVO_MODEL_HPP_

#include <optional>

namespace racer_state {

struct ServoModelConfig {
  double min_angle_rad = 0.0;             // steering.min_angle_rad (right lock, negative)
  double max_angle_rad = 0.0;             // steering.max_angle_rad (left lock, positive)
  double min_rate_rad_per_s = 0.0;        // steering.min_rate_rad_per_s (negative)
  double max_rate_rad_per_s = 0.0;        // steering.max_rate_rad_per_s (positive)
  std::optional<double> time_constant_s;  // steering.time_constant_s; nullopt = no lag
};

class ServoModel {
 public:
  // Throws std::invalid_argument on a config that cannot describe a servo: non-finite values,
  // min_angle >= max_angle, min_rate >= 0, max_rate <= 0, or a time constant <= 0.
  explicit ServoModel(const ServoModelConfig& config);

  // Advances the model by dt_s towards commanded_rad and returns the new angle (rad, left
  // positive). The command is clamped to the rack limits first. A non-finite command keeps
  // the previous target. A non-finite or non-positive dt_s changes nothing (time did not
  // move, or went backwards).
  double step(double commanded_rad, double dt_s);

  double angle_rad() const { return angle_rad_; }
  double target_rad() const { return target_rad_; }

  // Puts the wheels at angle_rad (clamped) and makes that the target too.
  void reset(double angle_rad);

 private:
  double clamp_angle(double angle_rad) const;

  ServoModelConfig config_;
  double angle_rad_ = 0.0;
  double target_rad_ = 0.0;
};

}  // namespace racer_state

#endif  // RACER_STATE_SERVO_MODEL_HPP_
