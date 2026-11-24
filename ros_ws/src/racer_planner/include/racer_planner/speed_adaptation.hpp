// Online adaptation of the dynamics model's speed channel (prototype, owner decision 29; design
// claude-docs/13 part B, "online adaptation", kept minimal). Off unless the node parameter
// dynamics_adaptation is true; with it off nothing in the planner changes.
//
// What is observable on the car at run time: the wheel speed (/odom/wheel, 50 Hz). There is no
// IMU and no scan matching odometry, so the yaw channel is NOT adapted (13 part B, known limit).
// The speed channel is: over each odometry interval the model's predicted acceleration a_pred
// (one model step of the bundle, residual included, from the measured speed and the model's
// steering, under the command acting then) is compared with the measured one a_meas, and a
// recursive least squares fit with forgetting estimates
//
//     a_meas - a_pred = bias + gain * a_pred
//
// The correction applied to the solver's rollouts and the delay roll-forward is then
//     dv <- dv * (1 + gain) + bias * dt      (only while the speed target is above 0)
// so a car that accelerates slower than the model (a low battery, a heavier car) is predicted
// slower. Hard limits: |bias| <= bias_max_mps2, |gain| <= gain_max (the node's parameter ranges
// cap both), the covariance is held under variance_max (no wind-up while there is no
// excitation), a sample with a non-finite value resets the estimate, and reset() returns to zero.
#ifndef RACER_PLANNER_SPEED_ADAPTATION_HPP_
#define RACER_PLANNER_SPEED_ADAPTATION_HPP_

#include <cstdint>

namespace racer_planner {

struct SpeedCorrection {
  double bias_mps2 = 0.0;
  double gain = 0.0;
};

struct SpeedAdaptationParams {
  bool enabled = false;
  double forgetting = 0.995;       // per update (50 Hz: a memory of about 4 s)
  double initial_variance = 0.05;  // diagonal of the covariance after a reset
  double variance_max = 1.0;       // no diagonal entry of the covariance above this
  double bias_max_mps2 = 0.5;      // |bias| hard limit (m/s^2)
  double gain_max = 0.3;           // |gain| hard limit
  double min_speed_mps = 0.3;      // no update below this wheel speed at the interval start
  double min_dt_s = 0.01;          // odometry intervals outside [min_dt_s, max_dt_s] are skipped
  double max_dt_s = 0.2;
  double innovation_max_mps2 = 4.0;  // a larger acceleration error is skipped (a bump, a spike)
};

struct SpeedAdaptationStatus {
  bool enabled = false;
  double bias_mps2 = 0.0;
  double gain = 0.0;
  std::uint64_t updates = 0;     // samples used since the last reset
  std::uint64_t skipped = 0;     // samples offered and not used (gated), since construction
  std::uint64_t resets = 0;      // resets since construction (requested or forced by a bad sample)
  bool at_limit = false;         // the last update hit a hard limit
  double last_error_mps2 = 0.0;  // a_meas - a_pred - correction of the last used sample
};

class SpeedAdaptation {
 public:
  explicit SpeedAdaptation(const SpeedAdaptationParams& params);

  // One odometry interval: the model's predicted and the measured acceleration over it, the
  // wheel speed at its start, its length, and whether the speed target was above 0 (the car was
  // being driven forward). Returns true when the sample was used. Never throws. Does nothing
  // (returns false) when the adaptation is disabled.
  bool update(double a_pred_mps2, double a_meas_mps2, double v_start_mps, double dt_s,
              bool driving_forward);

  void reset();

  // The correction to apply: zero while disabled or right after a reset.
  SpeedCorrection correction() const;
  SpeedAdaptationStatus status() const;
  const SpeedAdaptationParams& params() const { return p_; }

 private:
  void clamp_theta();

  SpeedAdaptationParams p_;
  double theta_[2] = {0.0, 0.0};  // bias, gain
  double P_[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
  std::uint64_t updates_ = 0;
  std::uint64_t skipped_ = 0;
  std::uint64_t resets_ = 0;
  bool at_limit_ = false;
  double last_error_ = 0.0;
};

}  // namespace racer_planner

#endif  // RACER_PLANNER_SPEED_ADAPTATION_HPP_
