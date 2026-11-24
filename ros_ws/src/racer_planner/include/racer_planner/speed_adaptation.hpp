// Online adaptation of the dynamics model's speed channel (prototype, owner decision 29; design
// claude-docs/13 part B, "online adaptation", kept minimal). Off unless the node parameter
// dynamics_adaptation is true; with it off nothing in the planner changes.
//
// What is observable on the car at run time: the wheel speed (/odom/wheel, 50 Hz). There is no
// IMU and no scan matching odometry, so the yaw channel is NOT adapted (13 part B, known limit).
// The speed step of the model is corrected
//     dv <- dv * (1 + gain) + bias * dt      (only while the speed target is above 0)
// in the solver's rollouts and the delay roll-forward, and (bias, gain) are fitted at run time by
// a recursive prediction error method: over each comparison window (window_s) the planner steps
// a shadow of the corrected model once per odometry sample from the measured speed at the window
// start under the commands that acted, with the sensitivities of its speed to bias and gain; at
// the window end the error (measured minus shadow) and the sensitivities are one sample of the
// recursive least squares below (normalised by error_sigma_mps, with forgetting). A window, not a
// sample, because the derivative of a noisy and delayed wheel speed is no measurement; the
// sensitivities, because they make the fitted numbers the step's own parameters.
//
// Hard limits: |bias| <= bias_max_mps2, |gain| <= gain_max (the node's parameter ranges cap
// both), the covariance is held under variance_max (no wind-up without excitation), a window
// with a non-finite value resets the estimate, and reset() returns to zero. The Python reference
// is training/racer_pnp_dynamics adaptation.py (same constants and gating).
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
  double forgetting = 0.98;       // per window
  double initial_variance = 0.1;  // prior variance of each parameter after a reset
  double variance_max = 1.0;      // no diagonal entry of the covariance above this
  double error_sigma_mps = 0.05;  // the window error a sample is normalised by
  double bias_max_mps2 = 0.5;     // |bias| hard limit (m/s^2)
  double gain_max = 0.3;          // |gain| hard limit
  double min_speed_mps = 0.3;     // no update below this wheel speed at the window start
  double min_dt_s = 0.05;         // windows shorter or longer than these are skipped
  double max_dt_s = 2.0;
  double window_s = 0.5;             // comparison window
  double innovation_max_mps2 = 4.0;  // |error| above this times the window is skipped
};

struct SpeedAdaptationStatus {
  bool enabled = false;
  double bias_mps2 = 0.0;
  double gain = 0.0;
  std::uint64_t updates = 0;     // samples used since the last reset
  std::uint64_t skipped = 0;     // samples offered and not used (gated), since construction
  std::uint64_t resets = 0;      // resets since construction (requested or forced by a bad sample)
  bool at_limit = false;         // the last update hit a hard limit
  double last_error_mps2 = 0.0;  // measured minus shadow speed of the last used window (m/s)
};

class SpeedAdaptation {
 public:
  explicit SpeedAdaptation(const SpeedAdaptationParams& params);

  // One closed window: the sensitivities of the shadow's end speed to bias and to gain, the
  // error measured minus shadow at its end (m/s), the wheel speed at its start, its length, and
  // whether every command in it drove forward. Returns true when the window was used. Never
  // throws. Does nothing (returns false) when the adaptation is disabled.
  bool update(double phi_bias, double phi_gain, double error_mps, double v_start, double span_s,
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
