// The dynamics contract's model step (claude-docs/13 section 4.4), in double. The same step
// is used for the solver's rollouts, the golden-rollout check at load, the per tick state
// update and the delay roll-forward. Only the time step differs between users.
#ifndef RACER_PLANNER_DYNAMICS_MODEL_HPP_
#define RACER_PLANNER_DYNAMICS_MODEL_HPP_

#include <array>
#include <cmath>
#include <vector>

#include "racer_planner/vehicle_geometry.hpp"

namespace racer_planner {

struct State {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;  // not wrapped
  double v = 0.0;
  double delta = 0.0;
};

struct Control {
  double speed_cmd = 0.0;
  double steering_cmd = 0.0;
};

struct NominalParams {
  double speed_gain = 1.0;
  // A speed command with |cmd| < speed_deadband_mps leaves the wheels still (steady state 0).
  double speed_deadband_mps = 0.0;
  double speed_tau_s = 0.25;
  double accel_max_mps2 = 2.0;
  double decel_max_mps2 = 1.5;
  double steer_gain_left = 1.0;
  double steer_gain_right = 1.0;
  double steer_offset_rad = 0.0;
  double steer_tau_s = 0.10;
  double command_delay_s = 0.08;
  double understeer_s2_per_m2 = 0.0;
};

// Fixed network shape of this version: 4 -> 32 -> 32 -> 2, tanh hidden, linear output.
constexpr int kMlpIn = 4;
constexpr int kMlpHidden = 32;
constexpr int kMlpOut = 2;
// W1 [32,4], b1 [32], W2 [32,32], b2 [32], W3 [2,32], b3 [2]
constexpr int kMlpWeightCount = kMlpHidden * kMlpIn + kMlpHidden + kMlpHidden * kMlpHidden +
                                kMlpHidden + kMlpOut * kMlpHidden + kMlpOut;

struct ResidualParams {
  bool enabled = false;
  std::array<double, kMlpIn> input_mean{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, kMlpIn> input_std{{1.0, 1.0, 1.0, 1.0}};
  std::array<double, kMlpOut> output_scale{{1.0, 1.0}};
  std::vector<double> weights;  // kMlpWeightCount entries when enabled
};

struct DynamicsParams {
  double dt_s = 0.05;
  NominalParams nominal;
  ResidualParams residual;
  double v_min_mps = 0.0;
  double v_max_mps = 0.9;
};

inline double clamp_double(double x, double lo, double hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

class DynamicsModel {
 public:
  // dt_s is the step this instance integrates with (the bundle's io.dt_s for the solver, the
  // control tick for the per tick state update).
  DynamicsModel(const DynamicsParams& params, const VehicleGeometry& geometry, double dt_s);

  double dt() const { return dt_; }
  const DynamicsParams& params() const { return params_; }

  // The contract step. Trig of the current yaw is passed in so callers that already have it
  // (the solver's footprint) do not pay for it twice. yaw_rate_out may be null.
  State step_trig(const State& s, const Control& u, double cos_yaw, double sin_yaw,
                  double* yaw_rate_out) const {
    const NominalParams& n = params_.nominal;
    const double g = u.steering_cmd >= 0.0 ? n.steer_gain_left : n.steer_gain_right;
    const double delta_target =
        clamp_double(g * u.steering_cmd + n.steer_offset_rad, d_min_, d_max_);
    const double d_delta = clamp_double(a_d_ * (delta_target - s.delta), -rate_dt_, rate_dt_);
    // "Not below the dead zone", so a NaN command stays NaN (same as the Python reference).
    const double target =
        std::fabs(u.speed_cmd) < n.speed_deadband_mps ? 0.0 : n.speed_gain * u.speed_cmd;
    double dv = clamp_double(a_v_ * (target - s.v), -decel_dt_, accel_dt_);
    double yaw_rate =
        s.v * std::tan(s.delta) / wheelbase_ / (1.0 + n.understeer_s2_per_m2 * s.v * s.v);
    if (residual_enabled_) {
      double r[kMlpOut];
      residual(s.v, s.delta, u.speed_cmd, u.steering_cmd, r);
      dv += r[0] * dt_;
      yaw_rate += r[1];
    }
    State out;
    out.x = s.x + s.v * cos_yaw * dt_;
    out.y = s.y + s.v * sin_yaw * dt_;
    out.yaw = s.yaw + yaw_rate * dt_;
    out.v = s.v + dv;
    if (out.v < 0.0) out.v = 0.0;  // max(0, v); NaN passes through
    out.delta = s.delta + d_delta;
    if (yaw_rate_out != nullptr) *yaw_rate_out = yaw_rate;
    return out;
  }

  State step(const State& s, const Control& u) const {
    return step_trig(s, u, std::cos(s.yaw), std::sin(s.yaw), nullptr);
  }

  // Nominal yaw rate of a (v, delta) pair, as used by the step and by dead reckoning.
  double yaw_rate(double v, double delta) const {
    return v * std::tan(delta) / wheelbase_ / (1.0 + params_.nominal.understeer_s2_per_m2 * v * v);
  }

  // Residual MLP: output_scale * MLP((inputs - mean) / std). Preallocated scratch, not
  // thread safe: one model instance per thread.
  void residual(double v, double delta, double speed_cmd, double steering_cmd,
                double out[kMlpOut]) const;

 private:
  DynamicsParams params_;
  double dt_;
  double wheelbase_;
  double d_min_, d_max_;
  double a_d_, a_v_;
  double rate_dt_, accel_dt_, decel_dt_;
  bool residual_enabled_;
  mutable std::array<double, kMlpHidden> h1_;
  mutable std::array<double, kMlpHidden> h2_;
};

}  // namespace racer_planner

#endif  // RACER_PLANNER_DYNAMICS_MODEL_HPP_
