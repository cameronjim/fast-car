// The dynamics contract's model step (claude-docs/13 section 4.4), in double. The same step
// is used for the solver's rollouts, the golden-rollout check at load, the per tick state
// update and the delay roll-forward. Only the time step differs between users.
//
// Two step contracts exist. Contract 1 (bundle schema 1.x) has one speed state: the wheel speed
// v advances position and yaw, and an additive residual may act on dv and the yaw rate. Contract 2
// (bundle schema 2.0.0, addendum 1 section 2.2, claude-docs/13 section 17.2) keeps the wheel
// channel of contract 1 exactly and gives ground motion its own quantity:
//
//   alpha = clamp(table(delta, v), alpha_min, alpha_max)        (+ bounded learned term)
//   v_g = alpha * v ; yaw_rate = (1 + rho) * v * tan(delta) / L / (1 + understeer * v^2)
//
// ONE MEANING PER QUANTITY (brief 43). Every user of a speed in the planner says which one it
// means; this is the list, and it is the same for both contracts (for contract 1 alpha = 1 and
// ground speed equals wheel speed, so nothing differs):
//   wheel speed v (State::v, the speed command channel): the speed state, the speed command and
//     its dead zone and snap, the lap profile's speed bound and trust cost, the lap station (the
//     lap gate counts wheel distance, so D stays in wheel distance), the safety gate predictor's
//     speed (time to collision, the wheel_speed_mps input), the stall and stop checks, the
//     warm start, the status v_end.
//   ground speed alpha * v: position (x, y), the path length measured against obstacles (the
//     gate's in-path distance along the rollout), the clearance headway, the lateral acceleration
//     (ground speed times yaw rate) and the stopping distance of the terminal term; in dead
//     reckoning the translation of the scan frame.
#ifndef RACER_PLANNER_DYNAMICS_MODEL_HPP_
#define RACER_PLANNER_DYNAMICS_MODEL_HPP_

#include <array>
#include <cmath>
#include <vector>

#include "racer_planner/speed_adaptation.hpp"
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

// Step contract 2 constants of the pre-registered addendum 1 (evaluation/prereg/
// pnp-v0.4-addendum-1.yaml, block ground_motion). A bundle whose numbers differ is refused
// (IoContractMismatch); a test reads the yaml and compares.
constexpr double kGroundAlphaMin = 0.60;
constexpr double kGroundAlphaMax = 1.05;
constexpr double kGroundDAlphaMax = 0.10;
constexpr double kGroundRhoMax = 0.25;
constexpr int kGroundSteerNodes = 7;
constexpr int kGroundSpeedNodes = 5;
constexpr double kGroundSteerNodeShare[kGroundSteerNodes] = {-1.0,   -0.6667, -0.3333, 0.0,
                                                             0.3333, 0.6667,  1.0};
constexpr double kGroundSpeedNodeMps[kGroundSpeedNodes] = {0.25, 0.6, 0.85, 1.25, 1.5};

// Contract 2 ground motion block. enabled is false for contract 1.
struct GroundMotionParams {
  bool enabled = false;
  double alpha_min = kGroundAlphaMin;
  double alpha_max = kGroundAlphaMax;
  double d_alpha_max = kGroundDAlphaMax;
  double rho_max = kGroundRhoMax;
  // table[i * kGroundSpeedNodes + j]: steering node i, wheel speed node j.
  std::array<double, kGroundSteerNodes * kGroundSpeedNodes> table{};
  bool learned_enabled = false;
  std::array<double, kMlpIn> input_mean{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, kMlpIn> input_std{{1.0, 1.0, 1.0, 1.0}};
  std::vector<double> weights;  // kMlpWeightCount entries when learned_enabled
};

struct DynamicsParams {
  double dt_s = 0.05;
  NominalParams nominal;
  ResidualParams residual;
  // Step contract of the bundle: 1 (the residual above may be enabled) or 2 (ground.enabled, the
  // residual is never enabled).
  int step_contract = 1;
  GroundMotionParams ground;
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
  // (the solver's footprint) do not pay for it twice. yaw_rate_out and alpha_out may be null.
  // alpha_out is the ground distance per wheel distance of this step: 1 for contract 1, so the
  // ground speed of the step is alpha * s.v under both contracts.
  State step_trig(const State& s, const Control& u, double cos_yaw, double sin_yaw,
                  double* yaw_rate_out, double* alpha_out = nullptr) const {
    if (ground_enabled_) return step_ground(s, u, cos_yaw, sin_yaw, yaw_rate_out, alpha_out);
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
    if (correction_active_ && target > 0.0)
      dv = dv * (1.0 + correction_.gain) + correction_bias_dt_;
    State out;
    out.x = s.x + s.v * cos_yaw * dt_;
    out.y = s.y + s.v * sin_yaw * dt_;
    out.yaw = s.yaw + yaw_rate * dt_;
    out.v = s.v + dv;
    if (out.v < 0.0) out.v = 0.0;  // max(0, v); NaN passes through
    out.delta = s.delta + d_delta;
    if (yaw_rate_out != nullptr) *yaw_rate_out = yaw_rate;
    if (alpha_out != nullptr) *alpha_out = 1.0;
    return out;
  }

  State step(const State& s, const Control& u) const {
    return step_trig(s, u, std::cos(s.yaw), std::sin(s.yaw), nullptr);
  }

  // Nominal yaw rate of a (v, delta) pair, as used by the step and by dead reckoning.
  double yaw_rate(double v, double delta) const {
    return v * std::tan(delta) / wheelbase_ / (1.0 + params_.nominal.understeer_s2_per_m2 * v * v);
  }

  // The yaw rate the step uses for state (v, delta) under command u: the nominal rate plus the
  // residual's yaw term when the bundle enables one. Dead reckoning (EgoMotion) calls this so the
  // scan frame compensation and the rollouts share one motion model. Without a residual it is
  // exactly yaw_rate(v, delta). Not thread safe (the residual has scratch space).
  double yaw_rate_for(double v, double delta, double speed_cmd, double steering_cmd) const {
    double w = yaw_rate(v, delta);
    if (residual_enabled_) {
      double r[kMlpOut];
      residual(v, delta, speed_cmd, steering_cmd, r);
      w += r[1];
    }
    return w;
  }
  bool has_residual() const { return residual_enabled_; }

  // Residual MLP: output_scale * MLP((inputs - mean) / std). Preallocated scratch, not
  // thread safe: one model instance per thread.
  void residual(double v, double delta, double speed_cmd, double steering_cmd,
                double out[kMlpOut]) const;

  bool has_ground_motion() const { return ground_enabled_; }

  // Online speed correction (speed_adaptation.hpp): dv <- dv * (1 + gain) + bias * dt while the
  // speed target is above 0, in both step contracts. A zero correction (the default, and always
  // when dynamics_adaptation is off) leaves the step bit for bit as the bundle's, so the golden
  // rollouts and every user that never sets one are unchanged. Not thread safe: set it from the
  // thread that steps this instance.
  void set_speed_correction(const SpeedCorrection& c) {
    const bool finite = std::isfinite(c.bias_mps2) && std::isfinite(c.gain);
    correction_ = finite ? c : SpeedCorrection{};
    correction_bias_dt_ = correction_.bias_mps2 * dt_;
    correction_active_ = correction_.bias_mps2 != 0.0 || correction_.gain != 0.0;
  }
  const SpeedCorrection& speed_correction() const { return correction_; }

  // Contract 2: alpha (ground distance per wheel distance) and rho (relative yaw correction) of a
  // state and the command acting on it. Contract 1: alpha = 1, rho = 0. Not thread safe when the
  // learned block is on (scratch space), like residual().
  void ground_factors(double v, double delta, double speed_cmd, double steering_cmd, double* alpha,
                      double* rho) const;

  // Ground speed alpha * v and the yaw rate of the step, of a state under the command acting on
  // it. For contract 1 these are v and the nominal yaw rate. Dead reckoning (EgoMotion) uses them
  // for a contract 2 bundle.
  double ground_speed(double v, double delta, double speed_cmd, double steering_cmd) const {
    double alpha = 1.0, rho = 0.0;
    ground_factors(v, delta, speed_cmd, steering_cmd, &alpha, &rho);
    return alpha * v;
  }
  double ground_yaw_rate(double v, double delta, double speed_cmd, double steering_cmd) const {
    double alpha = 1.0, rho = 0.0;
    ground_factors(v, delta, speed_cmd, steering_cmd, &alpha, &rho);
    return (1.0 + rho) * v * std::tan(delta) / wheelbase_ /
           (1.0 + params_.nominal.understeer_s2_per_m2 * v * v);
  }

 private:
  // Contract 2 step: the wheel channel of contract 1 (no residual), ground motion by alpha, rho.
  State step_ground(const State& s, const Control& u, double cos_yaw, double sin_yaw,
                    double* yaw_rate_out, double* alpha_out) const;
  // Bilinear table lookup, constant outside the grid (the Python reference's table_alpha).
  double table_alpha(double delta, double v) const;
  // 4 -> 32 -> 32 -> 2 network on already standardised inputs, raw output.
  void mlp(const double* weights, const double in[kMlpIn], double out[kMlpOut]) const;

  DynamicsParams params_;
  double dt_;
  double wheelbase_;
  double d_min_, d_max_;
  double a_d_, a_v_;
  double rate_dt_, accel_dt_, decel_dt_;
  bool residual_enabled_;
  bool ground_enabled_;
  SpeedCorrection correction_{};
  double correction_bias_dt_ = 0.0;
  bool correction_active_ = false;
  mutable std::array<double, kMlpHidden> h1_;
  mutable std::array<double, kMlpHidden> h2_;
};

}  // namespace racer_planner

#endif  // RACER_PLANNER_DYNAMICS_MODEL_HPP_
