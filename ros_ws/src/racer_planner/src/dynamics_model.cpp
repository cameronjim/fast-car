#include "racer_planner/dynamics_model.hpp"

#include <cmath>

namespace racer_planner {

DynamicsModel::DynamicsModel(const DynamicsParams& params, const VehicleGeometry& geometry,
                             double dt_s)
    : params_(params),
      dt_(dt_s),
      wheelbase_(geometry.wheelbase_m),
      d_min_(geometry.steering_min_rad),
      d_max_(geometry.steering_max_rad),
      a_d_(1.0 - std::exp(-dt_s / params.nominal.steer_tau_s)),
      a_v_(1.0 - std::exp(-dt_s / params.nominal.speed_tau_s)),
      rate_dt_(geometry.steering_max_rate_rad_per_s * dt_s),
      accel_dt_(params.nominal.accel_max_mps2 * dt_s),
      decel_dt_(params.nominal.decel_max_mps2 * dt_s),
      residual_enabled_(params.residual.enabled &&
                        static_cast<int>(params.residual.weights.size()) == kMlpWeightCount),
      ground_enabled_(params.step_contract == 2 && params.ground.enabled &&
                      static_cast<int>(params.ground.weights.size()) == kMlpWeightCount),
      h1_{},
      h2_{} {}

namespace {

// Lower node, upper node and weight of the upper node, held constant outside the grid. The
// Python reference's _interp_axis: the upper index is the first node above x, within 1..n-1.
template <int N>
void interp_axis(const double (&nodes)[N], double x, int* lo, int* hi, double* w) {
  const double xc = clamp_double(x, nodes[0], nodes[N - 1]);
  int h = 0;
  while (h < N && !(xc < nodes[h])) ++h;  // searchsorted(side="right"); a NaN ends at N
  if (h < 1) h = 1;
  if (h > N - 1) h = N - 1;
  *hi = h;
  *lo = h - 1;
  *w = (xc - nodes[*lo]) / (nodes[*hi] - nodes[*lo]);
}

}  // namespace

double DynamicsModel::table_alpha(double delta, double v) const {
  // Steering as a share of the limit of its own side (positive left).
  const double share = delta >= 0.0 ? delta / d_max_ : delta / -d_min_;
  int i0, i1, j0, j1;
  double wi, wj;
  interp_axis(kGroundSteerNodeShare, share, &i0, &i1, &wi);
  interp_axis(kGroundSpeedNodeMps, v, &j0, &j1, &wj);
  const auto& t = params_.ground.table;
  auto at = [&](int i, int j) { return t[static_cast<std::size_t>(i * kGroundSpeedNodes + j)]; };
  return (1 - wi) * (1 - wj) * at(i0, j0) + (1 - wi) * wj * at(i0, j1) +
         wi * (1 - wj) * at(i1, j0) + wi * wj * at(i1, j1);
}

void DynamicsModel::mlp(const double* weights, const double in[kMlpIn], double out[kMlpOut]) const {
  const double* w1 = weights;
  const double* b1 = w1 + kMlpHidden * kMlpIn;
  const double* w2 = b1 + kMlpHidden;
  const double* b2 = w2 + kMlpHidden * kMlpHidden;
  const double* w3 = b2 + kMlpHidden;
  const double* b3 = w3 + kMlpOut * kMlpHidden;
  for (int i = 0; i < kMlpHidden; ++i) {
    double acc = b1[i];
    for (int j = 0; j < kMlpIn; ++j) acc += w1[i * kMlpIn + j] * in[j];
    h1_[i] = std::tanh(acc);
  }
  for (int i = 0; i < kMlpHidden; ++i) {
    double acc = b2[i];
    for (int j = 0; j < kMlpHidden; ++j) acc += w2[i * kMlpHidden + j] * h1_[j];
    h2_[i] = std::tanh(acc);
  }
  for (int i = 0; i < kMlpOut; ++i) {
    double acc = b3[i];
    for (int j = 0; j < kMlpHidden; ++j) acc += w3[i * kMlpHidden + j] * h2_[j];
    out[i] = acc;
  }
}

void DynamicsModel::ground_factors(double v, double delta, double speed_cmd, double steering_cmd,
                                   double* alpha, double* rho) const {
  if (!ground_enabled_) {
    *alpha = 1.0;
    *rho = 0.0;
    return;
  }
  const GroundMotionParams& gm = params_.ground;
  double a = clamp_double(table_alpha(delta, v), gm.alpha_min, gm.alpha_max);
  double r = 0.0;
  if (gm.learned_enabled) {
    const double in[kMlpIn] = {(v - gm.input_mean[0]) / gm.input_std[0],
                               (delta - gm.input_mean[1]) / gm.input_std[1],
                               (speed_cmd - gm.input_mean[2]) / gm.input_std[2],
                               (steering_cmd - gm.input_mean[3]) / gm.input_std[3]};
    double m[kMlpOut];
    mlp(gm.weights.data(), in, m);
    a = clamp_double(a + gm.d_alpha_max * std::tanh(m[0]), gm.alpha_min, gm.alpha_max);
    r = gm.rho_max * std::tanh(m[1]);
  }
  *alpha = a;
  *rho = r;
}

State DynamicsModel::step_ground(const State& s, const Control& u, double cos_yaw, double sin_yaw,
                                 double* yaw_rate_out, double* alpha_out) const {
  // The wheel channel: steering lag, speed lag, limits and dead zone of contract 1, unchanged and
  // with no learned term acting on it.
  const NominalParams& n = params_.nominal;
  const double g = u.steering_cmd >= 0.0 ? n.steer_gain_left : n.steer_gain_right;
  const double delta_target = clamp_double(g * u.steering_cmd + n.steer_offset_rad, d_min_, d_max_);
  const double d_delta = clamp_double(a_d_ * (delta_target - s.delta), -rate_dt_, rate_dt_);
  const double target =
      std::fabs(u.speed_cmd) < n.speed_deadband_mps ? 0.0 : n.speed_gain * u.speed_cmd;
  double dv = clamp_double(a_v_ * (target - s.v), -decel_dt_, accel_dt_);
  if (correction_active_ && target > 0.0) dv = dv * (1.0 + correction_.gain) + correction_bias_dt_;
  // Ground motion: both corrections multiply the wheel speed, so a standing car neither moves nor
  // turns whatever the table or the weights say.
  double alpha, rho;
  ground_factors(s.v, s.delta, u.speed_cmd, u.steering_cmd, &alpha, &rho);
  const double v_g = alpha * s.v;
  const double yaw_rate = (1.0 + rho) * s.v * std::tan(s.delta) / wheelbase_ /
                          (1.0 + n.understeer_s2_per_m2 * s.v * s.v);
  State out;
  out.x = s.x + v_g * cos_yaw * dt_;
  out.y = s.y + v_g * sin_yaw * dt_;
  out.yaw = s.yaw + yaw_rate * dt_;
  out.v = s.v + dv;
  if (out.v < 0.0) out.v = 0.0;  // max(0, v); NaN passes through
  out.delta = s.delta + d_delta;
  if (yaw_rate_out != nullptr) *yaw_rate_out = yaw_rate;
  if (alpha_out != nullptr) *alpha_out = alpha;
  return out;
}

void DynamicsModel::residual(double v, double delta, double speed_cmd, double steering_cmd,
                             double out[kMlpOut]) const {
  const ResidualParams& r = params_.residual;
  const double in[kMlpIn] = {(v - r.input_mean[0]) / r.input_std[0],
                             (delta - r.input_mean[1]) / r.input_std[1],
                             (speed_cmd - r.input_mean[2]) / r.input_std[2],
                             (steering_cmd - r.input_mean[3]) / r.input_std[3]};
  const double* w1 = r.weights.data();
  const double* b1 = w1 + kMlpHidden * kMlpIn;
  const double* w2 = b1 + kMlpHidden;
  const double* b2 = w2 + kMlpHidden * kMlpHidden;
  const double* w3 = b2 + kMlpHidden;
  const double* b3 = w3 + kMlpOut * kMlpHidden;
  for (int i = 0; i < kMlpHidden; ++i) {
    double acc = b1[i];
    for (int j = 0; j < kMlpIn; ++j) acc += w1[i * kMlpIn + j] * in[j];
    h1_[i] = std::tanh(acc);
  }
  for (int i = 0; i < kMlpHidden; ++i) {
    double acc = b2[i];
    for (int j = 0; j < kMlpHidden; ++j) acc += w2[i * kMlpHidden + j] * h1_[j];
    h2_[i] = std::tanh(acc);
  }
  for (int i = 0; i < kMlpOut; ++i) {
    double acc = b3[i];
    for (int j = 0; j < kMlpHidden; ++j) acc += w3[i * kMlpHidden + j] * h2_[j];
    out[i] = r.output_scale[i] * acc;
  }
}

}  // namespace racer_planner
