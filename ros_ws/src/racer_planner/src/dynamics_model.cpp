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
      h1_{},
      h2_{} {}

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
