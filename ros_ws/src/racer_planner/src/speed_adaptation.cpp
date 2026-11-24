#include "racer_planner/speed_adaptation.hpp"

#include <algorithm>
#include <cmath>

namespace racer_planner {

SpeedAdaptation::SpeedAdaptation(const SpeedAdaptationParams& params) : p_(params) {
  // Parameters outside their sane range are pulled in, never trusted: the limits are hard.
  p_.forgetting = std::clamp(std::isfinite(p_.forgetting) ? p_.forgetting : 0.995, 0.9, 1.0);
  p_.bias_max_mps2 = std::isfinite(p_.bias_max_mps2) ? std::max(0.0, p_.bias_max_mps2) : 0.0;
  p_.gain_max = std::isfinite(p_.gain_max) ? std::clamp(p_.gain_max, 0.0, 0.9) : 0.0;
  p_.initial_variance =
      std::isfinite(p_.initial_variance) ? std::max(1e-9, p_.initial_variance) : 0.05;
  p_.variance_max =
      std::isfinite(p_.variance_max) ? std::max(p_.initial_variance, p_.variance_max) : 1.0;
  reset();
  resets_ = 0;
}

void SpeedAdaptation::reset() {
  theta_[0] = theta_[1] = 0.0;
  P_[0][0] = P_[1][1] = p_.initial_variance;
  P_[0][1] = P_[1][0] = 0.0;
  updates_ = 0;
  at_limit_ = false;
  last_error_ = 0.0;
  ++resets_;
}

void SpeedAdaptation::clamp_theta() {
  const double b = std::clamp(theta_[0], -p_.bias_max_mps2, p_.bias_max_mps2);
  const double g = std::clamp(theta_[1], -p_.gain_max, p_.gain_max);
  at_limit_ = b != theta_[0] || g != theta_[1];
  theta_[0] = b;
  theta_[1] = g;
}

bool SpeedAdaptation::update(double a_pred, double a_meas, double v_start, double dt,
                             bool driving_forward) {
  if (!p_.enabled) return false;
  if (!std::isfinite(a_pred) || !std::isfinite(a_meas) || !std::isfinite(v_start) ||
      !std::isfinite(dt)) {
    // A bad sample says the inputs cannot be trusted: forget what was learned from them.
    ++skipped_;
    reset();
    return false;
  }
  const double y = a_meas - a_pred;
  if (!driving_forward || v_start < p_.min_speed_mps || dt < p_.min_dt_s || dt > p_.max_dt_s ||
      std::fabs(y) > p_.innovation_max_mps2) {
    ++skipped_;
    return false;
  }
  const double phi[2] = {1.0, a_pred};
  const double e = y - (theta_[0] + theta_[1] * a_pred);
  const double pphi0 = P_[0][0] * phi[0] + P_[0][1] * phi[1];
  const double pphi1 = P_[1][0] * phi[0] + P_[1][1] * phi[1];
  const double denom = p_.forgetting + phi[0] * pphi0 + phi[1] * pphi1;
  if (!(denom > 0.0) || !std::isfinite(denom)) {
    ++skipped_;
    reset();
    return false;
  }
  const double k0 = pphi0 / denom;
  const double k1 = pphi1 / denom;
  theta_[0] += k0 * e;
  theta_[1] += k1 * e;
  // P <- (P - k phi^T P) / lambda, kept symmetric.
  const double p00 = (P_[0][0] - k0 * pphi0) / p_.forgetting;
  const double p01 = (P_[0][1] - k0 * pphi1) / p_.forgetting;
  const double p11 = (P_[1][1] - k1 * pphi1) / p_.forgetting;
  P_[0][0] = p00;
  P_[1][1] = p11;
  P_[0][1] = P_[1][0] = p01;
  // Covariance ceiling: with no excitation the forgetting would grow it without bound.
  const double dmax = std::max(P_[0][0], P_[1][1]);
  if (dmax > p_.variance_max) {
    const double s = p_.variance_max / dmax;
    P_[0][0] *= s;
    P_[1][1] *= s;
    P_[0][1] *= s;
    P_[1][0] *= s;
  }
  if (!std::isfinite(theta_[0]) || !std::isfinite(theta_[1]) || !std::isfinite(P_[0][0]) ||
      !std::isfinite(P_[1][1]) || !std::isfinite(P_[0][1]) || P_[0][0] < 0.0 || P_[1][1] < 0.0) {
    reset();
    return false;
  }
  clamp_theta();
  last_error_ = y - (theta_[0] + theta_[1] * a_pred);
  ++updates_;
  return true;
}

SpeedCorrection SpeedAdaptation::correction() const {
  SpeedCorrection c;
  if (!p_.enabled) return c;
  c.bias_mps2 = theta_[0];
  c.gain = theta_[1];
  return c;
}

SpeedAdaptationStatus SpeedAdaptation::status() const {
  SpeedAdaptationStatus s;
  s.enabled = p_.enabled;
  const SpeedCorrection c = correction();
  s.bias_mps2 = c.bias_mps2;
  s.gain = c.gain;
  s.updates = updates_;
  s.skipped = skipped_;
  s.resets = resets_;
  s.at_limit = at_limit_;
  s.last_error_mps2 = last_error_;
  return s;
}

}  // namespace racer_planner
