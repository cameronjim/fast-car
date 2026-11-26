#include "racer_planner/dynamics_model.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>

namespace racer_planner {

namespace {

// Packed layout of the 4 -> H -> H -> 2 weights for DynamicsModel::mlp(). The bundle's layout
// (the Python reference's) is W1 [H,4], b1, W2 [H,H], b2, W3 [2,H], b3, row major; the hidden
// layers are stored transposed here (W1^T [4,H], W2^T [H,H]) so a layer is "acc[i] += w[j][i] *
// x[j]" over the H units i, which vectorises, while each unit still sums b[i] + w[i][0] x[0] +
// w[i][1] x[1] + ... in the bundle's order (the same operations as the plain loop).
template <int H>
struct MlpLayout {
  static constexpr int kW1 = 0;
  static constexpr int kB1 = kW1 + H * kMlpIn;
  static constexpr int kW2 = kB1 + H;
  static constexpr int kB2 = kW2 + H * H;
  static constexpr int kW3 = kB2 + H;
  static constexpr int kB3 = kW3 + kMlpOut * H;
  static_assert(kB3 + kMlpOut == mlp_weight_count(H), "packed layout covers every weight");
  static_assert(H % 8 == 0, "the split output sums need a multiple of 8 units");
};

void pack_mlp_weights(const std::vector<double>& w, int h,
                      std::array<double, kMlpWeightCount>* out) {
  if (!mlp_hidden_supported(h) || static_cast<int>(w.size()) != mlp_weight_count(h)) return;
  const int b1 = h * kMlpIn, w2 = b1 + h, b2 = w2 + h * h, w3 = b2 + h;
  double* p = out->data();
  for (int i = 0; i < h; ++i)
    for (int j = 0; j < kMlpIn; ++j) p[j * h + i] = w[i * kMlpIn + j];
  for (int i = 0; i < h; ++i)
    for (int j = 0; j < h; ++j) p[w2 + j * h + i] = w[w2 + i * h + j];
  for (int i = 0; i < h; ++i) p[b1 + i] = w[b1 + i];
  for (int i = 0; i < h; ++i) p[b2 + i] = w[b2 + i];
  for (int k = w3; k < mlp_weight_count(h); ++k) p[k] = w[k];  // W3 and b3 as they are
}

// The fast mode's tanh (DynamicsParams::fast_mlp) of each of the n entries of a, in place,
// branch free so the loop vectorises:
// tanh|x| = 1 - 2 / (exp(2|x|) + 1) with the sign of x put back. exp(y) for y in [0, 40] is
// 2^k * exp(r), k = round(y / ln 2), |r| <= ln2 / 2, with a degree 13 Taylor polynomial (its
// truncation error is below 1e-17 relative) and 2^k built from the exponent bits. The absolute
// error against std::tanh is a few 1e-16 for every input (test_dynamics_model checks 1e-12).
// 2|x| above 40 is held at 40, where 1 - 2 / (e^40 + 1) is 1 in double, so saturation is exact
// (and tanh(+-inf) = +-1). A NaN stays NaN.
inline void tanh_inplace(double* a, int n) {
  constexpr double kLog2e = 1.4426950408889634;
  constexpr double kLn2Hi = 6.93147180369123816490e-01;  // fdlibm's split of ln 2
  constexpr double kLn2Lo = 1.90821492927058770002e-10;
  constexpr double kShift = 4503599627370496.0 + 1023.0;  // 2^52 + exponent bias
  for (int i = 0; i < n; ++i) {
    const double x = a[i];
    double y = 2.0 * std::fabs(x);
    y = std::fmin(y, 40.0);  // a NaN becomes 40 here and is put back at the end
    const double k = std::floor(y * kLog2e + 0.5);
    const double r = (y - k * kLn2Hi) - k * kLn2Lo;
    double p = 1.0 / 6227020800.0;  // 1/13!
    p = p * r + 1.0 / 479001600.0;
    p = p * r + 1.0 / 39916800.0;
    p = p * r + 1.0 / 3628800.0;
    p = p * r + 1.0 / 362880.0;
    p = p * r + 1.0 / 40320.0;
    p = p * r + 1.0 / 5040.0;
    p = p * r + 1.0 / 720.0;
    p = p * r + 1.0 / 120.0;
    p = p * r + 1.0 / 24.0;
    p = p * r + 1.0 / 6.0;
    p = p * r + 0.5;
    p = p * r + 1.0;
    p = p * r + 1.0;
    // 2^k: k + 1023 lands in the low mantissa bits of kShift + k (0 <= k <= 58), shifted up into
    // the exponent field.
    const double t = k + kShift;
    std::uint64_t bits;
    std::memcpy(&bits, &t, sizeof bits);
    bits <<= 52;
    double scale;
    std::memcpy(&scale, &bits, sizeof scale);
    const double e = p * scale;
    const double th = std::copysign(1.0 - 2.0 / (e + 1.0), x);
    // A NaN input passes through, selected with integer masks: a comparison and a ternary here
    // make GCC branch and the loop no longer vectorises.
    std::uint64_t xb, tb;
    std::memcpy(&xb, &x, sizeof xb);
    std::memcpy(&tb, &th, sizeof tb);
    const std::uint64_t nan =
        0 - static_cast<std::uint64_t>((xb & 0x7fffffffffffffffULL) > 0x7ff0000000000000ULL);
    tb = (tb & ~nan) | (xb & nan);
    std::memcpy(&a[i], &tb, sizeof tb);
  }
}

// The float version of tanh_inplace (float32_mlp together with fast_mlp): the same construction
// in single precision. exp(y) for y in [0, 20] is 2^k * exp(r), |r| <= ln2 / 2, with a degree 8
// Taylor polynomial (truncation error about 2e-10 relative, far below a float ulp); 2^k is built
// from the exponent bits of k + (2^23 + 127). 2|x| above 20 is held at 20, where 1 - 2 / (e^20 + 1)
// is 1 in float, so saturation is exact (and tanh(+-inf) = +-1). A NaN stays NaN.
inline void tanh_inplace_f32(float* a, int n) {
  constexpr float kLog2e = 1.44269504f;
  constexpr float kLn2Hi = 6.93145752e-01f;  // cephes' split of ln 2, hi has trailing zero bits
  constexpr float kLn2Lo = 1.42860677e-06f;
  constexpr float kShift = 8388608.0f + 127.0f;  // 2^23 + exponent bias
  for (int i = 0; i < n; ++i) {
    const float x = a[i];
    float y = 2.0f * std::fabs(x);
    y = std::fmin(y, 20.0f);  // a NaN becomes 20 here and is put back at the end
    const float k = std::floor(y * kLog2e + 0.5f);
    const float r = (y - k * kLn2Hi) - k * kLn2Lo;
    float p = 1.0f / 40320.0f;
    p = p * r + 1.0f / 5040.0f;
    p = p * r + 1.0f / 720.0f;
    p = p * r + 1.0f / 120.0f;
    p = p * r + 1.0f / 24.0f;
    p = p * r + 1.0f / 6.0f;
    p = p * r + 0.5f;
    p = p * r + 1.0f;
    p = p * r + 1.0f;
    const float t = k + kShift;
    std::uint32_t bits;
    std::memcpy(&bits, &t, sizeof bits);
    bits <<= 23;
    float scale;
    std::memcpy(&scale, &bits, sizeof scale);
    const float e = p * scale;
    const float th = std::copysign(1.0f - 2.0f / (e + 1.0f), x);
    std::uint32_t xb, tb;
    std::memcpy(&xb, &x, sizeof xb);
    std::memcpy(&tb, &th, sizeof tb);
    const std::uint32_t nan = 0 - static_cast<std::uint32_t>((xb & 0x7fffffffU) > 0x7f800000U);
    tb = (tb & ~nan) | (xb & nan);
    std::memcpy(&a[i], &tb, sizeof tb);
  }
}

}  // namespace

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
      residual_enabled_(params.residual.enabled && mlp_hidden_supported(params.residual.hidden) &&
                        static_cast<int>(params.residual.weights.size()) ==
                            mlp_weight_count(params.residual.hidden)),
      residual_hidden_(mlp_hidden_supported(params.residual.hidden) ? params.residual.hidden
                                                                     : kMlpHidden),
      ground_enabled_(params.step_contract == 2 && params.ground.enabled &&
                      static_cast<int>(params.ground.weights.size()) == kMlpWeightCount) {
  // Packed whenever the weights are there (residual() may be called on a model whose step has the
  // block off); pack_mlp_weights leaves zeros for a missing block.
  pack_mlp_weights(params_.residual.weights, residual_hidden_, &residual_packed_);
  pack_mlp_weights(params_.ground.weights, kMlpHidden, &ground_packed_);
  for (int k = 0; k < kMlpWeightCount; ++k) {
    residual_packed_f32_[static_cast<std::size_t>(k)] =
        static_cast<float>(residual_packed_[static_cast<std::size_t>(k)]);
    ground_packed_f32_[static_cast<std::size_t>(k)] =
        static_cast<float>(ground_packed_[static_cast<std::size_t>(k)]);
  }
}

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

template <int H, bool kFast>
void DynamicsModel::mlp(const double* packed, const double in[kMlpIn], double out[kMlpOut]) {
  using L = MlpLayout<H>;
  alignas(16) double h1[H];
  alignas(16) double h2[H];
  const double* w1t = packed + L::kW1;
  const double* b1 = packed + L::kB1;
  const double* w2t = packed + L::kW2;
  const double* b2 = packed + L::kB2;
  const double* w3 = packed + L::kW3;
  const double* b3 = packed + L::kB3;
  for (int i = 0; i < H; ++i) h1[i] = b1[i];
  for (int j = 0; j < kMlpIn; ++j) {
    const double xj = in[j];
    const double* wj = w1t + j * H;
    for (int i = 0; i < H; ++i) h1[i] += wj[i] * xj;
  }
  if (kFast) {
    tanh_inplace(h1, H);
  } else {
    for (int i = 0; i < H; ++i) h1[i] = std::tanh(h1[i]);
  }
  for (int i = 0; i < H; ++i) h2[i] = b2[i];
  for (int j = 0; j < H; ++j) {
    const double hj = h1[j];
    const double* wj = w2t + j * H;
    for (int i = 0; i < H; ++i) h2[i] += wj[i] * hj;
  }
  if (kFast) {
    tanh_inplace(h2, H);
  } else {
    for (int i = 0; i < H; ++i) h2[i] = std::tanh(h2[i]);
  }
  for (int i = 0; i < kMlpOut; ++i) {
    const double* wi = w3 + i * H;
    if (kFast) {
      // Four interleaved partial sums (j mod 4) instead of one chain of H dependent adds: a
      // different rounding order, a few 1e-16 apart, like the tanh above.
      double acc[4] = {0.0, 0.0, 0.0, 0.0};
      for (int j = 0; j < H; j += 4)
        for (int k = 0; k < 4; ++k) acc[k] += wi[j + k] * h2[j + k];
      out[i] = b3[i] + ((acc[0] + acc[1]) + (acc[2] + acc[3]));
    } else {
      double acc = b3[i];
      for (int j = 0; j < H; ++j) acc += wi[j] * h2[j];
      out[i] = acc;
    }
  }
}

template <int H, bool kFast>
void DynamicsModel::mlp_f32(const float* packed, const double in[kMlpIn], double out[kMlpOut]) {
  using L = MlpLayout<H>;
  alignas(16) float h1[H];
  alignas(16) float h2[H];
  const float* w1t = packed + L::kW1;
  const float* b1 = packed + L::kB1;
  const float* w2t = packed + L::kW2;
  const float* b2 = packed + L::kB2;
  const float* w3 = packed + L::kW3;
  const float* b3 = packed + L::kB3;
  for (int i = 0; i < H; ++i) h1[i] = b1[i];
  for (int j = 0; j < kMlpIn; ++j) {
    const float xj = static_cast<float>(in[j]);
    const float* wj = w1t + j * H;
    for (int i = 0; i < H; ++i) h1[i] += wj[i] * xj;
  }
  if (kFast) {
    tanh_inplace_f32(h1, H);
  } else {
    for (int i = 0; i < H; ++i) h1[i] = std::tanh(h1[i]);
  }
  for (int i = 0; i < H; ++i) h2[i] = b2[i];
  for (int j = 0; j < H; ++j) {
    const float hj = h1[j];
    const float* wj = w2t + j * H;
    for (int i = 0; i < H; ++i) h2[i] += wj[i] * hj;
  }
  if (kFast) {
    tanh_inplace_f32(h2, H);
  } else {
    for (int i = 0; i < H; ++i) h2[i] = std::tanh(h2[i]);
  }
  for (int i = 0; i < kMlpOut; ++i) {
    const float* wi = w3 + i * H;
    if (kFast) {
      // Eight interleaved partial sums (j mod 8): one float vector pair instead of H dependent
      // adds.
      float acc[8] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
      for (int j = 0; j < H; j += 8)
        for (int k = 0; k < 8; ++k) acc[k] += wi[j + k] * h2[j + k];
      out[i] = static_cast<double>(b3[i] + (((acc[0] + acc[1]) + (acc[2] + acc[3])) +
                                            ((acc[4] + acc[5]) + (acc[6] + acc[7]))));
    } else {
      float acc = b3[i];
      for (int j = 0; j < H; ++j) acc += wi[j] * h2[j];
      out[i] = static_cast<double>(acc);
    }
  }
}

template <int H>
void DynamicsModel::run_mlp_h(const double* packed, const float* packed_f32,
                              const double in[kMlpIn], double out[kMlpOut]) const {
  if (params_.float32_mlp) {
    if (params_.fast_mlp) {
      mlp_f32<H, true>(packed_f32, in, out);
    } else {
      mlp_f32<H, false>(packed_f32, in, out);
    }
  } else if (params_.fast_mlp) {
    mlp<H, true>(packed, in, out);
  } else {
    mlp<H, false>(packed, in, out);
  }
}

void DynamicsModel::run_mlp(const double* packed, const float* packed_f32, int hidden,
                            const double in[kMlpIn], double out[kMlpOut]) const {
  switch (hidden) {
    case 8:
      run_mlp_h<8>(packed, packed_f32, in, out);
      break;
    case 16:
      run_mlp_h<16>(packed, packed_f32, in, out);
      break;
    case 24:
      run_mlp_h<24>(packed, packed_f32, in, out);
      break;
    default:
      run_mlp_h<kMlpHidden>(packed, packed_f32, in, out);
      break;
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
    run_mlp(ground_packed_.data(), ground_packed_f32_.data(), kMlpHidden, in, m);
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
  double raw[kMlpOut];
  run_mlp(residual_packed_.data(), residual_packed_f32_.data(), residual_hidden_, in, raw);
  for (int i = 0; i < kMlpOut; ++i) out[i] = r.output_scale[i] * raw[i];
}

}  // namespace racer_planner
